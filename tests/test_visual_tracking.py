"""Repeated image-derived poses and losses; rendered motion is not a flight."""
import hashlib
import json
import subprocess
import sys

import pytest

np=pytest.importorskip('numpy')
cv=pytest.importorskip('cv2')

from neo2_control.metric_atlas import create_atlas
from neo2_control.metric_reference import anchor_scan
from neo2_control.reconstruction import reconstruct_pair
from neo2_control.visual_tracking import MapLocalizer,TemporalMapTracker,track_images
from test_metric_reference import MODEL,RV,detect,render_scene


@pytest.fixture(scope='module')
def motion_scene(tmp_path_factory):
    cv.setNumThreads(2);cv.setRNGSeed(16)
    folder=tmp_path_factory.mktemp('tracked-motion')
    R=cv.Rodrigues(RV)[0]
    centers=[-R.T@np.array(t) for t in ([-.09,-.07,.65],[-.17,-.07,.65])]
    seed=[folder/f'seed-{i}.png' for i in range(2)]
    for path,center in zip(seed,centers):
        render_scene(path,-R@center,texture=True,wide_scene=True,textured_background=True)
    metric=anchor_scan(reconstruct_pair(*seed,MODEL),*[detect(path) for path in seed])
    paths=[];expected=[]
    for i,distance in enumerate([0.,.25,.5,.75,1.]):
        center=centers[0]+R.T@np.array([distance,0.,0.])
        path=folder/f'query-{i}.png'
        render_scene(path,-R@center,texture=True,wide_scene=True,textured_background=True,board=False)
        paths.append(path);expected.append(center)
    rotated=folder/'rotated.png'
    R1=cv.Rodrigues(np.array([0.,np.radians(15),0.]))[0]@R
    render_scene(rotated,-R1@centers[0],texture=True,wide_scene=True,textured_background=True,
        board=False,rotation_vector=cv.Rodrigues(R1)[0])
    missing=folder/'no-landmarks.png'
    assert cv.imwrite(str(missing),np.full((MODEL.height,MODEL.width,3),180,np.uint8))
    return metric,paths,expected,rotated,missing


def test_camera_metre_out_and_back_is_measured_in_one_saved_reference(motion_scene):
    metric,paths,expected,_,_=motion_scene
    ordered=paths+paths[-2::-1]
    report=track_images(metric,ordered)
    assert report['localized_count']==9 and report['lost_count']==0
    assert report['continuous_segments']==1 and report['distance_unit']=='m'
    assert report['coordinate_frame']==metric['coordinate_frame']
    for sample,center in zip(report['samples'][:5],expected):
        assert np.linalg.norm(np.array(sample['pose']['camera_center'])-center)<.03
    assert report['samples'][4]['distance_from_first']==pytest.approx(1.,abs=.03)
    assert report['samples'][-1]['distance_from_first']<.01
    for sample in report['samples'][1:]:
        assert sample['translation_from_previous']==pytest.approx(.25,abs=.02)
        assert sample['rotation_from_previous_degrees']<1
    assert report['samples'][0]['translation_from_previous'] is None
    assert not any(report[k] for k in ('physical_accuracy_verified','tracking_accuracy_verified',
        'acquisition_synchronization_verified','current_pose_verified','flight_authorized'))


def test_nearby_translated_view_uses_consistent_landmark_flow_without_descriptor_search(tmp_path,motion_scene,monkeypatch):
    metric,paths,expected,_,_=motion_scene
    R=cv.Rodrigues(RV)[0];center=expected[0]+R.T@np.array([.01,0.,0.])
    nearby=tmp_path/'nearby.png'
    render_scene(nearby,-R@center,texture=True,wide_scene=True,textured_background=True,board=False)
    tracker=TemporalMapTracker(metric)
    initial=tracker.localize(paths[0]);assert initial['tracking_method']=='descriptors'
    def forbidden(*args):raise AssertionError('descriptor search used despite consistent flow')
    monkeypatch.setattr(tracker,'_solve',forbidden)
    pose=tracker.localize(nearby)
    assert pose['tracking_method']=='optical_flow' and pose['inliers']>=20
    assert np.linalg.norm(np.array(pose['camera_center'])-center)<.015
    assert pose['flow_consistency']['maximum_forward_backward_error_px']<=.75


def test_optional_flow_limit_refreshes_from_map_descriptors_and_resets_after_loss(motion_scene):
    metric, paths, _, _, _ = motion_scene
    tracker = TemporalMapTracker(metric, maximum_flow_frames=2)
    poses = [tracker.localize(paths[0]) for _ in range(5)]
    assert [p['tracking_method'] for p in poses] == [
        'descriptors', 'optical_flow', 'optical_flow', 'descriptors', 'optical_flow']
    assert [p['flow_frames_since_descriptors'] for p in poses] == [0, 1, 2, 0, 1]
    assert poses[3]['descriptor_refresh']
    with pytest.raises(ValueError):
        tracker.localize_bytes(b'not an image')
    assert tracker.localize(paths[0])['tracking_method'] == 'descriptors'


def test_zero_flow_limit_uses_independent_map_observations(motion_scene):
    tracker = TemporalMapTracker(motion_scene[0], maximum_flow_frames=0)
    assert all(tracker.localize(motion_scene[1][0])['tracking_method'] == 'descriptors' for _ in range(3))


def test_small_supported_rotation_can_refresh_descriptors_before_the_frame_limit(tmp_path, motion_scene):
    metric, paths, expected, _, _ = motion_scene
    R = cv.Rodrigues(np.array([0., np.radians(1.), 0.]))[0]@cv.Rodrigues(RV)[0]
    image = tmp_path/'one-degree.png'
    render_scene(image, -R@expected[0], texture=True, wide_scene=True, textured_background=True,
        board=False, rotation_vector=cv.Rodrigues(R)[0])
    tracker = TemporalMapTracker(metric, maximum_flow_frames=60, maximum_flow_rotation_degrees=.1)
    tracker.localize(paths[0])
    result = tracker.localize(image)
    assert result['tracking_method'] == 'descriptors'
    assert result['descriptor_refresh_reason'] == 'rotation_limit'
    assert result['flow_frames_since_descriptors'] == 0


@pytest.mark.parametrize('maximum', [-1, 61, True, 2.5])
def test_invalid_flow_age_limit_is_rejected(motion_scene, maximum):
    with pytest.raises(ValueError, match='flow_frame_limit_invalid'):
        TemporalMapTracker(motion_scene[0], maximum_flow_frames=maximum)


@pytest.mark.parametrize('maximum', [0., .05, 45.1, True, float('nan')])
def test_invalid_flow_rotation_limit_is_rejected_before_compiling_a_map(maximum):
    with pytest.raises(ValueError, match='flow_rotation_limit_invalid'):
        TemporalMapTracker({}, maximum_flow_rotation_degrees=maximum)


def test_large_view_jump_relocalizes_instead_of_following_unrelated_texture(motion_scene):
    metric,paths,expected,_,_=motion_scene
    tracker=TemporalMapTracker(metric);tracker.localize(paths[0])
    pose=tracker.localize(paths[-1])
    assert pose['tracking_method']=='descriptors'
    assert np.linalg.norm(np.array(pose['camera_center'])-expected[-1])<.03


def test_invalid_image_resets_flow_before_next_localization(motion_scene):
    tracker=TemporalMapTracker(motion_scene[0]);path=motion_scene[1][0]
    tracker.localize(path)
    assert tracker.localize(path)['tracking_method']=='optical_flow'
    with pytest.raises(ValueError):tracker.localize_bytes(b'invalid')
    assert tracker.localize(path)['tracking_method']=='descriptors'


def test_lost_view_has_no_stale_pose_and_recovery_starts_a_new_segment(motion_scene):
    metric,paths,_,_,missing=motion_scene
    report=track_images(metric,[paths[0],missing,paths[2]])
    first,lost,recovered=report['samples']
    assert first['segment']==1 and lost['state']=='lost' and lost['segment'] is None
    for key in ('pose','displacement_from_first','distance_from_first',
            'translation_from_previous','rotation_from_previous_degrees'):
        assert lost[key] is None
    assert recovered['segment']==2 and report['continuous_segments']==2
    assert recovered['translation_from_previous'] is None
    assert recovered['rotation_from_previous_degrees'] is None
    assert recovered['distance_from_first']==pytest.approx(.5,abs=.02)


def test_rotation_does_not_become_translation(motion_scene):
    metric,paths,_,rotated,_=motion_scene
    report=track_images(metric,[paths[0],rotated])
    assert report['localized_count']==2
    sample=report['samples'][1]
    assert sample['translation_from_previous']<.015
    assert sample['rotation_from_previous_degrees']==pytest.approx(15.,abs=.5)


def test_baseline_map_stays_in_baselines_instead_of_claiming_metres(motion_scene):
    metric,paths,_,_,_=motion_scene
    report=track_images(metric['scan'],[paths[0],paths[-1]])
    assert report['distance_unit']=='baseline'
    assert report['samples'][1]['distance_from_first']==pytest.approx(
        1./metric['metres_per_baseline'],rel=.03)
    assert report['samples'][1]['pose']['metric_scale_verified'] is False


def test_frozen_map_and_encoded_image_produce_the_same_pose_without_reassembly(motion_scene,monkeypatch):
    import neo2_control.visual_tracking as tracking
    metric,paths,_,_,_=motion_scene
    atlas=create_atlas(metric)
    localizer=MapLocalizer(atlas);identity=localizer.map_sha256
    def forbidden(*args):raise AssertionError('map was assembled again')
    monkeypatch.setattr(tracking,'_assemble',forbidden)
    atlas['seed']['scan']['points'][0][0]+=100
    atlas['coordinate_frame']='f'*64
    expected=localizer.localize(paths[1])
    actual=localizer.localize_bytes(paths[1].read_bytes())
    assert np.linalg.norm(np.array(actual['camera_center'])-expected['camera_center'])<.005
    assert actual['map_sha256']==identity and actual['coordinate_frame']==metric['coordinate_frame']
    assert actual['image_sha256']==hashlib.sha256(paths[1].read_bytes()).hexdigest()
    assert actual['map_point_count']==len(metric['scan']['points'])


@pytest.mark.parametrize('raw',[b'',b'not an image'])
def test_invalid_encoded_frame_is_not_a_pose(motion_scene,raw):
    with pytest.raises(ValueError):MapLocalizer(motion_scene[0]).localize_bytes(raw)


def test_all_lost_sequence_retains_explicit_failure_samples(motion_scene):
    metric,_,_,_,missing=motion_scene
    report=track_images(metric,[missing,missing])
    assert report['localized_count']==0 and report['lost_count']==2
    assert report['continuous_segments']==0 and all(s['pose'] is None for s in report['samples'])


def test_cli_reloads_saved_map_and_retains_partial_trajectory(tmp_path,motion_scene):
    metric,paths,_,_,missing=motion_scene
    source=tmp_path/'map.json';source.write_text(json.dumps(metric))
    output=tmp_path/'trajectory.json'
    p=subprocess.run([sys.executable,'-m','neo2_control.cli','scan-track',str(source),
        str(paths[0]),str(missing),str(paths[-1]),'--output',str(output)],
        capture_output=True,text=True)
    assert p.returncode==2,p.stderr
    report=json.loads(output.read_text());summary=json.loads(p.stdout)
    assert report['localized_count']==summary['localized_count']==2
    assert report['lost_count']==summary['lost_count']==1
    assert report['samples'][2]['distance_from_first']==pytest.approx(1.,abs=.03)
    assert report['samples'][2]['translation_from_previous'] is None
    assert output.stat().st_mode&0o777==0o600 and report['flight_authorized'] is False
    assert len(p.stderr.splitlines())==3
    failed=tmp_path/'lost.json'
    p=subprocess.run([sys.executable,'-m','neo2_control.cli','scan-track',str(source),
        str(missing),'--output',str(failed)],capture_output=True,text=True)
    assert p.returncode==1 and json.loads(failed.read_text())['localized_count']==0
