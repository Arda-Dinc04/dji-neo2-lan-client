"""RGB localization in saved model maps and original-PC-age pose expiry."""
from dataclasses import replace
import hashlib

import pytest

cv=pytest.importorskip('cv2')
np=pytest.importorskip('numpy')

from neo2_control.decoded_tracking import DecodedPoseTracker
from neo2_control.video_stream import DecodedObservation
from neo2_control.visual_tracking import MapLocalizer
from test_visual_tracking import motion_scene

OWNER,GENERATION='a'*48,'b'*24


def pixels(path):
    frame=cv.imread(str(path));height,width=frame.shape[:2]
    return cv.cvtColor(frame,cv.COLOR_BGR2RGB).tobytes(),width,height


class Reader:
    context=OWNER,GENERATION
    def __init__(self):
        self.now=10.;self.frame=None;self.closed=False;self.error=None
    def status(self):
        return {'ready':True,'worker_alive':not self.closed,'closed':self.closed,'finished':False,'error':self.error}
    def current(self):return None if self.closed or self.error else self.frame
    def put(self,path,index):
        raw,width,height=pixels(path)
        self.frame=DecodedObservation(*self.context,index,index,self.now,self.now,True,self.now,
            width,height,raw,hashlib.sha256(raw).hexdigest(),{'source_first_slice_byte_offset':index*100})


def test_rgb_pixels_produce_same_pose_as_png_without_reencoding(motion_scene):
    scan,paths,*_=motion_scene
    rgb,width,height=pixels(paths[0])
    localizer=MapLocalizer(scan)
    encoded=localizer.localize(paths[0]);decoded=localizer.localize_rgb(rgb,width,height)
    assert decoded['image_format']=='rgb24'
    assert decoded['image_sha256']==hashlib.sha256(rgb).hexdigest()
    for key in ('camera_center','world_to_camera_R'):
        assert np.allclose(encoded[key],decoded[key],atol=1e-10)
    assert encoded['inliers']==decoded['inliers']


def test_source_bound_model_out_and_back_remembers_one_client_map(motion_scene):
    scan,paths,*_=motion_scene
    reader=Reader();tracker=DecodedPoseTracker(reader,scan,clock=lambda:reader.now,maximum_feature_pixels=None)
    records=[]
    for index,path in enumerate(paths+paths[-2::-1]):
        reader.now+=.05;reader.put(path,index)
        result=tracker.sample();records.append(result)
        assert result['state']=='localized'
        assert tracker.current_pose is result['pose']
        assert result['source']['decoded_index']==index
        assert result['pose']['image_sha256']==reader.frame.rgb_sha256
        assert result['map_sha256']==records[0]['map_sha256']
        assert not result['flight_authorized'] and not result['current_body_pose_verified']
    assert records[4]['distance_from_first']==pytest.approx(1.,abs=.03)
    assert records[-1]['distance_from_first']<.01
    assert records[-1]['distance_unit']=='m'
    records[-1]['source']['pc_first_byte_received_at']=reader.now+100
    reader.now+=.351
    assert tracker.current_pose is None


def test_duplicate_decoded_output_cannot_republish_old_pose(motion_scene):
    reader=Reader();reader.put(motion_scene[1][0],0)
    tracker=DecodedPoseTracker(reader,motion_scene[0],clock=lambda:reader.now,maximum_feature_pixels=None)
    assert tracker.sample()['state']=='localized'
    result=tracker.sample()
    assert result['state']=='waiting' and result['reason']=='duplicate_decoded_image'
    assert tracker.current_pose is None


def test_pose_that_expires_during_real_localization_is_discarded(motion_scene,monkeypatch):
    reader=Reader();reader.put(motion_scene[1][0],0)
    tracker=DecodedPoseTracker(reader,motion_scene[0],clock=lambda:reader.now,maximum_feature_pixels=None)
    solve=tracker.localizer.localize_rgb
    def delayed(*args):
        pose=solve(*args);reader.now+=.4;return pose
    monkeypatch.setattr(tracker.localizer,'localize_rgb',delayed)
    result=tracker.sample()
    assert result['state']=='expired'
    assert result['pose'] is None and tracker.current_pose is None
    assert tracker.localizer._previous is None


@pytest.mark.parametrize('problem',['context','closed','error'])
def test_source_loss_during_localization_clears_flow_and_pose(motion_scene,monkeypatch,problem):
    reader=Reader();reader.put(motion_scene[1][0],0)
    tracker=DecodedPoseTracker(reader,motion_scene[0],clock=lambda:reader.now,maximum_feature_pixels=None)
    solve=tracker.localizer.localize_rgb
    def change(*args):
        pose=solve(*args)
        if problem=='context':reader.context=(OWNER,'c'*24)
        elif problem=='closed':reader.closed=True
        else:reader.error='session_lost'
        return pose
    monkeypatch.setattr(tracker.localizer,'localize_rgb',change)
    if problem=='context':
        with pytest.raises(ValueError,match='context_changed'):tracker.sample()
    else:
        result=tracker.sample();assert result['state']=='stopped' and result['pose'] is None
    assert tracker.current_pose is None and tracker.localizer._previous is None
    with pytest.raises(ValueError,match='tracking_stopped'):tracker.sample()


def test_native_rgb_hash_or_identity_change_is_terminal(motion_scene):
    reader=Reader();reader.put(motion_scene[1][0],0)
    tracker=DecodedPoseTracker(reader,motion_scene[0],clock=lambda:reader.now)
    reader.frame=replace(reader.frame,rgb_sha256='0'*64)
    with pytest.raises(ValueError,match='observation_invalid'):tracker.sample()
    assert tracker.current_pose is None


def test_old_source_does_not_gain_age_from_a_recent_decode(motion_scene):
    reader=Reader();reader.put(motion_scene[1][0],0)
    reader.now+=1.;reader.frame=replace(reader.frame,decoded_at=reader.now)
    tracker=DecodedPoseTracker(reader,motion_scene[0],clock=lambda:reader.now)
    result=tracker.sample()
    assert result['state']=='expired' and result['pose'] is None
