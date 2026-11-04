"""Image-derived atlas around a full turn and the two return headings.

These are rendered camera observations, not a flight or clearance validation.
"""
import json
import subprocess
import sys

import pytest

np=pytest.importorskip('numpy')
cv=pytest.importorskip('cv2')

from neo2_control.metric_atlas import create_atlas,extend_atlas,load_atlas,save_atlas
from neo2_control.metric_reference import _rotation_error,anchor_scan
from neo2_control.reconstruction import reconstruct_pair
from neo2_control.visual_tracking import MapLocalizer
from room_scene import RoomScene
from test_metric_reference import MODEL,detect


@pytest.fixture(scope='module')
def room_atlas(tmp_path_factory):
    cv.setNumThreads(2);cv.setRNGSeed(31)
    folder=tmp_path_factory.mktemp('full-room')
    room=RoomScene(folder);paths=room.pair(0)
    seed=anchor_scan(reconstruct_pair(*paths,MODEL),*[detect(p) for p in paths])
    atlas=create_atlas(seed)
    for degrees in range(30,360,30):atlas=extend_atlas(atlas,*room.pair(degrees))
    path=folder/'atlas.json';save_atlas(path,atlas)
    return room,seed,path


def test_full_turn_reloads_in_one_frame_and_retains_the_original_seed(room_atlas):
    room,seed,path=room_atlas
    atlas=load_atlas(path);assert atlas['seed']==seed
    assert len(atlas['extensions'])==11
    localizer=MapLocalizer(atlas)
    for degrees in range(0,361,30):
        image=room.folder/f'query-{degrees:03d}.png'
        expected=room.render(image,degrees,room.center)
        pose=localizer.localize(image)
        assert pose['inliers']>=20 and pose['coordinate_frame']==seed['coordinate_frame']
        assert np.linalg.norm(np.array(pose['camera_center'])-room.center)<.05
        assert _rotation_error(np.array(pose['world_to_camera_R']),expected)<1
        assert pose['physical_accuracy_verified'] is False


def test_out_and_back_camera_sequence_uses_opposite_headings_and_survives_cold_reload(room_atlas):
    room,seed,path=room_atlas
    # Centre positions are declared fixture geometry, not inferred gravity or
    # a body/camera transform on the physical Neo 2. No timed sticks execute.
    poses=[(room.center+np.array([0.,.5,0.]),0),(room.center,0),(room.center,90)]
    poses.extend((room.center+np.array([distance,0.,0.]),90) for distance in [.25,.5,.75,1.])
    poses.append((room.center+np.array([1.,0.,0.]),270))
    poses.extend((room.center+np.array([distance,0.,0.]),270) for distance in [.75,.5,.25,0.])
    poses.extend([(room.center,450),(room.center+np.array([0.,.5,0.]),450)])
    paths=[]
    for i,(center,degrees) in enumerate(poses):
        image=room.folder/f'route-{i:02d}.png'
        room.render(image,degrees,center);paths.append(image)
    output=room.folder/'route-track.json'
    run=subprocess.run([sys.executable,'-m','neo2_control.cli','scan-track',str(path),
        *map(str,paths),'--output',str(output)],capture_output=True,text=True)
    assert run.returncode==0,run.stderr
    report=json.loads(output.read_text())
    assert report['localized_count']==len(poses) and report['lost_count']==0
    # The half-metre view cut loses most tracked pixel identities. A few
    # surviving PnP inliers must not bypass independent descriptor recovery.
    assert report['samples'][1]['pose']['tracking_method']=='descriptors'
    assert report['samples'][1]['pose']['flow_fallback_reason']=='tracking_flow_insufficient_retained_landmarks'
    for sample,(center,degrees) in zip(report['samples'],poses):
        pose=sample['pose']
        assert np.linalg.norm(np.array(pose['camera_center'])-center)<.05
        assert _rotation_error(np.array(pose['world_to_camera_R']),room.orientation(degrees))<1
    assert np.linalg.norm(np.array(report['samples'][6]['pose']['camera_center'])-
        report['samples'][2]['pose']['camera_center'])==pytest.approx(1.,abs=.04)
    assert np.linalg.norm(np.array(report['samples'][11]['pose']['camera_center'])-
        report['samples'][2]['pose']['camera_center'])<.04
    assert report['flight_authorized'] is False and report['current_pose_verified'] is False
