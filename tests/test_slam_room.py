from copy import deepcopy
import numpy as np
import pytest

from neo2_control.reconstruction import CameraModel
from neo2_control.slam_room import surface_from_slam


def scene(baseline=.15):
    camera=CameraModel(1280,720,680.,680.,640.,360.,(),
        'candidate','Synthetic projection fixture, independent of the native tracker')
    xyz=np.array([[x,y,3.] for x in np.linspace(-.6,.6,6) for y in np.linspace(-.6,.6,6)])
    native={'width':640,'height':360,'fx':340.,'fy':340.,'cx':320.,'cy':180.}
    keyframes=[];frames={};observations={i:[] for i in range(len(xyz))}
    for identity,(offset,yaw) in enumerate(((-baseline,-.1),(0.,0.),(baseline,.1))):
        c,s=np.cos(yaw),np.sin(yaw)
        R=np.array([[c,0,s],[0,1,0],[-s,0,c]])
        cw=np.eye(4);cw[:3,:3]=R;cw[:3,3]=-R@np.array([offset,0,0])
        q=xyz@R.T+cw[:3,3];pixels=q[:,:2]/q[:,2:]*340+[320,180]
        timestamp=float(identity+1)
        keyframes.append({'id':identity,'timestamp':timestamp,'pose_camera_from_world':cw.tolist()})
        frames[timestamp]={'file':f'frame-{identity:05d}.jpg','sha256':'a'*64}
        for i,xy in enumerate(pixels):observations[i].append({'keyframe_id':identity,'xy':xy.tolist()})
    return {'format':'neo2-control/stella-map/v1','camera':native,'keyframes':keyframes,
        'points':[{'id':i,'xyz':p.tolist(),'keyframe_observations':observations[i]} for i,p in enumerate(xyz)],
        'pose_valid':True,'pose_world_from_camera':np.linalg.inv(cw).tolist(),
        'current_observations':[{'id':i,'xy':xy.tolist()} for i,xy in enumerate(pixels)]},frames,camera


def test_actual_projection_observations_make_photographic_geometry_and_current_support():
    snapshot,frames,camera=scene()
    result=surface_from_slam(snapshot,frames,camera)
    assert len(result['points'])==36 and result['triangle_count']>0
    assert result['current_supported_points']==36 and result['supports_current_geometry']
    assert np.asarray(result['current_pose_camera_from_world'])==pytest.approx(
        np.asarray(snapshot['keyframes'][-1]['pose_camera_from_world']))
    assert all(p['image'] in {f['file'] for f in frames.values()} for p in result['patches'])


def test_geometry_uses_original_pixel_error_after_native_half_size_processing():
    snapshot,frames,camera=scene()
    snapshot['points'][0]['keyframe_observations'][2]['xy'][0]+=1.1
    result=surface_from_slam(snapshot,frames,camera)
    assert len(result['points'])==35 and result['current_supported_points']==35
    snapshot,frames,camera=scene()
    for observation in snapshot['current_observations']:observation['xy'][0]+=1.1
    result=surface_from_slam(snapshot,frames,camera)
    assert len(result['points'])==36 and result['current_supported_points']==0
    assert not result['supports_current_geometry']


def test_rotations_at_one_position_do_not_establish_depth():
    snapshot,frames,camera=scene(baseline=0.)
    result=surface_from_slam(snapshot,frames,camera)
    assert not result['points'] and not result['patches']
    assert result['current_supported_points']==0


def test_a_missing_photograph_cannot_supply_a_third_observation():
    snapshot,frames,camera=scene();frames.pop(3.)
    result=surface_from_slam(snapshot,frames,camera)
    assert not result['points']


def test_lost_tracking_preserves_checked_map_but_has_no_current_support():
    snapshot,frames,camera=scene();snapshot['pose_valid']=False
    snapshot.pop('pose_world_from_camera')
    result=surface_from_slam(snapshot,frames,camera)
    assert len(result['points'])==36 and not result['supports_current_geometry']
    assert result['current_pose_camera_from_world'] is None


def test_incompatible_intrinsics_and_duplicate_tracks_are_rejected():
    snapshot,frames,camera=scene();changed=deepcopy(snapshot);changed['camera']['fx']+=1
    with pytest.raises(ValueError,match='intrinsics_mismatch'):surface_from_slam(changed,frames,camera)
    snapshot['points'][0]['keyframe_observations'].append(snapshot['points'][0]['keyframe_observations'][0])
    with pytest.raises(ValueError,match='duplicate_observation'):surface_from_slam(snapshot,frames,camera)
