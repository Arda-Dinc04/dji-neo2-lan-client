"""Known camera geometry and restartable PC memory, never aircraft readiness."""
import json

import pytest

np=pytest.importorskip('numpy')
cv=pytest.importorskip('cv2')

from neo2_control.reconstruction import (CameraModel,triangulate_correspondences,
    localize_correspondences,load_scan,save_scan)


MODEL=CameraModel(1280,720,680.,680.,640.,360.,(),'calibrated','synthetic exact pinhole fixture')


def scene(*,pure_rotation=False):
    random=np.random.default_rng(15)
    points=random.uniform([-1.5,-1.,4.],[1.5,1.,8.],(160,3))
    rotation=cv.Rodrigues(np.array([.02,-.03,.01]))[0]
    translation=np.array([0.,1.,0.]) if not pure_rotation else np.zeros(3)
    def project(R,t):
        q=(points@R.T+t)@MODEL.matrix().T
        return q[:,:2]/q[:,2:]
    return points,project(np.eye(3),np.zeros(3)),project(rotation,translation),rotation,translation


def fixture_scan():
    points,a,b,R,t=scene()
    cv.setRNGSeed(81)
    geometry=triangulate_correspondences(a,b,MODEL)
    selected=geometry.pop('selected_indices');n=len(selected)
    value={'format':'neo2-control/sparse-scan/v1','coordinate_frame':'a'*64,
        'axes':'first_camera_right_down_forward','distance_unit':'baseline',
        'metric_scale_verified':False,'free_space_observed':False,'camera_model':MODEL.as_dict(),
        'image_sha256':['a'*64,'b'*64],'rgb':[[120,150,180]]*n,
        'reference_descriptors':np.zeros((n,128)).tolist(),**geometry}
    return value,points[np.array(selected)]


def test_known_two_view_structure_retains_depth_and_rejects_navigation_unit_assumptions():
    scan,expected=fixture_scan()
    assert len(scan['points'])>=150
    assert np.array(scan['points'])==pytest.approx(expected,abs=1e-5)
    assert max(scan['reprojection_error_px'])<1e-5
    assert min(scan['parallax_degrees'])>1
    assert scan['distance_unit']=='baseline' and scan['metric_scale_verified'] is False
    assert scan['free_space_observed'] is False


def test_pure_rotation_never_turns_a_panorama_into_a_point_cloud():
    points,a,b,R,t=scene(pure_rotation=True)
    cv.setRNGSeed(22)
    with pytest.raises(ValueError,match='parallax_or_inliers'):
        triangulate_correspondences(a,b,MODEL)


def test_repeated_orientations_and_rotation_with_image_noise_are_not_a_baseline():
    points,a,b,R,t=scene()
    with pytest.raises(ValueError,match='distinct_points'):
        triangulate_correspondences(np.repeat(a[:10],5,axis=0),np.repeat(b[:10],5,axis=0),MODEL)
    points,a,b,R,t=scene(pure_rotation=True)
    random=np.random.default_rng(96);b+=random.normal(0,.05,b.shape)
    b[:12]=random.uniform([0,0],[1280,720],(12,2))
    cv.setRNGSeed(23)
    with pytest.raises(ValueError,match='parallax_or_inliers'):
        triangulate_correspondences(a,b,MODEL)


def test_collinear_points_cannot_define_a_camera_pose():
    first=np.column_stack([np.linspace(200,1000,40),np.full(40,300.)])
    second=first+np.array([0.,80.])
    with pytest.raises(ValueError,match='degenerate'):
        triangulate_correspondences(first,second,MODEL)


def test_reloaded_client_map_keeps_frame_and_can_relocalize_a_measured_displacement(tmp_path):
    scan,_=fixture_scan();path=tmp_path/'scan.json'
    result=save_scan(path,scan);restored=load_scan(path)
    assert restored==scan and result['coordinate_frame']=='a'*64
    assert path.stat().st_mode&0o777==0o600
    with pytest.raises(FileExistsError):save_scan(path,scan)
    center=np.array([.2,-.1,.15]);R=cv.Rodrigues(np.array([.01,.02,-.03]))[0]
    t=-R@center;points=np.array(restored['points'])
    q=(points@R.T+t)@MODEL.matrix().T;pixels=q[:,:2]/q[:,2:]
    pose=localize_correspondences(points,pixels,MODEL)
    assert pose['camera_center']==pytest.approx(center,abs=1e-5)
    assert pose['distance_unit']=='baseline' and pose['tracking_accuracy_verified'] is False


def test_pose_refinement_removes_outlier_without_raising_the_final_residual_limit():
    random=np.random.default_rng(7)
    points=random.uniform([-1.5,-1.,3.],[1.5,1.,6.],(320,3))
    rv=np.array([.1,-.2,.03]);R=cv.Rodrigues(rv)[0];center=np.array([.2,-.1,.15])
    pixels=cv.projectPoints(points,rv,-R@center,MODEL.matrix(),None)[0].reshape(-1,2)
    pixels+=random.normal(0,.85,pixels.shape)
    pixels[:30]+=random.normal(0,10.,(30,2))
    cv.setRNGSeed(42)
    pose=localize_correspondences(points,pixels,MODEL,include_inlier_indices=True)
    assert pose['refinement_rejected_inliers']>=1
    assert pose['inliers']>=200 and pose['maximum_reprojection_error_px']<=2.5
    selected=np.array(pose['inlier_indices'])
    assert len(selected)==pose['inliers'] and len(set(selected))==len(selected)
    estimated_R=np.array(pose['world_to_camera_R']);t=np.array(pose['world_to_camera_t'])
    projection=cv.projectPoints(points[selected],cv.Rodrigues(estimated_R)[0],t,
        MODEL.matrix(),None)[0].reshape(-1,2)
    assert np.linalg.norm(projection-pixels[selected],axis=1).max()<=2.5
    assert np.linalg.norm(np.array(pose['camera_center'])-center)<.02
    assert pose['tracking_accuracy_verified'] is False and pose['metric_scale_verified'] is False


@pytest.mark.parametrize('change',[
    {'distance_unit':'m'},{'metric_scale_verified':True},{'free_space_observed':True},
    {'coordinate_frame':'c'*64},{'points':[[float('nan'),0,1]]*30},
    {'second_world_to_camera_R':[[2,0,0],[0,1,0],[0,0,1]]}])
def test_reload_cannot_silently_change_units_frame_free_space_or_geometry(tmp_path,change):
    value,_=fixture_scan();value.update(change)
    path=tmp_path/'corrupted.json';path.write_text(json.dumps(value))
    with pytest.raises(ValueError):load_scan(path)


def test_wrong_calibration_size_nonfinite_coordinates_and_insufficient_data_are_rejected():
    with pytest.raises(ValueError):CameraModel.from_dict(MODEL.as_dict()|{'fx':float('nan')})
    points,a,b,R,t=scene()
    with pytest.raises(ValueError):triangulate_correspondences(a[:3],b[:3],MODEL)
    a[0,0]=float('nan')
    with pytest.raises(ValueError):triangulate_correspondences(a,b,MODEL)
