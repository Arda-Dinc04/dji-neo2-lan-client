"""Real matching/PnP after a low-support crop proposal; fixed feature fixtures."""
import hashlib

import pytest

np=pytest.importorskip('numpy')
cv=pytest.importorskip('cv2')

from neo2_control import visual_tracking
from neo2_control.landmark_region import propose_region,MAXIMUM_REGION_PIXELS
from neo2_control.reconstruction import CameraModel,triangulate_correspondences
from neo2_control.visual_tracking import MapLocalizer,TemporalMapTracker


@pytest.fixture(scope='module')
def scene():
    cv.setNumThreads(2);cv.setRNGSeed(72)
    rng=np.random.default_rng(72)
    model=CameraModel(1280,720,680.,680.,640.,360.,(),'candidate','rendered correspondence fixture')
    points=rng.uniform([.5,-.8,4.],[1.5,.8,7.],(80,3))
    pixel=lambda p,t:cv.projectPoints(p,np.zeros(3),np.array(t),model.matrix(),None)[0].reshape(-1,2)
    geometry=triangulate_correspondences(pixel(points,[0.,0.,0.]),pixel(points,[-1.,0.,0.]),model)
    geometry.pop('selected_indices');points=np.array(geometry['points'])
    desc=rng.integers(0,64,(len(points),128)).astype(np.float32)
    scan={'format':'neo2-control/sparse-scan/v1','coordinate_frame':'a'*64,
        'axes':'first_camera_right_down_forward','distance_unit':'baseline','metric_scale_verified':False,
        'free_space_observed':False,'camera_model':model.as_dict(),'image_sha256':['a'*64,'b'*64],
        'rgb':[[100,100,100] for _ in points],'reference_descriptors':desc.tolist(),**geometry}
    query_t=np.array([.05,.02,.01]);query=pixel(points,query_t)
    # Encode pixel positions so the feature fixture can recover a crop origin.
    yy,xx=np.indices((720,1280));code=yy*1280+xx
    image=np.stack(((code>>16)&255,(code>>8)&255,code&255),axis=2).astype(np.uint8)
    return scan,model,points,desc,query,query_t,image


def feature_fixture(monkeypatch,scene,*,coarse=18,refined=None,duplicates=False):
    _,_,_,desc,px,_,_=scene
    calls=[];rng=np.random.default_rng(75)
    def features(image,*,maximum_pixels=None):
        code=int(image[0,0,0])*65536+int(image[0,0,1])*256+int(image[0,0,2])
        offset=np.array([code%1280,code//1280])
        calls.append((image.shape,maximum_pixels))
        if maximum_pixels is not None:
            pixels=px[:coarse].copy()
            if duplicates:pixels[3:7]=pixels[0]
            return (np.vstack((pixels,rng.uniform([0,0],[1280,720],(20,2)))),
                np.vstack((desc[:coarse],rng.integers(90,120,(20,128)).astype(np.float32))))
        count=len(desc) if refined is None else refined
        return px[:count]-offset,desc[:count].copy()
    monkeypatch.setattr(visual_tracking,'_features',features)
    return calls


def test_rgb_region_uses_full_coordinate_pixels_and_original_strict_pose_fit(scene,monkeypatch):
    scan,_,_,_,_,t,image=scene
    calls=feature_fixture(monkeypatch,scene)
    rgb=image[:,:,::-1].tobytes()
    pose=MapLocalizer(scan,maximum_feature_pixels=230400,region_search=True).localize_rgb(rgb,1280,720)
    assert len(calls)==2
    assert calls[0][0][:2]==(720,1280) and calls[1][0][0]<720
    assert pose['region_search']['coarse_matches']==18
    assert pose['region_search']['coarse_distinct_inliers']>=16
    assert pose['region_search']['pixels']<=MAXIMUM_REGION_PIXELS
    assert not pose['region_search']['proposal_is_navigation_pose']
    assert pose['inliers']>=20 and pose['maximum_reprojection_error_px']<=2.5
    assert pose['camera_center']==pytest.approx(-t,abs=1e-5)
    assert pose['image_sha256']==hashlib.sha256(rgb).hexdigest()
    assert not pose['physical_accuracy_verified'] and not pose['free_space_observed']


def test_region_search_disabled_preserves_insufficient_landmark_result(scene,monkeypatch):
    feature_fixture(monkeypatch,scene)
    with pytest.raises(ValueError,match='insufficient_matches'):
        MapLocalizer(scene[0],maximum_feature_pixels=230400)._solve(scene[-1])


def test_proposal_cannot_supply_votes_missing_from_final_pose(scene,monkeypatch):
    calls=feature_fixture(monkeypatch,scene,refined=9)
    tracker=TemporalMapTracker(scene[0],maximum_feature_pixels=230400,region_search=True)
    with pytest.raises(ValueError,match='insufficient_matches'):
        tracker.localize_rgb(scene[-1][:,:,::-1].tobytes(),1280,720)
    assert len(calls)==2 and tracker._previous is None


def test_duplicate_coarse_pixels_cannot_supply_a_proposal(scene,monkeypatch):
    calls=feature_fixture(monkeypatch,scene,duplicates=True)
    with pytest.raises(ValueError,match='insufficient_distinct_points'):
        MapLocalizer(scene[0],maximum_feature_pixels=230400,region_search=True)._solve(scene[-1])
    assert len(calls)==1


def test_overlarge_proposed_region_is_rejected_before_feature_work(scene):
    _,model,points,_,_,t,_=scene
    points=points.copy();points[:,0]-=1.;points[:,:2]*=5
    px=cv.projectPoints(points,np.zeros(3),t,model.matrix(),None)[0].reshape(-1,2)
    with pytest.raises(ValueError,match='region_proposal_area_limit'):
        propose_region(points,px,[(i,i) for i in range(20)],model)


def test_existing_sufficient_matches_keep_same_pose_and_skip_refinement(scene,monkeypatch):
    calls=feature_fixture(monkeypatch,scene,coarse=30)
    original=MapLocalizer(scene[0],maximum_feature_pixels=230400)._solve(scene[-1])[0]
    enabled=MapLocalizer(scene[0],maximum_feature_pixels=230400,region_search=True)._solve(scene[-1])[0]
    assert len(calls)==2 and all(budget==230400 for _,budget in calls)
    assert enabled==original and 'region_search' not in enabled


@pytest.mark.parametrize('value',[1,None,'yes'])
def test_region_option_is_explicit_boolean(value):
    with pytest.raises(ValueError,match='region_option_invalid'):MapLocalizer({},region_search=value)
