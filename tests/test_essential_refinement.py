"""Calibrated nonlinear fit against known noisy image correspondences."""
import pytest

np=pytest.importorskip('numpy')
cv=pytest.importorskip('cv2')

from neo2_control.essential_refinement import refine_essential,_residual,_cost
from neo2_control.reconstruction import triangulate_correspondences
from test_reconstruction import MODEL,scene


def noisy_case():
    points,a,b,R,t=scene();rng=np.random.default_rng(37)
    for _ in range(2):
        p=a+rng.normal(0,.25,a.shape);q=b+rng.normal(0,.25,b.shape)
    E,mask=cv.findEssentialMat(p,q,MODEL.matrix(),method=cv.RANSAC,prob=.999,threshold=1.,maxIters=3000)
    return points,p,q,R,t,E,mask


def rotation_error(R,truth):
    return float(np.degrees(np.arccos(np.clip((np.trace(R@truth.T)-1)/2,-1,1))))


def test_refinement_improves_the_actual_calibrated_objective_and_known_rotation():
    _,a,b,truth,_,E,mask=noisy_case()
    original_a=a.copy();original_b=b.copy();original_mask=mask.copy()
    _,before_R,before_t,_=cv.recoverPose(E,a,b,MODEL.matrix(),mask=mask.copy())
    refined=refine_essential(E,a,b,MODEL.matrix(),mask)
    assert refined is not None
    _,after_R,after_t,_=cv.recoverPose(refined,a,b,MODEL.matrix(),mask=mask.copy())
    selected=np.flatnonzero(mask.ravel())
    x=np.c_[a[selected],np.ones(len(selected))];y=np.c_[b[selected],np.ones(len(selected))]
    inverse=np.linalg.inv(MODEL.matrix())
    before=_cost(_residual(before_R,before_t.ravel(),x,y,inverse))
    after=_cost(_residual(after_R,after_t.ravel(),x,y,inverse))
    assert after<before*.6
    # This finite noisy sample need not recover the exact ground truth.
    # Check the improvement over the original fitted candidate, without
    # inventing an absolute physical accuracy bound from synthetic noise.
    assert rotation_error(after_R,truth)<rotation_error(before_R,truth)*.25
    singular=np.linalg.svd(refined,compute_uv=False)
    assert singular[:2]==pytest.approx([1.,1.],abs=1e-10) and singular[2]<1e-10
    assert np.array_equal(a,original_a) and np.array_equal(b,original_b)
    assert np.array_equal(mask,original_mask)


def test_triangulation_keeps_known_scale_convention_and_positive_depths_after_refit():
    points,a,b,truth,_,initial,mask=noisy_case()
    result=triangulate_correspondences(a,b,MODEL)
    actual=np.array(result['points']);expected=points[result['selected_indices']]
    assert len(actual)>=140
    _,R,t,_=cv.recoverPose(initial,a,b,MODEL.matrix(),mask=mask.copy())
    P0=MODEL.matrix()@np.c_[np.eye(3),np.zeros(3)]
    P1=MODEL.matrix()@np.c_[R,t]
    selected=result['selected_indices']
    homogeneous=cv.triangulatePoints(P0,P1,a[selected].T,b[selected].T)
    before=(homogeneous[:3]/homogeneous[3]).T
    assert np.sqrt(np.mean((actual-expected)**2))<np.sqrt(np.mean((before-expected)**2))*.25
    assert rotation_error(np.array(result['second_world_to_camera_R']),truth)<rotation_error(R,truth)*.25
    assert np.linalg.norm(result['second_world_to_camera_t'])==pytest.approx(1.,abs=1e-10)
    assert (actual[:,2]>0).all() and max(result['reprojection_error_px'])<=1.


def test_insufficient_original_inliers_cannot_create_a_refined_model():
    _,a,b,_,_,E,mask=noisy_case();mask[:]=0;mask[:10]=1
    assert refine_essential(E,a,b,MODEL.matrix(),mask) is None
