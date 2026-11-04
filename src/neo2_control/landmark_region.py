"""Low-support PnP proposes a bounded search region, never a navigation pose."""
from .reconstruction import _vision

MAXIMUM_REGION_PIXELS=640*480
MINIMUM_PROPOSAL_POINTS=16


def propose_region(points,pixels,matches,model):
    """Return only crop bounds/diagnostics; final localization stays separate."""
    cv,np=_vision()
    if len(matches)<MINIMUM_PROPOSAL_POINTS:
        raise ValueError('region_proposal_insufficient_matches')
    ids,features=np.asarray(matches).T
    observed=pixels[features]
    # The shared final-fit helper requires20 unique pixels. This proposal
    # needs its own16-point check and must never relax the final-fit helper.
    _,selected=np.unique(np.rint(observed*4).astype(np.int64),axis=0,return_index=True)
    selected=np.sort(selected)
    ids,observed=ids[selected],observed[selected]
    if len(ids)<MINIMUM_PROPOSAL_POINTS or len(set(ids.tolist()))!=len(ids):
        raise ValueError('region_proposal_insufficient_distinct_points')
    distortion=np.array(model.distortion) if model.distortion else None
    ok,rv,tv,inliers=cv.solvePnPRansac(points[ids],observed,model.matrix(),distortion,
        iterationsCount=200,reprojectionError=4.,confidence=.999,flags=cv.SOLVEPNP_EPNP)
    if (not ok or inliers is None or len(inliers)<MINIMUM_PROPOSAL_POINTS
            or not np.isfinite(rv).all() or not np.isfinite(tv).all()):
        raise ValueError('region_proposal_insufficient_inliers')
    R=cv.Rodrigues(rv)[0]
    projected=cv.projectPoints(points,rv,tv,model.matrix(),distortion)[0].reshape(-1,2)
    visible=(np.isfinite(projected).all(1)&((points@R.T+tv.T)[:,2]>0)
        &(projected[:,0]>=0)&(projected[:,0]<model.width)
        &(projected[:,1]>=0)&(projected[:,1]<model.height))
    if visible.sum()<20:
        raise ValueError('region_proposal_insufficient_visible_landmarks')
    # Keep descriptor support away from edges; preserve original pixel phase
    # on the first four pyramid octaves. Positions are translated back later.
    lower=np.maximum(0,(np.floor((projected[visible].min(0)-64)/16)*16).astype(int))
    upper=np.minimum([model.width,model.height],
        (np.ceil((projected[visible].max(0)+64)/16)*16).astype(int))
    size=upper-lower;area=int(np.prod(size))
    if (size<32).any() or area>min(MAXIMUM_REGION_PIXELS,.75*model.width*model.height):
        raise ValueError('region_proposal_area_limit')
    return {'bounds_xyxy':[int(v) for v in (*lower,*upper)],'pixels':area,
        'coarse_matches':len(matches),'coarse_distinct_inliers':len(inliers),
        'visible_landmarks':int(visible.sum()),'proposal_is_navigation_pose':False}
