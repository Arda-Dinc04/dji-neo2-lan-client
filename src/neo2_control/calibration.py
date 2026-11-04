"""Offline optical calibration; image fitting does not verify metric scale."""
import hashlib
import itertools

from .reconstruction import CameraModel,_read,_vision


def calibrate_checkerboard(paths,*,columns=9,rows=6):
    cv,np=_vision()
    if (not isinstance(paths,(tuple,list)) or not 10<=len(paths)<=40
            or type(columns) is not int or type(rows) is not int
            or not 4<=columns<=15 or not 4<=rows<=15):
        raise ValueError('calibration_images_or_pattern_invalid')
    object_grid=np.zeros((columns*rows,3),dtype=np.float32)
    object_grid[:,:2]=np.mgrid[0:columns,0:rows].T.reshape(-1,2)
    observed=[];sources=[]
    for image,source_hash in _calibration_frames(paths):
        size=(image.shape[1],image.shape[0])
        found,corners=cv.findChessboardCornersSB(image,(columns,rows),
            flags=cv.CALIB_CB_NORMALIZE_IMAGE|cv.CALIB_CB_ACCURACY)
        sources.append({'sha256':source_hash,'pattern_found':bool(found),
            'corners_px':corners.reshape(-1,2).tolist() if found else None})
        if found:observed.append(corners)
    return _fit_pattern([object_grid]*len(observed),observed,sources,size,
        columns=columns,rows=rows,name=f'Checkerboard {columns}x{rows} inner corners')


def _calibration_frames(paths):
    cv,np=_vision()
    if not isinstance(paths,(tuple,list)) or not 10<=len(paths)<=40:
        raise ValueError('calibration_images_or_pattern_invalid')
    size=None;hashes=set()
    for path in paths:
        raw=_read(path,32*1024*1024)
        image=cv.imdecode(np.frombuffer(raw,dtype=np.uint8),cv.IMREAD_GRAYSCALE)
        if image is None or not 32<=image.shape[0]<=8192 or not 32<=image.shape[1]<=8192:
            raise ValueError('calibration_image_invalid')
        current=(image.shape[1],image.shape[0])
        if size is None:size=current
        if current!=size:raise ValueError('calibration_image_size_changed')
        source_hash=hashlib.sha256(raw).hexdigest()
        if source_hash in hashes:raise ValueError('calibration_duplicate_image')
        hashes.add(source_hash)
        yield image,source_hash


def calibrate_charuco(paths,board):
    """Use the same identified reference for optics and later metric anchoring."""
    from .metric_reference import ReferenceBoard
    if not isinstance(board,ReferenceBoard):raise ValueError('reference_board_invalid')
    cv,np=_vision()
    native=board.opencv_board(1.)
    detector=cv.aruco.CharucoDetector(native)
    observed=[];objects=[];sources=[]
    for image,source_hash in _calibration_frames(paths):
        size=(image.shape[1],image.shape[0])
        corners,ids,_,_=detector.detectBoard(image)
        found=ids is not None and len(ids)>=12
        if found:
            selected=native.getChessboardCorners()[ids.ravel()]
            found=bool((np.ptp(selected[:,:2],axis=0)>=1.9).all())
        sources.append({'sha256':source_hash,'pattern_found':found,
            'corners_px':corners.reshape(-1,2).tolist() if found else None,
            'corner_ids':ids.ravel().tolist() if found else None})
        if found:
            observed.append(corners)
            objects.append(selected)
    model,report=_fit_pattern(objects,observed,sources,size,columns=6,rows=4,
        name=f'ChArUco 7x5 {board.reference_id}')
    return model,report|{'pattern_type':'charuco','board':board.as_dict()}


def _fit_pattern(objects,observed,sources,size,*,columns,rows,name):
    cv,np=_vision()
    if len(observed)<10:raise ValueError('calibration_insufficient_pattern_views')
    all_points=np.concatenate(observed).reshape(-1,2)
    coverage=(all_points.max(axis=0)-all_points.min(axis=0))/np.array(size)
    if (coverage<.4).any():raise ValueError('calibration_insufficient_image_coverage')
    result=cv.calibrateCameraExtended(objects,observed,size,None,None,
        flags=cv.CALIB_FIX_K3,criteria=(cv.TERM_CRITERIA_COUNT|cv.TERM_CRITERIA_EPS,100,1e-8))
    rms,K,distortion,rvecs,tvecs,std_intrinsic,std_extrinsic,view_errors=result
    normals=[cv.Rodrigues(vector)[0][:,2] for vector in rvecs]
    separation=max(float(np.degrees(np.arccos(np.clip(a@b,-1.,1.))))
        for a,b in itertools.combinations(normals,2))
    if separation<10.:raise ValueError('calibration_insufficient_view_tilt')
    if not np.isfinite(rms) or rms>.8 or not np.isfinite(view_errors).all() or view_errors.max()>2.:
        raise ValueError('calibration_reprojection_error_excessive')
    if (not np.isfinite(K).all() or not np.isfinite(distortion).all()
            or not np.isfinite(std_intrinsic).all() or not np.isfinite(std_extrinsic).all()
            or any(not np.isfinite(v).all() for v in (*rvecs,*tvecs))):
        raise ValueError('calibration_nonfinite_model')
    # FIX_K3 gives k1,k2,p1,p2,k3=0. Reject a radial model that folds within
    # the observed image, even if its central reprojection residual is small.
    radius=max(float(np.linalg.norm([(x-K[0,2])/K[0,0],(y-K[1,2])/K[1,1]]))
        for x in (0,size[0]) for y in (0,size[1]))
    r=np.linspace(0,radius,100);k1,k2,_,_,k3=distortion.ravel()[:5]
    if (1+3*k1*r*r+5*k2*r**4+7*k3*r**6<=0).any():
        raise ValueError('calibration_nonmonotonic_radial_model')
    provenance=(f'{name}; {len(observed)} views; '
        f'RMS {rms:.4f} px. Fixed image mode required; physical scale and acquisition clock not verified.')
    model=CameraModel(size[0],size[1],float(K[0,0]),float(K[1,1]),float(K[0,2]),float(K[1,2]),
        tuple(float(v) for v in distortion.ravel()),'calibrated',provenance)
    report={'format':'neo2-control/camera-calibration/v1','model':model.as_dict(),
        'opencv':cv.__version__,'columns':columns,'rows':rows,'pattern_units':'squares',
        'sources':sources,'accepted_views':len(observed),'rms_pixels':float(rms),
        'view_errors_pixels':view_errors.ravel().tolist(),'image_coverage_fraction':coverage.tolist(),
        'maximum_board_normal_angle_degrees':separation,
        'intrinsic_standard_deviations':std_intrinsic.ravel().tolist(),
        'board_poses':[{'rotation_vector':rv.ravel().tolist(),'translation_in_squares':tv.ravel().tolist()}
            for rv,tv in zip(rvecs,tvecs)],
        'metric_scale_verified':False,'scope':'Lens/image-mode calibration only; no flight, no metric trajectory validation.'}
    return model,report
