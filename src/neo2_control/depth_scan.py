"""Offline multi-view depth, retaining unknown pixels and measured-reference units.

This is temporal stereo from one camera, not a hardware stereo depth sensor.
Static scene, optical mode and camera poses are assumptions to validate on the
aircraft. Match consistency is not a physical error bound or free-space proof.
"""
import json
import math
import re
from pathlib import Path

from .reconstruction import CameraModel, _finite, _image, _json_file, _read, _vision
from .metric_reference import _digest, _validate_metric_scan


def _rectify(model, rotation, translation):
    cv, np = _vision()
    K = model.matrix()
    dist = np.array(model.distortion, dtype=np.float64) if model.distortion else None
    return cv.stereoRectify(K, dist, K, dist, (model.width, model.height),
        rotation, translation, flags=cv.CALIB_ZERO_DISPARITY, alpha=0)


def _stereo_measurements(first, second, model, rotation, translation, *, minimum_m=.25, maximum_m=4.):
    cv, np = _vision()
    if (not isinstance(model, CameraModel) or model.status != 'calibrated'
            or model.width*model.height > 1280*720
            or any(image.shape != (model.height, model.width, 3) or image.dtype != np.uint8
                for image in (first, second))
            or not _finite(minimum_m) or not _finite(maximum_m)
            or not .15 <= minimum_m < maximum_m <= 8.):
        raise ValueError('depth_images_model_or_range_invalid')
    R = np.asarray(rotation, dtype=np.float64)
    t = np.asarray(translation, dtype=np.float64)
    if (R.shape != (3, 3) or t.shape != (3,) or not np.isfinite(R).all()
            or not np.isfinite(t).all() or not np.allclose(R@R.T, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(R), 1., atol=1e-6)
            or not .05 <= np.linalg.norm(t) <= 3):
        raise ValueError('depth_relative_pose_invalid')
    r0, r1, p0, p1, Q, roi0, roi1 = _rectify(model, R, t)
    if (any(not np.isfinite(matrix).all() for matrix in (r0, r1, p0, p1, Q))
            or p0[0, 0] <= 0 or p0[1, 1] <= 0):
        raise ValueError('depth_rectification_invalid')
    axis = int(abs(p1[1, 3]) > abs(p1[0, 3]))
    sign = 1 if p1[axis, 3] < 0 else -1
    dist = np.array(model.distortion) if model.distortion else None
    images = []
    for image, r, p in zip((first, second), (r0, r1), (p0, p1)):
        maps = cv.initUndistortRectifyMap(model.matrix(), dist, r, p,
            (model.width, model.height), cv.CV_32FC1)
        images.append(cv.remap(image, *maps, cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT))
    gray = [cv.cvtColor(image, cv.COLOR_BGR2GRAY) for image in images]
    if axis:
        gray = [image.T.copy() for image in gray]
        roi0, roi1 = [(y, x, h, w) for x, y, w, h in (roi0, roi1)]
    height, width = gray[0].shape
    requested = math.ceil((p0[0, 0]*np.linalg.norm(t)/minimum_m+4)/16)*16
    disparities = min(requested, 512, ((width-16)//16)*16)
    if disparities < 16:
        raise ValueError('depth_search_image_too_small')
    minima = [0, -disparities] if sign > 0 else [-disparities, 0]
    def matcher(minimum):
        return cv.StereoSGBM_create(minDisparity=minimum, numDisparities=disparities,
            blockSize=5, P1=8*25, P2=32*25, disp12MaxDiff=1, preFilterCap=31,
            uniquenessRatio=15, speckleWindowSize=80, speckleRange=1,
            mode=cv.STEREO_SGBM_MODE_SGBM_3WAY)
    left = matcher(minima[0]).compute(*gray).astype(np.float32)/16
    right = matcher(minima[1]).compute(*gray[::-1]).astype(np.float32)/16
    yy, xx = np.indices(left.shape, dtype=np.float32)
    matched_x = xx-left
    reverse = cv.remap(right, matched_x, yy, cv.INTER_NEAREST,
        borderMode=cv.BORDER_CONSTANT, borderValue=float('nan'))
    roi = cv.getValidDisparityROI(roi0, roi1, minima[0], disparities, 5)
    valid_roi = np.zeros(left.shape, dtype=np.uint8)
    x, y, w, h = roi
    valid_roi[y:y+h, x:x+w] = 1
    valid = ((valid_roi != 0) & (left > minima[0])
        & (left < minima[0]+disparities-2) & (reverse > minima[1])
        & (reverse < minima[1]+disparities-2) & (np.abs(left+reverse) <= .75)
        & (sign*left > 1.) & (matched_x >= 2) & (matched_x < width-3))
    warped = cv.remap(gray[1].astype(np.float32), matched_x, yy, cv.INTER_LINEAR,
        borderMode=cv.BORDER_CONSTANT)
    a, b = gray[0].astype(np.float32), warped
    mean = lambda image: cv.boxFilter(image, -1, (7, 7), normalize=True)
    ma, mb = mean(a), mean(b)
    va, vb = np.maximum(mean(a*a)-ma*ma, 0), np.maximum(mean(b*b)-mb*mb, 0)
    correlation = (mean(a*b)-ma*mb)/np.sqrt(np.maximum(va*vb, 1.))
    valid &= (va >= 64) & (vb >= 64) & (correlation >= .9)
    # Do not turn a foreground/background boundary into interpolated clearance.
    neighborhood = np.ones((3, 3), np.uint8)
    jump = cv.dilate(left, neighborhood)-cv.erode(left, neighborhood)
    valid &= jump <= 1.
    valid = cv.erode(valid.astype(np.uint8), neighborhood) != 0
    if axis:
        left, valid = left.T.copy(), valid.T.copy()
    points = cv.reprojectImageTo3D(left, Q)
    depth = points[:, :, 2]
    valid &= np.isfinite(points).all(axis=2) & (depth >= minimum_m) & (depth <= maximum_m)
    sensitivity = p0[0, 0]*float(np.linalg.norm(t))/np.maximum(left*left, 1.)
    return {'depth': depth, 'valid': valid, 'rgb': images[0][:, :, ::-1],
        'sensitivity': sensitivity, 'rectification_R': r0, 'rectified_K': p0[:, :3],
        'second_rectification_R': r1, 'second_projection': p1,
        'disparity_axis': 'vertical' if axis else 'horizontal',
        'number_of_disparities': int(disparities), 'search_range_limited': disparities < requested,
        'full_resolution_valid_fraction': float(valid.mean())}


def _metric_pair(scan):
    _, np = _vision()
    _validate_metric_scan(scan)
    base = scan['scan']
    return {'coordinate_frame': scan['coordinate_frame'], 'camera_model': base['camera_model'],
        'image_sha256': base['image_sha256'], 'relative_R': base['second_world_to_camera_R'],
        'relative_t_m': (scan['metres_per_baseline']*np.array(base['second_world_to_camera_t'])).tolist(),
        'first_pixels': base['reference_pixels'], 'second_pixels': base['second_pixels'],
        'first_pose': scan['reference_poses'][0], 'baseline_m': scan['metres_per_baseline']}


def densify_scan(scan, first, second, *, stride=4, minimum_m=.25, maximum_m=4.):
    """Depth in the existing map frame, only from that map's exact source images."""
    pair = _metric_pair(scan)
    source = {'format': 'neo2-control/depth-scan/v1',
        'source_metric_scan_sha256': _digest(scan), 'source_metric_scan': scan}
    return _densify_pair(pair, source, first, second, stride=stride,
        minimum_m=minimum_m, maximum_m=maximum_m)


def _densify_pair(pair, source, first, second, *, stride=4, minimum_m=.25, maximum_m=4.):
    """Shared engine, called only after the parent pose evidence is validated."""
    cv, np = _vision()
    if type(stride) is not int or not 4 <= stride <= 16:
        raise ValueError('depth_stride_invalid')
    model = CameraModel.from_dict(pair['camera_model'])
    images = [_image(path, model) for path in (first, second)]
    if [source_hash for _, source_hash in images] != pair['image_sha256']:
        raise ValueError('depth_source_images_mismatch')
    R = np.array(pair['relative_R'])
    t = np.array(pair['relative_t_m'])
    stereo = _stereo_measurements(images[0][0], images[1][0], model, R, t,
        minimum_m=minimum_m, maximum_m=maximum_m)
    # Check that rectification actually aligns the independently observed
    # sparse image matches, before admitting a dense result.
    dist = np.array(model.distortion) if model.distortion else None
    pixels = [cv.undistortPoints(np.array(pair[key], dtype=np.float64)[:, None, :],
        model.matrix(), dist, R=rectification, P=projection).reshape(-1, 2)
        for key, rectification, projection in (
            ('first_pixels', stereo['rectification_R'], stereo['rectified_K']),
            ('second_pixels', stereo['second_rectification_R'], stereo['second_projection']))]
    perpendicular = 0 if stereo['disparity_axis'] == 'vertical' else 1
    epipolar_error = np.abs(pixels[0][:, perpendicular]-pixels[1][:, perpendicular])
    if (epipolar_error <= 1.).mean() < .9:
        raise ValueError('depth_epipolar_alignment_invalid')
    valid = stereo['valid'][::stride, ::stride]
    depths = stereo['depth'][::stride, ::stride]
    sensitivities = stereo['sensitivity'][::stride, ::stride]
    pose = pair['first_pose']
    return {**source, 'coordinate_frame': pair['coordinate_frame'],
        'axes': 'opencv_charuco_board_xyz', 'distance_unit': 'm',
        'camera_model': model.as_dict(), 'image_sha256': pair['image_sha256'],
        'scale_source': 'declared_measured_charuco_square',
        'metres_per_baseline': pair['baseline_m'],
        'first_world_to_camera_R': pose['world_to_camera_R'],
        'first_world_to_camera_t': pose['world_to_camera_t'],
        'relative_R': R.tolist(), 'relative_t_m': t.tolist(),
        'rectification_R': stereo['rectification_R'].tolist(),
        'rectified_K': stereo['rectified_K'].tolist(),
        'stride': stride, 'grid_width': valid.shape[1], 'grid_height': valid.shape[0],
        'depth_m': [[float(z) if ok else None for z, ok in zip(row, mask)]
            for row, mask in zip(depths, valid)],
        'one_pixel_depth_sensitivity_m': [[float(z) if ok else None for z, ok in zip(row, mask)]
            for row, mask in zip(sensitivities, valid)],
        'rgb': stereo['rgb'][::stride, ::stride].tolist(),
        'minimum_depth_m': minimum_m, 'maximum_depth_m': maximum_m,
        'accepted_sample_count': int(valid.sum()),
        'full_resolution_valid_fraction': stereo['full_resolution_valid_fraction'],
        'maximum_sparse_epipolar_error_px': float(epipolar_error.max()),
        'number_of_disparities': stereo['number_of_disparities'],
        'search_range_limited': stereo['search_range_limited'],
        'disparity_axis': stereo['disparity_axis'], 'opencv': cv.__version__,
        'physical_accuracy_verified': False, 'free_space_observed': False,
        'acquisition_synchronization_verified': False,
        'scope': 'Temporal stereo requires a static scene and validated image-mode/poses. Unknown pixels remain unknown. One-pixel sensitivity is not a calibrated error bound. No flight authority.'}


def _validate_depth_scan(value):
    _, np = _vision()
    try:
        if value['format'] == 'neo2-control/depth-scan/v1':
            parent = value['source_metric_scan']
            pair = _metric_pair(parent)
            source_hash_key = 'source_metric_scan_sha256'
        elif value['format'] == 'neo2-control/depth-scan/v2':
            from .metric_atlas import atlas_pose_pair
            parent = value['source_metric_atlas']
            pair = atlas_pose_pair(parent, value['source_extension_index'])
            source_hash_key = 'source_metric_atlas_sha256'
        else:
            raise ValueError
        model = CameraModel.from_dict(value['camera_model'])
        stride, w, h = value['stride'], value['grid_width'], value['grid_height']
        if (model.status != 'calibrated'
                or model.width*model.height > 1280*720 or type(stride) is not int or not 4 <= stride <= 16
                or (w, h) != ((model.width+stride-1)//stride, (model.height+stride-1)//stride)
                or value['axes'] != 'opencv_charuco_board_xyz' or value['distance_unit'] != 'm'
                or value['scale_source'] != 'declared_measured_charuco_square'
                or any(value[key] is not False for key in ('physical_accuracy_verified',
                    'free_space_observed', 'acquisition_synchronization_verified'))
                or any(not re.fullmatch('[0-9a-f]{64}', value[key])
                    for key in ('coordinate_frame', source_hash_key))
                or len(value['image_sha256']) != 2
                or any(not re.fullmatch('[0-9a-f]{64}', item) for item in value['image_sha256'])):
            raise ValueError
        if (value[source_hash_key] != _digest(parent)
                or value['coordinate_frame'] != pair['coordinate_frame']
                or value['camera_model'] != pair['camera_model']
                or value['image_sha256'] != pair['image_sha256']
                or value['metres_per_baseline'] != pair['baseline_m']
                or value['first_world_to_camera_R'] != pair['first_pose']['world_to_camera_R']
                or value['first_world_to_camera_t'] != pair['first_pose']['world_to_camera_t']
                or value['relative_R'] != pair['relative_R']):
            raise ValueError
        lo, hi = value['minimum_depth_m'], value['maximum_depth_m']
        if not _finite(lo) or not _finite(hi) or not .15 <= lo < hi <= 8:
            raise ValueError
        for key in ('rectification_R', 'relative_R', 'first_world_to_camera_R'):
            R = np.asarray(value[key], dtype=np.float64)
            if (R.shape != (3, 3) or not np.isfinite(R).all()
                    or not np.allclose(R@R.T, np.eye(3), atol=1e-6)
                    or not np.isclose(np.linalg.det(R), 1., atol=1e-6)):
                raise ValueError
        for key in ('first_world_to_camera_t', 'relative_t_m'):
            t = np.asarray(value[key], dtype=np.float64)
            if t.shape != (3,) or not np.isfinite(t).all():
                raise ValueError
        baseline = value['metres_per_baseline']
        if (not _finite(baseline) or not .05 <= baseline <= 3
                or not np.isclose(np.linalg.norm(value['relative_t_m']), baseline, atol=1e-7)
                or not np.allclose(value['relative_t_m'], pair['relative_t_m'], atol=1e-7)):
            raise ValueError
        K = np.asarray(value['rectified_K'], dtype=np.float64)
        if (K.shape != (3, 3) or not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0
                or not np.allclose(K[[0, 1, 2, 2, 2], [1, 0, 0, 1, 2]], [0, 0, 0, 0, 1])):
            raise ValueError
        expected_r, _, expected_p, *_ = _rectify(model, np.array(value['relative_R']),
            np.array(value['relative_t_m']))
        if (not np.allclose(value['rectification_R'], expected_r, atol=1e-7)
                or not np.allclose(K, expected_p[:, :3], atol=1e-7)):
            raise ValueError
        for key in ('depth_m', 'one_pixel_depth_sensitivity_m'):
            if any(item is not None and not _finite(item) for row in value[key] for item in row):
                raise ValueError
        depth = np.asarray(value['depth_m'], dtype=np.float64)
        sensitivity = np.asarray(value['one_pixel_depth_sensitivity_m'], dtype=np.float64)
        rgb = np.asarray(value['rgb'])
        valid = np.isfinite(depth)
        if (depth.shape != (h, w) or sensitivity.shape != (h, w) or rgb.shape != (h, w, 3)
                or np.isinf(depth).any() or np.isinf(sensitivity).any()
                or not np.array_equal(np.isfinite(sensitivity), valid)
                or (depth[valid] < lo).any() or (depth[valid] > hi).any()
                or (sensitivity[valid] <= 0).any() or rgb.dtype.kind not in 'iu'
                or (rgb < 0).any() or (rgb > 255).any()
                or type(value['accepted_sample_count']) is not int
                or value['accepted_sample_count'] != valid.sum()
                or not np.allclose(sensitivity[valid], depth[valid]**2/(K[0, 0]*baseline), rtol=1e-5, atol=1e-7)
                or not _finite(value['full_resolution_valid_fraction'])
                or not 0 <= value['full_resolution_valid_fraction'] <= 1):
            raise ValueError
    except (KeyError, TypeError, ValueError, OverflowError):
        raise ValueError('depth_scan_invalid') from None
    return value


def save_depth_scan(path, scan):
    _validate_depth_scan(scan)
    _json_file(path, scan)
    return {'output': str(Path(path).absolute()), 'accepted_sample_count': scan['accepted_sample_count'],
        'distance_unit': 'm', 'physical_accuracy_verified': False, 'free_space_observed': False}


def load_depth_scan(path):
    return _validate_depth_scan(json.loads(_read(path, 64*1024*1024)))


def depth_geometry(scan):
    """Reconstruct point coordinates from the retained pixel grid and poses."""
    _, np = _vision()
    _validate_depth_scan(scan)
    depths = np.asarray(scan['depth_m'], dtype=np.float64)
    rows, columns = np.nonzero(np.isfinite(depths))
    z = depths[rows, columns]
    K = np.array(scan['rectified_K'])
    camera = np.column_stack(((columns*scan['stride']-K[0, 2])/K[0, 0]*z,
        (rows*scan['stride']-K[1, 2])/K[1, 1]*z, z))
    rectification = np.array(scan['rectification_R'])
    R = np.array(scan['first_world_to_camera_R'])
    t = np.array(scan['first_world_to_camera_t'])
    world = (camera@rectification-t)@R
    R1 = np.array(scan['relative_R'])@R
    t1 = np.array(scan['relative_R'])@t+np.array(scan['relative_t_m'])
    return {'points': world.tolist(), 'rgb': np.asarray(scan['rgb'])[rows, columns].tolist(),
        'camera_model': scan['camera_model'], 'image_sha256': scan['image_sha256'],
        'coordinate_frame': scan['coordinate_frame'], 'distance_unit': 'm',
        'metric_scale_verified': False, 'physical_accuracy_verified': False,
        'free_space_observed': False, 'kind': 'depth',
        'first_world_to_camera_R': R.tolist(), 'first_world_to_camera_t': t.tolist(),
        'second_world_to_camera_R': R1.tolist(), 'second_world_to_camera_t': t1.tolist(),
        'second_camera_center': (-R1.T@t1).tolist(),
        'depth_m': scan['depth_m'], 'minimum_depth_m': scan['minimum_depth_m'],
        'maximum_depth_m': scan['maximum_depth_m'],
        'grid_width': scan['grid_width'], 'grid_height': scan['grid_height'],
        'full_resolution_valid_fraction': scan['full_resolution_valid_fraction']}
