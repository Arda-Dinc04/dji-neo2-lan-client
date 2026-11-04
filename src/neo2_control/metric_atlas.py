"""Incremental scene landmarks in one measured reference, without flight I/O.

Each new pose is supported by existing landmarks. New points require two
translated views and retain their original image observations. The measured
seed is never rescaled or replaced. Drift and physical accuracy are unverified.
"""
import copy
import json
from pathlib import Path
import re

from .metric_reference import _digest, _rotation_error, _validate_metric_scan, metric_geometry
from .reconstruction import (
    CameraModel, _distinct_pixels, _features, _image, _json_file, _matches,
    _pixels, _read, _spread, _vision, localize_correspondences, triangulate_correspondences,
)

MAX_POINTS = 16000
MAX_EXTENSIONS = 16


def create_atlas(seed):
    _validate_metric_scan(seed)
    return {'format': 'neo2-control/metric-atlas/v1', 'seed': copy.deepcopy(seed),
        'seed_sha256': _digest(seed), 'coordinate_frame': seed['coordinate_frame'],
        'distance_unit': 'm', 'axes': 'opencv_charuco_board_xyz', 'extensions': [],
        'physical_accuracy_verified': False, 'tracking_accuracy_verified': False,
        'free_space_observed': False,
        'scope': 'Scene landmarks anchored to a declared measured reference. Existing points remain fixed. No bundle adjustment, loop-closure correction, verified acquisition clock or flight authority.'}


def _project(points, pose, model):
    cv, np = _vision()
    R, t = np.array(pose['world_to_camera_R']), np.array(pose['world_to_camera_t'])
    dist = np.array(model.distortion) if model.distortion else None
    pixels = cv.projectPoints(np.asarray(points, dtype=np.float64), cv.Rodrigues(R)[0],
        t, model.matrix(), dist)[0].reshape(-1, 2)
    positive = (np.asarray(points)@R.T+t)[:, 2] > 0
    return pixels, positive


def _validate_pose(pose, points, model):
    _, np = _vision()
    try:
        R, t = np.asarray(pose['world_to_camera_R']), np.asarray(pose['world_to_camera_t'])
        ids = np.asarray(pose['landmark_indices'])
        pixels = _pixels(pose['pixels'], model)
        if (R.shape != (3, 3) or t.shape != (3,) or not np.isfinite(R).all()
                or not np.isfinite(t).all() or not np.allclose(R@R.T, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(R), 1., atol=1e-6)
                or ids.ndim != 1 or ids.dtype.kind not in 'iu' or len(ids) != len(pixels)
                or (ids < 0).any() or (ids >= len(points)).any() or len(set(ids.tolist())) != len(ids)
                or len(_distinct_pixels(pixels, pixels)) != len(pixels)):
            raise ValueError
        _spread(pixels)
        projected, positive = _project(points[ids], pose, model)
        error = np.linalg.norm(projected-pixels, axis=1)
        if not positive.all() or not np.isfinite(error).all() or error.max() > 2.5:
            raise ValueError
    except (KeyError, TypeError, ValueError, OverflowError):
        raise ValueError('atlas_pose_evidence_invalid') from None


def _triangulate(first, second, model, poses):
    """Triangulate directly in the retained metric frame, using fixed poses."""
    cv, np = _vision()
    a, b = _pixels(first, model), _pixels(second, model)
    if len(a) != len(b):
        raise ValueError('atlas_correspondences_invalid')
    K = model.matrix()
    dist = np.array(model.distortion) if model.distortion else None
    pixels = [cv.undistortPoints(p[:, None, :], K, dist, P=K).reshape(-1, 2) for p in (a, b)]
    rotations = [np.array(p['world_to_camera_R']) for p in poses]
    translations = [np.array(p['world_to_camera_t']) for p in poses]
    centers = [-R.T@t for R, t in zip(rotations, translations)]
    baseline = float(np.linalg.norm(centers[1]-centers[0]))
    if not .05 <= baseline <= 3:
        raise ValueError('atlas_translational_baseline_required')
    projections = [K@np.column_stack((R, t)) for R, t in zip(rotations, translations)]
    raw = cv.triangulatePoints(*projections, pixels[0].T, pixels[1].T)
    with np.errstate(divide='ignore', invalid='ignore'):
        points = (raw[:3]/raw[3]).T
    errors, positive = [], np.isfinite(points).all(axis=1)
    for original, pose, center in zip((a, b), poses, centers):
        projected, front = _project(points, pose, model)
        errors.append(np.linalg.norm(projected-original, axis=1))
        distance = np.linalg.norm(points-center, axis=1)
        positive &= front & (distance >= .25) & (distance <= 8)
    rays = []
    for p, R in zip(pixels, rotations):
        ray = np.column_stack((p, np.ones(len(p))))@np.linalg.inv(K).T@R
        rays.append(ray/np.linalg.norm(ray, axis=1)[:, None])
    angle = np.degrees(np.arccos(np.clip((rays[0]*rays[1]).sum(axis=1), -1, 1)))
    error = np.maximum(*errors)
    keep = positive & np.isfinite(error) & (error <= 1.5) & (angle >= 1.)
    return points, keep, error, angle, baseline


def _novel_pixels(pixels, points, pose, model):
    cv, np = _vision()
    projected, positive = _project(points, pose, model)
    inside = positive & np.isfinite(projected).all(axis=1) & (projected >= 0).all(axis=1)
    inside &= (projected[:, 0] < model.width) & (projected[:, 1] < model.height)
    if not inside.any():
        return np.ones(len(pixels), dtype=bool)
    nearest = cv.BFMatcher(cv.NORM_L2).match(np.asarray(pixels, dtype=np.float32),
        projected[inside].astype(np.float32))
    # Pose-supported observations may deviate by up to 2.5 px; keep the
    # novelty threshold outside that admitted tracking residual.
    return np.array([match.distance > 3. for match in nearest])


def _assemble(atlas):
    _, np = _vision()
    try:
        seed = _validate_metric_scan(atlas['seed'])
        if (atlas['format'] != 'neo2-control/metric-atlas/v1'
                or atlas['seed_sha256'] != _digest(seed)
                or atlas['coordinate_frame'] != seed['coordinate_frame']
                or atlas['distance_unit'] != 'm' or atlas['axes'] != 'opencv_charuco_board_xyz'
                or any(atlas[key] is not False for key in ('physical_accuracy_verified',
                    'tracking_accuracy_verified', 'free_space_observed'))
                or not isinstance(atlas['extensions'], list) or len(atlas['extensions']) > MAX_EXTENSIONS):
            raise ValueError
        geometry = metric_geometry(seed)
        model = CameraModel.from_dict(geometry['camera_model'])
        points = np.array(geometry['points'])
        rgb = np.array(geometry['rgb'])
        descriptors = np.array(geometry['reference_descriptors'], dtype=np.float32)
        seen = set(seed['scan']['image_sha256'])
        known_poses = {h: {'world_to_camera_R': geometry[f'{label}_world_to_camera_R'],
            'world_to_camera_t': geometry[f'{label}_world_to_camera_t']}
            for h, label in zip(seed['scan']['image_sha256'], ('first', 'second'))}
        for extension in atlas['extensions']:
            hashes = extension['image_sha256']
            poses = extension['poses']
            if (extension['parent_point_count'] != len(points) or len(hashes) != 2
                    or hashes[0] == hashes[1] or all(h in seen for h in hashes)
                    or any(not re.fullmatch('[0-9a-f]{64}', h) for h in hashes) or len(poses) != 2):
                raise ValueError
            for source_hash, pose in zip(hashes, poses):
                _validate_pose(pose, points, model)
                if source_hash in known_poses:
                    previous = known_poses[source_hash]
                    old_R, new_R = np.array(previous['world_to_camera_R']), np.array(pose['world_to_camera_R'])
                    old_c = -old_R.T@np.array(previous['world_to_camera_t'])
                    new_c = -new_R.T@np.array(pose['world_to_camera_t'])
                    if _rotation_error(old_R, new_R) > .5 or np.linalg.norm(old_c-new_c) > .02:
                        raise ValueError
                else:
                    known_poses[source_hash] = pose
            a, b = (_pixels(extension[key], model) for key in ('first_pixels', 'second_pixels'))
            if len(_distinct_pixels(a, b)) != len(a):
                raise ValueError
            new = np.asarray(extension['points_m'], dtype=np.float64)
            colors = np.asarray(extension['rgb'])
            desc = np.asarray(extension['descriptors'], dtype=np.float32)
            if (not 20 <= len(new) <= 3000 or len(points)+len(new) > MAX_POINTS
                    or new.shape != (len(a), 3) or colors.shape != (len(a), 3)
                    or desc.shape != (len(a), 128) or not np.isfinite(new).all()
                    or not np.isfinite(desc).all() or (desc < 0).any() or (desc > 512).any()
                    or colors.dtype.kind not in 'iu' or (colors < 0).any() or (colors > 255).any()):
                raise ValueError
            derived, keep, _, _, _ = _triangulate(a, b, model, poses)
            if (not keep.all() or not np.allclose(new, derived, atol=1e-7, rtol=1e-7)
                    or not _novel_pixels(a, points, poses[0], model).all()
                    or not _novel_pixels(b, points, poses[1], model).all()):
                raise ValueError
            points = np.vstack((points, new))
            rgb = np.vstack((rgb, colors))
            descriptors = np.vstack((descriptors, desc))
            seen.update(hashes)
    except (KeyError, TypeError, ValueError, OverflowError):
        raise ValueError('metric_atlas_invalid') from None
    return model, points, rgb, descriptors


def _track(frame, model, points, descriptors,*,maximum_feature_pixels=None,descriptor_matcher=None):
    pixels, features = _features(frame,maximum_pixels=maximum_feature_pixels)
    pose,evidence=_fit_features(pixels,features,model,points,descriptors,descriptor_matcher=descriptor_matcher)
    return pose,evidence,pixels,features


def _fit_features(pixels,features,model,points,descriptors,*,descriptor_matcher=None):
    _, np = _vision()
    matches = _matches(descriptors, features) if descriptor_matcher is None else descriptor_matcher(features)
    if len(matches) < 20:
        raise ValueError('atlas_localization_insufficient_matches')
    ids, image_ids = np.array(matches).T
    pose = localize_correspondences(points[ids], pixels[image_ids], model, include_inlier_indices=True)
    selected = np.array(pose.pop('inlier_indices'))
    evidence = {key: pose[key] for key in ('world_to_camera_R', 'world_to_camera_t')}
    evidence.update(landmark_indices=ids[selected].tolist(), pixels=pixels[image_ids[selected]].tolist())
    return pose,evidence


def extend_atlas(atlas, first, second):
    cv, np = _vision()
    model, points, _, descriptors = _assemble(atlas)
    if len(atlas['extensions']) >= MAX_EXTENSIONS:
        raise ValueError('atlas_extension_limit')
    images = [_image(path, model) for path in (first, second)]
    hashes = [h for _, h in images]
    known = set(atlas['seed']['scan']['image_sha256'])
    for entry in atlas['extensions']:
        known.update(entry['image_sha256'])
    if hashes[0] == hashes[1] or all(h in known for h in hashes):
        raise ValueError('atlas_new_images_required')
    tracks = [_track(frame, model, points, descriptors) for frame, _ in images]
    poses = [track[1] for track in tracks]
    matches = _matches(tracks[0][3], tracks[1][3])
    if len(matches) < 20:
        raise ValueError('atlas_pair_insufficient_matches')
    i0, i1 = np.array(matches).T
    a, b = tracks[0][2][i0], tracks[1][2][i1]
    observed_a,observed_b=a,b
    # Independent two-view fitting rejects rotation-only pairs and checks
    # consistency with the poses supported by the previous map.
    independent = triangulate_correspondences(a, b, model)
    selected = np.array(independent['selected_indices'])
    a, b, i0 = a[selected], b[selected], i0[selected]
    relative_R = np.array(poses[1]['world_to_camera_R'])@np.array(poses[0]['world_to_camera_R']).T
    relative_t = np.array(poses[1]['world_to_camera_t'])-relative_R@np.array(poses[0]['world_to_camera_t'])
    baseline = np.linalg.norm(relative_t)
    if not .05 <= baseline <= 3:
        raise ValueError('atlas_translational_baseline_required')
    angle = np.degrees(np.arccos(np.clip(relative_t@np.array(independent['second_world_to_camera_t'])/baseline, -1, 1)))
    if _rotation_error(relative_R, np.array(independent['second_world_to_camera_R'])) > 2 or angle > 5:
        raise ValueError('atlas_relative_motion_inconsistent')
    from .pair_stability import assess_pair,PairStabilityError
    noise_check=assess_pair(observed_a,observed_b,model,_nominal=independent)
    noise_check['image_sha256']=hashes
    if not noise_check['passed']:raise PairStabilityError(noise_check)
    new, valid, error, _, _ = _triangulate(a, b, model, poses)
    valid &= _novel_pixels(a, points, poses[0], model) & _novel_pixels(b, points, poses[1], model)
    selected = np.flatnonzero(valid)
    selected = selected[np.argsort(error[selected], kind='stable')][:3000]
    if len(selected) < 20:
        raise ValueError('atlas_insufficient_novel_landmarks')
    if len(points)+len(selected) > MAX_POINTS:
        raise ValueError('atlas_point_limit')
    uv = np.rint(a[selected]).astype(int)
    uv[:, 0] = np.clip(uv[:, 0], 0, model.width-1)
    uv[:, 1] = np.clip(uv[:, 1], 0, model.height-1)
    entry = {'parent_point_count': len(points), 'image_sha256': hashes, 'poses': poses,
        'pair_noise_check':noise_check,
        'first_pixels': a[selected].tolist(), 'second_pixels': b[selected].tolist(),
        'points_m': new[selected].tolist(), 'descriptors': tracks[0][3][i0[selected]].tolist(),
        'rgb': images[0][0][uv[:, 1], uv[:, 0], ::-1].tolist()}
    result = copy.deepcopy(atlas)
    result['extensions'].append(entry)
    _assemble(result)
    return result


def localize_atlas(atlas, image):
    model, points, _, descriptors = _assemble(atlas)
    frame, source_hash = _image(image, model)
    pose, evidence, _, _ = _track(frame, model, points, descriptors)
    seed_count = len(atlas['seed']['scan']['points'])
    added = sum(index >= seed_count for index in evidence['landmark_indices'])
    return pose | {'distance_unit': 'm', 'coordinate_frame': atlas['coordinate_frame'],
        'image_sha256': source_hash, 'physical_accuracy_verified': False,
        'tracking_accuracy_verified': False, 'free_space_observed': False,
        'atlas_point_count': len(points), 'added_landmark_inliers': added,
        'seed_landmark_inliers': len(evidence['landmark_indices'])-added}


def save_atlas(path, atlas):
    _, points, _, _ = _assemble(atlas)
    if len(json.dumps(atlas, allow_nan=False)) > 60*1024*1024:
        raise ValueError('atlas_memory_limit')
    _json_file(path, atlas)
    return {'output': str(Path(path).absolute()), 'point_count': len(points),
        'extension_count': len(atlas['extensions']), 'coordinate_frame': atlas['coordinate_frame'],
        'distance_unit': 'm', 'physical_accuracy_verified': False, 'free_space_observed': False}


def load_atlas(path):
    value = json.loads(_read(path, 64*1024*1024))
    _assemble(value)
    return value


def atlas_geometry(atlas):
    _, points, rgb, _ = _assemble(atlas)
    seed = metric_geometry(atlas['seed'])
    hashes = list(seed['image_sha256'])
    for entry in atlas['extensions']:
        hashes.extend(h for h in entry['image_sha256'] if h not in hashes)
    return seed | {'points': points.tolist(), 'rgb': rgb.tolist(), 'kind': 'atlas',
        'image_sha256': hashes,
        'extension_count': len(atlas['extensions']),
        'new_point_count': len(points)-len(seed['points']),
        'keyframe_poses': [pose for entry in atlas['extensions'] for pose in entry['poses']]}


def atlas_pose_pair(atlas, extension_index):
    _, np = _vision()
    _assemble(atlas)
    if type(extension_index) is not int or not 0 <= extension_index < len(atlas['extensions']):
        raise ValueError('atlas_extension_index_invalid')
    entry = atlas['extensions'][extension_index]
    first, second = entry['poses']
    R = np.array(second['world_to_camera_R'])@np.array(first['world_to_camera_R']).T
    t = np.array(second['world_to_camera_t'])-R@np.array(first['world_to_camera_t'])
    return {'coordinate_frame': atlas['coordinate_frame'],
        'camera_model': atlas['seed']['scan']['camera_model'],
        'image_sha256': entry['image_sha256'], 'relative_R': R.tolist(),
        'relative_t_m': t.tolist(), 'baseline_m': float(np.linalg.norm(t)),
        'first_pixels': entry['first_pixels'], 'second_pixels': entry['second_pixels'],
        'first_pose': first}


def densify_atlas(atlas, extension_index, first, second, **options):
    from .depth_scan import _densify_pair
    pair = atlas_pose_pair(atlas, extension_index)
    source = {'format': 'neo2-control/depth-scan/v2', 'source_metric_atlas': atlas,
        'source_metric_atlas_sha256': _digest(atlas), 'source_extension_index': extension_index}
    return _densify_pair(pair, source, first, second, **options)
