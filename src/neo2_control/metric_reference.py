"""Image-only metric reference and persistent map anchoring, without flight I/O.

Metres come from an explicitly measured physical board, never from nominal
print dimensions or a good reprojection score. The reference must remain fixed.
Neither the sparse map nor the board measures unobserved free volume.
"""
from dataclasses import dataclass
import hashlib
import html
import json
import math
import os
from pathlib import Path
import re

from .reconstruction import (
    CameraModel, _finite, _image, _json_file, _read, _validate_scan, _vision,
    localize_scan,
)


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False,
        separators=(',', ':')).encode()).hexdigest()


@dataclass(frozen=True)
class ReferenceBoard:
    reference_id: str
    first_marker_id: int = 0

    def __post_init__(self):
        if (not isinstance(self.reference_id, str)
                or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', self.reference_id)
                or type(self.first_marker_id) is not int
                or not 0 <= self.first_marker_id <= 233):
            raise ValueError('reference_board_invalid')

    def as_dict(self):
        return {'format': 'neo2-control/charuco-reference/v1',
            'reference_id': self.reference_id, 'first_marker_id': self.first_marker_id,
            'dictionary': 'DICT_4X4_250', 'squares_x': 7, 'squares_y': 5,
            'marker_to_square_ratio': 24/35,
            'nominal_print_square_mm': 35, 'legacy_pattern': False}

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict):
            raise ValueError('reference_board_invalid')
        try:
            board = cls(value['reference_id'], value['first_marker_id'])
            if value != board.as_dict():
                raise ValueError
        except (KeyError, TypeError, ValueError):
            raise ValueError('reference_board_invalid') from None
        return board

    def opencv_board(self, square_m):
        cv, np = _vision()
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        return cv.aruco.CharucoBoard((7, 5), square_m, square_m*24/35,
            dictionary, np.arange(self.first_marker_id, self.first_marker_id+17,
                dtype=np.int32))


def read_reference_board(path):
    return ReferenceBoard.from_dict(json.loads(_read(path, 16384)))


def write_reference_board(board, output):
    """A4 SVG with exact marker bits, no raster resampling or inferred scale."""
    if not isinstance(board, ReferenceBoard):
        raise ValueError('reference_board_invalid')
    cv, _ = _vision()
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
    # Actual OpenCV marker layout, including the odd-row convention, is
    # obtained from the same board definition used by detection.
    native = board.opencv_board(.035)
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="297mm" height="210mm" '
        'viewBox="0 0 297 210"><rect width="297" height="210" fill="white"/>',
        '<g fill="black" shape-rendering="crispEdges">']
    for y in range(5):
        for x in range(7):
            if (x+y) % 2 == 0:
                parts.append(f'<rect x="{26+35*x}" y="{17.5+35*y}" width="35" height="35"/>')
    for marker_id, corners in zip(native.getIds(), native.getObjPoints()):
        x, y = corners[0, :2]*1000
        bits = cv.aruco.generateImageMarker(dictionary, int(marker_id), 6)
        for row in range(6):
            for column in range(6):
                if bits[row, column] == 0:
                    parts.append(f'<rect x="{26+x+4*column:.6f}" '
                        f'y="{17.5+y+4*row:.6f}" width="4" height="4"/>')
    label = html.escape(board.reference_id)
    parts.extend(['</g>', f'<text x="26" y="9" font-family="sans-serif" font-size="4">'
        f'Neo 2 — {label} — ChArUco 7 × 5 / 4X4_250 / IDs {board.first_marker_id}–{board.first_marker_id+16}</text>',
        '<text x="26" y="202" font-family="sans-serif" font-size="3.5">'
        'Print at 100%. Nominal square: 35 mm. Measure the real square before metric use. Keep the board fixed.</text></svg>'])
    fd = os.open(output, os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW|os.O_CLOEXEC, 0o600)
    with os.fdopen(fd, 'w', encoding='utf-8') as stream:
        stream.write(''.join(parts))
    return {'output': str(Path(output).absolute()), 'board': board.as_dict(),
        'physical_size_verified': False}


def _measurement(square_mm, note):
    if (not _finite(square_mm) or not 5 <= square_mm <= 300
            or not isinstance(note, str) or not 1 <= len(note.strip()) <= 512):
        raise ValueError('reference_physical_measurement_required')


def _solve_reference(board, model, square_mm, corner_ids, pixels):
    cv, np = _vision()
    if not isinstance(model, CameraModel) or model.status != 'calibrated':
        raise ValueError('reference_calibrated_camera_required')
    ids = np.asarray(corner_ids)
    pixels = np.asarray(pixels, dtype=np.float64)
    if (ids.ndim != 1 or not 12 <= len(ids) <= 24 or ids.dtype.kind not in 'iu'
            or len(set(ids.tolist())) != len(ids) or (ids < 0).any() or (ids >= 24).any()
            or pixels.shape != (len(ids), 2) or not np.isfinite(pixels).all()
            or (pixels < 0).any() or (pixels[:, 0] >= model.width).any()
            or (pixels[:, 1] >= model.height).any()):
        raise ValueError('reference_corners_invalid')
    native = board.opencv_board(square_mm/1000)
    objects = np.asarray(native.getChessboardCorners(), dtype=np.float64)[ids]
    if (np.ptp(objects[:, :2], axis=0) < square_mm/1000*1.9).any():
        raise ValueError('reference_corner_coverage_insufficient')
    if cv.contourArea(cv.convexHull(pixels.astype(np.float32))) < 800:
        raise ValueError('reference_image_coverage_insufficient')
    K = model.matrix()
    distortion = np.array(model.distortion, dtype=np.float64) if model.distortion else None
    count, vectors, translations, _ = cv.solvePnPGeneric(objects, pixels, K,
        distortion, flags=cv.SOLVEPNP_IPPE)
    candidates = []
    for rv, tv in zip(vectors, translations):
        R = cv.Rodrigues(rv)[0]
        projection = cv.projectPoints(objects, rv, tv, K, distortion)[0].reshape(-1, 2)
        error = np.linalg.norm(projection-pixels, axis=1)
        if (np.isfinite(error).all() and ((objects@R.T+tv.T)[:, 2] > 0).all()
                and (-R.T@tv).ravel()[2] < 0):
            candidates.append((float(np.sqrt(np.mean(error**2))), rv, tv))
    if not count or not candidates:
        raise ValueError('reference_pose_not_found')
    candidates.sort(key=lambda candidate: candidate[0])
    best, rv, tv = candidates[0]
    separation = None
    if len(candidates) > 1:
        other, rv2, tv2 = candidates[1]
        separation = other-best
        R, R2 = cv.Rodrigues(rv)[0], cv.Rodrigues(rv2)[0]
        distinct = (_rotation_error(R, R2) > 2
            or np.linalg.norm(-R.T@tv+R2.T@tv2) > .02)
        if distinct and separation < .25:
            raise ValueError('reference_planar_pose_ambiguous')
    rv, tv = cv.solvePnPRefineLM(objects, pixels, K, distortion, rv, tv)
    R = cv.Rodrigues(rv)[0]
    projected = cv.projectPoints(objects, rv, tv, K, distortion)[0].reshape(-1, 2)
    errors = np.linalg.norm(projected-pixels, axis=1)
    center = (-R.T@tv).ravel()
    if (not np.isfinite(center).all() or errors.max() > 1.5
            or np.sqrt(np.mean(errors**2)) > .6 or center[2] >= 0
            or ((objects@R.T+tv.T)[:, 2] <= 0).any()
            or abs(R[2, 2]) < math.cos(math.radians(75))):
        raise ValueError('reference_pose_fit_invalid')
    return {'world_to_camera_R': R.tolist(), 'world_to_camera_t': tv.ravel().tolist(),
        'camera_center': center.tolist(), 'maximum_reprojection_error_px': float(errors.max()),
        'rms_reprojection_error_px': float(np.sqrt(np.mean(errors**2))),
        'alternative_rms_gap_px': separation}


def _rotation_error(first, second):
    _, np = _vision()
    return float(np.degrees(np.arccos(np.clip((np.trace(first@second.T)-1)/2, -1, 1))))


def estimate_reference_pose(image, model, board, *, measured_square_mm, measurement_note):
    """Detect real marker IDs, retaining pixels and the declared measurement."""
    _measurement(measured_square_mm, measurement_note)
    if not isinstance(board, ReferenceBoard):
        raise ValueError('reference_board_invalid')
    if not isinstance(model, CameraModel) or model.status != 'calibrated':
        raise ValueError('reference_calibrated_camera_required')
    cv, np = _vision()
    frame, source_hash = _image(image, model)
    parameters = cv.aruco.CharucoParameters()
    parameters.cameraMatrix = model.matrix()
    parameters.distCoeffs = np.array(model.distortion) if model.distortion else None
    detector = cv.aruco.CharucoDetector(board.opencv_board(measured_square_mm/1000), parameters)
    corners, ids, _, marker_ids = detector.detectBoard(frame)
    if ids is None or marker_ids is None or len(marker_ids) < 4:
        raise ValueError('reference_board_not_found')
    ids, pixels = ids.ravel().tolist(), corners.reshape(-1, 2).tolist()
    pose = _solve_reference(board, model, measured_square_mm, ids, pixels)
    reference = {'board': board.as_dict(), 'measured_square_mm': float(measured_square_mm)}
    return {'format': 'neo2-control/reference-pose/v1', 'reference': reference,
        'coordinate_frame': _digest(reference), 'axes': 'opencv_charuco_board_xyz',
        'distance_unit': 'm', 'camera_model': model.as_dict(), 'image_sha256': source_hash,
        'measurement_note': measurement_note, 'corner_ids': ids, 'corners_px': pixels,
        'opencv': cv.__version__, 'physical_accuracy_verified': False,
        'free_space_observed': False, 'reference_must_remain_fixed': True, **pose}


def _validate_reference_pose(value):
    _, np = _vision()
    try:
        reference = value['reference']
        if set(reference) != {'board', 'measured_square_mm'}:
            raise ValueError
        board = ReferenceBoard.from_dict(reference['board'])
        model = CameraModel.from_dict(value['camera_model'])
        _measurement(reference['measured_square_mm'], value['measurement_note'])
        if (value['format'] != 'neo2-control/reference-pose/v1'
                or value['coordinate_frame'] != _digest(reference)
                or value['axes'] != 'opencv_charuco_board_xyz' or value['distance_unit'] != 'm'
                or not re.fullmatch('[0-9a-f]{64}', value['image_sha256'])
                or value['physical_accuracy_verified'] is not False
                or value['free_space_observed'] is not False
                or value['reference_must_remain_fixed'] is not True):
            raise ValueError
        pose = _solve_reference(board, model, reference['measured_square_mm'],
            value['corner_ids'], value['corners_px'])
        for field in pose:
            actual = value[field]
            if pose[field] is None:
                if actual is not None:
                    raise ValueError
            else:
                a, b = np.asarray(actual), np.asarray(pose[field])
                if a.shape != b.shape or not np.allclose(a, b, atol=1e-7, rtol=1e-7):
                    raise ValueError
    except (KeyError, TypeError, ValueError, OverflowError):
        raise ValueError('reference_pose_invalid') from None
    return value


def read_reference_pose(path):
    return _validate_reference_pose(json.loads(_read(path, 65536)))


def anchor_scan(scan, first_pose, second_pose):
    """Bind the sparse map to two independent, same-image metric board poses."""
    cv, np = _vision()
    _validate_scan(scan)
    for pose in (first_pose, second_pose):
        _validate_reference_pose(pose)
    if (first_pose['reference'] != second_pose['reference']
            or [first_pose['image_sha256'], second_pose['image_sha256']] != scan['image_sha256']
            or any(p['camera_model'] != scan['camera_model'] for p in (first_pose, second_pose))):
        raise ValueError('metric_anchor_source_mismatch')
    R0, R1 = (np.array(p['world_to_camera_R']) for p in (first_pose, second_pose))
    t0, t1 = (np.array(p['world_to_camera_t']) for p in (first_pose, second_pose))
    c0, c1 = (np.array(p['camera_center']) for p in (first_pose, second_pose))
    scale = float(np.linalg.norm(c1-c0))
    if not .05 <= scale <= 3:
        raise ValueError('metric_anchor_baseline_invalid')
    relative_R = R1@R0.T
    relative_t = (t1-relative_R@t0)/scale
    rotation_error = _rotation_error(relative_R, np.array(scan['second_world_to_camera_R']))
    direction_error = float(np.degrees(np.arccos(np.clip(
        relative_t@np.array(scan['second_world_to_camera_t']), -1, 1))))
    points = scale*np.array(scan['points'])@R0+c0
    model = CameraModel.from_dict(scan['camera_model'])
    distortion = np.array(model.distortion) if model.distortion else None
    projected = cv.projectPoints(points, cv.Rodrigues(R1)[0], t1, model.matrix(),
        distortion)[0].reshape(-1, 2)
    errors = np.linalg.norm(projected-np.array(scan['second_pixels']), axis=1)
    # This is agreement between two independently fitted poses, not the
    # feature fit's subpixel residual. A slightly biased planar pose can
    # differ by several pixels over the scene despite subpixel board fits.
    # Retain the discrepancy; do not interpret this admission as accuracy.
    if (rotation_error > 2 or direction_error > 5 or not np.isfinite(errors).all()
            or errors.max() > 10):
        raise ValueError('metric_anchor_geometry_inconsistent')
    return {'format': 'neo2-control/metric-scan/v1', 'scan': scan,
        'reference_poses': [first_pose, second_pose],
        'coordinate_frame': first_pose['coordinate_frame'],
        'distance_unit': 'm', 'metres_per_baseline': scale,
        'scale_source': 'declared_measured_charuco_square',
        'physical_accuracy_verified': False, 'free_space_observed': False,
        'maximum_anchor_reprojection_error_px': float(errors.max()),
        'rotation_agreement_degrees': rotation_error,
        'translation_direction_agreement_degrees': direction_error}


def _validate_metric_scan(value):
    try:
        if len(value['reference_poses']) != 2:
            raise ValueError
        rebuilt = anchor_scan(value['scan'], *value['reference_poses'])
        if value != rebuilt:
            raise ValueError
    except (KeyError, TypeError, ValueError, OverflowError):
        raise ValueError('metric_scan_invalid') from None
    return value


def save_metric_scan(path, scan):
    _validate_metric_scan(scan)
    _json_file(path, scan)
    return {'output': str(Path(path).absolute()), 'distance_unit': 'm',
        'coordinate_frame': scan['coordinate_frame'], 'point_count': len(scan['scan']['points']),
        'metres_per_baseline': scan['metres_per_baseline'],
        'physical_accuracy_verified': False, 'free_space_observed': False}


def load_metric_scan(path):
    return _validate_metric_scan(json.loads(_read(path, 32*1024*1024)))


def load_client_scan(path):
    value = json.loads(_read(path, 64*1024*1024))
    if isinstance(value, dict) and value.get('format') == 'neo2-control/metric-atlas/v1':
        from .metric_atlas import _assemble
        _assemble(value)
        return value
    if isinstance(value, dict) and value.get('format') == 'neo2-control/spatial-memory/v1':
        from .spatial_memory import _validate_memory
        return _validate_memory(value)
    if isinstance(value, dict) and value.get('format') in ('neo2-control/depth-scan/v1','neo2-control/depth-scan/v2'):
        from .depth_scan import _validate_depth_scan
        return _validate_depth_scan(value)
    if isinstance(value, dict) and value.get('format') == 'neo2-control/metric-scan/v1':
        return _validate_metric_scan(value)
    return _validate_scan(value)


def metric_geometry(scan):
    """Derived geometry for rendering, retaining measured-reference caveats."""
    _, np = _vision()
    _validate_metric_scan(scan)
    base = scan['scan']
    first = scan['reference_poses'][0]
    scale = scan['metres_per_baseline']
    R0, c0 = np.array(first['world_to_camera_R']), np.array(first['camera_center'])
    R1 = np.array(base['second_world_to_camera_R'])@R0
    t1 = scale*np.array(base['second_world_to_camera_t'])-R1@c0
    return base | {'points': (scale*np.array(base['points'])@R0+c0).tolist(),
        'first_world_to_camera_R': R0.tolist(),
        'first_world_to_camera_t': first['world_to_camera_t'],
        'second_world_to_camera_R': R1.tolist(), 'second_world_to_camera_t': t1.tolist(),
        'second_camera_center': (-R1.T@t1).tolist(),
        'coordinate_frame': scan['coordinate_frame'], 'distance_unit': 'm',
        'scale_source': scan['scale_source'], 'physical_accuracy_verified': False,
        'reference_id': first['reference']['board']['reference_id']}


def localize_metric_scan(scan, image):
    """PnP against saved scene features, also when the board is no longer visible."""
    _, np = _vision()
    _validate_metric_scan(scan)
    result = localize_scan(scan['scan'], image)
    first = scan['reference_poses'][0]
    R0, c0 = np.array(first['world_to_camera_R']), np.array(first['camera_center'])
    scale = scan['metres_per_baseline']
    R = np.array(result['world_to_camera_R'])@R0
    t = scale*np.array(result['world_to_camera_t'])-R@c0
    return result | {'coordinate_frame': scan['coordinate_frame'], 'distance_unit': 'm',
        'world_to_camera_R': R.tolist(), 'world_to_camera_t': t.tolist(),
        'camera_center': (-R.T@t).tolist(), 'scale_source': scan['scale_source'],
        'physical_accuracy_verified': False, 'reference_must_remain_fixed': True,
        'free_space_observed': False}
