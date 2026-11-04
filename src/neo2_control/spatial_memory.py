"""Persistent, conservative voxel observations and offline one-metre planning.

FREE means a complete voxel projects before measured surfaces under an explicit
depth/pose model. It is not verified physical clearance. Unknown pixels never
carve space, and any observed surface takes precedence over a free observation.
The module has no radio or flight authority.
"""
import itertools
import json
import math
from pathlib import Path
import re

from .depth_scan import _validate_depth_scan
from .metric_reference import _digest
from .reconstruction import _finite, _json_file, _read, _vision

UNKNOWN, FREE, OCCUPIED = 0, 1, 2


def _grid_spec(lower, upper, resolution, margin):
    _, np = _vision()
    lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    if (lower.shape != (3,) or upper.shape != (3,) or not np.isfinite(lower).all()
            or not np.isfinite(upper).all() or (np.abs(lower) > 30).any() or (np.abs(upper) > 30).any()
            or not _finite(resolution) or not .025 <= resolution <= .25
            or not _finite(margin) or not .01 <= margin <= .25 or margin > 2*resolution):
        raise ValueError('spatial_grid_spec_invalid')
    cells = (upper-lower)/resolution
    if ((cells < 1).any() or (cells > 240).any()
            or not np.allclose(cells, np.rint(cells), atol=1e-7)
            or np.prod(np.rint(cells)) > 250000):
        raise ValueError('spatial_grid_bounds_invalid')
    return lower, upper, np.rint(cells).astype(int)


def _depth_view(scan):
    _, np = _vision()
    _validate_depth_scan(scan)
    rectification = np.array(scan['rectification_R'])
    K = np.array(scan['rectified_K'])
    K[:2] /= scan['stride']
    return {'depth': np.asarray(scan['depth_m'], dtype=float), 'K': K,
        'R': rectification@np.array(scan['first_world_to_camera_R']),
        't': rectification@np.array(scan['first_world_to_camera_t']),
        'minimum_depth_m': scan['minimum_depth_m']}


def _integrate_views(views, lower, upper, resolution, margin):
    """Whole-voxel projective visibility; no sparse-ray or hole interpolation."""
    _, np = _vision()
    lower, upper, shape = _grid_spec(lower, upper, resolution, margin)
    indices = np.indices(tuple(shape)).reshape(3, -1).T
    centers = lower+(indices+.5)*resolution
    offsets = np.array(list(itertools.product((-.5, .5), repeat=3)))*resolution
    free = np.zeros(len(centers), dtype=bool)
    occupied = np.zeros(tuple(shape), dtype=bool)
    for view in views:
        depth, K, R, t = (np.asarray(view[key]) for key in ('depth', 'K', 'R', 't'))
        h, w = depth.shape
        camera = np.einsum('nj,ij->ni', centers, R)+t
        corners = camera[:, None, :]+np.einsum('nj,ij->ni', offsets, R)[None, :, :]
        zmin, zmax = corners[:, :, 2].min(axis=1), corners[:, :, 2].max(axis=1)
        with np.errstate(divide='ignore', invalid='ignore'):
            u = K[0, 0]*corners[:, :, 0]/corners[:, :, 2]+K[0, 2]
            v = K[1, 1]*corners[:, :, 1]/corners[:, :, 2]+K[1, 2]
        # The extra surrounding samples conservatively cover the retained
        # depth-grid footprint. A missing sample anywhere in that rectangle
        # prevents this view from declaring the voxel free.
        usable = ((zmin >= view['minimum_depth_m']) & np.isfinite(u).all(axis=1)
            & np.isfinite(v).all(axis=1))
        boxes = np.zeros((len(centers), 4), dtype=int)
        chosen = np.flatnonzero(usable)
        boxes[chosen] = np.column_stack((np.floor(u[chosen].min(axis=1))-1,
            np.floor(v[chosen].min(axis=1))-1, np.ceil(u[chosen].max(axis=1))+1,
            np.ceil(v[chosen].max(axis=1))+1)).astype(int)
        usable &= (boxes[:, 0] >= 0) & (boxes[:, 1] >= 0) & (boxes[:, 2] < w) & (boxes[:, 3] < h)
        # Round the farthest cube corner upward. This groups the integral
        # queries without admitting a surface closer than that actual corner.
        bins = np.ceil(np.maximum(zmax+margin, 0)/resolution).astype(int)
        for level in np.unique(bins[usable]):
            selected = np.flatnonzero(usable & (bins == level))
            invalid = ~np.isfinite(depth) | (depth <= level*resolution)
            summed = np.pad(invalid.astype(np.int32), ((1, 0), (1, 0))).cumsum(0).cumsum(1)
            x0, y0, x1, y1 = boxes[selected].T
            bad = summed[y1+1, x1+1]-summed[y0, x1+1]-summed[y1+1, x0]+summed[y0, x0]
            free[selected[bad == 0]] = True
        # Each surface sample contributes an axis-aligned uncertainty band.
        # This declaration is an input assumption, not a measured error bound.
        yy, xx = np.nonzero(np.isfinite(depth))
        z = depth[yy, xx]
        rays = np.column_stack(((xx-K[0, 2])/K[0, 0]*z, (yy-K[1, 2])/K[1, 1]*z, z))
        world = np.einsum('nj,ji->ni', rays-t, R)
        base = np.floor((world-lower)/resolution).astype(int)
        first = np.floor((world-margin-lower)/resolution).astype(int)
        last = np.floor((world+margin-lower)/resolution).astype(int)
        extent = math.ceil(margin/resolution)+1
        for delta in itertools.product(range(-extent, extent+1), repeat=3):
            cells = base+delta
            keep = ((cells >= first) & (cells <= last) & (cells >= 0) & (cells < shape)).all(axis=1)
            if keep.any():
                occupied[tuple(cells[keep].T)] = True
    conflict_count = int((free & occupied.ravel()).sum())
    states = np.where(occupied.ravel(), OCCUPIED, np.where(free, FREE, UNKNOWN)).astype(np.uint8)
    return states.reshape(tuple(shape)), conflict_count


def build_memory(scans, *, lower, upper, resolution=.1, surface_margin=.05):
    _, np = _vision()
    if not isinstance(scans, (tuple, list)) or not 1 <= len(scans) <= 8:
        raise ValueError('spatial_sources_invalid')
    views = [_depth_view(scan) for scan in scans]
    hashes = [_digest(scan) for scan in scans]
    if len(set(hashes)) != len(hashes):
        raise ValueError('spatial_duplicate_source')
    frame = scans[0]['coordinate_frame']
    if any(scan['coordinate_frame'] != frame for scan in scans):
        raise ValueError('spatial_reference_frame_mismatch')
    # Leave room for the state grid in the bounded, self-contained memory file.
    if sum(len(json.dumps(scan, allow_nan=False)) for scan in scans) > 48*1024*1024:
        raise ValueError('spatial_source_memory_limit')
    lo, hi, shape = _grid_spec(lower, upper, resolution, surface_margin)
    states, conflicts = _integrate_views(views, lo, hi, resolution, surface_margin)
    counts = np.bincount(states.ravel(), minlength=3)
    return {'format': 'neo2-control/spatial-memory/v1', 'coordinate_frame': frame,
        'axes': 'opencv_charuco_board_xyz', 'distance_unit': 'm',
        'lower': lo.tolist(), 'upper': hi.tolist(), 'resolution_m': resolution,
        'surface_margin_m': surface_margin, 'shape': shape.tolist(),
        'states': states.ravel().tolist(),
        'counts': dict(zip(('unknown', 'model_free', 'surface_or_margin'), map(int, counts))),
        'conflicting_cell_count': conflicts, 'sources': list(scans), 'source_sha256': hashes,
        'physical_accuracy_verified': False, 'free_space_verified': False,
        'current_scene_verified': False,
        'scope': 'Whole-voxel visibility under declared static depth/pose assumptions. Surface observations dominate. Unknown is never free. Historical PC memory, without flight authority.'}


def _validate_memory(value):
    try:
        rebuilt = build_memory(value['sources'], lower=value['lower'], upper=value['upper'],
            resolution=value['resolution_m'], surface_margin=value['surface_margin_m'])
        if value != rebuilt:
            raise ValueError
    except (KeyError, TypeError, ValueError, OverflowError):
        raise ValueError('spatial_memory_invalid') from None
    return value


def save_memory(path, memory):
    _validate_memory(memory)
    _json_file(path, memory)
    return {'output': str(Path(path).absolute()), 'coordinate_frame': memory['coordinate_frame'],
        'counts': memory['counts'], 'free_space_verified': False}


def load_memory(path):
    return _validate_memory(json.loads(_read(path, 64*1024*1024)))


def _segment_clearance(states, centers, lower, upper, resolution, start, end,
        radius_m, tracking_margin_m):
    _, np = _vision()
    start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    if (any(v.shape != (3,) or not np.isfinite(v).all() for v in (start, end))
            or not _finite(radius_m) or not .05 <= radius_m <= .5
            or not _finite(tracking_margin_m) or not .01 <= tracking_margin_m <= .5
            or np.linalg.norm(end-start) > 6):
        raise ValueError('clearance_segment_invalid')
    radius = radius_m+tracking_margin_m
    outside = bool((np.minimum(start, end)-radius < lower).any()
        or (np.maximum(start, end)+radius > upper).any())
    displacement = end-start
    length = float(np.linalg.norm(displacement))
    direction = displacement/length if length else np.zeros(3)
    projection = np.clip(np.einsum('ij,j->i', centers-start, direction), 0, length)
    separation = np.linalg.norm(centers-start-projection[:, None]*direction, axis=1)
    # The bounding sphere of a voxel over-approximates its intersection with
    # the swept aircraft sphere, including a zero-length hover/turn segment.
    swept = separation <= radius+math.sqrt(3)*resolution/2
    values = states.ravel()[swept]
    unknown, occupied = int((values == UNKNOWN).sum()), int((values == OCCUPIED).sum())
    return {'outside_memory': outside, 'swept_cell_count': len(values),
        'unknown_cell_count': unknown, 'surface_cell_count': occupied,
        'model_candidate': not outside and bool(len(values)) and not unknown and not occupied}


def _heading_candidates(states, lower, upper, resolution, position, forward, up, *,
        distance_m, radius_m, tracking_margin_m, angle_step_degrees, _centers=None):
    _, np = _vision()
    vectors = [np.asarray(v, dtype=float) for v in (position, forward, up)]
    if (any(v.shape != (3,) or not np.isfinite(v).all() for v in vectors)
            or not _finite(distance_m) or not .1 <= distance_m <= 3
            or not _finite(radius_m) or not .05 <= radius_m <= .5
            or not _finite(tracking_margin_m) or not .01 <= tracking_margin_m <= .5
            or type(angle_step_degrees) is not int or not 5 <= angle_step_degrees <= 90
            or 360 % angle_step_degrees):
        raise ValueError('clearance_request_invalid')
    position, forward, up = vectors
    if np.linalg.norm(up) < 1e-6:
        raise ValueError('clearance_up_vector_required')
    up = up/np.linalg.norm(up)
    forward = forward-up*(forward@up)
    if np.linalg.norm(forward) < 1e-6:
        raise ValueError('clearance_horizontal_forward_required')
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    lower, upper = np.asarray(lower), np.asarray(upper)
    centers = (_centers if _centers is not None else
        lower+(np.indices(states.shape).reshape(3, -1).T+.5)*resolution)
    angles = sorted(range(0, 360, angle_step_degrees), key=lambda a: min(a, 360-a))
    reports = []
    for angle in angles:
        signed = angle if angle <= 180 else angle-360
        radians = math.radians(signed)
        direction = forward*math.cos(radians)+right*math.sin(radians)
        end = position+distance_m*direction
        clearance = _segment_clearance(states, centers, lower, upper, resolution,
            position, end, radius_m, tracking_margin_m)
        reports.append({'yaw_degrees': signed, 'end_position_m': end.tolist(),
            'direction': direction.tolist(), **clearance})
    return reports


def _route_steps(selected, distance_m, position):
    # This descriptor is intentionally not a timed stick MotionProgram.
    # A flight executor still needs current metric pose/control observations.
    if not selected:
        return []
    return [{'kind': 'takeoff', 'target_is_planned_hover_pose': True},
            {'kind': 'rotate', 'degrees': selected['yaw_degrees']},
            {'kind': 'translate_measured', 'distance_m': distance_m,
                'target_position_m': selected['end_position_m']},
            {'kind': 'rotate', 'degrees': 180},
            {'kind': 'translate_measured', 'distance_m': distance_m,
                'target_position_m': list(position)},
        {'kind': 'rotate', 'degrees': 180}, {'kind': 'land'}]


def choose_heading(memory, *, position, forward, up, radius_m,
        distance_m=1., tracking_margin_m=.1, angle_step_degrees=15):
    return SpatialMemoryIndex(memory).plan(position=position, forward=forward, up=up,
        radius_m=radius_m, distance_m=distance_m, tracking_margin_m=tracking_margin_m,
        angle_step_degrees=angle_step_degrees)


class _ClearanceGrid:
    """Copied, read-only geometry shared by validated memories and test models."""

    def __init__(self, states, lower, upper, resolution, coordinate_frame, digest):
        _, np = _vision()
        lo, hi, shape = _grid_spec(lower, upper, resolution, .01)
        values = np.asarray(states)
        if (values.shape != tuple(shape) or not np.isin(values, [UNKNOWN, FREE, OCCUPIED]).all()
                or not isinstance(coordinate_frame, str) or not re.fullmatch('[0-9a-f]{64}', coordinate_frame)
                or not isinstance(digest, str) or not re.fullmatch('[0-9a-f]{64}', digest)):
            raise ValueError('clearance_grid_invalid')
        self.coordinate_frame, self.digest = coordinate_frame, digest
        self._states, self._lower, self._upper = values.astype(np.uint8, copy=True), lo.copy(), hi.copy()
        self._resolution = resolution
        self._centers = lo+(np.indices(tuple(shape)).reshape(3, -1).T+.5)*resolution
        for array in (self._states, self._lower, self._upper, self._centers):
            array.flags.writeable = False

    def assess_segment(self, start, end, *, radius_m, tracking_margin_m):
        return {**_segment_clearance(self._states, self._centers, self._lower, self._upper,
            self._resolution, start, end, radius_m, tracking_margin_m),
            'coordinate_frame': self.coordinate_frame, 'memory_sha256': self.digest,
            'flight_authorized': False}

    def plan(self, *, position, forward, up, radius_m,
            distance_m=1., tracking_margin_m=.1, angle_step_degrees=15):
        candidates = _heading_candidates(self._states, self._lower, self._upper, self._resolution,
            position, forward, up, distance_m=distance_m, radius_m=radius_m,
            tracking_margin_m=tracking_margin_m, angle_step_degrees=angle_step_degrees,
            _centers=self._centers)
        selected = next((item for item in candidates if item['model_candidate']), None)
        return {'format': 'neo2-control/clearance-route/v1', 'coordinate_frame': self.coordinate_frame,
            'memory_sha256': self.digest, 'position_m': list(position),
            'forward': list(forward), 'up': list(up), 'distance_m': distance_m,
            'aircraft_enclosing_radius_m': radius_m, 'tracking_margin_m': tracking_margin_m,
            'candidate': selected, 'headings': candidates, 'steps': _route_steps(selected, distance_m, position),
            'physical_clearance_verified': False, 'current_pose_verified': False,
            'flight_authorized': False,
            'scope': 'Offline candidate for the complete out-and-back sequence. A caller-declared sphere and orientation are not calibrated aircraft geometry or live flight authority.'}


class SpatialMemoryIndex(_ClearanceGrid):
    """Validate source lineage once, then query a private immutable grid copy.

    Replacing the index is explicit: a caller cannot silently mutate the source
    dictionary after validation. This remains historical, unverified geometry.
    """

    def __init__(self, memory):
        _, np = _vision()
        _validate_memory(memory)
        super().__init__(np.asarray(memory['states']).reshape(memory['shape']),
            memory['lower'], memory['upper'], memory['resolution_m'],
            memory['coordinate_frame'], _digest(memory))


def memory_geometry(memory):
    """All cells for slicing, with a bounded 3D preview of observed surfaces."""
    _, np = _vision()
    _validate_memory(memory)
    states = np.array(memory['states'], dtype=np.uint8).reshape(memory['shape'])
    cells = np.column_stack(np.nonzero(states == OCCUPIED))
    stride = max(1, math.ceil(len(cells)/30000))
    cells = cells[::stride]
    points = np.array(memory['lower'])+(cells+.5)*memory['resolution_m']
    first = memory['sources'][0]
    R = np.array(first['first_world_to_camera_R'])
    t = np.array(first['first_world_to_camera_t'])
    R1 = np.array(first['relative_R'])@R
    t1 = np.array(first['relative_R'])@t+np.array(first['relative_t_m'])
    return {'points': points.tolist(), 'rgb': [[245, 168, 90]]*len(points),
        'camera_model': first['camera_model'], 'image_sha256': first['image_sha256'],
        'coordinate_frame': memory['coordinate_frame'], 'distance_unit': 'm',
        'kind': 'memory', 'metric_scale_verified': False, 'free_space_observed': False,
        'first_world_to_camera_R': R.tolist(), 'first_world_to_camera_t': t.tolist(),
        'second_world_to_camera_R': R1.tolist(), 'second_world_to_camera_t': t1.tolist(),
        'second_camera_center': (-R1.T@t1).tolist(),
        'grid_states': memory['states'], 'grid_shape': memory['shape'],
        'grid_lower': memory['lower'], 'resolution_m': memory['resolution_m'],
        'state_counts': memory['counts'], 'source_count': len(memory['sources'])}
