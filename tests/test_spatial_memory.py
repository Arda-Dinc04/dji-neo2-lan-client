"""Known volumes and swept-body clearance; no flight or hardware readiness."""
import copy
import json
import subprocess
import sys

import pytest

np = pytest.importorskip('numpy')

from neo2_control.spatial_memory import (
    FREE, OCCUPIED, UNKNOWN, _heading_candidates, _integrate_views, _route_steps,
    SpatialMemoryIndex, _ClearanceGrid, build_memory, choose_heading, load_memory, save_memory,
)


def wall_view(depth=3.):
    return {'depth': np.full((65, 65), depth),
        'K': np.array([[24., 0, 32.], [0, 24., 32.], [0, 0, 1.]]),
        'R': np.eye(3), 't': np.zeros(3), 'minimum_depth_m': .25}


LOWER, UPPER = [-.8, -.8, .3], [.8, .8, 3.3]


def integrate(views):
    return _integrate_views(views, LOWER, UPPER, .1, .05)[0]


def at(states, point):
    return states[tuple(np.floor((np.array(point)-LOWER)/.1).astype(int))]


def assess(states, *, position=(0., 0., 1.), forward=(0., 0., 1.), up=(0., -1., 0.)):
    return _heading_candidates(states, LOWER, UPPER, .1, position, forward, up,
        distance_m=1., radius_m=.15, tracking_margin_m=.05, angle_step_degrees=30)


def test_full_projected_voxel_is_free_before_a_wall_and_unknown_behind_it():
    states = integrate([wall_view()])
    assert at(states, [0.05, .05, 1.05]) == FREE
    assert at(states, [0.05, .05, 2.95]) == OCCUPIED
    assert at(states, [0.05, .05, 3.25]) == UNKNOWN
    candidate = assess(states)[0]
    assert candidate['yaw_degrees'] == 0 and candidate['model_candidate']
    assert candidate['unknown_cell_count'] == candidate['surface_cell_count'] == 0


def test_clear_model_produces_the_requested_measured_out_and_back_sequence():
    from neo2_control.motion_program import MotionProgram
    candidate = assess(integrate([wall_view()]))[0]
    steps = _route_steps(candidate, 1., [0., 0., 1.])
    assert [step['kind'] for step in steps] == ['takeoff', 'rotate', 'translate_measured',
        'rotate', 'translate_measured', 'rotate', 'land']
    assert steps[2]['distance_m'] == steps[4]['distance_m'] == 1.
    assert steps[2]['target_position_m'] == pytest.approx([0., 0., 2.])
    assert steps[4]['target_position_m'] == [0., 0., 1.]
    assert steps[3]['degrees'] == steps[5]['degrees'] == 180
    assert all('seconds' not in step for step in steps)
    with pytest.raises(ValueError):
        MotionProgram.from_dict({'format': 'neo2-control/clearance-route/v1', 'steps': steps})


def test_one_missing_depth_sample_does_not_carve_the_surrounding_volume():
    view = wall_view()
    view['depth'][32, 32] = np.nan
    states = integrate([view])
    assert at(states, [.05, .05, 1.05]) == UNKNOWN
    assert not assess(states)[0]['model_candidate']
    assert assess(states)[0]['unknown_cell_count'] > 0


def test_full_turn_at_one_camera_origin_does_not_observe_its_own_start_volume():
    position=np.array([0.,0.,1.]);views=[]
    for angle in range(0,360,45):
        theta=np.radians(angle)
        R=np.array([[np.cos(theta),0.,-np.sin(theta)],[0.,1.,0.],
            [np.sin(theta),0.,np.cos(theta)]])
        view=wall_view();view.update(R=R,t=-R@position);views.append(view)
    states=integrate(views)
    assert at(states,position)==UNKNOWN
    assert not any(c['model_candidate'] for c in assess(states))
    assert all(c['unknown_cell_count']>0 for c in assess(states))


def test_all_unknown_image_leaves_all_voxels_unknown():
    view = wall_view()
    view['depth'][:] = np.nan
    states = integrate([view])
    assert not states.any()
    assert all(not item['model_candidate'] for item in assess(states))


def test_fusion_fills_an_observation_hole_but_never_erases_a_surface():
    partial = wall_view()
    partial['depth'][32, 32] = np.nan
    assert at(integrate([partial, wall_view()]), [.05, .05, 1.05]) == FREE
    near = wall_view(1.7)
    states, conflicts = _integrate_views([wall_view(), near], LOWER, UPPER, .1, .05)
    assert conflicts > 0
    assert at(states, [.05, .05, 1.65]) == OCCUPIED
    assert not assess(states)[0]['model_candidate']
    opposite = integrate([near, wall_view()])
    assert np.array_equal(states, opposite)


def test_side_obstacle_blocks_body_even_when_centerline_remains_clear():
    states = integrate([wall_view()])
    # A pole beside the centre ray intersects the enclosing aircraft sphere.
    states[tuple(np.floor((np.array([.25, .05, 1.65])-LOWER)/.1).astype(int))] = OCCUPIED
    assert at(states, [.05, .05, 1.65]) == FREE
    candidate = assess(states)[0]
    assert not candidate['model_candidate'] and candidate['surface_cell_count'] > 0


def test_planner_turns_toward_the_clear_swept_volume_and_keeps_yaw_sign():
    lower, upper, resolution = [-1., -1., -1.], [2., 1., 2.], .1
    states = np.full((30, 20, 30), FREE, dtype=np.uint8)
    # A broad wall blocks +Z; +X stays clear for a metre.
    states[:, :, 15:18] = OCCUPIED
    reports = _heading_candidates(states, lower, upper, resolution,
        [0., 0., 0.], [0., 0., 1.], [0., -1., 0.], distance_m=1.,
        radius_m=.15, tracking_margin_m=.05, angle_step_degrees=90)
    chosen = next(item for item in reports if item['model_candidate'])
    assert chosen['yaw_degrees'] == 90
    assert chosen['end_position_m'] == pytest.approx([1., 0., 0.])


def test_planning_cannot_leave_the_memory_or_infer_free_from_missing_cells():
    states = np.full((16, 16, 30), FREE, dtype=np.uint8)
    assert all(not item['model_candidate'] for item in assess(states, position=(.75, 0., 1.)))
    with pytest.raises(ValueError, match='up_vector'):
        assess(states, up=(0., 0., 0.))
    with pytest.raises(ValueError, match='horizontal_forward'):
        assess(states, forward=(0., -1., 0.))


@pytest.fixture(scope='module')
def observed_depth(tmp_path_factory):
    # The full renderer/SIFT/stereo fixture supplies actual retained pixels,
    # rather than manually injecting ideal free voxels into persisted memory.
    from test_depth_scan import depth_scene
    return depth_scene.__wrapped__(tmp_path_factory)[0]


def test_client_memory_round_trip_rebuilds_sources_and_remembers_unknown_space(tmp_path, observed_depth):
    memory = build_memory([observed_depth], lower=[-.5, -.5, -.5], upper=[1., 1., 1.5])
    assert memory['counts']['unknown'] > 0
    assert memory['counts']['surface_or_margin'] > 0
    assert memory['free_space_verified'] is False
    path = tmp_path/'memory.json'
    save_memory(path, memory)
    restored = load_memory(path)
    assert restored == memory
    assert path.stat().st_mode & 0o777 == 0o600
    with pytest.raises(FileExistsError):
        save_memory(path, memory)
    result = choose_heading(restored, position=[0., 0., -.3], forward=[0., 0., 1.],
        up=[0., -1., 0.], radius_m=.15)
    assert result['flight_authorized'] is False
    assert result['physical_clearance_verified'] is False
    assert result['current_pose_verified'] is False
    assert result['candidate'] is None and result['steps'] == []


def test_cli_can_reload_memory_report_and_assess_in_a_fresh_process(tmp_path, observed_depth):
    memory = build_memory([observed_depth], lower=[-.5, -.5, -.5], upper=[1., 1., 1.5])
    path, output, report = tmp_path/'memory.json', tmp_path/'route.json', tmp_path/'view.html'
    save_memory(path, memory)
    done = subprocess.run([sys.executable, '-m', 'neo2_control.cli', 'memory-plan', str(path),
        '--position', '0', '0', '-0.3', '--forward', '0', '0', '1', '--up', '0', '-1', '0',
        '--radius', '0.15', '--output', str(output)], capture_output=True, text=True, check=True)
    assert json.loads(done.stdout)['flight_authorized'] is False
    assert json.loads(output.read_text())['candidate'] is None
    subprocess.run([sys.executable, '-m', 'neo2_control.cli', 'scan-report', str(path),
        '--output', str(report)], capture_output=True, text=True, check=True)
    assert 'grid_states' in report.read_text()


@pytest.mark.parametrize('field,value', [
    ('distance_unit', 'cm'), ('free_space_verified', True), ('current_scene_verified', True),
    ('physical_accuracy_verified', True), ('states', []), ('coordinate_frame', 'a'*64),
])
def test_persisted_memory_cannot_change_observations_frame_or_evidence(tmp_path, observed_depth, field, value):
    memory = build_memory([observed_depth], lower=[-.2, -.2, -.2], upper=[.6, .6, .6])
    memory[field] = value
    path = tmp_path/'modified.json'
    path.write_text(json.dumps(memory))
    with pytest.raises(ValueError, match='spatial_memory_invalid'):
        load_memory(path)


def test_duplicate_views_are_not_independent_observations(observed_depth):
    with pytest.raises(ValueError, match='duplicate_source'):
        build_memory([observed_depth, observed_depth], lower=[0., 0., 0.], upper=[1., 1., 1.])


def test_declared_resolution_and_bounds_are_bounded_and_exact():
    with pytest.raises(ValueError, match='bounds'):
        _integrate_views([wall_view()], [0., 0., 0.], [20., 20., 20.], .025, .01)
    with pytest.raises(ValueError, match='bounds'):
        _integrate_views([wall_view()], [0., 0., 0.], [1.01, 1., 1.], .1, .05)


def test_cached_index_copies_validated_observations_and_preserves_plan(observed_depth):
    memory = build_memory([observed_depth], lower=[-.5, -.5, -.5], upper=[1., 1., 1.5])
    request = dict(position=[0., 0., -.3], forward=[0., 0., 1.], up=[0., -1., 0.], radius_m=.15)
    index = SpatialMemoryIndex(memory)
    expected = choose_heading(memory, **request)
    assert index.plan(**request) == expected
    memory['states'][:] = [FREE]*len(memory['states'])
    memory['lower'][0] = -30.
    assert index.plan(**request) == expected
    with pytest.raises(ValueError):
        index._states[0, 0, 0] = FREE


def test_cached_segments_match_headings_and_turns_require_a_free_sphere():
    states = integrate([wall_view()])
    grid = _ClearanceGrid(states, LOWER, UPPER, .1, 'a'*64, 'b'*64)
    start = [0., 0., 1.]
    for candidate in assess(states):
        report = grid.assess_segment(start, candidate['end_position_m'], radius_m=.15, tracking_margin_m=.05)
        for key in ('outside_memory', 'swept_cell_count', 'unknown_cell_count', 'surface_cell_count', 'model_candidate'):
            assert report[key] == candidate[key]
    assert grid.assess_segment(start, start, radius_m=.15, tracking_margin_m=.05)['model_candidate']
    states[tuple(np.floor((np.array(start)-LOWER)/.1).astype(int))] = UNKNOWN
    changed = _ClearanceGrid(states, LOWER, UPPER, .1, 'a'*64, 'c'*64)
    assert not changed.assess_segment(start, start, radius_m=.15, tracking_margin_m=.05)['model_candidate']


def test_cached_index_still_rejects_forged_memory(observed_depth):
    memory = build_memory([observed_depth], lower=[-.2, -.2, -.2], upper=[.6, .6, .6])
    memory['states'][:] = [FREE]*len(memory['states'])
    with pytest.raises(ValueError, match='spatial_memory_invalid'):
        SpatialMemoryIndex(memory)
