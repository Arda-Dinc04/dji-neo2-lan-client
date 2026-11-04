"""Metric feedback semantics, explicitly separate from physical flight."""
from dataclasses import replace
import json
import math
import subprocess
import sys

import pytest

np = pytest.importorskip('numpy')

from neo2_control.metric_model import FRAME, ModelVolume, SCENARIOS, simulate_metric_route
from neo2_control.metric_route import MetricBodyPose, MetricRouteController
from neo2_control.spatial_memory import UNKNOWN, _ClearanceGrid


def pose(at=100., *, position=(0., 0., 0.), heading=0., flying=False, motors=False):
    return MetricBodyPose(position, heading, at, flying, motors, FRAME, 'c'*48, 'd'*24, True)


def hovered(*, outbound=False):
    controller = MetricRouteController(ModelVolume())
    now = 100.
    sample = pose(now)
    assert controller.update(now, sample).action == 'takeoff'
    for height in (.1, .2, .3, .4, .5, .6, .6, .6, .6, .6, .6):
        now += .1
        sample = pose(now, position=(0., -height, 0.), flying=True, motors=True)
        controller.update(now, sample)
    assert controller.stage == 'turn_outbound'
    if outbound:
        target = controller.outward_heading
        for heading in np.linspace(0, target, 13)[1:].tolist()+[target]*6:
            now += .1
            sample = replace(sample, observed_at=now, heading_deg=heading)
            controller.update(now, sample)
        assert controller.stage == 'outbound'
    return controller, now, sample


def test_action_proposed_once_and_duplicate_poses_do_not_establish_stable_hover():
    controller = MetricRouteController(ModelVolume())
    first = pose()
    assert controller.update(100., first).action == 'takeoff'
    assert controller.update(100.1, first).action is None
    assert controller.update(100.2, first).action is None
    assert controller.stage == 'taking_off'
    result = controller.update(100.4, first)
    assert result.stage == 'aborted' and result.reason == 'body_pose_stale'
    assert result.action is None and result.landing_required


def test_duplicate_near_target_does_not_count_as_new_settling_evidence():
    controller, now, sample = hovered()
    target = controller.outward_heading
    for heading in np.linspace(0, target, 13)[1:].tolist()+[target]*2:
        now += .1
        sample = replace(sample, heading_deg=heading, observed_at=now)
        controller.update(now, sample)
    assert controller.stage == 'turn_outbound'
    for _ in range(3):
        now += .05
        controller.update(now, sample)
    assert controller.stage == 'turn_outbound'


def test_outbound_velocity_uses_current_body_axes_and_has_no_stick_channels():
    controller, now, sample = hovered(outbound=True)
    demand = controller.update(now+.1, replace(sample, observed_at=now+.1))
    assert demand.forward_mps == pytest.approx(.25)
    assert demand.right_mps == pytest.approx(0., abs=1e-12)
    assert demand.up_mps == pytest.approx(0.)
    assert 'channels' not in demand.as_dict() and not demand.flight_authorized


@pytest.mark.parametrize('field,value', [
    ('owner_epoch', 'e'*48), ('generation', 'f'*24), ('coordinate_frame', 'a'*64), ('synthetic', False),
])
def test_pose_source_changes_abort_without_addressing_the_replacement_source(field, value):
    controller, now, sample = hovered(outbound=True)
    result = controller.update(now+.1, replace(sample, observed_at=now+.1, **{field: value}))
    assert result.reason == 'body_pose_context_changed' and result.stage == 'aborted'
    assert result.landing_required and result.action is None
    assert not any((result.right_mps, result.forward_mps, result.up_mps, result.yaw_dps))
    assert controller.update(now+.2, replace(sample, observed_at=now+.2)).stage == 'aborted'


@pytest.mark.parametrize('kind,reason', [
    ('missing', 'body_pose_missing'), ('camera_dict', 'body_pose_missing'),
    ('stale', 'body_pose_stale'), ('future', 'body_pose_stale'),
    ('regressed', 'body_pose_time_regressed'), ('ambiguous', 'body_pose_time_ambiguous'),
    ('jump', 'body_pose_jump'), ('tick_gap', 'control_tick_gap'),
])
def test_invalid_or_lost_position_cannot_satisfy_a_metre(kind, reason):
    controller, now, sample = hovered(outbound=True)
    new_now = now+.1
    changed = replace(sample, observed_at=new_now)
    if kind == 'missing':
        changed = None
    elif kind == 'camera_dict':
        changed = {'camera_center_m': [0., 0., 0.], 'observed_at': new_now}
    elif kind == 'stale':
        changed = replace(sample, observed_at=now-1)
    elif kind == 'future':
        changed = replace(sample, observed_at=new_now+1)
    elif kind == 'regressed':
        changed = replace(sample, observed_at=now-.05)
    elif kind == 'ambiguous':
        changed = replace(sample, position_m=(.01, -.6, 0.))
    elif kind == 'jump':
        changed = replace(changed, position_m=(1., -.6, 0.))
    elif kind == 'tick_gap':
        new_now = now+.3
        changed = replace(sample, observed_at=new_now)
    result = controller.update(new_now, changed)
    assert result.reason == reason and result.stage == 'aborted'
    assert result.action is None and result.landing_required


def test_unobserved_vertical_volume_prevents_takeoff_proposal():
    template = ModelVolume()
    unknown = _ClearanceGrid(np.zeros_like(template._states), template._lower,
        template._upper, template._resolution, FRAME, 'a'*64)
    controller = MetricRouteController(unknown)
    demand = controller.update(100., pose())
    assert demand.reason == 'takeoff_volume_not_clear'
    assert demand.action is None and not demand.landing_required


def test_new_unknown_voxel_on_remaining_segment_is_terminal():
    controller, now, sample = hovered(outbound=True)
    previous = controller.memory
    states = previous._states.copy()
    halfway = (np.asarray(controller.end)+controller.hover)/2
    cell = np.floor((halfway-previous._lower)/previous._resolution).astype(int)
    states[tuple(cell)] = UNKNOWN
    changed = _ClearanceGrid(states, previous._lower, previous._upper,
        previous._resolution, FRAME, 'b'*64)
    result = controller.update(now+.1, replace(sample, observed_at=now+.1), memory=changed)
    assert result.reason == 'remaining_volume_not_clear' and result.landing_required
    result = controller.update(now+.2, replace(sample, observed_at=now+.2), memory=previous)
    assert result.stage == 'aborted'


def test_replacement_memory_must_share_the_body_pose_frame():
    controller, now, sample = hovered(outbound=True)
    old = controller.memory
    changed = _ClearanceGrid(old._states, old._lower, old._upper, old._resolution, 'a'*64, 'b'*64)
    result = controller.update(now+.1, replace(sample, observed_at=now+.1), memory=changed)
    assert result.reason == 'memory_frame_changed' and result.action is None


def test_grounded_telemetry_cannot_terminate_landing_while_motors_are_running():
    controller, now, sample = hovered(outbound=True)
    # Isolate the landing observation gate; the full stage order is exercised
    # by the kinematic integration cases below.
    controller._transition('landing', now)
    for height in (.5, .4, .3, .2, .1, 0., 0., 0., 0., 0., 0.):
        now += .1
        current = replace(sample, observed_at=now, position_m=(0., -height, 0.),
            flying=height > .02, motors_on=True)
        controller.update(now, current)
    assert controller.stage == 'landing'
    for _ in range(6):
        now += .1
        current = replace(current, observed_at=now, flying=False, motors_on=False)
        result = controller.update(now, current)
    assert controller.stage == 'complete' and not result.landing_required


def test_turn_correction_cannot_leave_the_declared_route_margin():
    controller, now, sample = hovered()
    # Two plausible samples can cumulatively leave the permitted turn volume.
    for x in (.1, .2):
        now += .1
        result = controller.update(now, replace(sample, position_m=(x, -.6, 0.), observed_at=now))
    assert result.reason == 'route_deviation'


@pytest.mark.parametrize('gain', [.4, .65, 1., 1.4])
def test_full_delayed_feedback_route_measures_endpoints_at_different_response_gains(gain):
    result = simulate_metric_route(response_gain=gain)
    assert result['scenario_passed'] and not result['model_motors_on']
    assert result['outbound_endpoint_error_m'] < .03
    assert result['return_endpoint_error_m'] < .03
    assert result['final_heading_error_deg'] < 2
    report = result['feedback']
    assert [e['stage'] for e in report['events']] == [
        'taking_off', 'turn_outbound', 'outbound', 'turn_back', 'inbound', 'turn_finish', 'landing', 'complete']
    assert report['outward_heading_deg'] == 75
    assert abs(report['heading_unwrapped_deg']-435) < 2
    assert [a['action'] for a in result['actions']] == ['takeoff', 'land']
    for row in result['trajectory']:
        demand = row['demand']
        assert math.hypot(demand['right_mps'], demand['forward_mps']) <= .25+1e-12
        assert abs(demand['yaw_dps']) <= 45 and not demand['flight_authorized']
    assert not report['current_body_pose_verified'] and not result['flight_authorized']


@pytest.mark.parametrize('scenario', SCENARIOS[1:])
def test_model_failures_never_report_a_completed_route_or_a_landed_aircraft(scenario):
    result = simulate_metric_route(scenario=scenario)
    assert result['scenario_passed']
    assert result['feedback']['stage'] == 'aborted' and not result['feedback']['complete']
    # Neutral/landing_required is a proposal; no fictitious successful recovery.
    assert result['model_motors_on']
    assert [a['action'] for a in result['actions']] == ['takeoff']
    demand = result['trajectory'][-1]['demand']
    assert demand['landing_required'] and demand['action'] is None
    assert not any(demand[key] for key in ('right_mps', 'forward_mps', 'up_mps', 'yaw_dps'))


def test_cli_model_is_cold_process_reproducible_and_writes_a_new_private_report(tmp_path):
    path = tmp_path/'model.json'
    args = [sys.executable, '-m', 'neo2_control.cli', 'simulate-metric-route', '--output', str(path)]
    done = subprocess.run(args, check=True, capture_output=True, text=True)
    assert json.loads(done.stdout)['scenario_passed']
    saved = json.loads(path.read_text())
    assert saved['synthetic'] and saved['feedback']['complete']
    assert path.stat().st_mode & 0o777 == 0o600
    digest = path.read_bytes()
    assert subprocess.run(args, capture_output=True).returncode != 0
    assert path.read_bytes() == digest


def test_native_model_height_is_relative_to_translated_ground_in_the_camera_map():
    from neo2_control.metric_model import KinematicBody
    from neo2_control.metric_route import MetricDemand
    ground = np.array([1., .3, -2.])
    volume = ModelVolume(origin=ground, coordinate_frame='a'*64)
    controller = MetricRouteController(volume)
    body = KinematicBody(controller, ground_position=ground, coordinate_frame='a'*64)
    assert body.pose(0.).position_m == tuple(ground)
    body.submit(0., MetricDemand('taking_off', action='takeoff'))
    for i in range(1, 101):
        body.advance(i*.05, .05)
    assert body.pose(5.).position_m[1] == pytest.approx(ground[1]-.65, abs=.01)
    body.submit(5., MetricDemand('landing', action='land'))
    for i in range(101, 201):
        body.advance(i*.05, .05)
    assert not body.motors_on and body.pose(10.).position_m == pytest.approx(ground)


def test_planning_reserves_the_allowed_deviation_in_addition_to_runtime_pose_margin():
    controller, _, _ = hovered(outbound=True)
    assert controller.route['candidate']['yaw_degrees'] == 75
    assert controller.route['tracking_margin_m'] == pytest.approx(.2)
    assert controller.report()['runtime_clearance_margin_m'] == .05
    start, end = np.array(controller.hover), np.array(controller.end)
    # Movement inside the admitted tracking envelope still has the runtime
    # sphere/pose margin. Planning only that smaller sphere had selected 60°.
    for shift in (-.14, 0., .14):
        current = start+.5*(end-start)+[0., 0., shift]
        assert controller.memory.assess_segment(current, end, radius_m=.15,
            tracking_margin_m=.05)['model_candidate']


@pytest.mark.parametrize('value', [float('nan'), 180., -181., True])
def test_pose_heading_is_finite_wrapped_and_typed(value):
    with pytest.raises(ValueError):
        replace(pose(), heading_deg=value)
