"""Scan/advance/land sequencing in a declared model; no device access."""
from dataclasses import replace
import json
import math
import subprocess
import sys

import pytest

np = pytest.importorskip('numpy')

from neo2_control.metric_model import FRAME, ModelVolume, SCENARIOS, simulate_metric_route
from neo2_control.metric_route import MetricBodyPose, MetricRouteController, _wrap
from neo2_control.spatial_memory import OCCUPIED, UNKNOWN, _ClearanceGrid


MISSION = 'scan-advance-land'


def hovered(memory=None):
    controller = MetricRouteController(memory if memory is not None else ModelVolume(), mission=MISSION)
    now = 100.
    sample = MetricBodyPose((0., 0., 0.), 0., now, False, False, FRAME, 'c'*48, 'd'*24, True)
    assert controller.update(now, sample).action == 'takeoff'
    for height in (.1, .2, .3, .4, .5, .6, .6, .6, .6, .6, .6):
        now += .1
        sample = replace(sample, position_m=(0., -height, 0.), observed_at=now, flying=True, motors_on=True)
        controller.update(now, sample)
    assert controller.stage == 'scanning' and controller.route is None
    return controller, now, sample


def scan(controller, now, sample, final=361.):
    for angle in list(np.arange(15., final, 15.))+[final]*7:
        now += .1
        sample = replace(sample, observed_at=now, heading_deg=_wrap(float(angle)))
        controller.update(now, sample)
    return now, sample


def test_no_heading_selection_before_a_complete_observed_scan(monkeypatch):
    controller, now, sample = hovered()
    calls=[];original=controller.memory.plan
    def plan(**kwargs):calls.append(controller._yaw);return original(**kwargs)
    monkeypatch.setattr(controller.memory, 'plan', plan)
    now, sample = scan(controller, now, sample, 290.)
    assert controller.stage == 'scanning' and controller.route is None and not calls
    # Even 359.9 degrees cannot be promoted by the ordinary 2-degree tolerance.
    for angle in (305.,320.,335.,350.,359.9,359.9,359.9,359.9,359.9,359.9):
        now += .1
        sample = replace(sample, observed_at=now, heading_deg=_wrap(angle))
        controller.update(now, sample)
    assert controller.route is None and not calls
    for _ in range(7):
        now += .1
        sample = replace(sample, observed_at=now, heading_deg=.5)
        controller.update(now, sample)
    assert controller.stage == 'turn_outbound' and len(calls)==1 and calls[0] >= 360
    assert controller.report()['scan_completed_degrees'] >= 360
    assert controller.route['format'] == 'neo2-control/scan-advance-route/v1'
    assert [step['kind'] for step in controller.route['steps']] == [
        'takeoff', 'rotate', 'choose_heading_after_scan', 'rotate', 'translate_measured', 'land']


@pytest.mark.parametrize('gain',[.4,.65,1.,1.4])
def test_delayed_scan_one_metre_then_land_at_destination_without_return(gain):
    result = simulate_metric_route(mission=MISSION, response_gain=gain)
    assert result['scenario_passed'] and result['feedback']['complete']
    assert result['outbound_endpoint_error_m'] < .03 and result['return_endpoint_error_m'] is None
    assert result['feedback']['scan_completed_degrees'] >= 360
    events = result['feedback']['events']
    assert [e['stage'] for e in events] == [
        'taking_off','scanning','turn_outbound','outbound','landing','complete']
    report=result['feedback'];hover=np.array(report['hover_position_m']);end=np.array(report['outbound_target_m'])
    assert np.linalg.norm(end-hover) == pytest.approx(1.)
    landing=np.array(report['landing_target_m']);final=np.array(events[-1]['position_m'])
    assert np.linalg.norm(final-landing) < .05
    assert np.linalg.norm((final-hover)[[0,2]]) > .95
    assert [event['action'] for event in result['actions']] == ['takeoff','land']
    assert not result['model_motors_on'] and not result['flight_authorized']
    for row in result['trajectory']:
        demand=row['demand']
        assert math.hypot(demand['right_mps'],demand['forward_mps']) <= .25+1e-12
        assert abs(demand['yaw_dps']) <= 45 and not demand['flight_authorized']


@pytest.mark.parametrize('scenario',SCENARIOS[1:])
def test_failures_stop_the_mission_without_inventing_recovery(scenario):
    result=simulate_metric_route(mission=MISSION, scenario=scenario)
    assert result['scenario_passed'] and result['feedback']['stage']=='aborted'
    assert result['model_motors_on'] and not result['feedback']['complete']
    demand=result['trajectory'][-1]['demand']
    assert demand['landing_required'] and demand['action'] is None
    assert not any(demand[key] for key in ('right_mps','forward_mps','up_mps','yaw_dps'))


def test_pose_lost_during_scan_is_terminal_without_selecting_a_direction():
    controller, now, sample = hovered()
    result=controller.update(now+.1,None)
    assert result.stage=='aborted' and result.landing_required and controller.route is None
    assert controller.update(now+.2,replace(sample,observed_at=now+.2)).stage=='aborted'


def test_unknown_volume_cannot_start_a_scan_mission():
    memory=ModelVolume()
    unknown=_ClearanceGrid(np.full_like(memory._states,UNKNOWN),memory._lower,memory._upper,
        memory._resolution,FRAME,'a'*64)
    controller=MetricRouteController(unknown,mission=MISSION)
    sample=MetricBodyPose((0.,0.,0.),0.,100.,False,False,FRAME,'c'*48,'d'*24,True)
    demand=controller.update(100.,sample)
    assert demand.reason=='takeoff_volume_not_clear' and demand.action is None


def test_a_clear_horizontal_path_with_blocked_destination_descent_is_not_selected():
    memory=ModelVolume();states=memory._states.copy();centers=memory._centers
    floor=(centers[:,1]>-.15)&(np.linalg.norm(centers[:,[0,2]],axis=1)>.55)
    states.ravel()[floor]=OCCUPIED
    memory=_ClearanceGrid(states,memory._lower,memory._upper,memory._resolution,FRAME,'b'*64)
    # A horizontal route exists, but its destination cannot be used for landing.
    assert memory.plan(position=(0.,-.6,0.),forward=(0.,0.,1.),up=(0.,-1.,0.),
        radius_m=.15,tracking_margin_m=.2)['candidate'] is not None
    controller,now,sample=hovered(memory)
    scan(controller,now,sample)
    assert controller.stage=='aborted' and controller.reason=='no_clear_metre_heading'
    assert controller.route['candidate'] is None
    assert not any(e['stage']=='outbound' for e in controller.events)


def test_new_obstacle_in_destination_descent_stops_before_translation():
    controller,now,sample=hovered();now,sample=scan(controller,now,sample)
    assert controller.stage=='turn_outbound'
    memory=controller.memory;states=memory._states.copy()
    landing=np.array(controller.report()['landing_target_m'])
    blocked=(np.linalg.norm(memory._centers[:,[0,2]]-landing[[0,2]],axis=1)<.15)&(memory._centers[:,1]>-.15)
    states.ravel()[blocked]=OCCUPIED
    changed=_ClearanceGrid(states,memory._lower,memory._upper,memory._resolution,FRAME,'c'*64)
    demand=controller.update(now+.1,replace(sample,observed_at=now+.1),memory=changed)
    assert demand.reason=='landing_volume_not_clear' and demand.stage=='aborted'
    assert not demand.forward_mps and not demand.right_mps and demand.landing_required


def test_new_cli_mode_is_reproducible_and_preserves_the_report(tmp_path):
    output=tmp_path/'scan.json'
    command=[sys.executable,'-m','neo2_control.cli','simulate-metric-route','--mission',MISSION,'--output',str(output)]
    result=subprocess.run(command,capture_output=True,text=True,check=True)
    assert json.loads(result.stdout)['scenario_passed']
    saved=json.loads(output.read_text())
    assert saved['mission']==MISSION and saved['feedback']['scan_completed_degrees']>=360
    assert output.stat().st_mode&0o777==0o600
    original=output.read_bytes()
    assert subprocess.run(command,capture_output=True).returncode!=0
    assert output.read_bytes()==original


def test_invalid_mission_cannot_silently_use_the_other_route():
    with pytest.raises(ValueError,match='mission_invalid'):
        MetricRouteController(ModelVolume(),mission='scan')
