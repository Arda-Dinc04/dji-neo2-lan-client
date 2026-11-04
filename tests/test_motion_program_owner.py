"""Movement program admission and execution by the existing owned supervisor."""
from dataclasses import replace
import json
import time

import pytest

from neo2_control.controller import IntentRejected
from neo2_control.flight import FlightAction
from test_controller import factory, actions
from test_control_owner import owner, wait_for
from test_remote_owner import remote_client
from test_remote_target import remote_flight


def spec(*steps):return {'version':1,'finish':'land','steps':list(steps)}


def test_timed_translation_path_executes_and_requests_land_once(factory):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    result=command('program_start',{'lease':lease,'program':spec(
        {'kind':'hold','seconds':.2},
        {'kind':'translate','seconds':.3,'right':100,'forward':0,'up':0})})
    assert result['program_id']==runner.status()['program']['program_id']
    run(.3)
    assert c.control[0]==(100,0,0,0)
    run(.8)
    status=runner.status()
    assert status['program']['phase']=='motion_complete'
    assert status['program']['completed_steps']==2 and not status['active']
    assert c.landing_confirmed and not c.flight_started
    assert actions(b)==[FlightAction.TAKEOFF,FlightAction.LAND]


@pytest.mark.parametrize('channel,sign',[(200,1),(-200,-1)])
def test_full_turn_requires_angle_progress_and_keepalives_never_reset_program(factory,monkeypatch,channel,sign):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    yaw=[0.]
    original=b.observation
    poses={}
    def observed_pose():
        observation=original()
        poses.setdefault(observation.received_at,yaw[0])
        return replace(observation,yaw_deg=poses[observation.received_at])
    monkeypatch.setattr(b,'observation',observed_pose)
    result=command('program_start',{'lease':lease,'program':spec(
        {'kind':'rotate','degrees':360,'channel':channel,'timeout_seconds':5})})
    program_id=result['program_id']
    deadline=runner.program.deadline
    for i in range(160):
        yaw[0]=(sign*i*3.+180)%360-180
        if i and i%20==0 and runner.program is not None:
            command('program_keepalive',{'lease':lease,'program_id':program_id})
            assert runner.program.deadline==deadline
        run(.025)
        if runner.program is None:break
    report=runner.status()['program']
    assert report['phase']=='motion_complete'
    assert report['last_rotation']['progress_degrees']>=360
    assert actions(b)==[FlightAction.TAKEOFF,FlightAction.LAND]
    run(.5)
    assert c.landing_confirmed


@pytest.mark.parametrize('reason',['operator_land','source_loss','manual_control','missing_yaw'])
def test_program_does_not_resume_after_authority_loss_or_manual_takeover(factory,reason):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    step=({'kind':'rotate','degrees':90,'channel':100,'timeout_seconds':3} if reason=='missing_yaw'
        else {'kind':'translate','seconds':3,'right':0,'forward':100,'up':0})
    result=command('program_start',{'lease':lease,'program':spec(step)})
    program_id=result['program_id']
    if reason=='operator_land':command('land')
    elif reason=='manual_control':command('control',{'lease':lease,'sequence':0,'channels':[0,0,0,0]})
    if reason=='source_loss':run(1.7)
    else:run(.1)
    assert runner.program is None and runner.status()['program']['phase']=='cancelled'
    if reason=='manual_control':assert c.state=='flying' and c.control[0]==(0,0,0,0)
    else:assert c.state in ('landing','fault') and actions(b)==[1,2]
    with pytest.raises(IntentRejected):
        command('program_keepalive',{'lease':lease,'program_id':program_id})


def test_regular_keepalive_wrong_program_and_duplicate_start_cannot_renew_program(factory):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    value=spec({'kind':'hold','seconds':3})
    command('program_start',{'lease':lease,'program':value})
    source=runner.source
    with pytest.raises(IntentRejected,match='program_active'):command('keepalive',{'lease':lease})
    with pytest.raises(IntentRejected,match='program_changed'):
        command('program_keepalive',{'lease':lease,'program_id':'old'})
    with pytest.raises(IntentRejected,match='program_active'):
        command('program_start',{'lease':lease,'program':value})
    assert runner.source is source
    run(1.7)
    assert actions(b)==[1,2]


def test_program_respects_profile_and_remaining_flight_budget(factory,monkeypatch):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    value=spec({'kind':'translate','seconds':1,'right':100,'forward':0,'up':0})
    original=b.observation
    monkeypatch.setattr(b,'observation',lambda:replace(original(),yaw_deg=0.))
    c.yaw_trial=True
    with pytest.raises(IntentRejected,match='yaw_trial_requires_yaw_only'):
        command('program_start',{'lease':lease,'program':value})
    c.yaw_trial=False
    c.flight_deadline=clock.now+.5
    with pytest.raises(IntentRejected,match='exceeds_flight_deadline'):
        command('program_start',{'lease':lease,'program':value})
    assert runner.program is None


def test_program_cannot_arm_or_takeoff_implicitly(factory):
    c,b,clock,journal=factory()
    from neo2_control.remote_target import RemoteTargetRunner, RemoteTargetIngress
    runner=RemoteTargetRunner(c,'a'*48)
    c.handle('open')
    source=RemoteTargetIngress('a'*48,None,1,clock.now,clock.now+2)
    with pytest.raises(IntentRejected):
        runner.handle('program_start',{'lease':'not-authorized','program':spec(
            {'kind':'hold','seconds':1})},source)
    assert not b.sent and c.state=='ready'


def test_real_owner_process_executes_timed_path_and_lands_without_http_pacing(tmp_path):
    with owner(tmp_path) as runtime:
        command=remote_client(runtime)
        command('open')
        token=command('prepare',{'operation':'arm'})['confirmation_token']
        lease=command('arm',{'confirmation_token':token})['lease']
        token=command('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
        command('takeoff',{'confirmation_token':token,'lease':lease})
        wait_for(lambda:runtime.status()['state']=='flying')
        result=command('program_start',{'lease':lease,'program':spec(
            {'kind':'translate','seconds':.3,'right':50,'forward':0,'up':0},
            {'kind':'hold','seconds':.2})})
        wait_for(lambda:runtime.status()['landing_confirmed'])
        report=runtime.status()['remote_control']['program']
        assert report['program_id']==result['program_id'] and report['phase']=='motion_complete'
        assert report['completed_steps']==2
        assert not runtime.status()['possible_airborne']
