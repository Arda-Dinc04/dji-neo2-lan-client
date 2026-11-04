"""Schema and clock/angle decisions only; no drone, sockets or flight model."""
from dataclasses import FrozenInstanceError, replace

import pytest

from neo2_control.controller import FlightObservation
from neo2_control.motion_program import MotionProgram, MotionProgramExecution, MotionStep

GENERATION='a'*24


def program(*steps):return MotionProgram.from_dict({'version':1,'finish':'land','steps':list(steps)})


def sample(now,**values):
    return replace(FlightObservation(now,False,True,True,90,True,1,0,'program_fixture',True,
        yaw_deg=0.,motion_received_at=now),**values)


def test_parse_immutable_timed_path_and_serialization_preserve_units():
    value={'version':1,'finish':'land','steps':[
        {'kind':'hold','seconds':.5},
        {'kind':'translate','seconds':.5,'right':100,'forward':-50,'up':0},
        {'kind':'rotate','degrees':90,'channel':-100,'timeout_seconds':5}]}
    parsed=MotionProgram.from_dict(value)
    value['steps'][1]['right']=-200
    assert parsed.steps[1].channels==(100,-50,0,0)
    assert parsed.as_dict()['steps'][1]['right']==100
    assert MotionProgram.from_dict(parsed.as_dict())==parsed
    assert parsed.budget_seconds==pytest.approx(6.3)
    with pytest.raises(FrozenInstanceError):parsed.steps=()


@pytest.mark.parametrize('value',[
    None,[],{}, {'version':True,'finish':'land','steps':[{'kind':'hold','seconds':1}]},
    {'version':1,'finish':'hover','steps':[{'kind':'hold','seconds':1}]},
    {'version':1,'finish':'land','steps':[]},
    {'version':1,'finish':'land','steps':[{'kind':'hold','seconds':1}]*33},
    {'version':1,'finish':'land','steps':[{'kind':'hold','seconds':5}]*6},
])
def test_untrusted_plan_schema_is_bounded(value):
    with pytest.raises(ValueError):MotionProgram.from_dict(value)


@pytest.mark.parametrize('step',[
    {},[],{'kind':'hold','seconds':True},{'kind':'hold','seconds':float('nan')},
    {'kind':'hold','seconds':.01},{'kind':'hold','seconds':6},
    {'kind':'hold','seconds':1,'extra':'ignored'},
    {'kind':'translate','seconds':1,'right':251,'forward':0,'up':0},
    {'kind':'translate','seconds':1,'right':True,'forward':0,'up':0},
    {'kind':'translate','seconds':1,'right':0,'forward':0,'up':0},
    {'kind':'translate','seconds':1,'right':50,'forward':0,'up':0,'metres':5},
    {'kind':'rotate','degrees':361,'channel':100,'timeout_seconds':5},
    {'kind':'rotate','degrees':float('inf'),'channel':100,'timeout_seconds':5},
    {'kind':'rotate','degrees':90,'channel':0,'timeout_seconds':5},
    {'kind':'rotate','degrees':90,'channel':601,'timeout_seconds':5},
    *({'kind':'rotate','degrees':90,'channel':400,'timeout_seconds':5,'minimum_channel':value}
      for value in [None,True,39,401,600]),
    {'kind':'rotate','degrees':90,'channel':100,'timeout_seconds':26},
])
def test_unknown_units_durations_or_out_of_range_channels_are_refused(step):
    with pytest.raises(ValueError):program(step)


def test_local_constructor_cannot_bypass_parsed_plan_invariants():
    with pytest.raises(ValueError):MotionProgram((MotionStep('translate',1.,(0,0,0,100)),))
    with pytest.raises(ValueError):MotionProgram((MotionStep('hold',1.,(100,0,0,0)),))
    with pytest.raises(ValueError):MotionProgram(())


def test_timed_legs_advance_once_and_insert_neutral_at_each_transition():
    plan=program({'kind':'translate','seconds':.2,'right':100,'forward':0,'up':0},
        {'kind':'translate','seconds':.2,'right':0,'forward':-50,'up':0})
    execution=MotionProgramExecution(plan,GENERATION,100.)
    outputs=[]
    for at in (100.,100.1,100.21,100.3,100.42):
        outputs.append(execution.update(at,sample(at),generation=GENERATION))
    assert outputs==[(100,0,0,0),(100,0,0,0),(0,0,0,0),(0,-50,0,0),(0,0,0,0)]
    report=execution.status()
    assert report['phase']=='motion_complete' and report['completed_steps']==2
    assert report['translation_units']=='normalized_stick_and_seconds'
    assert report['landing_confirmed'] is False


@pytest.mark.parametrize('channel,sign',[(100,1),(-100,-1)])
def test_angle_completion_crosses_wrap_and_never_uses_time_as_proof(channel,sign):
    execution=MotionProgramExecution(program({'kind':'rotate','degrees':90,
        'channel':channel,'timeout_seconds':5}),GENERATION,100.)
    for n in range(7):
        at=100.+n*.1
        yaw=(170+sign*15*n+180)%360-180
        output=execution.update(at,sample(at,yaw_deg=yaw),generation=GENERATION)
        if n<6:assert output[:3]==(0,0,0) and output[3]*sign>0
    assert output==(0,0,0,0)
    assert execution.status()['phase']=='motion_complete'
    assert execution.status()['last_rotation']['progress_degrees']==pytest.approx(90)


def test_duplicate_angle_sample_does_not_count_as_progress_and_rotation_times_out():
    execution=MotionProgramExecution(program({'kind':'rotate','degrees':360,
        'channel':200,'timeout_seconds':.5}),GENERATION,100.)
    repeated=sample(100.)
    for n in range(5):
        output=execution.update(100.+n*.1,repeated,generation=GENERATION)
        assert output==(0,0,0,200)
    assert execution.status()['rotation']['progress_degrees']==0
    with pytest.raises(ValueError,match='rotation_timeout'):
        execution.update(100.5,sample(100.5),generation=GENERATION)
    assert execution.status()['phase']=='running'  # The owner must handle failure and LAND.


@pytest.mark.parametrize('channel',[400,-600])
def test_higher_yaw_program_stays_yaw_only_and_needs_observed_completion(channel):
    execution=MotionProgramExecution(program({'kind':'rotate','degrees':360,
        'channel':channel,'timeout_seconds':5}),GENERATION,100.)
    assert execution.update(100.,sample(100.),generation=GENERATION)==(0,0,0,channel)
    assert execution.status()['phase']=='running'
    assert execution.status()['rotation']['progress_degrees']==0


@pytest.mark.parametrize('channel',[600,-600])
def test_rotation_floor_round_trips_and_neutralizes_only_after_measured_target(channel):
    value={'kind':'rotate','degrees':30,'channel':channel,'minimum_channel':600,'timeout_seconds':5}
    plan=program(value)
    assert plan.as_dict()['steps']==[value]
    execution=MotionProgramExecution(plan,GENERATION,100.)
    for index in range(4):
        at=100+index*.1
        output=execution.update(at,sample(at,yaw_deg=index*10),generation=GENERATION)
        assert output==((0,0,0,channel) if index<3 else (0,0,0,0))
    assert execution.status()['phase']=='motion_complete'
    assert execution.status()['last_rotation']['progress_degrees']==30


@pytest.mark.parametrize('problem',['generation','gap','ground','motors','stale','yaw'])
def test_plan_never_catches_up_or_substitutes_invalid_observations(problem):
    execution=MotionProgramExecution(program({'kind':'rotate','degrees':90,
        'channel':100,'timeout_seconds':5}),GENERATION,100.)
    execution.update(100.,sample(100.),generation=GENERATION)
    now=100.1;observation=sample(now);generation=GENERATION
    if problem=='generation':generation='b'*24
    elif problem=='gap':now=100.3;observation=sample(now)
    elif problem=='ground':observation=replace(observation,flying=False)
    elif problem=='motors':observation=replace(observation,motors_on=False)
    elif problem=='stale':observation=sample(99.)
    elif problem=='yaw':observation=replace(observation,yaw_deg=None)
    with pytest.raises(ValueError):execution.update(now,observation,generation=generation)
    assert execution.status()['completed_steps']==0


def test_cancelled_plan_never_resumes_or_emits_nonzero_targets():
    execution=MotionProgramExecution(program({'kind':'translate','seconds':1,
        'right':0,'forward':100,'up':0}),GENERATION,100.)
    assert execution.update(100.,sample(100.),generation=GENERATION)==(0,100,0,0)
    execution.cancel('operator_land')
    assert execution.update(100.1,sample(100.1),generation=GENERATION)==(0,0,0,0)
    assert execution.status()['reason']=='operator_land'
