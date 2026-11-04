"""Mission workflow against a deterministic aircraft model, without radio."""
import pytest

from neo2_control.yaw_mission import run_yaw_mission
from neo2_control.operator_client import OperatorError


class Aircraft:
    def __init__(self,*,direction=1,issue=None):
        self.now=100.;self.direction=direction;self.issue=issue;self.state='not_ready'
        self.connected=True;self.generation='a'*24;self.calls=[]
        self.started=self.landed=None;self.yaw=170.;self.altitude=0.;self.yaw_channel=0
        self.motors=self.flying=False
        self.expected=None
    def pin_connection(self,status):
        self.expected=(status['controller_owner']['epoch'],status['connection']['generation'])
    def intent(self,action,parameters=None):
        if self.issue=='generation_during_prepare' and action=='prepare':self.generation='b'*24
        if self.expected!=('c'*48,self.generation):raise OperatorError('operator_request_generation_changed')
        parameters=parameters or {};self.calls.append((action,parameters))
        if action=='prepare':return {'confirmation_token':'explicit-'+parameters['operation']}
        if action=='arm':self.state='armed';return {'lease':'private-fixture'}
        if action=='takeoff':self.started=self.now;self.state='taking_off';self.motors=True
        if action=='control':
            assert self.flying and parameters['channels'][:3]==[0,0,0]
            assert 0<=parameters['channels'][3]<=200
            self.yaw_channel=parameters['channels'][3]
            if self.issue=='uncertain_control':raise OperatorError('operator_request_uncertain')
        if action=='land':self.landed=self.now;self.state='landing';self.yaw_channel=0
        assert action not in ('close','confirm_landing')
        return {}
    def status(self):
        elapsed=0 if self.started is None else self.now-self.started
        if self.issue=='fault_without_land' and elapsed>3 and self.landed is None:self.state='fault'
        generation='b'*24 if self.issue=='generation' and elapsed>3 else self.generation
        stamp=self.now-1 if self.issue=='stale' and elapsed>3 else self.now
        yaw=self.yaw
        if self.issue=='jump' and elapsed>3:yaw=(yaw+100+180)%360-180
        return {'state':self.state,'connected':self.connected,'possible_airborne':self.started is not None,
            'vertical_trial':False,'yaw_trial':True,'controller_owner':{'pid':100,'epoch':'c'*48},
            'fault_reason':'link_unavailable' if self.state=='fault' else None,
            'land_requested_at':self.landed,
            'connection':{'generation':generation,'video':{'live':True}},
            'landing_confirmation':{'available':False},
            'telemetry':{'yaw_deg':yaw,'altitude_m':self.altitude,'motors_on':self.motors,
                'flying':self.flying,'motion_received_at':stamp,'control_received_at':self.now,
                'battery_received_at':self.now,'battery_percent':80}}
    def wait(self,elapsed):
        self.now+=elapsed
        if self.started is not None and self.landed is None:
            self.altitude=min(1.,(self.now-self.started)*2)
            if self.now-self.started>.2:self.flying=True;self.state='flying'
            if self.issue!='no_rotation':
                self.yaw=(self.yaw+self.direction*self.yaw_channel*.15*elapsed+180)%360-180
        if self.landed is not None:
            self.altitude=max(0.,1.-(self.now-self.landed)*.5)
            if self.altitude==0:self.flying=self.motors=False


@pytest.mark.parametrize('direction',[1,-1])
def test_takeoff_one_measured_turn_land_then_wait_for_physical_attestation(direction):
    model=Aircraft(direction=direction);events=[]
    result=run_yaw_mission(model,operator_confirmed_grounded=True,clock=lambda:model.now,
        wait=model.wait,event_sink=events.append)
    assert result['rotation']['complete'] and 360<=result['rotation']['progress_degrees']<=361
    assert result['airborne_seen'] and result['failure'] is None
    assert result['ground_attestation_required'] and model.connected
    assert not model.motors and not model.flying
    names=[a for a,_ in model.calls]
    assert names.count('takeoff')==1 and names.count('land')==1
    assert 'close' not in names and 'confirm_landing' not in names
    controls=[p for a,p in model.calls if a=='control']
    assert [p['sequence'] for p in controls]==list(range(len(controls)))
    assert all(p['channels'][:3]==[0,0,0] for p in controls)
    assert not any('private-fixture' in str(event) for event in events)


def test_motion_that_never_turns_times_out_and_lands_without_fabricating_360():
    model=Aircraft(issue='no_rotation')
    result=run_yaw_mission(model,operator_confirmed_grounded=True,clock=lambda:model.now,wait=model.wait)
    assert result['failure']=='rotation_flight_deadline' and not result['rotation']['complete']
    assert result['rotation']['progress_degrees']==0
    assert model.landed-model.started<25.2
    assert [a for a,_ in model.calls].count('land')==1


@pytest.mark.parametrize('issue',['stale','jump','uncertain_control'])
def test_faults_request_one_land_and_never_retry_takeoff_or_control(issue):
    model=Aircraft(issue=issue)
    result=run_yaw_mission(model,operator_confirmed_grounded=True,clock=lambda:model.now,wait=model.wait)
    names=[a for a,_ in model.calls]
    assert result['failure'] and not result['rotation']['complete']
    assert names.count('takeoff')==1 and names.count('land')==1
    if issue=='uncertain_control':assert names.count('control')==1
    assert model.connected and result['ground_attestation_required']


def test_changed_connection_never_inherits_old_mission_or_land_intent():
    model=Aircraft(issue='generation')
    result=run_yaw_mission(model,operator_confirmed_grounded=True,clock=lambda:model.now,wait=model.wait)
    assert result['failure']=='rotation_generation_changed'
    assert [a for a,_ in model.calls].count('land')==0
    assert model.connected and result['ground_attestation_required']


def test_readiness_is_mandatory_and_no_takeoff_is_caused_by_loading_or_reading():
    model=Aircraft()
    with pytest.raises(ValueError,match='readiness'):
        run_yaw_mission(model,clock=lambda:model.now,wait=model.wait)
    assert not model.calls


def test_first_journal_failure_while_reporting_an_interruption_does_not_block_land():
    model=Aircraft(issue='uncertain_control')
    def journal(event):
        if event['event']=='rotation_interrupted':raise OSError('disk full')
    result=run_yaw_mission(model,operator_confirmed_grounded=True,clock=lambda:model.now,
        wait=model.wait,event_sink=journal)
    names=[a for a,_ in model.calls]
    assert names.count('land')==1 and result['failure']=='operator_request_uncertain'


def test_fault_without_any_land_request_uses_one_explicit_recovery_land():
    model=Aircraft(issue='fault_without_land')
    result=run_yaw_mission(model,operator_confirmed_grounded=True,clock=lambda:model.now,wait=model.wait)
    assert result['failure']=='link_unavailable' and model.landed is not None
    assert [a for a,_ in model.calls].count('land')==1
    assert not model.flying and not model.motors


@pytest.mark.parametrize('invalid',['stale','invalid_yaw'])
def test_fault_and_invalid_observation_still_request_land_on_the_pinned_connection(invalid):
    model=Aircraft(issue='fault_without_land');original=model.status
    def status():
        value=original()
        if value['state']=='fault':
            if invalid=='stale':value['telemetry']['motion_received_at']=model.now-1.
            else:value['telemetry']['yaw_deg']=None
        return value
    model.status=status
    result=run_yaw_mission(model,operator_confirmed_grounded=True,clock=lambda:model.now,wait=model.wait)
    assert result['failure'] in ('rotation_observation_stale','rotation_yaw_invalid')
    assert [a for a,_ in model.calls].count('land')==1 and model.landed is not None


def test_existing_owner_land_with_invalid_observation_is_never_retried():
    model=Aircraft(issue='fault_without_land');original=model.status
    def status():
        value=original()
        if value['state']=='fault':
            value['telemetry']['motion_received_at']=model.now-1.
            value['land_requested_at']=model.now-.1
        return value
    model.status=status
    result=run_yaw_mission(model,operator_confirmed_grounded=True,clock=lambda:model.now,wait=model.wait)
    assert result['land_attempted'] and [a for a,_ in model.calls].count('land')==0


def test_generation_changed_during_first_prepare_never_arms_or_takes_off():
    model=Aircraft(issue='generation_during_prepare')
    result=run_yaw_mission(model,operator_confirmed_grounded=True,clock=lambda:model.now,wait=model.wait)
    assert result['failure']=='operator_request_generation_changed'
    assert not result['takeoff_attempted'] and model.started is None and not model.calls


def test_stricter_channel_and_flight_limits_are_preserved_without_claiming_a_full_turn():
    model=Aircraft();original=model.status
    def status():
        return {**original(),'limits':{'maximum_channel':30,'maximum_flight_seconds':10.}}
    model.status=status
    result=run_yaw_mission(model,operator_confirmed_grounded=True,clock=lambda:model.now,wait=model.wait)
    assert result['failure']=='rotation_flight_deadline' and not result['rotation']['complete']
    assert model.landed-model.started<9.2
    assert all(abs(p['channels'][3])<=30 for a,p in model.calls if a=='control')


@pytest.mark.parametrize('legacy_pause',[False,True])
def test_request_latency_does_not_gain_an_additional_100ms_sleep_or_extend_authority(legacy_pause):
    class DelayedAircraft(Aircraft):
        def __init__(self):
            super().__init__()
            self.control_deadline=None;self.expirations=0;self.accepted_controls=[]
        def advance(self,seconds):
            super().wait(seconds)
            if (self.control_deadline is not None and self.landed is None
                    and self.now>=self.control_deadline):
                self.expirations+=1;self.landed=self.now;self.state='landing';self.yaw_channel=0
        def wait(self,seconds):
            self.advance(max(seconds,.1) if legacy_pause else seconds)
        def status(self):
            self.advance(.040)
            return super().status()
        def intent(self,action,parameters=None):
            issued=self.now
            self.advance(.050)
            if action=='control' and self.landed is not None:
                raise OperatorError('operator_lease_invalid')
            result=super().intent(action,parameters)
            if action=='control':
                self.control_deadline=issued+.2
                self.accepted_controls.append(self.now)
            return result
    model=DelayedAircraft()
    result=run_yaw_mission(model,operator_confirmed_grounded=True,clock=lambda:model.now,wait=model.wait)
    if legacy_pause:
        assert model.expirations==1 and not result['rotation']['complete']
    else:
        assert model.expirations==0 and result['rotation']['complete'] and result['failure'] is None
        assert all(b-a>=.05 for a,b in zip(model.accepted_controls,model.accepted_controls[1:]))
        assert [a for a,_ in model.calls].count('takeoff')==1
        assert [a for a,_ in model.calls].count('land')==1
