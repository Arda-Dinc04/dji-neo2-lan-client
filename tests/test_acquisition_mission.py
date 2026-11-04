"""Kinematics, sampled feedback and failure landing; no hardware."""
from copy import deepcopy
import math
import pytest
from neo2_control.acquisition_mission import TwoLevelScan,run_acquisition_mission


class Client:
    def __init__(self,gain=.5):
        self.now=100.;self.height=0.;self.yaw=0.;self.channels=[0]*4;self.gain=gain
        self.state='ready';self.airborne=False;self.calls=[];self.land_at=None
        self.changed_owner=False;self.lose_video=False;self.stale=False
    def status(self):
        return {'now':self.now,'connected':True,'possible_airborne':self.airborne,
            'state':self.state,'acquisition_trial':True,
            'controller_owner':{'epoch':('c' if self.changed_owner else 'a')*48},
            'connection':{'generation':'b'*24,'video':{'live':not self.lose_video}},
            'limits':{'maximum_altitude_m':1.8,'maximum_flight_seconds':60.,'maximum_channel':600},
            'telemetry':{'flying':self.airborne,'motors_on':self.airborne,
                'altitude_m':round(self.height,1),'yaw_deg':self.yaw,
                'motion_received_at':self.now-(1. if self.stale else .1)}}
    def pin_connection(self,status):pass
    def intent(self,action,parameters=None):
        self.calls.append((action,parameters or {}));extra={}
        if action=='prepare':extra['confirmation_token']='token'
        if action=='arm':extra['lease']='lease'
        if action=='takeoff':self.airborne=True;self.state='flying';self.height=.6
        if action=='control':self.channels=parameters['channels']
        if action=='land':self.state='landing';self.land_at=self.now;self.channels=[0]*4
        return {'accepted':True,'status':self.status(),**extra}
    def wait(self,dt):
        self.now+=dt
        if self.state=='flying':
            self.height+=dt*self.gain*self.channels[2]/100.
            self.yaw=(self.yaw+dt*35*self.channels[3]/600.+180)%360-180
        if self.state=='landing':
            self.height=max(0,self.height-dt*.5)
            if self.height==0:self.airborne=False


@pytest.mark.parametrize('gain',[.25,.5,.8])
def test_two_levels_measured_rotation_and_land_with_quantized_altitude(gain):
    client=Client(gain);events=[]
    r=run_acquisition_mission(client,operator_confirmed_grounded=True,
        event_sink=events.append,clock=lambda:client.now,wait=client.wait)
    assert r['failure'] is None and r['motion_complete'] and r['land_requested']
    assert [x['target_altitude_m'] for x in r['scan']['completed']]==[1.,1.5]
    assert all(360<=x['rotation']['progress_degrees']<365 for x in r['scan']['completed'])
    controls=[p['channels'] for a,p in client.calls if a=='control']
    assert controls and all(c[:2]==[0,0] and abs(c[2])<=200 for c in controls)
    assert [a for a,_ in client.calls].count('takeoff')==1
    assert [a for a,_ in client.calls].count('land')==1
    assert not client.airborne and client.now-100<60
    assert not r['metric_map_qualified'] and not r['operator_ground_attestation_sent_after_flight']


@pytest.mark.parametrize('problem',['stale','video','owner','sink'])
def test_observation_or_journal_failure_requests_land_without_retakeoff(problem):
    client=Client();old=client.wait
    def wait(dt):
        old(dt)
        if client.now>101:
            if problem=='stale':client.stale=True
            if problem=='video':client.lose_video=True
            if problem=='owner':client.changed_owner=True
    def sink(event):
        if problem=='sink' and event['event']=='scan_progress':raise OSError('journal_failed')
    r=run_acquisition_mission(client,operator_confirmed_grounded=True,
        event_sink=sink,clock=lambda:client.now,wait=wait)
    assert r['failure'] and not r['motion_complete']
    actions=[a for a,_ in client.calls]
    assert actions.count('takeoff')==1
    assert actions.count('land')==(0 if problem=='owner' else 1)


def test_no_height_response_times_out_and_lands():
    client=Client(0)
    r=run_acquisition_mission(client,operator_confirmed_grounded=True,
        clock=lambda:client.now,wait=client.wait)
    assert r['failure']=='acquisition_height_timeout' and r['land_requested']
    assert not any(p['channels'][3] for a,p in client.calls if a=='control')


def test_wrong_vertical_sign_lands_before_rotating():
    client=Client(-.5)
    r=run_acquisition_mission(client,operator_confirmed_grounded=True,
        clock=lambda:client.now,wait=client.wait)
    assert r['failure']=='acquisition_climb_wrong_direction' and r['land_requested']


def test_readiness_and_explicit_profile_are_required_before_mutations():
    client=Client()
    with pytest.raises(ValueError,match='readiness'):run_acquisition_mission(client)
    original=client.status
    client.status=lambda:{**original(),'acquisition_trial':False}
    with pytest.raises(ValueError,match='explicit_profile'):
        run_acquisition_mission(client,operator_confirmed_grounded=True)
    assert not client.calls


@pytest.mark.parametrize('clear_floor',[False,True])
def test_final_descent_uses_only_explicit_clear_floor_authorization(clear_floor):
    client=Client();original_status=client.status;original_intent=client.intent;original_wait=client.wait
    confirmed=[False]
    def status():
        s=original_status()
        s['landing_confirmation']={'available':client.state=='landing' and client.height<=.3 and not confirmed[0]}
        return s
    def intent(action,parameters=None):
        if action=='confirm_landing':confirmed[0]=True
        return original_intent(action,parameters)
    def wait(dt):
        if client.state=='landing' and client.height<=.3 and not confirmed[0]:client.now+=dt
        else:original_wait(dt)
    client.status=status;client.intent=intent;client.wait=wait
    r=run_acquisition_mission(client,operator_confirmed_grounded=True,
        operator_confirmed_clear_floor=clear_floor,clock=lambda:client.now,wait=client.wait)
    assert r['landing_confirmation_sent'] is clear_floor
    assert r['needs_landing_confirmation'] is not clear_floor
    assert client.airborne is not clear_floor


def test_complete_mission_through_real_loopback_http_windows(tmp_path):
    import time
    from test_flight_web import server
    from neo2_control.operator_client import FlightOperatorClient
    model=Client()
    with server(tmp_path) as (base,app):
        app.runtime.epoch='a'*48
        app.runtime.status=model.status
        def handle(action,parameters,*,ingress=None):
            assert ingress is not None
            ingress.validate(time.monotonic())
            app.runtime.contexts.append(ingress)
            return model.intent(action,parameters)
        app.runtime.handle=handle
        r=run_acquisition_mission(FlightOperatorClient(base),operator_confirmed_grounded=True,
            clock=lambda:model.now,wait=model.wait)
        assert app.runtime.contexts and not app.runtime.calls
    assert r['failure'] is None and r['motion_complete'] and not model.airborne


@pytest.mark.parametrize('values',[(.3,1.,0.,.3),(.1,1.8,0.,.1),(.1,1.,0.,-.5)])
def test_controller_rejects_tick_gap_height_guard_and_stale_sample(values):
    controller=TwoLevelScan('b'*24,0)
    with pytest.raises(ValueError):controller.update(*values)


def test_buffered_feedback_tolerates_scheduling_jitter_but_not_stale_motion_or_long_gap():
    controller=TwoLevelScan('b'*24,0,buffered_feedback=True)
    assert controller.update(.3,.6,0.,.29)==(0,0,200,0)
    with pytest.raises(ValueError,match='motion_stale'):controller.update(.6,.6,0.,0.)
    with pytest.raises(ValueError,match='feedback_gap'):controller.update(1.,.6,0.,1.)


@pytest.mark.parametrize('buffered',[False,True])
def test_scheduling_jitter_completes_only_with_the_buffered_transport(buffered):
    from neo2_control.buffered_operator_client import BufferedFlightOperatorClient
    class BufferedModel(Client,BufferedFlightOperatorClient):pass
    client=BufferedModel() if buffered else Client()
    original=client.wait;count=[0]
    def wait(dt):
        count[0]+=1
        original(dt+(.2 if count[0]%25==0 else 0.))
    r=run_acquisition_mission(client,operator_confirmed_grounded=True,
        clock=lambda:client.now,wait=wait)
    assert r['motion_complete'] is buffered
    assert r['failure']==(None if buffered else 'acquisition_feedback_gap')
    assert r['land_requested']


def test_remote_clock_domain_is_rejected_before_any_client_access():
    client=Client();client.base_url='https://example.invalid/control'
    with pytest.raises(ValueError,match='pc_local_client'):
        run_acquisition_mission(client,operator_confirmed_grounded=True)
    assert not client.calls
