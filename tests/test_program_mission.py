"""Mission orchestration with a fake client and clock; no physical aircraft."""
from copy import deepcopy

import pytest

from neo2_control.operator_client import OperatorError
from neo2_control.program_mission import read_program,run_program_mission

PROGRAM={'version':1,'finish':'land','steps':[{'kind':'hold','seconds':.2}]}


class Client:
    def __init__(self):
        self.now=100.;self.calls=[];self.renewals=0;self.reads=0;self.change_epoch=False
        self.server_offset=0.;self.rtt=0.
        self.current={'now':100.,'connected':True,'possible_airborne':False,'state':'ready',
            'vertical_trial':False,'yaw_trial':False,
            'controller_owner':{'epoch':'a'*48},'connection':{'generation':'b'*24,'video':{'live':True}},
            'limits':{'maximum_channel':250,'maximum_flight_seconds':30},
            'telemetry':{'motors_on':False,'flying':False,'grounded':True,'altitude_m':0}}

    def pin_connection(self,value):self.pinned=value['connection']['generation']

    def status(self):
        self.reads+=1
        self.current['now']=self.now+self.server_offset
        self.current['telemetry']['motion_received_at']=self.now+self.server_offset
        self.current['_client_received_at']=self.now
        self.current['_client_round_trip_seconds']=self.rtt
        if self.change_epoch and self.current['possible_airborne']:
            self.current['controller_owner']['epoch']='c'*48
        if self.current['state']=='landing':
            self.current['telemetry'].update(motors_on=False,flying=False,grounded=None)
        return deepcopy(self.current)

    def intent(self,action,parameters=None):
        self.calls.append((action,parameters or {}))
        fields={}
        if action=='prepare':fields={'confirmation_token':'confirmation'}
        elif action=='arm':fields={'lease':'lease'}
        elif action=='takeoff':
            self.current.update(state='flying',possible_airborne=True)
            self.current['telemetry'].update(motors_on=True,flying=True,grounded=False,altitude_m=1)
        elif action=='program_start':
            fields={'program_id':'program'}
            self.current['remote_control']={'program':{'program_id':'program','phase':'running'}}
        elif action=='program_keepalive':
            self.renewals+=1
            if self.renewals>=2:
                self.current['remote_control']['program']['phase']='motion_complete'
                self.current.update(state='landing',land_requested_at=self.now+self.server_offset)
        elif action=='land':self.current.update(state='landing',land_requested_at=self.now+self.server_offset)
        return {'accepted':True,'status':self.status(),**fields}

    def wait(self,seconds):self.now+=seconds


def run(client,**options):
    return run_program_mission(client,PROGRAM,operator_confirmed_grounded=True,
        clock=lambda:client.now,wait=client.wait,**options)


def test_mission_sequences_takeoff_program_land_without_postflight_attestation():
    client=Client();result=run(client)
    assert result['motion_complete'] and result['failure'] is None and result['land_requested']
    assert result['needs_operator_ground_confirmation']
    actions=[a for a,_ in client.calls]
    assert actions.count('takeoff')==1 and actions.count('program_start')==1
    assert actions.count('confirm_ground')==1 and 'confirm_landing' not in actions
    assert actions.count('program_keepalive')==2 and 'close' not in actions
    assert 'land' not in actions  # The owned program's normal LAND was observed.


def test_higher_rotation_and_floor_require_explicit_profile_ceiling_before_takeoff():
    program={'version':1,'finish':'land','steps':[{'kind':'rotate','degrees':360,
        'channel':600,'minimum_channel':600,'timeout_seconds':20}]}
    ordinary=Client()
    with pytest.raises(ValueError,match='motion_channels_invalid'):
        run_program_mission(ordinary,program,operator_confirmed_grounded=True)
    assert ordinary.calls==[]
    yaw=Client();yaw.current.update(yaw_trial=True)
    yaw.current['limits']['maximum_channel']=600
    result=run_program_mission(yaw,program,operator_confirmed_grounded=True,
        clock=lambda:yaw.now,wait=yaw.wait)
    assert result['motion_complete'] and result['land_requested']
    supplied=next(p for a,p in yaw.calls if a=='program_start')['program']
    assert supplied==program


def test_readiness_and_invalid_program_refuse_before_any_client_access():
    client=Client()
    with pytest.raises(ValueError,match='readiness'):
        run_program_mission(client,PROGRAM)
    with pytest.raises(ValueError):
        run_program_mission(client,{},operator_confirmed_grounded=True)
    assert not client.calls and client.reads==0


def test_mission_cannot_adopt_changed_owner_or_send_land_to_replacement():
    client=Client();client.change_epoch=True
    result=run(client)
    assert result['failure']=='program_generation_changed'
    assert [a for a,_ in client.calls].count('takeoff')==1
    assert 'land' not in [a for a,_ in client.calls]


def test_sink_failure_after_takeoff_still_requests_one_land():
    client=Client()
    def failed(event):raise OSError('private journal unavailable')
    result=run(client,event_sink=failed)
    assert result['failure']=='program_event_sink_failed'
    assert [a for a,_ in client.calls].count('land')==1
    assert [a for a,_ in client.calls].count('takeoff')==1


def test_program_finish_racing_keepalive_is_observed_without_post_retry():
    client=Client();original=client.intent
    def raced(action,parameters=None):
        result=original(action,parameters)
        if action=='program_keepalive' and client.renewals==2:
            raise OperatorError('operator_lease_invalid')
        return result
    client.intent=raced
    result=run(client)
    assert result['failure'] is None and result['motion_complete']
    assert client.renewals==2


def test_explicit_final_descent_confirmation_is_returned_without_sending_it():
    client=Client();original=client.status
    def pending():
        if client.current['state']=='landing':
            client.current['landing_confirmation']={'available':True,'needed':True}
        return original()
    client.status=pending
    result=run(client)
    assert result['needs_landing_confirmation']
    assert 'confirm_landing' not in [a for a,_ in client.calls]


@pytest.mark.parametrize('offset',[1_000_000.,-1_000_000.])
def test_remote_mission_uses_pc_timestamps_without_comparing_them_to_client_clock(offset):
    client=Client();client.now=2_000_000.;client.server_offset=offset;client.rtt=.6
    result=run(client)
    assert result['failure'] is None and result['motion_complete'] and result['land_requested']
    assert 'land' not in [a for a,_ in client.calls]


def test_elapsed_client_time_cannot_be_hidden_by_a_large_network_allowance():
    client=Client();client.rtt=.6
    def delayed(seconds):client.now+=.8
    result=run_program_mission(client,PROGRAM,operator_confirmed_grounded=True,
        clock=lambda:client.now,wait=delayed)
    assert result['failure']=='program_observation_stale'
    assert [a for a,_ in client.calls].count('land')==1


def test_video_arriving_before_telemetry_does_not_start_preparations():
    client=Client();value=client.status();value['telemetry']=None
    client.status=lambda:value
    with pytest.raises(ValueError,match='connected_stopped_aircraft'):run(client)
    assert not client.calls


def test_uncertain_program_identifier_never_causes_another_start():
    client=Client();original=client.intent
    def damaged(action,parameters=None):
        result=original(action,parameters)
        if action=='program_start':result['program_id']=None
        return result
    client.intent=damaged
    result=run(client)
    assert result['failure']=='program_response_invalid'
    assert [a for a,_ in client.calls].count('program_start')==1
    assert [a for a,_ in client.calls].count('land')==1


@pytest.mark.parametrize('data',[
    b'{"version":1,"version":2,"steps":[],"finish":"land"}',
    b'{"version":1,"steps":[{"kind":"hold","seconds":NaN}],"finish":"land"}',
    b' '*8193,
])
def test_program_file_rejects_duplicates_nonfinite_and_oversize(tmp_path,data):
    path=tmp_path/'program.json';path.write_bytes(data)
    with pytest.raises(ValueError):read_program(path)
