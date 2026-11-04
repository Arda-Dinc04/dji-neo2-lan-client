"""Production private IPC and HTTP, with only the synthetic aircraft backend."""
import json
import time

import pytest

from neo2_control.control_owner import ProcessControllerRuntime
from neo2_control.remote_target import RemoteTargetIngress, RemoteTargetWindows
from test_control_owner import owner, wait_for, packets
from test_flight_web import server, raw_request, window_ingress


def remote_client(runtime):
    windows=RemoteTargetWindows()
    def command(action,parameters=None):
        context=runtime.command_context()
        window=windows.issue(context['owner_epoch'],context['connection_generation'])
        source=windows.consume(window_ingress(window),owner_epoch=context['owner_epoch'],
            connection_generation=context['connection_generation'])
        return runtime.remote_handle(action,parameters or {},ingress=source)
    return command


def test_real_owner_runs_target_between_wan_updates_and_land_clears_it(tmp_path):
    with owner(tmp_path) as runtime:
        command=remote_client(runtime)
        command('open')
        token=command('prepare',{'operation':'arm'})['confirmation_token']
        lease=command('arm',{'confirmation_token':token})['lease']
        time.sleep(.55)
        assert runtime.status()['state']=='armed'
        token=command('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
        command('takeoff',{'confirmation_token':token,'lease':lease})
        wait_for(lambda:runtime.status()['state']=='flying')
        command('control',{'lease':lease,'sequence':0,'channels':[0,0,0,100]})
        time.sleep(.55)
        status=runtime.status()
        assert status['state']=='flying' and status['remote_control']['channels']==[0,0,0,100]
        command('land')
        assert not runtime.status()['remote_control']['active']
        wait_for(lambda:runtime.status()['landing_confirmed'])
        assert [p.payload[0] for p in packets(tmp_path/'owner.jsonl')
            if (p.cmd_set,p.cmd_id)==(3,0x2a)]==[1,2]


def test_real_owner_source_expiration_not_parent_heartbeats_ends_flight(tmp_path):
    with owner(tmp_path) as runtime:
        command=remote_client(runtime)
        command('open')
        token=command('prepare',{'operation':'arm'})['confirmation_token']
        lease=command('arm',{'confirmation_token':token})['lease']
        token=command('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
        command('takeoff',{'confirmation_token':token,'lease':lease})
        wait_for(lambda:runtime.status()['state']=='flying')
        command('control',{'lease':lease,'sequence':0,'channels':[0,0,0,100]})
        expiry=runtime.status()['remote_control']['expires_at']
        wait_for(lambda:runtime.status()['landing_confirmed'],timeout=4)
        status=runtime.status()
        # A status RPC can notice the core lease boundary just before the
        # owner's next target tick. Both paths must respect the same deadline.
        assert status['fault_reason'] in ('remote_target_expired','operator_lease_expired','control_expired')
        assert status['land_requested_at']>=expiry
        assert not status['remote_control']['active']


def test_real_http_preserves_native_domain_and_piggybacks_remote_window(tmp_path):
    def simulation(journal_path,**_):return ProcessControllerRuntime(journal_path)
    with server(tmp_path,runtime_factory=simulation) as (base,app):
        code,_,body=raw_request(base,'/api/remote-flight/window')
        assert code==200
        remote=json.loads(body)
        assert remote['expires_in_seconds']==2
        payload={'action':'open','parameters':{},'ingress':window_ingress(remote),
            'round_trip_seconds':.55}
        # An envelope from the remote domain cannot loosen a native deadline.
        native_payload={k:v for k,v in payload.items() if k!='round_trip_seconds'}
        code,_,_=raw_request(base,'/api/flight/intent',payload=native_payload)
        assert code==409 and app.runtime.status()['state']=='disconnected'
        code,_,body=raw_request(base,'/api/remote-flight/intent',payload=payload)
        result=json.loads(body)
        assert code==202 and result['state']=='ready'
        assert result['status']['synthetic'] is True
        assert result['next_window']['expires_in_seconds']==pytest.approx(1.5)
        assert result['next_window']['mutation_sequence']>remote['mutation_sequence']
        assert raw_request(base,'/api/remote-flight/intent',payload=payload)[0]==409
        # A native envelope cannot enter the remote desired-state runner.
        native=json.loads(raw_request(base,'/api/flight/window')[2])
        payload['ingress']=window_ingress(native)
        assert raw_request(base,'/api/remote-flight/intent',payload=payload)[0]==409
        assert app.runtime.status()['state']=='ready'


def test_real_http_invalid_hint_has_no_side_effects(tmp_path):
    def simulation(journal_path,**_):return ProcessControllerRuntime(journal_path)
    with server(tmp_path,runtime_factory=simulation) as (base,app):
        remote=json.loads(raw_request(base,'/api/remote-flight/window')[2])
        payload={'action':'open','parameters':{},'ingress':window_ingress(remote),
            'round_trip_seconds':-1}
        assert raw_request(base,'/api/remote-flight/intent',payload=payload)[0]==400
        assert app.runtime.status()['state']=='disconnected'
        # Invalid sizing metadata did not consume a valid intended action.
        payload['round_trip_seconds']=.55
        assert raw_request(base,'/api/remote-flight/intent',payload=payload)[0]==202
