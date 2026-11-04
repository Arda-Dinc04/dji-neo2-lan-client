"""The new SDK crosses real HTTP/IPC, using only a synthetic aircraft."""
import time

import pytest

from neo2_control.buffered_operator_client import BufferedFlightOperatorClient
from neo2_control.control_owner import ProcessControllerRuntime
from neo2_control.operator_client import OperatorError
from test_control_owner import wait_for
from test_flight_web import server


def synthetic_runtime(path,**_):return ProcessControllerRuntime(path)


def test_client_construction_only_observes_and_cached_tickets_are_one_shot(tmp_path):
    with server(tmp_path,runtime_factory=synthetic_runtime) as (base,app):
        client=BufferedFlightOperatorClient(base)
        assert app.runtime.status()['state']=='disconnected'
        requests=[];original=client._json
        def record(path,payload=None):
            requests.append((path,None if payload is None else payload['action']))
            return original(path,payload)
        client._json=record
        client.intent('open')
        client.intent('prepare',{'operation':'arm'})
        assert requests==[(client.PREFIX+'window',None),(client.PREFIX+'intent','open'),
            (client.PREFIX+'intent','prepare')]


def test_client_does_not_retry_uncertain_post_or_reuse_its_ticket(tmp_path,monkeypatch):
    with server(tmp_path,runtime_factory=synthetic_runtime) as (base,app):
        client=BufferedFlightOperatorClient(base)
        client.intent('open')
        original=client._json;posts=[]
        def uncertain(path,payload=None):
            if payload is not None:
                posts.append(payload)
                raise OperatorError('operator_request_uncertain')
            return original(path,payload)
        monkeypatch.setattr(client,'_json',uncertain)
        with pytest.raises(OperatorError,match='uncertain'):client.intent('prepare',{'operation':'arm'})
        assert len(posts)==1 and client._next_window is None


def test_client_expired_cache_obtains_new_ticket_and_keeps_context(tmp_path):
    with server(tmp_path,runtime_factory=synthetic_runtime) as (base,app):
        client=BufferedFlightOperatorClient(base)
        client.intent('open')
        old=client._next_window
        client._next_window=old[0],time.monotonic()-3
        result=client.intent('prepare',{'operation':'arm'})
        assert result['next_window']['mutation_sequence']>old[0]['mutation_sequence']+1
        # A fresh ticket for another owner must not be silently adopted.
        client._expected_context=('b'*48,client._expected_context[1])
        with pytest.raises(OperatorError,match='generation_changed'):
            client.intent('arm',{'confirmation_token':result['confirmation_token']})
        assert app.runtime.status()['state']=='ready'


def test_client_program_protocol_executes_through_actual_owned_process(tmp_path):
    with server(tmp_path,runtime_factory=synthetic_runtime) as (base,app):
        client=BufferedFlightOperatorClient(base)
        client.intent('open')
        token=client.intent('prepare',{'operation':'arm'})['confirmation_token']
        lease=client.intent('arm',{'confirmation_token':token})['lease']
        token=client.intent('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
        client.intent('takeoff',{'confirmation_token':token,'lease':lease})
        def flying_with_authority():
            status=client.status()
            if status['state']=='flying':return True
            # Polling cannot renew a lease. Retain explicit authority during
            # simulated takeoff just as production mission clients do.
            if status['state']=='taking_off':client.intent('keepalive',{'lease':lease})
            return False
        wait_for(flying_with_authority)
        result=client.intent('program_start',{'lease':lease,'program':{
            'version':1,'finish':'land','steps':[
                {'kind':'translate','seconds':.8,'right':0,'forward':50,'up':0}]}})
        end=time.monotonic()+3
        while time.monotonic()<end:
            report=client.status()['remote_control']['program']
            if report['phase']!='running':break
            try:client.intent('program_keepalive',{'lease':lease,'program_id':result['program_id']})
            except OperatorError as error:
                if str(error) not in ('operator_lease_invalid','motion_program_changed'):raise
            time.sleep(.05)
        wait_for(lambda:client.status()['landing_confirmed'])
        report=client.status()['remote_control']['program']
        assert report['program_id']==result['program_id'] and report['phase']=='motion_complete'


@pytest.mark.parametrize('url',['https://127.0.0.1:8794','http://other.invalid',
    'http://127.0.0.1:8794?token=secret','http://name:secret@127.0.0.1:8794'])
def test_pc_sdk_keeps_existing_loopback_and_credential_boundaries(url):
    with pytest.raises(ValueError):BufferedFlightOperatorClient(url)
