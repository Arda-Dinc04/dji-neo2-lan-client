"""Connection ownership with explicit fake BLE/NM/socket resources; no RF."""
import asyncio
from dataclasses import replace
from functools import partial
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

from neo2_control.ble_bootstrap import WifiCredentials
from neo2_control.controller import ControlSupervisor,DecisionJournal,FlightObservation,IntentRejected
from neo2_control.device_lease import DeviceLease
from neo2_control.duml import DumlPacket
from neo2_control.flight_connection import FlightConfig,OwnedFlightBackend
from neo2_control.wifi_link import WifiLinkCancelled


CREDS=WifiCredentials(bytes.fromhex('020000000002'),b'flight-test-only',b'flight-secret-fixture')


def wait_for(predicate,timeout=2):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.005)
    assert predicate()


def result(**values):
    return SimpleNamespace(**values,as_dict=lambda:dict(values))


class Harness:
    def __init__(self,tmp_path):
        self.trace=[];self.sessions=[];self.failure=None
        self.ble_cleanup=self.wifi_cleanup=self.socket_cleanup=True
        self.omit_socket_callback=False
        self.pause_ble=self.pause_cleanup=False
        self.release_ble=threading.Event();self.release_cleanup=threading.Event()
        self.config=FlightConfig('wlanusb0',tmp_path/'wlm.json',tmp_path/'ble.json',
            ble_address='02:00:00:00:00:01',duration=20,capture_directory=tmp_path/'captures')
        self.backend=OwnedFlightBackend(self.config,session_runner=self.session,ble_runner=self.ble,
            wifi_factory=self.wifi,route_guard=self.guard,
            lease_factory=partial(DeviceLease,directory=tmp_path/'leases'))

    def guard(self,interface,**kwargs):
        self.trace.append('guard')
        if self.failure=='cancel_guard':
            kwargs['cancel_event'].set()
            raise WifiLinkCancelled('wifi_connection_cancelled')
        if self.failure=='guard':raise RuntimeError('synthetic_guard_failure')

    async def ble(self,**kwargs):
        self.trace.append('ble.enter')
        try:
            while self.pause_ble and not self.release_ble.is_set():
                if kwargs['stop_event'].is_set():raise asyncio.CancelledError
                await asyncio.sleep(.005)
            if self.failure=='ble':raise RuntimeError('synthetic_ble_failure')
            CREDS.save_new(kwargs['credentials_path'])
            await kwargs['while_connected']()
            while self.pause_cleanup and not self.release_cleanup.is_set():await asyncio.sleep(.005)
            return result(state='complete',reason='credentials_saved',connection_cleanup_verified=self.ble_cleanup)
        finally:
            self.trace.append('ble.exit')
            kwargs['event_sink']('ble_cleanup_result',cleanup_verified=self.ble_cleanup)

    def wifi(self,credentials,interface,**kwargs):
        h=self
        assert credentials==CREDS and interface==self.config.interface
        assert kwargs['stop_event'] is self.backend.connection.stop_event
        if h.failure=='wifi_constructor':raise RuntimeError('synthetic_wifi_constructor_failure')
        class Link:
            clean=False
            async def __aenter__(self):
                h.trace.append('wifi.enter')
                if h.failure=='wifi_enter':
                    self.clean=h.wifi_cleanup
                    raise RuntimeError('synthetic_wifi_enter_failure')
                return self
            async def __aexit__(self,*args):
                h.trace.append('wifi.exit');self.clean=h.wifi_cleanup
            def snapshot(self):return {'cleanup_verified':self.clean}
        return Link()

    def session(self,**kwargs):
        self.trace.append('session.enter');self.sessions.append(kwargs)
        channel=kwargs['flight_channel']
        try:
            channel.bind_transport('127.0.0.1',9003)
            channel.set_ready(True,time.monotonic())
            kwargs['snapshot_sink']({'state':'open','last_received_age_seconds':0})
            if self.failure=='session':raise RuntimeError('synthetic_session_failure')
            kwargs['stop_event'].wait(5)
            return result(state='closed',reason='requested',socket_cleanup_verified=self.socket_cleanup)
        finally:
            channel.cancel(time.monotonic(),'session_closed')
            if not self.omit_socket_callback:
                kwargs['socket_cleanup_sink']({'cleanup_verified':self.socket_cleanup})
            self.trace.append('session.exit')

    def open(self):
        count=len(self.sessions)
        self.backend.open(time.monotonic())
        wait_for(lambda:len(self.sessions)>count)


@pytest.fixture
def harness(tmp_path):
    h=Harness(tmp_path)
    yield h
    h.release_ble.set();h.release_cleanup.set()
    h.backend.close(time.monotonic())
    if h.backend.connection.thread is not None:
        h.backend.connection.thread.join(2)
        assert not h.backend.connection.thread.is_alive()


def test_constructor_is_inert_and_config_paths_serialize_without_credentials(harness):
    h=harness
    assert not h.trace and not Path(h.config.capture_directory).exists()
    assert h.backend.connection.status()['phase']=='idle'
    assert h.backend.connection.status()['cleanup_verified']
    assert 'flight-secret-fixture' not in json.dumps(h.config.as_dict())


def test_connection_status_exposes_passive_optics_metadata_without_opening_any_resources(harness):
    backend=OwnedFlightBackend(replace(harness.config,camera_preview=True),clock=lambda:1.)
    camera=backend.connection.camera
    camera.receive(DumlPacket(1,2,59783,0,2,0x8a,
        bytes.fromhex('00190000190e000001000000ea0b0000')),1.)
    status=backend.connection.status()
    assert status['phase']=='idle' and not status['worker_alive']
    assert status['camera']['fov_status']['sdk_fields_raw']['width']==6400
    assert status['camera']['fov_status']['fresh'] is False
    assert len(status['camera']['fov_observations'])==1
    assert camera.snapshot()['sent_count']==backend.channel.snapshot()['sent_count']==0
    assert not harness.trace


@pytest.mark.parametrize('link',[
    {'state':'failed'}, {'service_switch':{'state':'failed'}},
    {'capability_read':{'state':'timed_out'},'capability_query_retry_in_seconds':None},
])
def test_failed_command_setup_is_visible_without_inventing_a_ready_route(harness,link):
    h=harness
    h.backend.connection._publish({'state':'open','wlm_mode_follow':link,
        'video':{'preview_available':True,'frames_encoded':20}})
    status=h.backend.connection.status()
    assert status['control_setup']['state']=='failed'
    assert not status['worker_alive'] and not status['camera']['ready']
    assert not h.backend.connected and not h.trace
    assert h.backend.channel.snapshot()['sent_count']==0


def test_capability_retry_is_preparing_and_only_own_service_commit_marks_setup_ready(harness):
    connection=harness.backend.connection
    connection._publish({'state':'open','wlm_mode_follow':{
        'capability_read':{'state':'timed_out'},'capability_query_retry_in_seconds':1}})
    assert connection.status()['control_setup']['state']=='preparing'
    connection._publish({'state':'open','wlm_mode_follow':{'wifi_service_configured':True}})
    assert connection.status()['control_setup']['state']=='ready'
    assert not harness.trace and not harness.backend.connected


def test_an_owned_drone_rejects_flight_before_ble_or_network_work(harness):
    h=harness
    directory=Path(h.config.capture_directory).parent/'leases'
    with DeviceLease(h.config.interface,h.config.ble_address,directory=directory):
        h.backend.open(time.monotonic())
        wait_for(lambda:not h.backend.connection.thread.is_alive())
        assert not h.trace
        assert h.backend.connection.status()['error']=='drone_or_wifi_interface_already_owned'


def test_cancellation_before_ble_is_a_clean_local_stop(harness):
    h=harness;h.failure='cancel_guard';h.backend.open(time.monotonic())
    wait_for(lambda:not h.backend.connection.thread.is_alive())
    assert h.backend.connection.status()['phase']=='closed'
    assert h.backend.connection.status()['error'] is None
    assert h.backend.connection.status()['cleanup_verified']


def test_open_retains_ble_through_wifi_socket_and_verifies_reverse_cleanup(harness):
    h=harness;h.open()
    assert h.backend.connected and not h.backend.synthetic
    assert not h.backend.connection.status()['cleanup_verified']
    assert h.sessions[0]['heartbeat'] is True and h.sessions[0]['wlm_wifi_liveview'] is True
    assert 'camera_channel' not in h.sessions[0] and 'video_path' not in h.sessions[0]
    h.backend.close(time.monotonic());h.backend.finish_cleanup(timeout=2)
    assert h.backend.connection.status()['cleanup']=={'ble':True,'wifi':True,'socket':True}
    assert h.trace==['guard','ble.enter','wifi.enter','session.enter','session.exit','wifi.exit','ble.exit']
    assert not h.backend.connected
    assert (h.backend.connection.paths['directory']/'connection-result.json').stat().st_mode&0o777==0o600


def test_connecting_cancel_does_not_invent_ground_or_block_the_supervisor(harness,tmp_path):
    h=harness;h.pause_ble=True
    journal=DecisionJournal(tmp_path/'controller.jsonl')
    controller=ControlSupervisor(journal,mode='flight',backend=h.backend)
    try:
        controller.handle('open');wait_for(lambda:'ble.enter' in h.trace)
        start=time.monotonic();controller.handle('close')
        assert time.monotonic()-start<.3 and controller.state=='disconnected'
        assert controller.last_sample is None
        h.backend.finish_cleanup(timeout=2)
        assert h.backend.connection.status()['cleanup_verified']
    finally:journal.close()


def test_unused_connection_can_close_without_inventing_ground(harness,tmp_path):
    h=harness;journal=DecisionJournal(tmp_path/'controller.jsonl')
    controller=ControlSupervisor(journal,mode='flight',backend=h.backend)
    try:
        controller.handle('open');wait_for(lambda:h.backend.connected)
        controller.handle('close')
        assert controller.state=='disconnected' and controller.last_sample is None
    finally:journal.close()


@pytest.mark.parametrize('flying,motors',[(True,False),(False,True),(True,True)])
def test_connection_with_observed_motor_activity_cannot_be_cancelled_as_unused(harness,tmp_path,flying,motors):
    h=harness;journal=DecisionJournal(tmp_path/'controller.jsonl')
    controller=ControlSupervisor(journal,mode='flight',backend=h.backend)
    try:
        controller.handle('open');wait_for(lambda:h.backend.connected)
        h.backend.observation=lambda:FlightObservation(time.monotonic(),None,flying,motors,
            90,False,0.,None,'test_fixture',False)
        with pytest.raises(IntentRejected,match='ground_state_unconfirmed'):controller.handle('close')
        assert controller.state!='disconnected'
    finally:journal.close()


def test_reopen_waits_for_old_cleanup_then_uses_a_new_channel_and_generation(harness):
    h=harness;h.pause_cleanup=True;h.open()
    old=h.backend.channel;generation=h.backend.connection.generation
    h.backend.close(time.monotonic());wait_for(lambda:'wifi.exit' in h.trace)
    with pytest.raises(IntentRejected,match='previous_flight_cleanup_unconfirmed'):h.backend.open(time.monotonic())
    h.release_cleanup.set();h.backend.finish_cleanup(timeout=2)
    h.open()
    assert h.backend.channel is not old and old.snapshot()['revoked']
    assert h.backend.connection.generation!=generation and len(h.sessions)==2


@pytest.mark.parametrize('resource',['ble','wifi','socket','missing-socket-callback'])
def test_unknown_or_failed_cleanup_cannot_enable_another_generation(harness,resource):
    h=harness
    if resource=='missing-socket-callback':h.omit_socket_callback=True
    else:setattr(h,resource+'_cleanup',False)
    h.open();h.backend.close(time.monotonic())
    with pytest.raises(RuntimeError,match='cleanup_unconfirmed'):h.backend.finish_cleanup(timeout=2)
    with pytest.raises(IntentRejected,match='previous_flight_cleanup_unconfirmed'):h.backend.open(time.monotonic())


@pytest.mark.parametrize('failure',['guard','ble','wifi_constructor','wifi_enter','session'])
def test_failures_leave_cleanup_evidence_and_never_send_a_flight_intent(harness,failure):
    h=harness;h.failure=failure;h.backend.open(time.monotonic())
    wait_for(lambda:not h.backend.connection.thread.is_alive())
    assert h.backend.connection.status()['phase']=='failed'
    assert h.backend.connection.status()['cleanup_verified']
    assert h.backend.channel.snapshot()['sent_count']==0
    assert 'flight-secret-fixture' not in json.dumps(h.backend.connection.status())


@pytest.mark.parametrize('message,expected',[
    ('synthetic_session_failure','synthetic_session_failure'),
    ('private password=flight-secret-fixture','detail_suppressed')])
def test_private_session_failure_diagnostic_retains_only_safe_codes(harness,message,expected):
    h=harness;h.failure='session'
    def session(**kwargs):
        try:return h.session(**kwargs)
        except RuntimeError:raise RuntimeError(message) from None
    h.backend.connection.session_runner=session
    h.backend.open(time.monotonic())
    wait_for(lambda:not h.backend.connection.thread.is_alive())
    path=h.backend.connection.paths['directory']/'session-error.json'
    assert json.loads(path.read_text())=={'type':'RuntimeError','code':expected}
    assert 'flight-secret-fixture' not in path.read_text()
    assert h.backend.connection.status()['cleanup_verified']


@pytest.mark.parametrize('changes',[{'interface':'bad/name'},{'duration':True},{'duration':601},
    {'ble_address':'bad'},{'wlm_identity_file':None},{'ble_identity_file':None},{'camera_preview':1}])
def test_invalid_configuration_precedes_threads_files_and_radio(harness,changes):
    with pytest.raises(ValueError):replace(harness.config,**changes)
    assert not harness.trace and harness.backend.connection.thread is None
