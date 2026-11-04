"""Supervisor receipts use actual owner-assigned packets and send times."""
from dataclasses import replace

import pytest

from neo2_control.control_transport import FlightChannelBackend
from neo2_control.controller import ControlSupervisor, ControlLimits, DecisionJournal, FlightObservation, IntentRejected
from neo2_control.duml import DumlPacket, DumlSequence
from neo2_control.flight import FlightAction
from neo2_control.flight_channel import FlightChannel
from neo2_control.session import run_session


class Rig:
    def __init__(self, tmp_path, *, synthetic=True, mode='simulate', sample_synthetic=None):
        self.now=0.
        self.phase='ground'
        self.sample_synthetic=synthetic if sample_synthetic is None else sample_synthetic
        self.channel=FlightChannel(clock=lambda:self.now)
        self.backend=FlightChannelBackend(self.channel,self.sample,synthetic=synthetic)
        self.channel.bind_transport('127.0.0.1',9003)
        self.channel.set_ready(True,self.now)
        self.sequence=DumlSequence(current=0x9000)
        self.journal=DecisionJournal(tmp_path/'controller.jsonl')
        self.controller=ControlSupervisor(self.journal,mode=mode,backend=self.backend,
            clock=lambda:self.now,limits=ControlLimits(lease_seconds=2))

    def sample(self):
        return FlightObservation(self.now,self.phase=='ground',self.phase=='air',
            self.phase!='ground',90,True,1. if self.phase=='air' else 0.,0.,
            'explicit_controller_test_fixture',self.sample_synthetic,control_ready=True)

    def at(self, now):
        self.now=now
        self.channel.set_ready(True,now)

    def authorize(self):
        self.controller.handle('open')
        token=self.controller.handle('prepare',{'operation':'arm'})['confirmation_token']
        self.lease=self.controller.handle('arm',{'confirmation_token':token})['lease']
        token=self.controller.handle('prepare',{'operation':'takeoff','lease':self.lease})['confirmation_token']
        self.controller.handle('takeoff',{'lease':self.lease,'confirmation_token':token})

    def send(self, now):
        self.at(now)
        request=self.channel.next_packet(now,self.sequence.next)
        assert request is not None and self.channel.can_send(request.seq,now)
        self.channel.sent(request.seq,now)
        return request

    def ack(self, request, now, status=0):
        self.at(now)
        packet=DumlPacket(request.receiver,request.sender,request.seq,0xc0,
                          request.cmd_set,request.cmd_id,bytes([status]))
        self.channel.receive(packet,now)
        return packet


@pytest.fixture
def rig(tmp_path):
    value=Rig(tmp_path)
    yield value
    value.journal.close()


def test_controller_waits_for_owner_allocation_and_matches_exact_wire_response(rig):
    rig.authorize()
    controller=rig.controller
    assert controller.status()['pending_command_sequence'] is None
    request=rig.send(.02)
    assert request.seq==0x9001 and request.payload==b'\1'
    rig.at(.03);controller.tick()
    assert controller.status()['pending_command_sequence']==request.seq
    response=rig.ack(request,.07)
    receipt=rig.backend.action_receipt(controller.pending_intent_id)
    assert receipt['request']==request.encode().hex() and receipt['response']==response.encode().hex()
    rig.at(.08);controller.tick()
    assert controller.action_acknowledged and controller.state=='taking_off'
    rig.phase='air';rig.at(.12);controller.tick()
    assert controller.state=='flying'
    responses=[r for r in controller.events if r['event']=='action_response']
    assert responses[-1]['command_sequence']==request.seq and not responses[-1]['execution_confirmed']


def test_ack_deadline_starts_at_socket_send_instead_of_intent_enqueue(rig):
    rig.authorize()
    rig.at(.23);rig.controller.tick()
    assert rig.controller.state=='taking_off'
    request=rig.send(.24)
    rig.ack(request,.7)
    rig.controller.tick()
    assert rig.controller.action_acknowledged and rig.controller.fault_reason is None
    assert not any(r['event']=='action_ack_timeout' for r in rig.controller.events)


@pytest.mark.parametrize('delay',[.5,.51])
def test_late_actual_ack_does_not_satisfy_supervisor_deadline_even_when_channel_accepts_it(rig,delay):
    rig.authorize()
    request=rig.send(.1)
    rig.ack(request,.1+delay)
    rig.controller.tick()
    assert rig.controller.fault_reason=='action_ack_timeout'
    assert rig.controller.state=='landing'
    assert not rig.controller.action_acknowledged
    assert rig.channel.snapshot()['action']['action']=='land'


def test_expired_unsent_takeoff_is_not_replayed_and_failsafe_queues_neutral_then_land(rig):
    rig.authorize()
    stale=rig.channel.next_packet(0,rig.sequence.next)
    rig.at(.25);rig.controller.tick()
    assert rig.controller.fault_reason=='action_not_sent' and rig.controller.state=='landing'
    assert not rig.channel.can_send(stale.seq,.25)
    neutral=rig.send(.26)
    landing=rig.send(.27)
    assert (neutral.cmd_set,neutral.cmd_id)==(1,0xa)
    assert landing.payload==b'\2' and landing.seq!=stale.seq


def test_normal_landing_flushes_nonzero_control_and_sends_one_neutral_before_land(rig):
    rig.authorize();takeoff=rig.send(.01);rig.ack(takeoff,.02)
    rig.phase='air';rig.at(.05);rig.controller.tick()
    rig.controller.handle('control',{'lease':rig.lease,'sequence':1,'channels':[50,0,0,0],'expires_at':.2})
    control=rig.send(.06)
    rig.at(.07);rig.controller.handle('land')
    neutral=rig.send(.08)
    assert neutral.payload!=control.payload
    landing=rig.send(.09)
    assert landing.payload==b'\2'
    rig.ack(landing,.1);rig.controller.tick()
    assert rig.controller.state=='landing' and not rig.controller.landing_confirmed
    rig.phase='ground';rig.at(.15);rig.controller.tick()
    assert rig.controller.state=='ready' and rig.controller.landing_confirmed
    rig.controller.handle('close')
    assert rig.channel.snapshot()['revoked'] and not rig.backend.connected


def test_missing_receipt_faults_instead_of_fabricating_an_ack(rig):
    rig.authorize()
    rig.channel.action=None
    rig.at(.01);rig.controller.tick()
    assert rig.controller.fault_reason=='backend_failed'
    assert not rig.controller.action_acknowledged


def test_real_controller_rejects_synthetic_observations_even_on_a_ready_channel(tmp_path):
    rig=Rig(tmp_path,mode='flight',synthetic=False,sample_synthetic=True)
    try:
        rig.controller.handle('open')
        assert rig.controller.state=='not_ready'
        with pytest.raises(IntentRejected,match='invalid_state'):
            rig.controller.handle('prepare',{'operation':'arm'})
        assert rig.channel.snapshot()['sent_count']==0
    finally:rig.journal.close()


def test_unknown_observation_keeps_native_backend_not_ready(tmp_path):
    rig=Rig(tmp_path,mode='flight',synthetic=False)
    try:
        rig.backend.observation_provider=lambda:None
        rig.controller.handle('open')
        assert rig.controller.state=='not_ready' and rig.channel.snapshot()['action'] is None
    finally:rig.journal.close()


@pytest.mark.parametrize('mode,synthetic',[('simulate',True),('flight',False)])
def test_observation_provenance_change_cannot_confirm_landing(tmp_path,mode,synthetic):
    rig=Rig(tmp_path,mode=mode,synthetic=synthetic)
    try:
        rig.authorize();takeoff=rig.send(.01);rig.ack(takeoff,.02)
        rig.phase='air';rig.at(.05);rig.controller.tick()
        rig.at(.06);rig.controller.handle('land')
        rig.send(.07);landing=rig.send(.08);rig.ack(landing,.09)
        rig.controller.tick()
        assert rig.controller.state=='landing'
        rig.phase='ground';rig.sample_synthetic=not synthetic
        rig.at(.12);rig.controller.tick()
        assert rig.controller.fault_reason=='telemetry_source_mismatch'
        assert not rig.controller.landing_confirmed
        assert rig.controller.state=='landing'
        rig.send(.13);recovery_landing=rig.send(.14)
        rig.ack(recovery_landing,.15);rig.controller.tick()
        assert not rig.controller.landing_confirmed
        assert rig.controller.state=='landing'
        rig.sample_synthetic=synthetic;rig.at(.17);rig.controller.tick()
        assert rig.controller.landing_confirmed and rig.controller.state=='fault'
    finally:rig.journal.close()


def test_synthetic_backend_rejects_real_endpoint_before_files_or_identity_creation(tmp_path):
    channel=FlightChannel()
    FlightChannelBackend(channel,lambda:None,synthetic=True)
    capture=tmp_path/'never-created'/'capture.jsonl';identity=tmp_path/'identity.json'
    with pytest.raises(ValueError,match='loopback'):
        run_session(capture_path=capture,flight_channel=channel,wlm_wifi_liveview=True,
                    wlm_identity_file=identity)
    assert not capture.parent.exists() and not identity.exists()


def test_existing_real_channel_cannot_be_relabelled_as_synthetic():
    channel=FlightChannel();channel.bind_transport('192.168.2.1',9003)
    with pytest.raises(ValueError,match='loopback'):
        FlightChannelBackend(channel,lambda:None,synthetic=True)
    assert not channel.snapshot()['loopback_only']


def test_transport_binding_and_revoked_generation_cannot_be_reused():
    channel=FlightChannel();channel.bind_transport('127.0.0.1',9003)
    with pytest.raises(ValueError,match='generation'):
        channel.bind_transport('127.0.0.1',9004)
    channel=FlightChannel();channel.revoke()
    with pytest.raises(ValueError,match='generation'):
        channel.bind_transport('127.0.0.1',9003)
