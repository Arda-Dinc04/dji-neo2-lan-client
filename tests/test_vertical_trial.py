"""Explicit experimental vertical profile, without hardware or radio access."""
from dataclasses import replace
import json
import struct

import pytest

from neo2_control.control_owner import ProcessControllerRuntime
from neo2_control.control_transport import FlightChannelBackend
from neo2_control.controller import ControlLimits, ControlSupervisor, DecisionJournal, IntentRejected
from neo2_control.duml import DumlPacket
from neo2_control.flight import FlightAction
from neo2_control.flight_channel import FlightChannel
from neo2_control.flight_connection import FlightConfig
from neo2_control.simulator import SimulationBackend
from test_landing_supervisor import LandingWorld
from test_observation_envelope import carrier


class VerticalWorld(LandingWorld):
    """Received FC/battery/host fixtures with explicitly invalid navigation."""
    def __init__(self, *, vertical_trial=True):
        super().__init__()
        self.now = 100.
        self.gps = False
        self.channel = FlightChannel(clock=lambda:self.now, vertical_trial=vertical_trial)
        self.backend = FlightChannelBackend(self.channel, synthetic=True)
        self.channel.bind_transport('127.0.0.1',9003)


@pytest.fixture
def rig(tmp_path):
    journals = []

    def make(*, vertical_trial=True, limits=None):
        world = VerticalWorld(vertical_trial=vertical_trial)
        world.feed()
        world.attest()
        journal = DecisionJournal(tmp_path/f'vertical-{len(journals)}.jsonl')
        journals.append(journal)
        controller = ControlSupervisor(journal,mode='simulate',backend=world.backend,
            clock=lambda:world.now,limits=limits,vertical_trial=vertical_trial)
        controller.handle('open')
        return world,controller,journal

    yield make
    for journal in journals:
        journal.close()


def queue_takeoff(world,controller):
    arm = controller.handle('prepare',{'operation':'arm'})['confirmation_token']
    lease = controller.handle('arm',{'confirmation_token':arm})['lease']
    token = controller.handle('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
    controller.handle('takeoff',{'confirmation_token':token,'lease':lease})
    return lease,token


def takeoff(world,controller):
    lease,token = queue_takeoff(world,controller)
    request = world.send()
    assert request.payload == b'\1'
    world.now += .01
    world.phase = 12
    world.feed()
    controller.tick()
    assert controller.state == 'flying'
    return lease,token


def test_decoded_navigation_stays_invalid_through_explicit_vertical_lifecycle(rig):
    world,controller,journal = rig()
    initial = controller.status()
    assert initial['vertical_trial'] is True and initial['state'] == 'ready'
    assert initial['telemetry']['position_valid'] is False
    assert initial['telemetry']['distance_m'] is None
    assert world.channel.observer.origin is None
    assert controller.limits.maximum_flight_seconds == 10
    assert controller.limits.maximum_altitude_m == 1.5
    with pytest.raises(IntentRejected):
        controller.handle('arm',{'confirmation_token':'absent'})
    lease,token = takeoff(world,controller)
    with pytest.raises(IntentRejected):
        controller.handle('takeoff',{'confirmation_token':token,'lease':lease})
    for channels in ([0,0,0,0],[50,0,0,0],[0,0,50,0]):
        with pytest.raises(IntentRejected,match='vertical_trial_control_disabled'):
            controller.handle('control',{'lease':lease,'sequence':1,
                'channels':channels,'expires_at':world.now+.2})
    assert controller.control is None and world.channel.control is None
    controller.handle('land')
    neutral = world.send()
    assert (neutral.cmd_set,neutral.cmd_id) == (1,0xa)
    assert world.send().payload == b'\2'
    world.now += .01
    world.phase = 0
    world.feed()
    controller.tick()
    assert controller.flight_started and not controller.landing_confirmed
    token = controller.handle('prepare',{'operation':'ground'})['confirmation_token']
    controller.handle('confirm_ground',{'confirmation_token':token})
    status = controller.status()
    assert status['state'] == 'ready' and status['landing_confirmed']
    assert status['telemetry']['position_valid'] is False
    assert status['telemetry']['distance_m'] is None
    assert world.channel.observer.origin is None
    assert world.channel.next_packet(world.now,world.sequence.next) is None
    events = [json.loads(line) for line in journal.path.read_text().splitlines()]
    assert events[0]['vertical_trial'] is True
    assert [e['request_action'] for e in events if e['event']=='action_response'] == ['takeoff','land']
    assert all(not e['execution_confirmed'] for e in events if e['event']=='action_response')


@pytest.mark.parametrize('change',[
    {'synthetic':False},
    {'received_at':99.49,'motion_received_at':99.49},
    {'control_ready':False},
    {'received_at':99.49,'control_received_at':99.49},
    {'received_at':96.99,'battery_received_at':96.99},
    {'battery_percent':None}, {'battery_percent':19.9},
    {'grounded':None}, {'grounded':False},
    {'grounded':True,'motors_on':True},
    {'grounded':False,'flying':True,'motors_on':False},
    {'altitude_m':None}, {'altitude_m':-0.01},
    {'altitude_m':1.500001}, {'altitude_m':float('nan')},
])
def test_vertical_admission_retains_provenance_freshness_battery_ground_and_altitude(rig,change):
    world,controller,_ = rig()
    world.channel.observer.sample = replace(world.channel.observer.sample,**change)
    with pytest.raises(IntentRejected):
        controller.handle('prepare',{'operation':'arm'})
    status = controller.status()
    assert status['state'] == 'not_ready'
    # A stationary sample without a ground attestation can be fresh telemetry;
    # it still cannot authorize arming.
    if change != {'grounded':False}:
        assert not status['telemetry']['valid']
    assert not controller.flight_started and controller.lease is None
    assert world.channel.action is world.channel.control is None


@pytest.mark.parametrize('limit',[10.,2.])
def test_vertical_flight_deadline_requests_land_without_motion_intent(rig,limit):
    world,controller,_ = rig(limits=ControlLimits(maximum_flight_seconds=limit))
    lease,_ = takeoff(world,controller)
    deadline = controller.flight_deadline
    while world.now < deadline:
        world.now = min(world.now+.2,deadline)
        world.feed()
        if world.now < deadline:
            controller.handle('keepalive',{'lease':lease})
        else:
            controller.tick()
    assert controller.fault_reason == 'maximum_flight_time'
    assert controller.state == 'landing' and controller.flight_started
    assert world.send().cmd_id == 0xa
    assert world.send().payload == b'\2'
    assert world.channel.next_packet(world.now,world.sequence.next) is None


def test_vertical_altitude_threshold_requests_land_and_never_confirms_automatically(rig):
    world,controller,_ = rig()
    takeoff(world,controller)
    world.channel.observer.sample = replace(world.channel.observer.sample,altitude_m=1.500001)
    controller.tick()
    assert controller.fault_reason == 'altitude_m_invalid_or_limit'
    assert controller.state == 'landing' and not controller.landing_confirmed
    assert world.send().cmd_id == 0xa
    assert world.send().payload == b'\2'
    world.now += .01
    world.needed = True
    world.flight_mode = 12
    world.feed()
    controller.tick()
    assert controller.status()['landing_confirmation']['available']
    assert world.channel.next_packet(world.now,world.sequence.next) is None
    assert controller.flight_started and not controller.landing_confirmed


def test_profile_does_not_weaken_custom_limits_or_change_default_navigation_policy(rig):
    custom = ControlLimits(maximum_flight_seconds=3,maximum_altitude_m=.7,
        minimum_battery_percent=40,telemetry_age=.2)
    _,vertical,_ = rig(limits=custom)
    assert vertical.limits == custom
    world,normal,_ = rig(vertical_trial=False)
    status = normal.status()
    assert status['vertical_trial'] is False and status['state'] == 'not_ready'
    assert status['limits']['maximum_flight_seconds'] == 30
    assert status['limits']['maximum_altitude_m'] == 3
    assert not status['telemetry']['position_valid'] and status['telemetry']['distance_m'] is None
    with pytest.raises(IntentRejected):
        normal.handle('prepare',{'operation':'arm'})
    assert world.channel.action is None


@pytest.mark.parametrize('change',[{'battery_percent':39.9},{'altitude_m':.700001}])
def test_owner_send_rechecks_the_supervisors_stricter_limits_after_enqueue(rig,change):
    world,controller,_ = rig(limits=ControlLimits(
        minimum_battery_percent=40,maximum_altitude_m=.7))
    queue_takeoff(world,controller)
    intent = controller.pending_intent_id
    world.channel.observer.sample = replace(world.channel.observer.sample,**change)
    request = world.channel.next_packet(world.now,world.sequence.next)
    assert request is not None and request.payload == b'\1'
    assert not world.channel.can_send(request.seq,world.now)
    assert world.channel.receipt(intent)['state'] == 'cancelled'
    assert world.channel.sent_count == 0


@pytest.mark.parametrize('value',[0,1,None,'true',{},[]])
def test_profile_requires_an_explicit_boolean(tmp_path,value):
    journal = DecisionJournal(tmp_path/'invalid-profile.jsonl')
    try:
        with pytest.raises(ValueError,match='vertical_trial_must_be_boolean'):
            ControlSupervisor(journal,mode='simulate',backend=SimulationBackend(),vertical_trial=value)
        assert not journal.path.read_text()
    finally:
        journal.close()


def test_profile_requires_matching_semantic_channel_but_allows_explicit_simulator(tmp_path):
    journal = DecisionJournal(tmp_path/'profile-mismatch.jsonl')
    try:
        with pytest.raises(ValueError,match='vertical_trial_requires_control_backend'):
            ControlSupervisor(journal,vertical_trial=True)
        for supervisor_profile,channel_profile in ((True,False),(False,True)):
            backend = FlightChannelBackend(FlightChannel(vertical_trial=channel_profile),synthetic=False)
            with pytest.raises(ValueError,match='vertical_trial_channel_profile_mismatch'):
                ControlSupervisor(journal,mode='flight',backend=backend,vertical_trial=supervisor_profile)
        simulated = ControlSupervisor(journal,mode='simulate',backend=SimulationBackend(),vertical_trial=True)
        assert simulated.status()['vertical_trial'] is True
        assert simulated.limits.maximum_flight_seconds == 10
    finally:
        journal.close()


@pytest.mark.parametrize('enabled',[False,True])
def test_independent_owner_receives_profile_and_starts_inert(tmp_path,enabled):
    config = FlightConfig('fixture0',tmp_path/'wlm.json',tmp_path/'ble.json',
        capture_directory=tmp_path/'sessions',vertical_trial=enabled,incident_directory=tmp_path/'incidents')
    runtime = ProcessControllerRuntime(tmp_path/'owner.jsonl',mode='flight',flight_config=config)
    try:
        status = runtime.status()
        assert status['vertical_trial'] is enabled
        assert status['state'] == 'disconnected' and not status['connected']
        assert status['connection']['phase'] == 'idle' and not status['connection']['worker_alive']
        assert status['limits']['maximum_flight_seconds'] == (10 if enabled else 30)
        assert status['limits']['maximum_altitude_m'] == (1.5 if enabled else 3)
        assert status['limits']['acknowledgement_seconds'] == (1. if enabled else .5)
        assert not (tmp_path/'sessions').exists()
    finally:
        runtime.close()
    assert runtime.cleanup_verified and runtime.final_received
    assert not (tmp_path/'sessions').exists()


@pytest.mark.parametrize('vertical,limits,deadline,accepted',[
    (True,None,1.,True),
    (False,None,.5,False),
    (True,ControlLimits(acknowledgement_seconds=.2),.2,False),
    (True,ControlLimits(),.5,False),
],ids=['vertical-default','ordinary-default','vertical-custom-strict','vertical-custom-default'])
def test_measured_728ms_ack_only_extends_the_default_vertical_transaction(rig,vertical,limits,deadline,accepted):
    world,controller,journal = rig(vertical_trial=vertical,limits=limits)
    if not vertical:
        # Ordinary mode receives actual valid navigation fixture fields.
        world.gps = True
        world.feed()
        world.attest()
    assert controller.limits.acknowledgement_seconds == deadline
    assert controller.limits.telemetry_age == controller.limits.lease_seconds == .5
    lease,_ = queue_takeoff(world,controller)
    takeoff = world.send(acknowledge=False)
    sent_at = world.now
    world.phase = 8  # Motors active; no airborne motion confirmation yet.
    for elapsed in (.1,.2,.3,.4,.5,.6,.7):
        world.now = sent_at+elapsed
        world.feed()
        controller.tick()
        if controller.state == 'taking_off':
            controller.handle('keepalive',{'lease':lease})
    world.now = sent_at+.728166326
    world.feed()
    world.channel.receive(DumlPacket(3,2,takeoff.seq,0x80,3,0x2a,b'\0'),world.now)
    controller.tick()
    if accepted:
        assert controller.state == 'taking_off' and controller.fault_reason is None
        assert controller.action_acknowledged and controller.pending is None
        assert controller.flight_started and not controller.landing_confirmed
        assert world.channel.action is world.channel.control is None
        assert world.channel.sent_count == 1
        assert world.channel.next_packet(world.now,world.sequence.next) is None
        world.now += .01
        world.phase = 12
        world.feed()
        controller.tick()
        assert controller.state == 'flying'
    else:
        assert controller.fault_reason == 'action_ack_timeout'
        assert controller.state in ('landing','fault') and controller.flight_started
        assert controller.pending_action is FlightAction.LAND
        assert not controller.action_acknowledged and not controller.landing_confirmed
        land_receipt = world.channel.receipt(controller.pending_intent_id)
        assert land_receipt['action'] == 'land'
        # With the custom200ms timeout, the deliberately unsent LAND has
        # already expired by728ms. That failure must also remain unresolved.
        assert land_receipt['state'] == ('expired' if controller.state == 'fault' else 'queued')
        assert world.channel.sent_count == 1
    events = [json.loads(line) for line in journal.path.read_text().splitlines()]
    replies = [e for e in events if e['event']=='action_response']
    assert len(replies) == int(accepted)
    if accepted:
        assert replies[0]['request_action'] == 'takeoff'
        assert replies[0]['accepted'] and not replies[0]['execution_confirmed']
    intents = [e['action'] for e in events if e['event']=='action_intent']
    assert intents == (['takeoff'] if accepted else ['takeoff','land'])


def test_both_measured_late_acks_cannot_revive_authority_or_prove_physical_ground(rig):
    world,controller,journal = rig()
    assert controller.limits.acknowledgement_seconds == 1
    lease,old_token = queue_takeoff(world,controller)
    takeoff_id = controller.pending_intent_id
    takeoff = world.send(acknowledge=False)
    started = world.now

    def observe(elapsed, *, motors=False):
        world.now = started+elapsed
        world.phase = 8 if motors else 0
        world.feed()
        # Model the observed brief motor activity with zero relative height,
        # never an airborne flag or valid navigation. Decode the FC bytes.
        body = bytearray(85)
        struct.pack_into('<ddh',body,0,.2,.1,0)
        struct.pack_into('<I',body,32,8 if motors else 0)
        packet = DumlPacket(3,2,0x9a00,0,3,0x43,bytes(body))
        world.fc.feed(carrier(packet.encode(),source=world.route.peer_hash,
            destination=world.route.own_hash).encode(),world.now)
        world.project()
        controller.tick()
        sample = world.backend.observation()
        assert sample.flying is False and sample.motors_on is motors
        assert sample.altitude_m == 0 and not sample.position_valid
        assert sample.distance_m is None
        if controller.state == 'taking_off':
            controller.handle('keepalive',{'lease':lease})

    for elapsed in (.1,.2,.3,.4,.5,.6,.7,.8,.9,1.):
        observe(elapsed,motors=.2 <= elapsed <= .6)
    assert controller.fault_reason == 'action_ack_timeout'
    assert controller.state == 'landing' and controller.flight_started
    assert controller.pending_action is FlightAction.LAND
    land_id = controller.pending_intent_id
    assert land_id != takeoff_id
    assert world.channel.receipt(takeoff_id)['state'] == 'timed_out'
    # Preserve the measured spacing between the two actual wire writes.
    observe(1.105170682)
    neutral = world.send()
    assert (neutral.cmd_set,neutral.cmd_id) == (1,0xa)
    land = world.send(acknowledge=False)
    assert land.payload == b'\2' and land.seq != takeoff.seq
    land_sent_at = world.now
    controller.tick()
    for elapsed in (1.3,1.5,1.605681375):
        observe(elapsed)
    # TAKEOFF ACK0 arrives1605.681375ms after TAKEOFF, while LAND still
    # has a live ACK window. It must not satisfy LAND's different sequence.
    assert 0 < world.now-land_sent_at < 1
    current_land = world.channel.receipt(land_id)
    world.channel.receive(DumlPacket(3,2,takeoff.seq,0x80,3,0x2a,b'\0'),world.now)
    assert world.channel.receipt(land_id) == current_land
    controller.tick()
    assert controller.pending_intent_id == land_id
    assert controller.pending is not None and controller.pending[0].seq == land.seq
    assert not controller.action_acknowledged and not controller.land_acknowledged
    assert controller.state == 'landing' and controller.fault_reason == 'action_ack_timeout'
    assert world.channel.next_packet(world.now,world.sequence.next) is None
    for elapsed in (1.8,2.,2.2,2.4,2.6,2.8,2.903538448):
        observe(elapsed)
    assert world.now-land_sent_at == pytest.approx(1.798367766)
    assert controller.state == 'fault' and controller.pending is None
    assert world.channel.receipt(land_id)['state'] == 'timed_out'
    world.channel.receive(DumlPacket(3,2,land.seq,0x80,3,0x2a,b'\0'),world.now)
    controller.tick()
    status = controller.status()
    assert status['fault_reason'] == 'action_ack_timeout' and status['state'] == 'fault'
    assert status['possible_airborne'] and not status['landing_confirmed']
    assert status['telemetry']['flying'] is False and status['telemetry']['motors_on'] is False
    assert status['telemetry']['grounded'] is None
    assert controller.lease is controller.control is None
    assert not controller.action_acknowledged and not controller.land_acknowledged
    for action,parameters in (
        ('prepare',{'operation':'arm'}),
        ('takeoff',{'confirmation_token':old_token,'lease':lease}),
        ('control',{'lease':lease,'sequence':1,'channels':[0,0,50,0],'expires_at':world.now+.2}),
        ('reset',{}),('close',{}),
    ):
        with pytest.raises(IntentRejected):
            controller.handle(action,parameters)
    assert world.channel.sent_count == 3
    assert world.channel.action is world.channel.control is None
    assert world.channel.next_packet(world.now,world.sequence.next) is None
    token = controller.handle('prepare',{'operation':'ground'})['confirmation_token']
    assert controller.flight_started  # Preparing the attestation proves nothing.
    controller.handle('confirm_ground',{'confirmation_token':token})
    reconciled = controller.status()
    assert not reconciled['possible_airborne'] and reconciled['telemetry']['grounded'] is True
    assert reconciled['telemetry']['ground_source'] == 'operator_confirmation'
    assert reconciled['state'] == 'fault' and reconciled['fault_reason'] == 'action_ack_timeout'
    assert not reconciled['landing_confirmed'] and not controller.action_acknowledged
    assert controller.lease is controller.control is None
    events = [json.loads(line) for line in journal.path.read_text().splitlines()]
    assert [e['action'] for e in events if e['event']=='action_intent'] == ['takeoff','land']
    assert not any(e['event']=='action_response' for e in events)
    assert len([e for e in events if e['event']=='action_ack_timeout']) == 2
    assert any(e['event']=='ground_reconciled_by_operator' for e in events)
    assert world.channel.sent_count == 3
