"""Decode actual envelopes before projecting a bound synthetic aircraft."""
from dataclasses import replace
import struct

import pytest

from neo2_control.battery import BatteryReceiver
from neo2_control.control_transport import FlightChannelBackend
from neo2_control.controller import ControlSupervisor, DecisionJournal, IntentRejected
from neo2_control.duml import DumlPacket, DumlSequence
from neo2_control.flight import FlightAction
from neo2_control.flight_channel import FlightChannel
from neo2_control.flight_observation import FlightObserver, FlightRoute, connection_distance
from neo2_control.telemetry import TelemetryReceiver
from neo2_control.wifi_status import WifiControlReceiver
from test_observation_envelope import carrier


class World:
    def __init__(self):
        self.now=1.
        self.route=FlightRoute('a'*64,0x12345678,0x23456789,123,1.,True)
        self.fc=TelemetryReceiver();self.battery=BatteryReceiver(maximum_age=FlightObserver.BATTERY_MAXIMUM_AGE);self.wifi=WifiControlReceiver()
        self.channel=FlightChannel(clock=lambda:self.now)
        self.backend=FlightChannelBackend(self.channel,synthetic=True)
        self.channel.bind_transport('127.0.0.1',9003)
        self.sequence=DumlSequence(current=0x800a)

    def push(self,component,*,source=None,destination=None,phase=0,latitude=.1,longitude=.2,
             gps=True,percent=90,mode=2,guard=2,direct=False):
        if component=='fc':
            body=bytearray(85);struct.pack_into('<ddh',body,0,longitude,latitude,10 if phase else 0)
            struct.pack_into('<I',body,32,(0x388000 if gps else 0)|phase);body[36]=12
            packet=DumlPacket(3,2,0x9000,0,3,0x43,bytes(body));target=self.fc
        elif component=='battery':
            body=struct.pack('<IHBBBB',0,8000,20,10,percent,1)
            packet=DumlPacket(3,2,0x9001,0,3,0x55,body);target=self.battery
        else:
            body=bytearray(27);body[0]=1;body[20]=guard;body[24]=mode
            packet=DumlPacket(0xe9,2,0x9002,0,0x51,4,bytes(body));target=self.wifi
        wire=packet.encode() if direct else carrier(packet.encode(),
            source=self.route.peer_hash if source is None else source,
            destination=self.route.own_hash if destination is None else destination).encode()
        target.feed(wire,self.now)

    def feed(self,*,phase=0,gps=True,mode=2,guard=2):
        self.push('fc',phase=phase,gps=gps);self.push('battery');self.push('wifi',mode=mode,guard=guard)
        self.project()

    def project(self):
        self.route=replace(self.route,seen_at=self.now)
        self.channel.set_ready(True,self.now)
        self.channel.update_observations(self.route,self.fc.snapshot(self.now),
            self.battery.snapshot(self.now),self.wifi.snapshot(self.now),self.now)

    def attest(self):
        key=self.backend.ground_confirmation_key(self.now)
        self.backend.confirm_ground(key,self.now)


def test_radio_mode_cannot_replace_unimplemented_local_sdk_control_state():
    world=World();world.feed()
    real=FlightObserver(synthetic=False)
    real.update(world.route,world.fc.snapshot(world.now),world.battery.snapshot(world.now),
                world.wifi.snapshot(world.now),world.now)
    key=real.ground_confirmation_key(world.now)
    real.confirm_ground(key,world.now)
    sample=real.read(world.now)
    assert sample.position_valid and sample.grounded and sample.motors_on is False
    assert sample.control_ready is False and not real.permits_control(world.now)
    snapshot=real.snapshot(world.now)
    assert 'host_control_state_unavailable' in snapshot['problems']
    assert snapshot['control_state_provenance']=='unavailable'
    assert snapshot['origin']['location'] is None


def test_native_idle_flags_do_not_invent_ground_or_distance_but_operator_statement_is_explicit():
    world=World();world.feed();sample=world.backend.observation()
    assert sample.grounded is None and sample.distance_m is None
    assert sample.altitude_m==0 and sample.control_ready and sample.position_valid and sample.synthetic
    world.attest();sample=world.backend.observation()
    assert sample.grounded is True and sample.ground_source=='operator_confirmation'
    assert sample.distance_m==0 and sample.received_at==1
    observer=world.channel.observer
    assert observer.snapshot()['distance_reference']=='connection_origin'
    assert observer.snapshot()['ground_confirmation']['sensor_contact_verified'] is False


def test_source_bound_sample_uses_oldest_required_receipt_and_never_renews_it_on_read():
    world=World();world.feed();world.attest();world.now=1.2
    world.push('fc');world.push('wifi');world.project()
    assert world.backend.observation().received_at==1
    world.now=1.51;world.project()
    assert world.backend.observation().received_at==1
    world.now=4.01;world.push('fc');world.push('wifi');world.project()
    partial=world.backend.observation()
    assert partial is not None and partial.motion_received_at==4.01
    assert partial.battery_percent is None and partial.battery_received_at is None
    assert not world.channel.observer.permits_control(world.now)
    assert 'battery_unavailable_or_stale' in world.channel.observer.problems


@pytest.mark.parametrize('component',['fc','battery','wifi'])
@pytest.mark.parametrize('alteration',['foreign-peer','wrong-destination','bare'])
def test_unbound_diagnostic_sources_cannot_fill_required_components(component,alteration):
    world=World()
    for name in ['fc','battery','wifi']:
        options={}
        if name==component:
            options={'source':77} if alteration=='foreign-peer' else {'destination':88} if alteration=='wrong-destination' else {'direct':True}
        world.push(name,**options)
    world.project();sample=world.backend.observation()
    if component=='wifi':assert sample is not None and sample.control_ready is False
    elif component=='fc':assert sample is None
    else:
        assert sample is not None and sample.battery_percent is None and sample.battery_received_at is None
        assert not world.channel.observer.permits_control(world.now)


def test_newer_foreign_peer_does_not_replace_current_bound_fc_sample():
    world=World();world.feed();world.now=1.1
    world.push('fc',source=77,phase=12);world.project()
    sample=world.backend.observation()
    assert sample.flying is False and sample.motors_on is False
    assert world.channel.observer.sources['flight_controller']['forwarding']['source_hash']==world.route.peer_hash


@pytest.mark.parametrize('mode,guard',[(0,2),(1,2),(2,0),(2,1)])
def test_wifi_operation_mode_and_service_guard_are_independent_required_conditions(mode,guard):
    world=World();world.feed(mode=mode,guard=guard)
    assert world.backend.observation().control_ready is False


def test_gps_or_battery_unavailable_are_preserved_instead_of_fabricated():
    world=World();world.feed(gps=False);world.attest()
    sample=world.backend.observation()
    assert sample.position_valid is False and sample.distance_m is None
    world.now=1.1;world.push('battery',percent=255);world.project()
    assert world.backend.observation().battery_percent is None


@pytest.mark.parametrize('change',['motors-on','flying','stale-fc','new-nonce','takeoff-intent','revoked'])
def test_ground_attestation_cannot_survive_activity_or_a_changed_aircraft_context(change):
    world=World();world.feed();world.attest();key=world.backend.ground_confirmation_key(world.now)
    world.now=1.1
    if change in ('motors-on','flying'):world.feed(phase=8 if change=='motors-on' else 12)
    elif change=='stale-fc':world.now=1.6;world.push('battery');world.push('wifi');world.project()
    elif change=='new-nonce':world.route=replace(world.route,nonce=124);world.project()
    elif change=='takeoff-intent':world.channel.submit_action(FlightAction.TAKEOFF,expires_at=1.3)
    else:world.channel.revoke()
    sample=world.backend.observation()
    assert sample is None or sample.grounded is not True
    with pytest.raises(ValueError):world.backend.confirm_ground(key,world.now)


def test_operator_statement_expires_without_a_receive_to_refresh_it():
    world=World();world.feed();world.attest()
    world.now=6
    assert world.backend.observation() is None
    status=world.channel.snapshot()['observations']
    assert status['sample'] is None and not status['sample_fresh']
    world.feed()
    assert world.backend.observation().grounded is None


def test_connection_origin_persists_during_movement_but_never_across_generation():
    world=World();world.feed();world.attest();origin=world.channel.observer.origin
    world.now=1.1;world.push('fc',phase=12,latitude=.100001);world.push('battery');world.push('wifi');world.project()
    assert world.channel.observer.origin==origin
    assert world.backend.observation().distance_m>6
    world.now=1.2;world.route=replace(world.route,nonce=456);world.feed(phase=12)
    assert world.channel.observer.origin is None and world.backend.observation().distance_m is None


def test_connection_distance_uses_geographic_coordinates_and_handles_longitude_wrap():
    assert connection_distance((0,0),(0,0))==0
    assert connection_distance((0,0),(0,.00001))==pytest.approx(1.111949,rel=1e-5)
    assert connection_distance((0,179.999999),(0,-179.999999))<.23


def test_queued_takeoff_loses_preflight_when_control_mode_changes_but_neutral_land_is_available():
    world=World();world.feed();world.attest()
    intent=world.channel.submit_action(FlightAction.TAKEOFF,expires_at=1.25)
    packet=world.channel.next_packet(1,world.sequence.next)
    world.now=1.05;world.feed(mode=1)
    assert not world.channel.can_send(packet.seq,world.now)
    assert world.channel.receipt(intent['intent_id'])['state']=='cancelled'
    world.channel.submit_landing(expires_at=1.3)
    neutral=world.channel.next_packet(world.now,world.sequence.next)
    assert world.channel.can_send(neutral.seq,world.now)
    world.channel.sent(neutral.seq,world.now)
    land=world.channel.next_packet(world.now,world.sequence.next)
    assert land.payload==b'\2' and world.channel.can_send(land.seq,world.now)


def test_supervisor_ground_confirmation_is_explicit_single_use_and_emits_no_packet(tmp_path):
    world=World();world.feed();journal=DecisionJournal(tmp_path/'ground.jsonl')
    controller=ControlSupervisor(journal,mode='simulate',backend=world.backend,clock=lambda:world.now)
    try:
        controller.handle('open');assert controller.state=='not_ready'
        token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
        controller.handle('confirm_ground',{'confirmation_token':token})
        assert controller.state=='ready' and world.channel.sent_count==0
        with pytest.raises(IntentRejected,match='confirmation_invalid'):
            controller.handle('confirm_ground',{'confirmation_token':token})
        assert controller.status()['telemetry']['ground_source']=='operator_confirmation'
    finally:journal.close()


def test_ground_confirmation_cannot_reuse_a_token_after_motor_activity(tmp_path):
    world=World();world.feed();journal=DecisionJournal(tmp_path/'ground.jsonl')
    controller=ControlSupervisor(journal,mode='simulate',backend=world.backend,clock=lambda:world.now)
    try:
        controller.handle('open');token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
        world.now=1.1;world.feed(phase=8);world.now=1.2;world.feed()
        with pytest.raises(IntentRejected,match='ground_confirmation_context_changed'):
            controller.handle('confirm_ground',{'confirmation_token':token})
        assert controller.state=='not_ready' and not world.channel.sent_count
    finally:journal.close()


@pytest.mark.parametrize('mode,guard',[(1,2),(2,1)])
def test_owned_controller_cannot_arm_with_only_a_ground_statement_when_wifi_control_is_unconfirmed(tmp_path,mode,guard):
    world=World();world.feed(mode=mode,guard=guard);journal=DecisionJournal(tmp_path/'ground.jsonl')
    controller=ControlSupervisor(journal,mode='simulate',backend=world.backend,clock=lambda:world.now)
    try:
        controller.handle('open');token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
        controller.handle('confirm_ground',{'confirmation_token':token})
        assert controller.state=='not_ready'
        with pytest.raises(IntentRejected,match='invalid_state'):
            controller.handle('prepare',{'operation':'arm'})
        assert not world.channel.sent_count
    finally:journal.close()


def test_expired_operator_challenge_cannot_set_ground_or_create_an_origin(tmp_path):
    world=World();world.feed();journal=DecisionJournal(tmp_path/'ground.jsonl')
    controller=ControlSupervisor(journal,mode='simulate',backend=world.backend,clock=lambda:world.now)
    try:
        controller.handle('open');token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
        world.now=6.;world.feed()
        with pytest.raises(IntentRejected,match='confirmation_expired'):
            controller.handle('confirm_ground',{'confirmation_token':token})
        assert world.backend.observation().grounded is None and world.channel.observer.origin is None
    finally:journal.close()


def test_new_generation_does_not_adopt_cached_values_from_the_previous_boot():
    world=World();world.feed();world.attest();world.now=1.1
    world.route=replace(world.route,nonce=456);world.project()
    assert world.backend.observation() is None
    assert world.channel.observer.origin is None
    world.push('fc');world.push('battery');world.project()
    sample=world.backend.observation()
    assert sample is not None and sample.control_ready is False and sample.grounded is None


def test_current_fc_packets_do_not_extend_an_expired_peer_beacon():
    world=World();world.feed();world.attest();world.now=3.
    assert world.backend.observation() is None


def test_received_battery_cadence_has_its_own_deadline_without_renewing_motion(tmp_path):
    world=World();world.feed();journal=DecisionJournal(tmp_path/'cadence.jsonl')
    controller=ControlSupervisor(journal,mode='simulate',backend=world.backend,clock=lambda:world.now)
    try:
        controller.handle('open');token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
        controller.handle('confirm_ground',{'confirmation_token':token})
        world.now=3.5;world.push('fc');world.push('wifi');world.project();controller.tick()
        status=controller.status()
        assert controller.state=='ready' and status['telemetry']['valid']
        assert status['telemetry']['received_at']==1 and status['telemetry']['motion_received_at']==3.5
        assert status['telemetry']['component_age_seconds']=={'motion':0,'battery':2.5,'control':0}
        world.now=4.01;world.push('fc');world.push('wifi');world.project();controller.tick()
        partial=world.backend.observation()
        assert controller.state=='not_ready' and partial is not None
        assert partial.motion_received_at==4.01 and partial.battery_received_at is None
        assert partial.battery_percent is None and not world.channel.observer.permits_control(world.now)
    finally:journal.close()


def test_fresh_battery_does_not_make_old_flight_motion_current():
    world=World();world.feed();world.attest();world.now=1.6
    world.push('battery');world.push('wifi');world.project()
    assert world.backend.observation() is None
    assert 'flight_controller_unavailable_or_stale' in world.channel.observer.problems


def test_scheduled_takeoff_cannot_use_battery_after_its_deadline_even_without_an_owner_update():
    world=World();world.feed();world.attest();world.now=3.9
    world.push('fc');world.push('wifi');world.project()
    world.channel.submit_action(FlightAction.TAKEOFF,expires_at=4.15)
    packet=world.channel.next_packet(world.now,world.sequence.next)
    world.now=4.01
    assert not world.channel.can_send(packet.seq,world.now)
    assert not world.channel.sent_count


def test_operator_can_reconcile_ground_after_missing_land_ack_without_inventing_action_success(tmp_path):
    world=World();world.feed();journal=DecisionJournal(tmp_path/'ground.jsonl')
    controller=ControlSupervisor(journal,mode='simulate',backend=world.backend,clock=lambda:world.now)
    def send():
        packet=world.channel.next_packet(world.now,world.sequence.next)
        assert packet is not None and world.channel.can_send(packet.seq,world.now)
        world.channel.sent(packet.seq,world.now)
        return packet
    try:
        controller.handle('open')
        token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
        controller.handle('confirm_ground',{'confirmation_token':token})
        token=controller.handle('prepare',{'operation':'arm'})['confirmation_token']
        lease=controller.handle('arm',{'confirmation_token':token})['lease']
        token=controller.handle('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
        controller.handle('takeoff',{'lease':lease,'confirmation_token':token})
        world.now=1.01;request=send()
        response=DumlPacket(request.receiver,request.sender,request.seq,0xc0,3,0x2a,b'\0')
        world.now=1.02;world.channel.receive(response,world.now);world.feed(phase=12);controller.tick()
        assert controller.state=='flying'
        controller.handle('land');world.now=1.03;send();world.now=1.04;send()
        world.now=1.55;world.feed();controller.tick()
        assert controller.state=='fault' and controller.flight_started and not controller.landing_confirmed
        token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
        controller.handle('confirm_ground',{'confirmation_token':token})
        assert not controller.flight_started and not controller.landing_confirmed
        assert not controller.action_acknowledged and world.channel.sent_count==3
        controller.handle('close');assert controller.state=='disconnected'
    finally:journal.close()


@pytest.mark.parametrize('phase',[0,8])
def test_queued_receipt_is_evaluated_after_a_concurrent_ground_confirmation(phase):
    world=World();world.feed()
    world.now=1.1;world.attest()
    assert world.backend.observation().grounded is True
    # The UDP owner timestamped a packet before the operator thread took the
    # channel lock. Its eventual projection must use the current decision
    # clock while preserving all received component timestamps.
    receipt=1.05;world.now=receipt
    world.push('fc',phase=phase);world.push('battery');world.push('wifi')
    route=replace(world.route,seen_at=receipt)
    inputs=(world.fc.snapshot(receipt),world.battery.snapshot(receipt),world.wifi.snapshot(receipt))
    world.now=1.11
    world.channel.update_observations(route,*inputs,receipt)
    sample=world.backend.observation()
    assert sample.grounded is (True if phase==0 else False)
    assert sample.motion_received_at==sample.battery_received_at==sample.control_received_at==receipt
    assert world.channel.observer.last_time==world.now


def test_delayed_projection_cannot_make_an_expired_receipt_current():
    world=World();world.feed();world.now=1.1;world.attest()
    receipt=1.05
    inputs=(world.fc.snapshot(receipt),world.battery.snapshot(receipt),world.wifi.snapshot(receipt))
    world.now=1.6
    world.channel.update_observations(world.route,*inputs,receipt)
    assert world.backend.observation() is None
    assert world.channel.observer.ground_confirmed_at is None


@pytest.mark.parametrize('bad',[True,None,float('nan'),float('inf')])
def test_projection_clock_validation_is_not_hidden_by_current_owner_time(bad):
    world=World();world.feed()
    with pytest.raises(ValueError,match='clock'):
        world.channel.update_observations(world.route,{}, {}, {},bad)


@pytest.mark.parametrize('power_state',['missing','foreign','stale'])
def test_partial_fresh_fc_supports_explicit_ground_but_cannot_arm_takeoff_or_move(tmp_path,power_state):
    world=World()
    if power_state=='stale':
        world.push('battery');world.now=4.01
    elif power_state=='foreign':world.push('battery',source=77)
    world.push('fc');world.push('wifi');world.project()
    sample=world.backend.observation()
    assert sample is not None and sample.motion_received_at==world.now
    assert sample.battery_percent is None and sample.battery_received_at is None
    assert sample.grounded is None and world.channel.observer.origin is None
    snapshot=world.channel.observer.snapshot(world.now)
    assert snapshot['component_age_seconds']['battery'] is None and not snapshot['sample_fresh']
    journal=DecisionJournal(tmp_path/'partial-ground.jsonl')
    controller=ControlSupervisor(journal,mode='simulate',backend=world.backend,clock=lambda:world.now)
    try:
        controller.handle('open')
        token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
        controller.handle('confirm_ground',{'confirmation_token':token})
        attested=world.backend.observation()
        assert attested.grounded is True and attested.ground_source=='operator_confirmation'
        assert attested.battery_received_at is None and attested.battery_percent is None
        assert attested.distance_m==0 and world.channel.observer.origin_received_at==world.now
        assert controller.state=='not_ready' and not world.channel.observer.permits_control(world.now)
        with pytest.raises(IntentRejected):controller.handle('prepare',{'operation':'arm'})
        for action in ['takeoff','control']:
            if action=='takeoff':world.channel.submit_action(FlightAction.TAKEOFF,expires_at=world.now+.2)
            else:world.channel.submit_control([50,0,0,0],expires_at=world.now+.2)
            packet=world.channel.next_packet(world.now,world.sequence.next)
            assert packet is not None and not world.channel.can_send(packet.seq,world.now)
        assert world.channel.sent_count==0
    finally:journal.close()


def test_missing_battery_does_not_hide_airborne_state_or_allow_ground_attestation():
    world=World();world.push('fc',phase=12);world.push('wifi');world.project()
    sample=world.backend.observation()
    assert sample is not None and sample.flying and sample.motors_on and sample.grounded is False
    assert sample.battery_percent is None and sample.battery_received_at is None
    assert sample.motion_received_at==1. and sample.control_received_at==1.
    assert world.channel.observer.origin is None
    with pytest.raises(ValueError,match='fresh_stationary'):
        world.backend.ground_confirmation_key(world.now)
    assert not world.channel.observer.permits_control(world.now)


def test_invalid_current_battery_value_keeps_its_real_stamp_but_cannot_authorize_control():
    world=World();world.push('fc');world.push('wifi');world.push('battery',percent=255);world.project()
    sample=world.backend.observation()
    assert sample is not None and sample.battery_percent is None and sample.battery_received_at==1.
    assert 'battery_percent_invalid' in world.channel.observer.problems
    world.attest()
    assert not world.channel.observer.permits_control(world.now)
