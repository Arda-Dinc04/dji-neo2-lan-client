"""Explicit landing continuation, decoded fixture input and no radio hardware."""
from dataclasses import replace
import json
import struct

import pytest

from neo2_control.controller import ControlLimits, ControlSupervisor, DecisionJournal, IntentRejected
from neo2_control.duml import DumlPacket
from neo2_control.flight import FlightAction
from neo2_control.host_control import HostControlState
from neo2_control.landing_confirmation import LandingConfirmationReceiver
from neo2_control.wlm_switch import WIFI_SERVICE_MODES
from test_flight_observation import World
from test_observation_envelope import carrier


class LandingWorld(World):
    def __init__(self):
        super().__init__()
        self.landing = LandingConfirmationReceiver()
        self.phase = 0
        self.flight_mode = 0
        self.needed = False
        self.gps = True
        self.percent = 90

    def feed(self, *, flag=True):
        body = bytearray(85)
        struct.pack_into('<ddh',body,0,.2,.1,10 if self.phase else 0)
        struct.pack_into('<I',body,32,(0x388000 if self.gps else 0)|self.phase)
        body[30] = self.flight_mode
        body[36] = 12
        motion = DumlPacket(3,2,0x9000,0,3,0x43,bytes(body))
        self.fc.feed(carrier(motion.encode(),source=self.route.peer_hash,
            destination=self.route.own_hash).encode(),self.now)
        self.push('battery',percent=self.percent)
        if flag:
            request = DumlPacket(3,2,0x9001,0,3,0x53,
                (0x33 if self.needed else 0x13).to_bytes(4,'little'))
            self.landing.feed(carrier(request.encode(),source=self.route.peer_hash,
                destination=0xffffffff).encode(),self.now)
        self.project()

    def project(self):
        host = HostControlState(self.route.generation,self.now,self.now,self.now,
            WIFI_SERVICE_MODES,WIFI_SERVICE_MODES,True,True)
        self.route = replace(self.route,seen_at=self.now,host_control=host)
        self.channel.set_ready(True,self.now)
        self.channel.update_observations(self.route,self.fc.snapshot(self.now),
            self.battery.snapshot(self.now),self.wifi.snapshot(self.now),self.now,
            landing_confirmation=self.landing.snapshot(self.now))

    def send(self, *, acknowledge=True, status=0):
        request = self.channel.next_packet(self.now,self.sequence.next)
        assert request is not None and self.channel.can_send(request.seq,self.now)
        self.channel.sent(request.seq,self.now)
        if acknowledge and request.cmd_id == 0x2a:
            self.channel.receive(DumlPacket(3,2,request.seq,0x80,3,0x2a,bytes([status])),self.now)
        return request


@pytest.fixture
def rig(tmp_path):
    world = LandingWorld()
    world.feed()
    journal = DecisionJournal(tmp_path/'decisions.jsonl')
    controller = ControlSupervisor(journal,mode='simulate',backend=world.backend,
        clock=lambda:world.now,limits=ControlLimits(lease_seconds=2.,confirmation_seconds=.2))
    controller.handle('open')
    token = controller.handle('prepare',{'operation':'ground'})['confirmation_token']
    controller.handle('confirm_ground',{'confirmation_token':token})
    token = controller.handle('prepare',{'operation':'arm'})['confirmation_token']
    lease = controller.handle('arm',{'confirmation_token':token})['lease']
    token = controller.handle('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
    controller.handle('takeoff',{'confirmation_token':token,'lease':lease})
    takeoff = world.send()
    assert takeoff.payload == b'\1'
    world.now += .01
    world.phase = 12
    world.feed()
    controller.tick()
    assert controller.state == 'flying'
    yield world,controller,journal
    journal.close()


def request_land(world,controller, *, acknowledge=True):
    controller.handle('land')
    neutral = world.send()
    assert (neutral.cmd_set,neutral.cmd_id) == (1,0xa)
    request = world.send(acknowledge=acknowledge)
    assert request.payload == b'\2'
    world.now += .01
    world.flight_mode = 12
    world.needed = True
    world.feed()
    controller.tick()
    return request


def confirmation(controller):
    return controller.handle('prepare',{'operation':'landing'})['confirmation_token']


def test_land_request_does_not_automatically_confirm_and_ack_never_proves_touchdown(rig):
    world,controller,journal = rig
    request_land(world,controller)
    count = world.channel.sent_count
    for _ in range(5):
        world.now += .01
        world.feed()
        controller.tick()
        assert controller.status()['landing_confirmation']['available']
    assert world.channel.sent_count == count
    assert world.channel.next_packet(world.now,world.sequence.next) is None
    token = confirmation(controller)
    assert world.channel.sent_count == count
    controller.handle('confirm_landing',{'confirmation_token':token})
    confirm = world.send()
    assert (confirm.sender,confirm.receiver,confirm.cmd_type,confirm.cmd_set,confirm.cmd_id,confirm.payload) == (
        2,3,0x40,3,0x2a,b'\x1e')
    controller.tick()
    assert controller.action_acknowledged and controller.flight_started
    assert not controller.landing_confirmed
    assert controller.lease is None and controller.control is None
    with pytest.raises(IntentRejected,match='confirmation_invalid'):
        controller.handle('confirm_landing',{'confirmation_token':token})
    world.now += .01
    world.phase = 0
    world.needed = False
    world.feed()
    controller.tick()
    assert not controller.landing_confirmed  # no sensor contact invented
    token_ground = controller.handle('prepare',{'operation':'ground'})['confirmation_token']
    controller.handle('confirm_ground',{'confirmation_token':token_ground})
    assert controller.landing_confirmed and not controller.flight_started
    assert controller.state == 'ready'
    assert token not in journal.path.read_text() and token not in json.dumps(controller.status())
    replies = [e for e in controller.events if e['event']=='action_response']
    assert replies[-1]['request_action'] == 'confirm_landing'
    assert replies[-1]['accepted'] and not replies[-1]['execution_confirmed']


@pytest.mark.parametrize('acknowledge',[False,True])
def test_confirmation_requires_this_supervisors_current_acknowledged_land(rig,acknowledge):
    world,controller,journal = rig
    if acknowledge:
        world.needed = True
        world.flight_mode = 12
        world.feed()
        error = 'landing_intent_required'
    else:
        request_land(world,controller,acknowledge=False)
        error = 'flight_action_pending'
    with pytest.raises(IntentRejected,match=error):confirmation(controller)
    assert world.channel.action is None or world.channel.action['action'] != 'confirm_landing'


@pytest.mark.parametrize('change',['expired','flag-false','flag-cycle','mode','generation','same-identity-reconnect','stale-flag'])
def test_operator_confirmation_cannot_survive_context_changes(rig,change):
    world,controller,journal = rig
    request_land(world,controller)
    token = confirmation(controller)
    world.now += .01
    if change == 'expired':world.now += .21
    elif change == 'flag-false':world.needed = False
    elif change == 'flag-cycle':
        world.needed = False
        world.feed()
        world.needed = True
        world.now += .01
    elif change == 'mode':world.flight_mode = 6
    elif change == 'generation':world.route = replace(world.route,nonce=124,host_control=None)
    elif change == 'same-identity-reconnect':world.channel.observer.invalidate()
    elif change == 'stale-flag':world.now += .51
    world.feed(flag=change != 'stale-flag')
    with pytest.raises(IntentRejected):
        controller.handle('confirm_landing',{'confirmation_token':token})
    assert world.channel.action is None or world.channel.action['action'] != 'confirm_landing'
    with pytest.raises(IntentRejected,match='confirmation_invalid'):
        controller.handle('confirm_landing',{'confirmation_token':token})


def test_pending_confirmation_cannot_be_replaced_by_duplicate_land_or_watchdog(rig):
    world,controller,journal = rig
    request_land(world,controller)
    token = confirmation(controller)
    controller.handle('confirm_landing',{'confirmation_token':token})
    intent = controller.pending_intent_id
    assert controller.handle('land')['landing_already_pending']
    assert controller.pending_intent_id == intent
    world.gps = False
    world.now += .01
    world.feed()
    controller.tick()
    assert controller.pending_intent_id == intent
    assert controller.pending_action is FlightAction.CONFIRM_LANDING
    assert controller.fault_reason == 'position_invalid'
    confirm = world.send()
    assert confirm.payload == b'\x1e'
    controller.tick()
    assert not controller.landing_confirmed
    assert world.channel.next_packet(world.now,world.sequence.next) is None


@pytest.mark.parametrize('failure',['rejected','ack-timeout','send-expired'])
def test_confirmation_failure_never_queues_implicit_retry_or_force_land(rig,failure):
    world,controller,journal = rig
    request_land(world,controller)
    token = confirmation(controller)
    controller.handle('confirm_landing',{'confirmation_token':token})
    intent = controller.pending_intent_id
    if failure != 'send-expired':
        request = world.send(acknowledge=failure=='rejected',status=2)
        assert request.payload == b'\x1e'
    world.now += .51 if failure == 'ack-timeout' else .26 if failure == 'send-expired' else .01
    world.feed()
    controller.tick()
    assert controller.state == 'fault' and controller.flight_started
    assert not controller.landing_confirmed and controller.pending is None
    assert controller.pending_intent_id == intent
    assert world.channel.next_packet(world.now,world.sequence.next) is None
    assert not any(item.get('action') == 'force_landing' for item in world.channel.history)


def test_landing_confirmation_does_not_restore_preflight_or_motion_authority(rig):
    world,controller,journal = rig
    request_land(world,controller)
    world.gps = False
    world.percent = 10
    world.now += .01
    world.feed()
    controller.tick()  # existing failsafe may request one new LAND
    if controller.pending is not None:
        world.send()
        assert world.send().payload == b'\2'
        world.now += .01
        world.feed()
        controller.tick()
    assert controller.status()['landing_confirmation']['available']
    token = confirmation(controller)
    controller.handle('confirm_landing',{'confirmation_token':token})
    assert world.send().payload == b'\x1e'
    controller.tick()
    assert not controller.status()['telemetry']['valid']
    for operation in ('arm','takeoff'):
        with pytest.raises(IntentRejected):controller.handle('prepare',{'operation':operation})
    with pytest.raises(IntentRejected,match='operator_lease_invalid'):
        controller.handle('control',{'lease':'old','sequence':1,'channels':[0]*4,'expires_at':world.now+.2})


def test_an_explicit_retry_needs_new_operator_token_and_its_own_wire_ack(rig):
    world,controller,journal = rig
    request_land(world,controller)
    first_token = confirmation(controller)
    controller.handle('confirm_landing',{'confirmation_token':first_token})
    first = world.send()
    controller.tick()
    world.now += .01
    world.feed()
    controller.tick()
    assert world.channel.next_packet(world.now,world.sequence.next) is None
    second_token = confirmation(controller)
    assert second_token != first_token
    controller.handle('confirm_landing',{'confirmation_token':second_token})
    second = world.send(acknowledge=False)
    assert second.payload == b'\x1e' and second.seq != first.seq
    world.channel.receive(DumlPacket(3,2,first.seq,0x80,3,0x2a,b'\0'),world.now)
    controller.tick()
    assert controller.pending is not None and not controller.action_acknowledged
    world.channel.receive(DumlPacket(3,2,second.seq,0x80,3,0x2a,b'\0'),world.now)
    controller.tick()
    assert controller.action_acknowledged and not controller.landing_confirmed


def test_local_land_ack_and_observations_cannot_carry_across_route_loss(rig):
    world,controller,journal = rig
    request_land(world,controller)
    assert controller.status()['landing_confirmation']['available']
    world.now += .01
    world.channel.set_ready(False,world.now)
    world.now += .01
    world.feed()  # same WLM identity/nonce, different local route episode
    assert not controller.status()['landing_confirmation']['available']
    with pytest.raises(IntentRejected,match='landing_route_changed'):
        confirmation(controller)


def test_confirmation_flag_and_motion_must_follow_the_local_land_intent(rig):
    world,controller,journal = rig
    world.flight_mode = 12
    world.needed = True
    world.feed()
    world.now += .01
    controller.handle('land')
    world.send()
    world.send()
    controller.tick()
    with pytest.raises(IntentRejected,match='evidence_precedes_intent'):
        confirmation(controller)
    world.now += .01
    world.feed()
    assert controller.status()['landing_confirmation']['available']
