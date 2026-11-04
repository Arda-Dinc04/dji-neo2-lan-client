"""Fresh typed landing evidence and last-moment command checks, offline only."""
from dataclasses import replace
import json
import struct

import pytest

from neo2_control.control_transport import FlightChannelBackend
from neo2_control.duml import DumlPacket
from neo2_control.flight import FlightAction
from neo2_control.flight_channel import FlightChannel
from neo2_control.landing_confirmation import LandingConfirmationReceiver
from test_flight_observation import World
from test_landing_confirmation import owner, TRUE_FRAME, wrap
from test_observation_envelope import carrier


class LandingWorld(World):
    def __init__(self):
        super().__init__()
        self.confirmation=LandingConfirmationReceiver()
        self.backend.open(self.now)

    def owned_route(self,**changes):
        state=owner(self.now,generation=self.route.generation,**changes)
        self.route=replace(self.route,seen_at=self.now,host_control=state)

    def motion(self,*,mode=12,phase=12):
        body=bytearray(85)
        struct.pack_into('<ddh',body,0,.2,.1,4)
        struct.pack_into('<I',body,32,phase)
        body[30]=mode
        raw=DumlPacket(3,2,0x9000,0,3,0x43,bytes(body)).encode()
        self.fc.feed(carrier(raw,source=self.route.peer_hash,destination=0xffffffff).encode(),self.now)

    def flag(self,value=True,*,source=None,direct=False):
        packet=replace(DumlPacket.decode(TRUE_FRAME),payload=bytes.fromhex('33000000' if value else '13000000'))
        wire=packet.encode() if direct else wrap(packet.encode(),source=self.route.peer_hash if source is None else source)
        self.confirmation.feed(wire,self.now,direct_context=self.route.host_control)

    def project(self):
        self.channel.set_ready(True,self.now)
        self.channel.update_observations(self.route,self.fc.snapshot(self.now),{}, {},self.now,
            landing_confirmation=self.confirmation.snapshot(self.now))

    def feed(self,*,value=True,mode=12,phase=12,motion=True,flag=True,**owner_changes):
        self.owned_route(**owner_changes)
        if motion:self.motion(mode=mode,phase=phase)
        if flag:self.flag(value)
        self.project()

    def enqueue(self,*,since=1.):
        key=self.backend.landing_confirmation_key(self.now,since=since)
        ident=self.backend.submit_action(FlightAction.CONFIRM_LANDING,self.now,
            expires_at=self.now+.2,confirmation_key=key,since=since)
        return key,ident


def test_true_flag_alone_sends_nothing_and_evidence_status_contains_no_opaque_keys():
    world=LandingWorld();world.feed()
    assert world.channel.next_packet(world.now,world.sequence.next) is None
    status=world.backend.landing_confirmation_status(world.now)
    assert status=={'needed':True,'received_at':1.,'motion_received_at':1.,
        'control_received_at':1.,'route_seen_at':1.,'ready_at':1.,
        'reason':None,'evidence_available':True}
    json.dumps(status);json.dumps(world.channel.snapshot())


def test_explicit_guarded_confirmation_is_normal_one_byte_wire_without_ordinary_gps_battery_gate():
    world=LandingWorld();world.feed()
    assert not world.channel.observer.permits_control(world.now)
    key,ident=world.enqueue()
    world.channel.set_control_fresh(False,world.now)
    request=world.channel.next_packet(world.now,lambda:0x8101)
    assert request.encode().hex()=='550e04660203018140032a1ed0a9'
    assert world.channel.can_send(request.seq,world.now)
    world.channel.sent(request.seq,world.now)
    assert world.channel.next_packet(world.now,world.sequence.next) is None
    response=DumlPacket(3,2,request.seq,0x80,3,0x2a,b'\0')
    world.channel.receive(response,world.now)
    result=world.backend.action_receipt(ident)
    assert result['state']=='acknowledged' and result['execution_confirmed'] is False
    assert 'confirmation_key' not in result
    json.dumps(result)


@pytest.mark.parametrize('changes',[{'mode':11},{'mode':33},{'phase':0},{'phase':8},
    {'motion':False},{'value':False},{'committed_at':None}])
def test_flag_cannot_substitute_for_active_auto_landing_motion_and_owned_control(changes):
    world=LandingWorld();world.feed(**changes)
    with pytest.raises(ValueError):world.backend.landing_confirmation_key(world.now)
    assert not world.backend.landing_confirmation_status(world.now)['evidence_available']


@pytest.mark.parametrize('old',['motion','flag'])
def test_both_receipts_must_follow_current_land_intent(old):
    world=LandingWorld();world.feed();world.now=1.1
    world.feed(motion=old!='motion',flag=old!='flag')
    with pytest.raises(ValueError,match='precedes_intent'):
        world.backend.landing_confirmation_key(world.now,since=1.05)


def test_continuously_fresh_episode_key_survives_new_receipts_but_false_and_recovery_change_it():
    world=LandingWorld();world.feed();original=world.backend.landing_confirmation_key(1.)
    world.now=1.1;world.feed()
    assert world.backend.landing_confirmation_key(1.1)==original
    world.now=1.2;world.feed(value=False)
    with pytest.raises(ValueError):world.backend.landing_confirmation_key(1.2)
    world.now=1.3;world.feed()
    assert world.backend.landing_confirmation_key(1.3)!=original


def test_unpolled_evidence_expiry_between_updates_starts_new_episode():
    world=LandingWorld();world.feed();old=world.backend.landing_confirmation_key(1.)
    world.now=1.6;world.feed()
    assert world.backend.landing_confirmation_key(world.now)!=old


def test_same_peer_generation_cannot_reuse_keys_after_local_invalidation_or_new_channel():
    world=LandingWorld();world.feed();old=world.backend.landing_route_key(1.)
    world.channel.observer.invalidate();world.now=1.1;world.feed()
    assert world.backend.landing_route_key(1.1)!=old
    other=LandingWorld();other.feed()
    assert other.route.generation==world.route.generation
    assert other.backend.landing_route_key(1.)!=old


def test_newer_foreign_push_cannot_refresh_owned_confirmation():
    world=LandingWorld();world.feed(value=False);world.now=1.1
    world.owned_route();world.motion();world.flag(source=77);world.project()
    status=world.backend.landing_confirmation_status(world.now)
    assert status['needed'] is False and status['received_at']==1.
    with pytest.raises(ValueError):world.backend.landing_confirmation_key(world.now)


def test_unbound_direct_confirmation_cannot_use_radio_host_labels_as_owner():
    world=LandingWorld();world.owned_route();world.motion()
    world.confirmation.feed(TRUE_FRAME,world.now)
    world.project()
    with pytest.raises(ValueError):world.backend.landing_confirmation_key(world.now)
    world.now=1.1;world.owned_route();world.motion();world.flag(direct=True);world.project()
    assert world.backend.landing_confirmation_status(world.now)['evidence_available']


def test_equal_receipt_conflict_inhibits_confirmation_instead_of_using_route_iteration_order():
    world=LandingWorld();world.feed()
    world.flag(False,direct=True);world.project()
    assert world.backend.landing_confirmation_status(world.now)['reason']=='landing_confirmation_conflicting_observations'
    with pytest.raises(ValueError):world.backend.landing_confirmation_key(world.now)


@pytest.mark.parametrize('loss',['flag','mode','motion','host','generation','route','expiry'])
def test_scheduled_confirmation_revalidates_at_actual_send_boundary(loss):
    world=LandingWorld();world.feed();_,ident=world.enqueue()
    request=world.channel.next_packet(world.now,world.sequence.next)
    world.now=1.1
    if loss=='flag':world.feed(value=False)
    elif loss=='mode':world.feed(mode=11)
    elif loss=='motion':world.feed(phase=0)
    elif loss=='host':world.feed(committed_at=None)
    elif loss=='generation':
        world.route=replace(world.route,nonce=124,host_control=None);world.feed()
    elif loss=='route':world.channel.set_ready(False,world.now)
    else:world.now=1.2
    assert not world.channel.can_send(request.seq,world.now)
    assert world.backend.action_receipt(ident)['state'] in ('cancelled','expired')
    assert world.channel.sent_count==0


def test_pending_confirmation_cannot_be_replaced_but_new_explicit_terminal_intent_can_retry():
    world=LandingWorld();world.feed();key,ident=world.enqueue()
    with pytest.raises(RuntimeError,match='pending'):world.enqueue()
    request=world.channel.next_packet(world.now,world.sequence.next)
    assert world.channel.can_send(request.seq,world.now)
    world.channel.sent(request.seq,world.now)
    world.channel.receive(DumlPacket(3,2,request.seq,0x80,3,0x2a,b'\1'),world.now)
    assert world.backend.action_receipt(ident)['state']=='rejected'
    # No new packet exists until a second explicit submit; the evidence key is
    # not an operator token and can describe a still-fresh continuous episode.
    assert world.channel.next_packet(world.now,world.sequence.next) is None
    assert world.backend.landing_confirmation_key(world.now)==key
    _,second=world.enqueue()
    retry=world.channel.next_packet(world.now,world.sequence.next)
    assert second!=ident and retry.seq!=request.seq and retry.payload==b'\x1e'


def test_confirmation_cannot_replace_pending_land_or_use_arbitrary_caller_guard():
    world=LandingWorld();world.feed()
    for key in (None,object(),('caller','claim')):
        with pytest.raises(ValueError):
            world.channel.submit_action(FlightAction.CONFIRM_LANDING,expires_at=1.2,confirmation_key=key)
    world.channel.submit_action(FlightAction.LAND,expires_at=1.2)
    with pytest.raises(RuntimeError,match='pending'):world.enqueue()


def test_stale_control_retains_only_confirmation_whose_separate_landing_guard_is_current():
    world=LandingWorld();world.feed();_,ident=world.enqueue()
    world.now=1.1;world.feed(mode=11)
    world.channel.set_control_fresh(False,world.now)
    assert world.backend.action_receipt(ident)['state']=='cancelled'


def test_owner_readiness_expiry_is_exposed_without_opaque_keys_and_revokes_old_route_key():
    world=LandingWorld();world.feed();old=world.backend.landing_route_key(1.)
    world.now=1.500001
    status=world.backend.landing_confirmation_status(world.now)
    assert status['ready_at']==1. and not status['evidence_available']
    assert status['reason']=='landing_route_not_ready'
    world.now=1.6;world.feed()
    assert world.backend.landing_route_key(world.now)!=old


def test_backend_without_owned_observer_cannot_confirm_even_with_external_flight_sample():
    channel=FlightChannel(clock=lambda:1.)
    backend=FlightChannelBackend(channel,lambda:None,synthetic=True)
    channel.bind_transport('127.0.0.1',9003);channel.set_ready(True,1.);backend.open(1.)
    with pytest.raises(ValueError):backend.landing_route_key(1.)
    assert not backend.landing_confirmation_status(1.)['evidence_available']


def test_flag_can_expire_before_action_deadline_while_motion_and_owner_stay_current():
    world=LandingWorld();world.feed();world.now=1.4;world.feed(flag=False)
    _,ident=world.enqueue()
    packet=world.channel.next_packet(world.now,world.sequence.next)
    world.now=1.51;world.feed(flag=False)
    assert world.now<world.backend.action_receipt(ident)['expires_at']
    assert not world.channel.can_send(packet.seq,world.now)
    assert world.backend.action_receipt(ident)['state']=='cancelled'


def test_mode_loss_and_recovery_cannot_restore_an_old_operator_evidence_context():
    world=LandingWorld();world.feed();old=world.backend.landing_confirmation_key(1.)
    route=world.backend.landing_route_key(1.)
    world.now=1.1;world.feed(mode=11)
    world.now=1.2;world.feed()
    assert world.backend.landing_route_key(world.now)==route
    assert world.backend.landing_confirmation_key(world.now)!=old


def test_owned_session_delivery_projects_direct_flag_and_motion_without_any_socket():
    from neo2_control.session import SwudpSession
    from test_flight_wlm_preflight import committed_link
    from test_wlm_link import IDENTITY
    link=committed_link()
    world=LandingWorld();world.now=2.
    client=SwudpSession(wlm_identity=IDENTITY,wlm_wifi_liveview=True,
        wifi_backend_ready=lambda:True,flight_channel=world.channel)
    client.wlm_link=link;link.flight_channel=world.channel
    client.state='open'
    client.wlm.open()
    world.channel.set_ready(True,world.now)
    body=bytearray(85);body[30]=12;struct.pack_into('<I',body,32,12)
    motion=DumlPacket(3,0xe,123,0,3,0x43,bytes(body)).encode()
    client._deliver(3,motion+TRUE_FRAME,world.now)
    status=world.backend.landing_confirmation_status(world.now)
    assert status['needed'] is True and status['evidence_available']
    observed=client.landing_confirmation.snapshot(world.now)['observations'][0]
    assert observed['source']['direct_connection']['generation']==list(link.flight_route(world.now).generation)
    assert world.channel.action is None and world.channel.sent_count==0


@pytest.mark.parametrize('lookup',['landing_route_key','landing_confirmation_key','landing_confirmation_status'])
def test_landing_lookup_uses_current_clock_after_owner_advanced_while_caller_waited(lookup):
    world=LandingWorld();world.feed()
    route_key=world.backend.landing_route_key(1.)
    confirmation_key=world.backend.landing_confirmation_key(1.)
    supplied_now=1.05
    # Deterministic equivalent of owner projection completing before the
    # waiting API thread finally acquires channel.lock.
    world.now=1.1;world.feed()
    result=getattr(world.backend,lookup)(supplied_now)
    if lookup=='landing_route_key':assert result==route_key
    elif lookup=='landing_confirmation_key':assert result==confirmation_key
    else:assert result['evidence_available'] and result['received_at']==1.1
    assert world.backend.landing_route_key(1.1)==route_key


@pytest.mark.parametrize('lookup',['landing_route_key','landing_confirmation_key','landing_confirmation_status'])
def test_landing_lookup_does_not_mask_actual_future_evidence_when_local_clock_did_not_advance(lookup):
    world=LandingWorld();world.feed();world.now=1.1;world.feed()
    world.now=1.05
    if lookup=='landing_confirmation_status':
        assert not getattr(world.backend,lookup)(1.05)['evidence_available']
    else:
        with pytest.raises(ValueError):getattr(world.backend,lookup)(1.05)


@pytest.mark.parametrize('lookup',['landing_route_key','landing_confirmation_key','landing_confirmation_status'])
@pytest.mark.parametrize('bad',[True,None,float('nan'),float('inf')])
def test_landing_lookup_rejects_invalid_supplied_clock_before_clamping(lookup,bad):
    world=LandingWorld();world.feed()
    with pytest.raises(ValueError,match='time'):
        getattr(world.backend,lookup)(bad)


def test_confirmation_send_deadline_cannot_survive_waiting_for_channel_lock():
    world=LandingWorld();world.feed();_,ident=world.enqueue()
    request=world.channel.next_packet(world.now,world.sequence.next)
    captured_send_time=1.1
    world.now=1.21;world.feed()
    assert not world.channel.can_send(request.seq,captured_send_time)
    assert world.backend.action_receipt(ident)['state']=='expired'
