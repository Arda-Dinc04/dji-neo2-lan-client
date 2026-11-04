"""Owner-thread flight mailbox, without devices, sockets or aircraft effects."""
from dataclasses import replace

import pytest

from neo2_control.duml import DumlPacket, DumlSequence
from neo2_control.flight import FlightAction
from neo2_control.flight_channel import FlightChannel


@pytest.fixture
def rig():
    now = [0.0]
    channel = FlightChannel(clock=lambda: now[0])
    sequence = DumlSequence(current=0x8000)
    channel.set_ready(True, 0)
    return channel, now, sequence.next


def ack(request, **changes):
    packet = DumlPacket(request.receiver, request.sender, request.seq, 0xc0,
                        request.cmd_set, request.cmd_id, b'\0')
    return replace(packet, **changes)


@pytest.mark.parametrize('action', [1, True, 'land', FlightAction.FORCE_LANDING, FlightAction.RETURN_HOME])
def test_only_named_takeoff_and_land_can_enter_channel(rig, action):
    channel, _, _ = rig
    with pytest.raises(ValueError, match='unsupported'):
        channel.submit_action(action, expires_at=.2)
    assert channel.snapshot()['action'] is None


@pytest.mark.parametrize('expiry', [0, -.1, .250001, True, float('nan'), float('inf')])
def test_action_and_control_require_explicit_short_validity(rig, expiry):
    channel, _, _ = rig
    for operation in (lambda: channel.submit_action(FlightAction.TAKEOFF, expires_at=expiry),
                      lambda: channel.submit_control([0]*4, expires_at=expiry)):
        with pytest.raises(ValueError, match='deadline'):
            operation()


def test_route_readiness_does_not_grant_aircraft_or_operator_authority(rig):
    channel, _, _ = rig
    status = channel.snapshot()
    assert status['route_ready']
    assert not status['flight_ready'] and not status['supervisor_authorization_supplied']
    assert not status['execution_confirmed']


def test_stale_control_cancels_unsent_takeoff_but_retains_neutral_land_route(rig):
    channel,now,sequence=rig
    takeoff=channel.submit_action(FlightAction.TAKEOFF,expires_at=.2)
    old=channel.next_packet(0,sequence)
    now[0]=.01;channel.set_control_fresh(False,.01)
    assert channel.receipt(takeoff['intent_id'])['state']=='cancelled'
    assert not channel.can_send(old.seq,.01)
    assert channel.snapshot()['route_ready'] and not channel.snapshot()['ordinary_control_ready']
    with pytest.raises(RuntimeError,match='not_ready'):channel.submit_action(FlightAction.TAKEOFF,expires_at=.2)
    with pytest.raises(RuntimeError,match='not_ready'):channel.submit_control([0]*4,expires_at=.2)
    landing=channel.submit_landing(expires_at=.2)
    neutral=channel.next_packet(.01,sequence)
    assert (neutral.cmd_set,neutral.cmd_id)==(1,0xa) and channel.can_send(neutral.seq,.01)
    channel.sent(neutral.seq,.01)
    land=channel.next_packet(.02,sequence)
    assert land.payload==b'\x02' and channel.can_send(land.seq,.02)
    channel.sent(land.seq,.02);channel.receive(ack(land),.03)
    assert channel.receipt(landing['intent_id'])['state']=='acknowledged'
    now[0]=.04;channel.set_control_fresh(True,.04)
    assert channel.next_packet(.04,sequence) is None
    assert channel.receipt(takeoff['intent_id'])['state']=='cancelled'


def test_stale_control_keeps_an_emergency_landing_already_scheduled(rig):
    channel,now,sequence=rig
    channel.submit_landing(expires_at=.2);neutral=channel.next_packet(0,sequence)
    now[0]=.01;channel.set_control_fresh(False,.01)
    assert channel.can_send(neutral.seq,.01)
    channel.sent(neutral.seq,.01)
    land=channel.next_packet(.02,sequence)
    assert channel.can_send(land.seq,.02)


def test_ack_requires_actual_send_and_does_not_prove_execution(rig):
    channel, now, sequence = rig
    intent = channel.submit_action(FlightAction.TAKEOFF, expires_at=.2)
    request = channel.next_packet(0, sequence)
    channel.receive(ack(request), .01)
    assert channel.snapshot()['action']['state'] == 'scheduled'
    assert channel.can_send(request.seq, .02)
    channel.sent(request.seq, .02)
    channel.receive(ack(request), .03)
    result = channel.snapshot()['history'][-1]
    assert result['id'] == intent['intent_id'] and result['state'] == 'acknowledged'
    assert result['sent_at'] == .02 and result['acknowledged_at'] == .03
    assert result['status_code'] == 0 and not result['execution_confirmed']


@pytest.mark.parametrize('changes', [
    {'seq': 4}, {'sender': 4}, {'receiver': 4}, {'cmd_set': 4}, {'cmd_id': 4},
    {'cmd_type': 0x40}, {'cmd_type': 0xc1}, {'version': 2}, {'payload': b''},
])
def test_unmatched_or_encrypted_response_keeps_action_pending(rig, changes):
    channel, _, sequence = rig
    channel.submit_action(FlightAction.TAKEOFF, expires_at=.2)
    request = channel.next_packet(0, sequence)
    channel.sent(request.seq, .01)
    channel.receive(ack(request, **changes), .02)
    assert channel.snapshot()['action']['state'] == 'sent'
    assert channel.snapshot()['ignored_responses'] == 1


def test_landing_supersedes_scheduled_takeoff_and_control_before_socket_send(rig):
    channel, _, sequence = rig
    channel.submit_action(FlightAction.TAKEOFF, expires_at=.2)
    stale = channel.next_packet(0, sequence)
    channel.submit_control([10, 0, 0, 0], expires_at=.2)
    channel.submit_action(FlightAction.LAND, expires_at=.2)
    assert not channel.can_send(stale.seq, .01)
    assert channel.landing_pending(.01)
    request = channel.next_packet(.01, sequence)
    assert request.payload == bytes([FlightAction.LAND]) and request.seq != stale.seq
    assert channel.snapshot()['control'] is None
    with pytest.raises(RuntimeError, match='landing'):
        channel.submit_control([0]*4, expires_at=.2)


def test_landing_replaces_inflight_action_without_accepting_its_late_reply(rig):
    channel, _, sequence = rig
    channel.submit_action(FlightAction.TAKEOFF, expires_at=.2)
    first = channel.next_packet(0, sequence)
    channel.sent(first.seq, .01)
    channel.submit_action(FlightAction.LAND, expires_at=.2)
    second = channel.next_packet(.02, sequence)
    channel.sent(second.seq, .03)
    channel.receive(ack(first), .04)
    assert channel.snapshot()['action']['action'] == 'land'
    channel.receive(ack(second, payload=b'\x03'), .05)
    assert channel.snapshot()['history'][-1]['state'] == 'rejected'


def test_newest_control_supersedes_a_scheduled_value_without_replaying_old_one(rig):
    channel, _, sequence = rig
    channel.submit_control([20, 0, 0, 0], expires_at=.2)
    old = channel.next_packet(0, sequence)
    new = channel.submit_control([30, 0, 0, 0], expires_at=.2)
    assert not channel.can_send(old.seq, .01)
    request = channel.next_packet(.01, sequence)
    assert request.payload != old.payload and channel.can_send(request.seq, .01)
    channel.sent(request.seq, .01)
    status = channel.snapshot()
    assert status['history'][-1]['id'] == new['intent_id']
    assert status['history'][-1]['state'] == 'sent_unconfirmed'
    assert status['control'] is None and channel.next_packet(.02, sequence) is None


def test_control_cadence_begins_at_actual_send_and_has_no_automatic_replay(rig):
    channel, _, sequence = rig
    channel.submit_control([0]*4, expires_at=.2)
    first = channel.next_packet(0, sequence)
    channel.sent(first.seq, .04)
    channel.submit_control([20, 0, 0, 0], expires_at=.2)
    assert channel.next_packet(.089, sequence) is None
    second = channel.next_packet(.09, sequence)
    channel.sent(second.seq, .09)
    assert channel.next_packet(.15, sequence) is None
    assert channel.snapshot()['sent_count'] == 2


def test_expired_scheduled_packet_is_cancelled_before_send_and_no_new_sequence_is_allocated(rig):
    channel, now, sequence = rig
    channel.submit_action(FlightAction.TAKEOFF, expires_at=.2)
    request = channel.next_packet(0, sequence)
    assert not channel.can_send(request.seq, .2)
    assert channel.next_packet(.2, lambda: pytest.fail('allocated for expired intent')) is None
    assert channel.snapshot()['sent_count'] == 0


def test_ack_deadline_begins_at_completed_send_and_is_strict(rig):
    channel, now, sequence = rig
    channel.submit_action(FlightAction.LAND, expires_at=.25)
    request = channel.next_packet(0, sequence)
    channel.sent(request.seq, .24)
    now[0] = 1.24
    assert channel.snapshot()['action']['state'] == 'timed_out'
    channel.receive(ack(request), 1.24)
    assert channel.snapshot()['action'] is None
    assert channel.snapshot()['history'][-1]['state'] == 'timed_out'
    assert channel.next_packet(1.25, lambda: pytest.fail('automatic retry')) is None


@pytest.mark.parametrize('stop', ['revoke', 'route', 'close'])
def test_revocation_and_route_loss_prevent_already_scheduled_send(rig, stop):
    channel, now, sequence = rig
    channel.submit_control([10, 0, 0, 0], expires_at=.2)
    request = channel.next_packet(0, sequence)
    if stop == 'revoke': channel.revoke()
    elif stop == 'route': channel.set_ready(False, .01)
    else: channel.cancel(.01)
    assert not channel.can_send(request.seq, .02)
    assert channel.snapshot()['sent_count'] == 0


def test_snapshot_does_not_advance_owner_clock_or_mutate_pending_intent(rig):
    channel, now, sequence = rig
    channel.submit_control([10, 0, 0, 0], expires_at=.2)
    now[0] = .5
    detached = channel.snapshot()
    assert detached['control']['state'] == 'expired'
    detached['control']['channels'][0] = 200
    assert channel.last_time == 0 and channel.control['channels'][0] == 10
    assert channel.next_packet(.2, sequence) is None


def test_actual_send_after_preflight_deadline_is_reported_without_claiming_it_was_cancelled(rig):
    channel, _, sequence = rig
    channel.submit_action(FlightAction.LAND, expires_at=.2)
    request = channel.next_packet(0, sequence)
    assert channel.can_send(request.seq, .199)
    channel.sent(request.seq, .201)
    status = channel.snapshot()
    assert status['sent_count'] == 1 and status['history'][-1]['sent_after_deadline']
    assert status['history'][-1]['state'] == 'sent_unconfirmed'


def test_channel_history_and_pending_control_are_bounded(rig):
    channel, _, _ = rig
    for value in range(100):
        channel.submit_control([value, 0, 0, 0], expires_at=.2)
    assert len(channel.snapshot()['history']) == 32
    assert channel.snapshot()['control']['channels'][0] == 99


@pytest.mark.parametrize('channels', [[251, 0, 0, 0], [-251, 0, 0, 0], [True, 0, 0, 0], [0]*3])
def test_control_is_typed_and_has_existing_supervisor_local_limit(rig, channels):
    channel, _, _ = rig
    with pytest.raises(ValueError):
        channel.submit_control(channels, expires_at=.2)
