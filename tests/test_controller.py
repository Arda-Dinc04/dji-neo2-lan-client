from dataclasses import replace
import json

import pytest

from neo2_control.controller import (
    ControlLimits, ControllerMode, ControlSupervisor, DecisionJournal, IntentRejected,
)
from neo2_control.duml import DumlPacket
from neo2_control.flight import FlightAction
from neo2_control.simulator import SimulationBackend


class Clock:
    now = 100.0

    def __call__(self):
        return self.now


@pytest.fixture
def factory(tmp_path):
    journals = []

    def make(**limits):
        clock, backend = Clock(), SimulationBackend()
        journal = DecisionJournal(tmp_path / f'controller-{len(journals)}.jsonl')
        journals.append(journal)
        controller = ControlSupervisor(journal, mode='simulate', backend=backend,
                                       clock=clock, limits=ControlLimits(**limits))
        return controller, backend, clock, journal

    yield make
    for journal in journals:
        journal.close()


def advance(controller, clock, seconds):
    target = clock.now + seconds
    while clock.now < target - 1e-9:
        clock.now = min(clock.now + .025, target)
        controller.tick()


def authorize(controller):
    controller.handle('open')
    token = controller.handle('prepare', {'operation': 'arm'})['confirmation_token']
    return controller.handle('arm', {'confirmation_token': token})['lease']


def takeoff(controller, lease):
    token = controller.handle('prepare', {'operation': 'takeoff', 'lease': lease})['confirmation_token']
    controller.handle('takeoff', {'lease': lease, 'confirmation_token': token})
    return token


def airborne(factory, **limits):
    controller, backend, clock, journal = factory(**limits)
    lease = authorize(controller)
    takeoff(controller, lease)
    advance(controller, clock, .4)
    assert controller.state == 'flying'
    return controller, backend, clock, journal, lease


def actions(backend):
    return [packet.payload[0] for _, packet in backend.sent if (packet.cmd_set, packet.cmd_id) == (3, 0x2A)]


def test_complete_session_requires_observed_motion_after_ack_and_local_arm_has_no_packet(factory):
    controller, backend, clock, journal = factory()
    lease = authorize(controller)
    assert controller.state == 'armed' and not backend.sent
    takeoff(controller, lease)
    advance(controller, clock, .075)
    assert controller.state == 'taking_off' and controller.pending is None
    advance(controller, clock, .325)
    assert controller.state == 'flying'
    controller.handle('control', {'lease': lease, 'sequence': 1,
                                 'channels': [50, -50, 0, 0], 'expires_at': clock.now + .2})
    advance(controller, clock, .075)
    controller.handle('land')
    assert controller.state == 'landing'
    advance(controller, clock, .4)
    assert controller.state == 'ready' and not controller.fault_reason
    assert actions(backend) == [FlightAction.TAKEOFF, FlightAction.LAND]
    controller.handle('close')
    assert controller.state == 'disconnected'
    assert journal.path.stat().st_mode & 0o777 == 0o600
    events = [json.loads(line) for line in journal.path.read_text().splitlines()]
    assert any(row['event'] == 'action_response' and not row['execution_confirmed'] for row in events)
    assert [row['sequence'] for row in events] == list(range(1, len(events) + 1))


def test_confirmation_and_lease_tokens_are_single_use_and_not_logged(factory):
    controller, backend, clock, journal = factory()
    lease = authorize(controller)
    token = takeoff(controller, lease)
    with pytest.raises(IntentRejected, match='confirmation_invalid'):
        controller.handle('takeoff', {'lease': lease, 'confirmation_token': token})
    assert actions(backend) == [FlightAction.TAKEOFF]
    for value in (lease, token):
        assert value not in json.dumps(controller.status())
        assert value not in journal.path.read_text()


def test_confirmation_expiry_and_replacement(factory):
    controller, backend, clock, _ = factory(confirmation_seconds=.1)
    controller.handle('open')
    old = controller.handle('prepare', {'operation': 'arm'})['confirmation_token']
    token = controller.handle('prepare', {'operation': 'arm'})['confirmation_token']
    with pytest.raises(IntentRejected, match='confirmation_invalid'):
        controller.handle('arm', {'confirmation_token': old})
    advance(controller, clock, .11)
    with pytest.raises(IntentRejected, match='confirmation_expired'):
        controller.handle('arm', {'confirmation_token': token})
    assert not backend.sent


def test_unknown_ground_state_cannot_be_inferred_from_not_flying(factory, monkeypatch):
    controller, backend, clock, _ = factory()
    original = backend.observation
    monkeypatch.setattr(backend, 'observation', lambda: replace(original(), grounded=None))
    controller.handle('open')
    assert controller.state == 'not_ready'
    with pytest.raises(IntentRejected, match='invalid_state'):
        controller.handle('prepare', {'operation': 'arm'})
    assert not backend.sent


def test_observation_published_during_tick_is_compared_to_time_after_the_read(factory,monkeypatch):
    controller,backend,clock,journal=factory()
    original=backend.observation
    def observation():
        # A real UDP owner can receive/publish just after tick's initial clock.
        clock.now+=.001
        return replace(original(),received_at=clock.now)
    monkeypatch.setattr(backend,'observation',observation)
    controller.handle('open')
    assert controller.state=='ready'
    token=controller.handle('prepare',{'operation':'arm'})['confirmation_token']
    controller.handle('arm',{'confirmation_token':token})
    times=[r['time'] for r in map(json.loads,journal.path.read_text().splitlines())]
    assert times==sorted(times)


@pytest.mark.parametrize('field,value,reason', [
    ('battery_percent', None, 'battery_unavailable_or_low'),
    ('battery_percent', 19, 'battery_unavailable_or_low'),
    ('position_valid', False, 'position_invalid'),
    ('altitude_override', 3.1, 'altitude_m_invalid_or_limit'),
    ('distance_override', float('nan'), 'distance_m_invalid_or_limit'),
])
def test_readiness_loss_in_flight_revokes_control_and_requests_one_landing(factory, field, value, reason):
    controller, backend, clock, _, lease = airborne(factory)
    setattr(backend, field, value)
    advance(controller, clock, .075)
    assert controller.fault_reason == reason and controller.lease is None
    assert actions(backend) == [FlightAction.TAKEOFF, FlightAction.LAND]
    advance(controller, clock, .8)
    assert controller.state == 'fault'
    assert actions(backend).count(FlightAction.LAND) == 1
    with pytest.raises(IntentRejected):
        controller.handle('keepalive', {'lease': lease})


def test_lost_operator_is_detected_without_new_intents(factory):
    controller, backend, clock, _, _ = airborne(factory)
    advance(controller, clock, .2)
    assert controller.fault_reason == 'operator_lease_expired'
    assert actions(backend) == [FlightAction.TAKEOFF, FlightAction.LAND]
    advance(controller, clock, .4)
    assert controller.state == 'fault' and controller.status()['telemetry']['grounded']
    controller.handle('reset')
    assert controller.state == 'ready'


def test_expired_control_and_clock_jump_never_replay_a_burst(factory):
    controller, backend, clock, _, lease = airborne(factory)
    controller.handle('control', {'lease': lease, 'sequence': 10,
                                 'expires_at': clock.now + .1, 'channels': [100, 0, 0, 0]})
    sent = len(backend.sent)
    clock.now += .2
    controller.tick()
    assert controller.fault_reason == 'control_expired'
    assert len(backend.sent) == sent + 2  # neutral + landing, no catch-up controls
    assert actions(backend)[-1] == FlightAction.LAND


def test_control_rate_replay_bounds_and_deadlines(factory):
    controller, backend, clock, _, lease = airborne(factory)
    payload = {'lease': lease, 'sequence': 1, 'expires_at': clock.now + .25, 'channels': [0, 0, 0, 0]}
    controller.handle('control', payload)
    with pytest.raises(IntentRejected, match='rate_limited'):
        controller.handle('control', {**payload, 'sequence': 2})
    advance(controller, clock, .05)
    controller.handle('control', {**payload, 'sequence': 2})
    with pytest.raises(IntentRejected, match='sequence_replayed'):
        controller.handle('control', payload)
    for changes, error in [({'sequence': True}, 'sequence'),
                           ({'expires_at': clock.now}, 'deadline'),
                           ({'expires_at': float('inf')}, 'deadline'),
                           ({'expires_at': clock.now + 1}, 'deadline'),
                           ({'channels': [251, 0, 0, 0]}, 'channels'),
                           ({'channels': [True, 0, 0, 0]}, 'channels')]:
        with pytest.raises(IntentRejected, match=error):
            controller.handle('control', {**payload, 'sequence': 3, **changes})
    joystick_times = [now for now, packet in backend.sent if packet.cmd_id == 10]
    assert all(b - a >= .05 - 1e-9 for a, b in zip(joystick_times, joystick_times[1:]))


def test_ack_loss_never_retries_takeoff_and_landing_loss_remains_uncertain(factory):
    controller, backend, clock, _ = factory()
    backend.ack_enabled = False
    lease = authorize(controller)
    takeoff(controller, lease)
    advance(controller, clock, 1.1)
    assert controller.state == 'fault' and controller.fault_reason == 'action_ack_timeout'
    assert actions(backend) == [FlightAction.TAKEOFF, FlightAction.LAND]
    assert not controller.landing_confirmed and controller.flight_started
    with pytest.raises(IntentRejected):
        controller.handle('reset')
    with pytest.raises(IntentRejected):
        controller.handle('close')


def test_rejected_landing_is_not_retried_and_unknown_flight_cannot_be_closed(factory):
    controller, backend, clock, _, lease = airborne(factory)
    backend.action_status = 1
    controller.handle('land')
    advance(controller, clock, .1)
    assert controller.state == 'fault' and controller.fault_reason == 'landing_rejected'
    assert actions(backend) == [FlightAction.TAKEOFF, FlightAction.LAND]
    with pytest.raises(IntentRejected):
        controller.handle('close')
    other, backend, clock, _ = factory()
    backend.ack_delay = .3
    lease = authorize(other)
    takeoff(other, lease)
    advance(other, clock, .1)
    assert other.last_sample.grounded
    with pytest.raises(IntentRejected):
        other.handle('close')


def test_unrelated_response_does_not_release_pending_action(factory, monkeypatch):
    controller, backend, clock, _ = factory()
    original = backend.advance
    monkeypatch.setattr(backend, 'advance', lambda now: [replace(packet, seq=(packet.seq + 1) & 0xFFFF) for packet in original(now)])
    lease = authorize(controller)
    takeoff(controller, lease)
    advance(controller, clock, .4)
    assert controller.state == 'taking_off' and controller.pending is not None
    advance(controller, clock, .2)
    assert controller.fault_reason == 'action_ack_timeout'


def test_successful_ack_without_motion_is_not_flying(factory):
    controller, backend, clock, _ = factory(transition_seconds=.45)
    backend.motion_enabled = False
    lease = authorize(controller)
    takeoff(controller, lease)
    advance(controller, clock, .475)
    assert controller.fault_reason == 'takeoff_unconfirmed'
    assert controller.state != 'flying'


def test_telemetry_staleness_and_link_loss_are_distinct(factory):
    controller, backend, clock, _, lease = airborne(factory, telemetry_age=.1)
    backend.telemetry_enabled = False
    controller.handle('keepalive', {'lease': lease})
    advance(controller, clock, .15)
    assert controller.fault_reason == 'telemetry_stale'
    assert actions(backend)[-1] == FlightAction.LAND
    other, backend, clock, _, _ = airborne(factory)
    backend.connected = False
    other.tick()
    assert other.state == 'fault' and other.fault_reason == 'link_unavailable'
    assert actions(backend) == [FlightAction.TAKEOFF]


def test_flight_duration_is_bounded_even_when_operator_renews_lease(factory):
    controller, backend, clock, _, lease = airborne(factory, maximum_flight_seconds=.6)
    advance(controller, clock, .05)
    controller.handle('keepalive', {'lease': lease})
    advance(controller, clock, .2)
    assert controller.fault_reason == 'maximum_flight_time'


def test_journal_failure_during_flight_triggers_best_effort_landing_and_latches_fault(factory):
    controller, backend, clock, journal, lease = airborne(factory)
    journal.close()
    with pytest.raises(IntentRejected, match='journal_unavailable'):
        controller.handle('keepalive', {'lease': lease})
    assert not controller.journal_healthy
    assert actions(backend) == [FlightAction.TAKEOFF, FlightAction.LAND]
    advance(controller, clock, .5)
    assert controller.state == 'fault'


def test_restart_invalidates_old_authority_and_existing_journal_is_preserved(factory):
    controller, backend, clock, journal = factory()
    lease = authorize(controller)
    challenge = controller.handle('prepare', {'operation': 'takeoff', 'lease': lease})['confirmation_token']
    restarted, backend, _, _ = factory()
    restarted.handle('open')
    with pytest.raises(IntentRejected):
        restarted.handle('takeoff', {'lease': lease, 'confirmation_token': challenge})
    assert not backend.sent
    before = journal.path.read_bytes()
    with pytest.raises(FileExistsError):
        DecisionJournal(journal.path)
    assert journal.path.read_bytes() == before


def test_observe_mode_and_live_interlock(factory):
    _, backend, clock, journal = factory()
    controller = ControlSupervisor(journal, clock=clock)
    with pytest.raises(IntentRejected, match='observe_mode'):
        controller.handle('open')
    assert not backend.sent
    with pytest.raises(ValueError, match='not verified'):
        ControlSupervisor(journal, mode='flight')
    with pytest.raises(ValueError, match='synthetic'):
        ControlSupervisor(journal, mode='simulate')


def test_malformed_intents_and_unicode_secrets_do_not_fault_a_healthy_controller(factory):
    controller, _, _, _ = factory()
    lease = authorize(controller)
    for action, parameters in [(None, {}), ([], {}), ('keepalive', {'lease': 'é'}),
                                ('land', {'extra': 1}), ('land', []), ('arm', {'confirmation_token': 'é'})]:
        with pytest.raises(IntentRejected):
            controller.handle(action, parameters)
    assert controller.state == 'armed' and controller.fault_reason is None


def test_clock_must_be_finite_and_monotonic(factory):
    controller, _, clock, _ = factory()
    clock.now -= 1
    with pytest.raises(ValueError, match='monotonic'):
        controller.tick()


def test_stopping_is_latched_and_cannot_be_reset_by_a_late_http_intent(factory):
    controller, backend, clock, _ = factory()
    authorize(controller)
    controller.stop()
    with pytest.raises(IntentRejected, match='controller_stopping'):
        controller.handle('reset')
    assert not backend.sent


def test_backend_failure_keeps_timeout_checks_running(factory, monkeypatch):
    controller, backend, clock, _, _ = airborne(factory, transition_seconds=.5)
    def failure(now):
        raise OSError('synthetic receiver stopped')
    monkeypatch.setattr(backend, 'advance', failure)
    advance(controller, clock, .6)
    assert controller.state == 'fault' and controller.fault_reason == 'backend_failed'
    assert actions(backend) == [1, 2]


def test_unknown_or_inconsistent_flight_flags_cannot_keep_continuous_control(factory, monkeypatch):
    controller, backend, clock, _, _ = airborne(factory)
    original = backend.observation
    monkeypatch.setattr(backend, 'observation', lambda: replace(original(), flying=None))
    controller.tick()
    assert controller.fault_reason == 'flight_state_unknown'
    other, backend, clock, _, _ = airborne(factory)
    original2 = backend.observation
    monkeypatch.setattr(backend, 'observation', lambda: replace(original2(), grounded=True))
    other.tick()
    assert other.fault_reason == 'flight_state_inconsistent'


def test_nonfinite_numeric_observation_is_invalid_and_json_serializable(factory):
    controller, backend, clock, _, _ = airborne(factory)
    backend.altitude_override = float('nan')
    advance(controller, clock, .05)
    status = controller.status()
    assert status['telemetry']['altitude_m'] is None
    assert status['telemetry']['invalid_fields'] == ['altitude_m']
    json.dumps(status, allow_nan=False)


def test_explicit_landing_recovery_preserves_fault_and_matches_only_new_ack(factory, monkeypatch):
    controller, backend, clock, journal, lease = airborne(factory, transition_seconds=.6)
    backend.action_status = 1
    controller.handle('land')
    old_request = backend.sent[-1][1]
    advance(controller, clock, .1)
    assert controller.state == 'fault' and controller.fault_reason == 'landing_rejected'
    backend.action_status = 0
    backend.motion_enabled = False  # ACK0 alone must not claim a landing.
    controller.handle('land')
    request = backend.sent[-1][1]
    assert request.seq != old_request.seq and request.payload == b'\2'
    assert controller.state == 'landing' and controller.flight_started
    assert controller.fault_reason == 'landing_rejected'
    assert controller.lease is controller.control is controller.challenge is None
    with pytest.raises(IntentRejected, match='invalid_state'):
        controller.handle('prepare', {'operation': 'arm'})
    with pytest.raises(IntentRejected, match='operator_lease_invalid'):
        controller.handle('control', {'lease': lease, 'sequence': 1,
            'channels': [50, 0, 0, 0], 'expires_at': clock.now + .2})
    original = backend.advance
    old_ack = DumlPacket(old_request.receiver, old_request.sender, old_request.seq,
                         0x80, 3, 0x2a, b'\0')
    monkeypatch.setattr(backend, 'advance', lambda now: [old_ack, *original(now)])
    advance(controller, clock, .025)
    assert controller.pending[0].seq == request.seq and not controller.action_acknowledged
    monkeypatch.setattr(backend, 'advance', original)
    advance(controller, clock, .05)
    assert controller.pending is None and controller.action_acknowledged
    assert controller.flight_started and not controller.landing_confirmed
    advance(controller, clock, .7)
    assert controller.state == 'fault' and controller.fault_reason == 'landing_rejected'
    assert controller.flight_started and not controller.landing_confirmed
    assert actions(backend) == [FlightAction.TAKEOFF, FlightAction.LAND, FlightAction.LAND]
    events = [json.loads(line) for line in journal.path.read_text().splitlines()]
    assert sum(e['event'] == 'landing_recovery_requested' for e in events) == 1


def test_repeated_land_while_ack_is_pending_does_not_replace_or_duplicate_transaction(factory):
    controller, backend, clock, _, _ = airborne(factory)
    backend.motion_enabled = False
    backend.ack_delay = .2
    controller.handle('land')
    pending = controller.pending
    transition = controller.transition_started
    for _ in range(4):
        result = controller.handle('land')
        assert result['landing_already_pending'] is True
        assert controller.pending == pending and controller.transition_started == transition
        advance(controller, clock, .025)
    assert actions(backend) == [FlightAction.TAKEOFF, FlightAction.LAND]
    advance(controller, clock, .125)
    assert controller.pending is None and controller.action_acknowledged
    controller.handle('land')
    assert controller.pending[0].seq != pending[0].seq
    assert actions(backend) == [FlightAction.TAKEOFF, FlightAction.LAND, FlightAction.LAND]


def test_explicit_land_after_missing_ack_uses_retained_link_despite_stale_telemetry(factory):
    controller, backend, clock, _, _ = airborne(factory)
    backend.ack_enabled = backend.motion_enabled = False
    controller.handle('land')
    first_sequence = controller.pending[0].seq
    advance(controller, clock, .6)
    assert controller.state == 'fault' and controller.fault_reason == 'action_ack_timeout'
    backend.telemetry_enabled = False
    advance(controller, clock, .6)
    assert controller.status()['telemetry']['valid'] is False
    controller.handle('land')
    assert controller.state == 'landing' and controller.pending[0].seq != first_sequence
    assert controller.fault_reason == 'action_ack_timeout' and controller.flight_started
    advance(controller, clock, 1.)
    assert controller.state == 'fault' and controller.flight_started
    assert not controller.landing_confirmed
    assert actions(backend) == [FlightAction.TAKEOFF, FlightAction.LAND, FlightAction.LAND]
    assert backend.channels == (1024,) * 4


@pytest.mark.parametrize('unavailable', ['grounded', 'disconnected'])
def test_recovery_land_cannot_use_confirmed_ground_or_an_unavailable_link(factory, unavailable):
    controller, backend, clock, _, _ = airborne(factory)
    backend.action_status = 1
    controller.handle('land')
    advance(controller, clock, .1)
    if unavailable == 'grounded':
        backend.phase = 'grounded'
        advance(controller, clock, .05)
        expected = 'aircraft_already_grounded'
    else:
        backend.connected = False
        expected = 'link_unavailable'
    before = list(backend.sent)
    with pytest.raises(IntentRejected, match=expected):
        controller.handle('land')
    assert list(backend.sent) == before
    assert controller.state == 'fault' and controller.flight_started


def test_fault_before_takeoff_does_not_enable_recovery_land(factory):
    controller, backend, clock, _ = factory()
    authorize(controller)
    backend.position_valid = False
    advance(controller, clock, .05)
    assert controller.state == 'fault' and not controller.flight_started
    with pytest.raises(IntentRejected, match='invalid_state'):
        controller.handle('land')
    assert not backend.sent


@pytest.mark.parametrize('guard', ['stopping', 'journal'])
def test_landing_recovery_does_not_reopen_stopped_or_unaudited_owner(factory, monkeypatch, guard):
    controller, backend, clock, journal, _ = airborne(factory)
    backend.action_status = 1
    controller.handle('land')
    advance(controller, clock, .1)
    if guard == 'stopping':
        controller.stop()
        expected = 'controller_stopping'
    else:
        def unavailable(event):
            raise OSError('journal unavailable')
        monkeypatch.setattr(journal, 'record', unavailable)
        expected = 'journal_unavailable'
    before = list(backend.sent)
    with pytest.raises(IntentRejected, match=expected):
        controller.handle('land')
    assert list(backend.sent) == before
    assert controller.state == 'fault' and controller.flight_started
