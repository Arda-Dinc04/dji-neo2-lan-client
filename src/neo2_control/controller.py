"""Local control state machine with explicit evidence and expiring authority.

The socket adapter consumes typed intents and reports the owner's exact
request/response bytes. Aircraft observations remain a separate requirement;
the network service still exposes only observation and simulation modes.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, replace
from enum import Enum
import hmac
import json
import math
import os
from pathlib import Path
import secrets
import threading
import time
from typing import Protocol

from .command_ingress import CommandIngress
from .duml import DumlPacket, DumlSequence
from .flight import FlightAction, flight_action, parse_action_response, virtual_joystick


class ControllerMode(str, Enum):
    OBSERVE = 'observe'
    SIMULATE = 'simulate'
    FLIGHT = 'flight'


class IntentRejected(ValueError):
    pass


@dataclass(frozen=True)
class ControlLimits:
    telemetry_age: float = 0.5
    lease_seconds: float = 0.5
    confirmation_seconds: float = 5.0
    armed_seconds: float = 5.0
    maximum_flight_seconds: float = 30.0
    acknowledgement_seconds: float = 0.5
    transition_seconds: float = 5.0
    control_seconds: float = 0.25
    control_interval: float = 0.05
    minimum_battery_percent: float = 20.0
    maximum_altitude_m: float = 3.0
    maximum_distance_m: float = 5.0
    maximum_channel: int = 250
    battery_age: float = 3.0

    def __post_init__(self):
        for name, value in asdict(self).items():
            if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if type(self.maximum_channel) is not int or self.maximum_channel > 1000:
            raise ValueError('maximum_channel must be an integer in 1..1000')
        if self.minimum_battery_percent > 100:
            raise ValueError('minimum battery must not exceed 100 percent')
        if self.control_interval < .05 or self.control_seconds > .25:
            raise ValueError('control cadence must be at most 20 Hz and validity at most 250 ms')


@dataclass(frozen=True)
class FlightObservation:
    received_at: float
    grounded: bool | None
    flying: bool | None
    motors_on: bool | None
    battery_percent: float | None
    position_valid: bool
    altitude_m: float | None
    distance_m: float | None
    source: str
    synthetic: bool
    control_ready: bool | None = None
    ground_source: str | None = None
    motion_received_at: float | None = None
    battery_received_at: float | None = None
    control_received_at: float | None = None
    yaw_deg: float | None = None

    def __post_init__(self):
        if type(self.received_at) not in (int, float) or not math.isfinite(self.received_at):
            raise ValueError('observation receipt time must be finite')
        if any(value is not None and type(value) is not bool for value in (self.grounded, self.flying, self.motors_on)):
            raise ValueError('observation state flags must be booleans or unknown')
        if type(self.position_valid) is not bool or type(self.synthetic) is not bool:
            raise ValueError('observation validity and provenance must be boolean')
        if self.control_ready is not None and type(self.control_ready) is not bool:
            raise ValueError('control readiness must be boolean or unknown')
        if self.yaw_deg is not None and (type(self.yaw_deg) not in (int,float)
                or not math.isfinite(self.yaw_deg) or not -180 <= self.yaw_deg <= 180):
            raise ValueError('yaw must be finite degrees in -180..180 or unknown')
        if self.ground_source is not None and (not isinstance(self.ground_source,str) or not self.ground_source or len(self.ground_source)>64):
            raise ValueError('ground observation provenance must be named or unknown')
        for stamp in (self.motion_received_at,self.battery_received_at,self.control_received_at):
            if stamp is not None and (type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp < self.received_at):
                raise ValueError('component receipt time must be finite and follow the oldest receipt')
        if not isinstance(self.source, str) or not self.source or len(self.source) > 128:
            raise ValueError('observation source must be named')


class ControlBackend(Protocol):
    synthetic: bool
    connected: bool

    def open(self, now: float) -> None: ...
    def close(self, now: float) -> None: ...
    def advance(self, now: float) -> list[DumlPacket]: ...
    def observation(self) -> FlightObservation | None: ...
    def send(self, packet: DumlPacket, now: float) -> None: ...


class DecisionJournal:
    """Exclusive new private file; append and flush each record before effects."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.path, os.O_CREAT | os.O_EXCL | os.O_APPEND | os.O_WRONLY
                     | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        self.stream = os.fdopen(fd, 'w', encoding='utf-8')

    def record(self, event: dict) -> None:
        self.stream.write(json.dumps(event, allow_nan=False, separators=(',', ':')) + '\n')
        self.stream.flush()
        os.fsync(self.stream.fileno())

    def close(self):
        self.stream.close()


class ControlSupervisor:
    """Serialized intents, observations and watchdog decisions on a local clock."""

    AIR_STATES = frozenset(('taking_off', 'flying', 'landing'))
    LANDING_ACTIONS = (FlightAction.LAND, FlightAction.CONFIRM_LANDING)
    FIELDS = {
        'open': set(), 'reconnect':set(), 'close': set(), 'reset': set(), 'disarm': {'lease'},
        'prepare': {'operation', 'lease'}, 'arm': {'confirmation_token'},
        'confirm_ground': {'confirmation_token'},
        'confirm_landing': {'confirmation_token'},
        'takeoff': {'confirmation_token', 'lease'}, 'land': set(),
        'keepalive': {'lease'},
        'control': {'lease', 'sequence', 'expires_at', 'channels'},
    }

    def __init__(self, journal: DecisionJournal, *, mode=ControllerMode.OBSERVE,
                 backend: ControlBackend | None = None, limits: ControlLimits | None = None,
                 clock=time.monotonic, vertical_trial=False, yaw_trial=False, incident_store=None,
                 acquisition_trial=False):
        if type(vertical_trial) is not bool:
            raise ValueError('vertical_trial_must_be_boolean')
        if type(yaw_trial) is not bool:raise ValueError('yaw_trial_must_be_boolean')
        if vertical_trial and yaw_trial:raise ValueError('flight_trial_profiles_are_exclusive')
        if type(acquisition_trial) is not bool or acquisition_trial and not yaw_trial:
            raise ValueError('acquisition_trial_requires_yaw_profile')
        self.mode = ControllerMode(mode)
        from .control_transport import FlightChannelBackend
        self.semantic_backend = isinstance(backend, FlightChannelBackend)
        if self.mode is ControllerMode.FLIGHT and (not self.semantic_backend or backend.synthetic):
            raise ValueError('live flight adapter is not verified')
        if self.mode is ControllerMode.OBSERVE and backend is not None:
            raise ValueError('observe mode cannot own a command backend')
        if self.mode is ControllerMode.SIMULATE and (backend is None or not backend.synthetic):
            raise ValueError('simulate mode requires an explicitly synthetic backend')
        if vertical_trial and self.mode is ControllerMode.OBSERVE:
            raise ValueError('vertical_trial_requires_control_backend')
        if yaw_trial and self.mode is ControllerMode.OBSERVE:
            raise ValueError('yaw_trial_requires_control_backend')
        if (self.semantic_backend
                and getattr(backend.channel, 'vertical_trial', False) is not vertical_trial):
            raise ValueError('vertical_trial_channel_profile_mismatch')
        if self.semantic_backend and getattr(backend.channel,'yaw_trial',False) is not yaw_trial:
            raise ValueError('yaw_trial_channel_profile_mismatch')
        if self.semantic_backend and getattr(backend.channel,'acquisition_trial',False) is not acquisition_trial:
            raise ValueError('acquisition_trial_channel_profile_mismatch')
        self.backend, self.journal, self.clock = backend, journal, clock
        if incident_store is not None and not self.semantic_backend:
            raise ValueError('flight_incident_requires_owned_observations')
        self.incident_store = incident_store
        self.incident = None if incident_store is None else incident_store.active
        self.incident_peer = self.incident
        self.incident_error = None
        self.recovery_required = self.incident is not None
        self.vertical_trial = vertical_trial
        self.yaw_trial = yaw_trial
        self.acquisition_trial = acquisition_trial
        self.limits = limits or ControlLimits()
        if (self.vertical_trial or self.yaw_trial) and limits is None:
            self.limits = replace(self.limits, acknowledgement_seconds=1.)
        if self.vertical_trial:
            self.limits = replace(self.limits,
                maximum_flight_seconds=min(self.limits.maximum_flight_seconds, 10.),
                maximum_altitude_m=min(self.limits.maximum_altitude_m, 1.5))
        if self.yaw_trial:
            from .flight import ACQUISITION_ALTITUDE_LIMIT
            yaw_cap = self.backend.channel.yaw_channel_limit if self.semantic_backend else 200
            self.limits = replace(self.limits,
                maximum_flight_seconds=(60. if self.acquisition_trial and limits is None else
                    min(self.limits.maximum_flight_seconds,60. if self.acquisition_trial else 30.)),
                maximum_altitude_m=min(self.limits.maximum_altitude_m,
                    ACQUISITION_ALTITUDE_LIMIT if self.acquisition_trial else 1.5),
                maximum_channel=min(self.limits.maximum_channel if limits is not None else yaw_cap,yaw_cap),
                acknowledgement_seconds=min(self.limits.acknowledgement_seconds,1.))
        if self.semantic_backend and not self.yaw_trial and self.limits.maximum_channel > 250:
            raise ValueError('flight channel local control limit is250')
        self.lock = threading.RLock()
        self.state, self.fault_reason = 'disconnected', None
        self.journal_healthy = True
        self.events = deque(maxlen=50)
        self.event_number = 0
        self.last_now = None
        self.sequence = DumlSequence()
        self.challenge = self.lease = self.control = self.pending = None
        self.pending_action = self.pending_intent_id = self.pending_sent_at = None
        self.ground_challenge_key = None
        self.landing_challenge_key = self.land_route_key = None
        self.landing_episode = 0
        self.land_requested_at = None
        self.land_acknowledged = False
        self.lease_deadline = self.armed_deadline = self.flight_deadline = None
        self.transition_started = None
        self.last_control_sequence = -1
        self.last_control_intent = self.next_control_send = None
        self.last_sample = None
        self.sample_aircraft_context = None
        self.flight_started = self.landing_confirmed = self.action_acknowledged = False
        if self.recovery_required:
            self.flight_started = True
            self.fault_reason = 'unresolved_previous_flight'
        self.stopping = False
        self.observation_source = None
        self._event(self._now(), 'controller_started', vertical_trial=self.vertical_trial,
                    yaw_trial=self.yaw_trial)

    def _now(self):
        now = self.clock()
        if type(now) not in (float, int) or not math.isfinite(now):
            raise ValueError('controller clock must be finite')
        if self.last_now is not None and now < self.last_now:
            raise ValueError('controller clock must be monotonic')
        self.last_now = now
        return now

    def _event(self, now, event, **values):
        self.event_number += 1
        record = {'sequence': self.event_number, 'time': now, 'event': event,
                  'mode': self.mode.value, 'state': self.state, **values}
        self.events.append(record)
        try:
            self.journal.record(record)
        except (OSError, ValueError):
            self.journal_healthy = False

    def _transition(self, now, state, reason):
        previous, self.state = self.state, state
        if previous != state:
            self._event(now, 'state_changed', previous=previous, reason=reason)

    def _revoke(self):
        self.challenge = self.lease = self.control = None
        self.landing_challenge_key = None
        self.lease_deadline = self.armed_deadline = None
        self.next_control_send = None

    def _incident_context(self, now, *, retained=False):
        try:context = self.backend.aircraft_context(now,retained=retained)
        except ValueError as error:raise IntentRejected(str(error)) from None
        active = self.incident or self.incident_peer
        if active is not None and any(context[key] != active[key] for key in ('peer_id_sha256','peer_hash')):
            raise IntentRejected('incident_peer_mismatch')
        return context

    def _incident_storage_failure(self, now):
        self.incident_error = 'incident_storage_unavailable'
        self.recovery_required = self.flight_started = True
        self.landing_confirmed = False
        self.fault_reason = self.fault_reason or self.incident_error
        self._revoke()
        self.backend.invalidate_ground()
        self._transition(now,'fault',self.incident_error)

    def _mark_incident(self, now, reason, *, context=None):
        if self.incident_store is None:return None
        if self.incident_error:raise IntentRejected(self.incident_error)
        context = context or self._incident_context(now)
        self.incident_peer = context
        try:
            self.incident = self.incident_store.mark_airborne(
                context['peer_id_sha256'],context['peer_hash'],reason=reason)
        except (OSError,ValueError,RuntimeError):
            self._incident_storage_failure(self._now())
            raise IntentRejected('incident_storage_unavailable') from None
        # Disk sync occurs outside the UDP lock. Its delay never refreshes the
        # caller's command deadline or lets another generation inherit intent.
        try:unchanged = self._incident_context(self._now())['key'] == context['key']
        except IntentRejected:unchanged = False
        if not unchanged:
            self.recovery_required = self.flight_started = True
            self.fault_reason = self.fault_reason or 'flight_incident_context_changed'
            self._revoke()
            self._transition(self._now(),'fault','flight_incident_context_changed')
            raise IntentRejected('flight_incident_context_changed')
        return context['key']

    def _observe_incident(self, now):
        if self.incident_store is None:return
        sample = self.last_sample
        if (self.incident is None and self.incident_error is None and sample is not None
                and sample.synthetic is (self.mode is ControllerMode.SIMULATE)
                and sample.motion_received_at is not None
                and 0 <= now-sample.motion_received_at <= self.limits.telemetry_age
                and (sample.flying is True or sample.motors_on is True)):
            self.recovery_required = self.flight_started = True
            self.landing_confirmed = False
            self.fault_reason = 'unexpected_aircraft_activity'
            self._revoke()
            self.pending = self.pending_intent_id = self.pending_action = None
            self.land_acknowledged = self.action_acknowledged = False
            self.land_route_key = self.land_requested_at = None
            self.landing_episode += 1
            self._transition(now,'fault','unexpected_aircraft_activity')
            try:self._mark_incident(now,'airborne_observed',context=self.sample_aircraft_context)
            except IntentRejected:pass
        if self.recovery_required and self.state not in ('disconnected','landing','fault'):
            self._transition(now,'fault',self.incident_error or self.fault_reason or 'unresolved_previous_flight')

    def _resolve_incident(self, now):
        if self.incident_store is None:return
        if self.incident_error:raise IntentRejected(self.incident_error)
        if self.incident is None:return
        context = self._incident_context(now)
        try:self.incident_store.resolve_ground(context['peer_id_sha256'],context['peer_hash'])
        except (OSError,ValueError,RuntimeError):
            self._incident_storage_failure(self._now())
            raise IntentRejected('incident_storage_unavailable') from None
        self.incident = self.incident_peer = None
        self.recovery_required = False

    def _can_reconnect_recovery(self):
        if (not self.recovery_required or self.state!='fault' or self.pending is not None
                or not callable(getattr(self.backend,'connection_status',None))):return False
        connection = self.backend.connection_status()
        return connection.get('worker_alive') is False and connection.get('cleanup_verified') is True

    def _sample_problem(self, now, *, ground=False):
        sample = self.last_sample
        if not self.backend or not self.backend.connected:
            return 'link_unavailable'
        motion_at = None if sample is None else (sample.motion_received_at if sample.motion_received_at is not None else sample.received_at)
        if sample is None or not 0 <= now-motion_at <= self.limits.telemetry_age:
            return 'telemetry_stale'
        if self.mode is ControllerMode.SIMULATE and not sample.synthetic:
            return 'telemetry_source_mismatch'
        if self.mode is ControllerMode.FLIGHT and sample.synthetic:
            return 'telemetry_source_mismatch'
        if (self.mode is ControllerMode.FLIGHT or (self.semantic_backend and self.backend.channel.observer is not None)) and sample.control_ready is not True:
            return 'wifi_control_unconfirmed'
        control_at = sample.control_received_at if sample.control_received_at is not None else sample.received_at
        if (self.mode is ControllerMode.FLIGHT or (self.semantic_backend and self.backend.channel.observer is not None)) and not 0 <= now-control_at <= self.limits.telemetry_age:
            return 'wifi_control_stale'
        battery_at = sample.battery_received_at if sample.battery_received_at is not None else sample.received_at
        if not 0 <= now-battery_at <= self.limits.battery_age:
            return 'battery_stale'
        battery = sample.battery_percent
        if type(battery) not in (float, int) or not math.isfinite(battery) or not self.limits.minimum_battery_percent <= battery <= 100:
            return 'battery_unavailable_or_low'
        if not (self.vertical_trial or self.yaw_trial) and sample.position_valid is not True:
            return 'position_invalid'
        if self.yaw_trial and (type(sample.yaw_deg) not in (int,float)
                or not math.isfinite(sample.yaw_deg) or not -180 <= sample.yaw_deg <= 180):
            return 'yaw_unavailable_or_invalid'
        if any(type(value) is not bool for value in (sample.flying, sample.motors_on)):
            return 'flight_state_unknown'
        if sample.grounded is None and not (not ground and self.state in ('taking_off','landing')
                and sample.flying is False and sample.motors_on is False):
            return 'flight_state_unknown'
        if (sample.grounded and (sample.flying or sample.motors_on)) or (sample.flying and not sample.motors_on):
            return 'flight_state_inconsistent'
        bounds = [('altitude_m', self.limits.maximum_altitude_m)]
        if not (self.vertical_trial or self.yaw_trial):
            bounds.append(('distance_m', self.limits.maximum_distance_m))
        for name, limit in bounds:
            value = getattr(sample, name)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= limit:
                return name + '_invalid_or_limit'
        if ground and (sample.grounded is not True or sample.flying is not False or sample.motors_on is not False):
            return 'ground_state_unconfirmed'
        return None

    def _grounded(self, now):
        s = self.last_sample
        motion_at = None if s is None else (s.motion_received_at if s.motion_received_at is not None else s.received_at)
        action_at = max((t for t in (self.transition_started,self.pending_sent_at) if t is not None),default=None)
        return bool(s and 0 <= now-motion_at <= self.limits.telemetry_age
                    and s.synthetic is (self.mode is ControllerMode.SIMULATE)
                    and (action_at is None or motion_at > action_at)
                    and s.grounded is True and s.flying is False and s.motors_on is False)

    def _emit(self, packet, now, *, emergency=False, ingress=None, expires_at=None):
        self._event(now, 'packet_intent', raw_hex=packet.encode().hex(),
                    simulated=self.mode is ControllerMode.SIMULATE, emergency=emergency)
        if not self.journal_healthy and not emergency:
            raise IntentRejected('journal_unavailable')
        if ingress is not None:now = self._ingress_now(ingress)
        if expires_at is not None:
            now = self._now()
            if now >= expires_at:raise IntentRejected('control_deadline_invalid_or_expired')
        self.backend.send(packet, now)

    def _ingress_now(self, ingress, now=None):
        if ingress is None:
            return self._now() if now is None else now
        if not isinstance(ingress, CommandIngress):
            raise IntentRejected('invalid_command_ingress')
        # Obtain the actual connection under its own lock before sampling time:
        # waiting for that lock must not extend an operator's deadline.
        status = getattr(self.backend, 'connection_status', None)
        generation = status().get('generation') if callable(status) else None
        current = self._now()
        try: ingress.validate(current, connection_generation=generation)
        except ValueError as error: raise IntentRejected(str(error)) from None
        return current

    def _action(self, action, now, *, emergency=False, confirmation_key=None, aircraft_key=None,
                ingress=None):
        if ingress is not None:now = self._ingress_now(ingress)
        if self.pending:
            self._event(now, 'pending_action_abandoned', command_sequence=self._pending_sequence())
        self.pending_action = action
        self.pending_intent_id = self.pending_sent_at = None
        self.action_acknowledged = False
        self.transition_started = now
        if action in (FlightAction.LAND, FlightAction.TAKEOFF):
            self.landing_episode += 1
            self.land_acknowledged = False
            self.land_requested_at = now if action is FlightAction.LAND else None
            self.land_route_key = None
            self.landing_challenge_key = None
            if action is FlightAction.LAND and self.semantic_backend:
                # LAND remains available for recovery even without the extra
                # evidence needed by the separate, explicit confirmation.
                try:self.land_route_key = self.backend.landing_route_key(now)
                except ValueError:pass
        try:
            if self.semantic_backend:
                expiry = now + .25 if ingress is None else min(now + .25, ingress.expires_at)
                self._event(now, 'action_intent', action=action.name.lower(), expires_at=expiry,
                            emergency=emergency, simulated=self.mode is ControllerMode.SIMULATE)
                if not self.journal_healthy and not emergency:
                    raise IntentRejected('journal_unavailable')
                options = {} if action is not FlightAction.CONFIRM_LANDING else {
                    'confirmation_key':confirmation_key,'since':self.land_requested_at}
                if self.incident_store is not None and (self.incident is not None or self.incident_peer is not None):
                    options['aircraft_key'] = (aircraft_key if aircraft_key is not None else
                        self._incident_context(now,retained=action is not FlightAction.TAKEOFF)['key'])
                if action is FlightAction.TAKEOFF and self.vertical_trial:
                    options['vertical_limits'] = {
                        'minimum_battery_percent':self.limits.minimum_battery_percent,
                        'maximum_altitude_m':self.limits.maximum_altitude_m}
                if action is FlightAction.TAKEOFF and self.yaw_trial:
                    options['yaw_limits'] = {
                        'minimum_battery_percent':self.limits.minimum_battery_percent,
                        'maximum_altitude_m':self.limits.maximum_altitude_m,
                        'maximum_channel':self.limits.maximum_channel}
                if ingress is not None:now = self._ingress_now(ingress)
                self.pending_intent_id = self.backend.submit_action(action, now, expires_at=expiry, **options)
                self.pending = None, now
                self._event(now, 'action_queued', intent_id=self.pending_intent_id)
            else:
                request = flight_action(action, self.sequence.next())
                self.pending = request, now
                if ingress is not None:now = self._ingress_now(ingress)
                self._emit(request, now, emergency=emergency, ingress=ingress)
        except Exception:
            self.pending = None
            raise

    def _pending_sequence(self):
        return self.pending[0].seq if self.pending and self.pending[0] is not None else None

    def _send_control(self, channels, now, *, emergency=False, ingress=None):
        if not self.semantic_backend:
            self._emit(virtual_joystick(channels, self.sequence.next()), now, emergency=emergency,
                       ingress=ingress,
                       expires_at=None if emergency or self.control is None else self.control[1])
            return
        expiry = now + .25 if emergency else self.control[1]
        self._event(now, 'control_intent', channels=list(channels), expires_at=expiry,
                    emergency=emergency, simulated=self.mode is ControllerMode.SIMULATE)
        if not self.journal_healthy and not emergency:
            raise IntentRejected('journal_unavailable')
        self.backend.submit_control(channels, now, expires_at=expiry)

    def _action_receipt(self, now):
        receipt = self.backend.action_receipt(self.pending_intent_id)
        now = self._now()
        if receipt is None:
            raise RuntimeError('flight_action_receipt_missing')
        if 'request' in receipt:
            request = DumlPacket.decode(bytes.fromhex(receipt['request']))
            self.pending = request, self.pending[1]
            self.pending_sent_at = receipt['sent_at']
        if receipt['state'] in ('expired', 'cancelled', 'superseded', 'sent_unconfirmed'):
            self._event(now, 'action_transmission_failed', intent_id=self.pending_intent_id,
                        transmission_state=receipt['state'])
            self.pending = None
            if self.state == 'landing':
                self.fault_reason = self.fault_reason or 'landing_not_sent'
                self._revoke()
                self._transition(now, 'fault', 'landing_not_sent')
            else:
                self._failsafe(now, 'action_not_sent')
            return []
        if receipt['state'] in ('acknowledged', 'rejected'):
            if receipt['acknowledged_at'] - receipt['sent_at'] >= self.limits.acknowledgement_seconds:
                return []
            return [DumlPacket.decode(bytes.fromhex(receipt['response']))]
        if receipt['state'] in ('queued', 'scheduled') and now >= receipt['expires_at']:
            self.pending = None
            self._failsafe(now, 'action_not_sent')
        return []

    def _failsafe(self, now, reason):
        if self.fault_reason is not None:
            return
        airborne = self.state in self.AIR_STATES
        self.fault_reason = reason
        self._revoke()
        self._event(now, 'failsafe', reason=reason, possible_airborne=airborne)
        if not airborne or not self.backend or not self.backend.connected:
            self._transition(now, 'fault', reason)
            return
        try:
            if not self.semantic_backend:
                self._send_control((0, 0, 0, 0), now, emergency=True)
            confirming = (self.pending_action is FlightAction.CONFIRM_LANDING
                          and self.land_acknowledged)
            if not confirming and (not self.pending or self.pending_action is not FlightAction.LAND):
                self._action(FlightAction.LAND, now, emergency=True)
            self._transition(now, 'landing', reason)
        except Exception:
            self._transition(now, 'fault', 'failsafe_send_failed')

    def _update(self, now):
        if self.backend is None or self.state == 'disconnected':
            return now
        try:
            responses = self.backend.advance(now)
            if self.incident_store is not None:
                self.last_sample,self.sample_aircraft_context = self.backend.observation_context()
            else:self.last_sample = self.backend.observation()
            # The UDP owner can publish between tick start and this read.
            # Compare receipt timestamps against a clock read after sampling.
            now = self._now()
            self._observe_incident(now)
            if self.semantic_backend and self.pending is not None:
                responses = self._action_receipt(now)
                now = self._now()
        except Exception:
            now = self._now()
            self._failsafe(now, 'backend_failed')
            responses = []
        for packet in responses:
            if self.pending is None:
                self._event(now, 'unexpected_response', command_sequence=packet.seq)
                continue
            try:
                reply = parse_action_response(packet, self.pending[0])
            except ValueError:
                self._event(now, 'unmatched_response', command_sequence=packet.seq)
                continue
            request = self.pending[0]
            self.pending = None
            self._event(now, 'action_response', command_sequence=request.seq,
                        request_action=FlightAction(request.payload[0]).name.lower(), **reply.as_dict())
            if reply.accepted:
                self.action_acknowledged = True
                if request.payload == bytes([FlightAction.LAND]):
                    self.land_acknowledged = True
            elif request.payload[0] in self.LANDING_ACTIONS:
                self.fault_reason = self.fault_reason or 'landing_rejected'
                self._revoke()
                self._transition(now, 'fault', 'landing_rejected')
            else:
                self._failsafe(now, 'action_rejected')
        self._complete_landing(now)
        if not self.journal_healthy:
            self._failsafe(now, 'journal_unavailable')
        if self.state in ('armed', *self.AIR_STATES):
            problem = self._sample_problem(now, ground=self.state == 'armed')
            if problem:
                self._failsafe(now, problem)
            elif self.state == 'flying' and self.last_sample.flying is not True:
                self._failsafe(now, 'flight_state_changed')
        sent_at = (self.pending_sent_at if self.semantic_backend else self.pending[1]) if self.pending else None
        if self.pending and sent_at is not None and now - sent_at >= self.limits.acknowledgement_seconds:
            self._event(now, 'action_ack_timeout', command_sequence=self._pending_sequence())
            self.pending = None
            if self.state == 'landing':
                self.fault_reason = self.fault_reason or 'action_ack_timeout'
                self._revoke()
                self._transition(now, 'fault', 'landing_ack_timeout')
            else:
                self._failsafe(now, 'action_ack_timeout')
        if self.state in self.AIR_STATES and self.transition_started is not None:
            if self.state != 'flying' and now - self.transition_started >= self.limits.transition_seconds:
                if self.state == 'landing':
                    self.fault_reason = self.fault_reason or 'landing_unconfirmed'
                    self._transition(now, 'fault', 'landing_unconfirmed')
                else:
                    self._failsafe(now, 'takeoff_unconfirmed')
        if self.state == 'armed' and (now >= self.armed_deadline or now >= self.lease_deadline):
            self._revoke()
            self._transition(now, 'ready', 'arming_expired')
        if self.state in ('taking_off', 'flying'):
            if now >= self.lease_deadline:
                self._failsafe(now, 'operator_lease_expired')
            elif now >= self.flight_deadline:
                self._failsafe(now, 'maximum_flight_time')
            elif self.control and now >= self.control[1]:
                self._failsafe(now, 'control_expired')
        sample = self.last_sample
        if self.state == 'taking_off' and self.pending is None and self.action_acknowledged and sample:
            if not self._sample_problem(now) and sample.flying is True and sample.grounded is False and sample.received_at >= self.transition_started:
                self._transition(now, 'flying', 'airborne_observed_after_ack')
        if self.state == 'ready' and self._sample_problem(now, ground=True):
            self._transition(now, 'not_ready', 'readiness_lost')
        if self.state == 'not_ready' and not self.fault_reason and not self._sample_problem(now, ground=True):
            self._transition(now, 'ready', 'readiness_observed')
        return now

    def _complete_landing(self, now):
        if (self.state == 'landing' and self.pending is None and self.action_acknowledged
                and self.incident is None and self.incident_error is None
                and self.pending_action in self.LANDING_ACTIONS and self._grounded(now)):
            self.landing_confirmed, self.flight_started = True, False
            self._revoke()
            self._transition(now, 'fault' if self.fault_reason else 'ready', 'grounded_observed_after_ack')

    def _landing_confirmation_context(self, now):
        """An explicit continuation of an ACKed LAND, never a flight lease."""
        if not self.semantic_backend:
            raise IntentRejected('landing_confirmation_unavailable')
        if self.stopping or not self.journal_healthy:
            raise IntentRejected('landing_confirmation_unavailable')
        if self.state not in ('landing','fault') or not self.flight_started:
            raise IntentRejected('landing_intent_required')
        if not self.backend.connected:
            raise IntentRejected('link_unavailable')
        if self.pending is not None:
            raise IntentRejected('flight_action_pending')
        if not self.land_acknowledged or self.land_route_key is None:
            raise IntentRejected('acknowledged_land_required')
        try:
            route_key = self.backend.landing_route_key(now)
            if route_key != self.land_route_key:
                raise IntentRejected('landing_route_changed')
            key = self.backend.landing_confirmation_key(now,since=self.land_requested_at)
        except ValueError as error:
            raise IntentRejected(str(error)) from None
        return self.landing_episode, route_key, key

    def _pump(self, now):
        if self.state != 'flying' or self.control is None:
            return
        if now >= self.control[1]:
            self._failsafe(now, 'control_expired')
            return
        if self.next_control_send is None or now >= self.next_control_send:
            try:
                self._send_control(self.control[0], now)
                self.next_control_send = now + self.limits.control_interval
            except Exception:
                self._failsafe(now, 'control_send_failed')

    def tick(self):
        with self.lock:
            now = self._now()
            now = self._update(now)
            self._pump(now)

    def _check_lease(self, token, now):
        if (not isinstance(token, str) or not token.isascii() or not self.lease
            or not hmac.compare_digest(token, self.lease) or now >= self.lease_deadline):
            raise IntentRejected('operator_lease_invalid')

    def _validate_channels(self,channels):
        from .flight import trial_channels_allowed
        if not isinstance(channels,(list,tuple)) or len(channels)!=4 or any(
                type(v) is not int or abs(v)>self.limits.maximum_channel for v in channels):
            raise IntentRejected('control_channels_invalid_or_limit')
        if self.yaw_trial and not trial_channels_allowed(channels,acquisition_trial=self.acquisition_trial):
            raise IntentRejected('yaw_trial_requires_yaw_only')

    def _consume_confirmation(self, operation, token, now):
        item = self.challenge
        if (not isinstance(token, str) or not token.isascii() or not item or item[0] != operation
            or not hmac.compare_digest(token, item[1])):
            raise IntentRejected('confirmation_invalid')
        self.challenge = None
        if now >= item[2]:
            raise IntentRejected('confirmation_expired')

    def handle(self, action: str, parameters: dict | None = None, *, ingress=None):
        with self.lock:
            now = self._now()
            now = self._update(now)
            try:
                if self.mode is ControllerMode.OBSERVE:
                    raise IntentRejected('observe_mode')
                if self.stopping:
                    raise IntentRejected('controller_stopping')
                if not isinstance(action, str) or action not in self.FIELDS:
                    raise IntentRejected('unknown_intent')
                if parameters is None:
                    parameters = {}
                if not isinstance(parameters, dict) or set(parameters) - self.FIELDS[action]:
                    raise IntentRejected('invalid_parameters')
                # Tokens and arbitrary user strings never enter the journal.
                self._event(now, 'intent_received', action=action)
                if not self.journal_healthy:
                    raise IntentRejected('journal_unavailable')
                if ingress is not None:now = self._ingress_now(ingress)
                result = self._handle(action, parameters, now, ingress=ingress)
            except IntentRejected as error:
                self._event(now, 'intent_rejected', action=action if isinstance(action, str) and action in self.FIELDS else 'unknown', reason=str(error))
                if not self.journal_healthy:
                    self._failsafe(now, 'journal_unavailable')
                raise
            except Exception:
                self._failsafe(now, 'backend_failed')
                raise IntentRejected('backend_failed') from None
            now = self._now()
            self._event(now, 'intent_accepted', action=action)
            if not self.journal_healthy:
                self._failsafe(now, 'journal_unavailable')
                raise IntentRejected('journal_unavailable')
            self._pump(self._now())
            return {'accepted': True, 'mode': self.mode.value, 'state': self.state, **result}

    def _handle(self, action, p, now, *, ingress=None):
        if self.recovery_required and action in ('arm','takeoff','control'):
            raise IntentRejected('flight_recovery_required')
        if action == 'open':
            if self.state != 'disconnected':
                raise IntentRejected('session_already_open')
            self.backend.open(now)
            self._transition(now, 'not_ready', 'session_opened')
            self._update(now)
        elif action == 'reconnect':
            if not self._can_reconnect_recovery():raise IntentRejected('previous_flight_cleanup_unconfirmed')
            if ingress is not None:now = self._ingress_now(ingress)
            self._revoke()
            self.backend.close(now)
            self.pending_action = self.pending_intent_id = self.pending_sent_at = None
            self.land_route_key = self.land_requested_at = self.transition_started = None
            self.action_acknowledged = self.land_acknowledged = False
            self.landing_episode += 1
            self.last_sample = None
            if ingress is not None:now = self._ingress_now(ingress)
            self.backend.open(now)
            self._event(self._now(),'recovery_connection_requested')
        elif action == 'close':
            observed_active=bool(self.last_sample and (
                self.last_sample.flying is True or self.last_sample.motors_on is True))
            cancelling_setup=(self.semantic_backend and self.state in ('not_ready','ready','armed','fault')
                              and not self.flight_started and self.pending is None and not observed_active)
            if not cancelling_setup and (self.state in self.AIR_STATES or self.flight_started
                    or (self.state != 'disconnected' and not self._grounded(now))):
                raise IntentRejected('ground_state_unconfirmed')
            self.backend.close(now)
            self._revoke()
            self.pending = self.last_sample = self.fault_reason = None
            self._transition(now, 'disconnected', 'session_closed')
        elif action == 'reset':
            if self.state != 'fault' or self.flight_started or not self._grounded(now):
                raise IntentRejected('reset_requires_confirmed_ground')
            self.fault_reason = None
            self._transition(now, 'not_ready', 'local_reset')
            self._update(now)
        elif action == 'prepare':
            operation = p.get('operation')
            if self.recovery_required and operation in ('arm','takeoff'):
                raise IntentRejected('flight_recovery_required')
            if operation == 'landing':
                self.landing_challenge_key = self._landing_confirmation_context(now)
                if ingress is not None:now = self._ingress_now(ingress)
                token = secrets.token_urlsafe(32)
                if ingress is not None:now = self._ingress_now(ingress)
                self.challenge = operation,token,now+self.limits.confirmation_seconds
                return {'confirmation_token':token,'expires_at':self.challenge[2],
                        'operation':operation,'statement':'operator_confirms_clear_landing_area'}
            if operation == 'ground':
                if (not self.semantic_backend or self.backend.channel.observer is None
                        or self.state not in ('not_ready','ready','landing','fault')):
                    raise IntentRejected('ground_confirmation_unavailable')
                try:self.ground_challenge_key = self.backend.ground_confirmation_key(now)
                except ValueError as error:raise IntentRejected(str(error)) from None
                if ingress is not None:now = self._ingress_now(ingress)
                token = secrets.token_urlsafe(32)
                if ingress is not None:now = self._ingress_now(ingress)
                self.challenge = operation,token,now+self.limits.confirmation_seconds
                return {'confirmation_token':token,'expires_at':self.challenge[2],
                        'operation':operation,'statement':'operator_visually_confirms_grounded_and_disarmed'}
            if operation not in ('arm', 'takeoff'):
                raise IntentRejected('unsupported_preparation')
            expected = 'ready' if operation == 'arm' else 'armed'
            if self.state != expected:
                raise IntentRejected('invalid_state')
            if operation == 'takeoff':
                self._check_lease(p.get('lease'), now)
            problem = self._sample_problem(now, ground=True)
            if problem:
                raise IntentRejected(problem)
            token = secrets.token_urlsafe(32)
            if ingress is not None:now = self._ingress_now(ingress)
            self.challenge = operation, token, now + self.limits.confirmation_seconds
            return {'confirmation_token': token, 'expires_at': self.challenge[2], 'operation': operation}
        elif action == 'confirm_ground':
            self._consume_confirmation('ground',p.get('confirmation_token'),now)
            if (not self.semantic_backend or self.backend.channel.observer is None
                    or self.state not in ('not_ready','ready','landing','fault')):
                raise IntentRejected('ground_confirmation_unavailable')
            key,self.ground_challenge_key = self.ground_challenge_key,None
            if self.incident is not None or self.recovery_required:self._incident_context(now)
            if self.incident_error:raise IntentRejected(self.incident_error)
            action_at = max((t for t in (self.transition_started,self.pending_sent_at) if t is not None),default=None)
            if (self.incident is not None and action_at is not None
                    and (self.last_sample is None or self.last_sample.motion_received_at is None
                         or self.last_sample.motion_received_at <= action_at)):
                raise IntentRejected('ground_observation_precedes_last_action')
            self._event(now,'ground_attestation_intent',source='operator_confirmation')
            if not self.journal_healthy:raise IntentRejected('journal_unavailable')
            if ingress is not None:now = self._ingress_now(ingress)
            try:self.backend.confirm_ground(key,now)
            except ValueError as error:raise IntentRejected(str(error)) from None
            self._resolve_incident(self._now())
            self._update(now)
            now = self._now()
            if self.state == 'fault' and self.flight_started and self._grounded(now):
                self.flight_started = False
                self._event(now,'ground_reconciled_by_operator',action_acknowledged=self.action_acknowledged)
            return {'ground_source':'operator_confirmation'}
        elif action == 'confirm_landing':
            self._consume_confirmation('landing',p.get('confirmation_token'),now)
            key,self.landing_challenge_key = self.landing_challenge_key,None
            if key is None or key != self._landing_confirmation_context(now):
                raise IntentRejected('landing_confirmation_context_changed')
            # A click consumes its authority even if the owner subsequently
            # refuses transmission. No watchdog or ACK path creates a token.
            self._revoke()
            self._action(FlightAction.CONFIRM_LANDING,now,confirmation_key=key[2],ingress=ingress)
            self._transition(now,'landing','landing_confirmation_requested')
        elif action == 'arm':
            self._consume_confirmation('arm', p.get('confirmation_token'), now)
            if self.state != 'ready':
                raise IntentRejected('invalid_state')
            problem = self._sample_problem(now, ground=True)
            if problem:
                raise IntentRejected(problem)
            lease = secrets.token_urlsafe(32)
            if ingress is not None:now = self._ingress_now(ingress)
            self.lease = lease
            self.lease_deadline = (now if ingress is None else ingress.issued_at) + self.limits.lease_seconds
            self.armed_deadline = now + self.limits.armed_seconds
            self.last_control_sequence = -1
            self.last_control_intent = None
            self._transition(now, 'armed', 'local_authorization_only')
            return {'lease': self.lease, 'lease_expires_at': self.lease_deadline}
        elif action in ('keepalive', 'disarm'):
            self._check_lease(p.get('lease'), now)
            if action == 'disarm':
                if self.state != 'armed' or not self._grounded(now):
                    raise IntentRejected('invalid_state')
                self._revoke()
                self._transition(now, 'ready', 'local_authorization_revoked')
            else:
                if self.state not in ('armed', 'taking_off', 'flying'):
                    raise IntentRejected('invalid_state')
                self.lease_deadline = (now if ingress is None else ingress.issued_at) + self.limits.lease_seconds
        elif action == 'takeoff':
            self._check_lease(p.get('lease'), now)
            self._consume_confirmation('takeoff', p.get('confirmation_token'), now)
            if self.state != 'armed':
                raise IntentRejected('invalid_state')
            problem = self._sample_problem(now, ground=True)
            if problem:
                raise IntentRejected(problem)
            aircraft_key = self._mark_incident(now,'takeoff_requested')
            if self.incident_store is not None or ingress is not None:
                try:
                    self.last_sample = self.backend.observation()
                    current = self._now()
                    if ingress is not None:current = self._ingress_now(ingress)
                    self._check_lease(p.get('lease'),current)
                    problem = self._sample_problem(current,ground=True)
                    if problem:raise IntentRejected(problem)
                except IntentRejected:
                    if self.incident_store is not None:
                        self.recovery_required = self.flight_started = True
                        self.fault_reason = 'takeoff_authority_changed'
                        self._revoke()
                        self._transition(self._now(),'fault',self.fault_reason)
                    raise
                # Internal local callers also keep the original action time.
                # Waiting for incident fsync must not create a fresh 250 ms
                # enqueue window. HTTP contexts carry their own fixed expiry.
                if ingress is not None:now = current
            self.flight_deadline = now + self.limits.maximum_flight_seconds
            self.flight_started, self.landing_confirmed = True, False
            self._transition(now, 'taking_off', 'takeoff_requested')
            self._action(FlightAction.TAKEOFF, now,aircraft_key=aircraft_key,ingress=ingress)
        elif action == 'land':
            recovery = self.state in ('landing', 'fault') and self.flight_started
            if self.state not in ('taking_off', 'flying') and not recovery:
                raise IntentRejected('invalid_state')
            if not self.backend or not self.backend.connected:
                raise IntentRejected('link_unavailable')
            if recovery and self._grounded(now):
                raise IntentRejected('aircraft_already_grounded')
            if self.pending is not None and self.pending_action in self.LANDING_ACTIONS:
                # A second explicit click must not replace a transaction whose
                # actual send/ACK is still pending. No tick retries LAND.
                return {'landing_already_pending': True}
            self._revoke()
            if recovery:
                self._event(now, 'landing_recovery_requested', fault_reason=self.fault_reason)
            if not self.semantic_backend:self._send_control((0, 0, 0, 0), now, ingress=ingress)
            self._action(FlightAction.LAND, now,ingress=ingress)
            self._transition(now, 'landing', 'landing_recovery_requested' if recovery else 'landing_requested')
        elif action == 'control':
            if self.vertical_trial:
                raise IntentRejected('vertical_trial_control_disabled')
            self._check_lease(p.get('lease'), now)
            if self.state != 'flying':
                raise IntentRejected('invalid_state')
            seq, expiry, channels = p.get('sequence'), p.get('expires_at'), p.get('channels')
            if type(seq) is not int or not 0 <= seq < 1 << 31 or seq <= self.last_control_sequence:
                raise IntentRejected('control_sequence_replayed_or_invalid')
            if type(expiry) not in (float, int) or not math.isfinite(expiry) or not now < expiry <= now + self.limits.control_seconds:
                raise IntentRejected('control_deadline_invalid_or_expired')
            if ingress is not None:expiry = min(expiry, ingress.expires_at)
            self._validate_channels(channels)
            if self.yaw_trial and (self.last_sample.flying is not True or self.last_sample.motors_on is not True):
                raise IntentRejected('yaw_trial_requires_airborne_observation')
            if self.last_control_intent is not None and now - self.last_control_intent < self.limits.control_interval - 1e-9:
                raise IntentRejected('control_rate_limited')
            self.last_control_sequence, self.last_control_intent = seq, now
            self.control = tuple(channels), expiry
            self.lease_deadline = (now if ingress is None else ingress.issued_at) + self.limits.lease_seconds
        return {}

    def status(self):
        with self.lock:
            now = self._now()
            now = self._update(now)
            telemetry = None
            if self.last_sample is not None:
                telemetry = asdict(self.last_sample)
                invalid_fields = []
                for key, value in telemetry.items():
                    if isinstance(value, float) and not math.isfinite(value):
                        telemetry[key] = None
                        invalid_fields.append(key)
                telemetry.update(age_seconds=now - self.last_sample.received_at,
                                 valid=self._sample_problem(now) is None,
                                 invalid_fields=invalid_fields)
                telemetry['component_age_seconds'] = {
                    name:None if stamp is None else now-stamp for name,stamp in
                    (('motion',self.last_sample.motion_received_at),
                     ('battery',self.last_sample.battery_received_at),
                     ('control',self.last_sample.control_received_at))}
            result = {'mode': self.mode.value, 'state': self.state, 'now': now,
                    'vertical_trial': self.vertical_trial,
                    'yaw_trial': self.yaw_trial,
                    'acquisition_trial': self.acquisition_trial,
                    'live_flight_enabled': self.mode is ControllerMode.FLIGHT, 'synthetic': self.mode is ControllerMode.SIMULATE,
                    'connected': bool(self.backend and self.backend.connected),
                    'fault_reason': self.fault_reason, 'journal_healthy': self.journal_healthy,
                    'stopping': self.stopping,
                    'landing_confirmed': self.landing_confirmed,
                    'possible_airborne': self.flight_started if self.mode is not ControllerMode.OBSERVE else None,
                    'lease_expires_at': self.lease_deadline,
                    'limits': asdict(self.limits), 'events': list(self.events),
                    'pending_command_sequence': self._pending_sequence(),
                    'land_requested_at':self.land_requested_at,
                    'land_acknowledged':self.land_acknowledged,
                    'telemetry': telemetry}
            confirmation = {'needed':None,'available':False,'reason':'landing_confirmation_unavailable'}
            if self.semantic_backend:
                confirmation.update(self.backend.landing_confirmation_status(now))
                try:self._landing_confirmation_context(now)
                except IntentRejected as error:confirmation.update(available=False,reason=str(error))
                else:confirmation.update(available=True,reason=None)
            result['landing_confirmation'] = confirmation
            peer_match = None
            recovery_reason = self.incident_error or (self.fault_reason if self.recovery_required else None)
            if self.incident is not None or self.recovery_required:
                try:self._incident_context(now,retained=True)
                except IntentRejected as error:
                    if str(error)=='incident_peer_mismatch':peer_match = False
                    recovery_reason = self.incident_error or str(error)
                else:peer_match = True
            result['recovery'] = {'required':self.recovery_required,
                'incident_id':None if self.incident is None else self.incident['incident_id'],
                'peer_match':peer_match,'reason':recovery_reason,
                'connection_retry_available':self._can_reconnect_recovery()}
            if self.semantic_backend and callable(getattr(self.backend,'connection_status',None)):
                result['connection']=self.backend.connection_status()
            if self.observation_source is not None:
                observed = self.observation_source(now)
                result.update(observed)
                fields = (observed.get('telemetry') or {}).get('fields', {})
                flying = fields.get('is_flying', {})
                result['possible_airborne'] = True if flying.get('value') is True else None
                result['landing_confirmed'] = False
            return result

    def attach_observation_source(self, source):
        with self.lock:
            if self.mode is not ControllerMode.OBSERVE or not callable(source):
                raise ValueError('real observations can only be attached in observe mode')
            self.observation_source = source

    def observation_event(self, event, **metadata):
        with self.lock:
            if self.mode is not ControllerMode.OBSERVE or self.stopping:
                raise IntentRejected('observation_unavailable')
            self._event(self._now(), event, **metadata)
            if not self.journal_healthy:
                raise IntentRejected('journal_unavailable')

    def stop(self, reason='controller_stopped'):
        if reason not in {'controller_stopped', 'controller_parent_timeout',
                          'controller_parent_disconnected', 'controller_ipc_failed'}:
            raise ValueError('invalid controller stop reason')
        with self.lock:
            if self.stopping:
                return
            now = self._now()
            self.stopping = True
            self._failsafe(now, reason)
            self._event(now, 'controller_stop_requested', reason=reason)
