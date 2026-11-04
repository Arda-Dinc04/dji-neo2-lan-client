"""Typed flight intents consumed by the established UDP owner's thread.

This transport mailbox does not establish aircraft readiness or grant an
operator lease. Its caller must supply those through the flight supervisor.
No existing CLI or HTTP route creates this channel.
"""
from collections import deque
from copy import deepcopy
from ipaddress import IPv4Address
import math
import secrets
import threading
import time

from .duml import DumlPacket
from .flight import (FlightAction, flight_action, parse_action_response, virtual_joystick,
                     DEFAULT_YAW_CHANNEL_LIMIT, validate_yaw_channel_limit,
                     ACQUISITION_ALTITUDE_LIMIT, trial_channels_allowed)
from .flight_observation import FlightObserver, FlightRoute


class FlightChannel:
    VALIDITY_SECONDS = .25
    CONTROL_INTERVAL = .05
    ACKNOWLEDGEMENT_SECONDS = 1

    def __init__(self, *, clock=time.monotonic, vertical_trial=False, yaw_trial=False,
                 yaw_channel_limit=DEFAULT_YAW_CHANNEL_LIMIT, acquisition_trial=False):
        if type(vertical_trial) is not bool:raise ValueError('vertical_trial_must_be_boolean')
        if type(yaw_trial) is not bool:raise ValueError('yaw_trial_must_be_boolean')
        if vertical_trial and yaw_trial:raise ValueError('flight_trial_profiles_are_exclusive')
        if type(acquisition_trial) is not bool or acquisition_trial and not yaw_trial:
            raise ValueError('acquisition_trial_requires_yaw_profile')
        self.clock = clock
        self._vertical_trial = vertical_trial
        self._yaw_trial = yaw_trial
        self._acquisition_trial = acquisition_trial
        self._yaw_channel_limit = validate_yaw_channel_limit(yaw_channel_limit,yaw_trial=yaw_trial)
        self._yaw_limits = {'minimum_battery_percent':20.,'maximum_altitude_m':self.trial_altitude_limit,
                            'maximum_channel':self._yaw_channel_limit}
        self.lock = threading.RLock()
        self.ready = self.revoked = False
        self.control_fresh = True
        self.ready_at = self.last_time = self.last_control_sent = None
        self.action = self.control = self.scheduled = None
        self.history = deque(maxlen=32)
        self.sent_count = self.ignored = 0
        self.loopback_only = False
        self.bound_transport = None
        self.observer = None
        self._confirmation_guards = {}
        self._aircraft_identity = object()
        self._aircraft_guards = {}
        self._retained_aircraft = None

    def aircraft_context(self, now, *, retained=False):
        """Stable aircraft identity and an opaque current owner generation."""
        with self.lock:
            now = self._landing_evaluation_time(now)
            route = None if self.observer is None else self.observer.route
            if retained and not self.revoked and self.bound_transport is not None and self._retained_aircraft is not None:
                return dict(self._retained_aircraft)
            if (self.revoked or self.bound_transport is None or route is None
                    or (not retained and not 0 <= now-route.seen_at < 2.)
                    or (not self.observer.synthetic and (route.host_control is None
                        or not route.host_control.beacon_route_verified))):
                raise ValueError('recovery_route_unavailable')
            return {'peer_id_sha256':route.peer_id_sha256,'peer_hash':route.peer_hash,
                    'key':(self._aircraft_identity,route.generation)}

    def invalidate_ground(self):
        with self.lock:
            if self.observer is not None:self.observer.invalidate_ground()

    @property
    def vertical_trial(self):
        return self._vertical_trial

    @property
    def yaw_trial(self):
        return self._yaw_trial

    @property
    def yaw_channel_limit(self):
        return self._yaw_channel_limit

    @property
    def acquisition_trial(self):
        return self._acquisition_trial

    @property
    def trial_altitude_limit(self):
        return ACQUISITION_ALTITUDE_LIMIT if self.acquisition_trial else 1.5

    def enable_observations(self, *, synthetic=False):
        with self.lock:
            if self.bound_transport is not None or self.revoked or self.observer is not None:
                raise ValueError('flight observations require a new transport generation')
            if synthetic:
                self.require_loopback()
            self.observer = FlightObserver(synthetic=synthetic)

    def update_observations(self, route, telemetry, battery, wifi_control, now, *, landing_confirmation=None):
        with self.lock:
            if self.observer is not None:
                if self.revoked:
                    self.observer.invalidate()
                else:
                    if type(now) not in (int,float) or not math.isfinite(now):
                        raise ValueError('flight observation clock must be finite')
                    # A receipt may wait for this lock while the operator
                    # confirms ground. Evaluate after that local decision;
                    # every aircraft/route timestamp remains the actual
                    # receipt time, so waiting cannot refresh old data.
                    evaluated_at=max(now,self.clock())
                    if route is None:
                        self._retained_aircraft = None
                    elif (isinstance(route,FlightRoute) and 0 <= evaluated_at-route.seen_at < 2.
                            and (self.observer.synthetic or route.host_control is not None
                                 and route.host_control.beacon_route_verified)):
                        # Only the fixed, still-owned transport may retain a
                        # verified peer through discovery gaps for LAND. This
                        # neither renews telemetry nor enables TAKEOFF/1e.
                        self._retained_aircraft = {'peer_id_sha256':route.peer_id_sha256,
                            'peer_hash':route.peer_hash,'key':(self._aircraft_identity,route.generation)}
                    self.observer.update(route,telemetry,battery,wifi_control,evaluated_at,
                                         landing_confirmation=landing_confirmation)

    def observation(self):
        with self.lock:
            return self.observer.read(self.clock()) if self.observer is not None else None

    def transport_connected(self):
        """Connection predicate without copying diagnostic histories/sources."""
        with self.lock:
            return self.bound_transport is not None and self._available(self.clock())

    def ground_confirmation_key(self, now):
        with self.lock:
            if self.observer is None:raise ValueError('ground_confirmation_unavailable')
            return self.observer.ground_confirmation_key(now)

    def confirm_ground(self, key, now):
        with self.lock:
            if self.observer is None or self.revoked:raise ValueError('ground_confirmation_unavailable')
            self.observer.confirm_ground(key,now)

    def landing_route_key(self, now):
        with self.lock:
            return self._landing_route_key_at(self._landing_evaluation_time(now))

    def _landing_evaluation_time(self, now):
        # The owner can refresh evidence while an API caller waits for this
        # lock. Judge its unchanged receipt times at the current local clock,
        # never at a supervisor timestamp captured before that wait.
        if type(now) not in (int,float) or not math.isfinite(now):
            raise ValueError('landing_confirmation_time_must_be_finite')
        current = self.clock()
        if type(current) not in (int,float) or not math.isfinite(current):
            raise ValueError('landing_confirmation_clock_must_be_finite')
        return max(now,current)

    def _landing_route_key_at(self, now):
        if self.observer is None or not self._available(now):
            if self.observer is not None:self.observer.invalidate_landing_route(now)
            raise ValueError('landing_route_not_ready')
        return self.observer.landing_route_key(now)

    def landing_confirmation_key(self, now, *, since=None):
        with self.lock:
            now = self._landing_evaluation_time(now)
            self._landing_route_key_at(now)
            return self.observer.landing_confirmation_key(now, since=since)

    def landing_confirmation_status(self, now):
        with self.lock:
            now = self._landing_evaluation_time(now)
            if self.observer is None:
                value = {'needed':None, 'received_at':None, 'motion_received_at':None,
                    'control_received_at':None, 'route_seen_at':None,
                    'evidence_available':False, 'reason':'landing_observations_unavailable'}
            else:
                if not self._available(now):self.observer.invalidate_landing_route(now)
                value = self.observer.landing_confirmation_status(now)
                if not self._available(now):
                    value.update(evidence_available=False, reason='landing_route_not_ready')
            return {**value, 'ready_at':self.ready_at}

    def _confirmation_guard(self, item, now):
        guard = self._confirmation_guards.get(item['id'])
        if guard is None:
            return False
        key, since = guard
        try:
            return key == self.landing_confirmation_key(now, since=since)
        except ValueError:
            return False

    def require_loopback(self):
        with self.lock:
            if self.bound_transport is not None and not IPv4Address(self.bound_transport['host']).is_loopback:
                raise ValueError('synthetic flight backend requires a loopback transport')
            self.loopback_only = True

    def bind_transport(self, host, port):
        address = IPv4Address(host)
        if type(port) is not int or not 0 < port <= 65535:
            raise ValueError('flight transport port must be in1..65535')
        with self.lock:
            if self.bound_transport is not None or self.revoked:
                raise ValueError('flight channel requires a new transport generation')
            if self.loopback_only and not address.is_loopback:
                raise ValueError('synthetic flight backend requires a loopback transport')
            self.bound_transport = {'host': str(address), 'port': port}

    def _finish(self, item, state, **detail):
        self._confirmation_guards.pop(item['id'], None)
        self._aircraft_guards.pop(item['id'], None)
        item.update(state=state, **detail)
        self.history.append(deepcopy(item))
        if item is self.action:
            self.action = None
        if item is self.control:
            self.control = None

    def _time(self, now):
        if not math.isfinite(now) or (self.last_time is not None and now < self.last_time):
            raise ValueError('flight_channel_time_must_be_monotonic')
        self.last_time = now
        for item in (self.action, self.control):
            if item is None:
                continue
            deadline = item.get('ack_deadline', item['expires_at'])
            if now >= deadline:
                self._finish(item, 'timed_out' if 'sent_at' in item else 'expired')

    def _available(self, now):
        return (self.ready and not self.revoked and self.ready_at is not None
                and 0 <= now - self.ready_at <= .5)

    def set_ready(self, ready, now):
        if type(ready) is not bool:
            raise ValueError('invalid_flight_route_readiness')
        with self.lock:
            self._time(now)
            if self.observer is not None and (not ready or (self.ready and not self._available(now))):
                self.observer.invalidate_landing_route(now)
            self.ready, self.ready_at = ready and not self.revoked, now
            if not self.ready:
                for item in (self.action, self.control):
                    if item is not None:
                        self._finish(item, 'cancelled', reason='route_unavailable')

    def set_control_fresh(self, fresh, now):
        """Suspend ordinary commands without discarding an owned landing route."""
        if type(fresh) is not bool:raise ValueError('invalid_control_freshness')
        with self.lock:
            self._time(now)
            self.control_fresh=fresh
            if not fresh:
                for item in (self.action,self.control):
                    if (item is not None and item['state'] in ('queued','scheduled')
                            and item.get('action')!='land' and 'parent_land_id' not in item
                            and not (item.get('action') == 'confirm_landing'
                                     and self._confirmation_guard(item,now))):
                        self._finish(item,'cancelled',reason='control_evidence_stale')
                if self.scheduled is not None and self.scheduled[0]['state']=='cancelled':
                    self.scheduled=None

    def _intent(self, kind, expires_at, **detail):
        now = self.clock()
        if (type(expires_at) not in (int, float) or not math.isfinite(expires_at)
                or not now < expires_at <= now + self.VALIDITY_SECONDS):
            raise ValueError('flight_intent_deadline_invalid_or_expired')
        if not self._available(now):
            raise RuntimeError('flight_route_not_ready')
        return {'id': secrets.token_hex(8), 'kind': kind, 'created_at': now,
                'expires_at': expires_at, 'state': 'queued', **detail}

    def submit_action(self, action, *, expires_at, confirmation_key=None, since=None, vertical_limits=None,
                      yaw_limits=None,
                      aircraft_key=None):
        if not isinstance(action, FlightAction) or action not in (
                FlightAction.TAKEOFF, FlightAction.LAND, FlightAction.CONFIRM_LANDING):
            raise ValueError('unsupported_flight_channel_action')
        with self.lock:
            if aircraft_key is not None and aircraft_key != self.aircraft_context(
                    self.clock(),retained=action is not FlightAction.TAKEOFF)['key']:
                raise ValueError('flight_incident_context_changed')
            bounds = None
            if self.vertical_trial and action is FlightAction.TAKEOFF:
                if vertical_limits is not None and not isinstance(vertical_limits,dict):
                    raise ValueError('vertical_trial_limits_invalid')
                bounds = ({'minimum_battery_percent':20.,'maximum_altitude_m':1.5}
                          if vertical_limits is None else dict(vertical_limits))
                if (set(bounds)!={'minimum_battery_percent','maximum_altitude_m'}
                        or any(type(v) not in (int,float) or not math.isfinite(v) for v in bounds.values())
                        or not 20 <= bounds['minimum_battery_percent'] <= 100
                        or not 0 < bounds['maximum_altitude_m'] <= 1.5):
                    raise ValueError('vertical_trial_limits_invalid')
            elif vertical_limits is not None:
                raise ValueError('vertical_trial_limits_require_profile_takeoff')
            yaw_bounds = None
            if self.yaw_trial and action is FlightAction.TAKEOFF:
                if yaw_limits is not None and not isinstance(yaw_limits,dict):
                    raise ValueError('yaw_trial_limits_invalid')
                yaw_bounds = dict(self._yaw_limits if yaw_limits is None else yaw_limits)
                if (set(yaw_bounds) != {'minimum_battery_percent','maximum_altitude_m','maximum_channel'}
                        or any(type(v) not in (int,float) or not math.isfinite(v) for v in yaw_bounds.values())
                        or not 20 <= yaw_bounds['minimum_battery_percent'] <= 100
                        or not 0 < yaw_bounds['maximum_altitude_m'] <= self.trial_altitude_limit
                        or type(yaw_bounds['maximum_channel']) is not int
                        or not 1 <= yaw_bounds['maximum_channel'] <= self.yaw_channel_limit):
                    raise ValueError('yaw_trial_limits_invalid')
            elif yaw_limits is not None:raise ValueError('yaw_trial_limits_require_profile_takeoff')
            if action is FlightAction.CONFIRM_LANDING:
                if self.action is not None:
                    raise RuntimeError('flight_action_pending')
                if confirmation_key is None or confirmation_key != self.landing_confirmation_key(self.clock(),since=since):
                    raise ValueError('landing_confirmation_context_changed')
            elif confirmation_key is not None or since is not None:
                raise ValueError('landing_confirmation_guard_requires_confirmation_action')
            if action is FlightAction.TAKEOFF and not self.control_fresh:
                raise RuntimeError('flight_route_not_ready')
            item = self._intent('action', expires_at, action=action.name.lower())
            if bounds is not None:item['vertical_limits']=bounds
            if yaw_bounds is not None:
                item['yaw_limits']=yaw_bounds
            if self.action is not None:
                if action is not FlightAction.LAND:
                    raise RuntimeError('flight_action_pending')
                self._finish(self.action, 'superseded', reason='landing_requested')
            if action in (FlightAction.LAND,FlightAction.CONFIRM_LANDING) and self.control is not None:
                self._finish(self.control, 'superseded', reason='landing_requested')
            self.action = item
            if yaw_bounds is not None:self._yaw_limits=dict(yaw_bounds)
            if aircraft_key is not None:self._aircraft_guards[item['id']] = aircraft_key
            if action is FlightAction.CONFIRM_LANDING:
                self._confirmation_guards[item['id']] = confirmation_key, since
            if action is FlightAction.TAKEOFF and self.observer is not None:
                self.observer.invalidate_ground()
            return {'accepted': True, 'intent_id': item['id']}

    def submit_control(self, channels, *, expires_at):
        if self.vertical_trial:raise ValueError('directional_control_disabled_for_vertical_trial')
        if not isinstance(channels, (list, tuple)):
            raise ValueError('exactly four normalized channels are required')
        channels = tuple(channels)
        virtual_joystick(channels, 0)
        if self.yaw_trial and (not trial_channels_allowed(channels,acquisition_trial=self.acquisition_trial) or abs(channels[3]) > min(
                self.yaw_channel_limit,self._yaw_limits['maximum_channel'])):
            raise ValueError('yaw_trial_requires_yaw_only')
        if not self.yaw_trial and any(abs(value) > 250 for value in channels):
            raise ValueError('flight_channel_exceeds_local_control_limit')
        with self.lock:
            if not self.control_fresh:raise RuntimeError('flight_route_not_ready')
            item = self._intent('control', expires_at, channels=list(channels))
            if self.yaw_trial:item['yaw_limits']=dict(self._yaw_limits)
            if self.action is not None and self.action['action'] in ('land','confirm_landing'):
                raise RuntimeError('landing_in_progress')
            aircraft_key = self.aircraft_context(self.clock())['key'] if self.yaw_trial else None
            if self.control is not None:
                self._finish(self.control, 'superseded')
            if aircraft_key is not None:self._aircraft_guards[item['id']] = aircraft_key
            self.control = item
            return {'accepted': True, 'intent_id': item['id']}

    def submit_landing(self, *, expires_at, aircraft_key=None):
        """One explicit neutral followed by LAND, both under the same deadline."""
        with self.lock:
            result = self.submit_action(FlightAction.LAND, expires_at=expires_at,aircraft_key=aircraft_key)
            self.action['neutral_before_land'] = True
            self.action['neutral_sent'] = False
            self.control = {'id': secrets.token_hex(8), 'kind': 'control',
                            'channels': [0]*4, 'parent_land_id': self.action['id'],
                            'created_at': self.action['created_at'], 'expires_at': expires_at,
                            'state': 'queued'}
            return result

    def landing_pending(self, now):
        with self.lock:
            self._time(now)
            return bool(self.action and self.action['action'] == 'land'
                        and self.action['state'] == 'queued' and self._available(now))

    def next_packet(self, now, next_sequence):
        with self.lock:
            self._time(now)
            if self.scheduled is not None or not self._available(now):
                return None
            item = self.action if self.action and self.action['state'] == 'queued' else self.control
            if (self.action is not None and self.action.get('neutral_before_land')
                    and not self.action['neutral_sent']):
                item = self.control
                if item is None:
                    self._finish(self.action, 'cancelled', reason='landing_neutral_unavailable')
                    return None
            if item is None:
                return None
            if item.get('action') == 'confirm_landing' and not self._confirmation_guard(item,now):
                self._finish(item,'cancelled',reason='landing_confirmation_context_changed')
                return None
            if (item['kind'] == 'control' and 'parent_land_id' not in item and self.last_control_sent is not None
                    and now - self.last_control_sent < self.CONTROL_INTERVAL - 1e-9):
                return None
            sequence = next_sequence()
            packet = (flight_action(FlightAction[item['action'].upper()], sequence)
                      if item['kind'] == 'action' else virtual_joystick(item['channels'], sequence))
            item.update(state='scheduled', sequence=sequence)
            self.scheduled = (item, packet)
            return packet

    def can_send(self, sequence, now):
        with self.lock:
            if (self.scheduled is not None
                    and (self.scheduled[0].get('action') == 'confirm_landing'
                         or (self.vertical_trial or self.yaw_trial) and self.scheduled[0].get('action') == 'takeoff'
                         or self.yaw_trial and self.scheduled[0]['kind']=='control')):
                now = self._landing_evaluation_time(now)
            self._time(now)
            if self.scheduled is None or self.scheduled[1].seq != sequence:
                return False
            item, _ = self.scheduled
            current = self.action if item['kind'] == 'action' else self.control
            allowed = (item is current and item['state'] == 'scheduled'
                       and now < item['expires_at'] and self._available(now))
            aircraft_key = self._aircraft_guards.get(item.get('parent_land_id',item['id']))
            if aircraft_key is not None:
                try:
                    allowed = allowed and aircraft_key == self.aircraft_context(
                        now,retained=(item.get('action') != 'takeoff' and not (
                            self.yaw_trial and item['kind']=='control' and 'parent_land_id' not in item)))['key']
                except ValueError:allowed = False
            emergency = item.get('action') == 'land' or 'parent_land_id' in item
            confirmation = item.get('action') == 'confirm_landing'
            if confirmation:
                allowed = allowed and self._confirmation_guard(item,now)
            elif not emergency:allowed=allowed and self.control_fresh
            if self.yaw_trial and not emergency and not confirmation:
                allowed = allowed and self.observer is not None
                if allowed:
                    channels = item.get('channels')
                    bounds = item['yaw_limits']
                    allowed = (type(bounds.get('maximum_channel')) is int
                        and 1 <= bounds['maximum_channel'] <= self.yaw_channel_limit
                        and type(bounds.get('maximum_altitude_m')) in (int,float)
                        and 0 < bounds['maximum_altitude_m'] <= self.trial_altitude_limit
                        and type(bounds.get('minimum_battery_percent')) in (int,float)
                        and 20 <= bounds['minimum_battery_percent'] <= 100
                        and (item.get('action') == 'takeoff' or (
                        isinstance(channels,list) and len(channels)==4
                        and all(type(v) is int for v in channels)
                        and trial_channels_allowed(channels,acquisition_trial=self.acquisition_trial)
                        and abs(channels[3]) <= bounds['maximum_channel']))
                        and self.observer.permits_yaw_trial(now,
                            airborne=item.get('action') != 'takeoff',
                            minimum_battery_percent=bounds['minimum_battery_percent'],
                            maximum_altitude_m=bounds['maximum_altitude_m']))
            if allowed and not emergency and not confirmation and self.observer is not None:
                # TAKEOFF consumed the operator ground attestation at enqueue;
                # its source-bound preflight still requires current control mode.
                if not self.yaw_trial:
                    allowed = (self.observer.permits_vertical_takeoff(now,**item['vertical_limits']) if self.vertical_trial
                               and item.get('action')=='takeoff' else self.observer.permits_control(now))
            if allowed and self.vertical_trial and not emergency and not confirmation and self.observer is None:
                allowed = False
            if not allowed:
                if item is current:
                    self._finish(item, 'cancelled', reason='send_preflight_failed')
                self.scheduled = None
            return allowed

    def sent(self, sequence, now):
        with self.lock:
            if self.scheduled is None or self.scheduled[1].seq != sequence:
                raise RuntimeError('flight_send_does_not_match_scheduled')
            item, packet = self.scheduled
            self._time(now)
            self.scheduled = None
            self.sent_count += 1
            item.update(sent_at=now, execution_confirmed=False,
                        sent_after_deadline=now >= item['expires_at'])
            if item['kind'] == 'action':
                # No action retry. Deadline starts at actual socket completion.
                if item is self.action and not self.revoked:
                    item.update(state='sent', ack_deadline=now + self.ACKNOWLEDGEMENT_SECONDS,
                                request=packet.encode().hex())
                else:
                    self._finish(item, 'sent_unconfirmed')
            else:
                self.last_control_sent = now
                if self.action is not None and item.get('parent_land_id') == self.action['id']:
                    self.action['neutral_sent'] = True
                self._finish(item, 'sent_unconfirmed')

    def receive(self, packet, now):
        with self.lock:
            self._time(now)
            if self.action is None or self.action['state'] != 'sent':
                return
            try:
                reply = parse_action_response(packet, DumlPacket.decode(bytes.fromhex(self.action['request'])))
            except (ValueError, AttributeError, TypeError):
                self.ignored += 1
                return
            self._finish(self.action, 'acknowledged' if reply.accepted else 'rejected',
                         status_code=reply.status_code, acknowledged_at=now,
                         response=packet.encode().hex(), execution_confirmed=False)

    def receipt(self, intent_id):
        with self.lock:
            values = [self.action, self.control, *reversed(self.history)]
            return deepcopy(next((item for item in values if item is not None and item['id'] == intent_id), None))

    def revoke(self):
        # An API thread cannot advance the UDP owner's clock.
        with self.lock:
            self.revoked = True
            self.ready = False
            if self.observer is not None:self.observer.invalidate()

    def cancel(self, now, reason='session_closed'):
        with self.lock:
            self._time(now)
            self.revoked = True
            self.ready = False
            if self.observer is not None:self.observer.invalidate()
            for item in (self.action, self.control):
                if item is not None:
                    self._finish(item, 'cancelled', reason=reason)
            self.scheduled = None

    def snapshot(self):
        with self.lock:
            now = self.clock()
            def projected(item):
                value = deepcopy(item)
                if value is not None and now >= value.get('ack_deadline', value['expires_at']):
                    value['state'] = 'timed_out' if 'sent_at' in value else 'expired'
                return value
            return {'route_ready': self._available(now), 'revoked': self.revoked,
                    'vertical_trial':self.vertical_trial,
                    'yaw_trial':self.yaw_trial,
                    'acquisition_trial':self.acquisition_trial,
                    'ordinary_control_ready':not (self.vertical_trial or self.yaw_trial) and self._available(now) and self.control_fresh,
                    'transport': deepcopy(self.bound_transport), 'loopback_only': self.loopback_only,
                    'observations':{'enabled':False} if self.observer is None else
                        {'enabled':True,**self.observer.snapshot(now)},
                    'action': projected(self.action), 'control': projected(self.control),
                    'history': deepcopy(list(self.history)), 'sent_count': self.sent_count,
                    'ignored_responses': self.ignored, 'flight_ready': False,
                    'execution_confirmed': False, 'supervisor_authorization_supplied': False}
