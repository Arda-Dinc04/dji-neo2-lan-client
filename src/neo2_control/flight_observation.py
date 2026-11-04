"""Current-peer aircraft observations for the local flight supervisor.

The owning UDP thread supplies all inputs and timestamps. A connection-origin
distance is a PC guard, separate from DJI's distance to its home point. A
decoded value and a source binding do not establish physical navigation accuracy.
Wi-Fi control state comes from the typed PC owner, separately from received
aircraft telemetry. Radio51/04 alone cannot enable a real flight observation.
"""
from copy import deepcopy
from dataclasses import asdict, dataclass, replace
import math

from .controller import FlightObservation
from .host_control import HostControlState


@dataclass(frozen=True)
class FlightRoute:
    peer_id_sha256: str
    peer_hash: int
    own_hash: int
    nonce: int
    seen_at: float
    command_ready: bool
    product: int = 158
    host_control: HostControlState | None = None

    def __post_init__(self):
        if (not isinstance(self.peer_id_sha256, str) or len(self.peer_id_sha256) != 64
                or any(c not in '0123456789abcdef' for c in self.peer_id_sha256)):
            raise ValueError('flight route requires the observed peer identity digest')
        if any(type(v) is not int or not 0 <= v < 2**32 for v in (self.peer_hash, self.own_hash)):
            raise ValueError('flight route hashes must be uint32')
        if type(self.nonce) is not int or not 0 < self.nonce < 65536 or type(self.product) is not int or self.product != 158:
            raise ValueError('flight route requires the selected Neo2 session profile')
        if type(self.seen_at) not in (int,float) or not math.isfinite(self.seen_at) or type(self.command_ready) is not bool:
            raise ValueError('flight route time and state must be explicit')
        if self.host_control is not None and (
                not isinstance(self.host_control, HostControlState)
                or self.host_control.generation != self.generation
                or self.host_control.beacon_received_at != self.seen_at):
            raise ValueError('host control state must belong to this exact route receipt')

    @property
    def generation(self):
        return self.peer_id_sha256, self.peer_hash, self.own_hash, self.nonce


def connection_distance(origin, location):
    """Spherical horizontal metres from a received connection origin.

    The mean-radius sphere is a local approximation, not an accuracy estimate
    for the aircraft's positioning system.
    """
    latitude0, longitude0 = map(math.radians, origin)
    latitude1, longitude1 = map(math.radians, location)
    a = (math.sin((latitude1-latitude0)/2)**2
         + math.cos(latitude0)*math.cos(latitude1)*math.sin((longitude1-longitude0)/2)**2)
    return 2*6_371_000*math.asin(math.sqrt(min(1.,max(0.,a))))


class FlightObserver:
    MAXIMUM_AGE = .5
    BATTERY_MAXIMUM_AGE = 3.
    PEER_MAXIMUM_AGE = 2.
    GROUND_CONFIRMATION_SECONDS = 5.

    def __init__(self, *, synthetic=False):
        if type(synthetic) is not bool:
            raise ValueError('flight observation provenance must be boolean')
        self.synthetic = synthetic
        self.route = self.origin = self.origin_received_at = self.sample = None
        self.sources = {}
        self.problems = ['route_unavailable']
        self.last_time = None
        self.generation_started_at = None
        self.ground_epoch = 0
        self.ground_confirmed_at = self.location = self.location_received_at = None
        self._landing_identity = object()
        self._landing_epoch = self._landing_episode = 0
        self._landing_route_available = self._landing_active = False
        self._landing_started_at = None
        self._landing_flag = self._landing_motion = None
        self._landing_ambiguous = False

    def invalidate(self):
        self.invalidate_landing_route()
        self.invalidate_ground()
        self.route = self.origin = self.origin_received_at = self.sample = None
        self.generation_started_at = None
        self.location = self.location_received_at = None
        self.sources = {}
        self.problems = ['route_unavailable']

    def _end_landing_episode(self):
        if self._landing_active:
            self._landing_episode += 1
        self._landing_active = False

    def invalidate_landing_route(self, now=None):
        """Revoke opaque landing keys without changing ordinary flight policy."""
        self._landing_epoch += 1
        self._landing_route_available = False
        self._end_landing_episode()
        self._landing_started_at = now
        self._landing_flag = self._landing_motion = None
        self._landing_ambiguous = False

    def _landing_route_reason(self, now):
        route = self.route
        if type(now) not in (int,float) or not math.isfinite(now):
            reason = 'invalid_landing_confirmation_time'
        elif route is None or not 0 <= now-route.seen_at < self.PEER_MAXIMUM_AGE:
            reason = 'landing_route_unavailable_or_stale'
        elif (not route.command_ready or route.host_control is None
                or not route.host_control.permits_control(now)):
            reason = 'landing_owned_control_unavailable_or_stale'
        else:
            self._landing_route_available = True
            return None
        if self._landing_route_available:
            self.invalidate_landing_route(now)
        return reason

    def landing_route_key(self, now):
        reason = self._landing_route_reason(now)
        if reason is not None:
            raise ValueError(reason)
        # Identity is deliberately neither serialized nor copied into receipts.
        return self._landing_identity, self._landing_epoch, self.route.generation

    def _landing_reason(self, now):
        reason = self._landing_route_reason(now)
        if reason is not None:
            return reason
        flag, motion = self._landing_flag, self._landing_motion
        if self._landing_ambiguous:
            return 'landing_confirmation_conflicting_observations'
        if flag is None or not 0 <= now-flag['received_at'] <= self.MAXIMUM_AGE:
            return 'landing_confirmation_unavailable_or_stale'
        if self._value(flag, 'landing_confirmation_needed') is not True:
            return 'landing_confirmation_not_needed'
        if motion is None or not 0 <= now-motion['received_at'] <= self.MAXIMUM_AGE:
            return 'landing_motion_unavailable_or_stale'
        if (self._value(motion, 'is_flying') is not True
                or self._value(motion, 'motors_on') is not True):
            return 'landing_aircraft_not_active'
        if self._value(motion, 'flight_mode') != 12:
            return 'landing_auto_landing_mode_required'
        return None

    def _refresh_landing_episode(self, now):
        reason = self._landing_reason(now)
        if reason is not None:
            self._end_landing_episode()
        elif not self._landing_active:
            self._landing_episode += 1
            self._landing_active = True
        return reason

    def landing_confirmation_key(self, now, *, since=None):
        reason = self._refresh_landing_episode(now)
        if reason is not None:
            raise ValueError(reason)
        if since is not None:
            if (type(since) not in (int,float) or not math.isfinite(since) or since > now):
                raise ValueError('landing_intent_time_invalid')
            if min(self._landing_flag['received_at'], self._landing_motion['received_at']) < since:
                raise ValueError('landing_confirmation_evidence_precedes_intent')
        return self.landing_route_key(now), self._landing_episode

    def landing_confirmation_status(self, now):
        reason = self._refresh_landing_episode(now)
        flag, motion, route = self._landing_flag, self._landing_motion, self.route
        stamp = None if flag is None else flag['received_at']
        needed = (self._value(flag, 'landing_confirmation_needed') if flag is not None
                  and type(now) in (int,float) and math.isfinite(now)
                  and 0 <= now-stamp <= self.MAXIMUM_AGE else None)
        return {'needed': needed, 'received_at': stamp,
                'motion_received_at': None if motion is None else motion['received_at'],
                'control_received_at': (None if route is None or route.host_control is None
                                        else route.host_control.evaluated_at),
                'route_seen_at': None if route is None else route.seen_at,
                'evidence_available': reason is None, 'reason': reason}

    def _update_landing_evidence(self, snapshot, motion, now):
        if self._landing_started_at is None:
            self._landing_started_at = now
        eligible = []
        for item in (snapshot or {}).get('observations', []):
            selected = self._select({'observations':[item]}, (3,0x53), now,
                                    sender=3, receivers=(2,0x0e))
            if selected is None or selected['received_at'] < self._landing_started_at:
                continue
            try:
                body = bytes.fromhex(selected.get('raw_payload_hex', ''))
            except (TypeError, ValueError):
                continue
            value = self._value(selected, 'landing_confirmation_needed')
            if (len(body) != 4 or type(value) is not bool
                    or value != bool(int.from_bytes(body[:2], 'little') & 0x20)):
                continue
            eligible.append(selected)
        latest = max((item['received_at'] for item in eligible), default=None)
        current = [item for item in eligible if item['received_at'] == latest]
        self._landing_ambiguous = len({self._value(item,'landing_confirmation_needed') for item in current}) > 1
        self._landing_flag = deepcopy(current[-1]) if current else None
        self._landing_motion = (deepcopy(motion) if motion is not None
                                and motion['received_at'] >= self._landing_started_at else None)
        self._refresh_landing_episode(now)

    def invalidate_ground(self):
        self.ground_epoch += 1
        self.ground_confirmed_at = None
        if self.sample is not None and self.sample.grounded is True:
            self.sample = replace(self.sample,grounded=None,ground_source=None)

    def ground_confirmation_key(self, now):
        sample = self.read(now)
        if (self.route is None or not 0 <= now-self.route.seen_at < self.PEER_MAXIMUM_AGE
                or sample is None or not 0 <= now-sample.motion_received_at <= self.MAXIMUM_AGE
                or sample.flying is not False or sample.motors_on is not False):
            raise ValueError('fresh_stationary_aircraft_observation_required')
        return self.route.generation,self.ground_epoch

    def confirm_ground(self, key, now):
        if key != self.ground_confirmation_key(now):
            raise ValueError('ground_confirmation_context_changed')
        self.ground_confirmed_at = now
        self.sample = replace(self.sample,grounded=True,ground_source='operator_confirmation')
        self._seed_origin()
        if self.origin is not None and self.sample.position_valid:
            self.sample = replace(self.sample,distance_m=connection_distance(self.origin,self.location))
        self.problems = [p for p in self.problems if p != 'ground_state_unconfirmed'
                         and not (p == 'connection_origin_unavailable' and self.origin is not None)]

    def read(self, now):
        if self.route is None or not 0 <= now-self.route.seen_at < self.PEER_MAXIMUM_AGE:
            return None
        sample = self.sample
        if (sample is not None and sample.yaw_deg is not None
                and (sample.motion_received_at is None
                     or not 0 <= now-sample.motion_received_at <= self.MAXIMUM_AGE)):
            sample = replace(sample,yaw_deg=None)
        if (sample is not None and sample.control_ready is True
                and self.route.host_control is not None
                and not self.route.host_control.permits_control(now)):
            sample = replace(sample,control_ready=False)
        if (sample is not None and sample.grounded is True and (
                self.ground_confirmed_at is None or not 0 <= now-self.ground_confirmed_at < self.GROUND_CONFIRMATION_SECONDS)):
            sample = replace(sample,grounded=None,ground_source=None)
        return sample

    def _seed_origin(self):
        if (self.origin is None and self.sample is not None and self.sample.position_valid
                and self.sample.grounded is True and self.sample.control_ready is True):
            self.origin = tuple(self.location)
            self.origin_received_at = self.location_received_at

    def _select(self, snapshot, command, now, *, sender=None, receivers=None, maximum_age=None):
        selected = None
        for item in snapshot.get('observations', []):
            source = item.get('source', {})
            forwarding = source.get('forwarding', {})
            carrier = source.get('carrier', {})
            received = item.get('received_at')
            if (type(received) not in (int,float) or not math.isfinite(received)
                    or received < self.generation_started_at
                    or not 0 <= now-received <= (self.MAXIMUM_AGE if maximum_age is None else maximum_age)):
                continue
            if ((source.get('command_set'), source.get('command_id')) != command
                    or (sender is not None and source.get('sender') != sender)
                    or (receivers is not None and source.get('receiver') not in receivers)
                    or source.get('command_type') not in (0,0x20,0x40)):
                continue
            wrapped = (source.get('wifi_tunnel') is True
                    and (carrier.get('sender'),carrier.get('receiver')) == (0x27,0xee)
                    and carrier.get('command_type') in (0,0x20,0x40)
                    and forwarding.get('link_category') == 2
                    and forwarding.get('source_hash') == self.route.peer_hash
                    and forwarding.get('destination_hash') in (self.route.own_hash,0xffffffff))
            if command == (3,0x53):
                wrapped = (wrapped and source.get('command_type') == 0
                    and carrier.get('command_type') == 0
                    and forwarding.get('destination_hash') == 0xffffffff)
            direct = source.get('direct_connection', {})
            direct_profile = (
                (command == (3,0x43) and source.get('receiver') in (0xe,0xaa))
                or (command == (3,0x55) and source.get('receiver') in (2,0xe)
                    and source.get('command_type') == 0)
                or (command == (3,0x53) and source.get('receiver') in (2,0xe)
                    and source.get('command_type') == 0))
            bound_direct = (direct_profile and source.get('wifi_tunnel') is False
                    and not carrier and not forwarding
                    and direct.get('provenance') == 'owned_connected_swudp_peer_session'
                    and direct.get('generation') == list(self.route.generation)
                    and self.route.host_control is not None
                    and self.route.host_control.permits_control(now))
            if not wrapped and not bound_direct:
                continue
            if selected is None or received >= selected['received_at']:
                selected = item
        return selected

    @staticmethod
    def _value(item, name):
        value = item['fields'].get(name, {})
        return value.get('value') if value.get('valid') is True else None

    def update(self, route, telemetry, battery, wifi_control, now, *, landing_confirmation=None):
        if (type(now) not in (int,float) or not math.isfinite(now)
                or (self.last_time is not None and now < self.last_time)):
            raise ValueError('flight observation clock must be monotonic')
        self.last_time = now
        # Observe expiry before replacing old evidence with a new receipt. A
        # gap must end its episode even when nobody polled during that gap.
        self._refresh_landing_episode(now)
        if (not isinstance(route,FlightRoute) or not 0 <= now-route.seen_at < self.PEER_MAXIMUM_AGE):
            self.invalidate()
            return
        if self.route is not None and route.generation != self.route.generation:
            self.invalidate()
        if self.route is None:self.generation_started_at = now
        self.route = route
        fc = self._select(telemetry,(3,0x43),now,sender=3,receivers=(2,0xe,0xaa))
        self._update_landing_evidence(landing_confirmation,fc,now)
        power = self._select(battery,(3,0x55),now,sender=3,maximum_age=self.BATTERY_MAXIMUM_AGE)
        mode = self._select(wifi_control,(0x51,4),now)
        self.sources = {name:deepcopy(item['source']) for name,item in
                        (('flight_controller',fc),('battery',power),('wifi_control',mode)) if item is not None}
        self.problems = [name+'_unavailable_or_stale' for name,item in
                         (('flight_controller',fc),('battery',power)) if item is None]
        host = route.host_control
        if host is not None:
            self.sources['host_control'] = host.snapshot(now)
            self.problems.extend('host_control_'+reason for reason in host.problems(now))
        elif not self.synthetic:
            self.problems.append('host_control_state_unavailable')
        self.sample = None
        if fc is None:
            self.invalidate_ground()
            return
        flying = self._value(fc,'is_flying')
        motors = self._value(fc,'motors_on')
        if flying is True or motors is True:
            self.invalidate_ground()
        # True requires an explicit, current operator statement. Native activity
        # only disproves the combined "grounded and disarmed" precondition.
        active = flying is True or motors is True
        attested = (self.ground_confirmed_at is not None
                    and 0 <= now-self.ground_confirmed_at < self.GROUND_CONFIRMATION_SECONDS)
        grounded = False if active else (True if attested and flying is False and motors is False else None)
        altitude_field=fc['fields'].get('altitude',{})
        altitude = (self._value(fc,'altitude') if altitude_field.get('unit') == 'm'
                    and altitude_field.get('reference') == 'relative' else None)
        percent = self._value(power,'percent') if power is not None else None
        if power is not None and percent is None:
            self.problems.append('battery_percent_invalid')
        # The synthetic legacy fixture remains explicit. Real observations
        # require actual owner policy; received51/04 never substitutes for it.
        fixture_ready = bool(self.synthetic and host is None and mode is not None
            and self._value(mode,'wifi_mode') == 2 and self._value(mode,'service_guard') is True)
        control_ready = bool(route.command_ready and (
            host.permits_control(now) if host is not None else fixture_ready))
        control_at = host.evaluated_at if host is not None else (None if mode is None else mode['received_at'])
        if not control_ready:
            self.problems.append('wifi_control_unconfirmed')
        location = self._value(fc,'location')
        valid_location = (isinstance(location,(list,tuple)) and len(location)==2
                          and all(type(v) in (int,float) and math.isfinite(v) for v in location)
                          and abs(location[0]) <= 90 and abs(location[1]) <= 180)
        position_valid = bool(self._value(fc,'gps_valid') is True and valid_location)
        self.location = tuple(location) if valid_location else None
        self.location_received_at = fc['received_at'] if valid_location else None
        if not position_valid:
            self.problems.append('position_unavailable')
        if altitude is None:
            self.problems.append('relative_altitude_unavailable')
        if grounded is None:
            self.problems.append('ground_state_unconfirmed')
        distance = connection_distance(self.origin,location) if self.origin is not None and position_valid else None
        if distance is None:
            self.problems.append('connection_origin_unavailable')
        # Missing battery state must not hide a fresh aircraft activity
        # observation or invent its receipt time. The partial sample remains
        # ineligible for flight control until actual power data is available.
        receipts = [fc['received_at']]
        if power is not None:receipts.append(power['received_at'])
        if control_at is not None:receipts.append(control_at)
        timestamp = min(receipts)
        attitude = self._value(fc,'attitude')
        # FC body+28 i16LE * .1 is the third Attitude constructor argument;
        # its native/DEX serializer names that component yaw. This is an SDK
        # angle, without a verified north reference or physical turn sign.
        yaw = (attitude[2] if fc['fields'].get('attitude',{}).get('unit') == 'degree'
            and isinstance(attitude,(list,tuple)) and len(attitude)==3
            and type(attitude[2]) in (int,float) and math.isfinite(attitude[2])
            and -180 <= attitude[2] <= 180 else None)
        self.sample = FlightObservation(timestamp,grounded,flying,motors,percent,position_valid,
            altitude,distance,'bound_neo2_wifi',self.synthetic,control_ready=control_ready,
            ground_source='operator_confirmation' if grounded is True else None,
            motion_received_at=fc['received_at'],
            battery_received_at=power['received_at'] if power is not None else None,
            control_received_at=control_at,yaw_deg=yaw)
        self._seed_origin()
        if self.origin is not None and position_valid:
            self.sample = replace(self.sample,distance_m=connection_distance(self.origin,location))
            self.problems = [p for p in self.problems if p != 'connection_origin_unavailable']

    def permits_vertical_takeoff(self, now, *, minimum_battery_percent=20., maximum_altitude_m=1.5):
        """PC test policy, without a navigation-quality or physical-bound claim.

        The supervisor consumes the operator ground attestation at enqueue.
        The owner still requires fresh stationary FC, battery and control
        evidence at actual send. No geographic position is manufactured.
        """
        sample = self.read(now)
        return bool(sample is not None and sample.motion_received_at is not None
                    and 0 <= now-sample.motion_received_at <= self.MAXIMUM_AGE
                    and sample.battery_received_at is not None
                    and 0 <= now-sample.battery_received_at <= self.BATTERY_MAXIMUM_AGE
                    and type(sample.battery_percent) in (int,float)
                    and math.isfinite(sample.battery_percent) and minimum_battery_percent <= sample.battery_percent <= 100
                    and sample.control_received_at is not None
                    and 0 <= now-sample.control_received_at <= self.MAXIMUM_AGE
                    and self.route.host_control is not None
                    and self.route.host_control.permits_control(now)
                    and sample.control_ready is True
                    and sample.flying is False and sample.motors_on is False
                    and type(sample.altitude_m) in (int,float)
                    and math.isfinite(sample.altitude_m) and 0 <= sample.altitude_m <= maximum_altitude_m)

    def permits_control(self, now):
        sample = self.read(now)
        return bool(sample is not None and 0 <= now-sample.motion_received_at <= self.MAXIMUM_AGE
                    and sample.battery_received_at is not None
                    and 0 <= now-sample.battery_received_at <= self.BATTERY_MAXIMUM_AGE
                    and type(sample.battery_percent) in (int,float)
                    and math.isfinite(sample.battery_percent) and 0 <= sample.battery_percent <= 100
                    and sample.control_received_at is not None and 0 <= now-sample.control_received_at <= self.MAXIMUM_AGE
                    and (self.route.host_control is None or self.route.host_control.permits_control(now))
                    and sample.control_ready is True and sample.position_valid
                    and sample.altitude_m is not None and sample.distance_m is not None)

    def permits_yaw_trial(self, now, *, airborne, minimum_battery_percent=20., maximum_altitude_m=1.5):
        """Fresh current-peer evidence for the explicitly selected yaw profile."""
        sample = self.read(now)
        return bool(sample is not None and sample.motion_received_at is not None
            and 0 <= now-sample.motion_received_at <= self.MAXIMUM_AGE
            and sample.battery_received_at is not None
            and 0 <= now-sample.battery_received_at <= self.BATTERY_MAXIMUM_AGE
            and type(sample.battery_percent) in (int,float)
            and math.isfinite(sample.battery_percent) and minimum_battery_percent <= sample.battery_percent <= 100
            and sample.control_received_at is not None
            and 0 <= now-sample.control_received_at <= self.MAXIMUM_AGE
            and self.route.host_control is not None and self.route.host_control.permits_control(now)
            and sample.control_ready is True
            and sample.flying is airborne and sample.motors_on is airborne
            and (not airborne or sample.grounded is False)
            and type(sample.yaw_deg) in (int,float) and math.isfinite(sample.yaw_deg)
            and -180 <= sample.yaw_deg <= 180
            and type(sample.altitude_m) in (int,float) and math.isfinite(sample.altitude_m)
            and 0 <= sample.altitude_m <= maximum_altitude_m)

    def snapshot(self, now=None):
        now = self.last_time if now is None and self.last_time is not None else (0 if now is None else now)
        sample = self.read(now)
        problems = list(self.problems)
        sources = deepcopy(self.sources)
        if self.route is not None and self.route.host_control is not None:
            host=self.route.host_control
            sources['host_control']=host.snapshot(now)
            problems.extend('host_control_'+reason for reason in host.problems(now)
                            if 'host_control_'+reason not in problems)
        if self.route is not None and not 0 <= now-self.route.seen_at < self.PEER_MAXIMUM_AGE:
            problems.append('peer_beacon_expired')
        if sample is not None and sample.grounded is None and 'ground_state_unconfirmed' not in problems:
            problems.append('ground_state_unconfirmed')
        age = None if sample is None else now-sample.received_at
        component_ages = {} if sample is None else {
            name:None if stamp is None else now-stamp for name,stamp in
            (('motion',sample.motion_received_at),('battery',sample.battery_received_at),('control',sample.control_received_at))}
        fresh = (sample is not None and 0 <= component_ages['motion'] <= self.MAXIMUM_AGE
                 and component_ages['battery'] is not None
                 and 0 <= component_ages['battery'] <= self.BATTERY_MAXIMUM_AGE)
        return {'sample':asdict(sample) if sample is not None else None,
                'sample_age_seconds':age,'sample_fresh':fresh,
                'component_age_seconds':component_ages,
                'component_maximum_age_seconds':{'motion':self.MAXIMUM_AGE,'battery':self.BATTERY_MAXIMUM_AGE,'control':self.MAXIMUM_AGE},
                'last_sample_age_seconds':None if self.sample is None else now-self.sample.received_at,
                'sources':sources,'problems':problems,
                'route':asdict(self.route) if self.route is not None else None,
                'origin':{'location':self.origin,'received_at':self.origin_received_at,
                          'reference':'first_grounded_position_in_current_control_session'},
                'distance_reference':'connection_origin','synthetic':self.synthetic,
                'control_state_provenance':(HostControlState.SOURCE if self.route is not None
                    and self.route.host_control is not None else 'synthetic_fixture'
                    if self.synthetic else 'unavailable'),
                'ground_confirmation':{'confirmed_at':self.ground_confirmed_at,
                    'expires_at':None if self.ground_confirmed_at is None else self.ground_confirmed_at+self.GROUND_CONFIRMATION_SECONDS,
                    'source':'operator_confirmation','sensor_contact_verified':False},
                'landing_confirmation':self.landing_confirmation_status(now),
                'physical_navigation_accuracy_verified':False}
