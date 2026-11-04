"""Metric route feedback proposals, without a radio or an actuator adapter.

Inputs are body poses in one metric frame, NOT untransformed camera poses.
Outputs are velocity setpoints in SI units, NOT DJI normalized stick channels.
Neither an accepted model corridor nor a completed model run authorizes flight.
"""
from dataclasses import asdict, dataclass
import math
import re

from .reconstruction import _finite


def _add(a, b):
    return tuple(x+y for x, y in zip(a, b))


def _scale(a, factor):
    return tuple(x*factor for x in a)


def _dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def _norm(a):
    return math.sqrt(_dot(a, a))


def _difference(a, b):
    return _add(a, _scale(b, -1))


def _vector(value):
    if (not isinstance(value, (tuple, list)) or len(value) != 3
            or any(not _finite(v) or abs(v) > 30 for v in value)):
        raise ValueError('metric_vector_invalid')
    return tuple(float(v) for v in value)


def _unit(value):
    value = _vector(value)
    if _norm(value) < 1e-6:
        raise ValueError('metric_axis_invalid')
    return _scale(value, 1/_norm(value))


def _wrap(degrees):
    return (degrees+180) % 360-180


def _limit(value, maximum):
    return max(-maximum, min(maximum, value))


@dataclass(frozen=True)
class MetricBodyPose:
    position_m: tuple[float, float, float]
    heading_deg: float
    observed_at: float
    flying: bool
    motors_on: bool
    coordinate_frame: str
    owner_epoch: str
    generation: str
    synthetic: bool

    def __post_init__(self):
        if (not isinstance(self.position_m, tuple) or _vector(self.position_m) != self.position_m
                or not _finite(self.heading_deg) or not -180 <= self.heading_deg < 180
                or not _finite(self.observed_at) or self.observed_at < 0
                or any(type(v) is not bool for v in (self.flying, self.motors_on, self.synthetic))
                or self.flying and not self.motors_on):
            raise ValueError('metric_body_pose_invalid')
        for name, length in (('coordinate_frame', 64), ('owner_epoch', 48), ('generation', 24)):
            value = getattr(self, name)
            if not isinstance(value, str) or not re.fullmatch('[0-9a-f]{%d}' % length, value):
                raise ValueError('metric_body_pose_context_invalid')


@dataclass(frozen=True)
class MetricDemand:
    stage: str
    right_mps: float = 0.
    forward_mps: float = 0.
    up_mps: float = 0.
    yaw_dps: float = 0.
    action: str | None = None
    landing_required: bool = False
    reason: str | None = None
    flight_authorized: bool = False

    def as_dict(self):
        return asdict(self)


class MetricRouteController:
    """Metric proposals for an out/back route or a scan, one metre, and land.

    Gains/limits are model parameters, not measured aircraft response. Pose
    timestamps must share the update clock and refer to observation time.
    A stopped sequence cannot resume; its caller must handle landing through
    the real owner. No native command is sent by this class, including aborts.
    """

    def __init__(self, memory, *, zero_forward=(0., 0., 1.), up=(0., -1., 0.),
            radius_m=.15, tracking_margin_m=.05, distance_m=1., maximum_path_deviation_m=.15,
            mission='out-and-back'):
        if mission not in ('out-and-back', 'scan-advance-land'):
            raise ValueError('metric_route_mission_invalid')
        if (not _finite(radius_m) or not .05 <= radius_m <= .5
                or not _finite(tracking_margin_m) or not .01 <= tracking_margin_m <= .5
                or not _finite(distance_m) or not .1 <= distance_m <= 3
                or not _finite(maximum_path_deviation_m) or not .01 <= maximum_path_deviation_m <= .25
                or tracking_margin_m+maximum_path_deviation_m > .5):
            raise ValueError('metric_route_geometry_invalid')
        self.memory = memory
        self.up = _unit(up)
        forward = _vector(zero_forward)
        self.zero_forward = _unit(_difference(forward, _scale(self.up, _dot(forward, self.up))))
        f, u = self.zero_forward, self.up
        self.zero_right = (f[1]*u[2]-f[2]*u[1], f[2]*u[0]-f[0]*u[2], f[0]*u[1]-f[1]*u[0])
        self.radius_m, self.margin_m, self.distance_m = radius_m, tracking_margin_m, distance_m
        self.maximum_path_deviation_m = maximum_path_deviation_m
        self.mission = mission
        self.stage = 'idle'
        self.reason = None
        self.events = []
        self.route = None
        self._last_tick = self._start = self._stage_start = None
        self._last_pose = self._context = None
        self._velocity, self._yaw_rate, self._yaw = (0., 0., 0.), 0., None
        self._settled_since = None
        self._settled_samples = 0
        self._best_error = math.inf
        self._progress_at = None
        self._takeoff_proposed = False
        self._landing_observed = False
        self.ground = self.hover = self.end = self.outward_heading = None
        self._scan_start_yaw = self._scan_completed_degrees = None

    def _plan_route(self, now):
        _, forward = self.axes(self._yaw)
        self.route = self.memory.plan(position=self.hover, forward=forward, up=self.up,
            radius_m=self.radius_m, tracking_margin_m=self.margin_m+self.maximum_path_deviation_m,
            distance_m=self.distance_m)
        if self.mission == 'scan-advance-land':
            headings = []
            for heading in self.route['headings']:
                heading = dict(heading)
                if heading['model_candidate']:
                    endpoint = tuple(heading['end_position_m'])
                    landing = self.memory.assess_segment(endpoint, self._ground_below(endpoint),
                        radius_m=self.radius_m, tracking_margin_m=self.margin_m+self.maximum_path_deviation_m)
                    heading.update(landing_clearance=landing, model_candidate=landing['model_candidate'])
                headings.append(heading)
            selected = next((item for item in headings if item['model_candidate']), None)
            self.route = dict(self.route, format='neo2-control/scan-advance-route/v1',
                headings=headings, candidate=selected, steps=[] if selected is None else [
                    {'kind': 'takeoff'}, {'kind': 'rotate', 'minimum_degrees': 360},
                    {'kind': 'choose_heading_after_scan'},
                    {'kind': 'rotate', 'degrees': selected['yaw_degrees']},
                    {'kind': 'translate_measured', 'distance_m': self.distance_m,
                        'target_position_m': selected['end_position_m']},
                    {'kind': 'land', 'target_position_m': list(self._ground_below(selected['end_position_m']))}],
                scope='Model candidate after a measured scan; includes destination descent volume. No current physical clearance or DJI actuation calibration is established.')
        chosen = self.route['candidate']
        if chosen is None:
            return self._fail('no_clear_metre_heading', now)
        self.end = tuple(chosen['end_position_m'])
        self.outward_heading = self._yaw+chosen['yaw_degrees']
        self._transition('turn_outbound', now)
        return self._zero()

    def _ground_below(self, position):
        return _difference(position, _scale(self.up, _dot(_difference(position, self.ground), self.up)))

    def _landing_target(self):
        return self._ground_below(self.end if self.mission == 'scan-advance-land' else self.hover)

    def axes(self, heading):
        angle = math.radians(heading)
        forward = _add(_scale(self.zero_forward, math.cos(angle)), _scale(self.zero_right, math.sin(angle)))
        right = _add(_scale(self.zero_right, math.cos(angle)), _scale(self.zero_forward, -math.sin(angle)))
        return right, forward

    def _transition(self, stage, now):
        self.stage = stage
        self._stage_start = self._progress_at = now
        self._best_error = math.inf
        self._settled_since, self._settled_samples = None, 0
        self.events.append({'stage': stage, 'at': now,
            'position_m': list(self._last_pose.position_m) if self._last_pose else None,
            'heading_unwrapped_deg': self._yaw, 'memory_sha256': self.memory.digest})

    def _zero(self, *, action=None):
        return MetricDemand(self.stage, action=action, reason=self.reason,
            landing_required=self._takeoff_proposed and not self._landing_observed)

    def _fail(self, reason, now):
        self.reason = reason
        self._transition('aborted', now)
        return self._zero()

    def _clear(self, start, end, *, reserve=0.):
        return self.memory.assess_segment(start, end,
            radius_m=self.radius_m, tracking_margin_m=self.margin_m+reserve)['model_candidate']

    def _settled(self, condition, fresh, observed_at):
        if not condition:
            self._settled_since, self._settled_samples = None, 0
        elif fresh:
            if self._settled_since is None:
                self._settled_since = observed_at
            self._settled_samples += 1
        return (self._settled_since is not None and self._settled_samples >= 3
            and observed_at-self._settled_since >= .4-1e-9)

    def update(self, now, pose, *, memory=None):
        if self.stage in ('complete', 'aborted'):
            return self._zero()
        if not _finite(now) or now < 0:
            return self._fail('clock_invalid', self._last_tick or 0.)
        if self._last_tick is not None and not 0 < now-self._last_tick <= .2+1e-9:
            return self._fail('control_tick_gap', now)
        self._last_tick = now
        if not isinstance(pose, MetricBodyPose):
            return self._fail('body_pose_missing', now)
        if not 0 <= now-pose.observed_at <= .35+1e-9:
            return self._fail('body_pose_stale', now)
        context = (pose.coordinate_frame, pose.owner_epoch, pose.generation, pose.synthetic)
        if self._context is not None and context != self._context:
            return self._fail('body_pose_context_changed', now)
        candidate_memory = memory if memory is not None else self.memory
        if candidate_memory.coordinate_frame != pose.coordinate_frame:
            return self._fail('memory_frame_changed', now)
        self.memory = candidate_memory
        self._context = context
        fresh = self._last_pose is None or pose.observed_at > self._last_pose.observed_at
        if self._last_pose is not None:
            if pose.observed_at < self._last_pose.observed_at:
                return self._fail('body_pose_time_regressed', now)
            if not fresh and pose != self._last_pose:
                return self._fail('body_pose_time_ambiguous', now)
            if fresh:
                dt = pose.observed_at-self._last_pose.observed_at
                velocity = _scale(_difference(pose.position_m, self._last_pose.position_m), 1/dt)
                delta = _wrap(pose.heading_deg-self._last_pose.heading_deg)
                if _norm(velocity) > 1.5 or abs(delta/dt) > 180:
                    return self._fail('body_pose_jump', now)
                self._velocity, self._yaw_rate = velocity, delta/dt
                self._yaw += delta
        else:
            self._yaw = pose.heading_deg
        self._last_pose = pose
        if self._start is None:
            self._start = now
        if now-self._start > 90:
            return self._fail('mission_timeout', now)
        if self._stage_start is not None and now-self._stage_start > 25:
            return self._fail('stage_timeout', now)

        if self.stage == 'idle':
            if pose.flying or pose.motors_on:
                return self._fail('start_not_grounded', now)
            self.ground = pose.position_m
            maximum_hover = _add(self.ground, _scale(self.up, 1.2))
            if not self._clear(self.ground, maximum_hover, reserve=self.maximum_path_deviation_m):
                return self._fail('takeoff_volume_not_clear', now)
            self._transition('taking_off', now)
            self._takeoff_proposed = True
            return self._zero(action='takeoff')

        if self.stage == 'taking_off':
            displacement = _difference(pose.position_m, self.ground)
            height = _dot(displacement, self.up)
            horizontal = _difference(displacement, _scale(self.up, height))
            if height > 1.2 or height < -.05 or _norm(horizontal) > self.maximum_path_deviation_m:
                return self._fail('takeoff_outside_model_envelope', now)
            if not self._clear(pose.position_m, _add(self.ground, _scale(self.up, 1.2))):
                return self._fail('takeoff_volume_not_clear', now)
            stable = (pose.flying and pose.motors_on and height >= .4
                and _norm(self._velocity) <= .04 and abs(self._yaw_rate) <= 5)
            if self._settled(stable, fresh, pose.observed_at):
                self.hover = pose.position_m
                if self.mission == 'scan-advance-land':
                    self._scan_start_yaw = self._yaw
                    self._transition('scanning', now)
                else:
                    return self._plan_route(now)
            return self._zero()

        if self.stage == 'landing':
            target = self._landing_target()
            # This models an abstract ground reference, not surface contact or
            # the actual firmware's descent-confirmation behavior.
            if not self._clear(pose.position_m, target):
                return self._fail('landing_volume_not_clear', now)
            stopped = not pose.flying and not pose.motors_on and _norm(_difference(pose.position_m, target)) <= .05
            if self._settled(stopped, fresh, pose.observed_at):
                self._landing_observed = True
                self._transition('complete', now)
            return self._zero()

        if not pose.flying or not pose.motors_on:
            return self._fail('flight_state_lost', now)
        translating = self.stage in ('outbound', 'inbound')
        target = self.end if self.stage in ('outbound', 'turn_back') else self.hover
        scanning = self.stage == 'scanning'
        # Two degrees of approach reserve avoid accepting 358 degrees as a
        # full scan at the ordinary 2-degree settling tolerance.
        target_yaw = self._scan_start_yaw+362 if scanning else self.outward_heading
        if self.stage in ('turn_back', 'inbound'):
            target_yaw += 180
        elif self.stage == 'turn_finish':
            target_yaw += 360
        if not self._clear(pose.position_m, target):
            return self._fail('remaining_volume_not_clear', now)
        if (self.mission == 'scan-advance-land' and self.end is not None
                and not self._clear(self.end, self._landing_target())):
            return self._fail('landing_volume_not_clear', now)
        offset = _difference(pose.position_m, self.hover)
        if translating:
            direction = _scale(_difference(self.end, self.hover), 1/self.distance_m)
            along = max(0., _limit(_dot(offset, direction), self.distance_m))
            deviation = _norm(_difference(offset, _scale(direction, along)))
        else:
            deviation = _norm(_difference(pose.position_m, target))
        if deviation > self.maximum_path_deviation_m:
            return self._fail('route_deviation', now)
        error = _difference(target, pose.position_m)
        yaw_error = target_yaw-self._yaw
        error_score = _norm(error)+abs(yaw_error)/180
        if fresh and error_score < self._best_error-.002:
            self._best_error, self._progress_at = error_score, now
        if now-self._progress_at > 6:
            return self._fail('no_measured_progress', now)
        stable = (_norm(error) <= .02 and abs(yaw_error) <= 2
            and _norm(self._velocity) <= .04 and abs(self._yaw_rate) <= 5
            and (not scanning or self._yaw-self._scan_start_yaw >= 360))
        if self._settled(stable, fresh, pose.observed_at):
            if scanning:
                self._scan_completed_degrees = self._yaw-self._scan_start_yaw
                return self._plan_route(now)
            following = {'turn_outbound': 'outbound', 'outbound': 'turn_back',
                'turn_back': 'inbound', 'inbound': 'turn_finish', 'turn_finish': 'landing'}[self.stage]
            if self.stage == 'outbound' and self.mission == 'scan-advance-land':
                following = 'landing'
            if following == 'landing' and not self._clear(pose.position_m, self._landing_target()):
                return self._fail('landing_volume_not_clear', now)
            self._transition(following, now)
            return self._zero(action='land' if following == 'landing' else None)

        world_velocity = _difference(_scale(error, 1.2), _scale(self._velocity, .35))
        vertical = _dot(world_velocity, self.up)
        horizontal = _difference(world_velocity, _scale(self.up, vertical))
        if _norm(horizontal) > .25:
            horizontal = _scale(horizontal, .25/_norm(horizontal))
        world_velocity = _add(horizontal, _scale(self.up, _limit(vertical, .15)))
        right, forward = self.axes(self._yaw)
        return MetricDemand(self.stage, _dot(world_velocity, right), _dot(world_velocity, forward),
            _dot(world_velocity, self.up), _limit(1.5*yaw_error-.15*self._yaw_rate, 45),
            landing_required=True)

    def report(self):
        return {'format': 'neo2-control/metric-route-feedback/v1', 'stage': self.stage,
            'mission': self.mission, 'scan_completed_degrees': self._scan_completed_degrees,
            'complete': self.stage == 'complete', 'reason': self.reason,
            'coordinate_frame': self._context[0] if self._context else None,
            'route': self.route, 'events': list(self.events),
            'ground_position_m': self.ground, 'hover_position_m': self.hover,
            'outbound_target_m': self.end, 'outward_heading_deg': self.outward_heading,
            'landing_target_m': self._landing_target() if self.end is not None else None,
            'heading_unwrapped_deg': self._yaw,
            'runtime_clearance_margin_m': self.margin_m,
            'maximum_path_deviation_m': self.maximum_path_deviation_m,
            'planning_clearance_margin_m': self.margin_m+self.maximum_path_deviation_m,
            'physical_accuracy_verified': False, 'current_body_pose_verified': False,
            'actuator_mapping_verified': False, 'flight_authorized': False,
            'scope': 'Metric body-pose feedback and SI velocity proposals only. No DJI stick mapping, aircraft timing, scene validity or real flight authority is established.'}
