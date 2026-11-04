"""Delayed kinematic fixture for metric feedback; no device or network I/O.

The volume below is explicitly hand-declared model occupancy. It is not a
scan, and the motion equations are not DJI flight dynamics or floor contact.
"""
from collections import deque
from dataclasses import replace
import hashlib
import math

from .metric_route import (
    MetricBodyPose, MetricDemand, MetricRouteController, _add, _difference,
    _dot, _limit, _norm, _scale, _wrap,
)
from .reconstruction import _finite, _vision
from .spatial_memory import FREE, OCCUPIED, _ClearanceGrid

FRAME = hashlib.sha256(b'neo2-control/explicit-kinematic-model/v1').hexdigest()
SCENARIOS = ('normal', 'stalled', 'pose-loss', 'owner-change', 'obstacle')


class ModelVolume(_ClearanceGrid):
    """An explicit synthetic scene; deliberately not a persisted scan format."""

    def __init__(self, *, obstacle_position=None, origin=(0., 0., 0.), coordinate_frame=FRAME):
        _, np = _vision()
        origin = np.asarray(origin, dtype=float)
        if origin.shape != (3,) or not np.isfinite(origin).all() or not isinstance(coordinate_frame, str):
            raise ValueError('metric_model_origin_invalid')
        lower, upper, resolution = origin+[-1.5, -1.6, -1.5], origin+[1.5, .5, 1.5], .1
        shape = np.rint((upper-lower)/resolution).astype(int)
        centers = lower+(np.indices(tuple(shape)).reshape(3, -1).T+.5)*resolution
        # A wall ahead forces a nonzero initial turn. Ground-reference volume
        # is abstract FREE here; no floor contact/launch geometry is claimed.
        local = centers-origin
        occupied = (np.abs(local[:, 0]) < .4) & (local[:, 2] >= .5) & (local[:, 2] < .9)
        if obstacle_position is not None:
            occupied |= np.linalg.norm(centers-np.asarray(obstacle_position), axis=1) <= .18
        states = np.where(occupied, OCCUPIED, FREE).astype(np.uint8).reshape(tuple(shape))
        digest = hashlib.sha256(b'explicit-model-occupancy/v1'+states.tobytes()+origin.tobytes()
            +coordinate_frame.encode()).hexdigest()
        super().__init__(states, lower, upper, resolution, coordinate_frame, digest)


class KinematicBody:
    def __init__(self, controller, *, response_gain=1., command_delay=.1,
            ground_position=(0., 0., 0.), coordinate_frame=FRAME):
        self.controller = controller
        self.gain, self.command_delay = response_gain, command_delay
        self.ground_position = tuple(float(v) for v in ground_position)
        self.coordinate_frame = coordinate_frame
        self.position, self.velocity = self.ground_position, (0., 0., 0.)
        self.heading = self.yaw_rate = 0.
        self.motors_on = False
        self.native = None
        self.pending = deque()
        self.demand = MetricDemand('idle')

    def submit(self, now, demand):
        self.pending.append((now+self.command_delay, demand))

    def advance(self, now, dt):
        while self.pending and self.pending[0][0] <= now+1e-9:
            _, self.demand = self.pending.popleft()
            if self.demand.action == 'takeoff':
                self.native, self.motors_on = 'takeoff', True
            elif self.demand.action == 'land':
                self.native = 'land'
        up = self.controller.up
        height = _dot(_difference(self.position, self.ground_position), up)
        if self.native:
            height_error = (.65 if self.native == 'takeoff' else 0.)-height
            vertical = _limit(2*height_error, .35)
            desired = _scale(up, vertical)
            yaw = 0.
            if abs(height_error) < .004 and _norm(self.velocity) < .015:
                if self.native == 'land':
                    self.position = _difference(self.position, _scale(up, height))
                    self.motors_on = False
                self.native = None
        elif self.motors_on:
            right, forward = self.controller.axes(self.heading)
            desired = _add(_add(_scale(right, self.demand.right_mps*self.gain),
                _scale(forward, self.demand.forward_mps*self.gain)), _scale(up, self.demand.up_mps))
            yaw = self.demand.yaw_dps
        else:
            desired, yaw = (0., 0., 0.), 0.
        alpha = 1-math.exp(-dt/.2)
        self.velocity = _add(self.velocity, _scale(_difference(desired, self.velocity), alpha))
        self.yaw_rate += (yaw-self.yaw_rate)*alpha
        self.position = _add(self.position, _scale(self.velocity, dt))
        self.heading += self.yaw_rate*dt
        if not self.motors_on:
            self.position = _difference(self.position, _scale(up,
                _dot(_difference(self.position, self.ground_position), up)))

    def pose(self, observed_at):
        return MetricBodyPose(self.position, _wrap(self.heading), observed_at,
            self.motors_on and _dot(_difference(self.position, self.ground_position), self.controller.up) > .02,
            self.motors_on, self.coordinate_frame, 'c'*48, 'd'*24, True)


def simulate_metric_route(*, scenario='normal', response_gain=1., mission='out-and-back'):
    if scenario not in SCENARIOS or not _finite(response_gain) or not .4 <= response_gain <= 1.4:
        raise ValueError('metric_model_configuration_invalid')
    controller = MetricRouteController(ModelVolume(), mission=mission)
    body = KinematicBody(controller, response_gain=0. if scenario == 'stalled' else response_gain)
    now, dt, pose_delay = 100., .05, .15
    current = body.pose(now-pose_delay)
    poses = deque()
    trajectory = []
    outbound_at = None
    fault_applied = False
    actions = []
    for tick in range(1802):
        now = 100.+tick*dt
        if tick:
            body.advance(now, dt)
        if tick % 2 == 0:
            poses.append((now+pose_delay, body.pose(now)))
        while poses and poses[0][0] <= now+1e-9:
            _, current = poses.popleft()
        if controller.stage == 'outbound' and outbound_at is None:
            outbound_at = now
        trigger = outbound_at is not None and now-outbound_at >= .7
        if trigger and not fault_applied and scenario == 'obstacle':
            obstacle = _add(controller.hover, _scale(_difference(controller.end, controller.hover), .6))
            controller.memory = ModelVolume(obstacle_position=obstacle)
            fault_applied = True
        sample = current
        if trigger and scenario == 'pose-loss':
            sample, fault_applied = None, True
        elif trigger and scenario == 'owner-change':
            sample, fault_applied = replace(current, owner_epoch='e'*48), True
        demand = controller.update(now, sample)
        if demand.action:
            actions.append({'at': now, 'action': demand.action})
        body.submit(now, demand)
        trajectory.append({'at': now, 'position_m': body.position, 'heading_deg': body.heading,
            'pose_observed_at': sample.observed_at if sample else None,
            'demand': demand.as_dict()})
        if controller.stage in ('complete', 'aborted'):
            break
    report = controller.report()
    events = {event['stage']: event for event in controller.events}
    outbound_error = return_error = heading_error = None
    if 'turn_back' in events:
        outbound_error = _norm(_difference(events['turn_back']['position_m'], controller.end))
    elif mission == 'scan-advance-land' and 'landing' in events:
        outbound_error = _norm(_difference(events['landing']['position_m'], controller.end))
    if 'turn_finish' in events:
        return_error = _norm(_difference(events['turn_finish']['position_m'], controller.hover))
    if controller.outward_heading is not None:
        heading_error = abs(controller.outward_heading+(360 if mission == 'out-and-back' else 0)-body.heading)
    expected_abort = {'stalled': 'no_measured_progress', 'pose-loss': 'body_pose_missing',
        'owner-change': 'body_pose_context_changed', 'obstacle': 'remaining_volume_not_clear'}
    passed = (report['complete'] and outbound_error is not None and outbound_error <= .03
        and (return_error is not None and return_error <= .03 if mission == 'out-and-back'
            else report['scan_completed_degrees'] is not None and report['scan_completed_degrees'] >= 360)
        and heading_error <= 2
        and not body.motors_on) if scenario == 'normal' else (
            report['stage'] == 'aborted' and report['reason'] == expected_abort[scenario])
    return {'format': 'neo2-control/metric-route-model/v1', 'synthetic': True,
        'mission': mission,
        'scenario': scenario, 'scenario_passed': passed, 'response_gain': response_gain,
        'control_interval_s': dt, 'pose_interval_s': .1, 'pose_delay_s': pose_delay,
        'command_delay_s': body.command_delay, 'response_time_constant_s': .2,
        'outbound_endpoint_error_m': outbound_error, 'return_endpoint_error_m': return_error,
        'final_heading_error_deg': heading_error, 'duration_s': now-100.,
        'model_motors_on': body.motors_on, 'actions': actions, 'feedback': report,
        'trajectory': trajectory, 'flight_authorized': False,
        'scope': 'Explicit synthetic occupancy and delayed kinematics. True positions are supplied as model observations, not recovered from pixels. No aircraft dynamics, actuation calibration or real clearance is verified.'}
