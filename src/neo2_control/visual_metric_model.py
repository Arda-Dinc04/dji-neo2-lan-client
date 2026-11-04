"""Image -> estimated body pose -> SI feedback -> rendered kinematic model.

Clocks and dynamics are virtual. The supplied frame renderer sees model truth;
the route controller receives only image-derived poses and synthetic motor
state. Occupancy is an explicitly declared model, not inferred by this loop.
"""
from collections import deque
from dataclasses import asdict
import time

from .body_pose import CameraBodyProjector
from .metric_model import KinematicBody
from .metric_route import MetricRouteController, _difference, _norm, _wrap
from .reconstruction import _finite, _vision
from .visual_tracking import TemporalMapTracker


def run_visual_metric_model(scan, render_frame, mount, volume, *, ground_position,
        response_gain=1., maximum_feature_pixels=None, event_sink=None):
    """Render each exposure from the motion caused by previous image feedback.

    render_frame(body_position, body_heading) must return encoded image bytes.
    Computation wall time is reported separately from the virtual exposure and
    delivery clocks. A lost localization is delivered as a missing body pose.
    """
    if not callable(render_frame) or not _finite(response_gain) or not .4 <= response_gain <= 1.4:
        raise ValueError('visual_metric_model_configuration_invalid')
    cv, _ = _vision()
    tracker = TemporalMapTracker(scan, maximum_feature_pixels=maximum_feature_pixels,
        maximum_flow_frames=4, maximum_flow_rotation_degrees=2.)
    if tracker.distance_unit != 'm' or tracker.coordinate_frame != volume.coordinate_frame:
        raise ValueError('visual_metric_model_frame_invalid')
    projector = CameraBodyProjector(tracker.coordinate_frame, tracker.map_sha256, mount)
    controller = MetricRouteController(volume)
    body = KinematicBody(controller, ground_position=ground_position,
        coordinate_frame=tracker.coordinate_frame, response_gain=response_gain)
    observations, trajectory, transitions = [], [], []
    queue = deque()
    actions = []
    dt, pose_delay = .05, .15

    def observe(observed_at):
        started = time.perf_counter()
        raw = render_frame(body.position, body.heading)
        truth = body.pose(observed_at)
        record = {'index': len(observations), 'observed_at': observed_at,
            'delivered_at': observed_at+pose_delay, 'body_pose': None, 'reason': None,
            'model_true_position_m': list(body.position), 'model_true_heading_deg': body.heading}
        projection = None
        try:
            camera = tracker.localize_bytes(raw)
            projection = projector.project(camera, observed_at=observed_at,
                flying=truth.flying, motors_on=truth.motors_on,
                owner_epoch=truth.owner_epoch, generation=truth.generation, synthetic=True)
            record.update(body_pose=asdict(projection.body), evidence=projection.evidence,
                tracking_method=camera['tracking_method'],
                flow_frames_since_descriptors=camera['flow_frames_since_descriptors'],
                descriptor_refresh=camera['descriptor_refresh'],
                descriptor_refresh_reason=camera['descriptor_refresh_reason'],
                descriptor_matching=camera.get('descriptor_matching'),
                model_position_error_m=_norm(_difference(projection.body.position_m, body.position)),
                model_heading_error_deg=abs(_wrap(projection.body.heading_deg-body.heading)))
        except (ValueError, cv.error) as error:
            tracker.reset()
            record['reason'] = str(error) if isinstance(error, ValueError) else 'visual_model_localization_failed'
        record['render_and_localize_wall_seconds'] = time.perf_counter()-started
        observations.append(record)
        if event_sink:
            event_sink(record)
        return projection.body if projection is not None else None, record

    # Initial image is acquired before a takeoff proposal, without supplying a
    # truth pose as a fallback if localization fails.
    current, current_record = observe(100.-pose_delay)
    previous_stage = controller.stage
    for tick in range(1802):
        now = 100.+tick*dt
        if tick:
            body.advance(now, dt)
        if tick % 2 == 0:
            queue.append((now+pose_delay, *observe(now)))
        while queue and queue[0][0] <= now+1e-9:
            _, current, current_record = queue.popleft()
        demand = controller.update(now, current)
        if demand.action:
            actions.append({'at': now, 'action': demand.action})
        if controller.stage != previous_stage:
            transitions.append({'at': now, 'stage': controller.stage,
                'model_true_position_m': list(body.position), 'model_true_heading_deg': body.heading})
            previous_stage = controller.stage
        body.submit(now, demand)
        trajectory.append({'at': now, 'model_true_position_m': list(body.position),
            'model_true_heading_deg': body.heading, 'observation_index': current_record['index'],
            'demand': demand.as_dict()})
        if controller.stage in ('complete', 'aborted'):
            break
    report = controller.report()
    events = {event['stage']: event for event in transitions}
    outbound_length = return_length = return_error = None
    if 'turn_back' in events:
        outbound_length = _norm(_difference(events['turn_back']['model_true_position_m'],
            events['outbound']['model_true_position_m']))
    if 'turn_finish' in events:
        return_length = _norm(_difference(events['turn_finish']['model_true_position_m'],
            events['inbound']['model_true_position_m']))
        return_error = _norm(_difference(events['turn_finish']['model_true_position_m'],
            events['outbound']['model_true_position_m']))
    localized = [r for r in observations if r['body_pose'] is not None]
    passed = (report['complete'] and not body.motors_on and outbound_length is not None
        and abs(outbound_length-1.) <= .05 and abs(return_length-1.) <= .05 and return_error <= .05)
    return {'format': 'neo2-control/visual-metric-model/v1', 'synthetic': True,
        'scenario_passed': passed, 'feedback': report, 'actions': actions,
        'map_sha256': tracker.map_sha256, 'coordinate_frame': tracker.coordinate_frame,
        'camera_model': tracker.model.as_dict(), 'camera_mount': mount.as_dict(),
        'observation_count': len(observations), 'localized_count': len(localized),
        'lost_count': len(observations)-len(localized),
        'maximum_position_error_m': max((r['model_position_error_m'] for r in localized), default=None),
        'maximum_heading_error_deg': max((r['model_heading_error_deg'] for r in localized), default=None),
        'maximum_observation_wall_seconds': max(r['render_and_localize_wall_seconds'] for r in observations),
        'maximum_flow_frames': tracker.maximum_flow_frames,
        'maximum_flow_rotation_degrees': tracker.maximum_flow_rotation_degrees,
        'model_outbound_length_m': outbound_length, 'model_return_length_m': return_length,
        'model_return_error_m': return_error, 'model_motors_on': body.motors_on,
        'response_gain': response_gain, 'duration_s': now-100.,
        'control_interval_s': dt, 'pose_interval_s': .1, 'pose_delay_s': pose_delay,
        'command_delay_s': body.command_delay,
        'observations': observations, 'transitions': transitions, 'trajectory': trajectory,
        'real_time_verified': False, 'physical_accuracy_verified': False,
        'flight_authorized': False,
        'scope': 'Rendered exposures localized in a frozen image-derived metric atlas, projected through a declared rigid mount, and used for delayed feedback. Explicit model occupancy and virtual clocks; no real scan coverage, aircraft dynamics, timing or DJI actuation validation.'}
