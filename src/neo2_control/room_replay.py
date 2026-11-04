"""A visual replay in the recorded SfM frame, never a new flight program.

Vertical scale is fitted to unverified FC relative altitude. Camera/body
extrinsics and camera exposure synchronization remain unverified.
"""
from pathlib import Path
import json
import math


def unwrap_degrees(values):
    if not values:return []
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
        raise ValueError('room_replay_yaw_invalid')
    result=[float(values[0])]
    for previous,current in zip(values,values[1:]):
        result.append(result[-1]+(current-previous+180)%360-180)
    return result


def axis_rotation(axis,degrees):
    import numpy as np
    axis=np.asarray(axis,dtype=float)
    if axis.shape!=(3,) or not np.isfinite(axis).all() or np.linalg.norm(axis)<1e-9:
        raise ValueError('room_replay_axis_invalid')
    axis=axis/np.linalg.norm(axis);x,y,z=axis
    cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    angle=math.radians(degrees)
    return math.cos(angle)*np.eye(3)+(1-math.cos(angle))*np.outer(axis,axis)+math.sin(angle)*cross


def build_replay(poses_path,mission_directory,session_events):
    import numpy as np
    from .room_mvs import digest
    poses_path,mission_directory,session_events=map(Path,(poses_path,mission_directory,session_events))
    native=json.loads(poses_path.read_text())
    poses=native['poses']
    if native['format']!='neo2-control/room-poses/v1' or not 3<=len(poses)<=96:
        raise ValueError('room_replay_poses_invalid')
    events=[json.loads(line) for line in (mission_directory/'events.jsonl').read_text().splitlines()]
    result=json.loads((mission_directory/'mission-result.json').read_text())
    if not result['motion_complete'] or len(result['scan']['completed'])!=2:
        raise ValueError('room_replay_requires_double_rotation')
    start=next(e['monotonic'] for e in events if e['event']=='takeoff_requested')
    land=next(e['monotonic'] for e in events if e['event']=='land_requested')
    stop=json.loads((mission_directory/'stopped-telemetry.json').read_text())['telemetry']['motion_received_at']
    if not start<land<stop<=start+120:raise ValueError('room_replay_flight_times_invalid')
    groups=[]
    for level in (0,1):
        group=[p for p in poses if p['selection']['selection_target']['level']==level
            and not p['selection']['selection_target'].get('intermediate_climb',False)]
        if len(group)<3:raise ValueError('room_replay_level_poses_missing')
        groups.append(group)
    centers=[np.mean([p['center'] for p in group],axis=0) for group in groups]
    heights=[np.mean([p['selection']['selection_target']['event']['altitude_m'] for p in group]) for group in groups]
    axis=centers[1]-centers[0];separation=np.linalg.norm(axis)
    if separation<1e-6 or heights[1]-heights[0]<.2:raise ValueError('room_replay_levels_degenerate')
    up=axis/separation;units_per_fc_metre=separation/(heights[1]-heights[0])
    intercept=centers[0]@up-heights[0]*units_per_fc_metre
    knots=sorted(poses,key=lambda p:p['selection']['picture']['last_byte_delivered_monotonic'])
    times=np.array([p['selection']['picture']['last_byte_delivered_monotonic'] for p in knots])
    locations=np.array([p['center'] for p in knots])
    first=knots[0];base_R=np.array(first['R']).T
    first_yaw=first['selection']['selection_target']['event']['yaw_deg']
    origins=[];raw_samples=[]
    with session_events.open() as stream:
        for line in stream:
            event=json.loads(line)
            if event.get('event')!='telemetry_updated':continue
            telemetry=event['telemetry']
            for observation in telemetry.get('observations',[]):
                if observation.get('received_at') is not None and observation.get('age_seconds') is not None:
                    origins.append(observation['received_at']+observation['age_seconds']-event['elapsed_seconds'])
            fields=telemetry.get('fields',{})
            altitude=fields.get('altitude',{});attitude=fields.get('attitude',{})
            if (altitude.get('valid') is True and attitude.get('valid') is True
                    and altitude.get('age_seconds',1)<=.5 and attitude.get('age_seconds',1)<=.5):
                raw_samples.append((event['elapsed_seconds'],altitude['value'],attitude['value'][2]))
    if not origins or max(origins)-min(origins)>1e-6:
        raise ValueError('room_replay_pc_clock_mismatch')
    origin=origins[0];selected={}
    for elapsed,altitude,yaw in raw_samples:
        received=origin+elapsed
        if start<=received<=stop and 0<=altitude<=1.8:
            selected[int((received-start)/.075)]=(received,altitude,yaw)
    raw=sorted(selected.values())
    if len(raw)<50:raise ValueError('room_replay_telemetry_insufficient')
    yaw_unwrapped=unwrap_degrees([r[2] for r in raw]);samples=[]
    for (received,altitude,yaw),unwrapped in zip(raw,yaw_unwrapped):
        location=np.array([np.interp(received,times,locations[:,axis]) for axis in range(3)])
        location+=up*(intercept+altitude*units_per_fc_metre-location@up)
        rotation=axis_rotation(up,-(yaw-first_yaw))@base_R
        samples.append({'seconds':received-start,'position':location.tolist(),
            'world_from_camera_R':rotation.tolist(),'FC_relative_altitude_m':altitude,
            'yaw_degrees':yaw,'unwrapped_yaw_degrees':unwrapped,
            'position_source':'SfM lateral interpolation and fitted FC altitude'})
    intervals=[]
    for level in (0,1):
        progress=[e for e in events if e.get('event')=='scan_progress'
            and e.get('phase')=='rotate' and e.get('level')==level]
        completed=result['scan']['completed'][level]['rotation']
        intervals.append({'level':level,'start_seconds':progress[0]['monotonic']-start,
            'end_seconds':completed['last_received_at']-start,
            'tracked_rotation_degrees':completed['progress_degrees']})
    return {'format':'neo2-control/room-flight-replay/v1','source':native['source'],
        'poses_sha256':digest(poses_path),'events_sha256':digest(mission_directory/'events.jsonl'),
        'session_events_sha256':digest(session_events),'start_pc_monotonic':start,
        'duration_seconds':stop-start,'land_requested_seconds':land-start,
        'rotation_intervals':intervals,'samples':samples,'up_axis':up.tolist(),
        'model_units_per_FC_metre':float(units_per_fc_metre),
        'scale_source':'fit to unverified FC relative altitude; not physical calibration',
        'position_scope':'Camera trajectory reconstructed in SfM frame; takeoff and descent use fitted FC altitude.',
        'camera_body_extrinsics_verified':False,'exposure_synchronization_verified':False,
        'metric_scale_verified':False,'physical_accuracy_verified':False,'flight_authorized':False}
