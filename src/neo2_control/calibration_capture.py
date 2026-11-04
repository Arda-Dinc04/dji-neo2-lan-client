"""Read-only, stationary-aircraft calibration capture from the local preview.

No connect, lease, camera or motor intent is sent. Epoch/generation and fresh
stationary motion are checked around every frame read. Preview encoding time
is retained as such; camera exposure timing and physical scale stay unverified.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import time

from .camera import decode_camera_fov
from .metric_reference import ReferenceBoard
from .operator_client import OperatorError
from .reconstruction import _finite,_json_file,_vision


def _fov_observation(status,now):
    observation=status.get('connection',{}).get('camera',{}).get('fov_status')
    # Older runtimes and missing pushes remain explicitly unknown. If present,
    # require current metadata rather than silently mixing optical states.
    if observation is None:return None
    if not isinstance(observation,dict):raise ValueError('calibration_fov_metadata_invalid')
    at=observation.get('received_at')
    if not _finite(at) or not 0<=now-at<=1 or observation.get('fresh') is not True:
        raise ValueError('calibration_fov_metadata_unavailable')
    try:
        payload=bytes.fromhex(observation['payload_hex'])
        decoded=decode_camera_fov(payload)
        if any(observation.get(key)!=value for key,value in decoded.items()):raise ValueError()
        revision=observation['revision']
        if type(revision) is not int or revision<1:raise ValueError()
    except (KeyError,TypeError,ValueError):
        raise ValueError('calibration_fov_metadata_invalid') from None
    return {**decoded,'received_at':at,'revision':revision,'source':observation.get('source')}


def _same_fov(first,second):
    return all((first or {}).get(key)==(second or {}).get(key) for key in ('payload_hex','revision'))


def _stationary(status, now):
    if not isinstance(status,dict):
        raise ValueError('calibration_status_invalid')
    sample=status.get('telemetry') or {}
    at=sample.get('motion_received_at')
    if (status.get('connected') is not True or status.get('possible_airborne') is not False
            or status.get('state') not in ('ready','not_ready') or status.get('synthetic') is not False
            or status.get('connection',{}).get('video',{}).get('live') is not True
            or sample.get('flying') is not False or sample.get('motors_on') is not False
            or not _finite(at) or not 0<=now-at<=.5):
        raise ValueError('calibration_requires_live_stationary_session')
    return {'owner_epoch':status.get('controller_owner',{}).get('epoch'),
        'connection_generation':status.get('connection',{}).get('generation'),
        'motion_received_at':at,'flying':False,'motors_on':False,'checked_at':now,
        'fov_info':_fov_observation(status,now)}


def _pattern(detector, image):
    cv,np=_vision()
    corners,ids,_,_=detector.detectBoard(image)
    if ids is None or len(ids)<12:
        return None,'pattern_not_found'
    ids,pixels=ids.ravel(),corners.reshape(-1,2)
    objects=detector.getBoard().getChessboardCorners()[ids]
    if (np.ptp(objects[:,:2],axis=0)<1.9).any():
        return None,'pattern_coverage_insufficient'
    if cv.contourArea(cv.convexHull(pixels.astype(np.float32)))<image.shape[0]*image.shape[1]*.01:
        return None,'pattern_too_small'
    if not np.isfinite(pixels).all():
        return None,'pattern_corners_invalid'
    return {'corner_ids':ids.tolist(),'corners_px':pixels.tolist()},None


def _distinct_view(observation, accepted, minimum_shift=12.):
    _,np=_vision()
    current=dict(zip(observation['corner_ids'],observation['corners_px']))
    for previous in accepted:
        other=dict(zip(previous['corner_ids'],previous['corners_px']))
        shared=sorted(current.keys()&other.keys())
        if len(shared)>=8:
            delta=np.array([current[i] for i in shared])-np.array([other[i] for i in shared])
            if float(np.sqrt(np.mean(np.sum(delta**2,axis=1))))<minimum_shift:
                return False
    return True


def capture_calibration(client, board, output, *, views=16, duration=120., interval=.5,
        clock=time.monotonic, wait=time.sleep, event_sink=None):
    cv,np=_vision()
    if (not isinstance(board,ReferenceBoard) or type(views) is not int or not 10<=views<=40
            or not _finite(duration) or not 5<=duration<=600
            or not _finite(interval) or not .1<=interval<=5):
        raise ValueError('calibration_capture_options_invalid')
    initial=client.status()
    context=_stationary(initial,clock())
    client.pin_connection(initial)
    output=Path(output)
    output.mkdir(mode=0o700)
    accepted=[];seen_ids=set();seen_hashes=set();size=None
    started=clock();deadline=started+duration;reason=None;last_feedback=-math.inf
    detector=cv.aruco.CharucoDetector(board.opencv_board(1.))
    event_fd=os.open(output/'events.jsonl',os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
    with os.fdopen(event_fd,'w',encoding='utf-8') as journal:
        def emit(event,**values):
            nonlocal last_feedback
            item={'event':event,'pc_monotonic':clock(),**values}
            journal.write(json.dumps(item,ensure_ascii=False,allow_nan=False)+'\n');journal.flush()
            if event_sink is not None and (event in ('accepted','finished') or clock()-last_feedback>=5):
                event_sink(item);last_feedback=clock()
        emit('started',board=board.as_dict(),context=context,target_views=views,
            acquisition_synchronization_verified=False,physical_scale_verified=False)
        try:
            while clock()<deadline and len(accepted)<views:
                before=client.status();first_state=_stationary(before,clock());client.pin_connection(before)
                if (first_state['owner_epoch'],first_state['connection_generation'])!=(
                        context['owner_epoch'],context['connection_generation']):
                    raise ValueError('calibration_connection_changed')
                if not _same_fov(first_state['fov_info'],context['fov_info']):
                    raise ValueError('calibration_fov_metadata_changed')
                try:
                    frame=client.read_frame()
                except OperatorError as error:
                    if str(error)!='frame_unavailable':raise
                    emit('skipped',reason='frame_unavailable');wait(interval);continue
                after=client.status();last_state=_stationary(after,clock());client.pin_connection(after)
                if (last_state['owner_epoch'],last_state['connection_generation'])!=(
                        context['owner_epoch'],context['connection_generation']):
                    raise ValueError('calibration_connection_changed')
                if not _same_fov(last_state['fov_info'],context['fov_info']):
                    raise ValueError('calibration_fov_metadata_changed')
                if frame.connection_generation!=context['connection_generation']:
                    raise ValueError('calibration_frame_generation_changed')
                now=clock()
                if (not _finite(frame.read_started_at) or not _finite(frame.read_finished_at)
                        or not 0<=frame.read_finished_at-frame.read_started_at<=5
                        or not 0<=now-frame.read_finished_at<=.5
                        or not _finite(frame.encoded_age_seconds) or frame.encoded_age_seconds<0):
                    raise ValueError('calibration_frame_clock_invalid')
                age_upper=frame.encoded_age_seconds+now-frame.read_started_at
                if age_upper>.75:
                    emit('skipped',reason='encoded_preview_stale',encoded_age_upper_seconds=age_upper)
                    wait(interval);continue
                digest=hashlib.sha256(frame.data).hexdigest()
                if frame.frame_id in seen_ids or digest in seen_hashes:
                    emit('skipped',reason='duplicate_frame');wait(interval);continue
                seen_ids.add(frame.frame_id);seen_hashes.add(digest)
                image=cv.imdecode(np.frombuffer(frame.data,dtype=np.uint8),cv.IMREAD_GRAYSCALE)
                if image is None or not 32<=image.shape[0]<=8192 or not 32<=image.shape[1]<=8192:
                    raise ValueError('calibration_capture_image_invalid')
                current=(image.shape[1],image.shape[0])
                if size is None:size=current
                if size!=current:raise ValueError('calibration_capture_mode_changed')
                observed,problem=_pattern(detector,image)
                if problem is not None:
                    emit('skipped',reason=problem);wait(interval);continue
                if not _distinct_view(observed,accepted):
                    emit('skipped',reason='view_too_similar');wait(interval);continue
                name=f'view-{len(accepted)+1:03d}.jpg'
                fd=os.open(output/name,os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
                with os.fdopen(fd,'wb') as stream:stream.write(frame.data)
                record={'file':name,'sha256':digest,'frame_id':frame.frame_id,
                    'pc_read_started_at':frame.read_started_at,'pc_read_finished_at':frame.read_finished_at,
                    'encoded_file_age_seconds':frame.encoded_age_seconds,
                    'encoded_age_upper_seconds_at_check':age_upper,
                    'stationary_before':first_state,'stationary_after':last_state,**observed}
                accepted.append(record)
                emit('accepted',file=name,accepted_views=len(accepted),corner_count=len(observed['corner_ids']))
                wait(min(interval,max(0.,deadline-clock())))
            reason='target_views_reached' if len(accepted)>=views else 'capture_deadline'
        except (OperatorError,ValueError,OSError) as error:
            reason=str(error)
        except KeyboardInterrupt:
            reason='capture_interrupted'
        result={'format':'neo2-control/calibration-capture/v1','output':str(output.absolute()),
            'board':board.as_dict(),'context':context,'image_size':list(size) if size else None,
            'accepted_views':len(accepted),'target_views':views,
            'target_reached':len(accepted)>=views and reason=='target_views_reached',
            'reason':reason,'elapsed_seconds':clock()-started,'sources':accepted,
            'optical_calibration_verified':False,'physical_scale_verified':False,
            'complete_optical_mode_verified':False,
            'acquisition_synchronization_verified':False,
            'scope':'Read-only JPEG capture with stationary telemetry around receipt. Encoding age is not camera exposure age. No flight or camera intents; optical fit remains a separate operation.'}
        _json_file(output/'capture.json',result)
        emit('finished',accepted_views=len(accepted),target_reached=result['target_reached'],reason=reason)
    return result
