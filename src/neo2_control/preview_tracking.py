"""Session-bound camera-pose observations for a stopped-aircraft bench test.

Only status and preview GETs are used. File encoding age is not exposure age;
camera poses never acquire body navigation or flight authority here.
"""
from collections import deque
from dataclasses import dataclass,field
import hashlib
import json
import os
from pathlib import Path
import time

from .calibration_capture import _stationary
from .operator_client import OperatorError
from .reconstruction import _finite,_json_file,_vision
from .visual_tracking import TemporalMapTracker


@dataclass(frozen=True)
class PreviewObservation:
    record: dict
    image: bytes|None=field(default=None,repr=False)


class PreviewPoseTracker:
    MAXIMUM_ENCODED_AGE=.75

    def __init__(self,client,scan,*,clock=time.monotonic,maximum_feature_pixels=640*360):
        self.localizer=TemporalMapTracker(scan,maximum_feature_pixels=maximum_feature_pixels)
        self.client=client;self.clock=clock;self._last_time=None
        self.context=None;self.current=None;self.origin=None
        self._seen=deque(maxlen=600);self._last_image_hash=None;self._last_frame_at=None
        self._active=True
        self.context=self._status()

    def _now(self):
        now=self.clock()
        if not _finite(now) or (self._last_time is not None and now<self._last_time):
            raise ValueError('preview_tracking_clock_invalid')
        self._last_time=now
        return now

    def _status(self):
        status=self.client.status()
        try:state=_stationary(status,self._now())
        except ValueError as error:
            raise ValueError(str(error).replace('calibration_','preview_',1)) from None
        except (AttributeError,TypeError):raise ValueError('preview_status_invalid') from None
        self.client.pin_connection(status)
        if self.context is not None and (state['owner_epoch'],state['connection_generation'])!=(
                self.context['owner_epoch'],self.context['connection_generation']):
            raise ValueError('preview_connection_changed')
        return state

    def _reset_flow(self):
        self.localizer.reset();self._last_frame_at=None

    def stop(self):
        self._active=False;self.current=None;self._reset_flow()

    @property
    def current_pose(self):
        if self.current is None or self.current.record.get('pose') is None:return None
        try:now=self._now()
        except ValueError:self.stop();raise
        record=self.current.record
        age=record['source']['encoded_age_upper_seconds_at_check']+now-record['checked_at']
        if age>self.MAXIMUM_ENCODED_AGE:
            self.current=None;self._reset_flow()
            return None
        return record['pose']

    def sample(self):
        self.current=None
        if not self._active:raise ValueError('preview_tracking_stopped')
        try:observation=self._sample()
        except (OperatorError,ValueError):self.stop();raise
        self.current=observation
        return observation

    def _sample(self):
        cv,np=_vision()
        before=self._status()
        empty={'pose':None,'distance_from_first':None,'displacement_from_first':None,
            'distance_unit':self.localizer.distance_unit,'physical_accuracy_verified':False,
            'current_body_pose_verified':False,'flight_authorized':False}
        try:frame=self.client.read_frame()
        except OperatorError as error:
            if str(error)!='frame_unavailable':raise
            after=self._status();self._reset_flow()
            return PreviewObservation(empty|{'state':'waiting','reason':'frame_unavailable',
                'checked_at':self._now(),'stationary_before_read':before,'stationary_after_read':after})
        after=self._status();now=self._now()
        if frame.connection_generation!=self.context['connection_generation']:
            raise ValueError('preview_frame_generation_changed')
        if (not _finite(frame.read_started_at) or not _finite(frame.read_finished_at)
                or not 0<=frame.read_finished_at-frame.read_started_at<=5
                or not 0<=now-frame.read_finished_at<=.5
                or not _finite(frame.encoded_age_seconds) or frame.encoded_age_seconds<0):
            raise ValueError('preview_frame_clock_invalid')
        age=frame.encoded_age_seconds+now-frame.read_started_at
        source={'frame_id':frame.frame_id,'image_sha256':hashlib.sha256(frame.data).hexdigest(),
            'connection_generation':frame.connection_generation,'owner_epoch':before['owner_epoch'],
            'pc_read_started_at':frame.read_started_at,'pc_read_finished_at':frame.read_finished_at,
            'encoded_file_age_seconds':frame.encoded_age_seconds,
            'encoded_age_upper_seconds_at_check':age}
        record=empty|{'source':source,'stationary_before_read':before,'stationary_after_read':after}
        if age>self.MAXIMUM_ENCODED_AGE:
            self._reset_flow()
            return PreviewObservation(record|{'state':'expired','reason':'encoded_preview_stale','checked_at':now})
        if frame.frame_id in self._seen:
            # A repeated poll is not a new observation. Keep only the internal
            # pixel cache; no previous pose is exposed as current.
            return PreviewObservation(record|{'state':'waiting','reason':'duplicate_frame','checked_at':now})
        self._seen.append(frame.frame_id)
        if self._last_frame_at is not None and now-self._last_frame_at>self.MAXIMUM_ENCODED_AGE:
            self._reset_flow()
        source['same_encoded_image_as_previous']=source['image_sha256']==self._last_image_hash
        self._last_image_hash=source['image_sha256']
        started=self._now();pose=None;problem=None
        try:pose=self.localizer.localize_bytes(frame.data)
        except ValueError as error:
            if str(error) in ('reconstruction_image_or_size_invalid','tracking_encoded_image_invalid'):
                raise ValueError('preview_image_or_mode_invalid') from None
            problem=str(error)
        except cv.error:problem='preview_localization_failed'
        final=self._status();now=self._now()
        source['encoded_age_upper_seconds_at_check']=frame.encoded_age_seconds+now-frame.read_started_at
        record.update(stationary_after_localization=final,checked_at=now,
            localization_and_status_seconds=now-started)
        if source['encoded_age_upper_seconds_at_check']>self.MAXIMUM_ENCODED_AGE:
            self._reset_flow()
            return PreviewObservation(record|{'state':'expired',
                'reason':'preview_expired_during_localization'},frame.data)
        if pose is None:
            self._reset_flow()
            return PreviewObservation(record|{'state':'lost','reason':problem},frame.data)
        if self.origin is None:self.origin=tuple(pose['camera_center'])
        delta=np.array(pose['camera_center'])-np.array(self.origin)
        self._last_frame_at=frame.read_finished_at
        return PreviewObservation(record|{'state':'localized','reason':None,'pose':pose,
            'displacement_from_first':delta.tolist(),'distance_from_first':float(np.linalg.norm(delta))},frame.data)


def track_preview(client,scan,output,*,duration=60.,observations=240,interval=.25,
        clock=time.monotonic,wait=time.sleep,event_sink=None,maximum_feature_pixels=640*360):
    """Save bounded ground observations; never connect or command a device."""
    if (not _finite(duration) or not 1<=duration<=600 or type(observations) is not int
            or not 1<=observations<=600 or not _finite(interval) or not .1<=interval<=5):
        raise ValueError('preview_tracking_options_invalid')
    tracker=PreviewPoseTracker(client,scan,clock=clock,maximum_feature_pixels=maximum_feature_pixels)
    output=Path(output);output.mkdir(mode=0o700)
    fd=os.open(output/'events.jsonl',os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
    samples=[];total_bytes=0;started=tracker._now();deadline=started+duration;sink_failed=False
    with os.fdopen(fd,'w',encoding='utf-8') as journal:
        def emit(event,**fields):
            nonlocal sink_failed
            item={'event':event,'pc_monotonic':tracker._now() if tracker._active else tracker._last_time,**fields}
            journal.write(json.dumps(item,ensure_ascii=False,allow_nan=False)+'\n');journal.flush()
            if event_sink is not None and not sink_failed:
                try:event_sink(item)
                except Exception:
                    sink_failed=True
                    raise OperatorError('preview_tracking_event_sink_failed') from None
        reason=None
        try:
            emit('started',context=tracker.context,map_sha256=tracker.localizer.map_sha256,
                coordinate_frame=tracker.localizer.coordinate_frame,distance_unit=tracker.localizer.distance_unit)
            while tracker._now()<deadline and len(samples)<observations:
                observation=tracker.sample();record=observation.record
                if observation.image is not None:
                    if total_bytes+len(observation.image)>128*1024*1024:
                        raise ValueError('preview_tracking_image_budget_exhausted')
                    name=f'frame-{len(samples):04d}.jpg'
                    image_fd=os.open(output/name,os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
                    with os.fdopen(image_fd,'wb') as stream:stream.write(observation.image)
                    record=record|{'file':name};total_bytes+=len(observation.image)
                samples.append(record)
                emit('observation',**record)
                if len(samples)<observations:wait(min(interval,max(0.,deadline-tracker._now())))
            reason='observation_limit' if len(samples)>=observations else 'capture_deadline'
        except (OperatorError,ValueError,OSError) as error:reason=str(error)
        except KeyboardInterrupt:reason='preview_tracking_interrupted'
        finally:tracker.stop()
        result={'format':'neo2-control/preview-track/v1','output':str(output.absolute()),
            'map_sha256':tracker.localizer.map_sha256,'coordinate_frame':tracker.localizer.coordinate_frame,
            'distance_unit':tracker.localizer.distance_unit,'camera_model':tracker.localizer.model.as_dict(),
            'maximum_feature_pixels':maximum_feature_pixels,
            'context':tracker.context,'samples':samples,'reason':reason,
            'elapsed_seconds':max(0.,tracker._last_time-started),
            'localized_count':sum(s['state']=='localized' for s in samples),
            'lost_count':sum(s['state'] in ('lost','expired') for s in samples),
            'waiting_count':sum(s['state']=='waiting' for s in samples),'image_bytes':total_bytes,
            'physical_accuracy_verified':False,'acquisition_synchronization_verified':False,
            'optical_mode_verified':False,'current_body_pose_verified':False,'flight_authorized':False,
            'scope':'Stopped-aircraft camera observations tied to one PC owner/generation. JPEG file age is not exposure age. No aircraft extrinsics, calibrated tracking accuracy or flight authority. No connect, lease, camera or motor intents.'}
        _json_file(output/'tracking.json',result)
        emit('finished',reason=reason,localized_count=result['localized_count'],lost_count=result['lost_count'])
    return result
