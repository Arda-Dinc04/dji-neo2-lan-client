"""Current camera poses from source-bound decoder RGB and a frozen client map.

Call sample outside the UDP owner thread. PC byte receipt is not camera
exposure, and this tracker never supplies body navigation or flight authority.
"""
import hashlib
import time

from .reconstruction import _finite,_vision
from .video_stream import DecodedObservation,_context
from .visual_tracking import TemporalMapTracker


class DecodedPoseTracker:
    def __init__(self,reader,scan,*,clock=time.monotonic,maximum_age=.35,
            maximum_feature_pixels=640*360,region_search=False):
        self.context=_context(*reader.context)
        if not _finite(maximum_age) or not .01<=maximum_age<=.35:
            raise ValueError('decoded_tracking_maximum_age_invalid')
        self.reader,self.clock,self.maximum_age=reader,clock,maximum_age
        self.localizer=TemporalMapTracker(scan,maximum_feature_pixels=maximum_feature_pixels,
            maximum_flow_frames=4,maximum_flow_rotation_degrees=2.,region_search=region_search)
        self.current=None;self.origin=None
        self._last_time=self._last_checked=None
        self._pose_received_at=None
        self._last_index=-1;self._active=True

    def _now(self):
        now=self.clock()
        if not _finite(now) or (self._last_time is not None and now<self._last_time):
            raise ValueError('decoded_tracking_clock_invalid')
        self._last_time=now
        return now

    def _ready(self):
        if tuple(self.reader.context)!=self.context:
            raise ValueError('decoded_tracking_context_changed')
        state=self.reader.status()
        if state['error'] or state['closed'] or state['finished']:
            self.stop()
            return False
        return state['ready'] and state['worker_alive']

    def _reset(self):
        self.current=None;self._last_checked=None;self._pose_received_at=None;self.localizer.reset()

    def stop(self):
        self._active=False;self._reset()

    @property
    def current_pose(self):
        if self.current is None or self.current['pose'] is None:return None
        try:
            if (not self._ready() or self.reader.current() is None
                    or self._now()-self._pose_received_at>self.maximum_age):
                self._reset();return None
        except ValueError:self.stop();raise
        return self.current['pose']

    def sample(self):
        self.current=None
        if not self._active:raise ValueError('decoded_tracking_stopped')
        try:return self._sample()
        except ValueError:self.stop();raise

    def _sample(self):
        cv,np=_vision()
        ready=self._ready();now=self._now()
        empty={'pose':None,'checked_at':now,'map_sha256':self.localizer.map_sha256,
            'distance_unit':self.localizer.distance_unit,'distance_from_first':None,
            'displacement_from_first':None,'current_body_pose_verified':False,
            'exposure_age_bound_verified':False,'physical_accuracy_verified':False,'flight_authorized':False}
        sample=self.reader.current() if ready else None
        if sample is None:
            self._reset()
            return empty|{'state':'waiting' if self._active else 'stopped','reason':'decoder_observation_unavailable'}
        if (not isinstance(sample,DecodedObservation)
                or (sample.owner_epoch,sample.connection_generation)!=self.context
                or type(sample.decoded_index) is not int or sample.decoded_index<self._last_index
                or not all(_finite(t) for t in (sample.pc_first_byte_received_at,
                    sample.pc_first_byte_delivered_at,sample.decoded_at))
                or not 0<=sample.pc_first_byte_received_at<=sample.pc_first_byte_delivered_at<=sample.decoded_at<=now
                or not isinstance(sample.rgb,bytes)
                or hashlib.sha256(sample.rgb).hexdigest()!=sample.rgb_sha256):
            raise ValueError('decoded_tracking_observation_invalid')
        record=empty|{'source':sample.record(),
            'pc_input_age_at_check_seconds':now-sample.pc_first_byte_received_at}
        if record['pc_input_age_at_check_seconds']>self.maximum_age:
            self._reset();return record|{'state':'expired','reason':'decoded_input_stale'}
        if sample.decoded_index==self._last_index:
            return record|{'state':'waiting','reason':'duplicate_decoded_image'}
        self._last_index=sample.decoded_index
        if self._last_checked is not None and now-self._last_checked>self.maximum_age:
            self._reset()
        started=self._now();pose=problem=None
        try:pose=self.localizer.localize_rgb(sample.rgb,sample.width,sample.height)
        except ValueError as error:
            if str(error)=='tracking_rgb_image_invalid':raise
            problem=str(error)
        except cv.error:problem='decoded_tracking_localization_failed'
        ready=self._ready();latest=self.reader.current() if ready else None
        final=self._now()
        record.update(checked_at=final,localization_seconds=final-started,
            pc_input_age_at_check_seconds=final-sample.pc_first_byte_received_at)
        if not ready or latest is None:
            self._reset();return record|{'state':'waiting' if self._active else 'stopped','reason':'decoder_source_lost_during_localization'}
        if final-sample.pc_first_byte_received_at>self.maximum_age:
            self._reset();return record|{'state':'expired','reason':'decoded_input_expired_during_localization'}
        if pose is None:
            self._reset();return record|{'state':'lost','reason':problem}
        if self.origin is None:self.origin=tuple(pose['camera_center'])
        delta=np.array(pose['camera_center'])-np.array(self.origin)
        record.update(state='localized',reason=None,pose=pose,
            displacement_from_first=delta.tolist(),distance_from_first=float(np.linalg.norm(delta)))
        self._last_checked=final;self._pose_received_at=sample.pc_first_byte_received_at;self.current=record
        return record
