"""Measured yaw progress for one explicit rotation; this opens no connection.

Normalized stick values are not degrees/second. Completion comes from fresh,
same-generation FC yaw observations, including signed-angle wrap, never from
time spent sending the stick. The caller must LAND on any invalid observation.
"""
import math
import re

from .flight import MAX_YAW_CHANNEL_LIMIT


class YawRotation:
    TARGET_DEGREES=360.
    MAXIMUM_SAMPLE_AGE=.5
    MAXIMUM_RATE_DEGREES_PER_SECOND=180.

    def __init__(self,generation):
        if not isinstance(generation,str) or not re.fullmatch('[0-9a-f]{24}',generation):
            raise ValueError('rotation_generation_required')
        self.generation=generation
        self.last_yaw=self.last_received_at=None
        self.net_degrees=0.
        self.direction=None
        self.peak_progress=0.
        self.samples=0

    def observe(self,yaw_degrees,received_at,now,*,generation):
        if generation!=self.generation:raise ValueError('rotation_generation_changed')
        if (any(type(v) not in (int,float) or not math.isfinite(v)
                for v in (yaw_degrees,received_at,now))
                or not -180<=yaw_degrees<=180):raise ValueError('rotation_yaw_invalid')
        if not 0<=now-received_at<=self.MAXIMUM_SAMPLE_AGE:
            raise ValueError('rotation_yaw_stale')
        if self.last_received_at is not None:
            dt=received_at-self.last_received_at
            if dt<0:raise ValueError('rotation_yaw_reordered')
            if dt==0:
                if yaw_degrees!=self.last_yaw:raise ValueError('rotation_yaw_ambiguous')
                return self.snapshot()
            if dt>self.MAXIMUM_SAMPLE_AGE:raise ValueError('rotation_observation_gap')
            delta=(yaw_degrees-self.last_yaw+180.)%360.-180.
            if abs(delta)>self.MAXIMUM_RATE_DEGREES_PER_SECOND*dt+3.:
                raise ValueError('rotation_yaw_discontinuity')
            self.net_degrees+=delta
            if self.direction is None and abs(self.net_degrees)>=2.:
                self.direction=1 if self.net_degrees>0 else -1
            progress=self.progress_degrees
            if self.peak_progress-progress>12.:raise ValueError('rotation_progress_reversed')
            if progress>self.TARGET_DEGREES+15.:raise ValueError('rotation_target_overshot')
            self.peak_progress=max(self.peak_progress,progress)
        self.last_yaw,self.last_received_at=yaw_degrees,received_at
        self.samples+=1
        return self.snapshot()

    @property
    def progress_degrees(self):
        return 0. if self.direction is None else max(0.,self.net_degrees*self.direction)

    @property
    def complete(self):
        return self.progress_degrees>=self.TARGET_DEGREES

    def channels(self,maximum_channel=200,*,minimum_channel=40):
        if type(maximum_channel) is not int or not 1<=maximum_channel<=MAX_YAW_CHANNEL_LIMIT:
            raise ValueError('rotation_channel_limit_invalid')
        if type(minimum_channel) is not int or not 40<=minimum_channel<=MAX_YAW_CHANNEL_LIMIT:
            raise ValueError('rotation_minimum_channel_invalid')
        if self.complete:return (0,0,0,0)
        remaining=self.TARGET_DEGREES-self.progress_degrees
        # Positive stick follows the native UI's turn-right label. Its actual
        # angular rate and relation to the yaw sign are deliberately unclaimed.
        return (0,0,0,min(maximum_channel,max(minimum_channel,round(remaining*4))))

    def snapshot(self):
        return {'target_degrees':self.TARGET_DEGREES,'progress_degrees':self.progress_degrees,
            'signed_degrees':self.net_degrees,'direction':self.direction,
            'complete':self.complete,'samples':self.samples,'last_received_at':self.last_received_at}
