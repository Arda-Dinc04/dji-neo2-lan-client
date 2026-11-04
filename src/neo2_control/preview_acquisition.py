"""Read-only, generation-bound input for the progressive scene builder."""
import math
import time
from .operator_client import PreviewFrame


class ReadOnlyPreviewFeed:
    def __init__(self,client,expected_context,*,clock=time.monotonic,maximum_age=.35):
        self.client,self.expected,self.clock=client,tuple(expected_context),clock
        if len(self.expected)!=2 or not 0<maximum_age<=.35:
            raise ValueError('acquisition_preview_configuration_invalid')
        self.maximum_age=maximum_age;self.last_frame_id=None

    def _check(self,status):
        context=(status.get('controller_owner',{}).get('epoch'),status.get('connection',{}).get('generation'))
        if context!=self.expected:raise ValueError('acquisition_source_generation_changed')
        if (status.get('connected') is not True
                or status.get('connection',{}).get('video',{}).get('live') is not True):
            raise ValueError('acquisition_live_source_unavailable')

    def read(self):
        before=self.client.status();self._check(before)
        frame=self.client.read_frame()
        self._check(self.client.status())
        now=self.clock()
        if (not isinstance(frame,PreviewFrame) or frame.connection_generation!=self.expected[1]
                or not all(type(v) in (int,float) and math.isfinite(v) for v in
                    (now,frame.encoded_age_seconds,frame.read_started_at,frame.read_finished_at))
                or not 0<=frame.read_started_at<=frame.read_finished_at<=now
                or frame.encoded_age_seconds<0):
            raise ValueError('acquisition_preview_evidence_invalid')
        age=frame.encoded_age_seconds+now-frame.read_started_at
        if age>self.maximum_age or frame.frame_id==self.last_frame_id:return None
        self.last_frame_id=frame.frame_id
        result={'raw':frame.data,'received_at':frame.read_finished_at,'metadata':{
            'frame_id':frame.frame_id,'encoded_age_seconds':frame.encoded_age_seconds,
            'preview_availability_age_bound_seconds':age,
            'read_started_at':frame.read_started_at,'read_finished_at':frame.read_finished_at,
            'scope':'preview file availability and receipt; camera exposure age unverified'}}
        from .room_altitude import status_altitude_observation
        altitude=status_altitude_observation(before)
        if altitude is not None and altitude['available_at']<=frame.read_finished_at:
            result['metadata']['altitude_observation']=altitude
        return result
