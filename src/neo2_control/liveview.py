"""Direct native Wi-Fi keyframe request and a bounded PC request policy.

The DUML bytes follow the pinned SDK producer. Three attempts spaced one
second apart are a PC policy; sending never proves a refresh or an ACK.
"""
from copy import deepcopy
import math
import struct

from .duml import DumlPacket


def wifi_keyframe_request(ssfn,sequence):
    if type(ssfn) is not int or not 0<=ssfn<=0xffffffff:
        raise ValueError('keyframe SSFN must fit uint32')
    if type(sequence) is not int or not 0<=sequence<=0xffff:
        raise ValueError('keyframe sequence must fit uint16')
    return DumlPacket(2,0xe9,sequence,0,0x18,0x47,
                      b'\0\x08'+struct.pack('<I',ssfn)+b'\x01\0\0\0')


class WifiKeyframeRequests:
    """One socket-owner pending send, no automatic ACK or IDR inference."""
    MAXIMUM_REQUESTS=3
    INTERVAL_SECONDS=1.0

    def __init__(self):
        self.last_time=self.last_sent_at=self.next_at=None
        self.pending=None
        self.requests=[]
        self.sent_count=self.scheduled_count=self.cancelled_pending=0
        self.closed=False
        self.needed=None
        self.state,self.reason='waiting_for_video',None

    def _time(self,now):
        if not math.isfinite(now) or (self.last_time is not None and now<self.last_time):
            raise ValueError('keyframe time must be finite and monotonic')
        self.last_time=now

    def _cancel_pending(self):
        if self.pending is not None:self.cancelled_pending+=1
        self.pending=None

    def next_packet(self,now,*,needed,ssfn,next_sequence,blocked_by=None):
        self._time(now)
        if self.closed:return None
        if type(needed) is not bool:raise ValueError('keyframe need must be boolean')
        self.needed=needed
        if not needed or ssfn is None:
            self._cancel_pending()
            self.state='not_needed' if not needed else 'waiting_for_stream'
            self.reason='not_needed' if not needed else blocked_by
            return None
        self.reason=None
        if self.sent_count>=self.MAXIMUM_REQUESTS:
            self.state='limit_reached'
            return None
        if self.pending is not None:return None
        if self.next_at is not None and now<self.next_at:
            self.state='cooldown'
            return None
        packet=wifi_keyframe_request(ssfn,next_sequence())
        self.pending={'sequence':packet.seq,'ssfn':ssfn,'scheduled_at':now}
        self.scheduled_count+=1
        self.state='scheduled'
        return packet

    def sent(self,sequence,now):
        self._time(now)
        if self.closed or self.pending is None or sequence!=self.pending['sequence']:
            raise RuntimeError('keyframe send does not match pending request')
        self.requests.append({**self.pending,'sent_at':now})
        self.pending=None
        self.sent_count+=1
        self.last_sent_at=now
        self.next_at=now+self.INTERVAL_SECONDS
        self.state='limit_reached' if self.sent_count>=self.MAXIMUM_REQUESTS else 'cooldown'

    def close(self,now,reason='session_closed'):
        self._time(now)
        self._cancel_pending()
        self.closed=True
        self.state,self.reason='closed',reason

    def snapshot(self,now):
        return {'enabled':True,'state':self.state,'reason':self.reason,'needed':self.needed,
                'maximum_requests':self.MAXIMUM_REQUESTS,'interval_seconds':self.INTERVAL_SECONDS,
                'scheduled_count':self.scheduled_count,'sent_count':self.sent_count,
                'cancelled_pending':self.cancelled_pending,'pending':deepcopy(self.pending),
                'last_sent_at':self.last_sent_at,'next_allowed_at':self.next_at,
                'next_allowed_in_seconds':None if self.next_at is None else max(0,self.next_at-now),
                'requests':deepcopy(self.requests),'policy_source':'bounded_pc_policy',
                'acknowledgement_expected':False,'refresh_confirmed':False}
