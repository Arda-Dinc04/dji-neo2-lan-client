"""Peer-bound Wi-Fi service switching on the measured Neo2 WLM v5 route.

APP sends51/02, answers the peer's51/08, prepares its local Wi-Fi backend,
then completes51/06 in both directions before committing current modes.
Physical preparation and commit are explicit owner callbacks. No motor or
flight-control command is constructed here.
"""
from dataclasses import dataclass
import math
import struct

from .duml import DumlPacket
from .identity import WlmIdentity
from .wlm_discovery import ABILITY_ACK_TYPES
from .wlm_modes import WlmModes


WIFI_SERVICE_MODES = WlmModes(5,4,4,2,1)


def _uint(value,bits):
    if type(value) is not int or not 0 <= value < 1 << bits:
        raise ValueError(f'WLM switch field must fit uint{bits}')
    return value


def _device_id(value):
    if not isinstance(value,bytes) or len(value)!=23:
        raise ValueError('WLM switch identity requires exactly23bytes')
    return value


def _packet(command,sequence,payload,kind=0x40):
    return DumlPacket(sender=0xee,receiver=0xe9,seq=_uint(sequence,16),
                      cmd_type=kind,cmd_set=0x51,cmd_id=command,payload=payload)


def wifi_service_request(current_modes,sequence,*,requester_host=0x0200):
    """Native common-version5 request, with current service bytes retained.

    Cache bytes3..4 are zero on this local51/1a path. Native sequence storage
    is unwritten; the PC supplies an intentional explicit sequence.
    """
    if not isinstance(current_modes,WlmModes):
        raise ValueError('WLM switch needs validated current modes')
    body=bytes([5,4,4,0,0])+struct.pack('<H',_uint(requester_host,16))
    body+=bytes([0,2,current_modes.liveview_service,current_modes.download_service])
    return _packet(2,sequence,body)


@dataclass(frozen=True)
class SwitchPrepare:
    producer: bytes
    target: bytes
    modes: tuple[int,int,int]

    @classmethod
    def decode(cls,payload):
        # PC profile deliberately excludes the native ignored trailing bytes.
        if not isinstance(payload,bytes) or len(payload)!=49:
            raise ValueError('WLM51/08 request requires exactly49bytes')
        return cls(payload[:23],payload[23:46],tuple(payload[46:49]))


@dataclass(frozen=True)
class SwitchCompletion:
    producer: bytes
    target: bytes
    status: int
    detail: int

    @classmethod
    def decode(cls,payload):
        # Native accepts some shorter bodies and reads unwritten detail. The
        # PC accepts only the fully defined51byte producer format.
        if not isinstance(payload,bytes) or len(payload)!=51:
            raise ValueError('WLM51/06 completion requires exactly51bytes')
        return cls(payload[:23],payload[23:46],payload[46],int.from_bytes(payload[47:51],'little'))


def identity_status(payload):
    if not isinstance(payload,bytes) or len(payload)!=24:
        raise ValueError('WLM switch acknowledgement requires exactly24bytes')
    return payload[:23],payload[23]


def switch_ack(producer,command,sequence,*,status=0):
    if command not in (6,8) or type(command) is not int:
        raise ValueError('WLM switch ACK command must be6or8')
    return _packet(command,sequence,_device_id(producer)+bytes([_uint(status,8)]),0xc0)


def switch_completion(identity,peer_id,sequence,*,status=0,detail=0):
    if not isinstance(identity,WlmIdentity):
        raise ValueError('WLM completion requires an owned identity')
    body=identity.wlm_device_id+_device_id(peer_id)+bytes([_uint(status,8)])
    return _packet(6,sequence,body+struct.pack('<I',_uint(detail,32)))


@dataclass(frozen=True)
class SwitchTransmission:
    packet: DumlPacket
    kind: str


class WiFiServiceSwitch:
    """Bounded APP transaction, with one pending actual socket send.

    The native global deadline is3seconds, trigger retries400ms, completion
    retries200ms. Retries preserve the PC request sequence; the outer counter
    belongs to the transport owner and advances for each actual transmission.
    Peer identities and phase/correlation checks are stricter PC policy than
    native SN-only consumers. A transport ACK never commits service modes.
    """

    TERMINAL=frozenset(('complete','failed','cancelled'))

    def __init__(self,identity,peer_id,current_modes,next_sequence,*,prepare,commit,timeout=3):
        if not isinstance(identity,WlmIdentity) or not isinstance(current_modes,WlmModes):
            raise ValueError('WLM switch needs owned identity and current modes')
        if not all(callable(x) for x in (next_sequence,prepare,commit)):
            raise ValueError('WLM switch requires sequence, backend preparation and commit callbacks')
        if not math.isfinite(timeout) or not 0 < timeout <= 3:
            raise ValueError('WLM switch deadline must be finite and at most3seconds')
        self.identity,self.peer_id=identity,_device_id(peer_id)
        self.current_modes,self.target_modes=current_modes,WIFI_SERVICE_MODES
        self.next_sequence,self.prepare,self.commit=next_sequence,prepare,commit
        self.timeout=timeout
        self.trigger=wifi_service_request(current_modes,next_sequence())
        self.completion=None
        self.pending=self.prepare_ack=self.result_ack=None
        self.state,self.reason='prepared',None
        self.started_at=self.deadline=self.last_time=None
        self.next_trigger_at=self.next_completion_at=0
        self.next_prepare_ack_at=0
        self.prepared_local=self.committed=False
        self.own_completion_ack=self.peer_result_ack_sent=False
        self.peer_completion=None
        self.trigger_sent=self.prepare_acks_sent=self.completions_sent=self.result_acks_sent=0
        self.ignored=self.malformed=0
        self.local_failure=False
        self.completed_at=None
        self.responder_closed=False
        self.next_result_ack_at=0

    @property
    def busy(self):
        return self.state not in self.TERMINAL

    def _time(self,now):
        if not math.isfinite(now) or (self.last_time is not None and now<self.last_time):
            raise ValueError('WLM switch time must be finite and monotonic')
        self.last_time=now
        if self.busy and self.deadline is not None and now>=self.deadline:
            self._fail('switch_timeout')

    def _fail(self,reason):
        self.state,self.reason='failed',reason
        self.pending=self.prepare_ack=self.result_ack=None

    def cancel(self,now,reason='session_closed'):
        self._time(now)
        self.responder_closed=True
        self.pending=self.prepare_ack=self.result_ack=None
        if self.busy:
            self.state,self.reason='cancelled',reason

    def _maybe_commit(self):
        if (not self.busy or self.pending is not None or not self.prepared_local or self.local_failure or not self.own_completion_ack
                or self.peer_completion is None or self.peer_completion.status or not self.peer_result_ack_sent):
            return
        try:success=self.commit(self.target_modes)
        except Exception:success=False
        if success is not True:
            self._fail('local_commit_failed')
            return
        self.committed=True
        self.state='complete'
        self.completed_at=self.last_time
        self.pending=self.prepare_ack=self.result_ack=None

    def _responding(self,now):
        # Bounded PC duplicate responder: the peer may retry a completion
        # whose final ACK was lost. Never repeat the local commit callback.
        return (self.state=='complete' and not self.responder_closed
                and now-self.completed_at<3)

    def receive(self,packet,now):
        self._time(now)
        if not self.busy:
            if (self._responding(now) and isinstance(packet,DumlPacket)
                    and (packet.version,packet.sender,packet.receiver,packet.cmd_set,
                         packet.cmd_id,packet.cmd_type)==(1,0xe9,0xee,0x51,6,0x40)):
                try:result=SwitchCompletion.decode(packet.payload)
                except ValueError:
                    self.malformed+=1
                    return
                if result==self.peer_completion:
                    self.result_ack=switch_ack(result.producer,6,packet.seq,status=result.status)
                else:self.ignored+=1
            return
        if (self.started_at is None or not isinstance(packet,DumlPacket) or (
                packet.version,packet.sender,packet.receiver,packet.cmd_set)!=(1,0xe9,0xee,0x51)):
            self.ignored+=1
            return
        try:
            if packet.cmd_id==8 and packet.cmd_type==0x40:
                request=SwitchPrepare.decode(packet.payload)
                if (request.producer,request.target,request.modes)!=(self.peer_id,
                        self.identity.wlm_device_id,(5,4,4)):
                    self.ignored+=1
                    return
                self.prepare_ack=switch_ack(request.producer,8,packet.seq)
                if not self.prepared_local:self.state='prepare_ack_pending'
            elif packet.cmd_id==6 and packet.cmd_type in ABILITY_ACK_TYPES:
                own,status=identity_status(packet.payload)
                if (not self.completions_sent or own!=self.identity.wlm_device_id
                        or packet.seq!=self.completion.seq):
                    self.ignored+=1
                    return
                if status:
                    self._fail('peer_completion_ack_error')
                    return
                self.own_completion_ack=True
                self.state='awaiting_peer_completion'
                self._maybe_commit()
            elif packet.cmd_id==6 and packet.cmd_type==0x40:
                result=SwitchCompletion.decode(packet.payload)
                if (result.producer,result.target)!=(self.peer_id,self.identity.wlm_device_id):
                    self.ignored+=1
                    return
                if self.peer_completion is not None and result!=self.peer_completion:
                    self._fail('conflicting_peer_completion')
                    return
                self.peer_completion=result
                self.result_ack=switch_ack(result.producer,6,packet.seq,status=result.status)
                self.peer_result_ack_sent=False
            else:self.ignored+=1
        except ValueError:self.malformed+=1

    def next_packet(self,now):
        self._time(now)
        if self.pending is not None:return None
        if not self.busy:
            if self._responding(now) and self.result_ack is not None and now>=self.next_result_ack_at:
                self.pending=SwitchTransmission(self.result_ack,'result_ack')
                self.result_ack=None
                return self.pending
            return None
        if self.result_ack is not None and self.result_ack.payload[23]:
            self.pending=SwitchTransmission(self.result_ack,'result_ack')
            self.result_ack=None
        elif self.prepare_ack is not None and now>=self.next_prepare_ack_at:
            self.pending=SwitchTransmission(self.prepare_ack,'prepare_ack')
            self.prepare_ack=None
        elif self.prepared_local or self.local_failure:
            if self.completion is None:
                self.completion=switch_completion(self.identity,self.peer_id,self.next_sequence(),
                                                  status=int(self.local_failure))
            if not self.completions_sent:
                self.pending=SwitchTransmission(self.completion,'completion')
            elif self.result_ack is not None:
                self.pending=SwitchTransmission(self.result_ack,'result_ack')
                self.result_ack=None
            elif not self.own_completion_ack and now>=self.next_completion_at:
                self.pending=SwitchTransmission(self.completion,'completion')
        elif now>=self.next_trigger_at:
            self.pending=SwitchTransmission(self.trigger,'trigger')
        return self.pending

    def sent(self,kind,now):
        pending=self.pending
        if pending is None or pending.kind!=kind:
            raise RuntimeError('WLM switch send does not match pending transmission')
        self._time(now)
        # A known socket transmission may finish across the deadline. Its
        # bytes were sent, but the expired transaction must never commit.
        if not self.busy:
            if self.state=='complete' and kind=='result_ack':
                self.result_acks_sent+=1
                self.next_result_ack_at=now+.05
            self.pending=None
            return
        packet=self.pending.packet
        self.pending=None
        if kind=='trigger':
            if self.started_at is None:
                self.started_at=now
                self.deadline=now+self.timeout
                self.state='awaiting_prepare'
            self.trigger_sent+=1
            self.next_trigger_at=now+.4
        elif kind=='prepare_ack':
            self.prepare_acks_sent+=1
            self.next_prepare_ack_at=now+.05
            if not self.prepared_local and not self.local_failure:
                try:success=self.prepare(self.target_modes)
                except Exception:success=False
                if success is True:
                    self.prepared_local=True
                    self.state='completion_pending'
                else:
                    self.local_failure=True
                    self.reason='local_prepare_failed'
                    self.state='failure_completion_pending'
        elif kind=='completion':
            self.completions_sent+=1
            self.next_completion_at=now+.2
            if self.local_failure:self._fail('local_prepare_failed')
            else:self.state='awaiting_peer_completion' if self.own_completion_ack else 'awaiting_completion_ack'
        elif kind=='result_ack':
            self.result_acks_sent+=1
            self.next_result_ack_at=now+.05
            if packet.payload[23]:self._fail('peer_completion_error')
            else:
                self.peer_result_ack_sent=True
                self._maybe_commit()
        self._maybe_commit()

    def snapshot(self,now):
        self._time(now)
        return {'enabled':True,'state':self.state,'reason':self.reason,'busy':self.busy,
            'trigger_sent':self.trigger_sent,'prepare_acks_sent':self.prepare_acks_sent,
            'completions_sent':self.completions_sent,'result_acks_sent':self.result_acks_sent,
            'deadline_seconds':self.timeout,'deadline_started':self.started_at is not None,
            'prepared_local':self.prepared_local,'own_completion_ack':self.own_completion_ack,
            'peer_completion_received':self.peer_completion is not None,
            'peer_completion_status':None if self.peer_completion is None else self.peer_completion.status,
            'peer_completion_detail':None if self.peer_completion is None else self.peer_completion.detail,
            'peer_result_ack_sent':self.peer_result_ack_sent,'committed':self.committed,
            'target_modes':self.target_modes.metadata(),'ignored_frames':self.ignored,
            'malformed_frames':self.malformed,'application_ready':False}
