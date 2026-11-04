"""Camera/gimbal-only DUML producers and one bounded socket-owner mailbox.

Wire fields come from the pinned Fly producers. Local pitch limits are a
conservative first-test policy, not a measured Neo2 mechanical capability.
No flight command or arbitrary packet submission is available here.
"""
from collections import deque
from copy import deepcopy
import hashlib
import math
import secrets
import struct
import threading
import time

from .duml import DumlPacket
from .command_ingress import CommandIngress

PITCH_MINIMUM=-30
PITCH_MAXIMUM=0
ACTIONS=frozenset(('pitch','recenter','photo','record','stop_record'))
TERMINAL=frozenset(('acknowledged','sent_unconfirmed','rejected','timed_out','cancelled','expired'))


def _sequence(sequence):
    if type(sequence) is not int or not 0<=sequence<=65535:raise ValueError('invalid_camera_sequence')
    return sequence


def gimbal_pitch(degrees,sequence):
    if type(degrees) not in (int,float) or not math.isfinite(degrees) or not PITCH_MINIMUM<=degrees<=PITCH_MAXIMUM:
        raise ValueError('pitch_outside_local_limits')
    # yaw and roll ignored; absolute pitch only. SDK duration value1 encodes10
    # by trunc(duration) *10, matching the native producer's operation order.
    payload=struct.pack('<hhhBB',0,0,math.trunc(degrees*10),7,10)
    return DumlPacket(2,4,_sequence(sequence),0x40,4,0x14,payload)


def gimbal_recenter(sequence):
    # Request front-facing absolute pitch. A downward orientation was observed
    # after the earlier pitch/reset trial; native reset enum10 does not
    # establish a physical zero-degree target on this firmware.
    return gimbal_pitch(0,sequence)


def camera_recording(enabled,sequence):
    if type(enabled) is not bool:raise ValueError('record_selection_must_be_boolean')
    return DumlPacket(2,1,_sequence(sequence),0x40,2,2,bytes([enabled]))


def camera_photo(sequence):
    # Normal shooting type0 is converted to wire1 by the native helper.
    return DumlPacket(2,1,_sequence(sequence),0x40,2,1,b'\x01')


def camera_work_mode(mode,sequence):
    if type(mode) is not int or mode not in (0,1):raise ValueError('unsupported_camera_work_mode')
    return DumlPacket(2,1,_sequence(sequence),0x40,2,0x10,bytes([mode]))


def decode_camera_state(payload):
    if not isinstance(payload,bytes) or len(payload)!=60:raise ValueError('camera_state_requires_60_bytes')
    flags=int.from_bytes(payload[:4],'little')
    return {'recording_state':(flags>>6)&3,'work_mode':payload[4],
            'shutter_active':bool(flags&0x38),'capture_saved_confirmed':False}


def decode_camera_fov(payload):
    """SDK FOVInfo fields, without inferring units or a preview projection.

    UAV158 registers 02/8a; the native callback and Java serializer copy four
    signed little-endian int32 values. Direct scaling of these fields to the
    recorded 1280x720 preview fails the held-out rotation check. See
    docs/CAMERA-OPTICS.md before using them in any camera model.
    """
    if not isinstance(payload,bytes) or len(payload)!=16:
        raise ValueError('camera_fov_requires_16_bytes')
    values=struct.unpack('<iiii',payload)
    return {'sdk_fields_raw':dict(zip(('width','height','imageRatio','lensFocalLength'),values)),
            'payload_hex':payload.hex(),'payload_sha256':hashlib.sha256(payload).hexdigest(),
            'units_verified':False,'preview_projection_verified':False}


def camera_request(action,parameters,sequence):
    if not isinstance(parameters,dict):raise ValueError('parameters_object_required')
    if action=='pitch':
        if set(parameters)!={'degrees'}:raise ValueError('pitch_requires_degrees')
        return gimbal_pitch(parameters['degrees'],sequence)
    if parameters:raise ValueError('camera_action_takes_no_parameters')
    if action=='recenter':return gimbal_recenter(sequence)
    if action=='photo':return camera_photo(sequence)
    if action in ('record','stop_record'):return camera_recording(action=='record',sequence)
    raise ValueError('unsupported_camera_action')


class CameraChannel:
    """HTTP intents become packets only on the established UDP owner's thread.

    One intent may await a send/ACK, with no automatic action retries. An ACK
    reports acceptance, never successful capture or physical gimbal motion.
    Public snapshots do not advance the socket owner's monotonic clock.
    """
    def __init__(self,*,clock=time.monotonic):
        self.clock=clock
        self.lock=threading.RLock()
        self.pending=self.request=self.last_time=None
        self.ready=False
        self.revoked=False
        self.ready_at=None
        self.last_command=None
        self.history=deque(maxlen=16)
        self.commanded_pitch=None
        self.recording=None
        self.camera_observations={}
        self.camera_status=None
        self.fov_observations={}
        self.fov_status=None
        self.fov_revision=0
        self.sent_count=self.ignored=0

    def _time(self,now):
        if not math.isfinite(now) or (self.last_time is not None and now<self.last_time):
            raise ValueError('camera_time_must_be_monotonic')
        self.last_time=now
        self._expire(now)

    def _expire(self,now):
        if self.pending is not None and now>=min(self.pending['deadline'],self.pending['overall_deadline']):
            self.pending['state']='timed_out' if 'sent_at' in self.pending else 'expired'
            self._finish()

    def _advance_mode(self,now):
        if (self.pending is not None and self.pending['state']=='waiting_mode'
                and self.camera_status is not None
                and self.camera_status['received_at']>=self.pending['mode_sent_at']
                and self.camera_status['work_mode']==self.pending['desired_mode']
                and self.camera_status['recording_state']==0):
            self.pending.update(state='queued',stage='action',
                deadline=self._stage_deadline(now,2))
            self.request=None

    def _stage_deadline(self,now,seconds):
        return min(now+seconds,self.pending.get('ingress_expires_at',now+seconds))

    def _finish(self):
        self.last_command=deepcopy(self.pending)
        self.history.append(self.last_command)
        self.pending=self.request=None

    def set_ready(self,ready,now):
        if type(ready) is not bool:raise ValueError('invalid_camera_readiness')
        with self.lock:
            self._time(now)
            self.ready,self.ready_at=ready and not self.revoked,now
            if not ready and self.pending is not None and self.pending['state'] in ('queued','scheduled'):
                self.pending['state']='cancelled';self._finish()

    def submit(self,action,parameters, *, ingress=None):
        # Validate using an inert sequence before accepting a browser intent.
        camera_request(action,parameters,0)
        with self.lock:
            now=self.clock()
            if ingress is not None:
                if not isinstance(ingress, CommandIngress):raise ValueError('invalid_command_ingress')
                ingress.validate(now)
            if not self.ready or self.ready_at is None or now-self.ready_at>.5:
                raise RuntimeError('camera_not_ready')
            if self.pending is not None:raise RuntimeError('command_pending')
            if (action in ('photo','record') and self.camera_status is not None
                    and now-self.camera_status['received_at']<=1
                    and self.camera_status['recording_state']!=0):
                raise RuntimeError('camera_busy')
            self.pending={'id':secrets.token_hex(8),'action':action,'parameters':deepcopy(parameters),
                          'state':'queued','stage':'initial','created_at':now,'deadline':now+2,
                          'overall_deadline':now+5,'sequence':None}
            if ingress is not None:
                self.pending.update(deadline=min(now+2, ingress.expires_at),
                    overall_deadline=min(now+5, ingress.expires_at),
                    ingress_expires_at=ingress.expires_at)
            if action in ('photo','record'):self.pending['desired_mode']=int(action=='record')
            return {'accepted':True,'command_id':self.pending['id'],'state':'queued',
                    'message':'Commande caméra en attente d’envoi.'}

    def next_packet(self,now,next_sequence):
        with self.lock:
            self._time(now)
            if self.pending is not None and 'ingress_expires_at' in self.pending:
                self._expire(max(now,self.clock()))
            self._advance_mode(now)
            if (not self.ready or self.pending is None or self.pending['state']!='queued'
                    or self.ready_at is None or now-self.ready_at>.5):return None
            if (self.pending['action'] in ('photo','record') and self.camera_status is not None
                    and now-self.camera_status['received_at']<=1
                    and self.camera_status['recording_state']!=0):
                self.pending.update(state='cancelled',reason='camera_busy')
                self._finish()
                return None
            desired=self.pending.get('desired_mode')
            if desired is not None and self.pending['stage']=='action' and not (
                    self.camera_status is not None and now-self.camera_status['received_at']<=1
                    and self.camera_status['work_mode']==desired):
                self.pending.update(state='cancelled',reason='camera_state_changed')
                self._finish()
                return None
            if desired is not None and self.pending['stage']=='initial' and not (
                    self.camera_status is not None and now-self.camera_status['received_at']<=1
                    and self.camera_status['work_mode']==desired):
                self.request=camera_work_mode(desired,next_sequence())
                self.pending['stage']='select_mode'
            else:
                self.request=camera_request(self.pending['action'],self.pending['parameters'],next_sequence())
                self.pending['stage']='action'
            self.pending.update(state='scheduled',sequence=self.request.seq)
            return self.request

    def can_send(self,sequence,now):
        """Final socket-owner guard for a packet already scheduled by WLM."""
        with self.lock:
            self._time(now)
            # A final guard may wait for the lock, but must not rewrite the
            # socket owner's receipt clock used by its subsequent sent().
            now=max(now,self.clock())
            self._expire(now)
            return bool(not self.revoked and self.ready and self.ready_at is not None
                and 0 <= now-self.ready_at <= .5 and self.pending is not None
                and self.pending['state']=='scheduled' and self.pending['sequence']==sequence)

    def sent(self,sequence,now):
        with self.lock:
            known=self.pending is not None and self.pending['state']=='scheduled' and self.pending['sequence']==sequence
            self._time(now)
            if not known:raise RuntimeError('camera_send_does_not_match_pending')
            self.sent_count+=1
            if self.pending is None:return
            if self.revoked:
                self.pending['state']='cancelled';self._finish()
                return
            # The native stop-record producer overrides its timeout to1500ms.
            acknowledgement_seconds=1.5 if self.pending['action']=='stop_record' else 1
            self.pending.update(state='sent',sent_at=now,
                deadline=self._stage_deadline(now,acknowledgement_seconds))
            if self.pending['stage']=='select_mode':
                self.pending['mode_sent_at']=now
                return
            if self.pending['action']=='pitch':self.commanded_pitch=self.pending['parameters']['degrees']
            elif self.pending['action']=='recenter':self.commanded_pitch=0
            if self.pending['action'] in ('pitch','recenter'):
                # The native pitch action supplies no response callback and
                # completes at enqueue. Record only our actual socket send.
                self.pending.update(state='sent_unconfirmed',execution_confirmed=False)
                self._finish()

    def receive(self,packet,now):
        with self.lock:
            self._time(now)
            # Caller binds these inner frames to the current Wi-Fi peer.
            if (isinstance(packet,DumlPacket) and (
                    packet.version,packet.sender,packet.cmd_type,packet.cmd_set,packet.cmd_id)==(1,1,0,2,0x8a)
                    and packet.receiver in (2,0x1c) and len(packet.payload)==16):
                fields=decode_camera_fov(packet.payload)
                if self.fov_status is None or fields['payload_hex']!=self.fov_status['payload_hex']:
                    self.fov_revision+=1
                self.fov_status={**fields,'received_at':now,'revision':self.fov_revision,
                    'source':{'version':packet.version,'sender':packet.sender,'receiver':packet.receiver,
                        'sequence':packet.seq,'cmd_type':packet.cmd_type,
                        'cmd_set':packet.cmd_set,'cmd_id':packet.cmd_id}}
                self.fov_observations[packet.receiver]=self.fov_status
            # Measured Neo2 camera pushes have60bytes and two recipients.
            if (isinstance(packet,DumlPacket) and (
                    packet.version,packet.sender,packet.cmd_set,packet.cmd_id)==(1,1,2,0x80)
                    and packet.receiver in (2,0x1c) and packet.cmd_type in (0,0x20,0x40)
                    and len(packet.payload)==60):
                self.camera_status={**decode_camera_state(packet.payload),'received_at':now,
                    'source':{'sender':packet.sender,'receiver':packet.receiver,'sequence':packet.seq},
                    'capture_saved_confirmed':False}
                self.camera_observations[packet.receiver]=self.camera_status
                self._advance_mode(now)
            if self.pending is None or self.pending['state']!='sent' or not isinstance(packet,DumlPacket):return
            request=self.request
            if ((packet.version,packet.sender,packet.receiver,packet.cmd_set,packet.cmd_id,packet.seq)
                    !=(1,request.receiver,request.sender,request.cmd_set,request.cmd_id,request.seq)
                    or packet.cmd_type not in (0x80,0xc0) or not packet.payload):
                self.ignored+=1
                return
            status=packet.payload[0]
            if self.pending['stage']=='select_mode' and status==0:
                self.pending.update(state='waiting_mode',mode_acknowledged_at=now,
                    deadline=self._stage_deadline(now,2))
                self._advance_mode(now)
                return
            self.pending.update(state='acknowledged' if status==0 else 'rejected',status_code=status,
                                acknowledged_at=now,execution_confirmed=False)
            self._finish()

    def cancel(self,now,reason='session_closed'):
        with self.lock:
            self._time(now)
            self.ready=False
            self.revoked=True
            if self.pending is not None:
                self.pending.update(state='cancelled',reason=reason);self._finish()

    def revoke(self):
        # Browser-side cancellation cannot advance the socket owner's clock.
        with self.lock:
            self.ready=False
            self.revoked=True

    def snapshot(self):
        now=self.clock()
        with self.lock:
            current=deepcopy(self.pending or self.last_command)
            camera_status=deepcopy(self.camera_status)
            def fov_snapshot(observation):
                result=deepcopy(observation)
                if result is not None:
                    age=now-result['received_at']
                    result.update(age_seconds=max(0,age),fresh=bool(0<=age<=1 and self.ready and not self.revoked))
                return result
            if camera_status is not None:
                camera_status['age_seconds']=max(0,now-camera_status['received_at'])
                camera_status['fresh']=camera_status['age_seconds']<=1 and self.ready
            if current:
                current['age_seconds']=max(0,now-current['created_at'])
                # The owner performs expiry; detached UI state never renews it.
                if (current['state'] not in TERMINAL
                        and now>=min(current['deadline'],current['overall_deadline'])):
                    current['state']='timed_out' if 'sent_at' in current else 'expired'
                if self.revoked and current['state'] not in TERMINAL:current['state']='cancelled'
            return {'ready':self.ready and self.ready_at is not None and 0<=now-self.ready_at<=.5,
                    'capabilities':{'pitch':True,'recenter':True,'record':True,'stop_record':True,'photo':True},
                    'pitch_limits':{'minimum':PITCH_MINIMUM,'maximum':PITCH_MAXIMUM},
                    'commanded_pitch':self.commanded_pitch,
                    'recording':(camera_status['recording_state']==2
                                 if camera_status is not None and camera_status['fresh'] else None),
                    'camera_status':camera_status,
                    'camera_observations':deepcopy(list(self.camera_observations.values())),
                    'fov_status':fov_snapshot(self.fov_status),
                    'fov_observations':[fov_snapshot(value) for value in self.fov_observations.values()],
                    'last_command':current,'sent_count':self.sent_count,'ignored_frames':self.ignored,
                    'pending':self.pending is not None,'execution_confirmed':False,'flight_commands_available':False}
