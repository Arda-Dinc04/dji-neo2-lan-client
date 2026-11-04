"""One explicit, bounded read of WLM module capabilities on the Neo2 route.

The request advertises no capability vector. It is separate from ability
negotiation and does not enable video, control or application readiness.
"""

import hashlib
import math
import struct

from .duml import DumlPacket
from .duml_stream import DumlStreamDecoder
from .identity import WlmIdentity
from .wlm_wire import wifi_forward
from .wlm_discovery import (ABILITY_ACK_TYPES, AbilityAcknowledgement, CapabilityVector,
                            WlmBeacon, unpack_forwarded)


def _uint(value, bits):
    if type(value) is not int or not 0 <= value < 1 << bits:
        raise ValueError(f'WLM query field must be uint{bits}')
    return value


def capability_query(own_hash, sequence, forwarding_sequence):
    """Native query13bytes, Wi-Fi agent envelope, canonical zeroed tail.

    The final7 attribute bytes are indeterminate in the native dispatcher;
    zeroing them is an explicit PC choice. Only the Wi-Fi link is selected.
    """
    own_hash, sequence, forwarding_sequence = (_uint(own_hash,32), _uint(sequence,16),
                                               _uint(forwarding_sequence,32))
    body = b'\x20\x01\0'+struct.pack('<I',own_hash)+b'\x04\0\x06\0\x0a\0'
    inner = DumlPacket(sender=0xee,receiver=0xe9,seq=sequence,cmd_type=0x40,
                       cmd_set=0x51,cmd_id=0x41,payload=body)
    return wifi_forward(inner,own_hash,forwarding_sequence)


def _peer_hash(device_id):
    value = 0
    for byte in device_id[3:].split(b'\0',1)[0]+b'_0':
        value = (value*256+byte) % 0xfffffffb
    return value


class WlmCapabilityRead:
    """One query prepared from a current validated Neo2 beacon, no retries.

    The caller owns discovery freshness and actual socket transmission. Only
    sent() starts the response deadline. Transport ACKs never complete a read.
    """

    def __init__(self, identity, beacon, *, sequence, forwarding_sequence, timeout=1.5):
        if not isinstance(identity,WlmIdentity) or not isinstance(beacon,WlmBeacon):
            raise ValueError('WLM query requires an owned identity and validated beacon')
        metadata = beacon.metadata()
        if (metadata['device_type'],metadata['device_index'],metadata['device_model']) != (0,0,36):
            raise ValueError('WLM query is limited to the measured Neo2 peer')
        if not math.isfinite(timeout) or not 0 < timeout <= 5:
            raise ValueError('WLM query timeout must be finite and at most5seconds')
        self.request = capability_query(identity.wlm_device_hash,sequence,forwarding_sequence)
        self.sequence, self.timeout = sequence, timeout
        self.peer_hash = _peer_hash(beacon.device_id)
        self.peer_fingerprint = hashlib.sha256(beacon.device_id).hexdigest()
        self.decoder = DumlStreamDecoder()
        self.state = 'prepared'
        self.sent_at = self.received_at = self.last_time = None
        self.status = None
        self.cancel_reason = None
        self.capabilities = None
        self.ignored = self.malformed = 0

    def _time(self,now):
        if not math.isfinite(now) or (self.last_time is not None and now < self.last_time):
            raise ValueError('WLM query time must be finite and monotonic')
        self.last_time = now
        if self.state == 'awaiting_response' and now-self.sent_at >= self.timeout:
            self.state = 'timed_out'

    def sent(self,now):
        self._time(now)
        if self.state != 'prepared':
            raise RuntimeError('WLM query can only be sent once')
        self.state, self.sent_at = 'awaiting_response', now

    def cancel(self,now,reason='session_closed'):
        self._time(now)
        if self.state in ('prepared','awaiting_response'):
            self.state,self.cancel_reason = 'cancelled',reason

    def feed(self,data,now):
        self._time(now)
        if not isinstance(data,bytes) or len(data)>1024*1024:
            raise ValueError('WLM query input exceeds bounded byte profile')
        for offset in range(0,len(data),1024):
            for outer,_ in self.decoder.feed(data[offset:offset+1024]):
                if self.state != 'awaiting_response' or (
                        outer.version,outer.sender,outer.receiver,outer.cmd_set,outer.cmd_id) != (
                        1,0x27,0xee,0x51,1) or outer.cmd_type not in (0,0x20,0x40):
                    self.ignored += 1
                    continue
                try:
                    forwarded = unpack_forwarded(outer.payload)
                except ValueError:
                    self.malformed += 1
                    continue
                packet = forwarded.packet
                if (forwarded.link_category,packet.sender,packet.receiver,packet.cmd_set,
                        packet.cmd_id,packet.seq) != (2,0xe9,0xee,0x51,0x41,self.sequence
                        ) or packet.cmd_type not in ABILITY_ACK_TYPES:
                    self.ignored += 1
                    continue
                try:
                    ack = AbilityAcknowledgement.decode(packet.payload)
                except ValueError:
                    self.malformed += 1
                    continue
                if (ack.operation,ack.flags,ack.subtype,ack.reserved,ack.device_hash) != (
                        2,0,1,(0,0),self.peer_hash):
                    self.ignored += 1
                    continue
                if ack.status:
                    self.state,self.status,self.received_at = 'response_error',ack.status,now
                    continue
                try:
                    if len(ack.tlvs)!=2 or {kind for kind,_ in ack.tlvs}!={6,10}:
                        raise ValueError('query response must contain exactly CAP6 and CAP10')
                    capabilities={kind:CapabilityVector.decode(raw) for kind,raw in ack.tlvs}
                    if any(vector.word_count!=2 for vector in capabilities.values()):
                        raise ValueError('query response differs from measured two-word profile')
                except ValueError:
                    self.malformed += 1
                    continue
                self.capabilities,self.status,self.received_at = capabilities,0,now
                self.state = 'complete'

    def snapshot(self,now):
        self._time(now)
        return {'enabled':True,'state':self.state,'datagram_sent':self.sent_at is not None,
            'sequence':self.sequence,'peer_id_sha256':self.peer_fingerprint,
            'timeout_seconds':self.timeout,'response_status':self.status,
            'cancel_reason':self.cancel_reason,
            'response_age_seconds':None if self.received_at is None else now-self.received_at,
            'capabilities':None if self.capabilities is None else {
                str(kind):{'word_count':v.word_count,'mask':v.mask,'support':v.support}
                for kind,v in self.capabilities.items()},
            'ignored_frames':self.ignored,'malformed_frames':self.malformed,
            'discarded_bytes':self.decoder.discarded_bytes,
            'negotiation_enabled':False,'negotiated':False,'application_ready':False}
