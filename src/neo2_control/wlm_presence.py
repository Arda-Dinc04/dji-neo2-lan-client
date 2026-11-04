"""Bounded PC presence before a WLM capability read, without negotiation.

The payload layout comes from the native beacon producer. Wi-Fi only, no
V3 or capability negotiation, SSFN zero and a zero DUML sequence are explicit
PC policy choices. This is not the Android application's service profile.
"""

import math
import secrets
import struct

from .duml import DumlPacket
from .identity import WlmIdentity
from .wlm_modes import WlmModes
from .wlm_wire import wifi_forward


def own_presence(identity, *, peer_format, nonce, counter, forwarding_sequence=1, modes=None):
    """Encode the native v5 presence layout for an owned PC identity.

    A current v5 peer is required: native older-version handling forces the
    V3 bit even when disabled. The native broadcast attribute 0x81 does not
    advance the forwarding counter; the following query can reuse it.
    """
    if not isinstance(identity, WlmIdentity) or type(peer_format) is not int or peer_format != 5:
        raise ValueError('PC presence requires an owned identity and a current v5 peer')
    for value, bits in ((nonce, 16), (counter, 32), (forwarding_sequence, 32)):
        if type(value) is not int or not 0 <= value < 1 << bits:
            raise ValueError(f'PC presence field must be uint{bits}')
    if nonce == 0:
        raise ValueError('PC presence requires a nonzero session nonce')
    if modes is None:
        modes = WlmModes(4,4,4,0,0)
    if not isinstance(modes,WlmModes):
        raise ValueError('PC presence requires validated actual modes')
    payload = bytearray(55)
    payload[0] = 5
    payload[1:24] = identity.wlm_device_id
    payload[24:27] = modes.encode()[:3]
    payload[33:39] = bytes([0, 0, 1, 0, 0, 1])
    struct.pack_into('<I', payload, 39, counter)
    struct.pack_into('<H', payload, 49, nonce)
    payload[51] = 4
    payload[52:54] = modes.encode()[3:]
    inner = DumlPacket(sender=0xee, receiver=0xe9, seq=0, cmd_type=0,
                       cmd_set=0x51, cmd_id=0x13, payload=bytes(payload))
    # The dispatcher does not initialize the last seven bytes. Canonical
    # zeros are our choice, as for the independently verified query wrapper.
    return wifi_forward(inner,identity.wlm_device_hash,forwarding_sequence)


class WlmPresence:
    """Three announcements, 500 ms apart, followed by 500 ms settling.

    This finite schedule is a PC experiment policy. Actual socket completion
    drives counters and spacing. Sending does not prove peer registration.
    """

    def __init__(self, identity):
        if not isinstance(identity, WlmIdentity):
            raise ValueError('PC presence requires an owned identity')
        self.identity = identity
        self.nonce = secrets.randbelow(65535) + 1
        self.sent_count = self.scheduled_count = 0
        self.pending = False
        self.next_at = None
        self.last_time = None
        self.state = 'prepared'
        self.cancel_reason = None

    def _time(self, now):
        if not math.isfinite(now) or (self.last_time is not None and now < self.last_time):
            raise ValueError('PC presence time must be finite and monotonic')
        self.last_time = now
        if self.state == 'settling' and now >= self.next_at:
            self.state = 'complete'

    def next_packet(self, now):
        self._time(now)
        if self.state not in ('prepared', 'announcing') or self.pending or (
                self.next_at is not None and now < self.next_at):
            return None
        packet = own_presence(self.identity, peer_format=5, nonce=self.nonce,
                              counter=self.sent_count)
        self.pending = True
        self.scheduled_count += 1
        return packet

    def sent(self, now):
        self._time(now)
        if not self.pending or self.state not in ('prepared', 'announcing'):
            raise RuntimeError('PC presence requires one scheduled socket send')
        self.pending = False
        self.sent_count += 1
        self.next_at = now + .5
        self.state = 'settling' if self.sent_count == 3 else 'announcing'

    def cancel(self, now, reason):
        self._time(now)
        if self.state not in ('cancelled', 'complete'):
            self.state, self.cancel_reason, self.pending = 'cancelled', reason, False

    def snapshot(self, now):
        self._time(now)
        return {'state': self.state, 'scheduled': self.scheduled_count,
                'datagrams_sent': self.sent_count, 'maximum_datagrams': 3,
                'interval_seconds': .5, 'cancel_reason': self.cancel_reason,
                'peer_registration_confirmed': False, 'negotiation_enabled': False,
                'application_ready': False}
