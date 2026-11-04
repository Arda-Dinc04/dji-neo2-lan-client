"""Observe Neo2's exact FC03/53 landing-confirmation profile, without sending.

The pinned UAV158 getter reads u16LE(body+0), bit5. All measured bodies have
four bytes; the getter's two-byte read is not a complete structure length.
Direct observations require separate typed owner provenance for flight use.
"""
from collections import OrderedDict
import math

from .duml_stream import DumlStreamDecoder
from .host_control import HostControlState
from .wlm_discovery import unpack_forwarded


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class LandingConfirmationReceiver:
    MAXIMUM_SOURCES = 8

    def __init__(self, *, maximum_age=.5):
        if not _finite(maximum_age) or maximum_age <= 0:
            raise ValueError('landing confirmation maximum age must be finite and positive')
        self.maximum_age = maximum_age
        self.decoder = DumlStreamDecoder()
        self.decoder_generation = None
        self.observations = OrderedDict()
        self.last_time = None
        self.accepted = self.ignored = self.malformed = self.evicted = 0

    def _time(self, now):
        if not _finite(now) or (self.last_time is not None and now < self.last_time):
            raise ValueError('landing confirmation time must be finite and monotonic')
        self.last_time = now

    def _accept(self, packet, now, *, direct_context=None, carrier=None, attributes=None):
        if packet.version != 1 or packet.cmd_type != 0:
            self.ignored += 1
            return
        if (packet.cmd_set, packet.cmd_id) == (0x51, 1) and carrier is None:
            if (packet.sender, packet.receiver) != (0x27, 0xee):
                self.ignored += 1
                return
            try:
                forwarded = unpack_forwarded(packet.payload)
            except ValueError:
                self.malformed += 1
                return
            if (forwarded.link_category != 2
                    or int.from_bytes(forwarded.attributes[9:13], 'little') != 0xffffffff):
                self.ignored += 1
                return
            self._accept(forwarded.packet, now, carrier=packet, attributes=forwarded.attributes)
            return
        if ((packet.sender, packet.cmd_set, packet.cmd_id) != (3, 3, 0x53)
                or packet.receiver not in (2, 0x0e)):
            self.ignored += 1
            return
        if len(packet.payload) != 4:
            self.malformed += 1
            return
        generation = direct_context.generation if carrier is None and direct_context is not None else None
        hashes = None if attributes is None else (attributes[:4], attributes[9:13])
        key = (packet.sender, packet.receiver,
               None if carrier is None else (carrier.sender, carrier.receiver), hashes, generation)
        self.observations[key] = packet, now, carrier, attributes, generation
        self.observations.move_to_end(key)
        if len(self.observations) > self.MAXIMUM_SOURCES:
            self.observations.popitem(last=False)
            self.evicted += 1
        self.accepted += 1

    def feed(self, data, now, *, direct_context: HostControlState | None = None):
        self._time(now)
        if len(data) > 1024 * 1024:
            raise ValueError('landing confirmation input exceeds bounded size')
        if direct_context is not None and not isinstance(direct_context, HostControlState):
            raise ValueError('direct landing confirmation requires typed owning connection evidence')
        if direct_context is not None and not direct_context.permits_control(now):
            direct_context = None
        generation = None if direct_context is None else direct_context.generation
        if self.decoder.buffer and generation != self.decoder_generation:
            self.decoder.discarded_bytes += len(self.decoder.buffer)
            self.decoder.buffer.clear()
        self.decoder_generation = generation
        for offset in range(0, len(data), 1024):
            for packet, _ in self.decoder.feed(data[offset:offset + 1024]):
                self._accept(packet, now, direct_context=direct_context)

    def snapshot(self, now):
        self._time(now)
        observations = []
        for packet, received, carrier, attributes, generation in self.observations.values():
            age = now - received
            fresh = 0 <= age <= self.maximum_age
            source = {'sender': packet.sender, 'receiver': packet.receiver,
                      'sequence': packet.seq, 'command_set': 3, 'command_id': 0x53,
                      'command_type': packet.cmd_type, 'wifi_tunnel': carrier is not None}
            if carrier is not None:
                source['carrier'] = {'sender': carrier.sender, 'receiver': carrier.receiver,
                                     'sequence': carrier.seq, 'command_type': carrier.cmd_type}
                source['forwarding'] = {'source_hash': int.from_bytes(attributes[:4], 'little'),
                    'destination_hash': int.from_bytes(attributes[9:13], 'little'),
                    'link_category': attributes[4] & 15}
            if generation is not None:
                source['direct_connection'] = {'generation': list(generation),
                    'provenance': 'owned_connected_swudp_peer_session'}
            observations.append({'source': source, 'received_at': received,
                'age_seconds': age, 'fresh': fresh, 'raw_payload_hex': packet.payload.hex(),
                'fields': {'landing_confirmation_needed': {
                    'value': bool(int.from_bytes(packet.payload[:2], 'little') & 0x20),
                    'valid': fresh, 'reason': None if fresh else 'stale'}}})
        return {'available': bool(observations), 'observations': observations,
            'maximum_age_seconds': self.maximum_age, 'maximum_sources': self.MAXIMUM_SOURCES,
            'freshness_basis': 'pc_receipt_monotonic', 'source_clock_verified': False,
            'source_route_verified': False, 'flight_ready': False,
            'accepted': self.accepted, 'ignored': self.ignored, 'malformed': self.malformed,
            'evicted': self.evicted, 'discarded_bytes': self.decoder.discarded_bytes}
