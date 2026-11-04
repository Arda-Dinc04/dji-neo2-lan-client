"""FC OSD values projected through the saved SDK's field getters.

Native numeric conversions are distinct from physical validation. The current
decoder cannot establish battery, navigation suitability or flight readiness.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import math
import struct

from .duml import DumlPacket
from .duml_stream import DumlStreamDecoder
from .host_control import HostControlState
from .wlm_discovery import PLAINTEXT_PUSH_TYPES, unpack_forwarded


FC_OSD = (3, 0x43)
MAX_TELEMETRY_SOURCES = 8


def ground_velocity_to_aircraft(velocity, yaw_degrees):
    """Saved SDK GroundToUAV equation; physical axis labels remain uncalibrated."""
    if (not isinstance(velocity,(tuple,list)) or len(velocity)!=3
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in velocity)
            or type(yaw_degrees) not in (int,float) or not math.isfinite(yaw_degrees)
            or not -180<=yaw_degrees<=180):
        raise ValueError('velocity_transform_invalid')
    x,y,z=velocity
    sine,cosine=math.sin(math.radians(yaw_degrees)),math.cos(math.radians(yaw_degrees))
    return cosine*x+sine*y,cosine*y-sine*x,z


@dataclass(frozen=True, repr=False)
class FcOsd:
    payload: bytes

    def __post_init__(self):
        if not isinstance(self.payload, bytes) or not 37 <= len(self.payload) <= 1010:
            raise ValueError('FC OSD requires a bounded payload of at least 37 bytes')

    @property
    def flags(self):
        return struct.unpack_from('<I', self.payload, 32)[0]

    def sdk_values(self) -> dict:
        """Constructor arguments emitted by the SDK's Key...Push functions."""
        longitude, latitude = struct.unpack_from('<dd', self.payload)
        flags = self.flags
        nonzero = any(abs(v) >= 1e-9 or math.isnan(v) for v in (latitude, longitude))
        available = bool(flags & 0x380000 and flags & 0x8000 and nonzero)
        if len(self.payload) >= 69 and self.payload[68] & 7 == 1:
            available = True
        return {
            'altitude': (struct.unpack_from('<h', self.payload, 16)[0] * 0.1,),
            'velocity': tuple(v * 0.1 for v in struct.unpack_from('<3h', self.payload, 18)),
            'attitude': tuple(v * 0.1 for v in struct.unpack_from('<3h', self.payload, 24)),
            'is_flying': int(flags & 0xE == 0xC),
            'motors_on': int(bool(flags & 8)),
            'flight_mode': self.payload[30] & 0x7F,
            'gps_valid': int(bool(flags & 0x8000)),
            'satellite_count': self.payload[36],
            'location': (latitude * 180.0 / math.pi, longitude * 180.0 / math.pi) if available else None,
        }

    def fields(self) -> dict:
        values = self.sdk_values()
        fields = {}
        layout = {
            'altitude': (16, 2, 'm', values['altitude'][0]),
            'velocity': (18, 6, 'm/s', values['velocity']),
            'attitude': (24, 6, 'degree', values['attitude']),
            'is_flying': (32, 4, 'boolean', bool(values['is_flying'])),
            'motors_on': (32, 4, 'boolean', bool(values['motors_on'])),
            'flight_mode': (30, 1, 'code', values['flight_mode']),
            'gps_valid': (32, 4, 'boolean', bool(values['gps_valid'])),
            'satellite_count': (36, 1, 'count', values['satellite_count']),
            'location': (0, 16, 'degree', values['location']),
        }
        for name, (offset, size, unit, value) in layout.items():
            valid = value is not None
            reason = 'decoded' if valid else 'sdk_location_unavailable'
            if name == 'location' and valid:
                valid = all(math.isfinite(v) for v in value) and abs(value[0]) <= 90 and abs(value[1]) <= 180
                if not valid:
                    reason, value = 'invalid_geographic_value', None
            if name == 'attitude' and any(abs(v) > 180 for v in value):
                valid, reason, value = False, 'invalid_attitude_range', None
            if name == 'velocity':
                reason = 'sdk_unit_verified_physical_accuracy_unverified'
            fields[name] = {'value': value, 'raw_hex': self.payload[offset:offset + size].hex(),
                            'offset_bytes': offset, 'length_bytes': size, 'unit': unit,
                            'valid': valid, 'reason': reason}
            if name == 'velocity':
                fields[name].update(reference='sdk_ground_frame',axis_labels_verified=False,
                    physical_accuracy_verified=False)
            if name == 'altitude':
                # KeyAltitude -> Location3D(RELATIVE) -> height OSD -> metric
                # DistanceUnitConstant: unchanged value labelled metres.
                fields[name].update(reference='relative',physical_accuracy_verified=False)
        return fields


def decode_fc_osd(payload: bytes) -> FcOsd:
    return FcOsd(payload)


class TelemetryReceiver:
    """Bounded DUML framing, one-level Wi-Fi tunnel and monotonic freshness.

    Legacy FC3.0→2.0 and the measured Wi-Fi broadcast recipients14.0/10.5
    remain separate observations. Native wrappers contain exactly one DUML
    plus22 forwarding bytes; attributes are never searched for more frames.
    Incoming bytes have already passed the reliable transport's receive window.
    Direct85byte broadcasts require typed evidence from the committed socket
    owner; their connection provenance stays distinct from WLM forwarding.
    """

    def __init__(self, *, maximum_age: float = 1.0):
        if not math.isfinite(maximum_age) or maximum_age <= 0:
            raise ValueError('telemetry maximum age must be finite and positive')
        self.maximum_age = maximum_age
        self.decoder = DumlStreamDecoder()
        self.decoder_generation = None
        self.observations = OrderedDict()
        self.last = None
        self.last_time = self.last_input_time = None
        self.accepted = self.ignored = self.malformed = self.evicted = 0

    def _accept(self, packet: DumlPacket, now: float, carrier=None, attributes=None,
                direct_context=None):
        if packet.version != 1:
            self.ignored += 1
            return
        if (packet.cmd_set, packet.cmd_id) == (0x51, 1) and carrier is None:
            if ((packet.sender, packet.receiver) != (0x27, 0xee)
                    or packet.cmd_type not in PLAINTEXT_PUSH_TYPES):
                self.ignored += 1
                return
            payload = packet.payload
            bare_length = int.from_bytes(payload[1:3], 'little') & 0x3ff
            try:
                if len(payload) == bare_length:
                    # Historical bare-inner compatibility is one exact frame.
                    # This branch is selected by length before native parsing,
                    # never as a fallback for malformed forwarding attributes.
                    child = DumlPacket.decode(payload)
                    self._accept(child, now, packet)
                else:
                    forwarded = unpack_forwarded(payload)
                    if forwarded.link_category != 2:
                        self.ignored += 1
                        return
                    self._accept(forwarded.packet, now, packet, forwarded.attributes)
            except ValueError:
                self.malformed += 1
            return
        if ((packet.sender, packet.cmd_set, packet.cmd_id) != (3, *FC_OSD)
                or packet.cmd_type not in PLAINTEXT_PUSH_TYPES):
            self.ignored += 1
            return
        direct_generation = None
        if packet.receiver != 2:
            if packet.receiver not in (0x0e, 0xaa):
                self.ignored += 1
                return
            if attributes is None:
                # During measured motor startup the same connected Neo2
                # sends these FC broadcasts without a WLM wrapper. Only the
                # current committed socket owner can supply their provenance;
                # bare-inner compatibility must never inherit that context.
                if carrier is not None or direct_context is None:
                    self.ignored += 1
                    return
                direct_generation = direct_context.generation
            elif int.from_bytes(attributes[9:13], 'little') != 0xffffffff:
                self.ignored += 1
                return
            if len(packet.payload) != 85:
                self.malformed += 1
                return
        try:
            decoded = decode_fc_osd(packet.payload)
        except ValueError:
            self.malformed += 1
            return
        # Transport deduplicates reliable blocks. A DUML push may reuse its
        # sequence; do not invent a second sequence policy for these samples.
        carrier_route = None if carrier is None else (carrier.sender, carrier.receiver)
        hashes = None if attributes is None else (attributes[:4], attributes[9:13])
        key = packet.sender, packet.receiver, carrier_route, hashes, direct_generation
        self.last = packet, decoded, now, carrier, attributes, direct_generation
        self.observations[key] = self.last
        self.observations.move_to_end(key)
        if len(self.observations) > MAX_TELEMETRY_SOURCES:
            self.observations.popitem(last=False)
            self.evicted += 1
        self.last_time = now
        self.accepted += 1

    def feed(self, data: bytes, now: float, *, direct_context: HostControlState | None = None):
        if not math.isfinite(now) or (self.last_input_time is not None and now < self.last_input_time):
            raise ValueError('telemetry time must be finite and monotonic')
        self.last_input_time = now
        if len(data) > 1024 * 1024:
            raise ValueError('telemetry input block exceeds bounded size')
        if direct_context is not None and not isinstance(direct_context, HostControlState):
            raise ValueError('direct telemetry requires typed owning connection evidence')
        if direct_context is not None and not direct_context.permits_control(now):
            direct_context = None
        generation = None if direct_context is None else direct_context.generation
        if self.decoder.buffer and generation != self.decoder_generation:
            # An unfinished frame cannot cross a connection generation or
            # acquire provenance that was unavailable when its bytes arrived.
            self.decoder.discarded_bytes += len(self.decoder.buffer)
            self.decoder.buffer.clear()
        self.decoder_generation = generation
        for offset in range(0, len(data), 1024):
            for packet, _ in self.decoder.feed(data[offset:offset + 1024]):
                self._accept(packet, now, direct_context=direct_context)

    def _sample(self, observation, now):
        packet, decoded, received_at, carrier, attributes, direct_generation = observation
        age = now - received_at
        source = {'command_set': 3, 'command_id': 0x43, 'sequence': packet.seq,
                  'sender': packet.sender, 'receiver': packet.receiver,
                  'command_type': packet.cmd_type, 'wifi_tunnel': carrier is not None}
        if carrier is not None:
            source['carrier'] = {'sender': carrier.sender, 'receiver': carrier.receiver,
                                 'sequence': carrier.seq, 'command_type': carrier.cmd_type}
        if attributes is not None:
            source['forwarding'] = {
                'source_hash': int.from_bytes(attributes[:4], 'little'),
                'destination_hash': int.from_bytes(attributes[9:13], 'little'),
                'link_category': attributes[4] & 0xf,
            }
        if direct_generation is not None:
            source['direct_connection'] = {
                'generation': list(direct_generation),
                'provenance': 'owned_connected_swudp_peer_session',
            }
        result = {'age_seconds': age, 'fresh': age <= self.maximum_age, 'source': source,
                  'received_at': received_at, 'raw_flags': decoded.flags,
                  'flight_phase_code': (decoded.flags >> 1) & 7,
                  'flight_ready': False, 'fields': {}}
        for name, field in decoded.fields().items():
            result['fields'][name] = {**field, 'age_seconds': age, 'source': source,
                                      'valid': field['valid'] and age <= self.maximum_age,
                                      'reason': field['reason'] if age <= self.maximum_age else 'stale'}
        return result

    def snapshot(self, now: float) -> dict:
        if not math.isfinite(now) or (self.last_input_time is not None and now < self.last_input_time):
            raise ValueError('telemetry time must be finite and monotonic')
        self.last_input_time = now
        result = {'available': self.last is not None, 'flight_ready': False,
                  'maximum_age_seconds': self.maximum_age,
                  'freshness_basis': 'pc_receipt_monotonic', 'source_clock_verified': False,
                  'accepted': self.accepted, 'ignored': self.ignored, 'malformed': self.malformed,
                  'discarded_bytes': self.decoder.discarded_bytes, 'fields': {},
                  'selection': 'last_received', 'maximum_sources': MAX_TELEMETRY_SOURCES,
                  'evicted': self.evicted,
                  'observations': [self._sample(item, now) for item in self.observations.values()]}
        if self.last is None:
            return result
        # Project the latest receipt for existing callers. Build it separately
        # so mutating/aging this projection cannot also age a listed sample.
        result.update(self._sample(self.last, now))
        return result
