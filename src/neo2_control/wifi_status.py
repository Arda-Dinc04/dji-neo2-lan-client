"""Read-only observations of the saved UAV77 51/04 Wi-Fi mode decoder.

The wire route and runtime product binding remain unverified. Keep separate
sources separate: these observations cannot establish aircraft authority.
"""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import math

from .duml import DumlPacket
from .duml_stream import DumlStreamDecoder
from .wlm_discovery import PLAINTEXT_PUSH_TYPES, unpack_observation_payload

MODE_NAMES = {0: 'NOT_IN', 1: 'MEDIA_TRANSFER', 2: 'OPERATION_CONTROL'}
UNKNOWN_MODE = 65535


def _finite_number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True, repr=False)
class WifiControlOsd:
    payload: bytes

    def __post_init__(self):
        if not isinstance(self.payload, bytes) or not 21 <= len(self.payload) <= 1010:
            raise ValueError('Wi-Fi control OSD requires 21..1010 payload bytes')

    @property
    def service_guard(self) -> bool:
        """The SDK's stored guard byte, not a flight permission."""
        return self.payload[20] > 1

    @property
    def mode_code(self) -> int:
        if self.payload[20] == 1:
            return (self.payload[15] >> 2) & 1
        if len(self.payload) < 27 or not any(self.payload[:3]):
            return UNKNOWN_MODE
        if self.payload[24] == 2:
            return 2
        return int(self.payload[26] == 1)

    def values(self) -> dict:
        mode = self.mode_code
        return {'mode_code': mode, 'mode_name': MODE_NAMES.get(mode),
                'service_guard': self.service_guard}


class WifiControlReceiver:
    """Bounded, per-route diagnostic samples; no transmitter or readiness gate."""
    def __init__(self, *, maximum_age: float = 1.0):
        if not _finite_number(maximum_age) or maximum_age <= 0:
            raise ValueError('Wi-Fi OSD maximum age must be finite and positive')
        self.maximum_age = maximum_age
        self.decoder = DumlStreamDecoder()
        self.observations = OrderedDict()
        self.last_input_time = None
        self.accepted = self.ignored = self.malformed = self.evicted = 0

    def _accept(self, packet: DumlPacket, now: float, carrier=None, attributes=None):
        if packet.version != 1 or packet.cmd_type not in PLAINTEXT_PUSH_TYPES:
            self.ignored += 1
            return
        if (packet.cmd_set, packet.cmd_id) == (0x51, 1) and carrier is None:
            try:
                child, attributes = unpack_observation_payload(packet.payload)
                if attributes is not None and attributes[4] & 0xf != 2:
                    self.ignored += 1
                    return
                self._accept(child, now, packet, attributes)
            except ValueError:
                self.malformed += 1
            return
        if ((packet.cmd_set, packet.cmd_id) != (0x51, 4)
            or packet.cmd_type not in (0, 0x20, 0x40)):
            self.ignored += 1
            return
        try:
            decoded = WifiControlOsd(packet.payload)
        except ValueError:
            self.malformed += 1
            return
        outer_route = None if carrier is None else (carrier.sender, carrier.receiver)
        hashes = None if attributes is None else (attributes[:4], attributes[9:13])
        key = packet.sender, packet.receiver, outer_route, hashes
        self.observations[key] = packet, decoded, now, carrier, attributes
        self.observations.move_to_end(key)
        if len(self.observations) > 8:
            self.observations.popitem(last=False)
            self.evicted += 1
        self.accepted += 1

    def _time(self, now):
        if (not _finite_number(now)
            or (self.last_input_time is not None and now < self.last_input_time)):
            raise ValueError('Wi-Fi OSD time must be finite and monotonic')
        self.last_input_time = now

    def feed(self, data: bytes, now: float):
        self._time(now)
        if len(data) > 1024 * 1024:
            raise ValueError('Wi-Fi OSD input block exceeds bounded size')
        for offset in range(0, len(data), 1024):
            for packet, _ in self.decoder.feed(data[offset:offset + 1024]):
                self._accept(packet, now)

    def snapshot(self, now: float) -> dict:
        self._time(now)
        samples = []
        for packet, decoded, received_at, carrier, attributes in self.observations.values():
            age = now - received_at
            fresh = age <= self.maximum_age
            mode = decoded.mode_code
            source = {'sender': packet.sender, 'receiver': packet.receiver,
                      'command_set': 0x51, 'command_id': 4, 'command_type': packet.cmd_type,
                      'sequence': packet.seq, 'wifi_tunnel': carrier is not None}
            if carrier is not None:
                source['carrier'] = {'sender': carrier.sender, 'receiver': carrier.receiver,
                                     'sequence': carrier.seq, 'command_type': carrier.cmd_type}
            if attributes is not None:
                source['forwarding'] = {
                    'source_hash': int.from_bytes(attributes[:4], 'little'),
                    'destination_hash': int.from_bytes(attributes[9:13], 'little'),
                    'link_category': attributes[4] & 0xf,
                }
            samples.append({
                **decoded.values(), 'received_at': received_at, 'age_seconds': age,
                'fresh': fresh, 'source': source, 'raw_payload_hex': decoded.payload.hex(),
                'fields': {
                    'wifi_mode': {'value': mode if mode in MODE_NAMES else None,
                                  'unit': 'sdk_wifi_mode', 'valid': fresh and mode in MODE_NAMES,
                                  'reason': 'stale' if not fresh else
                                            'decoded' if mode in MODE_NAMES else 'sdk_mode_unavailable'},
                    'service_guard': {'value': decoded.service_guard, 'unit': 'boolean',
                                      'valid': fresh, 'offset_bytes': 20, 'length_bytes': 1,
                                      'raw_hex': decoded.payload[20:21].hex(),
                                      'reason': 'decoded' if fresh else 'stale'},
                },
            })
        return {
            'available': bool(samples), 'observations': samples,
            'maximum_age_seconds': self.maximum_age, 'maximum_sources': 8,
            'freshness_basis': 'pc_receipt_monotonic', 'source_clock_verified': False,
            'source_route_verified': False, 'runtime_product_verified': False,
            'flight_ready': False, 'accepted': self.accepted, 'ignored': self.ignored,
            'malformed': self.malformed, 'evicted': self.evicted,
            'discarded_bytes': self.decoder.discarded_bytes,
        }


def age_wifi_control(snapshot: dict, elapsed: float, session_active: bool) -> dict:
    """Age an already detached owner snapshot, preserving source separation."""
    if not _finite_number(elapsed) or elapsed < 0:
        raise ValueError('Wi-Fi OSD elapsed time must be finite and nonnegative')
    snapshot.update(session_active=bool(session_active), flight_ready=False)
    for item in snapshot.get('observations', []):
        item['age_seconds'] += elapsed
        fresh = item['age_seconds'] <= snapshot['maximum_age_seconds']
        item['fresh'] = fresh
        for field in item['fields'].values():
            field['valid'] = field['valid'] and fresh and bool(session_active)
            if not session_active:
                field['reason'] = 'session_inactive'
            elif not fresh:
                field['reason'] = 'stale'
    return snapshot
