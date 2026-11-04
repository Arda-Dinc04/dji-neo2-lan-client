"""Read-only FC 03/55 battery observations bound to the saved SDK getters.

A decoded percentage is not a healthy battery or an authorization to take off.
Different DUML routes are retained independently; no source is auto-selected.
"""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import math

from .duml import DumlPacket
from .duml_stream import DumlStreamDecoder
from .host_control import HostControlState
from .wlm_discovery import PLAINTEXT_PUSH_TYPES, unpack_observation_payload


def _finite_number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True, repr=False)
class FcBatteryOsd:
    payload: bytes

    def __post_init__(self):
        if not isinstance(self.payload, bytes) or not 10 <= len(self.payload) <= 1010:
            raise ValueError('FC battery OSD requires 10..1010 payload bytes')

    def sdk_values(self):
        flags = int.from_bytes(self.payload[:4], 'little')
        thresholds = int.from_bytes(self.payload[6:8], 'little')
        return {
            'BatteryPowerPercent': self.payload[8],
            'BatteryVoltage': int.from_bytes(self.payload[4:6], 'little'),
            'FCBatteryType': self.payload[9] & 3,
            'RequireGoHomeByLowBatteryVoltage': bool(flags & (1 << 4)),
            'RequireLandingByLowBatteryVoltage': bool(flags & (1 << 5)),
            'RequireGoHomeBySmartBatteryPercent': bool(flags & (1 << 2)),
            'RequireLandingBySmartBatteryPercent': bool(flags & (1 << 3)),
            'IsBatteryFirstChargeNotFull': bool(flags & (1 << 11)),
            'LowBatteryWarningThreshold': thresholds & 0x7F,
            'SeriousLowBatteryWarningThreshold': (thresholds >> 8) & 0x7F,
            'VoltageWarningInfo': (bool(flags & (1 << 4)), bool(flags & (1 << 5))),
        }

    def fields(self):
        values = self.sdk_values()
        layout = (
            ('percent', 'BatteryPowerPercent', 8, 1, 'percent'),
            ('voltage', 'BatteryVoltage', 4, 2, 'sdk_voltage'),
            ('type_code', 'FCBatteryType', 9, 1, 'code'),
            ('require_go_home_low_voltage', 'RequireGoHomeByLowBatteryVoltage', 0, 4, 'boolean'),
            ('require_land_low_voltage', 'RequireLandingByLowBatteryVoltage', 0, 4, 'boolean'),
            ('require_go_home_smart_percent', 'RequireGoHomeBySmartBatteryPercent', 0, 4, 'boolean'),
            ('require_land_smart_percent', 'RequireLandingBySmartBatteryPercent', 0, 4, 'boolean'),
            ('first_charge_not_full', 'IsBatteryFirstChargeNotFull', 0, 4, 'boolean'),
            ('low_warning_percent', 'LowBatteryWarningThreshold', 6, 2, 'percent'),
            ('serious_low_warning_percent', 'SeriousLowBatteryWarningThreshold', 6, 2, 'percent'),
            ('voltage_warning_info', 'VoltageWarningInfo', 0, 4, 'boolean_pair'),
        )
        result = {}
        for name, sdk_name, offset, length, unit in layout:
            value = values[sdk_name]
            valid = unit != 'percent' or 0 <= value <= 100
            reason = 'decoded' if valid else 'invalid_percent_range'
            if unit == 'sdk_voltage':
                reason = 'scale_and_unit_unconfirmed'
            if name == 'type_code':
                reason = 'sdk_enum_semantics_unverified'
            result[name] = {
                'value': value if valid else None, 'unit': unit, 'valid': valid, 'reason': reason,
                'offset_bytes': offset, 'length_bytes': length,
                'raw_hex': self.payload[offset:offset + length].hex(), 'sdk_key': sdk_name,
            }
        return result


class BatteryReceiver:
    """Observe FC 03/55 pushes, including one-level Wi-Fi tunnels.

    Allow plaintext request/push ACK modes0/1/2. A response never refreshes
    this store; it may carry a request status instead of a raw observation.
    Measured direct FC pushes carry separate typed socket-owner provenance;
    other unbound direct sources remain diagnostic observations.
    """
    def __init__(self, *, maximum_age: float = 1.0):
        if not _finite_number(maximum_age) or maximum_age <= 0:
            raise ValueError('battery maximum age must be finite and positive')
        self.maximum_age = maximum_age
        self.decoder = DumlStreamDecoder()
        self.decoder_generation = None
        self.observations = OrderedDict()
        self.last_input_time = None
        self.accepted = self.ignored = self.malformed = self.evicted = 0

    def _time(self, now):
        if not _finite_number(now) or (self.last_input_time is not None and now < self.last_input_time):
            raise ValueError('battery time must be finite and monotonic')
        self.last_input_time = now

    def _accept(self, packet: DumlPacket, now: float, carrier=None, attributes=None,
                direct_context=None):
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
        if ((packet.cmd_set, packet.cmd_id) != (3, 0x55)
            or packet.cmd_type not in (0, 0x20, 0x40)):
            self.ignored += 1
            return
        try:
            decoded = FcBatteryOsd(packet.payload)
        except ValueError:
            self.malformed += 1
            return
        # The measured airborne FC sends this exact ten-byte broadcast
        # directly on its owned UDP session. Preserve that provenance; do not
        # manufacture WLM forwarding hashes for an unwrapped packet. Other
        # direct battery messages remain diagnostic observations only.
        direct_generation = None
        if (carrier is None and attributes is None and direct_context is not None
                and packet.sender == 3 and packet.receiver in (2, 0x0e)
                and packet.cmd_type == 0 and len(packet.payload) == 10):
            direct_generation = direct_context.generation
        hashes = None if attributes is None else (attributes[:4], attributes[9:13])
        key = (packet.sender, packet.receiver,
               None if carrier is None else (carrier.sender, carrier.receiver),
               hashes, direct_generation)
        self.observations[key] = packet, decoded, now, carrier, attributes, direct_generation
        self.observations.move_to_end(key)
        if len(self.observations) > 8:
            self.observations.popitem(last=False)
            self.evicted += 1
        self.accepted += 1

    def feed(self, data: bytes, now: float, *, direct_context: HostControlState | None = None):
        self._time(now)
        if len(data) > 1024 * 1024:
            raise ValueError('battery input block exceeds bounded size')
        if direct_context is not None and not isinstance(direct_context, HostControlState):
            raise ValueError('direct battery requires typed owning connection evidence')
        if direct_context is not None and not direct_context.permits_control(now):
            direct_context = None
        generation = None if direct_context is None else direct_context.generation
        if self.decoder.buffer and generation != self.decoder_generation:
            # Bytes started without this binding cannot inherit it when the
            # remainder arrives, or cross to a new owner/connection generation.
            self.decoder.discarded_bytes += len(self.decoder.buffer)
            self.decoder.buffer.clear()
        self.decoder_generation = generation
        for offset in range(0, len(data), 1024):
            for packet, _ in self.decoder.feed(data[offset:offset + 1024]):
                self._accept(packet, now, direct_context=direct_context)

    def snapshot(self, now: float):
        self._time(now)
        samples = []
        for packet, decoded, received_at, carrier, attributes, direct_generation in self.observations.values():
            age = now - received_at
            fresh = age <= self.maximum_age
            source = {'sender': packet.sender, 'receiver': packet.receiver, 'sequence': packet.seq,
                      'command_set': 3, 'command_id': 0x55, 'command_type': packet.cmd_type,
                      'wifi_tunnel': carrier is not None}
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
            fields = decoded.fields()
            for value in fields.values():
                value['valid'] = value['valid'] and fresh
                if not fresh:
                    value['reason'] = 'stale'
            samples.append({
                'source': source, 'received_at': received_at, 'age_seconds': age, 'fresh': fresh,
                'raw_payload_hex': decoded.payload.hex(), 'fields': fields,
            })
        return {
            'available': bool(samples), 'observations': samples, 'maximum_age_seconds': self.maximum_age,
            'maximum_sources': 8, 'freshness_basis': 'pc_receipt_monotonic',
            'source_clock_verified': False, 'source_route_verified': False,
            'runtime_product_verified': False, 'flight_ready': False,
            'accepted': self.accepted, 'ignored': self.ignored, 'malformed': self.malformed,
            'evicted': self.evicted, 'discarded_bytes': self.decoder.discarded_bytes,
        }


def age_battery(snapshot: dict, elapsed: float, session_active: bool):
    if not _finite_number(elapsed) or elapsed < 0:
        raise ValueError('battery elapsed time must be finite and nonnegative')
    snapshot.update(session_active=bool(session_active), flight_ready=False)
    for item in snapshot.get('observations', []):
        item['age_seconds'] += elapsed
        item['fresh'] = item['age_seconds'] <= snapshot['maximum_age_seconds']
        for field in item['fields'].values():
            field['valid'] = field['valid'] and item['fresh'] and bool(session_active)
            if not session_active:
                field['reason'] = 'session_inactive'
            elif not item['fresh']:
                field['reason'] = 'stale'
    return snapshot
