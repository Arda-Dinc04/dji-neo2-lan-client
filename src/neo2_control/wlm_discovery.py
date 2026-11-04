"""WLM forwarding and discovery received on the measured Neo2 Wi-Fi route.

The 22-byte forwarding attributes follow the complete inner DUML. This module
observes discovery; it does not infer a ready link or transmit negotiation.
"""

from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import math

from .duml import DumlPacket
from .duml_stream import DumlStreamDecoder


FORWARD_ATTRIBUTE_LENGTH = 22
BEACON_LENGTH = 55
PLAINTEXT_PUSH_TYPES = (0, 0x20, 0x40)
# APK native replies retain request flags (c0); the real Neo2 firmware also
# emits plain ACK80 for the same op2/sub1 response. Encrypted variants stay out.
ABILITY_ACK_TYPES = (0x80, 0xc0)


@dataclass(frozen=True)
class CapabilityVector:
    """Native v0 header, little-endian mask bank and support bank."""
    word_count: int
    mask: int
    support: int

    @classmethod
    def decode(cls, raw: bytes):
        if not isinstance(raw, bytes) or not raw:
            raise ValueError('empty WLM capability vector')
        version, count = raw[0] & 15, raw[0] >> 4
        if version != 0 or count == 0 or len(raw) != 1 + count * 8:
            raise ValueError('unsupported WLM capability header or bank length')
        middle = 1 + count * 4
        return cls(count, int.from_bytes(raw[1:middle], 'little'),
                   int.from_bytes(raw[middle:], 'little'))


@dataclass(frozen=True, repr=False)
class ForwardedPacket:
    packet: DumlPacket
    attributes: bytes

    @property
    def link_category(self):
        return self.attributes[4] & 0x0f


def unpack_forwarded(payload: bytes) -> ForwardedPacket:
    """Validate one native DUML+attributes envelope, without scanning inside it."""
    if not isinstance(payload, bytes) or not 35 <= len(payload) <= 1010:
        raise ValueError('invalid WLM forwarding length')
    raw, attributes = payload[:-FORWARD_ATTRIBUTE_LENGTH], payload[-FORWARD_ATTRIBUTE_LENGTH:]
    if (int.from_bytes(raw[1:3], 'little') & 0x3ff) != len(raw):
        raise ValueError('WLM inner frame length must end at the attribute trailer')
    packet = DumlPacket.decode(raw)
    if packet.version != 1:
        raise ValueError('unsupported WLM forwarded DUML version')
    return ForwardedPacket(packet, attributes)


def unpack_observation_payload(payload: bytes) -> tuple[DumlPacket, bytes | None]:
    """One diagnostic inner frame, with exact legacy-bare compatibility.

    Select the format by encoded length before decoding. A malformed native
    envelope never falls back to a stream scan or an inner-only parse.
    """
    if not isinstance(payload, bytes) or not 13 <= len(payload) <= 1010:
        raise ValueError('invalid WLM observation envelope length')
    encoded_length = int.from_bytes(payload[1:3], 'little') & 0x3ff
    if len(payload) == encoded_length:
        packet = DumlPacket.decode(payload)
        if packet.version != 1:
            raise ValueError('unsupported WLM observation DUML version')
        return packet, None
    forwarded = unpack_forwarded(payload)
    return forwarded.packet, forwarded.attributes


@dataclass(frozen=True, repr=False)
class WlmBeacon:
    payload: bytes
    forwarding_source_hash: int | None = None
    forwarding_destination_hash: int | None = None

    def __post_init__(self):
        if not isinstance(self.payload, bytes) or len(self.payload) != BEACON_LENGTH:
            raise ValueError('measured WLM beacon requires exactly 55 bytes')
        for value in (self.forwarding_source_hash, self.forwarding_destination_hash):
            if value is not None and (type(value) is not int or not 0 <= value < 2**32):
                raise ValueError('beacon forwarding hashes must be uint32 or unknown')

    @property
    def device_id(self):
        # Native wlm_dev_id23: type/index/model/SN20, unlike07/94 device28.
        return self.payload[1:24]

    def metadata(self):
        return {'device_id_sha256': hashlib.sha256(self.device_id).hexdigest(),
                'device_type': self.device_id[0], 'device_index': self.device_id[1],
                'device_model': self.device_id[2],
                'format_code': self.payload[0],
                'beacon_counter': int.from_bytes(self.payload[39:43], 'little'),
                'native_ssfn': int.from_bytes(self.payload[43:47], 'little'),
                'native_word_49': int.from_bytes(self.payload[49:51], 'little'),
                'flags': self.payload[51],
                'ability_negotiation_advertised': bool(self.payload[51] & 0x40)}


@dataclass(frozen=True, repr=False)
class AbilityMessage:
    """Common51/41 header and bounded TLVs; bit semantics remain separate."""
    operation: int
    flags: int
    subtype: int
    reserved: int
    device_hash: int
    tlvs: tuple[tuple[int, bytes], ...]

    @classmethod
    def decode(cls, payload: bytes):
        if not isinstance(payload, bytes) or not 9 <= len(payload) <= 1010:
            raise ValueError('invalid WLM ability message length')
        length = int.from_bytes(payload[7:9], 'little')
        if length != len(payload) - 9:
            raise ValueError('WLM ability TLV length mismatch')
        cursor, tlvs = 9, []
        while cursor < len(payload):
            if cursor + 2 > len(payload):
                raise ValueError('truncated WLM ability TLV header')
            kind, size = payload[cursor:cursor + 2]
            cursor += 2
            if cursor + size > len(payload):
                raise ValueError('truncated WLM ability TLV value')
            tlvs.append((kind, payload[cursor:cursor + size]))
            cursor += size
        return cls(payload[0] >> 4, payload[0] & 15, payload[1], payload[2],
                   int.from_bytes(payload[3:7], 'little'), tuple(tlvs))


@dataclass(frozen=True, repr=False)
class AbilityAcknowledgement:
    """Native ACK has an11byte header, distinct from the9byte request."""
    status: int
    operation: int
    flags: int
    subtype: int
    reserved: tuple[int, int]
    device_hash: int
    tlvs: tuple[tuple[int, bytes], ...]

    @classmethod
    def decode(cls, payload: bytes):
        if not isinstance(payload, bytes) or not 11 <= len(payload) <= 1010:
            raise ValueError('invalid WLM ability acknowledgement length')
        # Project the shared fields through the already bounded TLV decoder.
        message = AbilityMessage.decode(payload[1:2] + payload[3:4] + payload[4:])
        return cls(payload[0], message.operation, message.flags, message.subtype,
                   (payload[2], message.reserved), message.device_hash, message.tlvs)


def age_discovery(snapshot, now, *, session_active):
    result = deepcopy(snapshot)
    if not math.isfinite(now):
        raise ValueError('WLM observation time must be finite')
    elapsed = max(0., now - result.get('observed_at', now))
    result['observed_at'] = now
    result['session_active'] = bool(session_active)
    for peer in result.get('peers', []):
        peer['age_seconds'] += elapsed
        peer['fresh'] = bool(session_active and peer['age_seconds'] <= result['maximum_age_seconds'])
    result['discovered'] = any(peer['fresh'] for peer in result.get('peers', []))
    return result


class WlmDiscovery:
    """Bounded passive observation; never allocates host IDs or sends a packet."""

    def __init__(self, *, maximum_peers=8, maximum_age=2.):
        if type(maximum_peers) is not int or not 1 <= maximum_peers <= 64:
            raise ValueError('WLM peer bound must be between1 and64')
        if not math.isfinite(maximum_age) or maximum_age <= 0:
            raise ValueError('WLM maximum age must be finite and positive')
        self.maximum_peers, self.maximum_age = maximum_peers, maximum_age
        self.decoder = DumlStreamDecoder()
        self.peers = OrderedDict()
        self.beacons = self.abilities = self.ability_acknowledgements = 0
        self.ignored = self.malformed = self.evicted = 0
        self.last_input_time = None

    def feed(self, data, now, event=lambda *args, **kwargs: None, *, beacon_sink=None):
        if not math.isfinite(now) or (self.last_input_time is not None and now < self.last_input_time):
            raise ValueError('WLM receipt time must be finite and monotonic')
        if not isinstance(data, bytes) or len(data) > 1024 * 1024:
            raise ValueError('WLM input exceeds the bounded byte profile')
        self.last_input_time = now
        for cursor in range(0, len(data), 1024):
            for carrier, _ in self.decoder.feed(data[cursor:cursor + 1024]):
                if (carrier.version, carrier.sender, carrier.receiver, carrier.cmd_set, carrier.cmd_id) != (
                        1, 0x27, 0xee, 0x51, 1) or carrier.cmd_type not in PLAINTEXT_PUSH_TYPES:
                    self.ignored += 1
                    continue
                try:
                    forwarded = unpack_forwarded(carrier.payload)
                except ValueError:
                    self.malformed += 1
                    continue
                packet = forwarded.packet
                is_ability_ack = packet.cmd_id == 0x41 and packet.cmd_type in ABILITY_ACK_TYPES
                if (forwarded.link_category, packet.sender, packet.receiver, packet.cmd_set) != (
                        2, 0xe9, 0xee, 0x51) or (
                        packet.cmd_type not in PLAINTEXT_PUSH_TYPES and not is_ability_ack):
                    self.ignored += 1
                    continue
                if packet.cmd_id == 0x13:
                    try:
                        beacon = WlmBeacon(packet.payload,
                            int.from_bytes(forwarded.attributes[:4], 'little'),
                            int.from_bytes(forwarded.attributes[9:13], 'little'))
                    except ValueError:
                        self.malformed += 1
                        continue
                    self.beacons += 1
                    key = hashlib.sha256(beacon.device_id).hexdigest()
                    first = key not in self.peers
                    if first and len(self.peers) == self.maximum_peers:
                        self.peers.popitem(last=False)
                        self.evicted += 1
                    previous = self.peers.pop(key, {})
                    self.peers[key] = {**beacon.metadata(), 'received_at': now,
                        'beacons': previous.get('beacons', 0) + 1, 'link_category': 2,
                        'sender': packet.sender, 'receiver': packet.receiver}
                    if beacon_sink is not None:
                        beacon_sink(beacon,now)
                    if first:
                        event('wlm_peer_discovered', **beacon.metadata(), link_category=2,
                              application_ready=False)
                elif packet.cmd_id == 0x41:
                    try:
                        message = (AbilityAcknowledgement.decode(packet.payload) if is_ability_ack
                                   else AbilityMessage.decode(packet.payload))
                    except ValueError:
                        self.malformed += 1
                        continue
                    if is_ability_ack:
                        self.ability_acknowledgements += 1
                        event('wlm_ability_ack_observed', status=message.status,
                              operation=message.operation, subtype=message.subtype,
                              tlv_types=[kind for kind, _ in message.tlvs], application_ready=False)
                    else:
                        self.abilities += 1
                        event('wlm_ability_observed', operation=message.operation, subtype=message.subtype,
                              tlv_types=[kind for kind, _ in message.tlvs], application_ready=False)
                else:
                    self.ignored += 1

    def snapshot(self, now, *, session_active=True):
        if not math.isfinite(now) or (self.last_input_time is not None and now < self.last_input_time):
            raise ValueError('WLM snapshot time must be finite and monotonic')
        self.last_input_time = now
        peers = [{key: value for key, value in peer.items() if key != 'received_at'} |
                 {'age_seconds': now - peer['received_at'], 'fresh': False}
                 for peer in self.peers.values()]
        return age_discovery({'observed_at': now, 'maximum_age_seconds': self.maximum_age,
            'beacons_received': self.beacons, 'ability_messages_received': self.abilities,
            'ability_acknowledgements_received': self.ability_acknowledgements,
            'maximum_peers': self.maximum_peers, 'peers': peers, 'ignored_frames': self.ignored,
            'malformed_frames': self.malformed, 'evicted_peers': self.evicted,
            'discarded_bytes': self.decoder.discarded_bytes, 'negotiation_enabled': False,
            'negotiated': False, 'application_ready': False}, now, session_active=session_active)
