"""DJI DUML framing codec and bit-level annotations.

The framing implementation is derived from public DJI reverse-engineering
research. Command meanings are not assumed to be compatible with the Neo 2.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
import secrets
from typing import Iterable, Iterator

SOF = 0x55
DUML_SEQUENCE_MIN = 0x8000
DUML_SEQUENCE_MAX = 0xFFFE
MINSTD_MODULUS = 2_147_483_647
MINSTD_MULTIPLIER = 48_271

STATIC_CAPABILITY_BODY_VERSION = 1
STATIC_CAPABILITY_MAX_BATCH = 21
STATIC_CAPABILITY_DEVELOPER_SETTING = 4
STATIC_CAPABILITY_WIFI_MEDIA_RATE = 14
XRCE_AGENT_COMMAND_SET = 0x00
XRCE_AGENT_COMMAND_ID = 0x99


def encode_host_id(device_type: int, device_index: int) -> int:
    """Pack the native ``HostId(type, index)`` into one DUML wire byte."""

    if not 0 <= device_type <= 0x1F:
        raise ValueError("DUML device type must fit in 5 bits")
    if not 0 <= device_index <= 0x07:
        raise ValueError("DUML device index must fit in 3 bits")
    return device_type | (device_index << 5)


def decode_host_id(value: int) -> tuple[int, int]:
    """Return ``(device_type, device_index)`` from a DUML wire byte."""

    value &= 0xFF
    return value & 0x1F, value >> 5


def _crc_table(polynomial):
    table=[]
    for value in range(256):
        for _ in range(8):
            value=(value>>1)^polynomial if value&1 else value>>1
        table.append(value)
    return tuple(table)


_CRC8_TABLE=_crc_table(0x8C)
_CRC16_TABLE=_crc_table(0x8408)


def crc8(data: bytes, init: int = 0x77) -> int:
    crc = init
    for byte in data:
        value=crc^byte
        crc=(value>>8)^_CRC8_TABLE[value&0xff]
    return crc & 0xFF


def crc16(data: bytes, init: int = 0x3692) -> int:
    crc = init
    for byte in data:
        value=crc^byte
        crc=(value>>8)^_CRC16_TABLE[value&0xff]
    return crc & 0xFFFF


@dataclass(frozen=True)
class BitField:
    name: str
    offset_bits: int
    width_bits: int
    value: int | str
    description: str = ""

    def as_dict(self) -> dict[str, int | str]:
        return {
            "name": self.name,
            "offset_bits": self.offset_bits,
            "width_bits": self.width_bits,
            "value": self.value,
            "description": self.description,
        }


@dataclass(frozen=True)
class DumlPacket:
    sender: int
    receiver: int
    seq: int
    cmd_type: int
    cmd_set: int
    cmd_id: int
    payload: bytes = b""
    version: int = 1

    def encode(self) -> bytes:
        length = 13 + len(self.payload)
        if not 13 <= length <= 0x3FF:
            raise ValueError(f"invalid DUML packet length: {length}")
        version_length = (self.version << 10) | length
        header = bytes((SOF, version_length & 0xFF, version_length >> 8))
        header += bytes((crc8(header),))
        body = header + bytes(
            (
                self.sender & 0xFF,
                self.receiver & 0xFF,
                self.seq & 0xFF,
                self.seq >> 8,
                self.cmd_type & 0xFF,
                self.cmd_set & 0xFF,
                self.cmd_id & 0xFF,
            )
        ) + self.payload
        checksum = crc16(body)
        return body + checksum.to_bytes(2, "little")

    @classmethod
    def decode(cls, raw: bytes) -> "DumlPacket":
        # Receivers inspect the same checked frame independently. Reuse only
        # immutable byte parses, never route, generation, age or authority.
        # Bound both entry count and input size; preserve other input types.
        if cls is DumlPacket and type(raw) is bytes and len(raw)<=0x3ff:
            return _decode_immutable_frame(raw)
        return cls._decode_uncached(raw)

    @classmethod
    def _decode_uncached(cls, raw: bytes) -> "DumlPacket":
        if len(raw) < 13:
            raise ValueError("DUML packet is shorter than 13 bytes")
        if raw[0] != SOF:
            raise ValueError(f"unexpected DUML SOF 0x{raw[0]:02x}")
        if crc8(raw[:3]) != raw[3]:
            raise ValueError("DUML header CRC-8 mismatch")
        version_length = int.from_bytes(raw[1:3], "little")
        length = version_length & 0x3FF
        if length < 13 or len(raw) < length:
            raise ValueError(f"truncated DUML packet: {len(raw)} < {length}")
        packet = raw[:length]
        expected = crc16(packet[:-2])
        observed = int.from_bytes(packet[-2:], "little")
        if expected != observed:
            raise ValueError(
                f"DUML CRC-16 mismatch: expected 0x{expected:04x}, got 0x{observed:04x}"
            )
        return cls(
            sender=packet[4],
            receiver=packet[5],
            seq=int.from_bytes(packet[6:8], "little"),
            cmd_type=packet[8],
            cmd_set=packet[9],
            cmd_id=packet[10],
            payload=packet[11:-2],
            version=version_length >> 10,
        )

    def fields(self) -> list[BitField]:
        length = 13 + len(self.payload)
        values = [
            BitField("sof", 0, 8, SOF, "Start of frame"),
            BitField("length", 8, 10, length, "Whole packet length"),
            BitField("version", 18, 6, self.version, "DUML framing version"),
            BitField("header_crc8", 24, 8, self.encode()[3], "CRC-8 over bits 0..23"),
            BitField("sender", 32, 8, self.sender, "DJI device identifier"),
            BitField("receiver", 40, 8, self.receiver, "DJI device identifier"),
            BitField("sequence", 48, 16, self.seq, "Little-endian sequence"),
            BitField("command_type", 64, 8, self.cmd_type, "Flags and request/response type"),
            BitField("command_set", 72, 8, self.cmd_set, "Subsystem"),
            BitField("command_id", 80, 8, self.cmd_id, "Command within subsystem"),
        ]
        if self.payload:
            values.append(BitField("payload", 88, len(self.payload) * 8, self.payload.hex(), "Opaque until observed"))
        values.append(BitField("packet_crc16", (length - 2) * 8, 16, int.from_bytes(self.encode()[-2:], "little"), "CRC-16 over preceding bytes"))
        return values


@lru_cache(maxsize=256)
def _decode_immutable_frame(raw):
    return DumlPacket._decode_uncached(raw)


@dataclass
class DumlSequence:
    """Neo 2 DUML sequence allocator reconstructed from DJI Fly.

    This counter is application-level state and is independent from the
    reliable-UDP send window. ``current`` is the encoder's stored value;
    :meth:`next` mirrors ``UavProtocolEncoder::GetNextSequenceNumber``.
    """

    current: int = field(
        default_factory=lambda: DumlSequence.initial_from_random_word(
            secrets.randbits(32)
        )
    )

    def __post_init__(self) -> None:
        if not DUML_SEQUENCE_MIN <= self.current <= DUML_SEQUENCE_MAX:
            raise ValueError(
                "DUML sequence state must be between 0x8000 and 0xfffe"
            )

    @staticmethod
    def initial_from_random_word(random_word: int) -> int:
        """Reproduce the encoder constructor from a 32-bit random-device word."""

        seed = (random_word & 0xFFFFFFFF) % MINSTD_MODULUS
        if seed == 0:
            seed = 1
        generated = (seed * MINSTD_MULTIPLIER) % MINSTD_MODULUS
        sequence_span = DUML_SEQUENCE_MAX - DUML_SEQUENCE_MIN + 1
        return DUML_SEQUENCE_MIN + (generated % sequence_span)

    def next(self) -> int:
        candidate = self.current + 1
        self.current = (
            DUML_SEQUENCE_MIN if candidate >= 0xFFFF else candidate
        )
        return self.current


def neo2_heartbeat(sequence: int) -> DumlPacket:
    """Build the exact V1 heartbeat emitted by DJI Fly for the observed Neo 2 state.

    The native path uses ground host ``0x0200`` and aircraft host ``0x0801``.
    On the wire those two-part host IDs become ``0x02`` and ``0x28``. The
    four-byte body is the version-2 body observed with datalink type 255,
    application state 0, and no keep-aircraft-active request.
    """

    return DumlPacket(
        sender=encode_host_id(0x02, 0),
        receiver=encode_host_id(0x08, 1),
        seq=sequence & 0xFFFF,
        cmd_type=0x00,
        cmd_set=0x00,
        cmd_id=0x00,
        payload=bytes.fromhex("00020100"),
    )


def neo2_link_device_info(
    sequence: int,
    *,
    physical_port: int,
    device_id: str,
) -> DumlPacket:
    """Build the 24-byte WLM link-info body, without selecting a radio route.

    SendDeviceSnAndWifiLinkInfo uses host 7.0 and a 20-byte identity field.
    The physical port is explicit: its BLE value is not inferred from UDP.
    The same body is constructed for raw-link and primary-WLM routing.
    """

    if not 1 <= physical_port <= 0xFFFF:
        raise ValueError("physical link port must be between 1 and 65535")
    try:
        identity = device_id.encode("ascii")
    except UnicodeEncodeError as error:
        raise ValueError("link device ID must contain ASCII only") from error
    if not 1 <= len(identity) <= 20 or b"\0" in identity:
        raise ValueError("link device ID must contain 1..20 non-NUL ASCII bytes")
    payload = bytearray(24)
    payload[0] = 1
    payload[1:3] = physical_port.to_bytes(2, "little")
    payload[4:4 + len(identity)] = identity
    return DumlPacket(
        sender=encode_host_id(2, 0), receiver=encode_host_id(7, 0),
        seq=sequence & 0xFFFF, cmd_type=0x40, cmd_set=7, cmd_id=0x93,
        payload=bytes(payload),
    )


def neo2_wifi_device_info(
    sequence: int,
    *,
    physical_port: int,
    device_id: str,
) -> DumlPacket:
    """Build the Neo 2 V1 Wi-Fi software-device information request.

    The request reports the physical SWUDP port and a caller-owned device
    identifier. It targets the dedicated Wi-Fi host and requests one DUML
    response; it is not a flight-control command.
    """

    if not 1 <= physical_port <= 0xFFFF:
        raise ValueError("physical Wi-Fi port must be between 1 and 65535")
    try:
        encoded_device_id = device_id.encode("ascii")
    except UnicodeEncodeError as error:
        raise ValueError("Wi-Fi device ID must contain ASCII only") from error
    if not 1 <= len(encoded_device_id) <= 24:
        raise ValueError("Wi-Fi device ID must contain between 1 and 24 bytes")

    payload = bytearray(31)
    payload[0] = 1
    payload[1:3] = physical_port.to_bytes(2, "little")
    payload[3] = 4
    payload[7 : 7 + len(encoded_device_id)] = encoded_device_id
    return DumlPacket(
        sender=encode_host_id(0x02, 0),
        receiver=encode_host_id(0x1B, 0),
        seq=sequence & 0xFFFF,
        cmd_type=0x40,
        cmd_set=0x07,
        cmd_id=0x93,
        payload=bytes(payload),
    )


def neo2_static_capability(
    sequence: int,
    capability_ids: Iterable[int],
) -> DumlPacket:
    """Build the Neo 2 V1 static-capability read request.

    Both observed producers instantiate the same ``uav_cmd_base_req<1, 0, 183>``
    and target the aircraft host. The body is ``version | uint16 count |
    uint16 identifiers``, so its length is exactly ``3 + 2N``. The native Wi-Fi
    producer splits its identifier list into batches of at most 21; a caller
    asking for more is building a packet the application never emits.

    The request reads declared capabilities and applies no setting to the
    drone: it is not a motor or flight command.
    """

    identifiers = tuple(capability_ids)
    if not 1 <= len(identifiers) <= STATIC_CAPABILITY_MAX_BATCH:
        raise ValueError(
            "static capability batch must hold between 1 and "
            f"{STATIC_CAPABILITY_MAX_BATCH} identifiers"
        )
    payload = bytearray((STATIC_CAPABILITY_BODY_VERSION,))
    payload += len(identifiers).to_bytes(2, "little")
    for identifier in identifiers:
        if not 0 <= identifier <= 0xFFFF:
            raise ValueError("static capability identifier must fit in 16 bits")
        payload += identifier.to_bytes(2, "little")
    return DumlPacket(
        sender=encode_host_id(0x02, 0),
        receiver=encode_host_id(0x08, 1),
        seq=sequence & 0xFFFF,
        cmd_type=0x40,
        cmd_set=0x00,
        cmd_id=0xB7,
        payload=bytes(payload),
    )


def neo2_xrce_agent(sequence: int, payload: bytes) -> DumlPacket:
    """Wrap one DJI proprietary XRCE-agent payload in its Neo 2 DUML request.

    Static analysis proves that ``uav_cmd_base_req<1, 0, 153>`` uses mode 2,
    which the native encoder places in command-type bits 6:5 as ``0x40``.
    ``SendDDSPack`` then targets aircraft host ``8.1`` from ground host
    ``2.0``.  This function only builds bytes and performs no I/O.
    """

    if not isinstance(payload, bytes):
        raise TypeError("XRCE agent payload must be bytes")
    if not payload:
        raise ValueError("XRCE agent payload must not be empty")
    if len(payload) > 0x3FF - 13:
        raise ValueError("XRCE agent payload exceeds the DUML length field")
    return DumlPacket(
        sender=encode_host_id(0x02, 0),
        receiver=encode_host_id(0x08, 1),
        seq=sequence & 0xFFFF,
        cmd_type=0x40,
        cmd_set=XRCE_AGENT_COMMAND_SET,
        cmd_id=XRCE_AGENT_COMMAND_ID,
        payload=payload,
    )


def find_packets(blob: bytes) -> Iterator[tuple[int, DumlPacket, bytes]]:
    offset = 0
    while offset + 4 <= len(blob):
        if blob[offset] != SOF or crc8(blob[offset : offset + 3]) != blob[offset + 3]:
            offset += 1
            continue
        length = int.from_bytes(blob[offset + 1 : offset + 3], "little") & 0x3FF
        if length < 13 or offset + length > len(blob):
            offset += 1
            continue
        raw = blob[offset : offset + length]
        try:
            packet = DumlPacket.decode(raw)
        except ValueError:
            offset += 1
            continue
        yield offset, packet, raw
        offset += length
