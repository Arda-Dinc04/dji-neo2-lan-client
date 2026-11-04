"""DJI's compact payload carried by DUML ``0x00/0x99``.

This is an offline serializer only.  It does not open a transport or send a
packet to an aircraft.
"""

from __future__ import annotations

from dataclasses import dataclass

from .duml import neo2_xrce_agent
from .transport import ReliableSendState


DDS_SUPPORT_VERSION = 2
_MINSTD_MODULUS = 2_147_483_647
_MINSTD_MULTIPLIER = 48_271
_DDS_SEQUENCE_MAX = 0xFFFE


def _bounded_integer(name: str, value: int, maximum: int) -> int:
    if not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if not 0 <= value <= maximum:
        raise ValueError(f"{name} must be between 0 and {maximum}")
    return value


@dataclass
class DdsSequence:
    """Offline model of ``XrceDdsClient``'s 16-bit sequence allocator.

    ``value`` is the stored base.  Like the native implementation, ``next``
    increments before returning and wraps ``0xffff`` to 1.
    """

    value: int

    def __post_init__(self) -> None:
        _bounded_integer("value", self.value, _DDS_SEQUENCE_MAX)
        if self.value == 0:
            raise ValueError("value must be between 1 and 65534")

    @classmethod
    def from_random_word(cls, random_word: int) -> DdsSequence:
        """Reproduce the initialization performed by the native SDK."""

        random_word = _bounded_integer("random_word", random_word, 0xFFFFFFFF)
        seed = random_word % _MINSTD_MODULUS
        if seed == 0:
            seed = 1
        generated = (seed * _MINSTD_MULTIPLIER) % _MINSTD_MODULUS
        return cls(1 + generated % _DDS_SEQUENCE_MAX)

    def next(self) -> int:
        self.value += 1
        if self.value >= 0xFFFF:
            self.value = 1
        return self.value


@dataclass(frozen=True)
class AgentPushHeader:
    dds_version: int
    command: int


@dataclass(frozen=True)
class ObserverRegistryReset:
    """Command 3: the observed callback clears the manager observer registry."""

    header: AgentPushHeader


@dataclass(frozen=True)
class TopicDataPush:
    """Command 6 topic data after the native byte-order reconstruction."""

    header: AgentPushHeader
    topic_base_id: int
    sequence: int
    topic_base: bytes
    host_type: int
    host_index: int
    data: bytes


@dataclass(frozen=True)
class DdsVersionAnnouncement:
    """Command 7: announce/update the DDS support version in byte zero."""

    header: AgentPushHeader


@dataclass(frozen=True)
class UnknownAgentPush:
    header: AgentPushHeader
    payload: bytes


AgentPush = (
    ObserverRegistryReset
    | TopicDataPush
    | DdsVersionAnnouncement
    | UnknownAgentPush
)


@dataclass(frozen=True)
class SubscriptionEnvelope:
    """Three exact offline layers for one command-2 subscription request."""

    agent_payload: bytes
    duml_packet: bytes
    swudp_datagram: bytes


def parse_agent_push(payload: bytes) -> AgentPush:
    """Parse the inner payload consumed by ``XrceDdsClient::ParsePushData``.

    This parser is intentionally stricter than the native code: all declared
    lengths must describe the complete supplied buffer.  It is suitable for
    inspecting captures, not for accepting untrusted traffic on a socket.
    """

    if not isinstance(payload, bytes):
        raise TypeError("payload must be bytes")
    if len(payload) < 2:
        raise ValueError("agent push needs at least the version and command bytes")

    header = AgentPushHeader(dds_version=payload[0], command=payload[1])
    if header.command == 3:
        return ObserverRegistryReset(header)
    if header.command == 7:
        return DdsVersionAnnouncement(header)
    if header.command != 6:
        return UnknownAgentPush(header, payload)

    if len(payload) < 15:
        raise ValueError("command 6 is shorter than its fixed topic header")
    remaining_length = int.from_bytes(payload[11:13], "little")
    topic_length = int.from_bytes(payload[13:15], "little")
    if remaining_length <= 2:
        raise ValueError("command 6 remaining length must be greater than 2")
    if topic_length + 10 > remaining_length:
        raise ValueError("command 6 topic length exceeds the remaining frame")
    if len(payload) != 13 + remaining_length:
        raise ValueError("command 6 declared length does not match the payload")

    tail_offset = 15 + topic_length
    if tail_offset + 8 > len(payload):
        raise ValueError("command 6 is shorter than its host/data trailer")
    data_length = int.from_bytes(payload[tail_offset + 6 : tail_offset + 8], "little")
    expected_remaining = topic_length + 10 + data_length
    if remaining_length != expected_remaining:
        raise ValueError("command 6 data length does not match the remaining frame")

    return TopicDataPush(
        header=header,
        topic_base_id=int.from_bytes(payload[2:4], "little"),
        sequence=int.from_bytes(payload[4:6], "little"),
        topic_base=payload[15:tail_offset],
        host_type=payload[tail_offset + 1],
        host_index=payload[tail_offset],
        data=payload[tail_offset + 8 :],
    )


def serialize_agent_request(
    topic_base: str | bytes,
    *,
    dds_version: int,
    command: int,
    topic_base_id: int,
    sequence: int,
    host_type: int,
    host_index: int,
) -> bytes:
    """Serialize the inner request payload passed to ``SendDDSPack``.

    ``host_index`` precedes ``host_type`` on the wire, matching the native
    ``HostId`` byte accesses.
    """

    if isinstance(topic_base, str):
        topic = topic_base.encode("utf-8")
    elif isinstance(topic_base, bytes):
        topic = topic_base
    else:
        raise TypeError("topic_base must be str or bytes")
    if len(topic) > 0xFFFF - 6:
        raise ValueError("topic_base is too long for the 16-bit framed length")

    dds_version = _bounded_integer("dds_version", dds_version, 0xFF)
    command = _bounded_integer("command", command, 0xFF)
    topic_base_id = _bounded_integer("topic_base_id", topic_base_id, 0xFFFF)
    sequence = _bounded_integer("sequence", sequence, 0xFFFF)
    host_type = _bounded_integer("host_type", host_type, 0xFF)
    host_index = _bounded_integer("host_index", host_index, 0xFF)

    length = len(topic)
    payload = bytearray(length + 19)
    payload[0] = dds_version
    payload[1] = command
    payload[2:4] = topic_base_id.to_bytes(2, "little")
    payload[4:6] = sequence.to_bytes(2, "little")
    # Bytes 6..10 are reserved and remain zero.
    payload[11:13] = (length + 6).to_bytes(2, "little")
    payload[13:15] = length.to_bytes(2, "little")
    payload[15 : 15 + length] = topic
    payload[15 + length] = host_index
    payload[16 + length] = host_type
    # The final two reserved bytes remain zero.
    return bytes(payload)


def serialize_subscription(
    topic_base: str | bytes,
    *,
    sequence: int,
    topic_base_id: int = 0,
    host_type: int = 0,
    host_index: int = 0,
    dds_version: int = DDS_SUPPORT_VERSION,
) -> bytes:
    """Serialize command 2, the value used by ``XrceDdsClient::Subscribe``."""

    return serialize_agent_request(
        topic_base,
        dds_version=dds_version,
        command=2,
        topic_base_id=topic_base_id,
        sequence=sequence,
        host_type=host_type,
        host_index=host_index,
    )


def encapsulate_subscription(
    send_state: ReliableSendState,
    topic_base: str | bytes,
    *,
    dds_sequence: int,
    duml_sequence: int,
    topic_base_id: int = 0,
    host_type: int = 0,
    host_index: int = 0,
    dds_version: int = DDS_SUPPORT_VERSION,
) -> SubscriptionEnvelope:
    """Build ``XRCE -> DUML 0x00/0x99 -> reliable SWUDP type 5``.

    The caller owns both independent application counters and the ACK-gated
    reliable state.  No socket is opened and no datagram is transmitted.
    """

    agent_payload = serialize_subscription(
        topic_base,
        sequence=dds_sequence,
        topic_base_id=topic_base_id,
        host_type=host_type,
        host_index=host_index,
        dds_version=dds_version,
    )
    duml_packet = neo2_xrce_agent(duml_sequence, agent_payload).encode()
    swudp_datagram = send_state.send(duml_packet)
    return SubscriptionEnvelope(
        agent_payload=agent_payload,
        duml_packet=duml_packet,
        swudp_datagram=swudp_datagram,
    )
