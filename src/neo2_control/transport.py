"""Candidate DJI reliable UDP transport, instrumented for Neo 2 validation."""

from __future__ import annotations

from dataclasses import dataclass, field
import secrets

from .duml import BitField


TRANSPORT_HEADER_LENGTH = 8
STREAM_HEADER_LENGTH = 20
TYPE_1_HEADER_LENGTH = 34
MAX_SINGLE_FRAGMENT_PAYLOAD = 1452
MAX_DATAGRAM_LENGTH = 1472


def xor_checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


SYN_TAIL = bytes.fromhex(
    "c0af64006400c005140000640000019001c005140000640014006400c00514000064000101040102"
)


@dataclass(frozen=True)
class ReceiveWindowStatus:
    """Native receive frontier plus ordered missing sequence/state words."""

    start: int
    end: int
    missing: tuple[int, ...] = ()

    def encode(self, *, compressed: bool = True) -> bytes:
        if not all(type(v) is int and 0 <= v <= 0xFFFF for v in (self.start, self.end, *self.missing)):
            raise ValueError("receive sequence words must be unsigned 16-bit integers")
        window_slots = ((self.end - (self.start & 0xFFF8)) & 0xFFFF) >> 3
        distances = [((seq - (self.start & 0xFFF8)) & 0xFFFF) >> 3 for seq in self.missing]
        if window_slots > 256 or any(not 1 <= d <= window_slots for d in distances):
            raise ValueError("missing sequence is outside the bounded receive window")
        if distances != sorted(set(distances)):
            raise ValueError("missing sequences must be unique and ordered from the frontier")
        window = self.start.to_bytes(2, "little") + self.end.to_bytes(2, "little")
        if not compressed:
            return window + len(self.missing).to_bytes(2, "little") + b"".join(
                seq.to_bytes(2, "little") for seq in self.missing
            )
        if not self.missing:
            return window + bytes(4)
        slot_count = distances[-1] - distances[0] + 1
        bitmap = bytearray(b"\xff" * ((slot_count + 3) // 4))
        for seq, distance in zip(self.missing, distances):
            slot = distance - distances[0]
            state = min((seq >> 1) & 3, 2)
            shift = (slot % 4) * 2
            bitmap[slot // 4] = (bitmap[slot // 4] & ~(3 << shift)) | (state << shift)
        return window + slot_count.to_bytes(2, "little") + distances[0].to_bytes(2, "little") + bytes(bitmap)


@dataclass
class ReliableTransport:
    session: int = field(default_factory=lambda: secrets.randbelow(0xEFFF) + 0x1000)
    base_sequence: int = field(default_factory=lambda: (secrets.randbelow(0xE000) + 0x1000) & 0xFFF8)
    ack_mode: int = 1

    def __post_init__(self) -> None:
        self.session &= 0xFFFF
        self.base_sequence &= 0xFFF8
        if self.ack_mode not in (0, 1):
            raise ValueError("ACK mode must be 0 or 1")

    def open_session(self) -> bytes:
        tail = bytearray(SYN_TAIL)
        tail[:2] = self.base_sequence.to_bytes(2, "little")
        return _build_datagram(self.session, 0, bytes(tail))

    def idle_window(
        self,
        *,
        periodic: bool = True,
        send_window_start: int | None = None,
        send_window_end: int | None = None,
        video_receive: ReceiveWindowStatus | None = None,
        protocol_receive: ReceiveWindowStatus | None = None,
        video_changes: tuple[int, ...] = (),
        protocol_changes: tuple[int, ...] = (),
    ) -> bytes:
        """Build a ground ACK: periodic type 6 carries incremental lists.

        Type 4 advertises all holes, compressed in negotiated mode 1 and
        explicit in mode 0. Callers supply the receiver's ordered event FIFO
        for type 6, then consume it after either ACK form is built successfully.
        """
        return self._ground_datagram(
            b"", periodic=periodic, send_window_start=send_window_start,
            send_window_end=send_window_end, video_receive=video_receive,
            protocol_receive=protocol_receive,
            video_changes=video_changes, protocol_changes=protocol_changes,
        )

    def qos_data(
        self, payload: bytes, *, periodic: bool = False,
        send_window_start: int | None = None, send_window_end: int | None = None,
        video_receive: ReceiveWindowStatus | None = None,
        protocol_receive: ReceiveWindowStatus | None = None,
        video_changes: tuple[int, ...] = (),
        protocol_changes: tuple[int, ...] = (),
    ) -> bytes:
        """Encode raw DUML bytes without advancing the legacy reliable window.

        QoS queue draining does not create a reliable fragment, transport ACK
        obligation or automatic replay of this payload.
        """
        if not payload:
            raise ValueError("QoS application payload must not be empty")
        if len(payload) > MAX_DATAGRAM_LENGTH - 30:
            raise ValueError("ground datagram exceeds the native 1472-byte MTU")
        return self._ground_datagram(
            payload, periodic=periodic, send_window_start=send_window_start,
            send_window_end=send_window_end, video_receive=video_receive,
            protocol_receive=protocol_receive,
            video_changes=video_changes, protocol_changes=protocol_changes,
        )

    def _ground_datagram(
        self, payload: bytes, *, periodic: bool,
        send_window_start: int | None, send_window_end: int | None,
        video_receive: ReceiveWindowStatus | None,
        protocol_receive: ReceiveWindowStatus | None,
        video_changes: tuple[int, ...], protocol_changes: tuple[int, ...],
    ) -> bytes:
        initial = ReceiveWindowStatus(self.base_sequence, self.base_sequence)
        statuses = {"video": video_receive or initial, "protocol": protocol_receive or initial}
        blocks = []
        for channel, status in statuses.items():
            # The event FIFO may include superseded states and words behind
            # the frontier. Validate full window and FIFO independently.
            status.encode(compressed=False)
            if periodic:
                changes = video_changes if channel == 'video' else protocol_changes
                if len(changes) > 256 or any(type(word) is not int or not 0 <= word <= 0xffff for word in changes):
                    raise ValueError("ACK event FIFO must contain at most 256 unsigned 16-bit words")
                blocks.append(status.start.to_bytes(2, 'little') + status.end.to_bytes(2, 'little')
                              + len(changes).to_bytes(2, 'little')
                              + b''.join(word.to_bytes(2, 'little') for word in changes))
            else:
                blocks.append(status.encode(compressed=self.ack_mode == 1))
        start = self.base_sequence if send_window_start is None else send_window_start
        end = self.base_sequence if send_window_end is None else send_window_end
        send_window = (
            (start & 0xFFFF).to_bytes(2, "little")
            + (end & 0xFFFF).to_bytes(2, "little")
            + b"\x00\x00\x00\x00"
        )
        body = b"".join(blocks) + send_window + len(payload).to_bytes(2, "little") + payload
        if TRANSPORT_HEADER_LENGTH + len(body) > MAX_DATAGRAM_LENGTH:
            raise ValueError("ground datagram exceeds the native 1472-byte MTU")
        result = _build_datagram(self.session, 6 if periodic else 4, body)
        return result

    def first_reliable_data(self, payload: bytes) -> bytes:
        """Build the first one-fragment ground-to-Neo-2 reliable datagram.

        This reproduces the native direct-send path selected when negotiated
        feature bit 1 is clear. It deliberately models only the first packet
        after link initialization: later packets depend on ACK processing and
        must not be synthesized by merely incrementing counters.
        """

        send_sequence = (self.base_sequence + 8) & 0xFFFF
        return self._reliable_data(
            payload,
            window_start=self.base_sequence,
            send_sequence=send_sequence,
            stream_slot=1,
        )

    def _reliable_data(
        self,
        payload: bytes,
        *,
        window_start: int,
        send_sequence: int,
        stream_slot: int,
    ) -> bytes:
        if not payload:
            raise ValueError("reliable application payload must not be empty")
        if len(payload) > MAX_SINGLE_FRAGMENT_PAYLOAD:
            raise ValueError(
                f"single-fragment payload exceeds {MAX_SINGLE_FRAGMENT_PAYLOAD} bytes"
            )
        send_window = (
            (window_start & 0xFFFF).to_bytes(2, "little")
            + (send_sequence & 0xFFFF).to_bytes(2, "little")
            + b"\x00\x00\x00\x00"
        )
        # One-fragment group, no tail-length flag, fragment index zero.
        stream_word = (stream_slot & 0xFF) | (1 << 8)
        body = send_window + stream_word.to_bytes(4, "little") + payload
        return _build_datagram(
            self.session,
            0x05,
            body,
            send_sequence=send_sequence,
        )


@dataclass
class ReliableSendState:
    """ACK-gated native send-window state for sequential one-fragment packets."""

    transport: ReliableTransport
    acknowledged_sequence: int = field(init=False)
    next_stream_slot: int = 1
    waiting_for_ack: int | None = None

    def __post_init__(self) -> None:
        self.acknowledged_sequence = self.transport.base_sequence

    def send(self, payload: bytes) -> bytes:
        if self.waiting_for_ack is not None:
            raise RuntimeError(
                "next reliable payload is blocked until the previous ACK"
            )
        send_sequence = (self.acknowledged_sequence + 8) & 0xFFFF
        frame = self.transport._reliable_data(
            payload,
            window_start=self.acknowledged_sequence,
            send_sequence=send_sequence,
            stream_slot=self.next_stream_slot,
        )
        self.waiting_for_ack = send_sequence
        self.next_stream_slot = (self.next_stream_slot + 1) & 0xFF
        return frame

    def acknowledge(self, raw: bytes) -> bool:
        if self.waiting_for_ack is None:
            return False
        if not is_uplink_ack(
            raw,
            self.transport.session,
            self.waiting_for_ack,
        ):
            return False
        self.acknowledged_sequence = self.waiting_for_ack
        self.waiting_for_ack = None
        return True

    def idle_window(
        self,
        *,
        periodic: bool = True,
        video_receive: ReceiveWindowStatus | None = None,
        protocol_receive: ReceiveWindowStatus | None = None,
        video_changes: tuple[int, ...] = (),
        protocol_changes: tuple[int, ...] = (),
    ) -> bytes:
        """Advertise the actual uplink window, including an unacknowledged send.

        Callers admitting downlink streams supply their actual receive status;
        otherwise those channels remain at the initial anchor.
        """

        return self.transport.idle_window(
            periodic=periodic,
            video_receive=video_receive,
            protocol_receive=protocol_receive,
            video_changes=video_changes, protocol_changes=protocol_changes,
            send_window_start=self.acknowledged_sequence,
            send_window_end=(
                self.acknowledged_sequence
                if self.waiting_for_ack is None
                else self.waiting_for_ack
            ),
        )


def _build_datagram(
    session: int,
    transport_type: int,
    payload: bytes,
    *,
    send_sequence: int = 0,
) -> bytes:
    total = TRANSPORT_HEADER_LENGTH + len(payload)
    if total > 0x3FFF:
        raise ValueError("transport datagram exceeds the 14-bit length field")
    header = bytearray(TRANSPORT_HEADER_LENGTH)
    header[:2] = (0x8000 | total).to_bytes(2, "little")
    header[2:4] = (session & 0xFFFF).to_bytes(2, "little")
    header[4:6] = (send_sequence & 0xFFFF).to_bytes(2, "little")
    header[6] = transport_type & 0xFF
    header[7] = xor_checksum(header[:7])
    return bytes(header) + payload


def _describe_ground_ack_block(
    raw: bytes,
    offset: int,
    *,
    compressed: bool,
    prefix: str,
) -> tuple[dict[str, int], list[BitField], int]:
    minimum = 8 if compressed else 6
    if len(raw) < offset + minimum:
        raise ValueError(f"truncated {prefix} ACK block")

    decoded = {
        f"{prefix}_window_start": int.from_bytes(raw[offset : offset + 2], "little"),
        f"{prefix}_window_end": int.from_bytes(raw[offset + 2 : offset + 4], "little"),
    }
    fields = [
        BitField(
            f"{prefix}_window_start",
            offset * 8,
            16,
            decoded[f"{prefix}_window_start"],
            "Reliable receive-window start",
        ),
        BitField(
            f"{prefix}_window_end",
            (offset + 2) * 8,
            16,
            decoded[f"{prefix}_window_end"],
            "Reliable receive-window end",
        ),
    ]

    if compressed:
        slot_count = int.from_bytes(raw[offset + 4 : offset + 6], "little")
        base_delta = int.from_bytes(raw[offset + 6 : offset + 8], "little")
        bitmap_length = (slot_count + 3) // 4
        end = offset + 8 + bitmap_length
        if len(raw) < end:
            raise ValueError(f"truncated {prefix} ACK bitmap")
        decoded.update(
            {
                f"{prefix}_ack_slot_count": slot_count,
                f"{prefix}_ack_base_delta": base_delta,
                f"{prefix}_ack_bitmap_length": bitmap_length,
            }
        )
        fields.extend(
            (
                BitField(
                    f"{prefix}_ack_slot_count",
                    (offset + 4) * 8,
                    16,
                    slot_count,
                    "Number of two-bit ACK states",
                ),
                BitField(
                    f"{prefix}_ack_base_delta",
                    (offset + 6) * 8,
                    16,
                    base_delta,
                    "Base window delta for the ACK bitmap",
                ),
            )
        )
        return decoded, fields, end

    ack_count = int.from_bytes(raw[offset + 4 : offset + 6], "little")
    end = offset + 6 + ack_count * 2
    if len(raw) < end:
        raise ValueError(f"truncated {prefix} explicit ACK list")
    decoded[f"{prefix}_ack_count"] = ack_count
    fields.append(
        BitField(
            f"{prefix}_ack_count",
            (offset + 4) * 8,
            16,
            ack_count,
            "Number of explicit 16-bit ACK states",
        )
    )
    return decoded, fields, end


def describe_transport(raw: bytes, *, ack_mode: int = 1) -> tuple[dict[str, int | str | bool], list[BitField]]:
    if ack_mode not in (0, 1):
        raise ValueError("ACK mode must be 0 or 1")
    if len(raw) < TRANSPORT_HEADER_LENGTH:
        raise ValueError("transport datagram shorter than 8 bytes")
    length_flags = int.from_bytes(raw[:2], "little")
    declared_length = length_flags & 0x3FFF
    packet_type = raw[6]
    decoded: dict[str, int | str | bool] = {
        "declared_length": declared_length,
        "actual_length": len(raw),
        "flags": length_flags >> 14,
        "session": int.from_bytes(raw[2:4], "little"),
        "send_sequence": int.from_bytes(raw[4:6], "little"),
        "type": packet_type,
        "header_checksum_valid": xor_checksum(raw[:7]) == raw[7],
    }
    fields = [
        BitField("length", 0, 14, declared_length, "Transport datagram length"),
        BitField("flags", 14, 2, length_flags >> 14, "Transport version/flags"),
        BitField("session", 16, 16, decoded["session"], "Session selected by controller"),
        BitField("send_sequence", 32, 16, decoded["send_sequence"], "Reliable window sequence"),
        BitField(
            "transport_type",
            48,
            8,
            packet_type,
            "Neo 2: 0=management, 1=status, 2=video, 3=protocol, "
            "4/6=ground ACK, 5=ground reliable data",
        ),
        BitField("header_xor", 56, 8, raw[7], "XOR of bytes 0..6"),
    ]
    if packet_type == 0x00 and len(raw) >= 9:
        if declared_length == 15 and len(raw) >= 15:
            decoded.update(
                management_mode=raw[8],
                feature_flags=int.from_bytes(raw[9:11], "little"),
                monitor_ack_info=raw[11],
                ack_limit=raw[12],
                max_bitrate_kbps=int.from_bytes(raw[13:15], "little"),
            )
            fields.extend(
                (
                    BitField("management_mode", 64, 8, raw[8], "Negotiated management/link mode"),
                    BitField("feature_flags", 72, 16, decoded["feature_flags"], "Negotiated link feature flags"),
                    BitField("monitor_ack_info", 88, 8, raw[11], "Link-monitor ACK parameter"),
                    BitField("ack_limit", 96, 8, raw[12], "Negotiated ACK limit"),
                    BitField(
                        "max_bitrate_kbps",
                        104,
                        16,
                        decoded["max_bitrate_kbps"],
                        "Maximum monitored bitrate in kbit/s",
                    ),
                )
            )
        elif declared_length in (45, 48) and len(raw) >= declared_length:
            decoded["link_word"] = int.from_bytes(raw[8:10], "little")
            fields.append(
                BitField("link_word", 64, 16, decoded["link_word"], "Random initial window anchor")
            )
    elif packet_type == 0x01 and len(raw) >= 26:
        decoded["acknowledged_uplink"] = int.from_bytes(raw[24:26], "little")
        fields.append(
            BitField(
                "acknowledged_uplink",
                192,
                16,
                decoded["acknowledged_uplink"],
                "Drone ACK of the controller uplink window",
            )
        )
        if len(raw) >= TYPE_1_HEADER_LENGTH:
            # In negotiated mode 1, video/protocol use fixed 8-byte send
            # windows. Only the uplink ACK at 24 has a variable bitmap.
            # SW_Pro_Gnd's receiver at 0x4078d90 walks this layout.
            for prefix, offset in (
                ("drone_video", 8),
                ("drone_protocol", 16),
                ("drone_send", 24),
            ):
                decoded.update(
                    {
                        f"{prefix}_window_start": int.from_bytes(
                            raw[offset : offset + 2], "little"
                        ),
                        f"{prefix}_window_end": int.from_bytes(
                            raw[offset + 2 : offset + 4], "little"
                        ),
                        f"{prefix}_window_flags_0": int.from_bytes(
                            raw[offset + 4 : offset + 6], "little"
                        ),
                        f"{prefix}_window_flags_1": int.from_bytes(
                            raw[offset + 6 : offset + 8], "little"
                        ),
                    }
                )
                fields.extend(
                    (
                        BitField(
                            f"{prefix}_window_start",
                            offset * 8,
                            16,
                            decoded[f"{prefix}_window_start"],
                            "Drone-side reliable window start",
                        ),
                        BitField(
                            f"{prefix}_window_end",
                            (offset + 2) * 8,
                            16,
                            decoded[f"{prefix}_window_end"],
                            "Drone-side reliable window end",
                        ),
                    )
                )
            ack_slots = int.from_bytes(raw[28:30], "little")
            bitmap_length = (ack_slots + 3) // 4
            aggregate_offset = 32 + bitmap_length
            if len(raw) < aggregate_offset + 2:
                raise ValueError("truncated type-1 ACK bitmap or aggregate length")
            aggregate_length = int.from_bytes(raw[aggregate_offset : aggregate_offset + 2], "little")
            decoded.update(
                transport_direction="drone_to_ground",
                uplink_ack_slot_count=ack_slots,
                uplink_ack_base_delta=int.from_bytes(raw[30:32], "little"),
                uplink_ack_bitmap_length=bitmap_length,
                downlink_aggregate_length=aggregate_length,
                payload_offset=aggregate_offset + 2,
                payload_length=max(
                    0,
                    min(len(raw), declared_length) - aggregate_offset - 2,
                ),
            )
            fields.append(
                BitField(
                    "downlink_aggregate_length",
                    aggregate_offset * 8,
                    16,
                    decoded["downlink_aggregate_length"],
                    "DUML bytes following the type-1 state",
                )
            )
    elif packet_type in (0x04, 0x06):
        compressed = ack_mode == 1 and packet_type == 0x04
        cursor = TRANSPORT_HEADER_LENGTH
        for prefix in ("video", "protocol"):
            block, block_fields, cursor = _describe_ground_ack_block(
                raw,
                cursor,
                compressed=compressed,
                prefix=prefix,
            )
            decoded.update(block)
            fields.extend(block_fields)
        if len(raw) < cursor + 10:
            raise ValueError("truncated ground send-window block")
        decoded.update(
            send_window_start=int.from_bytes(raw[cursor : cursor + 2], "little"),
            send_window_end=int.from_bytes(raw[cursor + 2 : cursor + 4], "little"),
            send_window_flags_0=int.from_bytes(raw[cursor + 4 : cursor + 6], "little"),
            send_window_flags_1=int.from_bytes(raw[cursor + 6 : cursor + 8], "little"),
            qos_aggregate_length=int.from_bytes(raw[cursor + 8 : cursor + 10], "little"),
            qos_aggregate_offset=cursor + 10,
        )
        aggregate_end = cursor + 10 + int(decoded["qos_aggregate_length"])
        if aggregate_end > min(len(raw), declared_length):
            raise ValueError("truncated ground QoS aggregate")
        fields.extend(
            (
                BitField("send_window_start", cursor * 8, 16, decoded["send_window_start"], "Ground send-window start"),
                BitField("send_window_end", (cursor + 2) * 8, 16, decoded["send_window_end"], "Ground send-window end"),
                BitField(
                    "send_window_flags_0",
                    (cursor + 4) * 8,
                    16,
                    decoded["send_window_flags_0"],
                    "Ground send-window flags",
                ),
                BitField(
                    "send_window_flags_1",
                    (cursor + 6) * 8,
                    16,
                    decoded["send_window_flags_1"],
                    "Ground send-window flags",
                ),
                BitField(
                    "qos_aggregate_length",
                    (cursor + 8) * 8,
                    16,
                    decoded["qos_aggregate_length"],
                    "Optional QoS aggregate bytes following the transport state",
                ),
            )
        )
    elif packet_type in (0x02, 0x03, 0x05) and len(raw) >= STREAM_HEADER_LENGTH:
        stream_word = int.from_bytes(raw[16:20], "little")
        direction = "ground_to_drone" if packet_type == 0x05 else "drone_to_ground"
        decoded.update(
            transport_direction=direction,
            window_start=int.from_bytes(raw[8:10], "little"),
            window_end=int.from_bytes(raw[10:12], "little"),
            window_flags_0=int.from_bytes(raw[12:14], "little"),
            window_flags_1=int.from_bytes(raw[14:16], "little"),
            stream_word=stream_word,
            stream_slot=stream_word & 0xFF,
            group_span_raw=(stream_word >> 8) & 0x3F,
            tail_length_present=bool((stream_word >> 14) & 1),
            fragment_index=(stream_word >> 15) & 0x3F,
            tail_payload_length=(stream_word >> 21) & 0x7FF,
            payload_offset=STREAM_HEADER_LENGTH,
            payload_length=max(0, min(len(raw), declared_length) - STREAM_HEADER_LENGTH),
        )
        fields.extend(
            (
                BitField("window_start", 64, 16, decoded["window_start"], "Reliable window start"),
                BitField("window_end", 80, 16, decoded["window_end"], "Reliable window end"),
                BitField("window_flags_0", 96, 16, decoded["window_flags_0"], "Window/ACK flags"),
                BitField("window_flags_1", 112, 16, decoded["window_flags_1"], "Window/ACK flags"),
                BitField("stream_slot", 128, 8, decoded["stream_slot"], "Stream block/slot identifier"),
                BitField(
                    "group_span_raw",
                    136,
                    6,
                    decoded["group_span_raw"],
                    "Raw group span used by the receive algorithm",
                ),
                BitField(
                    "tail_length_present",
                    142,
                    1,
                    decoded["tail_length_present"],
                    "Tail-length handling flag",
                ),
                BitField("fragment_index", 143, 6, decoded["fragment_index"], "Fragment index in the stream group"),
                BitField(
                    "tail_payload_length",
                    149,
                    11,
                    decoded["tail_payload_length"],
                    "Encoded final-fragment payload length",
                ),
            )
        )
    return decoded, fields


def extract_stream_payload(raw: bytes) -> bytes:
    """Return a raw type-2/3 body for inspection, before receive processing.

    The native receiver validates/reorders type 2 and 3 datagrams, removes the
    20-byte transport/stream header, filters parity and trims the terminal
    block. Use ReliableReceiveState for ordered, length-correct callbacks.
    """

    decoded, _ = describe_transport(raw)
    if decoded["type"] not in (0x02, 0x03):
        raise ValueError("transport datagram is not a Neo 2 stream packet")
    if len(raw) < STREAM_HEADER_LENGTH:
        raise ValueError("stream datagram shorter than 20 bytes")
    if decoded["declared_length"] > len(raw):
        raise ValueError("truncated stream datagram")
    return raw[STREAM_HEADER_LENGTH : int(decoded["declared_length"])]


def is_session_accept(raw: bytes, session: int) -> bool:
    """Return whether *raw* is the validated 15-byte Neo 2 session response."""

    if len(raw) != 15:
        return False
    try:
        decoded, _ = describe_transport(raw)
    except ValueError:
        return False
    return bool(
        decoded["declared_length"] == len(raw)
        and decoded["flags"] == 2
        and decoded["type"] == 0
        and decoded["session"] == (session & 0xFFFF)
        and decoded["header_checksum_valid"]
        and decoded.get("management_mode") == 1
    )


def is_uplink_ack(raw: bytes, session: int, sequence: int) -> bool:
    """Return whether a type-1 state packet acknowledges one uplink sequence."""

    try:
        decoded, _ = describe_transport(raw)
    except ValueError:
        return False
    return bool(
        decoded["declared_length"] == len(raw)
        and decoded["flags"] == 2
        and decoded["type"] == 1
        and decoded["session"] == (session & 0xFFFF)
        and decoded["header_checksum_valid"]
        and len(raw) >= TYPE_1_HEADER_LENGTH
        and int(decoded.get("uplink_ack_slot_count", 0)) <= 20
        and decoded.get("downlink_aggregate_length") == len(raw) - int(decoded.get("payload_offset", 0))
        and decoded.get("acknowledged_uplink") == (sequence & 0xFFFF)
    )
