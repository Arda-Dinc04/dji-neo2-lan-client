"""Bounded SWUDP video/protocol receive windows reconstructed from the SDK."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .fec import InsufficientShards, recover_data_shards
from .transport import ReceiveWindowStatus, describe_transport


def _distance(sequence: int, anchor: int) -> int:
    return ((sequence - (anchor & 0xFFF8)) & 0xFFFF) >> 3


@dataclass(frozen=True)
class _Fragment:
    word: int
    payload: bytes
    received_at: float | None = None
    recovered: bool = False

    def delivered_payload(self) -> bytes | None:
        if not self.word & (1 << 14):
            return self.payload
        k, index = (self.word >> 8) & 63, (self.word >> 15) & 63
        if index >= k:
            return None
        return self.payload[:self.word >> 21] if index == k - 1 else self.payload


@dataclass(frozen=True)
class DeliveredPayload:
    """One admitted payload with the receipt of its original accepted packet.

    Unknown/reconstructed receipt stays None. Release time belongs to the
    caller that advances the receive frontier, not to this original packet.
    """
    payload: bytes
    sequence: int
    received_at: float | None
    recovered: bool


@dataclass
class _Group:
    base: int
    k: int
    tail: int
    size: int
    word: int
    shards: dict[int, bytes] = field(default_factory=dict)


class ReliableReceiveState:
    """One channel, one session, modulo-8192 sequence slots of eight units.

    No gap is skipped. ``fec=False`` matches the current SYN's receive
    configuration. With FEC enabled, recovered data and parity placeholders
    can advance the frontier only after enough independent shards arrive.
    """

    def __init__(self, *, session: int, anchor: int, channel: int, capacity: int, fec: bool = False):
        if type(channel) is not int or channel not in (2, 3) or type(capacity) is not int or not 1 <= capacity <= 256:
            raise ValueError("receive channel must be 2/3 with capacity 1..256")
        if type(session) is not int or type(anchor) is not int or not 0 <= session <= 0xFFFF or not 0 <= anchor <= 0xFFFF or anchor & 7:
            raise ValueError("session/anchor must be 16-bit, with an aligned anchor")
        self.session, self.channel, self.capacity, self.fec = session, channel, capacity, fec
        self.start = self.end = anchor
        self._pending: dict[int, _Fragment] = {}
        self._missing: dict[int, int] = {}
        self._ack_events: list[int] = []
        self.ack_events_dropped = 0
        self._groups: dict[int, _Group] = {}
        self.fragments_received = self.fragments_recovered = 0
        self.payloads_delivered = self.bytes_delivered = self.duplicates = 0
        self.last_deliveries: tuple[DeliveredPayload, ...] = ()

    @property
    def status(self) -> ReceiveWindowStatus:
        missing = tuple(
            sequence | (self._missing[sequence] << 1)
            for sequence in sorted(self._missing, key=lambda s: _distance(s, self.start))
        )
        return ReceiveWindowStatus(self.start, self.end, missing)

    @property
    def ack_events(self) -> tuple[int, ...]:
        """Native bounded FIFO, including superseded and now-received holes."""
        return tuple(self._ack_events)

    def consume_ack_events(self) -> None:
        """Both a full ACK and an incremental ACK consume the pending FIFO."""
        self._ack_events.clear()

    def _ack_changed(self, sequence: int, state: int) -> None:
        if len(self._ack_events) < self.capacity:
            self._ack_events.append(sequence | (state << 1))
        else:
            self.ack_events_dropped += 1

    def _validate_window(self, sequence: int, start: int, end: int, flags0: int, flags1: int, *, data: bool):
        minimum = 1 if data else 0
        if not minimum <= _distance(sequence, start) <= self.capacity:
            raise ValueError("sequence is outside the advertised send window")
        if not minimum <= _distance(end, start) <= self.capacity:
            raise ValueError("advertised send window exceeds receive capacity")
        if _distance(end, sequence) >= self.capacity:
            raise ValueError("sequence follows the advertised window end")
        for flags in (flags0, flags1):
            if flags & 1 and not (
                1 <= _distance(flags, start) <= self.capacity
                and _distance(end, flags) < self.capacity
            ):
                raise ValueError("retransmission checkpoint is outside the send window")
        if not minimum <= _distance(sequence, self.start) <= self.capacity:
            raise ValueError("stale sequence or receive window overrun")
        if _distance(self.start, start) > self.capacity:
            raise ValueError("peer send frontier is outside the receive window")

    def _advance_end(self, sequence: int, *, include: bool):
        count = _distance(sequence, self.end)
        if not 1 <= count <= self.capacity:
            return
        for step in range(1, count + int(include)):
            missing = ((self.end & 0xFFF8) + step * 8) & 0xFFFF
            if missing not in self._missing:
                self._missing[missing] = 0
                self._ack_changed(missing, 0)
        self.end = sequence

    def _checkpoints(self, flags0: int, flags1: int):
        for checkpoint, previous in ((flags0, 0), (flags1, 1)):
            if checkpoint & 1:
                for sequence, state in self._missing.items():
                    if state == previous and _distance(checkpoint, sequence) < self.capacity:
                        self._missing[sequence] = previous + 1
                        self._ack_changed(sequence, previous + 1)

    def observe_window(self, start: int, end: int, flags0: int = 0, flags1: int = 0):
        """Apply the fixed eight-byte video/protocol window from type 1."""
        if not all(type(v) is int and 0 <= v <= 0xFFFF for v in (start, end, flags0, flags1)):
            raise ValueError("window words must be unsigned 16-bit integers")
        self._validate_window(end, start, end, flags0, flags1, data=False)
        self._advance_end(end, include=True)
        self._checkpoints(flags0, flags1)

    def _recover(self, slot: int, group: _Group):
        try:
            recovered = recover_data_shards(group.k, group.shards)
        except InsufficientShards:
            return
        total = 2 if group.k == 1 else group.k + 2
        for index in range(total):
            sequence = (group.base + index * 8) & 0xFFFF
            if not 1 <= _distance(sequence, self.start) <= self.capacity:
                continue
            if sequence not in self._pending:
                word = (group.word & ~(63 << 15)) | (index << 15)
                payload = recovered[index] if index < group.k else b""
                self._pending[sequence] = _Fragment(word, payload, recovered=True)
                if index < group.k:
                    self.fragments_recovered += 1
            self._missing.pop(sequence, None)
            self._advance_end(sequence, include=False)
        del self._groups[slot]

    def receive(self, raw: bytes, *, received_at: float | None = None) -> tuple[bytes, ...]:
        self.last_deliveries = ()
        if received_at is not None and (type(received_at) not in (int,float)
                or not math.isfinite(received_at) or received_at < 0):
            raise ValueError('receive_packet_clock_invalid')
        decoded, _ = describe_transport(raw)
        if (
            len(raw) < 20 or len(raw) > 1472 or decoded['declared_length'] != len(raw)
            or decoded['type'] != self.channel or decoded['session'] != self.session
            or decoded['flags'] != 2 or not decoded['header_checksum_valid']
        ):
            raise ValueError("invalid receive envelope or channel")
        sequence = int(decoded['send_sequence'])
        start, end = int(decoded['window_start']), int(decoded['window_end'])
        flags0, flags1 = int(decoded['window_flags_0']), int(decoded['window_flags_1'])
        self._validate_window(sequence, start, end, flags0, flags1, data=True)
        sequence &= 0xFFF8
        word, payload = int(decoded['stream_word']), raw[20:]
        if self.fec and not word & (1 << 14):
            raise ValueError("FEC-enabled channel requires the coded-tail wire profile")
        group = None
        if word & (1 << 14):
            k, index, tail, slot = (word >> 8) & 63, (word >> 15) & 63, word >> 21, word & 255
            if not k or index >= (2 if k == 1 else k + 2) or tail > len(payload):
                raise ValueError("invalid FEC group or tail length")
            if self.fec:
                if k > 62:
                    raise ValueError("FEC group exceeds the six-bit wire index capacity")
                if not payload or len(payload) % 4:
                    raise ValueError("FEC payload must be padded to 32-bit words")
                base = (sequence - index * 8) & 0xFFFF
                group = self._groups.get(slot)
                if group is None:
                    if len(self._groups) >= min(self.capacity, 100):
                        raise ValueError("too many incomplete FEC groups")
                    group = _Group(base, k, tail, len(payload), word)
                if (group.base, group.k, group.tail, group.size) != (base, k, tail, len(payload)):
                    raise ValueError("inconsistent metadata or premature FEC slot reuse")

        self.fragments_received += 1
        if sequence in self._pending:
            self.duplicates += 1
        else:
            self._pending[sequence] = _Fragment(word, payload, received_at)
            self._missing.pop(sequence, None)
            self._advance_end(sequence, include=False)
            if group is not None:
                group.shards[index] = payload
                self._groups[slot] = group
                self._recover(slot, group)
        self._advance_end(end, include=True)
        delivered = []
        deliveries = []
        while (next_sequence := (self.start + 8) & 0xFFFF) in self._pending:
            fragment = self._pending.pop(next_sequence)
            self.start = next_sequence
            block = fragment.delivered_payload()
            if block is not None:
                delivered.append(block)
                deliveries.append(DeliveredPayload(block,next_sequence,fragment.received_at,fragment.recovered))
                self.payloads_delivered += 1
                self.bytes_delivered += len(block)
        if not self._missing:
            self.end = self.start | (self.end & 7)
        self._checkpoints(flags0, flags1)
        self.last_deliveries = tuple(deliveries)
        return tuple(delivered)
