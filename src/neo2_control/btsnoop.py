"""Lossless Android/Linux btsnoop reader with BLE ATT and DUML extraction."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
import struct
from typing import Iterator

from .duml import find_packets


BTSNOOP_HEADER = struct.Struct(">8sII")
BTSNOOP_RECORD = struct.Struct(">IIIIQ")
BTSNOOP_EPOCH_DELTA_US = 0x00DCDDB30F2F8000
ATT_CID = 0x0004


@dataclass(frozen=True)
class BtsnoopRecord:
    index: int
    flags: int
    drops: int
    timestamp_us: int
    packet: bytes

    @property
    def received(self) -> bool:
        return bool(self.flags & 1)

    @property
    def timestamp(self) -> str:
        unix_seconds = (self.timestamp_us - BTSNOOP_EPOCH_DELTA_US) / 1_000_000
        return datetime.fromtimestamp(unix_seconds, timezone.utc).isoformat()


@dataclass(frozen=True)
class AttPdu:
    record_index: int
    timestamp: str
    direction: str
    connection_handle: int
    opcode: int
    attribute_handle: int | None
    raw: bytes


def read_records(path: str | Path) -> list[BtsnoopRecord]:
    data = Path(path).read_bytes()
    if len(data) < BTSNOOP_HEADER.size:
        raise ValueError("btsnoop file is shorter than its header")
    magic, version, datalink = BTSNOOP_HEADER.unpack_from(data)
    if magic != b"btsnoop\0":
        raise ValueError("invalid btsnoop magic")
    if version != 1:
        raise ValueError(f"unsupported btsnoop version {version}")
    if datalink != 1002:
        raise ValueError(f"unsupported btsnoop datalink {datalink}; expected HCI UART (1002)")

    records = []
    offset = BTSNOOP_HEADER.size
    while offset < len(data):
        if offset + BTSNOOP_RECORD.size > len(data):
            raise ValueError("truncated btsnoop record header")
        original, included, flags, drops, timestamp = BTSNOOP_RECORD.unpack_from(data, offset)
        offset += BTSNOOP_RECORD.size
        if included > original or offset + included > len(data):
            raise ValueError("invalid or truncated btsnoop record payload")
        packet = data[offset : offset + included]
        offset += included
        records.append(BtsnoopRecord(len(records), flags, drops, timestamp, packet))
    return records


def _format_address(raw_little_endian: bytes) -> str:
    return ":".join(f"{octet:02X}" for octet in reversed(raw_little_endian))


def connection_handles(records: list[BtsnoopRecord], address: str) -> set[int]:
    target = address.upper()
    handles = set()
    for record in records:
        packet = record.packet
        if (
            len(packet) >= 15
            and packet[0] == 0x04
            and packet[1] == 0x3E
            and packet[3] in (0x01, 0x0A)
            and packet[4] == 0
            and _format_address(packet[9:15]) == target
        ):
            handles.add(int.from_bytes(packet[5:7], "little") & 0x0FFF)
    return handles


def iter_att_pdus(records: list[BtsnoopRecord], handles: set[int]) -> Iterator[AttPdu]:
    fragments: dict[tuple[int, str], tuple[int, int, bytearray, BtsnoopRecord]] = {}
    for record in records:
        packet = record.packet
        if len(packet) < 5 or packet[0] != 0x02:
            continue
        handle_flags = int.from_bytes(packet[1:3], "little")
        handle = handle_flags & 0x0FFF
        boundary = (handle_flags >> 12) & 0x03
        if handle not in handles:
            continue
        direction = "rx" if record.received else "tx"
        acl_length = int.from_bytes(packet[3:5], "little")
        acl = packet[5 : 5 + acl_length]
        key = (handle, direction)

        if boundary in (0, 2) and len(acl) >= 4:
            expected = int.from_bytes(acl[0:2], "little")
            cid = int.from_bytes(acl[2:4], "little")
            fragments[key] = (expected, cid, bytearray(acl[4:]), record)
        elif boundary == 1 and key in fragments:
            fragments[key][2].extend(acl)
        else:
            continue

        expected, cid, payload, first_record = fragments[key]
        if len(payload) < expected:
            continue
        del fragments[key]
        if cid != ATT_CID or not payload:
            continue
        raw = bytes(payload[:expected])
        opcode = raw[0]
        attribute_handle = (
            int.from_bytes(raw[1:3], "little")
            if opcode in (0x0A, 0x12, 0x1B, 0x1D, 0x52) and len(raw) >= 3
            else None
        )
        yield AttPdu(
            record_index=first_record.index,
            timestamp=first_record.timestamp,
            direction=direction,
            connection_handle=handle,
            opcode=opcode,
            attribute_handle=attribute_handle,
            raw=raw,
        )


def decode_neo2_session(path: str | Path, address: str) -> dict[str, object]:
    records = read_records(path)
    handles = connection_handles(records, address)
    att_rows = []
    duml_rows = []
    for pdu in iter_att_pdus(records, handles):
        row = asdict(pdu)
        row["raw_hex"] = row.pop("raw").hex()
        att_rows.append(row)
        if pdu.opcode not in (0x1B, 0x1D, 0x52) or len(pdu.raw) <= 3:
            continue
        for offset, packet, raw in find_packets(pdu.raw[3:]):
            duml_rows.append(
                {
                    "record_index": pdu.record_index,
                    "timestamp": pdu.timestamp,
                    "direction": pdu.direction,
                    "attribute_handle": pdu.attribute_handle,
                    "value_offset": offset,
                    "raw_hex": raw.hex(),
                    "sender": packet.sender,
                    "receiver": packet.receiver,
                    "sequence": packet.seq,
                    "command_type": packet.cmd_type,
                    "command_set": packet.cmd_set,
                    "command_id": packet.cmd_id,
                    "payload_hex": packet.payload.hex(),
                }
            )
    return {
        "source": str(path),
        "address": address.upper(),
        "record_count": len(records),
        "connection_handles": sorted(handles),
        "att": att_rows,
        "duml": duml_rows,
    }
