"""Layer-aware decoder used by capture and graphical reports."""

from __future__ import annotations

from typing import Any

from .duml import BitField, find_packets
from .transport import describe_transport


def decode_datagram(raw: bytes) -> dict[str, Any]:
    decoded: dict[str, Any] = {"length": len(raw), "layers": [], "fields": []}
    payload = raw
    payload_offset = 0

    if raw and raw[0] != 0x55 and len(raw) >= 8:
        try:
            transport, fields = describe_transport(raw)
        except ValueError as error:
            decoded["errors"] = [str(error)]
        else:
            decoded["layers"].append({"name": "dji-reliable-udp", **transport})
            decoded["fields"].extend(field.as_dict() for field in fields)
            if (
                transport["type"] in (1, 5)
                and transport["declared_length"] <= len(raw)
                and "payload_offset" in transport
            ):
                payload_offset = int(transport["payload_offset"])
                payload = raw[payload_offset : int(transport["declared_length"])]
            else:
                payload = b""

    packets = []
    for offset, packet, packet_raw in find_packets(payload):
        fields = []
        for field in packet.fields():
            shifted = BitField(
                field.name,
                field.offset_bits + (payload_offset + offset) * 8,
                field.width_bits,
                field.value,
                field.description,
            )
            fields.append(shifted.as_dict())
        packets.append(
            {
                "offset": payload_offset + offset,
                "length": len(packet_raw),
                "sender": packet.sender,
                "receiver": packet.receiver,
                "sequence": packet.seq,
                "command_type": packet.cmd_type,
                "command_set": packet.cmd_set,
                "command_id": packet.cmd_id,
                "payload_hex": packet.payload.hex(),
                "fields": fields,
            }
        )
        decoded["fields"].extend(fields)
    if packets:
        decoded["layers"].append({"name": "duml", "packets": packets})
    return decoded
