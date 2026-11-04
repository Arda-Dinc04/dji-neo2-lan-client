"""The SWUDP 0x51/0x01 tunnel decoder recurses into the wrapped command."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))

from neo2_control.duml import DumlPacket, neo2_heartbeat  # noqa: E402
from tools.decode_swudp_tunnel import walk  # noqa: E402


def _tunnel(inner: DumlPacket) -> DumlPacket:
    return DumlPacket(
        sender=0x3B, receiver=0xE9, seq=0x8002,
        cmd_type=0x00, cmd_set=0x51, cmd_id=0x01,
        payload=inner.encode(),
    )


def test_walk_finds_concatenated_tunnels_and_recurses():
    inner = neo2_heartbeat(0x8001)
    blob = b"\x80\x00\x11\x22junk" + _tunnel(inner).encode() + _tunnel(inner).encode()

    outer = list(walk(blob))
    assert [(p.cmd_set, p.cmd_id) for p in outer] == [(0x51, 0x01), (0x51, 0x01)]

    nested = list(walk(outer[0].payload))
    assert len(nested) == 1
    assert (nested[0].cmd_set, nested[0].cmd_id) == (0x00, 0x00)
    assert nested[0].payload == inner.payload


def test_walk_rejects_bad_crc():
    # A bare heartbeat has no nested frame in its 4-byte body, so corrupting
    # its CRC-16 leaves nothing else for the byte scanner to lock onto.
    frame = bytearray(neo2_heartbeat(0x8001).encode())
    frame[-1] ^= 0xFF
    assert list(walk(bytes(frame))) == []
