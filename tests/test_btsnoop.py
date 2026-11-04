import struct
import tempfile
import unittest
from pathlib import Path

from neo2_control.btsnoop import BTSNOOP_EPOCH_DELTA_US, decode_neo2_session
from neo2_control.duml import DumlPacket


def _record(packet: bytes, *, received: bool, timestamp: int) -> bytes:
    flags = 1 if received else 0
    return struct.pack(">IIIIQ", len(packet), len(packet), flags, 0, timestamp) + packet


class BtsnoopTests(unittest.TestCase):
    def test_extracts_crc_valid_duml_from_att_write_command(self):
        address_le = bytes.fromhex("010000000002")
        connection = bytes((4, 0x3E, 19, 1, 0, 11, 0, 0, 0)) + address_le + bytes(8)
        duml = DumlPacket(2, 7, 1, 0x40, 7, 0x0C).encode()
        att = bytes.fromhex("521000") + duml
        l2cap = len(att).to_bytes(2, "little") + bytes.fromhex("0400") + att
        acl = bytes((2, 11, 0)) + len(l2cap).to_bytes(2, "little") + l2cap
        timestamp = BTSNOOP_EPOCH_DELTA_US + 1_000_000
        content = struct.pack(">8sII", b"btsnoop\0", 1, 1002)
        content += _record(connection, received=True, timestamp=timestamp)
        content += _record(acl, received=False, timestamp=timestamp + 1)
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / "capture.btsnoop"
            capture.write_bytes(content)
            decoded = decode_neo2_session(capture, "02:00:00:00:00:01")
        self.assertEqual(decoded["connection_handles"], [11])
        self.assertEqual(decoded["duml"][0]["command_id"], 0x0C)
        self.assertEqual(decoded["duml"][0]["attribute_handle"], 0x0010)


if __name__ == "__main__":
    unittest.main()
