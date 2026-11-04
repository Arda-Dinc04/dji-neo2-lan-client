import unittest

from neo2_control.duml import (
    DumlSequence,
    DumlPacket,
    crc8,
    crc16,
    decode_host_id,
    encode_host_id,
    find_packets,
    neo2_heartbeat,
    neo2_static_capability,
    neo2_xrce_agent,
    neo2_wifi_device_info,
)


class DumlTests(unittest.TestCase):
    def test_known_packet_encoding(self):
        packet = DumlPacket(
            sender=0x2A,
            receiver=0x28,
            seq=0x1234,
            cmd_type=0x40,
            cmd_set=0x04,
            cmd_id=0x09,
            payload=bytes.fromhex("deadbeef"),
        )
        self.assertEqual(packet.encode().hex(), "551104922a283412400409deadbeef35e6")
        self.assertEqual(DumlPacket.decode(packet.encode()), packet)

    def test_corruption_is_rejected(self):
        raw = bytearray.fromhex("551104922a283412400409deadbeef35e6")
        raw[7] ^= 0xFF
        with self.assertRaisesRegex(ValueError, "CRC-16"):
            DumlPacket.decode(bytes(raw))

    def test_find_packets_preserves_offsets(self):
        raw = bytes.fromhex("551104922a283412400409deadbeef35e6")
        found = list(find_packets(b"\x00\xff" + raw + b"\x01"))
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0][0], 2)
        self.assertEqual(found[0][2], raw)

    def test_crc_primitives_are_stable(self):
        self.assertEqual(crc8(bytes.fromhex("551104")), 0x92)
        self.assertEqual(crc16(bytes.fromhex("551104922a283412400409deadbeef")), 0xE635)

    def test_host_id_bit_packing(self):
        self.assertEqual(encode_host_id(0x02, 0), 0x02)
        self.assertEqual(encode_host_id(0x08, 1), 0x28)
        self.assertEqual(decode_host_id(0x28), (0x08, 1))
        with self.assertRaises(ValueError):
            encode_host_id(0x20, 0)

    def test_exact_neo2_v1_heartbeat(self):
        packet = neo2_heartbeat(0x1234)
        self.assertEqual(packet.sender, 0x02)
        self.assertEqual(packet.receiver, 0x28)
        self.assertEqual(packet.cmd_type, 0x00)
        self.assertEqual(packet.payload, bytes.fromhex("00020100"))
        self.assertEqual(
            packet.encode().hex(),
            "551104920228341200000000020100ae8b",
        )

    def test_neo2_sequence_constructor_matches_native_reduction(self):
        self.assertEqual(DumlSequence.initial_from_random_word(0), 0xBC90)
        self.assertEqual(DumlSequence.initial_from_random_word(1), 0xBC90)
        self.assertEqual(DumlSequence.initial_from_random_word(2), 0xF920)
        self.assertEqual(
            DumlSequence.initial_from_random_word(0x7FFFFFFE),
            0xC370,
        )
        self.assertEqual(
            DumlSequence.initial_from_random_word(0xFFFFFFFF),
            0xBC90,
        )

    def test_neo2_sequence_increments_and_wraps_before_ffff(self):
        sequence = DumlSequence(0x8000)
        self.assertEqual(sequence.next(), 0x8001)
        sequence.current = 0xFFFE
        self.assertEqual(sequence.next(), 0x8000)
        self.assertEqual(sequence.next(), 0x8001)
        with self.assertRaises(ValueError):
            DumlSequence(0xFFFF)

    def test_exact_neo2_wifi_device_info(self):
        packet = neo2_wifi_device_info(
            0x8123,
            physical_port=9003,
            device_id="01234567-89ab-cdef-01234",
        )
        self.assertEqual(packet.sender, 0x02)
        self.assertEqual(packet.receiver, 0x1B)
        self.assertEqual(packet.cmd_type, 0x40)
        self.assertEqual((packet.cmd_set, packet.cmd_id), (0x07, 0x93))
        self.assertEqual(
            packet.payload.hex(),
            "012b230400000030313233343536372d383961622d636465662d3031323334",
        )
        self.assertEqual(
            packet.encode().hex(),
            "552c0436021b2381400793012b230400000030313233343536372d383961622d636465662d3031323334bd83",
        )

    def test_wifi_device_info_rejects_implicit_truncation(self):
        with self.assertRaises(ValueError):
            neo2_wifi_device_info(
                0x8123,
                physical_port=9003,
                device_id="x" * 25,
            )

    def test_exact_neo2_static_capability_requests(self):
        product = neo2_static_capability(0x1234, (4,))
        self.assertEqual(product.sender, 0x02)
        self.assertEqual(product.receiver, 0x28)
        self.assertEqual(product.cmd_type, 0x40)
        self.assertEqual((product.cmd_set, product.cmd_id), (0x00, 0xB7))
        self.assertEqual(product.payload.hex(), "0101000400")
        self.assertEqual(
            product.encode().hex(),
            "551204c7022834124000b701010004007e16",
        )
        wifi = neo2_static_capability(0x1234, (14,))
        self.assertEqual(wifi.payload.hex(), "0101000e00")
        self.assertEqual(
            wifi.encode().hex(),
            "551204c7022834124000b70101000e000eeb",
        )

    def test_static_capability_body_length_is_three_plus_two_per_id(self):
        for count in (1, 2, 21):
            packet = neo2_static_capability(0x8000, range(count))
            self.assertEqual(len(packet.payload), 3 + 2 * count)
            self.assertEqual(packet.payload[0], 1)
            self.assertEqual(
                int.from_bytes(packet.payload[1:3], "little"),
                count,
            )
            self.assertEqual(len(packet.encode()), 13 + 3 + 2 * count)

    def test_static_capability_rejects_batches_the_application_never_emits(self):
        with self.assertRaisesRegex(ValueError, "between 1 and 21"):
            neo2_static_capability(0x8000, ())
        with self.assertRaisesRegex(ValueError, "between 1 and 21"):
            neo2_static_capability(0x8000, range(22))
        with self.assertRaisesRegex(ValueError, "16 bits"):
            neo2_static_capability(0x8000, (0x10000,))

    def test_exact_neo2_xrce_agent_envelope(self):
        inner = bytes.fromhex("02 07")
        packet = neo2_xrce_agent(0x8123, inner)

        self.assertEqual(packet.sender, 0x02)
        self.assertEqual(packet.receiver, 0x28)
        self.assertEqual(packet.cmd_type, 0x40)
        self.assertEqual((packet.cmd_set, packet.cmd_id), (0x00, 0x99))
        self.assertEqual(packet.payload, inner)
        self.assertEqual(packet.encode().hex(), "550f04a2022823814000990207a15d")

    def test_xrce_agent_rejects_invalid_payloads(self):
        with self.assertRaisesRegex(ValueError, "must not be empty"):
            neo2_xrce_agent(0x8000, b"")
        with self.assertRaisesRegex(ValueError, "length field"):
            neo2_xrce_agent(0x8000, b"x" * 1011)


if __name__ == "__main__":
    unittest.main()
