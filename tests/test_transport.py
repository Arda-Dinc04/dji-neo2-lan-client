import unittest

from neo2_control.transport import (
    ReliableTransport,
    ReliableSendState,
    describe_transport,
    extract_stream_payload,
    is_session_accept,
    is_uplink_ack,
    xor_checksum,
)


class TransportTests(unittest.TestCase):
    def test_known_syn(self):
        transport = ReliableTransport(session=0xD5F8, base_sequence=0xAFC0)
        self.assertEqual(
            transport.open_session().hex(),
            "3080f8d50000009dc0af64006400c005140000640000019001c005140000640014006400c00514000064000101040102",
        )

    def test_periodic_idle_window_has_no_application_payload(self):
        transport = ReliableTransport(session=0x1234, base_sequence=0x4000)
        frame = transport.idle_window()
        decoded, _ = describe_transport(frame)
        self.assertEqual(len(frame), 30)
        self.assertEqual(decoded["type"], 6)
        self.assertEqual(decoded["video_window_start"], 0x4000)
        self.assertEqual(decoded["protocol_window_start"], 0x4000)
        self.assertEqual(decoded["send_window_start"], 0x4000)
        self.assertEqual(decoded["video_ack_count"], 0)
        self.assertEqual(decoded["protocol_ack_count"], 0)
        self.assertEqual(decoded["qos_aggregate_length"], 0)
        self.assertTrue(decoded["header_checksum_valid"])

    def test_normal_idle_window_uses_compressed_ack_in_mode_one(self):
        transport = ReliableTransport(session=0x1234, base_sequence=0x4000)
        frame = transport.idle_window(periodic=False)
        decoded, _ = describe_transport(frame)
        self.assertEqual(len(frame), 34)
        self.assertEqual(decoded["type"], 4)
        self.assertEqual(decoded["video_ack_slot_count"], 0)
        self.assertEqual(decoded["protocol_ack_slot_count"], 0)
        self.assertEqual(decoded["qos_aggregate_length"], 0)

    def test_first_reliable_data_matches_native_type_5_layout(self):
        transport = ReliableTransport(session=0x1234, base_sequence=0x4000)
        duml = bytes.fromhex("551104920228341200000000020100ae8b")
        frame = transport.first_reliable_data(duml)
        decoded, _ = describe_transport(frame)

        self.assertEqual(
            frame.hex(),
            "25803412084005ce004008400000000001010000"
            "551104920228341200000000020100ae8b",
        )
        self.assertEqual(decoded["type"], 5)
        self.assertEqual(decoded["transport_direction"], "ground_to_drone")
        self.assertEqual(decoded["send_sequence"], 0x4008)
        self.assertEqual(decoded["window_start"], 0x4000)
        self.assertEqual(decoded["window_end"], 0x4008)
        self.assertEqual(decoded["stream_slot"], 1)
        self.assertEqual(decoded["group_span_raw"], 1)
        self.assertEqual(decoded["payload_offset"], 20)
        self.assertEqual(decoded["payload_length"], 17)

    def test_first_reliable_data_rejects_empty_or_fragmented_input(self):
        transport = ReliableTransport(session=0x1234, base_sequence=0x4000)
        with self.assertRaises(ValueError):
            transport.first_reliable_data(b"")
        with self.assertRaises(ValueError):
            transport.first_reliable_data(b"x" * 1453)

    def test_second_reliable_data_advances_only_after_ack(self):
        transport = ReliableTransport(session=0x1234, base_sequence=0x4000)
        state = ReliableSendState(transport)
        first = state.send(b"first")
        with self.assertRaisesRegex(RuntimeError, "blocked"):
            state.send(b"second")

        wrong_ack = bytearray(34)
        wrong_ack[:2] = (0x8000 | len(wrong_ack)).to_bytes(2, "little")
        wrong_ack[2:4] = (0x1234).to_bytes(2, "little")
        wrong_ack[6] = 1
        wrong_ack[24:26] = (0x4010).to_bytes(2, "little")
        wrong_ack[7] = xor_checksum(wrong_ack[:7])
        self.assertFalse(state.acknowledge(bytes(wrong_ack)))

        ack = bytearray(wrong_ack)
        ack[24:26] = (0x4008).to_bytes(2, "little")
        self.assertTrue(state.acknowledge(bytes(ack)))
        second = state.send(b"second")

        first_decoded, _ = describe_transport(first)
        second_decoded, _ = describe_transport(second)
        self.assertEqual(first_decoded["window_start"], 0x4000)
        self.assertEqual(first_decoded["window_end"], 0x4008)
        self.assertEqual(first_decoded["send_sequence"], 0x4008)
        self.assertEqual(first_decoded["stream_slot"], 1)
        self.assertEqual(second_decoded["window_start"], 0x4008)
        self.assertEqual(second_decoded["window_end"], 0x4010)
        self.assertEqual(second_decoded["send_sequence"], 0x4010)
        self.assertEqual(second_decoded["stream_slot"], 2)

    def test_third_reliable_data_advances_to_the_next_slot(self):
        transport = ReliableTransport(session=0x1234, base_sequence=0x4000)
        state = ReliableSendState(transport)

        def acknowledge(sequence: int) -> bytes:
            ack = bytearray(34)
            ack[:2] = (0x8000 | len(ack)).to_bytes(2, "little")
            ack[2:4] = (0x1234).to_bytes(2, "little")
            ack[6] = 1
            ack[24:26] = sequence.to_bytes(2, "little")
            ack[7] = xor_checksum(ack[:7])
            return bytes(ack)

        state.send(b"first")
        self.assertTrue(state.acknowledge(acknowledge(0x4008)))
        state.send(b"second")
        self.assertTrue(state.acknowledge(acknowledge(0x4010)))
        third = state.send(b"third")

        decoded, _ = describe_transport(third)
        self.assertEqual(decoded["window_start"], 0x4010)
        self.assertEqual(decoded["window_end"], 0x4018)
        self.assertEqual(decoded["send_sequence"], 0x4018)
        self.assertEqual(decoded["stream_slot"], 3)
        self.assertEqual(decoded["stream_word"], 0x00000103)
        self.assertEqual(third[16:20].hex(), "03010000")
        with self.assertRaisesRegex(RuntimeError, "blocked"):
            state.send(b"fourth")

    def test_exact_session_accept_is_recognized(self):
        raw = bytearray.fromhex("0f803412000000a90101000104401f")
        raw[7] = 0
        for byte in raw[:7]:
            raw[7] ^= byte
        self.assertTrue(is_session_accept(bytes(raw), 0x1234))
        self.assertFalse(is_session_accept(bytes(raw), 0x1235))
        decoded, _ = describe_transport(bytes(raw))
        self.assertEqual(decoded["max_bitrate_kbps"], 8000)

    def test_type_1_uplink_ack_is_recognized(self):
        raw = bytearray(34)
        raw[:2] = (0x8000 | len(raw)).to_bytes(2, "little")
        raw[2:4] = (0x1234).to_bytes(2, "little")
        raw[6] = 1
        raw[24:26] = (0x4008).to_bytes(2, "little")
        raw[7] = xor_checksum(raw[:7])
        self.assertTrue(is_uplink_ack(bytes(raw), 0x1234, 0x4008))
        self.assertFalse(is_uplink_ack(bytes(raw), 0x1234, 0x4009))
        self.assertFalse(is_uplink_ack(bytes(raw), 0x1235, 0x4008))

    def test_uplink_ack_rejects_truncated_or_shifted_status(self):
        raw = bytearray(34)
        raw[:2] = (0x8000 | len(raw)).to_bytes(2, "little")
        raw[2:4] = (0x1234).to_bytes(2, "little")
        raw[6] = 1
        raw[24:26] = (0x4008).to_bytes(2, "little")
        raw[7] = xor_checksum(raw[:7])
        short = raw[:26]
        short[:2] = (0x8000 | len(short)).to_bytes(2, "little")
        short[7] = xor_checksum(short[:7])
        self.assertFalse(is_uplink_ack(bytes(short), 0x1234, 0x4008))
        # Fixed video-window flags do not shift the separate uplink ACK.
        raw[12:14] = (1).to_bytes(2, "little")
        self.assertTrue(is_uplink_ack(bytes(raw), 0x1234, 0x4008))
        # The actual variable ACK bitmap starts at offset 28.
        raw[28:30] = (1).to_bytes(2, "little")
        self.assertFalse(is_uplink_ack(bytes(raw), 0x1234, 0x4008))
        raw[28:30] = b"\x00\x00"
        raw[32:34] = (1).to_bytes(2, "little")
        self.assertFalse(is_uplink_ack(bytes(raw), 0x1234, 0x4008))

    def test_management_reply_without_acceptance_mode_does_not_open_link(self):
        raw = bytearray.fromhex("0f803412000000000001000104401f")
        raw[7] = xor_checksum(raw[:7])
        self.assertFalse(is_session_accept(bytes(raw), 0x1234))

    def test_type_1_state_uses_the_same_three_window_blocks_as_type_6(self):
        duml = bytes.fromhex("551104920228341200000000020100ae8b")
        raw = bytearray(34 + len(duml))
        raw[:2] = (0x8000 | len(raw)).to_bytes(2, "little")
        raw[2:4] = (0x1234).to_bytes(2, "little")
        raw[6] = 1
        # Blocks 1 and 2 stay at the link anchor in every observed session.
        raw[8:10] = (0x4000).to_bytes(2, "little")
        raw[10:12] = (0x4000).to_bytes(2, "little")
        raw[16:18] = (0x4000).to_bytes(2, "little")
        raw[18:20] = (0x4000).to_bytes(2, "little")
        # Block 3 collapses on the acknowledged uplink sequence.
        raw[24:26] = (0x4010).to_bytes(2, "little")
        raw[26:28] = (0x4010).to_bytes(2, "little")
        raw[32:34] = len(duml).to_bytes(2, "little")
        raw[7] = xor_checksum(raw[:7])
        raw[34:] = duml

        decoded, _ = describe_transport(bytes(raw))
        self.assertEqual(decoded["type"], 1)
        self.assertEqual(decoded["transport_direction"], "drone_to_ground")
        self.assertEqual(decoded["drone_video_window_start"], 0x4000)
        self.assertEqual(decoded["drone_video_window_end"], 0x4000)
        self.assertEqual(decoded["drone_protocol_window_start"], 0x4000)
        self.assertEqual(decoded["drone_protocol_window_end"], 0x4000)
        self.assertEqual(decoded["drone_send_window_start"], 0x4010)
        self.assertEqual(decoded["drone_send_window_end"], 0x4010)
        self.assertEqual(decoded["acknowledged_uplink"], 0x4010)
        self.assertEqual(decoded["downlink_aggregate_length"], len(duml))
        self.assertEqual(decoded["payload_offset"], 34)
        self.assertEqual(decoded["payload_length"], len(duml))
        self.assertTrue(is_uplink_ack(bytes(raw), 0x1234, 0x4010))

    def test_short_transport_is_rejected(self):
        with self.assertRaises(ValueError):
            describe_transport(b"\x00" * 7)

    def test_transport_length_uses_fourteen_bits(self):
        raw = bytearray(1472)
        raw[:2] = (0x8000 | len(raw)).to_bytes(2, "little")
        raw[6] = 2
        raw[7] = 0
        for byte in raw[:7]:
            raw[7] ^= byte
        decoded, fields = describe_transport(bytes(raw))
        self.assertEqual(decoded["declared_length"], 1472)
        self.assertEqual(decoded["flags"], 2)
        self.assertEqual(fields[0].width_bits, 14)

    def test_video_stream_header_and_payload(self):
        payload = bytes((index % 251 for index in range(1452)))
        stream_word = (
            7
            | (4 << 8)
            | (1 << 14)
            | (3 << 15)
            | (len(payload) << 21)
        )
        raw = bytearray(20)
        raw[:2] = (0x8000 | (20 + len(payload))).to_bytes(2, "little")
        raw[2:4] = (0x1234).to_bytes(2, "little")
        raw[4:6] = (0x5678).to_bytes(2, "little")
        raw[6] = 2
        raw[8:10] = (0x4000).to_bytes(2, "little")
        raw[10:12] = (0x4020).to_bytes(2, "little")
        raw[12:14] = (1).to_bytes(2, "little")
        raw[14:16] = (0).to_bytes(2, "little")
        raw[16:20] = stream_word.to_bytes(4, "little")
        raw[7] = 0
        for byte in raw[:7]:
            raw[7] ^= byte
        raw += payload

        decoded, _ = describe_transport(bytes(raw))
        self.assertEqual(decoded["type"], 2)
        self.assertEqual(decoded["payload_length"], 1452)
        self.assertEqual(decoded["stream_slot"], 7)
        self.assertEqual(decoded["group_span_raw"], 4)
        self.assertTrue(decoded["tail_length_present"])
        self.assertEqual(decoded["fragment_index"], 3)
        self.assertEqual(decoded["tail_payload_length"], 1452)
        self.assertEqual(extract_stream_payload(bytes(raw)), payload)

    def test_non_stream_payload_is_rejected(self):
        transport = ReliableTransport(session=0x1234, base_sequence=0x4000)
        with self.assertRaises(ValueError):
            extract_stream_payload(transport.open_session())


if __name__ == "__main__":
    unittest.main()
