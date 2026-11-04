from __future__ import annotations

import unittest

from neo2_control.xrce import (
    DdsSequence,
    DdsVersionAnnouncement,
    ObserverRegistryReset,
    TopicDataPush,
    UnknownAgentPush,
    encapsulate_subscription,
    parse_agent_push,
    serialize_agent_request,
    serialize_subscription,
)
from neo2_control.protocol import decode_datagram
from neo2_control.transport import ReliableSendState, ReliableTransport


class XrcePayloadTests(unittest.TestCase):
    def test_video_codec_subscription_vector(self) -> None:
        payload = serialize_subscription(
            "camcap_video_codec",
            sequence=0x1234,
            host_type=7,
            host_index=4,
        )

        self.assertEqual(len(payload), 37)
        self.assertEqual(
            payload.hex(),
            "020200003412000000000018001200"
            "63616d6361705f766964656f5f636f646563"
            "04070000",
        )

    def test_layout_accepts_raw_topic_bytes(self) -> None:
        payload = serialize_agent_request(
            b"x\x00y",
            dds_version=0x22,
            command=6,
            topic_base_id=0xABCD,
            sequence=1,
            host_type=2,
            host_index=3,
        )
        self.assertEqual(payload[0:6], bytes.fromhex("22 06 cd ab 01 00"))
        self.assertEqual(payload[6:11], b"\x00" * 5)
        self.assertEqual(payload[11:15], bytes.fromhex("09 00 03 00"))
        self.assertEqual(payload[15:], b"x\x00y\x03\x02\x00\x00")

    def test_rejects_out_of_range_fields(self) -> None:
        with self.assertRaisesRegex(ValueError, "sequence"):
            serialize_subscription("topic", sequence=0x10000)
        with self.assertRaisesRegex(ValueError, "too long"):
            serialize_subscription(b"x" * 65_530, sequence=1)

    def test_native_dds_sequence_initialization_vectors(self) -> None:
        expected = {
            0: 0xBC90,
            1: 0xBC90,
            2: 0x7921,
            0x7FFFFFFE: 0x4371,
            0xFFFFFFFF: 0xBC90,
            0x12345678: 0xBD52,
        }
        for random_word, base in expected.items():
            with self.subTest(random_word=random_word):
                self.assertEqual(DdsSequence.from_random_word(random_word).value, base)

    def test_native_dds_sequence_increments_then_wraps(self) -> None:
        sequence = DdsSequence(0xFFFE)
        self.assertEqual(sequence.next(), 1)
        self.assertEqual(sequence.next(), 2)

    def test_parse_topic_data_push(self) -> None:
        topic = b"camcap_video_codec"
        data = bytes.fromhex("01 23 45 67")
        remaining = len(topic) + 10 + len(data)
        payload = (
            bytes.fromhex("02 06 34 12 78 56 00 00 00 00 00")
            + remaining.to_bytes(2, "little")
            + len(topic).to_bytes(2, "little")
            + topic
            + bytes.fromhex("04 07 00 00 00 00")
            + len(data).to_bytes(2, "little")
            + data
        )

        parsed = parse_agent_push(payload)

        self.assertIsInstance(parsed, TopicDataPush)
        assert isinstance(parsed, TopicDataPush)
        self.assertEqual(parsed.header.dds_version, 2)
        self.assertEqual(parsed.topic_base_id, 0x1234)
        self.assertEqual(parsed.sequence, 0x5678)
        self.assertEqual(parsed.topic_base, topic)
        self.assertEqual((parsed.host_type, parsed.host_index), (7, 4))
        self.assertEqual(parsed.data, data)

    def test_parse_control_pushes_and_preserve_unknown(self) -> None:
        reset = parse_agent_push(bytes.fromhex("02 03"))
        version = parse_agent_push(bytes.fromhex("05 07"))
        unknown = parse_agent_push(bytes.fromhex("02 99 aa"))

        self.assertIsInstance(reset, ObserverRegistryReset)
        self.assertIsInstance(version, DdsVersionAnnouncement)
        assert isinstance(version, DdsVersionAnnouncement)
        self.assertEqual(version.header.dds_version, 5)
        self.assertIsInstance(unknown, UnknownAgentPush)
        assert isinstance(unknown, UnknownAgentPush)
        self.assertEqual(unknown.payload, bytes.fromhex("02 99 aa"))

    def test_rejects_malformed_topic_data_lengths(self) -> None:
        with self.assertRaisesRegex(ValueError, "fixed topic header"):
            parse_agent_push(bytes.fromhex("02 06"))

        payload = bytearray(23)
        payload[0:2] = bytes.fromhex("02 06")
        payload[11:13] = (11).to_bytes(2, "little")
        payload[13:15] = (1).to_bytes(2, "little")
        with self.assertRaisesRegex(ValueError, "declared length"):
            parse_agent_push(bytes(payload))

    def test_complete_subscription_stack_vector(self) -> None:
        state = ReliableSendState(
            ReliableTransport(session=0x2468, base_sequence=0x4000)
        )

        envelope = encapsulate_subscription(
            state,
            "camcap_video_codec",
            dds_sequence=0x1234,
            duml_sequence=0x8123,
        )

        self.assertEqual(
            envelope.agent_payload.hex(),
            "020200003412000000000018001200"
            "63616d6361705f766964656f5f636f646563"
            "00000000",
        )
        self.assertEqual(
            envelope.duml_packet.hex(),
            "5532040602282381400099020200003412000000000018001200"
            "63616d6361705f766964656f5f636f64656300000000954e",
        )
        self.assertEqual(
            envelope.swudp_datagram.hex(),
            "46806824084005c7004008400000000001010000"
            "5532040602282381400099020200003412000000000018001200"
            "63616d6361705f766964656f5f636f64656300000000954e",
        )
        decoded = decode_datagram(envelope.swudp_datagram)
        self.assertEqual(decoded["layers"][0]["type"], 5)
        self.assertEqual(state.waiting_for_ack, 0x4008)
        self.assertEqual(state.next_stream_slot, 2)
        packet = decoded["layers"][1]["packets"][0]
        self.assertEqual(packet["offset"], 20)
        self.assertEqual(packet["command_type"], 0x40)
        self.assertEqual((packet["command_set"], packet["command_id"]), (0, 0x99))
        self.assertEqual(packet["payload_hex"], envelope.agent_payload.hex())


if __name__ == "__main__":
    unittest.main()
