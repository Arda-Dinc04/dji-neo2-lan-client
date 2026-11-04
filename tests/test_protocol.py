from neo2_control.duml import neo2_heartbeat
from neo2_control.protocol import decode_datagram
from neo2_control.transport import ReliableTransport
from neo2_control.transport import xor_checksum


def test_type_5_payload_is_decoded_as_duml_with_absolute_offsets():
    transport = ReliableTransport(session=0x1234, base_sequence=0x4000)
    raw = transport.first_reliable_data(neo2_heartbeat(0x1234).encode())

    decoded = decode_datagram(raw)

    assert [layer["name"] for layer in decoded["layers"]] == [
        "dji-reliable-udp",
        "duml",
    ]
    packet = decoded["layers"][1]["packets"][0]
    assert packet["offset"] == 20
    assert packet["sender"] == 0x02
    assert packet["receiver"] == 0x28
    assert packet["command_type"] == 0
    assert packet["command_set"] == 0
    assert packet["command_id"] == 0
    assert packet["payload_hex"] == "00020100"
    assert packet["fields"][0]["offset_bits"] == 20 * 8


def test_type_1_payload_is_decoded_as_duml_after_control_state():
    heartbeat = neo2_heartbeat(0x8123).encode()
    raw = bytearray(34 + len(heartbeat))
    raw[:2] = (0x8000 | len(raw)).to_bytes(2, "little")
    raw[2:4] = (0x1234).to_bytes(2, "little")
    raw[6] = 1
    raw[24:26] = (0x4008).to_bytes(2, "little")
    raw[7] = xor_checksum(raw[:7])
    raw[34:] = heartbeat

    decoded = decode_datagram(bytes(raw))

    assert [layer["name"] for layer in decoded["layers"]] == [
        "dji-reliable-udp",
        "duml",
    ]
    transport = decoded["layers"][0]
    assert transport["payload_offset"] == 34
    assert transport["payload_length"] == len(heartbeat)
    packet = decoded["layers"][1]["packets"][0]
    assert packet["offset"] == 34
    assert packet["sequence"] == 0x8123
    assert packet["fields"][0]["offset_bits"] == 34 * 8
