from neo2_emulator import Neo2Emulator
import json
import os
import pytest

from neo2_control.duml import DumlPacket
from neo2_control.transport import describe_transport
from neo2_control.udp_probe import run_probe
from neo2_control.identity import load_or_create_identity


def _next_sequence(value: int) -> int:
    """Mirror ``DumlSequence.next`` so the assertions survive a wrap."""

    return 0x8000 if value + 1 >= 0xFFFF else value + 1


def test_bounded_heartbeat_is_sent_once_after_session_acceptance(tmp_path):
    emulator = Neo2Emulator()
    thread = emulator.run_in_background(None)
    try:
        result = run_probe(
            capture_path=tmp_path / "heartbeat.jsonl",
            host="127.0.0.1",
            port=emulator.port,
            duration=1.2,
            send_syn=True,
            send_heartbeat=True,
        )
    finally:
        thread.join(timeout=2)
        emulator.close()

    assert not thread.is_alive()
    assert len(emulator.received) == 2
    assert result.transmitted == 2
    assert result.session_ack_seen
    assert result.heartbeat_sent
    assert result.heartbeat_transport_ack_seen
    assert result.application_sent_after_accept_ms >= 797
    assert 0x8000 <= result.heartbeat_sequence <= 0xFFFE

    transport, _ = describe_transport(emulator.received[1])
    assert transport["type"] == 5
    heartbeat = DumlPacket.decode(emulator.received[1][20:])
    assert heartbeat.seq == result.heartbeat_sequence
    assert (heartbeat.cmd_set, heartbeat.cmd_id) == (0, 0)
    assert heartbeat.payload == bytes.fromhex("00020100")


def test_bounded_wifi_device_info_tracks_transport_and_duml_response(tmp_path):
    emulator = Neo2Emulator()
    thread = emulator.run_in_background(b"\x00")
    try:
        result = run_probe(
            capture_path=tmp_path / "wifi-device-info.jsonl",
            host="127.0.0.1",
            port=emulator.port,
            duration=0.2,
            send_syn=True,
            send_wifi_device_info=True,
        )
    finally:
        thread.join(timeout=2)
        emulator.close()

    assert not thread.is_alive()
    assert len(emulator.received) == 2
    assert result.transmitted == 2
    assert result.session_ack_seen
    assert result.wifi_device_info_sent
    assert result.wifi_device_info_transport_ack_seen
    assert result.wifi_device_info_response_seen
    assert result.application_sent_after_accept_ms >= 19

    request = DumlPacket.decode(emulator.received[1][20:])
    assert request.seq == result.wifi_device_info_sequence
    assert request.receiver == 0x1B
    assert request.cmd_type == 0x40
    assert (request.cmd_set, request.cmd_id) == (0x07, 0x93)
    assert int.from_bytes(request.payload[1:3], "little") == emulator.port
    assert len(request.payload[7:].rstrip(b"\x00")) == 19
    assert request.payload[26:] == bytes(5)
    assert result.wifi_device_info_receiver == 0x1B
    assert result.wifi_device_info_identity['device_sn_length'] == 19


def test_corrected_probe_reuses_private_identity_across_independent_sessions(tmp_path):
    store = tmp_path / 'pc-identity.json'
    identity = load_or_create_identity(store)
    payloads = []
    for index in range(2):
        emulator = Neo2Emulator()
        thread = emulator.run_in_background(b'\x00')
        try:
            result = run_probe(capture_path=tmp_path / f'probe-{index}.jsonl',
                               host='127.0.0.1', port=emulator.port, duration=0.2,
                               send_syn=True, send_wifi_device_info=True, identity_file=store)
        finally:
            thread.join(timeout=2)
            emulator.close()
        packet = DumlPacket.decode(emulator.received[1][20:])
        assert packet.receiver == 0x1B
        payloads.append(packet.payload[7:])
        assert packet.payload[7:26] == identity.device_sn.encode('ascii')
        assert packet.payload[26:] == bytes(5)
        assert result.wifi_device_info_transport_ack_seen
        assert result.wifi_device_info_identity == identity.metadata()
        assert identity.identification_id not in json.dumps(result.as_dict())
    assert payloads[0] == payloads[1]
    assert load_or_create_identity(store) == identity


def test_inapplicable_identity_or_overlapping_capture_fails_before_mutation(tmp_path):
    store = tmp_path / 'identity.json'
    with pytest.raises(ValueError, match='requires a Wi-Fi'):
        run_probe(capture_path=tmp_path / 'unused.jsonl', identity_file=store)
    assert not store.exists()
    with pytest.raises(ValueError, match='distinct'):
        run_probe(capture_path=store, identity_file=store, send_syn=True, send_wifi_device_info=True)
    assert not store.exists()
    identity = load_or_create_identity(store)
    alias = tmp_path / 'alias.jsonl'
    os.link(store, alias)
    original = store.read_bytes()
    with pytest.raises(ValueError, match='distinct'):
        run_probe(capture_path=alias, identity_file=store, send_syn=True, send_wifi_device_info=True)
    assert store.read_bytes() == original
    assert load_or_create_identity(store) == identity


def test_stateful_wifi_then_heartbeat_advances_after_ack(tmp_path):
    emulator = Neo2Emulator()
    thread = emulator.run_in_background(None, None)
    try:
        result = run_probe(
            capture_path=tmp_path / "stateful-wifi-heartbeat.jsonl",
            host="127.0.0.1",
            port=emulator.port,
            duration=1.1,
            send_syn=True,
            send_wifi_then_heartbeat=True,
        )
    finally:
        thread.join(timeout=2)
        emulator.close()

    assert not thread.is_alive()
    assert len(emulator.received) == 3
    assert result.transmitted == 3
    assert result.session_ack_seen
    assert result.wifi_device_info_transport_ack_seen
    assert result.heartbeat_transport_ack_seen
    assert result.stateful_sequence_requested
    assert result.stateful_sequence_completed
    assert result.wifi_device_info_sent_after_accept_ms >= 19
    assert result.heartbeat_sent_after_accept_ms >= 797

    first, _ = describe_transport(emulator.received[1])
    second, _ = describe_transport(emulator.received[2])
    base = result.base_sequence
    assert first["window_start"] == base
    assert first["window_end"] == (base + 8) & 0xFFFF
    assert first["send_sequence"] == (base + 8) & 0xFFFF
    assert first["stream_slot"] == 1
    assert second["window_start"] == (base + 8) & 0xFFFF
    assert second["window_end"] == (base + 16) & 0xFFFF
    assert second["send_sequence"] == (base + 16) & 0xFFFF
    assert second["stream_slot"] == 2

    wifi_request = DumlPacket.decode(emulator.received[1][20:])
    heartbeat = DumlPacket.decode(emulator.received[2][20:])
    assert (wifi_request.cmd_set, wifi_request.cmd_id) == (0x07, 0x93)
    assert (heartbeat.cmd_set, heartbeat.cmd_id) == (0, 0)
    # One DUML allocator per session, exactly like the native encoder.
    assert heartbeat.seq == _next_sequence(wifi_request.seq)


def test_static_capability_extends_the_stateful_sequence_after_heartbeat_ack(tmp_path):
    emulator = Neo2Emulator()
    thread = emulator.run_in_background(None, None, b"\x00")
    try:
        result = run_probe(
            capture_path=tmp_path / "stateful-static-capability.jsonl",
            host="127.0.0.1",
            port=emulator.port,
            duration=1.3,
            send_syn=True,
            send_wifi_then_heartbeat=True,
            send_static_capability=True,
        )
    finally:
        thread.join(timeout=2)
        emulator.close()

    assert not thread.is_alive()
    assert len(emulator.received) == 4
    assert result.transmitted == 4
    assert result.heartbeat_transport_ack_seen
    assert result.static_capability_sent
    assert result.static_capability_ids == [4]
    assert result.static_capability_transport_ack_seen
    assert result.static_capability_response_seen
    assert result.stateful_sequence_completed
    assert result.static_capability_sent_after_accept_ms >= 810
    # The ACK-gated packet order below is authoritative.  A fast local
    # emulator can complete both sends within one monotonic-clock tick.
    assert (
        result.static_capability_sent_after_accept_ms
        >= result.heartbeat_sent_after_accept_ms
    )

    third, _ = describe_transport(emulator.received[3])
    base = result.base_sequence
    assert third["type"] == 5
    assert third["window_start"] == (base + 16) & 0xFFFF
    assert third["window_end"] == (base + 24) & 0xFFFF
    assert third["send_sequence"] == (base + 24) & 0xFFFF
    assert third["stream_slot"] == 3
    assert third["stream_word"] == 0x00000103

    heartbeat = DumlPacket.decode(emulator.received[2][20:])
    capability = DumlPacket.decode(emulator.received[3][20:])
    assert capability.sender == 0x02
    assert capability.receiver == 0x28
    assert capability.cmd_type == 0x40
    assert (capability.cmd_set, capability.cmd_id) == (0x00, 0xB7)
    assert capability.payload == bytes.fromhex("0101000400")
    assert capability.seq == result.static_capability_sequence
    assert capability.seq == _next_sequence(heartbeat.seq)


def test_drone_stream_windows_are_reported_as_anchored(tmp_path):
    emulator = Neo2Emulator()
    thread = emulator.run_in_background(None, None)
    try:
        result = run_probe(
            capture_path=tmp_path / "anchored-windows.jsonl",
            host="127.0.0.1",
            port=emulator.port,
            duration=1.1,
            send_syn=True,
            send_wifi_then_heartbeat=True,
        )
    finally:
        thread.join(timeout=2)
        emulator.close()

    assert result.drone_stream_window_observed
    assert not result.drone_stream_window_advanced
    assert result.drone_video_window_end == result.base_sequence
    assert result.drone_protocol_window_end == result.base_sequence


def test_an_advancing_drone_video_window_is_detected(tmp_path):
    emulator = Neo2Emulator()
    emulator.video_window_advance = 8
    thread = emulator.run_in_background(None, None)
    try:
        result = run_probe(
            capture_path=tmp_path / "advancing-window.jsonl",
            host="127.0.0.1",
            port=emulator.port,
            duration=1.1,
            send_syn=True,
            send_wifi_then_heartbeat=True,
        )
    finally:
        thread.join(timeout=2)
        emulator.close()

    assert result.drone_stream_window_observed
    assert result.drone_stream_window_advanced
    assert result.drone_video_window_end == (result.base_sequence + 8) & 0xFFFF
    assert result.drone_protocol_window_end == result.base_sequence


def test_static_capability_waits_for_the_heartbeat_transport_ack(tmp_path):
    emulator = Neo2Emulator()
    # The heartbeat is never acknowledged, so the third type 5 must not leave.
    thread = emulator.run_in_background(None)
    try:
        result = run_probe(
            capture_path=tmp_path / "static-capability-blocked.jsonl",
            host="127.0.0.1",
            port=emulator.port,
            duration=1.3,
            send_syn=True,
            send_wifi_then_heartbeat=True,
            send_static_capability=True,
        )
    finally:
        thread.join(timeout=2)
        emulator.close()

    assert not thread.is_alive()
    assert result.heartbeat_sent
    assert not result.heartbeat_transport_ack_seen
    assert not result.static_capability_sent
    assert result.static_capability_sequence is None
    assert not result.stateful_sequence_completed
    assert result.transmitted == 3
