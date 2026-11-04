"""Protocol and socket tests for a persistent session, without drone access."""

from __future__ import annotations

import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import threading

import pytest

from neo2_control.duml import DumlPacket, DumlSequence
from neo2_control.session import SessionTiming, SwudpSession, run_session
from neo2_control.transport import ReliableTransport, _build_datagram, describe_transport, xor_checksum


def acceptance(session=0x1234, *, flags=1):
    raw = bytearray.fromhex("0f803412000000000101000104401f")
    raw[2:4] = session.to_bytes(2, "little")
    raw[9:11] = flags.to_bytes(2, "little")
    raw[7] = xor_checksum(raw[:7])
    return bytes(raw)


def status(session=0x1234, *, ack=0x4000, anchor=0x4000, payload=b""):
    raw = bytearray(34 + len(payload))
    raw[:2] = (0x8000 | len(raw)).to_bytes(2, "little")
    raw[2:4] = session.to_bytes(2, "little")
    raw[6] = 1
    for offset in (8, 10, 16, 18):
        raw[offset:offset + 2] = anchor.to_bytes(2, "little")
    for offset in (24, 26):
        raw[offset:offset + 2] = ack.to_bytes(2, "little")
    raw[32:34] = len(payload).to_bytes(2, "little")
    raw[34:] = payload
    raw[7] = xor_checksum(raw[:7])
    return bytes(raw)


def opened_client(*, heartbeat=False, anchor=0x4000):
    client = SwudpSession(
        heartbeat=heartbeat,
        transport=ReliableTransport(session=0x1234, base_sequence=anchor),
        duml_sequence=DumlSequence(current=0xFFFE),
    )
    client.start(0.0)
    client.receive(acceptance(), 0.0)
    return client


def test_long_session_keeps_one_sequence_allocator_and_actual_send_windows():
    client = opened_client(heartbeat=True, anchor=0xFFF8)
    heartbeat_frames = []
    for step in range(1, 6001):
        now = step / 10
        client.receive(status(ack=client.sender.acknowledged_sequence, anchor=0xFFF8), now)
        for packet in client.tick(now):
            decoded, _ = describe_transport(packet.raw)
            if decoded["type"] == 5:
                heartbeat_frames.append(packet.raw)
                client.receive(status(ack=decoded["send_sequence"], anchor=0xFFF8), now)
    assert client.state == "open"
    assert len(heartbeat_frames) >= 1000
    assert client.heartbeats_acknowledged == len(heartbeat_frames)
    first = describe_transport(heartbeat_frames[0])[0]
    second = describe_transport(heartbeat_frames[1])[0]
    assert (first["window_start"], first["window_end"]) == (0xFFF8, 0)
    assert (second["window_start"], second["window_end"]) == (0, 8)
    for index, raw in enumerate(heartbeat_frames):
        duml = DumlPacket.decode(raw[20:])
        assert describe_transport(raw)[0]["stream_slot"] == (index + 1) & 0xFF
        assert duml.seq == 0x8000 + index
        assert (duml.cmd_set, duml.cmd_id, duml.payload) == (0, 0, b"\x00\x02\x01\x00")
    window = describe_transport(client.sender.idle_window())[0]
    assert window["send_window_start"] == window["send_window_end"]
    assert window["send_window_end"] == client.sender.acknowledged_sequence


def test_pending_window_reports_wrapped_zero_and_no_second_send_before_ack():
    client = opened_client(heartbeat=True, anchor=0xFFF8)
    outgoing = client.tick(0.8)
    window = next(p.raw for p in outgoing if p.raw[6] == 4)
    decoded, _ = describe_transport(window)
    assert (decoded["send_window_start"], decoded["send_window_end"]) == (0xFFF8, 0)
    client.receive(status(ack=8, anchor=0xFFF8), 1.1)
    assert not any(p.raw[6] == 5 for p in client.tick(1.4))
    assert client.heartbeats_acknowledged == 0
    assert client.tick(1.81) == []
    assert client.reason == "acknowledgement_timeout"


def test_transport_only_session_never_sends_application_packets():
    client = opened_client()
    for step in range(1, 101):
        now = step / 10
        client.receive(status(), now)
        assert all(p.raw[6] == 4 for p in client.tick(now))
    assert client.heartbeats_sent == 0


def test_invalid_and_foreign_packets_cannot_open_or_keep_a_session_alive():
    client = opened_client()
    client.receive(status(session=0x5678), 0.9)
    corrupt = bytearray(status())
    corrupt[7] ^= 1
    client.receive(bytes(corrupt), 1.1)
    client.receive(acceptance(), 1.9)  # delayed duplicate, no liveness refresh
    assert client.tick(2.01) == []
    assert client.reason == "remote_timeout"
    assert client.rejected_datagrams == 2


def test_short_or_misdeclared_status_does_not_release_pending_ack():
    client = opened_client(heartbeat=True)
    client.tick(0.8)
    truncated = bytearray(status(ack=0x4008)[:26])
    truncated[:2] = (0x8000 | len(truncated)).to_bytes(2, "little")
    truncated[7] = xor_checksum(truncated[:7])
    client.receive(bytes(truncated), 0.9)
    misdeclared = bytearray(status(ack=0x4008))
    misdeclared[32:34] = (1).to_bytes(2, "little")
    client.receive(bytes(misdeclared), 1.0)
    assert client.sender.waiting_for_ack == 0x4008
    assert client.heartbeats_acknowledged == 0
    client.receive(status(ack=0x4008), 1.1)
    assert client.sender.waiting_for_ack is None
    assert client.heartbeats_acknowledged == 1


def test_acceptance_profile_and_deadline_are_enforced():
    client = SwudpSession(transport=ReliableTransport(session=0x1234))
    client.start(0)
    client.receive(acceptance(flags=0x10), 0.1)
    assert client.reason == "unsupported_feature_flags"
    assert client.tick(1) == []
    client = SwudpSession(transport=ReliableTransport(session=0x1234))
    client.start(0)
    client.receive(acceptance(), 3)
    assert client.reason == "handshake_timeout"
    with pytest.raises(RuntimeError):
        client.start(4)


@pytest.mark.parametrize("stream_type", [2, 3])
def test_invalid_stream_window_never_advances_ack_or_delivers_bytes(stream_type):
    client = opened_client()
    raw = bytearray(20)
    raw[:2] = (0x8000 | len(raw)).to_bytes(2, "little")
    raw[2:4] = (0x1234).to_bytes(2, "little")
    raw[6] = stream_type
    raw[7] = xor_checksum(raw[:7])
    client.receive(bytes(raw), 0.1)
    assert client.state == "open"
    assert client.rejected_datagrams == 1
    assert client.receivers[stream_type].start == 0x4000
    window = describe_transport(client.tick(0.2)[0].raw)[0]
    assert window['video_window_start'] == window['protocol_window_start'] == 0x4000


def test_video_checkpoint_words_leave_uplink_ack_and_protocol_window_independent():
    client = opened_client(heartbeat=True)
    client.tick(0.8)
    raw = bytearray(status(ack=0x4008))
    raw[10:12] = (0x4008).to_bytes(2, 'little')
    raw[12:14] = (0x4009).to_bytes(2, "little")
    client.receive(bytes(raw), 0.9)
    assert client.state == 'open'
    assert client.sender.waiting_for_ack is None
    assert client.receivers[2].status.missing == (0x400A,)
    assert client.receivers[3].status.missing == ()


def test_video_and_protocol_queues_deliver_in_order_and_refresh_the_link():
    delivered = []
    client = SwudpSession(
        transport=ReliableTransport(session=0x1234, base_sequence=0x4000),
        payload_sink=lambda channel, payload: delivered.append((channel, payload)),
    )
    client.start(0)
    client.receive(acceptance(), 0)
    def packet(channel, sequence, payload):
        body = struct.pack('<HHHHI', 0x4000, 0x4010, 0, 0, 0x101) + payload
        return _build_datagram(0x1234, channel, body, send_sequence=sequence)
    client.receive(packet(2, 0x4010, b'b'), 1)
    client.receive(packet(3, 0x4008, b'protocol'), 1.5)
    assert delivered == [(3, b'protocol')]
    client.receive(packet(2, 0x4008, b'a'), 2.5)
    assert delivered == [(3, b'protocol'), (2, b'a'), (2, b'b')]
    window = describe_transport(client.tick(3.0)[0].raw)[0]
    assert window['video_window_start'] == 0x4010
    assert window['protocol_window_start'] == 0x4008
    assert window['protocol_ack_slot_count'] == 1
    assert client.video_bytes_delivered == 2
    assert client.protocol_bytes_delivered == 8
    assert client.state == 'open'


def test_type_1_uplink_bitmap_walks_to_aggregate_and_releases_ack():
    delivered = []
    client = opened_client(heartbeat=True)
    client.payload_sink = lambda channel, payload: delivered.append((channel, payload))
    client.tick(0.8)
    raw = bytearray(status(ack=0x4008, payload=b'aggregate'))
    raw[28:30] = (4).to_bytes(2, 'little')
    raw[30:32] = (1).to_bytes(2, 'little')
    raw[32:32] = b'\xff'
    raw[:2] = (0x8000 | len(raw)).to_bytes(2, 'little')
    raw[7] = xor_checksum(raw[:7])
    decoded = describe_transport(bytes(raw))[0]
    assert decoded['payload_offset'] == 35
    assert decoded['uplink_ack_bitmap_length'] == 1
    assert decoded['downlink_aggregate_length'] == 9
    client.receive(bytes(raw), 0.9)
    assert client.sender.waiting_for_ack is None
    assert delivered == [(3, b'aggregate')]
    raw[28:30] = (21).to_bytes(2, 'little')
    # Full bitmap present, but beyond the measured ground send capacity.
    raw[33:33] = bytes(5)
    raw[:2] = (0x8000 | len(raw)).to_bytes(2, 'little')
    raw[7] = xor_checksum(raw[:7])
    client.receive(bytes(raw), 0.1)
    assert client.rejected_datagrams == 1
    assert delivered == [(3, b'aggregate')]


def test_video_output_never_overwrites_capture_journal_or_existing_file(tmp_path):
    path = tmp_path / 'capture.jsonl'
    for video in (path, path.with_name(path.name + '.session.jsonl')):
        with pytest.raises(ValueError, match='distinct'):
            run_session(capture_path=path, video_path=video)
        assert not path.exists()
    video = tmp_path / 'video.bin'
    video.write_bytes(b'previous')
    with pytest.raises(ValueError, match='new path'):
        run_session(capture_path=path, video_path=video)
    assert video.read_bytes() == b'previous'
    assert not path.exists()


@pytest.mark.parametrize("duration", [float("nan"), float("inf"), -1, 0, 86401])
def test_invalid_duration_cannot_create_files_or_open_socket(tmp_path, duration):
    path = tmp_path / "capture.jsonl"
    with pytest.raises(ValueError):
        run_session(capture_path=path, duration=duration)
    assert not path.exists()


def test_existing_capture_is_never_appended_to_by_a_new_session(tmp_path):
    path = tmp_path / "capture.jsonl"
    path.write_text("previous session\n")
    with pytest.raises(ValueError):
        run_session(capture_path=path)
    assert path.read_text() == "previous session\n"


def test_nonfinite_timeouts_are_rejected():
    with pytest.raises(ValueError):
        SessionTiming(acknowledgement_timeout=float("nan"))


def test_cli_runs_real_udp_session_and_keeps_packet_and_decision_logs(tmp_path):
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("127.0.0.1", 0))
    server.settimeout(0.05)
    port = server.getsockname()[1]
    stopped = threading.Event()
    requests = []
    errors = []
    osd_body = bytearray(37)
    struct.pack_into('<I', osd_body, 32, 8)
    osd_body[30], osd_body[36] = 17, 4
    osd_frame = DumlPacket(3, 2, 0, 0x40, 3, 0x43, bytes(osd_body)).encode()

    def serve():
        session = anchor = acknowledged = None
        try:
            while not stopped.is_set():
                try:
                    raw, peer = server.recvfrom(65535)
                except socket.timeout:
                    continue
                requests.append(raw)
                if raw[6] == 0:
                    session = int.from_bytes(raw[2:4], "little")
                    anchor = acknowledged = int.from_bytes(raw[8:10], "little")
                    server.sendto(acceptance(session), peer)
                    for step, payload in ((2, b'second'), (1, b'first')):
                        end = (anchor + 16) & 0xFFFF
                        body = struct.pack('<HHHHI', anchor, end, 0, 0, 0x101) + payload
                        server.sendto(_build_datagram(session, 2, body, send_sequence=(anchor + step * 8) & 0xFFFF), peer)
                elif session is not None:
                    if raw[6] == 5:
                        acknowledged = int.from_bytes(raw[4:6], "little")
                    reply = bytearray(status(session, ack=acknowledged, anchor=anchor,
                                             payload=osd_frame))
                    reply[10:12] = ((anchor + 16) & 0xFFFF).to_bytes(2, 'little')
                    server.sendto(bytes(reply), peer)
        except BaseException as error:
            errors.append(error)

    worker = threading.Thread(target=serve)
    worker.start()
    path = tmp_path / "session.jsonl"
    video = tmp_path / 'video.bin'
    try:
        process = subprocess.run([
            sys.executable, "-m", "neo2_control", "session",
            "--host", "127.0.0.1", "--port", str(port),
            "--duration", "1.7", "--heartbeat", "--capture", str(path),
            '--video-output', str(video),
        ], capture_output=True, text=True, timeout=6)
    finally:
        stopped.set()
        worker.join(timeout=1)
        server.close()
    assert not worker.is_alive()
    assert not errors
    assert process.returncode == 0, process.stderr
    result = json.loads(process.stdout)
    assert result["state"] == "closed" and result["reason"] == "duration_elapsed"
    assert result["transport_accepted"] and not result["application_ready"]
    assert result["heartbeats_sent"] >= 1
    assert result["heartbeats_acknowledged"] == result["heartbeats_sent"]
    assert result['video_bytes_delivered'] == 11
    assert result['protocol_bytes_delivered'] >= len(osd_frame)
    telemetry = result['telemetry']
    assert telemetry['available'] and telemetry['fresh']
    assert not telemetry['flight_ready']
    assert telemetry['fields']['motors_on']['value'] is True
    assert telemetry['fields']['is_flying']['value'] is False
    assert telemetry['fields']['satellite_count']['value'] == 4
    assert result['video_path'] == str(video)
    assert video.read_bytes() == b'firstsecond'
    for private_path in (path, Path(result['journal_path']), video):
        assert private_path.stat().st_mode & 0o777 == 0o600
    packet_log = [json.loads(line) for line in path.read_text().splitlines()]
    assert result["transmitted"] == len(requests)
    assert result["transmitted"] == sum(row["direction"] == "tx" for row in packet_log)
    assert result["received"] == sum(row["direction"] == "rx" for row in packet_log)
    journal = [json.loads(line) for line in Path(result["journal_path"]).read_text().splitlines()]
    assert any(row["event"] == "transport_accepted" for row in journal)
    assert any(row["event"] == "uplink_acknowledged" for row in journal)
    updates = [row['telemetry'] for row in journal if row['event'] == 'telemetry_updated']
    assert updates and updates[-1]['fields']['flight_mode']['value'] == 17
    assert journal[-1]["event"] == "session_result"
    assert journal[-1]["application_ready"] is False
