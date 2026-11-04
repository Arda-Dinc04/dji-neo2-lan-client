"""Synthetic native fixtures and actual PC UDP exercise the WLM path."""

import json
from pathlib import Path
import struct
import subprocess
import sys

import pytest

from neo2_control.duml import DumlPacket, DumlSequence
from neo2_control.identity import WlmIdentity, load_or_create_ble_identity, load_or_create_identity
from neo2_control.session import SwudpSession, run_session
from neo2_control.transport import ReliableTransport, _build_datagram
from neo2_control.wlm_bootstrap import WlmWifiBootstrap, connected_device_prefix
from test_observation import LoopbackDrone, observed, until
from test_session import acceptance, status

FIXTURE = json.loads((Path(__file__).parent / 'fixtures/wlm-bootstrap-native.json').read_text())
BODY = bytes.fromhex(next(v['synthetic_body_hex'] for v in FIXTURE['device_push']
                          if v['state_before'] == 0 and v['count_byte'] == 1 and v['device_type'] == 2))
IDENTITY = WlmIdentity('synthetic-id')


def push(body=BODY, **changes):
    return DumlPacket(**(dict(sender=7, receiver=2, seq=0, cmd_type=0,
                              cmd_set=7, cmd_id=0x94, payload=body) | changes)).encode()


def bootstrap():
    client = WlmWifiBootstrap(IDENTITY, 9003)
    client.open()
    return client


def test_pc_producers_match_complete_native_fixture_and_keep_one_identity():
    client, events = bootstrap(), []
    first, kind, _ = client.next_report(0x8123)
    assert kind == 'link24' and not client.has_pending
    client.feed(push(), lambda name, **detail: events.append((name, detail)))
    repeat, kind, _ = client.next_report(0x8123)
    assert kind == 'link24'
    wifi, kind, _ = client.next_report(0x8123)
    assert kind == 'wifi31' and not client.has_pending
    for packet, receiver in ((first, 7), (repeat, 7), (wifi, 27)):
        expected = next(v['result']['native_frame_hex'] for v in FIXTURE['producers']
                        if v['result'] is not None and v['result']['receiver_type'] == receiver)
        assert packet.encode().hex() == expected
    assert first.payload[4:24] == wifi.payload[7:27]
    assert events[0][0] == 'wlm_device_observed'
    assert events[0][1]['native_cache_state'] == 3
    assert events[0][1]['application_ready'] is False


@pytest.mark.parametrize('row', [r for r in FIXTURE['device_push'] if r['state_before'] in (0, 65535)])
def test_first_device_branch_agrees_with_native_guard_vectors(row):
    client = bootstrap()
    client.next_report(0x8123)
    client.feed(push(bytes.fromhex(row['synthetic_body_hex'])), lambda *a, **k: None)
    native_repeat = any(call['producer'] == 'report' for call in row['calls'])
    assert client.has_pending is native_repeat
    assert client.snapshot()['native_cache_state'] == (3 if native_repeat else 0)
    if native_repeat:
        assert client.device.device.hex() == row['cached_device_hex']


@pytest.mark.parametrize('body', [b'', b'\x01', b'\x01' + bytes(29), bytes(1011)])
def test_prefix_requires_bounded_complete_first_entry(body):
    with pytest.raises(ValueError):
        connected_device_prefix(body)


@pytest.mark.parametrize('count', [0, 0x20, 0xE0])
def test_zero_low_five_bit_count_needs_no_device_entry(count):
    assert connected_device_prefix(bytes([count])) is None


def test_only_first_prefix_is_interpreted_even_with_larger_count_or_unknown_tail():
    prefix = connected_device_prefix(bytes([0xFF]) + BODY[1:] + b'unknown-tail')
    assert prefix.count == 31 and prefix.device == BODY[3:31]
    assert BODY[3:31].hex() not in repr(prefix)


@pytest.mark.parametrize('changes', [dict(sender=27), dict(receiver=3), dict(cmd_set=8),
                                    dict(cmd_id=0x95), dict(version=2)])
def test_unrelated_or_unsupported_frames_do_not_unlock_wifi_report(changes):
    client = bootstrap()
    client.next_report(0)
    client.feed(push(**changes), lambda *a, **k: None)
    assert not client.has_pending and client.device is None


def test_crc_invalid_carrier_cannot_promote_an_embedded_device_push():
    client = bootstrap()
    client.next_report(0)
    carrier = bytearray(DumlPacket(7, 2, 0, 0, 7, 0x94, push()).encode())
    carrier[-1] ^= 1
    client.feed(bytes(carrier), lambda *a, **k: None)
    assert client.device is None and not client.has_pending
    assert client.decoder.discarded_bytes == len(carrier)


def test_fragmented_push_and_duplicate_storm_schedule_at_most_three_reports():
    client, events = bootstrap(), []
    raw = push()
    for byte in raw:
        client.feed(bytes([byte]), lambda name, **data: events.append((name, data)))
    for _ in range(100):
        client.feed(raw, lambda name, **data: events.append((name, data)))
    assert client.snapshot()['reports_pending'] == 3
    assert len(events) == 1 and client.pushes == 101
    for sequence in range(3):
        client.next_report(sequence)
    assert not client.has_pending and len(client.reports) == 3
    assert BODY.hex() not in json.dumps(client.snapshot())
    assert IDENTITY.identification_id not in json.dumps(client.snapshot())


def test_missing_prefix_does_not_prevent_a_later_valid_push():
    client, events = bootstrap(), []
    client.next_report(0)
    client.feed(push(b'\x01'), lambda name, **detail: events.append((name, detail)))
    assert not client.has_pending and client.malformed == 1
    client.feed(push(), lambda name, **detail: events.append((name, detail)))
    assert client.snapshot()['reports_pending'] == 2
    assert [event[0] for event in events] == ['wlm_device_rejected', 'wlm_device_observed']


def test_report_response_matching_is_bounded_and_cannot_unlock_a_report():
    client, events = bootstrap(), []
    first, _, _ = client.next_report(0x8123)
    response = DumlPacket(7, 2, first.seq, 0xC0, 7, 0x93, b'\x07')
    from dataclasses import replace
    unrelated = [replace(response, sender=27), replace(response, receiver=3),
                 replace(response, seq=0x8124), replace(response, cmd_type=0x40)]
    data = (b''.join(packet.encode() for packet in unrelated)
            + replace(response, payload=b'').encode() + response.encode()
            + replace(response, payload=b'\x00').encode())
    client.feed(data, lambda name, **detail: events.append((name, detail)))
    assert client.snapshot()['reports'][0]['response_status'] == 7
    assert client.malformed == 1 and not client.has_pending
    assert len([e for e in events if e[0] == 'wlm_report_response']) == 1
    assert not client.snapshot()['application_ready']


def test_observer_requires_open_and_bounds_input_without_parsing_excess_data():
    client = WlmWifiBootstrap(IDENTITY, 9003)
    with pytest.raises(RuntimeError, match='not open'):
        client.feed(push(), lambda *a, **k: None)
    client.open()
    with pytest.raises(RuntimeError, match='once'):
        client.open()
    with pytest.raises(ValueError, match='bounded size'):
        client.feed(bytes(1024 * 1024 + 1), lambda *a, **k: None)
    assert client.device is None and len(client.reports) == 0


def opened(heartbeat=False):
    events = []
    client = SwudpSession(heartbeat=heartbeat, wlm_identity=IDENTITY,
        transport=ReliableTransport(session=0x1234, base_sequence=0xFFF8),
        duml_sequence=DumlSequence(current=0xFFFE), event_sink=events.append)
    client.start(0)
    client.receive(acceptance(), 0)
    return client, events


def application(client, now):
    return [DumlPacket.decode(packet.raw[20:]) for packet in client.tick(now) if packet.raw[6] == 5]


def test_one_allocator_transport_ack_gates_reports_but_duml_response_does_not():
    client, events = opened(heartbeat=True)
    first = application(client, 0)[0]
    assert (first.seq, len(first.payload), first.receiver) == (0x8000, 24, 7)
    client.receive(status(ack=0xFFF8, anchor=0xFFF8, payload=push()), .1)
    assert application(client, .2) == []
    client.receive(status(ack=8, anchor=0xFFF8), .3)
    assert application(client, .3) == []
    client.receive(status(ack=0, anchor=0xFFF8), .4)
    repeat = application(client, .4)[0]
    assert (repeat.seq, len(repeat.payload), repeat.receiver) == (0x8001, 24, 7)
    response = DumlPacket(7, 2, first.seq, 0xC0, 7, 0x93, b'\x07').encode()
    client.receive(status(ack=0, anchor=0xFFF8, payload=response), .5)
    assert application(client, .5) == []
    client.receive(status(ack=8, anchor=0xFFF8), .6)
    wifi = application(client, .6)[0]
    assert (wifi.seq, len(wifi.payload), wifi.receiver) == (0x8002, 31, 27)
    assert client.heartbeats_acknowledged == client.heartbeats_sent == 0
    client.receive(status(ack=16, anchor=0xFFF8), .7)
    heartbeat = application(client, .8)[0]
    assert (heartbeat.seq, heartbeat.cmd_set, heartbeat.cmd_id) == (0x8003, 0, 0)
    client.receive(status(ack=24, anchor=0xFFF8), .9)
    assert client.heartbeats_acknowledged == client.heartbeats_sent == 1
    snapshot = client.snapshot(.9)
    assert snapshot['wlm_bootstrap']['reports_transport_acknowledged'] == 3
    assert snapshot['wlm_bootstrap']['reports'][0]['response_status'] == 7
    assert not snapshot['application_ready']
    assert not any(e.get('application_ready') for e in events)


def test_report_ack_timeout_ends_session_without_retry_or_heartbeat():
    client, _ = opened(heartbeat=True)
    assert len(application(client, 0)) == 1
    client.receive(status(ack=0xFFF8, anchor=0xFFF8, payload=push()), .2)
    assert client.tick(1.01) == [] and client.reason == 'acknowledgement_timeout'
    assert client.wlm.snapshot()['reports_scheduled'] == 1
    assert client.heartbeats_sent == 0


def test_duplicate_acceptance_does_not_reset_observer_or_report_queue():
    client, _ = opened()
    application(client, 0)
    client.receive(status(ack=0, anchor=0xFFF8, payload=push()), .1)
    client.receive(acceptance(), .2)
    assert client.wlm.snapshot()['reports_pending'] == 2
    assert client.wlm.snapshot()['native_cache_state'] == 3


def test_out_of_order_protocol_blocks_only_unlock_after_complete_validated_frame():
    client, _ = opened()
    application(client, 0)
    raw = push()
    for step, payload in ((2, raw[17:]), (1, raw[:17])):
        body = struct.pack('<HHHHI', 0xFFF8, 8, 0, 0, 0x101) + payload
        client.receive(_build_datagram(0x1234, 3, body,
                       send_sequence=(0xFFF8 + step * 8) & 0xFFFF), .1)
        if step == 2:
            assert client.wlm.device is None
    assert client.wlm.device.device == BODY[3:31]
    assert client.receivers[3].start == 8


@pytest.fixture
def peer():
    drone = LoopbackDrone()
    drone.osd = push() + drone.osd
    yield drone
    drone.close()


def test_actual_cli_udp_path_schedules_three_reports_and_keeps_identity_private(tmp_path, peer):
    identity, capture = tmp_path / 'wlm.json', tmp_path / 'session.jsonl'
    process = subprocess.run([sys.executable, '-m', 'neo2_control', 'session',
        '--host', '127.0.0.1', '--port', str(peer.port), '--duration', '.5',
        '--capture', str(capture), '--wlm-identity-file', str(identity)],
        capture_output=True, text=True, timeout=5)
    assert process.returncode == 0, process.stderr
    result = json.loads(process.stdout)
    wlm = result['wlm_bootstrap']
    assert wlm['reports_scheduled'] == wlm['reports_transport_acknowledged'] == 3
    packets = [DumlPacket.decode(raw[20:]) for raw in peer.requests if raw[6] == 5]
    assert [len(p.payload) for p in packets] == [24, 24, 31]
    assert [p.receiver for p in packets] == [7, 7, 27]
    assert all(p.payload[1:3] == peer.port.to_bytes(2, 'little') for p in packets)
    assert len({p.seq for p in packets}) == 3
    assert result['heartbeats_sent'] == result['heartbeats_acknowledged'] == 0
    stored = load_or_create_identity(identity)
    public = process.stdout + Path(result['journal_path']).read_text()
    assert stored.identification_id not in public and BODY[3:31].hex() not in public
    assert not result['application_ready']
    for path in (identity, capture, Path(result['journal_path'])):
        assert path.stat().st_mode & 0o777 == 0o600


def test_managed_observation_uses_local_wlm_option_and_preserves_result_on_close(tmp_path, peer):
    identity = tmp_path / 'wlm.json'
    with observed(tmp_path, peer, wlm_identity_file=identity) as (controller, manager):
        with pytest.raises(ValueError, match='only observation duration'):
            manager.open({'wlm_identity_file': 'remote.json'})
        manager.open({'duration': 3})
        until(lambda: manager.snapshot()['session'].get('wlm_bootstrap', {}).get('reports_transport_acknowledged') == 3)
        live = controller.status()
        assert live['observation']['wlm_bootstrap_enabled']
        assert not live['flight_ready'] and not live['application_ready']
        manager.close()
        until(lambda: not manager.busy)
        final = manager.snapshot()
        assert final['session']['wlm_bootstrap']['reports_transport_acknowledged'] == 3
        assert str(identity) not in json.dumps(final)
        assert load_or_create_identity(identity).identification_id not in json.dumps(final)


def test_managed_reopening_uses_same_identity_but_fresh_device_cache(tmp_path, peer):
    identity, fingerprints = tmp_path / 'wlm.json', []
    with observed(tmp_path, peer, wlm_identity_file=identity) as (_, manager):
        for _ in range(2):
            manager.open({'duration': 3})
            until(lambda: manager.snapshot()['session'].get('wlm_bootstrap', {}).get('reports_transport_acknowledged') == 3)
            manager.close()
            until(lambda: not manager.busy)
            info = manager.snapshot()['session']['wlm_bootstrap']
            fingerprints.append(info['identity_sha256'])
            assert info['reports_scheduled'] == 3
        assert fingerprints[0] == fingerprints[1]
    assert sum(raw[6] == 0 for raw in peer.requests) == 2
    assert sum(raw[6] == 5 for raw in peer.requests) == 6


def test_invalid_ble_identity_is_rejected_before_session_socket_or_capture(tmp_path, monkeypatch):
    identity, capture = tmp_path / 'ble.json', tmp_path / 'capture.jsonl'
    load_or_create_ble_identity(identity)
    monkeypatch.setattr('neo2_control.session.socket.socket', lambda *a, **k: pytest.fail('socket opened'))
    with pytest.raises(ValueError, match='identity store format'):
        run_session(capture_path=capture, wlm_identity_file=identity)
    assert not capture.exists()


@pytest.mark.parametrize('target', ['capture', 'journal', 'video'])
def test_identity_output_collisions_are_rejected_before_creating_identity_or_socket(tmp_path, monkeypatch, target):
    capture, video = tmp_path / 'capture.jsonl', tmp_path / 'video.bin'
    identity = {'capture': capture, 'journal': Path(str(capture) + '.session.jsonl'), 'video': video}[target]
    monkeypatch.setattr('neo2_control.session.socket.socket', lambda *a, **k: pytest.fail('socket opened'))
    with pytest.raises(ValueError, match='WLM identity must be distinct'):
        run_session(capture_path=capture, video_path=video, wlm_identity_file=identity)
    assert not identity.exists() and not capture.exists()
