"""Measured mode1/flags0x0f: native ACK bytes and a real local UDP session."""

import json
from pathlib import Path
import socket
import struct
import threading

import pytest

from neo2_control.duml import DumlPacket, DumlSequence
from neo2_control.session import SwudpSession, run_session
from neo2_control.receiver import ReliableReceiveState
from neo2_control.transport import ReceiveWindowStatus, ReliableTransport, _build_datagram, describe_transport
from test_session import acceptance, status
from test_telemetry import frame
from test_wlm_bootstrap import IDENTITY, push


def application(raw):
    decoded, _ = describe_transport(raw)
    offset = decoded['qos_aggregate_offset']
    return raw[offset:offset + decoded['qos_aggregate_length']]


def qos_client():
    client = SwudpSession(heartbeat=True, wlm_identity=IDENTITY,
        transport=ReliableTransport(session=0x1234, base_sequence=0x4000),
        duml_sequence=DumlSequence(current=0x8000))
    client.start(0)
    client.receive(acceptance(flags=0x0f), 0)
    return client


def test_full_ack_then_incremental_matches_complete_native_ground_builder():
    transport = ReliableTransport(session=0x1234, base_sequence=0x4000)
    holes = ReceiveWindowStatus(0x4000, 0x4018, (0x4008, 0x4010))
    full = transport.idle_window(periodic=False, video_receive=holes)
    assert full.hex() == '23803412000004810040184002000100f0004000400000000000400040000000000000'
    incremental = transport.idle_window(video_receive=holes)
    assert incremental.hex() == '1e803412000006be00401840000000400040000000400040000000000000'
    changed = ReceiveWindowStatus(0x4000, 0x4018, (0x400a, 0x4010))
    delta = transport.idle_window(video_receive=changed, video_changes=(0x400a,))
    assert delta[8:16] == bytes.fromhex('0040184001000a40')
    assert describe_transport(transport.idle_window(video_receive=changed))[0]['video_ack_count'] == 0
    received = ReceiveWindowStatus(0x4008, 0x4018, (0x4010,))
    advanced = describe_transport(transport.idle_window(video_receive=received))[0]
    assert advanced['video_window_start'] == 0x4008 and advanced['video_ack_count'] == 0


def test_mode_zero_full_ack_requires_explicit_decoder_context():
    transport = ReliableTransport(session=0x1234, base_sequence=0x4000, ack_mode=0)
    raw = transport.idle_window(periodic=False)
    decoded, _ = describe_transport(raw, ack_mode=0)
    assert len(raw) == 30 and decoded['type'] == 4
    assert decoded['video_ack_count'] == decoded['protocol_ack_count'] == 0
    with pytest.raises(ValueError):
        describe_transport(raw)


def test_every_native_ack_transition_in_both_modes_matches_production():
    fixture = json.loads((Path(__file__).parent / 'fixtures/swudp-negotiation-native.json').read_text())
    for case in fixture['ack_cases']:
        transport = ReliableTransport(session=0x1234, base_sequence=0x4000,
                                      ack_mode=case['management_mode'])
        receiver = ReliableReceiveState(session=0x1234, anchor=0x4000, channel=2, capacity=100)
        for record in case['records']:
            if record['kind'] == 'packet':
                receiver.receive(bytes.fromhex(record['input_hex']))
            else:
                receiver.observe_window(*record['window'])
            for output in record['outputs']:
                raw = transport.idle_window(periodic=output['incremental'], video_receive=receiver.status,
                                            video_changes=receiver.ack_events)
                receiver.consume_ack_events()
                assert raw.hex() == output['output_hex']
        for pending in case.get('pending_cases', []):
            receiver = ReliableReceiveState(session=0x1234, anchor=0x4000, channel=2,
                                            capacity=pending['capacity'])
            for record in pending['inputs']:
                if record['kind'] == 'packet':
                    receiver.receive(bytes.fromhex(record['input_hex']))
                else:
                    receiver.observe_window(*record['window'])
            for output in pending['outputs']:
                raw = transport.idle_window(periodic=output['incremental'], video_receive=receiver.status,
                                            video_changes=receiver.ack_events)
                receiver.consume_ack_events()
                assert raw.hex() == output['output_hex'], pending['name']


def test_native_qos_encoder_vectors_match_production_byte_for_byte():
    fixture = json.loads((Path(__file__).parent / 'fixtures/swudp-qos-native.json').read_text())
    for example in fixture['examples']:
        if example['ack_profile'] == 'mixed':
            continue  # Artificial partial FEC words are outside the PC SYN profile.
        transport = ReliableTransport(session=0xafc5, base_sequence=0x3020)
        before = example['ack_state_before_first']
        statuses = {key: ReceiveWindowStatus(value['start'], value['end'], tuple(value['missing']))
                    for key, value in before.items()}
        options = dict(video_receive=statuses['video'], protocol_receive=statuses['protocol'],
                       send_window_start=0x3020, send_window_end=0x3028)
        payload = b''.join(bytes.fromhex(row['duml_hex']) for row in example['queued'])
        first = transport.qos_data(payload, periodic=example['periodic'], **options,
            video_changes=tuple(before['video']['unreported_missing_delta']),
            protocol_changes=tuple(before['protocol']['unreported_missing_delta']))
        second = transport.idle_window(periodic=example['periodic'], **options)
        for actual, field in ((first, 'first'), (second, 'second_without_requeue')):
            expected = bytearray.fromhex(example[field]['raw_hex'])
            cursor = describe_transport(expected)[0]['qos_aggregate_offset'] - 10
            # The native fixture seeds opaque legacy flags to show preservation;
            # the PC initializes them to zero, as proven for a fresh SYN.
            expected[cursor + 4:cursor + 8] = bytes(4)
            assert actual == expected


def test_qos_reports_continue_without_transport_ack_and_are_never_replayed():
    client = qos_client()
    first = client.tick(0)
    link = next(p for p in first if p.application_kind == 'link24')
    assert link.raw[6] == 4 and link.tos == 128
    assert not client.wlm.snapshot()['reports'][0]['datagram_sent']
    client.datagram_sent(link, 0)
    client.receive(status(payload=push()), .1)
    sent = [link]
    for now in (.1, .2, .8, 1.4, 1.9):
        client.receive(status(), now)
        sent.extend(p for p in client.tick(now) if p.application_kind is not None)
    packets = [DumlPacket.decode(application(p.raw)) for p in sent]
    assert [(p.cmd_set, p.cmd_id) for p in packets[:3]] == [(7, 0x93)] * 3
    assert [len(p.payload) for p in packets[:3]] == [24, 24, 31]
    assert len({p.seq for p in packets}) == len(packets)
    assert len([p for p in packets if p.cmd_set == 0]) == 3
    assert all(p.tos == 128 for p in sent)
    assert client.sender.waiting_for_ack is None and client.ack_deadline is None
    assert client.sender.acknowledged_sequence == 0x4000
    assert client.pending_application is None and client.heartbeats_acknowledged == 0
    assert client.wlm.snapshot()['reports_transport_acknowledged'] == 0
    assert client.snapshot(1.9)['delivery_mode'] == 'qos'
    assert client.tick(3.91) == [] and client.reason == 'remote_timeout'


def test_duplicate_acceptance_cannot_switch_a_running_qos_profile_or_refresh_it():
    client = qos_client()
    client.receive(acceptance(flags=1), .9)
    client.receive(acceptance(flags=0x80), 1.8)
    assert client.qos and client.feature_flags == 15
    assert client.tick(2.01) == [] and client.reason == 'remote_timeout'


@pytest.mark.parametrize('flags,tos', [(3, 192), (15, 128)])
def test_qos_category_depends_on_negotiated_bit_three(flags, tos):
    client = SwudpSession(heartbeat=True, wlm_identity=IDENTITY,
        transport=ReliableTransport(session=0x1234, base_sequence=0x4000))
    client.start(0)
    client.receive(acceptance(flags=flags), 0)
    packets = client.tick(.8)
    assert all(packet.tos == tos for packet in packets if packet.application_kind is not None)
    assert next(packet for packet in packets if packet.application_kind is None).tos == 192


@pytest.mark.parametrize('flags,limit,interval', [(15, 0, .02), (15, 1, .02),
    (15, 3, .03), (15, 5, .05), (2, 255, .02), (15, 255, .1)])
def test_mode_one_full_ack_cadence_uses_negotiated_monitor_limit(flags, limit, interval):
    client = SwudpSession(transport=ReliableTransport(session=0x1234, base_sequence=0x4000))
    client.start(0)
    raw = bytearray(acceptance(flags=flags))
    raw[12] = limit
    client.receive(raw, 0)
    assert client.window_interval == interval
    assert client.tick(0)[0].raw[6] == 4
    assert client.tick(interval / 2) == []
    assert client.tick(interval)[0].raw[6] == 4


@pytest.mark.parametrize('payload', [b'', bytes(1443), bytes(65536)])
def test_qos_mtu_validation_does_not_consume_ack_changes(payload):
    transport = ReliableTransport(session=0x1234, base_sequence=0x4000)
    holes = ReceiveWindowStatus(0x4000, 0x4018, (0x4008, 0x4010))
    with pytest.raises(ValueError):
        transport.qos_data(payload, video_receive=holes)
    assert describe_transport(transport.idle_window(video_receive=holes,
        video_changes=holes.missing))[0]['video_ack_count'] == 2


def test_actual_udp_qos_session_receives_video_and_fc_without_any_reliable_ack(tmp_path, monkeypatch):
    # Graphical annotation must never hold up the live protocol owner. A
    # broken diagnostic decoder must not affect exact capture or UDP delivery.
    def unavailable_annotation(_):raise AssertionError('diagnostic decoder reached live UDP loop')
    monkeypatch.setattr('neo2_control.capture.decode_datagram', unavailable_annotation)
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    server.settimeout(.025)
    server.setsockopt(socket.IPPROTO_IP, socket.IP_RECVTOS, 1)
    stopped = threading.Event()
    requests, reports, errors, markings = [], [], [], []
    video = b'synthetic-video'

    def serve():
        session = anchor = None
        try:
            while not stopped.is_set():
                try:
                    raw, ancillary, _, peer = server.recvmsg(65535, 128)
                except socket.timeout:
                    continue
                requests.append(raw)
                markings.append(next(data[0] for level, kind, data in ancillary
                                     if level == socket.IPPROTO_IP and kind == socket.IP_TOS))
                if raw[6] == 0:
                    session = int.from_bytes(raw[2:4], 'little')
                    anchor = int.from_bytes(raw[8:10], 'little')
                    server.sendto(acceptance(session, flags=15), peer)
                    body = struct.pack('<HHHHI', anchor, (anchor + 8) & 0xffff, 0, 0, 0x101) + video
                    server.sendto(_build_datagram(session, 2, body,
                        send_sequence=(anchor + 8) & 0xffff), peer)
                    continue
                decoded, _ = describe_transport(raw)
                assert decoded['send_window_start'] == decoded['send_window_end'] == anchor
                assert decoded['send_sequence'] == 0 and len(raw) <= 1472
                reply = push() + frame()
                if application(raw):
                    packet = DumlPacket.decode(application(raw))
                    if (packet.cmd_set, packet.cmd_id) == (7, 0x93):
                        reports.append(packet)
                        reply += DumlPacket(packet.receiver, 2, packet.seq, 0xc0, 7, 0x93, b'\0').encode()
                # The reliable ACK frontier stays at the initial anchor forever.
                response = bytearray(status(session, ack=anchor, anchor=anchor, payload=reply))
                response[10:12] = ((anchor + 8) & 0xffff).to_bytes(2, 'little')
                server.sendto(response, peer)
        except BaseException as error:
            errors.append(error)

    thread = threading.Thread(target=serve)
    thread.start()
    try:
        result = run_session(host='127.0.0.1', port=server.getsockname()[1], duration=2.1,
            capture_path=tmp_path / 'qos.jsonl', heartbeat=True,
            video_path=tmp_path / 'video.bin', wlm_identity_file=tmp_path / 'id.json')
    finally:
        stopped.set()
        thread.join(timeout=1)
        server.close()
    assert not thread.is_alive() and not errors
    assert result.reason == 'duration_elapsed' and result.feature_flags == 15
    assert result.delivery_mode == 'qos' and not result.application_ready
    assert result.heartbeats_sent >= 3 and result.heartbeats_acknowledged == 0
    assert result.video_bytes_delivered == len(video) and (tmp_path / 'video.bin').read_bytes() == video
    capture = [json.loads(line) for line in Path(result.capture_path).read_text().splitlines()]
    assert [row['sequence'] for row in capture] == list(range(1, len(capture)+1))
    assert [bytes.fromhex(row['raw_hex']) for row in capture if row['direction']=='tx'] == requests
    assert all(row['decoded'] == {'length':len(bytes.fromhex(row['raw_hex'])),
                                  'annotations_deferred':True} for row in capture)
    assert sum(row['direction']=='rx' for row in capture) == result.received
    assert result.telemetry['accepted'] > 0
    assert result.wlm_bootstrap['reports_sent'] == 3 and len(reports) == 3
    assert result.wlm_bootstrap['reports_transport_acknowledged'] == 0
    assert all(r['datagram_sent'] and r['delivery_mode'] == 'qos' and r['response_status'] == 0
               for r in result.wlm_bootstrap['reports'])
    assert all(tos == (128 if application(raw) else 192)
               for raw, tos in zip(requests[1:], markings[1:]))
    journal = [json.loads(line) for line in Path(result.journal_path).read_text().splitlines()]
    assert not any(row['event'] == 'uplink_acknowledged' for row in journal)
    assert len([row for row in journal if row['event'] == 'wlm_report_scheduled']) == 3
