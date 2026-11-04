"""Owned Wi-Fi service switching through actual loopback UDP and the CLI."""
from dataclasses import replace
import json
import socket
import struct
import threading

import pytest

from neo2_control.duml import DumlPacket, DumlSequence
from neo2_control.duml_stream import DumlStreamDecoder
from neo2_control.identity import WlmIdentity
from neo2_control.session import SwudpSession, run_session
from neo2_control.transport import _build_datagram, describe_transport
from neo2_control.wlm_discovery import unpack_forwarded
from neo2_control.wlm_link import WlmWifiLink
from test_session import acceptance, status
from test_wlm_link import follow, send
from test_wlm_query import beacon
from test_wlm_query_session import TEMPLATE, FORWARDED, BEACON
from test_wlm_real_ack import carrier


IDENTITY = WlmIdentity('synthetic-liveview-pc')
OWN = IDENTITY.wlm_device_id
PEER = beacon().device_id
TARGET_MODES = [5, 4, 4, 2, 1]


def link_client(*, enabled=True, prepare=lambda modes: True):
    link = WlmWifiLink(IDENTITY, DumlSequence(current=0x8000).next,
                       switch_wifi_service=enabled, prepare_wifi=prepare)
    link.on_beacon(beacon(), 0)
    return link


def peer_frame(link, command, payload, *, sequence=0x9123, kind=0x40):
    """Synthetic peer-to-owned-PC envelope, including both routing hashes."""
    inner = replace(FORWARDED.packet, sender=0xe9, receiver=0xee, cmd_set=0x51,
                    cmd_id=command, cmd_type=kind, seq=sequence, payload=payload)
    attributes = struct.pack('<IBIIBB7x', link.read.peer_hash, 2, 0x76543210,
                             IDENTITY.wlm_device_hash, 1, 0x82)
    return replace(TEMPLATE, sender=0x27, receiver=0xee, cmd_type=0,
                   payload=inner.encode()+attributes).encode()


def capabilities_and_follow(link, *, response=None):
    send(link, 0, 'wlm_link_presence')
    link.feed(follow(link), .1)
    send(link, .1, 'wlm_link_follow_reply')
    for when in (.5, 1):
        link.on_beacon(beacon(), when)
        send(link, when, 'wlm_link_presence')
    query = send(link, 1.5, 'wlm_link_capability_query')
    reply = carrier(sequence=query.sequence) if response is None else response(query.sequence)
    link.feed(reply, 1.51)
    assert link.snapshot(1.51)['capability_read']['state'] == 'complete'


def test_service_switch_is_disabled_by_default_after_follow_and_capability_read():
    link = link_client(enabled=False)
    capabilities_and_follow(link)
    for when in (1.52, 2.02, 2.52):
        link.on_beacon(beacon(), when)
        send(link, when, 'wlm_link_presence')
    state = link.snapshot(2.52)
    assert state['modes_raw'] == [1, 1, 1, 0, 0]
    assert state['service_switch']['enabled'] is False
    assert not state['video_service_requested'] and not state['wifi_service_configured']
    assert state['video_bytes_received'] == 0
    session = SwudpSession(wlm_identity=IDENTITY)
    assert session.wlm_link is None


@pytest.mark.parametrize('blocked_by', ['support', 'mask'])
def test_completed_query_without_required_cap10_cannot_start_a_service_switch(blocked_by):
    def reply(sequence):
        outer = DumlPacket.decode(carrier(sequence=sequence))
        forwarded = unpack_forwarded(outer.payload)
        body = bytearray(forwarded.packet.payload)
        tag = body.index(b'\x0a\x11\x20', 11)
        if blocked_by == 'support':
            body[tag+11] &= ~4  # CAP10 bit2: liveview switching.
        else:
            body[tag+3] |= 4
        return replace(outer, payload=replace(forwarded.packet, payload=bytes(body)).encode()
                       +forwarded.attributes).encode()

    link = link_client()
    capabilities_and_follow(link, response=reply)
    send(link, 1.52, 'wlm_link_presence')
    assert link.service_switch is None
    assert not link.snapshot(1.52)['video_service_requested']


@pytest.mark.parametrize('command', [8, 6])
@pytest.mark.parametrize('change', ['source_hash', 'destination_hash', 'category'])
def test_switch_frames_require_both_owned_routing_hashes_and_wifi_category(command, change):
    link = link_client()
    capabilities_and_follow(link)
    send(link, 1.52, 'wlm_link_switch_trigger')
    if command == 8:
        raw = peer_frame(link, 8, PEER+OWN+bytes([5, 4, 4]))
    else:
        link.feed(peer_frame(link, 8, PEER+OWN+bytes([5, 4, 4])), 1.53)
        send(link, 1.53, 'wlm_link_switch_prepare_ack')
        own = send(link, 1.54, 'wlm_link_switch_completion')
        raw = peer_frame(link, 6, OWN+b'\0', sequence=own.sequence, kind=0x80)
    outer = DumlPacket.decode(raw)
    forwarded = unpack_forwarded(outer.payload)
    attributes = bytearray(forwarded.attributes)
    attributes[{'source_hash': 0, 'destination_hash': 9, 'category': 4}[change]] ^= 1
    malformed = replace(outer, payload=forwarded.packet.encode()+bytes(attributes)).encode()
    link.feed(malformed, 1.55)
    switch = link.service_switch
    assert not switch.own_completion_ack and not switch.committed
    if command == 8:
        assert switch.prepare_ack is None and not switch.prepared_local
    link.feed(raw, 1.56)
    if command == 8:
        send(link, 1.56, 'wlm_link_switch_prepare_ack')
        assert switch.prepared_local
    else:
        assert switch.own_completion_ack
    assert not switch.committed


@pytest.mark.parametrize('option', [True, 1, 'yes'])
def test_invalid_liveview_opt_in_fails_before_capture_or_network(tmp_path, option):
    path = tmp_path/'invalid.jsonl'
    with pytest.raises(ValueError):
        run_session(capture_path=path, host='127.0.0.1', wlm_wifi_liveview=option)
    with pytest.raises(ValueError):
        SwudpSession(wlm_wifi_liveview=option)
    assert not path.exists() and not path.with_name(path.name+'.session.jsonl').exists()


def test_liveview_requires_explicit_backend_owner_and_excludes_separate_query(tmp_path):
    for ready in (None, False, 1):
        with pytest.raises(ValueError):
            SwudpSession(wlm_identity=IDENTITY, wlm_wifi_liveview=True,
                         wifi_backend_ready=ready)
    with pytest.raises(ValueError):
        SwudpSession(wlm_identity=IDENTITY, wlm_wifi_liveview=True,
                     wifi_backend_ready=lambda: True, wlm_capability_query=True)
    path = tmp_path/'invalid.jsonl'
    with pytest.raises(ValueError):
        run_session(capture_path=path, host='127.0.0.1', wlm_wifi_liveview=True,
                    wlm_identity_file=tmp_path/'unused-identity.json', wlm_capability_query=True)
    assert not path.exists()


def test_cli_liveview_requires_owned_identity_before_creating_capture(tmp_path, capsys):
    from neo2_control.cli import main
    capture = tmp_path/'invalid.jsonl'
    assert main(['session', '--host', '127.0.0.1', '--capture', str(capture),
                 '--wlm-wifi-liveview']) == 1
    assert 'identity' in capsys.readouterr().err
    assert not capture.exists()


@pytest.mark.parametrize('via_cli', [False, True])
@pytest.mark.parametrize('peer_result_first', [False, True])
def test_loopback_udp_commits_wifi_service_only_after_bilateral_exchange(
        tmp_path, capsys, via_cli, peer_result_first):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    sock.settimeout(.02)
    stop = threading.Event()
    errors, seen, committed_presences = [], [], []
    registered = followed = completed = False
    oracle = link_client()
    request = follow(oracle, sequence=0x9050)
    video = b'\x00\x00\x00\x01\x65synthetic-loopback-video'

    def peer():
        nonlocal registered, followed, completed
        session = anchor = video_end = None
        try:
            while not stop.is_set():
                try:
                    raw, address = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                if raw[6] == 0:
                    session = int.from_bytes(raw[2:4], 'little')
                    anchor = video_end = int.from_bytes(raw[8:10], 'little')
                    sock.sendto(acceptance(session, flags=15), address)
                    continue
                decoded, _ = describe_transport(raw)
                offset = decoded.get('qos_aggregate_offset')
                size = decoded.get('qos_aggregate_length', 0)
                aggregate = b'' if offset is None else raw[offset:offset+size]
                outgoing = BEACON
                for outer, _ in DumlStreamDecoder().feed(aggregate):
                    if (outer.cmd_set, outer.cmd_id) != (0x51, 1):
                        continue
                    forwarded = unpack_forwarded(outer.payload)
                    inner, attributes = forwarded.packet, forwarded.attributes
                    assert (outer.sender, outer.receiver, inner.sender, inner.receiver) == (
                        0x3b, 0xe9, 0xee, 0xe9)
                    assert attributes[:4] == IDENTITY.wlm_device_hash.to_bytes(4, 'little')
                    assert attributes[4] == 2 and attributes[13:] == b'\x01\x82'+bytes(7)
                    seen.append(inner)
                    if inner.cmd_id == 0x13:
                        assert inner.payload[1:24] == OWN and inner.payload[51] == 4
                        if completed:
                            assert list(inner.payload[24:27]+inner.payload[52:54]) == TARGET_MODES
                            committed_presences.append(inner)
                        elif followed:
                            assert inner.payload[24:27] == bytes([1, 1, 1])
                        if not registered:
                            outgoing += request
                        registered = True
                    elif inner.cmd_id == 0x1c:
                        assert registered and inner.seq == 0x9050 and inner.cmd_type == 0xc0
                        assert inner.payload[8:12] == bytes(4)
                        assert inner.payload[12:17] == bytes([1, 1, 1, 0, 0])
                        followed = True
                    elif inner.cmd_id == 0x41:
                        assert registered and followed
                        assert inner.payload[9:] == b'\x06\0\x0a\0'
                        assert attributes[9:13] == bytes.fromhex('ffffffff')
                        outgoing += carrier(sequence=inner.seq)
                    else:
                        assert attributes[9:13] == oracle.read.peer_hash.to_bytes(4, 'little')
                        assert any(packet.cmd_id == 0x41 for packet in seen)
                        if inner.cmd_id == 2:
                            assert inner.cmd_type == 0x40
                            assert inner.payload == bytes.fromhex('0504040000000200020000')
                            outgoing += peer_frame(oracle, 8, PEER+OWN+bytes([5, 4, 4]))
                        elif inner.cmd_id == 8:
                            assert inner.cmd_type == 0xc0 and inner.seq == 0x9123
                            assert inner.payload == PEER+b'\0'
                        elif inner.cmd_id == 6 and inner.cmd_type == 0x40:
                            assert any(packet.cmd_id == 8 for packet in seen)
                            assert inner.payload == OWN+PEER+bytes(5)
                            ack = peer_frame(oracle, 6, OWN+b'\0', sequence=inner.seq,
                                             kind=0xc0 if via_cli else 0x80)
                            result = peer_frame(oracle, 6, PEER+OWN+bytes(5), sequence=0x9234)
                            outgoing += result+ack if peer_result_first else ack+result
                        elif inner.cmd_id == 6 and inner.cmd_type == 0xc0:
                            assert inner.seq == 0x9234 and inner.payload == PEER+b'\0'
                            assert not completed
                            completed = True
                            video_end = (anchor+8) & 0xffff
                            body = struct.pack('<HHHHI', anchor, video_end, 0, 0, 0x101)+video
                            sock.sendto(_build_datagram(session, 2, body,
                                                       send_sequence=video_end), address)
                        else:
                            raise AssertionError('unexpected application command')
                response = bytearray(status(session, anchor=anchor, ack=anchor, payload=outgoing))
                response[8:12] = struct.pack('<HH', video_end, video_end)
                sock.sendto(bytes(response), address)
        except BaseException as error:
            errors.append(repr(error))

    worker = threading.Thread(target=peer, daemon=True)
    worker.start()
    identity = tmp_path/'identity.json'
    identity.write_text(json.dumps({'format': 'neo2-control/wlm-identity/v1',
                                    'identification_id': IDENTITY.identification_id}))
    capture, video_path = tmp_path/'liveview.jsonl', tmp_path/'video.bin'
    try:
        if via_cli:
            from neo2_control.cli import main
            assert main(['session', '--capture', str(capture), '--host', '127.0.0.1',
                         '--port', str(sock.getsockname()[1]), '--duration', '2.4',
                         '--wlm-identity-file', str(identity), '--wlm-wifi-liveview',
                         '--video-output', str(video_path)]) == 0
            result = json.loads(capsys.readouterr().out)
        else:
            result = run_session(capture_path=capture, host='127.0.0.1',
                                 port=sock.getsockname()[1], duration=2.4,
                                 wlm_identity_file=identity, wlm_wifi_liveview=True,
                                 video_path=video_path).as_dict()
    finally:
        stop.set()
        worker.join(timeout=1)
        sock.close()
    assert not worker.is_alive() and not errors, errors
    assert completed and committed_presences
    assert result['state'] == 'closed' and result['reason'] == 'duration_elapsed'
    assert result['transport_accepted'] and not result['application_ready']
    state = result['wlm_mode_follow']
    assert state['capability_read']['state'] == 'complete'
    assert state['capability_read']['capabilities']['10']['support'] == 0x26d
    assert len([packet for packet in seen if packet.cmd_id == 0x41]) == 1
    switch = state['service_switch']
    assert switch['state'] == 'complete' and switch['committed'] and switch['prepared_local']
    assert switch['own_completion_ack'] and switch['peer_result_ack_sent']
    assert switch['trigger_sent'] == switch['prepare_acks_sent'] == 1
    assert switch['completions_sent'] == switch['result_acks_sent'] == 1
    assert state['modes_raw'] == TARGET_MODES
    assert state['video_service_requested'] and state['wifi_service_configured']
    assert state['video_bytes_received'] == result['video_bytes_delivered'] == len(video)
    assert video_path.read_bytes() == video
    assert not state['wifi_tunnel_active'] and not state['controlled_wifi_route_available']
    assert not state['application_ready'] and not switch['application_ready']
