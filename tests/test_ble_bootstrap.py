"""PC GATT peer: fragments both directions and never touches a radio."""

import asyncio
from dataclasses import replace
import json
from pathlib import Path
import stat
from types import SimpleNamespace

import pytest

from neo2_control import ble_bootstrap as setup
from neo2_control.capture import load_capture
from neo2_control.cli import main
from neo2_control.duml import DumlPacket
from neo2_control.duml_stream import DumlStreamDecoder
from neo2_control.identity import load_or_create_ble_identity, load_or_create_identity


ADDRESS = '02:00:00:00:00:01'
SSID = b'NEO2-LAB'
PASSWORD = b'test-only-secret'
VALUES = {0x45: b'\0\1', 0x0C: bytes.fromhex('00020000000002'),
          7: bytes([0, len(SSID)]) + SSID,
          0x0E: bytes([0, len(PASSWORD)]) + PASSWORD, 0x93: b'\0'}


class Peer:
    def __init__(self, *, limit=20, transform=None, mode=None):
        self.writer = SimpleNamespace(uuid=setup.WRITE_UUID, handle=16,
                                      service_uuid=setup.SERVICE_UUID,
                                      properties=['write-without-response'],
                                      max_write_without_response_size=limit)
        self.notifier = SimpleNamespace(uuid=setup.NOTIFY_UUID, handle=13,
                                        service_uuid=setup.SERVICE_UUID, properties=['notify'])
        self.services = SimpleNamespace(get_characteristic=lambda uuid: {
            setup.WRITE_UUID: self.writer, setup.NOTIFY_UUID: self.notifier}.get(uuid))
        self.decoder = DumlStreamDecoder()
        self.transform, self.mode = transform, mode
        self.packets, self.chunks = [], []
        self.is_connected = self.exited = False
        self.callback = None

    def factory(self, address, **kwargs):
        assert address == ADDRESS and kwargs['pair'] is False
        self.disconnected = kwargs['disconnected_callback']
        return self

    async def __aenter__(self):
        self.is_connected = True
        return self

    async def __aexit__(self, *args):
        self.is_connected, self.exited = False, True
        self.disconnected(self)
        if self.mode == 'cleanup_error':
            raise RuntimeError(PASSWORD.decode())

    async def start_notify(self, characteristic, callback):
        assert characteristic is self.notifier
        self.callback = callback

    async def write_gatt_char(self, characteristic, data, *, response):
        assert self.callback is not None and characteristic is self.writer and response is False
        assert len(data) <= self.writer.max_write_without_response_size
        self.chunks.append(bytes(data))
        if self.mode == 'write_error':
            raise RuntimeError(PASSWORD.decode())
        if self.mode == 'disconnect':
            self.is_connected = False
            self.disconnected(self)
            return
        if self.mode == 'cancel':
            raise asyncio.CancelledError
        if self.mode == 'stall':
            await asyncio.Event().wait()
        for packet, _ in self.decoder.feed(data):
            self.packets.append(packet)
            if packet.cmd_id == 0x46 and packet.cmd_type == 0xC0:
                continue  # An ACK has no response of its own.
            reply = DumlPacket(7, 2, packet.seq, 0xC0, 7, packet.cmd_id, VALUES[packet.cmd_id])
            raw = self.transform(packet, reply) if self.transform else reply.encode()
            if raw is not None:
                self.emit(raw)

    def emit(self, raw):
        # Split header, payload and CRC across arbitrary callbacks.
        for start in range(0, len(raw), 3):
            self.callback(self.notifier, bytearray(raw[start:start + 3]))


@pytest.fixture
def paths(tmp_path):
    return dict(address=ADDRESS, identity_file=tmp_path / 'ble-id.json',
                credentials_path=tmp_path / 'credentials.json', capture_path=tmp_path / 'ble.jsonl',
                timeout=0.5, request_timeout=0.1)


def execute(monkeypatch, peer, paths):
    monkeypatch.setattr(setup, '_load_bleak_client', lambda: peer.factory)
    return asyncio.run(setup.run_ble_wifi_setup(**paths))


@pytest.mark.parametrize('limit,expected_chunks', [(20, [20, 20, 14, 13, 13, 13]),
                                                  (244, [54, 13, 13, 13])])
def test_complete_exchange_fragmentation_and_private_outputs(monkeypatch, paths, limit, expected_chunks):
    peer = Peer(limit=limit)
    result = execute(monkeypatch, peer, paths)
    assert result.state == 'complete' and result.reason == 'credentials_saved'
    assert result.stage == 'complete' and peer.exited
    assert result.requests_queued == result.responses_received == 4
    assert [p.cmd_id for p in peer.packets] == [0x45, 0x0C, 7, 0x0E]
    assert len({p.seq for p in peer.packets}) == 4
    assert [len(c) for c in peer.chunks] == expected_chunks
    assert all((p.sender, p.receiver, p.cmd_set, p.cmd_type) == (2, 7, 7, 0x40)
               for p in peer.packets)
    identity = load_or_create_ble_identity(paths['identity_file'])
    assert peer.packets[0].payload == b'\x20' + identity.identification_id.encode() + b'\x07DJI Fly'
    credentials = setup.load_wifi_credentials(paths['credentials_path'])
    assert credentials.ssid == SSID and credentials.passphrase == PASSWORD
    assert result.as_dict()['application_ready'] is False
    safe_output = json.dumps(result.as_dict()) + Path(result.journal_path).read_text()
    for secret in [SSID.decode(), PASSWORD.decode(), identity.identification_id]:
        assert secret not in safe_output and secret not in repr(credentials)
    for path in [paths['capture_path'], paths['identity_file'], paths['credentials_path'],
                 Path(result.journal_path)]:
        assert stat.S_IMODE(path.stat().st_mode) == 0o600
    capture = load_capture(paths['capture_path'])
    assert sum(e['interface'] == 'ble-duml' and e['direction'] == 'rx' for e in capture) == 4
    assert sum(e['interface'] == 'ble-gatt' and e['direction'] == 'tx' for e in capture) == len(peer.chunks)
    frames = [e for e in capture if e['interface'] == 'ble-duml']
    assert all(e['decoded']['layers'][0]['name'] == 'duml' for e in frames)
    assert all(e['decoded']['fields'] for e in frames)


def test_unrelated_and_duplicate_frames_cannot_advance_exchange(monkeypatch, paths):
    def transform(request, reply):
        unrelated = [replace(reply, sender=8), replace(reply, receiver=3),
                     replace(reply, seq=reply.seq ^ 1), replace(reply, cmd_type=0x40),
                     replace(reply, cmd_set=0), replace(reply, cmd_id=0xFF)]
        return b'noise' + b''.join(p.encode() for p in unrelated + [reply, reply])
    result = execute(monkeypatch, Peer(transform=transform), paths)
    assert result.state == 'complete' and result.responses_received == 4
    assert result.ignored_frames == 28 and result.discarded_bytes == 20


def test_write_limit_is_rechecked_when_mtu_changes(monkeypatch, paths):
    peer = Peer()
    original = peer.write_gatt_char
    async def change_mtu(*args, **kwargs):
        await original(*args, **kwargs)
        peer.writer.max_write_without_response_size = 244
    peer.write_gatt_char = change_mtu
    result = execute(monkeypatch, peer, paths)
    assert result.state == 'complete'
    assert [len(c) for c in peer.chunks] == [20, 34, 13, 13, 13]


@pytest.mark.parametrize('mutation', [dict(seq=1), dict(sender=8), dict(receiver=3),
                                     dict(cmd_type=0x40), dict(cmd_set=0), dict(cmd_id=0xFF),
                                     dict(version=2)])
def test_only_matching_reply_releases_next_request(monkeypatch, paths, mutation):
    peer = Peer(transform=lambda request, reply: replace(reply, **mutation).encode())
    result = execute(monkeypatch, peer, paths)
    assert result.state == 'failed' and result.reason == 'response_timeout'
    assert len(peer.packets) == 1 and result.responses_received == 0 and peer.exited
    assert not paths['credentials_path'].exists()


@pytest.mark.parametrize('command,payload', [(0x45, b'\x01'),
    (0x0C, b'\0\1'), (7, b'\0\x03ab'), (7, b'\0\0'), (0x0E, b'\0\x01ab')])
def test_malformed_or_rejected_reply_stops_at_that_stage(monkeypatch, paths, command, payload):
    peer = Peer(transform=lambda req, reply: replace(reply, payload=payload).encode()
                if req.cmd_id == command else reply.encode())
    result = execute(monkeypatch, peer, paths)
    assert result.state == 'failed' and result.stage == dict(setup.STEPS)[command]
    assert peer.packets[-1].cmd_id == command and peer.exited
    assert not paths['credentials_path'].exists()


@pytest.mark.parametrize('mode,reason', [('write_error', 'bluetooth_error'),
                                       ('disconnect', 'bluetooth_disconnected'),
                                       ('stall', 'response_timeout')])
def test_backend_failures_stop_writes_and_cleanup(monkeypatch, paths, mode, reason):
    peer = Peer(mode=mode)
    result = execute(monkeypatch, peer, paths)
    assert result.state == 'failed' and result.reason == reason and peer.exited
    assert len(peer.chunks) == 1 and not paths['credentials_path'].exists()
    assert PASSWORD.decode() not in Path(result.journal_path).read_text()


@pytest.mark.parametrize('value,reason', [
    (3, 'verification_device_busy'), (4, 'verification_high_temperature')])
def test_native_verification_states_are_reported_without_advancing(monkeypatch, paths, value, reason):
    peer = Peer(transform=lambda req, reply: replace(reply, payload=bytes([0, value])).encode())
    result = execute(monkeypatch, peer, paths)
    assert result.reason == reason and result.stage == 'identify_client'
    assert result.state == 'failed' and len(peer.packets) == 1
    assert not paths['credentials_path'].exists()


def verification_push(payload=b'\x01', **changes):
    return replace(DumlPacket(7, 2, 0, 0x40, 7, 0x46, payload), **changes)


@pytest.mark.parametrize('cmd_type', [0x00, 0x40, 0x80, 0xC0])
@pytest.mark.parametrize('before_reply', [False, True])
def test_pending_verification_ack_precedes_credentials_and_copies_push_sequence(
        monkeypatch, paths, cmd_type, before_reply):
    push = verification_push(cmd_type=cmd_type)
    def transform(request, reply):
        if request.cmd_id != 0x45:
            return reply.encode()
        pending = replace(reply, payload=b'\x00\x02').encode()
        return push.encode() + pending if before_reply else pending + push.encode()
    peer = Peer(transform=transform)
    result = execute(monkeypatch, peer, paths)
    assert result.state == 'complete' and result.verification_acks == 1
    assert result.requests_queued == result.responses_received == 4
    assert [p.cmd_id for p in peer.packets] == [0x45, 0x46, 0x0C, 7, 0x0E]
    assert peer.packets[1] == DumlPacket(2, 7, push.seq, 0xC0, 7, 0x46, b'\x00')
    assert len({p.seq for p in peer.packets if p.cmd_type == 0x40}) == 4
    events = [json.loads(line)['event'] for line in Path(result.journal_path).read_text().splitlines()]
    assert events.index('verification_ack_queued') < events.index('verification_push')
    assert events.index('verification_push') < events.index('reply_accepted')


def test_pending_decision_can_take_longer_than_request_timeout(monkeypatch, paths):
    paths.update(timeout=2, request_timeout=0.1)
    peer = Peer()
    def transform(request, reply):
        if request.cmd_id == 0x45:
            asyncio.get_running_loop().call_later(
                0.15, peer.emit, verification_push(seq=0x1234).encode())
            return replace(reply, payload=b'\x00\x02').encode()
        return reply.encode()
    peer.transform = transform
    result = execute(monkeypatch, peer, paths)
    assert result.state == 'complete' and result.verification_acks == 1
    assert peer.packets[1].seq == 0x1234


@pytest.mark.parametrize('sequence', [0, 0x4567, 0xFFFF])
def test_pc_ack_matches_retained_native_encoder_fixture(monkeypatch, paths, sequence):
    fixture = json.loads((Path(__file__).parent / 'fixtures/ble-pending-native.json').read_text())
    expected = next(v['native_ack_frame_hex'] for v in fixture['vectors']
                    if v['incoming_sequence_fixture'] == sequence and v['profile_fixture'] == 1
                    and v['native_ack_frame_hex'] is not None)
    peer = Peer(transform=lambda req, reply: (
        replace(reply, payload=b'\x00\x02').encode() + verification_push(seq=sequence).encode()
        if req.cmd_id == 0x45 else reply.encode()))
    result = execute(monkeypatch, peer, paths)
    assert result.state == 'complete' and peer.packets[1].encode().hex() == expected


@pytest.mark.parametrize('payload,reason', [
    (b'\x00', 'verification_denied'),
    (b'\x00\x01', 'verification_denied'),  # No retcode prefix in a push.
    (b'\x02', 'verification_timeout'),
    (b'\x03', 'verification_needs_request'),
    (b'\x04', 'verification_device_busy'),
    (b'\x05', 'verification_high_temperature'),
    (b'\x06', 'unsupported_verification_result'),
    (b'\xff', 'unsupported_verification_result'),
    (b'', 'malformed_verification_push'),
])
def test_pending_rejection_is_acknowledged_without_reading_credentials(monkeypatch, paths, payload, reason):
    def transform(request, reply):
        return replace(reply, payload=b'\x00\x02').encode() + verification_push(payload).encode()
    peer = Peer(transform=transform)
    result = execute(monkeypatch, peer, paths)
    assert result.reason == reason and result.state == 'failed'
    assert [p.cmd_id for p in peer.packets] == [0x45, 0x46]
    assert result.verification_acks == 1 and not paths['credentials_path'].exists()


def test_duplicate_decisions_are_acknowledged_but_cannot_replace_a_denial(monkeypatch, paths):
    def transform(request, reply):
        return (replace(reply, payload=b'\x00\x02').encode()
                + verification_push(b'\x00').encode() + verification_push(seq=1).encode())
    peer = Peer(transform=transform)
    result = execute(monkeypatch, peer, paths)
    assert result.reason == 'verification_denied' and result.verification_acks == 2
    assert [p.cmd_id for p in peer.packets] == [0x45, 0x46, 0x46]


def test_push_before_verification_is_acked_but_not_used_as_permission(monkeypatch, paths):
    class EarlyPeer(Peer):
        async def start_notify(self, characteristic, callback):
            await super().start_notify(characteristic, callback)
            self.emit(verification_push().encode())
    paths.update(timeout=0.04, request_timeout=0.02)
    peer = EarlyPeer(transform=lambda req, reply: replace(reply, payload=b'\x00\x02').encode())
    result = execute(monkeypatch, peer, paths)
    assert result.reason == 'setup_timeout' and result.verification_acks == 1
    assert [p.cmd_id for p in peer.packets] == [0x45, 0x46]
    assert not paths['credentials_path'].exists()


@pytest.mark.parametrize('changes', [dict(sender=8), dict(receiver=3), dict(version=2),
                                    dict(cmd_set=8), dict(cmd_id=0x47)])
def test_unrelated_push_cannot_grant_permission(monkeypatch, paths, changes):
    paths.update(timeout=0.04, request_timeout=0.02)
    peer = Peer(transform=lambda req, reply: replace(reply, payload=b'\x00\x02').encode()
                + verification_push(**changes).encode())
    result = execute(monkeypatch, peer, paths)
    assert result.reason == 'setup_timeout' and result.verification_acks == 0
    assert len(peer.packets) == 1 and not paths['credentials_path'].exists()


def test_area_code_hint_is_recorded_without_sending_region_commands(monkeypatch, paths):
    peer = Peer(transform=lambda req, reply: (
        replace(reply, payload=b'\x00\x02').encode() + verification_push(b'\x01\x01').encode()
        if req.cmd_id == 0x45 else reply.encode()))
    result = execute(monkeypatch, peer, paths)
    assert result.state == 'complete'
    assert [p.cmd_id for p in peer.packets] == [0x45, 0x46, 0x0C, 7, 0x0E]
    events = [json.loads(line) for line in Path(result.journal_path).read_text().splitlines()]
    assert next(e for e in events if e['event'] == 'verification_push')['area_code_requested'] is True


def test_push_queue_is_bounded_before_any_acknowledgement(monkeypatch, paths):
    peer = Peer(transform=lambda req, reply: replace(reply, payload=b'\x00\x02').encode()
                + verification_push().encode() * 65)
    result = execute(monkeypatch, peer, paths)
    assert result.reason == 'verification_notification_limit_exceeded'
    assert result.verification_acks == 0 and len(peer.packets) == 1
    assert not paths['credentials_path'].exists()


@pytest.mark.parametrize('failure', ['disconnect', 'cancel', 'stall'])
def test_waiting_verification_exits_on_disconnect_cancel_or_deadline(monkeypatch, paths, failure):
    paths.update(timeout=0.5 if failure != 'stall' else 0.05, request_timeout=0.04)
    peer = Peer()
    def transform(request, reply):
        def stop():
            if failure == 'disconnect':
                peer.is_connected = False
                peer.disconnected(peer)
        if failure == 'cancel':
            asyncio.get_running_loop().call_later(0.01, asyncio.current_task().cancel)
        else:
            asyncio.get_running_loop().call_later(0.01, stop)
        return replace(reply, payload=b'\x00\x02').encode()
    peer.transform = transform
    if failure == 'cancel':
        with pytest.raises(asyncio.CancelledError):
            execute(monkeypatch, peer, paths)
    else:
        result = execute(monkeypatch, peer, paths)
        assert result.reason == ('bluetooth_disconnected' if failure == 'disconnect' else 'setup_timeout')
    assert peer.exited and len(peer.packets) == 1 and not paths['credentials_path'].exists()


@pytest.mark.parametrize('failure', ['error', 'stall'])
@pytest.mark.parametrize('late_push', [False, True])
def test_failed_ack_never_releases_network_credentials(monkeypatch, paths, failure, late_push):
    peer = Peer()
    def transform(request, reply):
        pending = replace(reply, payload=b'\x00\x02').encode()
        if late_push:
            asyncio.get_running_loop().call_later(0.01, peer.emit, verification_push().encode())
            return pending
        return pending + verification_push().encode()
    peer.transform = transform
    original = peer.write_gatt_char
    async def write(*args, **kwargs):
        if peer.packets:
            if failure == 'error':
                raise RuntimeError(PASSWORD.decode())
            await asyncio.Event().wait()
        await original(*args, **kwargs)
    peer.write_gatt_char = write
    result = execute(monkeypatch, peer, paths)
    reason = ('bluetooth_error' if failure == 'error' else
              'verification_ack_timeout' if late_push else 'response_timeout')
    assert result.reason == reason and result.verification_acks == 0
    assert len(peer.packets) == 1 and not paths['credentials_path'].exists()


def test_push_during_fragmented_request_does_not_interleave_ack_bytes(monkeypatch, paths):
    peer = Peer(transform=lambda req, reply: replace(reply, payload=b'\x00\x02').encode()
                if req.cmd_id == 0x45 else reply.encode())
    original = peer.write_gatt_char
    async def write(*args, **kwargs):
        await original(*args, **kwargs)
        if len(peer.chunks) == 1:
            peer.emit(verification_push(seq=0x2222).encode())
    peer.write_gatt_char = write
    result = execute(monkeypatch, peer, paths)
    assert result.state == 'complete' and peer.decoder.discarded_bytes == 0
    assert [len(c) for c in peer.chunks[:4]] == [20, 20, 14, 14]
    assert [p.cmd_id for p in peer.packets] == [0x45, 0x46, 0x0C, 7, 0x0E]


def test_contradictory_early_denial_and_immediate_allow_do_not_grant_permission(monkeypatch, paths):
    peer = Peer(transform=lambda req, reply: verification_push(b'\x00').encode() + reply.encode())
    result = execute(monkeypatch, peer, paths)
    assert result.reason == 'verification_conflicting_result'
    assert result.verification_acks == 1 and not paths['credentials_path'].exists()


@pytest.mark.parametrize('payload', [b'', b'\x00', b'\x01', b'\xff'])
def test_observer_acknowledges_late_push_without_replacing_completed_verification(monkeypatch, paths, payload):
    peer = Peer(transform=lambda req, reply: verification_push(payload).encode() + reply.encode()
                if req.cmd_id == 0x0C else reply.encode())
    result = execute(monkeypatch, peer, paths)
    assert result.state == 'complete' and result.verification_acks == 1
    assert [p.cmd_id for p in peer.packets] == [0x45, 0x0C, 0x46, 7, 0x0E]


def test_global_timeout_also_bounds_connect_context(monkeypatch, paths):
    class SlowPeer(Peer):
        async def __aenter__(self):
            await asyncio.Event().wait()
    paths.update(timeout=0.02, request_timeout=0.01)
    result = execute(monkeypatch, SlowPeer(), paths)
    assert result.reason == 'setup_timeout' and result.stage == 'connect'


def test_cancellation_reaches_context_cleanup(monkeypatch, paths):
    peer = Peer(mode='cancel')
    with pytest.raises(asyncio.CancelledError):
        execute(monkeypatch, peer, paths)
    assert peer.exited and not paths['credentials_path'].exists()
    journal = paths['capture_path'].with_name('ble.jsonl.setup.jsonl').read_text()
    assert 'setup_cancelled' in journal


def test_cleanup_failure_reports_saved_credentials_without_claiming_complete(monkeypatch, paths):
    result = execute(monkeypatch, Peer(mode='cleanup_error'), paths)
    assert result.state == 'failed' and result.reason == 'bluetooth_error'
    assert result.credentials_path == str(paths['credentials_path'])
    assert setup.load_wifi_credentials(result.credentials_path).ssid == SSID


def test_missing_gatt_profile_never_writes(monkeypatch, paths):
    peer = Peer()
    peer.writer.service_uuid = 'another-service'
    result = execute(monkeypatch, peer, paths)
    assert result.reason == 'unsupported_gatt_profile' and not peer.chunks and peer.exited


def test_notification_flood_is_bounded(monkeypatch, paths):
    peer = Peer(transform=lambda req, reply: b'')
    original = peer.write_gatt_char
    async def flood(*args, **kwargs):
        await original(*args, **kwargs)
        peer.callback(peer.notifier, bytearray(513))
    peer.write_gatt_char = flood
    result = execute(monkeypatch, peer, paths)
    assert result.reason == 'notification_limit_exceeded' and len(peer.chunks) == 1


@pytest.mark.parametrize('field', ['capture_path', 'credentials_path'])
def test_existing_output_is_preserved_before_identity_or_bluetooth(monkeypatch, paths, field):
    paths[field].write_text('existing')
    with pytest.raises(ValueError, match='already exists'):
        execute(monkeypatch, Peer(), paths)
    assert paths[field].read_text() == 'existing' and not paths['identity_file'].exists()


def test_aliases_are_rejected_before_creating_any_output(monkeypatch, paths):
    paths['credentials_path'] = paths['identity_file']
    with pytest.raises(ValueError, match='distinct'):
        execute(monkeypatch, Peer(), paths)
    assert not paths['capture_path'].exists() and not paths['identity_file'].exists()


def test_wlm_store_is_not_reinterpreted_as_ble_identity(monkeypatch, paths):
    wlm = load_or_create_identity(paths['identity_file'])
    original = paths['identity_file'].read_bytes()
    with pytest.raises(ValueError, match='identity store format'):
        execute(monkeypatch, Peer(), paths)
    assert paths['identity_file'].read_bytes() == original
    assert load_or_create_identity(paths['identity_file']) == wlm


@pytest.mark.parametrize('raw', [b'[]', b'null', b'{}', b'\xff', b'{}' + b' ' * 4096])
def test_credentials_store_rejects_invalid_records(tmp_path, raw):
    path = tmp_path / 'invalid.json'
    path.write_bytes(raw)
    with pytest.raises(ValueError, match='credential store'):
        setup.load_wifi_credentials(path)


def test_cli_runs_setup_and_prints_metadata_only(monkeypatch, paths, capsys):
    peer = Peer()
    monkeypatch.setattr(setup, '_load_bleak_client', lambda: peer.factory)
    assert main(['ble-wifi-setup', ADDRESS, '--identity-file', str(paths['identity_file']),
                 '--credentials-file', str(paths['credentials_path']),
                 '--capture', str(paths['capture_path'])]) == 0
    output = capsys.readouterr().out
    assert json.loads(output)['state'] == 'complete'
    assert SSID.decode() not in output and PASSWORD.decode() not in output
