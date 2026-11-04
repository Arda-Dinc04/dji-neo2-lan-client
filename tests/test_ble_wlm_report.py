"""Optional raw WLM identity uses a synthetic GATT peer, without radio."""
import asyncio
from dataclasses import replace
import json
from pathlib import Path

import pytest

from neo2_control import ble_bootstrap as setup
from neo2_control.cli import main
from neo2_control.duml import DumlPacket, DumlSequence
from neo2_control.identity import (IDENTITY_STORE_FORMAT, load_or_create_ble_identity,
                                   load_or_create_identity)
from test_ble_bootstrap import Peer, PASSWORD, execute, paths


@pytest.fixture
def wlm_paths(paths, tmp_path):
    paths['wlm_identity_file'] = tmp_path / 'wlm-id.json'
    return paths


def test_raw_wlm_frame_matches_native_fixture_and_uses_shared_sequence(monkeypatch, wlm_paths):
    source = wlm_paths['wlm_identity_file']
    source.write_text(json.dumps({'format': IDENTITY_STORE_FORMAT, 'identification_id': 'synthetic-id'}))
    source.chmod(0o600)
    monkeypatch.setattr(setup, 'DumlSequence', lambda: DumlSequence(current=0x8121))
    peer = Peer()
    result = execute(monkeypatch, peer, wlm_paths)
    assert result.state == 'complete'
    assert result.requests_queued == result.responses_received == 4
    assert [p.cmd_id for p in peer.packets] == [0x45, 0x93, 0x0C, 7, 0x0E]
    assert [p.seq for p in peer.packets] == list(range(0x8122, 0x8127))
    fixture = json.loads((Path(__file__).parent / 'fixtures/wlm-bootstrap-native.json').read_text())
    expected = next(v['result']['native_frame_hex'] for v in fixture['producers']
                    if v['result'] is not None and v['result']['receiver_type'] == 7)
    assert peer.packets[1].encode().hex() == expected
    assert result.wlm_report['queued'] and result.wlm_report['response_status'] == 0
    assert result.wlm_report['physical_port'] == 9003
    assert result.wlm_report['identity_sha256'] == load_or_create_identity(source).fingerprint
    public = json.dumps(result.as_dict()) + Path(result.journal_path).read_text()
    assert 'synthetic-id' not in public
    assert load_or_create_ble_identity(wlm_paths['identity_file']).identification_id not in public


@pytest.mark.parametrize('status', [None, b'\0', b'\x07', b'\xff', b''])
def test_report_response_does_not_block_credentials(monkeypatch, wlm_paths, status):
    def transform(request, reply):
        if request.cmd_id == 0x93:
            return None if status is None else replace(reply, payload=status).encode()
        return reply.encode()
    peer = Peer(transform=transform)
    result = execute(monkeypatch, peer, wlm_paths)
    assert result.state == 'complete' and result.requests_queued == 4
    assert result.wlm_report['queued'] and not result.wlm_report['application_ready']
    assert result.wlm_report['response_status'] == (status[0] if status else None)
    assert result.wlm_report['malformed_responses'] == int(status == b'')
    assert len(peer.packets) == 5 and peer.exited
    assert wlm_paths['wlm_identity_file'].stat().st_mode & 0o777 == 0o600


def test_late_response_requires_route_and_sequence_and_retains_first_status(monkeypatch, wlm_paths):
    response = None
    def transform(request, reply):
        nonlocal response
        if request.cmd_id == 0x93:
            response = replace(reply, payload=b'\x07')
            return None
        if request.cmd_id == 0x0C:
            foreign = [replace(response, sender=27), replace(response, receiver=3),
                       replace(response, seq=response.seq ^ 1), replace(response, cmd_type=0)]
            return (b''.join(p.encode() for p in foreign) + response.encode()
                    + replace(response, payload=b'\0').encode() + reply.encode())
        return reply.encode()
    result = execute(monkeypatch, Peer(transform=transform), wlm_paths)
    assert result.state == 'complete' and result.wlm_report['response_status'] == 7
    assert result.responses_received == 4 and result.ignored_frames == 4


def test_pending_allow_and_its_ack_precede_raw_report(monkeypatch, wlm_paths):
    decision = DumlPacket(7, 2, 0, 0x40, 7, 0x46, b'\x01')
    peer = Peer(transform=lambda req, reply: (
        replace(reply, payload=b'\0\2').encode() + decision.encode()
        if req.cmd_id == 0x45 else reply.encode()))
    result = execute(monkeypatch, peer, wlm_paths)
    assert result.state == 'complete' and result.verification_acks == 1
    assert [p.cmd_id for p in peer.packets] == [0x45, 0x46, 0x93, 0x0C, 7, 0x0E]
    expected = peer.packets[0].seq + 1 if peer.packets[0].seq < 0xFFFE else 0x8000
    assert peer.packets[2].seq == expected and peer.packets[1].seq == 0


def test_permission_rejection_never_emits_wlm(monkeypatch, wlm_paths):
    peer = Peer(transform=lambda req, reply: replace(reply, payload=b'\0\3').encode())
    result = execute(monkeypatch, peer, wlm_paths)
    assert result.reason == 'verification_device_busy' and result.wlm_report is None
    assert [p.cmd_id for p in peer.packets] == [0x45]


@pytest.mark.parametrize('mode,reason', [('error', 'bluetooth_error'),
    ('stall', 'wlm_report_write_timeout'), ('disconnect', 'bluetooth_disconnected'),
    ('cancel', None)])
def test_raw_write_failure_stops_credentials_and_cleans_ble(monkeypatch, wlm_paths, mode, reason):
    peer = Peer()
    original = peer.write_gatt_char
    async def write(*args, **kwargs):
        if len(peer.packets) == 1:
            if mode == 'error':
                raise RuntimeError(PASSWORD.decode())
            if mode == 'stall':
                await asyncio.Event().wait()
            if mode == 'disconnect':
                peer.is_connected = False
                peer.disconnected(peer)
                return
            raise asyncio.CancelledError
        await original(*args, **kwargs)
    peer.write_gatt_char = write
    if mode == 'cancel':
        with pytest.raises(asyncio.CancelledError):
            execute(monkeypatch, peer, wlm_paths)
    else:
        result = execute(monkeypatch, peer, wlm_paths)
        assert result.reason == reason and result.stage == 'report_wlm_identity'
        assert not result.wlm_report['queued']
    assert peer.exited and len(peer.packets) == 1
    assert not wlm_paths['credentials_path'].exists()


@pytest.mark.parametrize('field', ['identity_file', 'capture_path', 'credentials_path', 'journal'])
def test_wlm_alias_is_rejected_before_ble(monkeypatch, paths, field):
    paths['wlm_identity_file'] = paths['capture_path'].with_name('ble.jsonl.setup.jsonl') if field == 'journal' else paths[field]
    with pytest.raises(ValueError, match='WLM identity must be distinct'):
        execute(monkeypatch, Peer(), paths)
    assert not paths['capture_path'].exists() and not paths['identity_file'].exists()


def test_wlm_store_cannot_be_a_hardlink_to_ble_store(monkeypatch, wlm_paths):
    source = wlm_paths['identity_file']
    load_or_create_ble_identity(source)
    wlm_paths['wlm_identity_file'].hardlink_to(source)
    before = source.read_bytes()
    with pytest.raises(ValueError, match='WLM identity must be distinct'):
        execute(monkeypatch, Peer(), wlm_paths)
    assert source.read_bytes() == before and not wlm_paths['capture_path'].exists()


def test_same_wlm_store_drives_ble_and_wifi_reports_without_merging_ble_identity(monkeypatch, wlm_paths):
    from neo2_control.session import SwudpSession
    from neo2_control.transport import ReliableTransport
    from test_session import acceptance, status
    from test_wlm_bootstrap import push
    peer = Peer()
    result = execute(monkeypatch, peer, wlm_paths)
    identity = load_or_create_identity(wlm_paths['wlm_identity_file'])
    client = SwudpSession(wlm_identity=identity,
        transport=ReliableTransport(session=0x1234, base_sequence=0x4000))
    client.start(0)
    client.receive(acceptance(), 0)
    first = next(DumlPacket.decode(d.raw[20:]) for d in client.tick(0) if d.raw[6] == 5)
    client.receive(status(ack=0x4008, payload=push()), .1)
    repeat = next(DumlPacket.decode(d.raw[20:]) for d in client.tick(.1) if d.raw[6] == 5)
    client.receive(status(ack=0x4010), .2)
    wifi = next(DumlPacket.decode(d.raw[20:]) for d in client.tick(.2) if d.raw[6] == 5)
    raw = peer.packets[1]
    assert raw.payload == first.payload == repeat.payload
    assert raw.payload[4:24] == wifi.payload[7:27]
    assert result.wlm_report['identity_sha256'] == client.wlm.snapshot()['identity_sha256']
    assert result.identity_sha256 != result.wlm_report['identity_sha256']


def test_wrong_store_format_is_preserved_before_capture_or_radio(monkeypatch, wlm_paths):
    source = wlm_paths['wlm_identity_file']
    load_or_create_ble_identity(source)
    before = source.read_bytes()
    with pytest.raises(ValueError, match='identity store format'):
        execute(monkeypatch, Peer(), wlm_paths)
    assert source.read_bytes() == before and not wlm_paths['capture_path'].exists()


def test_cli_wlm_option_outputs_no_raw_identity(monkeypatch, wlm_paths, capsys):
    peer = Peer()
    monkeypatch.setattr(setup, '_load_bleak_client', lambda: peer.factory)
    assert main(['ble-wifi-setup', wlm_paths['address'],
        '--identity-file', str(wlm_paths['identity_file']),
        '--wlm-identity-file', str(wlm_paths['wlm_identity_file']),
        '--credentials-file', str(wlm_paths['credentials_path']),
        '--capture', str(wlm_paths['capture_path'])]) == 0
    output = capsys.readouterr().out
    assert json.loads(output)['wlm_report']['queued']
    assert load_or_create_identity(wlm_paths['wlm_identity_file']).identification_id not in output
