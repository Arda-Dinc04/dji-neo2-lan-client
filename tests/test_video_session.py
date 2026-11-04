"""Real codec decoding after SWUDP reordering, owned by CLI/service sessions."""
from contextlib import contextmanager
import json
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading

import pytest

from neo2_control import cli, observation as obs, service, video as video_module, wifi_link as wifi
from neo2_control.controller import ControlSupervisor, DecisionJournal
from neo2_control.identity import load_or_create_identity
from neo2_control.observation import ManagedObservation, ObservationConfig, ObservationError
from neo2_control.session import run_session
from neo2_control.transport import _build_datagram, describe_transport
from test_observation import until
from test_observation_service import api
from test_video import fake_decoder
from test_wifi_link import network, SSID, PASSWORD
from neo2_control.ble_bootstrap import WifiCredentials


@pytest.fixture(scope='module')
def elementary():
    return subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error',
        '-f', 'lavfi', '-i', 'testsrc2=size=160x96:rate=10', '-t', '4',
        '-c:v', 'libx264', '-preset', 'ultrafast', '-g', '10',
        '-f', 'h264', 'pipe:1'], check=True, capture_output=True, timeout=10).stdout


class VideoPeer:
    """One actual UDP endpoint, fragmented Annex B, synthetic video only."""
    def __init__(self, elementary):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 0))
        self.socket.settimeout(.025)
        self.port = self.socket.getsockname()[1]
        self.stop = threading.Event()
        self.blocks = [elementary[i:i+1452] for i in range(0, len(elementary), 1452)]
        self.requests, self.errors = [], []
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        from test_session import acceptance, status
        session = anchor = acknowledged = None
        next_block = 0
        video_frontier = None
        try:
            while not self.stop.is_set():
                try:
                    raw, peer = self.socket.recvfrom(65535)
                except socket.timeout:
                    continue
                self.requests.append(raw)
                if raw[6] == 0:
                    session = int.from_bytes(raw[2:4], 'little')
                    anchor = acknowledged = int.from_bytes(raw[8:10], 'little')
                    video_frontier = anchor
                    self.socket.sendto(acceptance(session), peer)
                elif session is not None and raw[6] in (4, 6):
                    decoded, _ = describe_transport(raw)
                    video_frontier = decoded['video_window_start']
                if session is None:
                    continue
                if raw[6] == 5:
                    acknowledged = int.from_bytes(raw[4:6], 'little')
                # Short bursts leave room in the actual kernel socket and
                # exercise receive-frontier ACKs across multiple windows.
                if next_block < len(self.blocks):
                    end_block = min(next_block + 12, len(self.blocks))
                    start = (anchor + next_block * 8) & 0xFFFF
                    end = (anchor + end_block * 8) & 0xFFFF
                    order = list(range(next_block, end_block))
                    if len(order) > 2:
                        order[0], order[1] = order[1], order[0]
                    for i in order:
                        payload = struct.pack('<HHHHI', start, end, 0, 0, 0x101) + self.blocks[i]
                        self.socket.sendto(_build_datagram(session, 2, payload,
                            send_sequence=(anchor + (i + 1) * 8) & 0xFFFF), peer)
                    next_block = end_block
                packet = bytearray(status(session, ack=acknowledged, anchor=anchor))
                for offset in (8, 10):
                    packet[offset:offset+2] = video_frontier.to_bytes(2, 'little')
                # Header XOR covers only bytes 0..6, window words do not alter it.
                self.socket.sendto(bytes(packet), peer)
        except BaseException as error:
            self.errors.append(error)

    def runner(self, **options):
        options.update(host='127.0.0.1', port=self.port, interface=None)
        return run_session(**options)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        self.socket.close()
        assert not self.thread.is_alive() and not self.errors


@pytest.fixture
def peer(elementary):
    # Four seconds limit wall time while retaining enough real NALs to probe.
    instance = VideoPeer(elementary)
    yield instance
    instance.close()


def decode_playlist(directory):
    playlist = Path(directory) / 'index.m3u8'
    assert playlist.is_file()
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-i', str(playlist),
        '-f', 'null', '-'], check=True, capture_output=True, timeout=10)


def test_cli_session_decodes_reordered_udp_payloads_and_reaps_ffmpeg(tmp_path, peer, capsys):
    assert cli.main(['session', '--host', '127.0.0.1', '--port', str(peer.port),
        '--duration', '1', '--capture', str(tmp_path / 'session.jsonl'),
        '--video-output', str(tmp_path / 'raw.h264'), '--hls-dir', str(tmp_path / 'hls'),
        '--video-fps', '10']) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['video']['frames_encoded'] == 40
    assert result['video']['cleanup_verified'] and not result['video']['worker_alive']
    assert result['video']['error'] is None and not result['application_ready']
    assert (tmp_path / 'raw.h264').read_bytes() == b''.join(peer.blocks)
    decode_playlist(tmp_path / 'hls')


def test_http_local_hls_option_cancels_and_preserves_playable_artifacts(tmp_path, peer):
    journal = DecisionJournal(tmp_path / 'controller.jsonl')
    controller = ControlSupervisor(journal)
    manager = ManagedObservation(ObservationConfig('testwifi0', maximum_duration=5,
        record_hls=True, video_nominal_fps=10), tmp_path / 'captures',
        session_runner=peer.runner, route_guard=lambda interface: None,
        event_sink=controller.observation_event)
    controller.attach_observation_source(manager.snapshot)
    try:
        state = service.ServiceState(tmp_path, tmp_path, True, controller=controller, observation=manager)
        with api(state) as request:
            for extra in ({'hls_directory': '/tmp/foreign'}, {'video_codec': 'hevc'}, {'video_fps': 10}):
                assert request('/v1/drone/session/open', extra)[0] == 400
            assert request('/v1/drone/session/open', {'duration': 4})[0] == 202
            until(lambda: manager.snapshot()['session'].get('video', {}).get('frames_encoded', 0) > 20, timeout=3)
            live = manager.snapshot()['session']['video']
            assert live['enabled'] and live['state'] in ('live', 'connecting')
            assert request('/v1/drone/session/close', {})[0] == 202
            until(lambda: not manager.busy, timeout=8)
        final = manager.snapshot()
        assert final['state'] == 'closed'
        assert final['session']['video']['state'] == 'offline'
        assert final['session']['video']['cleanup_verified']
        decode_playlist(final['observation']['hls_directory'])
    finally:
        manager.shutdown()
        journal.close()


def test_decoder_failure_preserves_udp_ack_and_transport_session(tmp_path, peer, monkeypatch):
    fake_decoder(monkeypatch, tmp_path, 'raise SystemExit(7)\n')
    result = peer.runner(capture_path=tmp_path / 'session.jsonl', duration=.5,
        hls_directory=tmp_path / 'hls')
    assert result.state == 'closed' and result.transport_accepted
    assert result.video_bytes_delivered == sum(map(len, peer.blocks))
    assert result.video['state'] == 'failed' and result.video['cleanup_verified']
    expected = (result.base_sequence + len(peer.blocks) * 8) & 0xFFFF
    assert any(describe_transport(raw)[0]['video_window_start'] == expected
        for raw in peer.requests if raw[6] in (4, 6))


def test_wifi_owner_finishes_decoder_before_network_cleanup(tmp_path, peer, network, monkeypatch):
    source = tmp_path / 'credentials.json'
    WifiCredentials(bytes.fromhex('020000000002'), SSID, PASSWORD).save_new(source)
    monkeypatch.setattr(wifi, 'run_session', peer.runner)
    original = network.run
    def command(*args, **options):
        if 'down' in args:
            assert not any(t.name == 'neo2-video' for t in threading.enumerate())
        return original(*args, **options)
    monkeypatch.setattr(wifi, '_run', command)
    result = wifi.run_wifi_session(credentials_file=source, interface='wlanusb0',
        capture_path=tmp_path / 'session.jsonl', duration=1, apply=True,
        hls_directory=tmp_path / 'hls', video_nominal_fps=10)
    assert result['state'] == 'closed' and result['wifi_cleanup_verified']
    assert result['session']['video']['frames_encoded'] == 40
    assert result['session']['video']['cleanup_verified']
    decode_playlist(tmp_path / 'hls')


@pytest.mark.parametrize('target', ['capture', 'identity', 'credentials'])
def test_hls_directory_cannot_contain_other_session_paths(tmp_path, target):
    hls = tmp_path / 'hls'
    options = dict(capture_path=tmp_path / 'session.jsonl', hls_directory=hls)
    if target == 'capture':
        options['capture_path'] = hls / 'session.jsonl'
    elif target == 'identity':
        options['wlm_identity_file'] = hls / 'segment-00000000.ts'
    else:
        options['video_path'] = hls / 'raw.h264'
    with pytest.raises(ValueError, match='must be separate'):
        run_session(**options)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('options', [
    {'observe_hls': True}, {'observe_video_fps': 10}, {'observe_video_codec': 'hevc'},
    {'observe_interface': 'test0', 'allow_active_syn': True, 'observe_video_fps': 10},
    {'observe_interface': 'test0', 'allow_active_syn': True, 'drone_mode': 'simulate', 'observe_hls': True},
])
def test_invalid_service_video_configuration_precedes_effects(tmp_path, options):
    with pytest.raises(ValueError):
        service.serve(listen='127.0.0.1', port=0, token='x' * 32,
            capture_dir=tmp_path / 'captures', report_dir=tmp_path / 'reports', **options)
    assert not list(tmp_path.iterdir())


def test_video_cleanup_fault_latches_managed_owner(tmp_path):
    from neo2_control.video import VideoError
    def runner(**options):
        raise VideoError('video_cleanup_unconfirmed', cleanup_verified=False)
    manager = ManagedObservation(ObservationConfig('test0', record_hls=True), tmp_path / 'captures',
        session_runner=runner, route_guard=lambda interface: None)
    manager.open()
    until(lambda: not manager.busy)
    assert manager.snapshot()['observation']['error'] == 'video_cleanup_unconfirmed'
    assert manager.owns_link and manager.video_cleanup_pending
    with pytest.raises(ObservationError, match='video_cleanup_unconfirmed'):
        manager.open()
    with pytest.raises(ObservationError, match='video_cleanup_unconfirmed'):
        manager.shutdown()


def test_continuous_ble_wifi_owner_keeps_links_until_decoder_is_reaped(tmp_path, network, peer, monkeypatch, capsys):
    from test_ble_bootstrap import ADDRESS, Peer
    from neo2_control import ble_bootstrap as ble
    gatt = Peer()
    monkeypatch.setattr(ble, '_load_bleak_client', lambda: gatt.factory)
    def runner(**options):
        assert gatt.is_connected and network.connected
        result = peer.runner(**options)
        assert gatt.is_connected and network.connected and result.video['cleanup_verified']
        return result
    monkeypatch.setattr(obs, 'run_session', runner)
    original_exit = gatt.__class__.__aexit__
    async def leave(self, *args):
        assert not network.connected
        assert not any(t.name == 'neo2-video' for t in threading.enumerate())
        return await original_exit(self, *args)
    gatt.__class__ = type('OwnedGatt', (Peer,), {'__aexit__': leave})
    assert cli.main(['connect-session', ADDRESS, '--interface', 'wlanusb0',
        '--ble-identity-file', str(tmp_path / 'ble.json'),
        '--wlm-identity-file', str(tmp_path / 'wlm.json'),
        '--capture-dir', str(tmp_path / 'captures'),
        '--duration', '1', '--hls', '--video-fps', '10']) == 0
    final = json.loads(capsys.readouterr().out)
    assert final['session']['video']['frames_encoded'] == 40
    assert final['session']['video']['cleanup_verified'] and gatt.exited
    assert final['observation']['wifi']['cleanup_verified']
    assert final['observation']['ble']['cleanup_verified']
    decode_playlist(final['observation']['hls_directory'])


def test_serve_local_hls_configuration_and_sigterm_own_decoder(tmp_path, peer, monkeypatch):
    monkeypatch.setattr(obs, 'run_session', peer.runner)
    states = []
    original = service.build_handler
    def handler(state, token):
        states.append(state)
        return original(state, token)
    monkeypatch.setattr(service, 'build_handler', handler)
    monkeypatch.setattr(obs, 'require_dedicated_drone_route', lambda *a, **k: None)
    class Server:
        server_port = 0
        def __init__(self, *args): pass
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def serve_forever(self):
            with api(states[0]) as request:
                assert request('/v1/drone/session/open', {'duration': 4})[0] == 202
                until(lambda: states[0].observation.snapshot()['session'].get(
                    'video', {}).get('frames_encoded', 0) > 20, timeout=3)
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
    monkeypatch.setattr(service, 'ThreadingHTTPServer', Server)
    with pytest.raises(KeyboardInterrupt):
        service.serve(listen='127.0.0.1', port=0, token='x' * 32,
            capture_dir=tmp_path / 'captures', report_dir=tmp_path / 'reports',
            allow_active_syn=True, observe_interface='testwifi0',
            observe_hls=True, observe_video_fps=10)
    final = states[0].observation.snapshot()
    assert final['session']['video']['cleanup_verified'] and not states[0].observation.busy
    assert not any(t.name == 'neo2-video' for t in threading.enumerate())


def test_hls_missing_dependency_is_refused_before_ble_or_wifi(tmp_path, network, monkeypatch):
    from test_ble_bootstrap import ADDRESS
    from neo2_control import ble_bootstrap as ble
    monkeypatch.setattr(video_module.shutil, 'which', lambda name: None)
    monkeypatch.setattr(ble, '_load_bleak_client', lambda: pytest.fail('Bluetooth loaded'))
    manager = ManagedObservation(ObservationConfig('wlanusb0', ble_address=ADDRESS,
        ble_identity_file=tmp_path / 'ble.json', wlm_identity_file=tmp_path / 'wlm.json',
        record_hls=True), tmp_path / 'captures')
    manager.open()
    until(lambda: not manager.busy)
    assert manager.snapshot()['observation']['error'] == 'video_requires_ffmpeg_and_prlimit'
    assert not network.calls and not list(tmp_path.iterdir())
    manager.shutdown()


def test_wifi_dry_plan_has_no_decoder_profile_or_identity_side_effects(tmp_path, network, monkeypatch, capsys):
    source = tmp_path / 'credentials.json'
    WifiCredentials(bytes.fromhex('020000000002'), SSID, PASSWORD).save_new(source)
    def forbidden(*args, **kwargs):
        pytest.fail('Decoder process started in dry plan')
    monkeypatch.setattr(video_module.subprocess, 'Popen', forbidden)
    assert cli.main(['wifi-session', '--credentials-file', str(source),
        '--interface', 'wlanusb0', '--capture', str(tmp_path / 'session.jsonl'),
        '--wlm-identity-file', str(tmp_path / 'wlm.json'),
        '--hls-dir', str(tmp_path / 'hls'), '--video-fps', '10']) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['state'] == 'planned' and result['hls_enabled']
    assert list(tmp_path.iterdir()) == [source]
    assert not network.directory.exists()


def test_transport_acceptance_without_video_is_not_successful_hls(tmp_path, capsys):
    peer = VideoPeer(b'')
    try:
        assert cli.main(['session', '--host', '127.0.0.1', '--port', str(peer.port),
            '--duration', '.15', '--capture', str(tmp_path / 'session.jsonl'),
            '--hls-dir', str(tmp_path / 'hls')]) == 2
        result = json.loads(capsys.readouterr().out)
        assert result['transport_accepted'] and result['video']['frames_encoded'] == 0
        assert result['video']['error'] is None and result['video']['cleanup_verified']
        assert not result['application_ready']
    finally:
        peer.close()


@pytest.mark.parametrize('uncertain', [False, True])
def test_decoder_cleanup_metadata_survives_parent_audit_failure(tmp_path, peer, monkeypatch, uncertain):
    if uncertain:
        original = video_module.HlsVideo.snapshot
        def snapshot(consumer, *args, **kwargs):
            result = original(consumer, *args, **kwargs)
            if consumer.closed:
                result['cleanup_verified'] = False
            return result
        monkeypatch.setattr(video_module.HlsVideo, 'snapshot', snapshot)
    def audit(name, **details):
        if name == 'video_cleanup_result':
            raise OSError('full parent journal')
    manager = ManagedObservation(ObservationConfig('testwifi0', record_hls=True, video_nominal_fps=10),
        tmp_path / 'captures', session_runner=peer.runner, route_guard=lambda interface: None,
        event_sink=audit)
    manager.open({'duration': .5})
    until(lambda: not manager.busy, timeout=8)
    result = manager.snapshot()
    assert manager.video_cleanup_pending is uncertain
    assert result['session']['video']['cleanup_verified'] is not uncertain
    assert not any(t.name == 'neo2-video' for t in threading.enumerate())
    if uncertain:
        assert manager.owns_link
        assert result['observation']['error'] == 'video_cleanup_unconfirmed'
        with pytest.raises(ObservationError, match='video_cleanup_unconfirmed'):
            manager.shutdown()
    else:
        assert not manager.owns_link and result['state'] == 'failed'
        manager.shutdown()


def test_startup_failure_still_reports_verified_decoder_cleanup(tmp_path, monkeypatch):
    snapshots = []
    def unavailable(*args, **kwargs):
        raise OSError('backend launch failed')
    monkeypatch.setattr(video_module.subprocess, 'Popen', unavailable)
    with pytest.raises(OSError):
        run_session(capture_path=tmp_path / 'session.jsonl', hls_directory=tmp_path / 'hls',
            video_cleanup_sink=snapshots.append)
    assert len(snapshots) == 1 and snapshots[0]['cleanup_verified']
    assert snapshots[0]['error'] == 'video_decoder_start_failed'
