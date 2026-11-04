"""Actual local UDP -> FFmpeg -> owned HTTPS worker, no drone/cloud traffic."""
from contextlib import closing
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
from types import SimpleNamespace
from urllib.parse import parse_qs, urlsplit

import pytest

from neo2_control import ble_bootstrap as ble, cli, observation as obs, service, video_publish as module, wifi_link as wifi
from neo2_control.ble_bootstrap import WifiCredentials
from neo2_control.controller import ControlSupervisor, DecisionJournal
from neo2_control.observation import ManagedObservation, ObservationConfig, ObservationError
from neo2_control.session import run_session
from neo2_control.video_publish import ManagedPublisher, PublishError, publication_succeeded
from test_ble_bootstrap import ADDRESS, Peer
from test_observation import until
from test_observation_service import api
from test_video_publish import TOKEN, certificate, remote, profile
from test_video_session import peer, elementary
from test_wifi_link import network, SSID, PASSWORD


def replay_uploaded_video(tmp_path, calls):
    uploads = [c for c in calls if c['method'] == 'PUT']
    assert uploads
    archive = tmp_path / 'server-received.ts'
    archive.write_bytes(b''.join(c['body'] for c in uploads))
    decoded = subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error',
        '-i', str(archive), '-f', 'framemd5', 'pipe:1'], check=True, capture_output=True, timeout=10).stdout
    frames = [line for line in decoded.splitlines() if line and not line.startswith(b'#')]
    assert len(frames) >= 10
    return uploads


def test_cli_session_publishes_real_segments_and_acknowledges_stop(tmp_path, peer, remote, certificate, capsys):
    config = profile(tmp_path, remote, certificate)
    assert cli.main(['session', '--host', '127.0.0.1', '--port', str(peer.port),
        '--duration', '1.2', '--capture', str(tmp_path / 'session.jsonl'),
        '--hls-dir', str(tmp_path / 'hls'), '--video-fps', '10',
        '--publish-config', str(config)]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['transport_accepted'] and result['video']['frames_encoded'] == 40
    delivery = result['video_publish']
    assert publication_succeeded(delivery) and not delivery['worker_alive']
    uploads = replay_uploaded_video(tmp_path, remote.calls)
    for call in uploads:
        sequence = int(urlsplit(call['path']).path.rsplit('/', 1)[-1])
        assert call['body'] == (tmp_path / 'hls' / f'segment-{sequence:08d}.ts').read_bytes()
        assert 0 < float(parse_qs(urlsplit(call['path']).query)['duration'][0]) <= 5
    assert remote.calls[-1]['path'].endswith('/stop')
    public = json.dumps(result) + Path(result['journal_path']).read_text()
    assert TOKEN not in public and str(config) not in public
    assert remote.url not in public and not result['application_ready']


def test_http_close_reaps_publication_before_finishing_owner(tmp_path, peer, remote, certificate):
    config = profile(tmp_path, remote, certificate)
    with closing(DecisionJournal(tmp_path / 'controller.jsonl')) as journal:
        controller = ControlSupervisor(journal)
        manager = ManagedObservation(ObservationConfig('testwifi0', maximum_duration=10,
            record_hls=True, video_nominal_fps=10, publish_config_file=config),
            tmp_path / 'captures', session_runner=peer.runner, route_guard=lambda _: None,
            event_sink=controller.observation_event)
        controller.attach_observation_source(manager.snapshot)
        try:
            state = service.ServiceState(tmp_path, tmp_path, True, controller=controller, observation=manager)
            with api(state) as request:
                assert request('/v1/drone/session/open', {'publish_config': '/tmp/foreign'})[0] == 400
                assert request('/v1/drone/session/open', {'duration': 5})[0] == 202
                until(lambda: manager.snapshot()['session'].get('video_publish', {}).get('segments_uploaded', 0) > 0, timeout=5)
                assert request('/v1/drone/session/close', {})[0] == 202
                until(lambda: not manager.busy, timeout=15)
                final = request('/v1/drone/status')[1]
            delivery = final['session']['video_publish']
            assert publication_succeeded(delivery) and not manager.owns_link
            assert delivery['state'] == 'offline'
            assert TOKEN not in json.dumps(final) + journal.path.read_text()
        finally:
            manager.shutdown()


def test_cloud_auth_failure_keeps_local_udp_ack_and_decode_working(tmp_path, peer, remote, certificate):
    config = profile(tmp_path, remote, certificate)
    remote.response = lambda *args: (401, {'echoed_token': TOKEN}, {})
    result = peer.runner(capture_path=tmp_path / 'session.jsonl', duration=1,
        hls_directory=tmp_path / 'hls', video_nominal_fps=10, publish_config_file=config)
    assert result.state == 'closed' and result.transport_accepted
    assert result.video['frames_encoded'] == 40 and result.video['cleanup_verified']
    assert result.video_publish['error'] == 'publish_http_401'
    assert result.video_publish['cleanup_verified'] and not result.video_publish['worker_alive']
    assert not publication_succeeded(result.video_publish)
    assert any(raw[6] in (4, 6) for raw in peer.requests)
    assert TOKEN not in json.dumps(result.as_dict())


def test_continuous_ble_wifi_publisher_lifetime_and_sigterm(tmp_path, peer, remote, certificate, network, monkeypatch):
    config = profile(tmp_path, remote, certificate)
    gatt = Peer()
    monkeypatch.setattr(ble, '_load_bleak_client', lambda: gatt.factory)
    def runner(**options):
        assert network.connected and gatt.is_connected
        result = peer.runner(**options)
        assert result.video_publish['cleanup_verified'] and result.video['cleanup_verified']
        assert network.connected and gatt.is_connected
        return result
    monkeypatch.setattr(obs, 'run_session', runner)
    original = network.run
    def command(*args, **kwargs):
        if 'down' in args:
            assert gatt.is_connected
            assert remote.calls[-1]['path'].endswith('/stop')
        return original(*args, **kwargs)
    monkeypatch.setattr(wifi, '_run', command)
    real_sleep = cli.time.sleep
    signalled = []
    def wait(seconds):
        assert threading.current_thread() is threading.main_thread()
        if any(c['method'] == 'PUT' for c in remote.calls) and not signalled:
            signalled.append(True)
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        real_sleep(seconds)
    # Patching the shared time module also interrupted publisher cleanup in
    # its owning worker, delivering our synthetic SIGTERM in the wrong thread.
    monkeypatch.setattr(cli, 'time', SimpleNamespace(sleep=wait))
    assert cli.main(['connect-session', ADDRESS, '--interface', 'wlanusb0',
        '--ble-identity-file', str(tmp_path / 'ble.json'),
        '--wlm-identity-file', str(tmp_path / 'wlm.json'),
        '--capture-dir', str(tmp_path / 'captures'), '--duration', '10',
        '--hls', '--video-fps', '10', '--publish-config', str(config)]) == 130
    assert gatt.exited and not network.connected
    assert signalled and remote.calls[-1]['path'].endswith('/stop')


@pytest.mark.parametrize('options', [
    {'observe_publish_config': '/tmp/config'},
    {'observe_interface': 'test0', 'allow_active_syn': True, 'observe_publish_config': '/tmp/config'},
    {'observe_interface': 'test0', 'allow_active_syn': True, 'observe_hls': True,
     'observe_publish_config': '/tmp/config', 'drone_mode': 'simulate'},
])
def test_invalid_service_config_precedes_files(tmp_path, options):
    with pytest.raises(ValueError):
        service.serve(listen='127.0.0.1', port=0, token='x' * 32,
            capture_dir=tmp_path / 'captures', report_dir=tmp_path / 'reports', **options)
    assert not list(tmp_path.iterdir())


def test_invalid_private_profile_fails_before_any_ble_or_nm(tmp_path, network, monkeypatch):
    config = tmp_path / 'public-config.json'
    config.write_text(json.dumps({'format': module.FORMAT, 'server': 'https://unused.invalid/api',
        'agent_id': 'pc', 'token': TOKEN}))  # Deliberately world-readable.
    monkeypatch.setattr(ble, '_load_bleak_client', lambda: pytest.fail('Bluetooth loaded'))
    manager = ManagedObservation(ObservationConfig('wlanusb0', record_hls=True,
        ble_address=ADDRESS, ble_identity_file=tmp_path / 'ble.json',
        wlm_identity_file=tmp_path / 'wlm.json', publish_config_file=config), tmp_path / 'captures')
    manager.open()
    until(lambda: not manager.busy)
    assert manager.snapshot()['observation']['error'] == 'publish_config_invalid_or_unavailable'
    assert not network.calls and not (tmp_path / 'captures').exists()
    manager.shutdown()


def test_wifi_dry_plan_reads_config_but_creates_no_worker_profile_or_identity(tmp_path, remote, certificate, network, monkeypatch, capsys):
    source = tmp_path / 'credentials.json'
    WifiCredentials(bytes.fromhex('020000000002'), SSID, PASSWORD).save_new(source)
    config = profile(tmp_path, remote, certificate)
    monkeypatch.setattr(module.ManagedPublisher, 'start', lambda *args: pytest.fail('publisher started'))
    assert cli.main(['wifi-session', '--credentials-file', str(source),
        '--interface', 'wlanusb0', '--capture', str(tmp_path / 'session.jsonl'),
        '--hls-dir', str(tmp_path / 'hls'), '--publish-config', str(config),
        '--wlm-identity-file', str(tmp_path / 'wlm.json')]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['state'] == 'planned' and result['video_publish_enabled']
    assert not remote.calls and not network.directory.exists()
    assert not (tmp_path / 'hls').exists() and not (tmp_path / 'wlm.json').exists()
    assert TOKEN not in json.dumps(result)


@pytest.mark.parametrize('uncertain', [False, True])
def test_publication_cleanup_metadata_survives_broken_parent_journal(tmp_path, peer, remote, certificate, monkeypatch, uncertain):
    config = profile(tmp_path, remote, certificate)
    if uncertain:
        original = ManagedPublisher.snapshot
        def snapshot(publisher, *args, **kwargs):
            result = original(publisher, *args, **kwargs)
            if publisher.closed:
                result['cleanup_verified'] = False
            return result
        monkeypatch.setattr(ManagedPublisher, 'snapshot', snapshot)
    def audit(name, **detail):
        if name == 'publish_cleanup_result':
            raise OSError('full parent journal')
    manager = ManagedObservation(ObservationConfig('test0', record_hls=True,
        video_nominal_fps=10, publish_config_file=config), tmp_path / 'captures',
        session_runner=peer.runner, route_guard=lambda _: None, event_sink=audit)
    manager.open({'duration': 1})
    until(lambda: not manager.busy, timeout=15)
    assert manager.publish_cleanup_pending is uncertain
    final = manager.snapshot()
    assert final['session']['video_publish']['cleanup_verified'] is not uncertain
    if uncertain:
        assert manager.owns_link and final['observation']['error'] == 'publish_cleanup_unconfirmed'
        with pytest.raises(ObservationError, match='publish_cleanup_unconfirmed'):
            manager.open()
        with pytest.raises(ObservationError, match='publish_cleanup_unconfirmed'):
            manager.shutdown()
    else:
        assert not manager.owns_link
        manager.shutdown()


def test_token_and_ca_paths_cannot_be_decoder_outputs(tmp_path, remote, certificate):
    directory = tmp_path / 'hls'
    directory.mkdir()
    config = profile(directory, remote, certificate)
    with pytest.raises(PublishError):
        run_session(capture_path=tmp_path / 'capture.jsonl', hls_directory=directory,
            publish_config_file=config)
    assert not (tmp_path / 'capture.jsonl').exists()
    assert not remote.calls
