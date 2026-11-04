"""Temporary Wi-Fi and service ownership with WLM selected only locally."""

import json

import pytest

from neo2_control import observation as observation_module, service, wifi_link as wifi
from neo2_control.ble_bootstrap import WifiCredentials
from neo2_control.cli import main
from neo2_control.identity import load_or_create_ble_identity, load_or_create_identity
from neo2_control.observation import ManagedObservation, ObservationConfig
from test_observation import until
from test_observation_service import api
from test_wifi_link import SSID, PASSWORD, network
from test_wlm_bootstrap import peer


def credentials(path):
    WifiCredentials(bytes.fromhex('020000000002'), SSID, PASSWORD).save_new(path)
    return path


def test_cli_wifi_plan_selects_wlm_without_creating_identity_or_profile(network, tmp_path, capsys):
    source = credentials(tmp_path / 'wifi.json')
    identity = tmp_path / 'wlm.json'
    assert main(['wifi-session', '--credentials-file', str(source), '--interface', 'wlanusb0',
        '--capture', str(tmp_path / 'capture.jsonl'), '--wlm-identity-file', str(identity)]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['state'] == 'planned' and result['wlm_bootstrap_enabled']
    assert not identity.exists() and not network.directory.exists()
    assert not any('load' in c or 'up' in c for c in network.calls)


def test_temporary_wifi_owns_profile_through_wlm_reports_and_verified_cleanup(network, tmp_path, peer, monkeypatch):
    source, identity = credentials(tmp_path / 'wifi.json'), tmp_path / 'wlm.json'
    def runner(**options):
        assert network.connected
        return peer.runner(**options)
    monkeypatch.setattr(wifi, 'run_session', runner)
    result = wifi.run_wifi_session(credentials_file=source, interface='wlanusb0',
        capture_path=tmp_path / 'capture.jsonl', duration=.3, apply=True, wlm_identity_file=identity)
    assert result['state'] == 'closed' and result['wifi_cleanup_verified']
    assert result['session']['wlm_bootstrap']['reports_transport_acknowledged'] == 3
    assert not network.connected and not list(network.directory.iterdir())
    assert load_or_create_identity(identity).identification_id not in json.dumps(result)
    assert not result['application_ready']


@pytest.mark.parametrize('apply', [False, True])
def test_wrong_identity_store_precedes_any_wifi_preparation_or_activation(network, tmp_path, apply):
    source, identity = credentials(tmp_path / 'wifi.json'), tmp_path / 'ble.json'
    load_or_create_ble_identity(identity)
    before = identity.read_bytes()
    with pytest.raises(ValueError, match='identity store format'):
        wifi.run_wifi_session(credentials_file=source, interface='wlanusb0',
            capture_path=tmp_path / 'capture.jsonl', apply=apply, wlm_identity_file=identity)
    assert not network.calls and not network.directory.exists()
    assert identity.read_bytes() == before


@pytest.mark.parametrize('hardlink', [False, True])
def test_wifi_credentials_cannot_be_reused_as_wlm_store(network, tmp_path, hardlink):
    source = credentials(tmp_path / 'wifi.json')
    identity = tmp_path / 'identity.json' if hardlink else source
    if hardlink:
        identity.hardlink_to(source)
    before = source.read_bytes()
    with pytest.raises(ValueError, match='WLM identity must be distinct'):
        wifi.run_wifi_session(credentials_file=source, interface='wlanusb0',
            capture_path=tmp_path / 'capture.jsonl', apply=True, wlm_identity_file=identity)
    assert not network.calls and source.read_bytes() == before


def test_managed_invalid_wlm_store_fails_before_network_or_udp(network, tmp_path):
    source, identity = credentials(tmp_path / 'wifi.json'), tmp_path / 'ble.json'
    load_or_create_ble_identity(identity)
    manager = ManagedObservation(ObservationConfig('wlanusb0', credentials_file=source,
        wlm_identity_file=identity), tmp_path / 'captures',
        session_runner=lambda **kw: pytest.fail('UDP opened'))
    try:
        manager.open()
        until(lambda: not manager.busy)
        snapshot = manager.snapshot()
        assert snapshot['state'] == 'failed'
        assert snapshot['observation']['error'] == 'wlm_identity_invalid_or_unavailable'
        assert not network.calls and not network.directory.exists()
    finally:
        manager.shutdown()


def test_serve_selects_identity_locally_http_runs_full_profile_wlm_and_close(network, tmp_path, peer, monkeypatch):
    source, identity = credentials(tmp_path / 'wifi.json'), tmp_path / 'wlm.json'
    monkeypatch.setattr(observation_module, 'run_session', peer.runner)
    captured = []
    original = service.build_handler
    def handler(state, token):
        captured.append(state)
        return original(state, token)
    monkeypatch.setattr(service, 'build_handler', handler)
    class Server:
        server_port = 0
        def __init__(self, *args): pass
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def serve_forever(self):
            with api(captured[0]) as request:
                assert request('/v1/drone/session/open', {'wlm_identity_file': '/tmp/remote'})[0] == 400
                assert request('/v1/drone/session/open', {'duration': 3})[0] == 202
                until(lambda: request('/v1/drone/status')[1]['session'].get('wlm_bootstrap', {}).get('reports_transport_acknowledged') == 3)
                live = request('/v1/drone/status')[1]
                assert live['observation']['wlm_bootstrap_enabled'] and network.connected
                assert str(identity) not in json.dumps(live)
                assert request('/v1/drone/session/close', {})[0] == 202
                until(lambda: not captured[0].observation.busy)
            raise KeyboardInterrupt
    monkeypatch.setattr(service, 'ThreadingHTTPServer', Server)
    with pytest.raises(KeyboardInterrupt):
        service.serve(listen='127.0.0.1', port=0, token='x' * 32,
            capture_dir=tmp_path / 'captures', report_dir=tmp_path / 'reports',
            allow_active_syn=True, observe_interface='wlanusb0', observe_credentials_file=source,
            observe_wlm_identity_file=identity)
    final = captured[0].observation.snapshot()
    assert final['session']['wlm_bootstrap']['reports_transport_acknowledged'] == 3
    assert final['observation']['wifi']['cleanup_verified']
    assert not network.connected and not list(network.directory.iterdir())


@pytest.mark.parametrize('options', [
    {'observe_wlm_identity_file': '/tmp/local.json'},
    {'observe_interface': 'test0', 'observe_wlm_identity_file': '/tmp/local.json'},
    {'observe_interface': 'test0', 'observe_wlm_identity_file': '/tmp/local.json',
     'allow_active_syn': True, 'drone_mode': 'simulate'},
])
def test_invalid_service_wlm_configuration_precedes_files_threads_and_sockets(tmp_path, options):
    with pytest.raises(ValueError):
        service.serve(listen='127.0.0.1', port=0, token='x' * 32,
            capture_dir=tmp_path / 'captures', report_dir=tmp_path / 'reports', **options)
    assert not list(tmp_path.iterdir())


def test_serve_cli_forwards_only_local_wlm_selection(tmp_path, monkeypatch):
    token = tmp_path / 'token'
    token.write_text('x' * 32)
    import neo2_control.cli as cli
    calls = []
    monkeypatch.setattr(cli, 'serve', lambda **options: calls.append(options))
    identity = tmp_path / 'wlm.json'
    assert main(['serve', '--token-file', str(token), '--allow-active-syn',
        '--observe-interface', 'wlanusb0', '--observe-wlm-identity-file', str(identity)]) == 0
    assert calls[0]['observe_wlm_identity_file'] == identity
    assert not identity.exists()
