"""Continuous BLE -> owned Wi-Fi -> real loopback SWUDP, without radio."""
from contextlib import contextmanager, closing
from dataclasses import replace
import asyncio
import json
from pathlib import Path
import signal
import threading

import pytest

from neo2_control import ble_bootstrap as ble, cli, observation as obs, service, wifi_link as wifi
from neo2_control.duml import DumlPacket
from neo2_control.controller import ControlSupervisor, DecisionJournal
from neo2_control.identity import load_or_create_ble_identity, load_or_create_identity
from neo2_control.observation import ManagedObservation, ObservationConfig, ObservationError
from test_ble_bootstrap import ADDRESS, PASSWORD, SSID, Peer
from test_observation import until
from test_observation_service import api
from test_wifi_link import network
from test_wlm_bootstrap import peer


def config(tmp_path, **changes):
    options = dict(interface='wlanusb0', maximum_duration=5,
        ble_address=ADDRESS, ble_identity_file=tmp_path / 'ble-id.json',
        wlm_identity_file=tmp_path / 'wlm-id.json', ble_setup_timeout=.5)
    return ObservationConfig(**(options | changes))


@pytest.fixture
def gatt(monkeypatch, network):
    class Gatt(Peer):
        async def __aexit__(self, *args):
            assert not network.connected
            assert not list(network.directory.glob('*.nmconnection'))
            await super().__aexit__(*args)
    device = Gatt()
    monkeypatch.setattr(ble, '_load_bleak_client', lambda: device.factory)
    return device


@contextmanager
def owner(tmp_path, network, peer, gatt, **changes):
    def runner(**options):
        assert network.connected and gatt.is_connected
        return peer.runner(**options)
    manager = ManagedObservation(config(tmp_path, **changes), tmp_path / 'captures', session_runner=runner)
    try:
        yield manager
    finally:
        try:
            manager.shutdown()
        except ObservationError:
            if not (manager.ble_cleanup_pending or manager.wifi_cleanup_pending):
                raise


def test_http_continuous_owner_preserves_one_identity_and_cleans_in_order(tmp_path, network, peer, gatt):
    with closing(DecisionJournal(tmp_path / 'controller.jsonl')) as journal, owner(tmp_path, network, peer, gatt, record_video=True) as manager:
        controller = ControlSupervisor(journal)
        controller.attach_observation_source(manager.snapshot)
        manager.event_sink = controller.observation_event
        state = service.ServiceState(tmp_path, tmp_path, True, controller=controller, observation=manager)
        with api(state) as request:
            for parameters in ({'ble_address': ADDRESS}, {'ble_identity_file': '/tmp/x'},
                               {'credentials_file': '/tmp/x'}, {'wlm_identity_file': '/tmp/x'}):
                assert request('/v1/drone/session/open', parameters)[0] == 400
            assert request('/v1/drone/session/open', {'duration': 3})[0] == 202
            until(lambda: manager.snapshot()['session'].get('wlm_bootstrap', {}).get('reports_transport_acknowledged') == 3)
            live = manager.snapshot()
            assert live['observation']['ble']['retained'] and gatt.is_connected
            assert network.connected and manager.owns_link
            assert not live['flight_ready'] and not live['application_ready']
            assert request('/v1/drone/session/close', {})[0] == 202
            until(lambda: not manager.busy)
        final = manager.snapshot()
        assert final['state'] == 'closed' and not network.connected and gatt.exited
        assert final['observation']['wifi']['cleanup_verified']
        assert final['observation']['ble']['cleanup_verified']
        assert not final['observation']['ble']['retained'] and not manager.owns_link
        assert Path(final['observation']['video_path']).read_bytes() == peer.video
        raw_report = next(p for p in gatt.packets if p.cmd_id == 0x93)
        udp_reports = [DumlPacket.decode(raw[20:]) for raw in peer.requests if raw[6] == 5]
        assert [len(p.payload) for p in udp_reports] == [24, 24, 31]
        assert raw_report.payload[4:24] == udp_reports[0].payload[4:24] == udp_reports[2].payload[7:27]
        public = json.dumps(final) + Path(final['observation']['wifi_journal_path']).read_text()
        for secret in (SSID.decode(), PASSWORD.decode(), ADDRESS,
                       str(manager._credentials_path), str(manager.config.ble_identity_file),
                       load_or_create_ble_identity(manager.config.ble_identity_file).identification_id,
                       load_or_create_identity(manager.config.wlm_identity_file).identification_id):
            assert secret not in public
        for path in (manager._credentials_path, *[Path(p) for p in manager.paths.values() if p]):
            assert path.stat().st_mode & 0o777 == 0o600


@pytest.mark.parametrize('point', ['activation', 'session', 'cleanup'])
def test_cancellation_waits_for_wifi_cleanup_before_ble_exit(tmp_path, network, peer, gatt, monkeypatch, point):
    entered, release = threading.Event(), threading.Event()
    original = network.run
    def command(*args, **options):
        if point == 'activation' and 'up' in args:
            network.calls.append(args)
            network.connected = True
            entered.set()
            assert options['cancel_event'].wait(2)
            raise wifi.WifiLinkCancelled('wifi_connection_cancelled')
        if point == 'cleanup' and 'delete' in args:
            entered.set()
            assert release.wait(2)
        return original(*args, **options)
    monkeypatch.setattr(wifi, '_run', command)
    try:
        with owner(tmp_path, network, peer, gatt) as manager:
            manager.open({'duration': .15 if point == 'cleanup' else 3})
            if point == 'session':
                until(lambda: manager.snapshot()['connected'])
            else:
                assert entered.wait(2)
            assert gatt.is_connected
            manager.close()
            if point == 'cleanup':
                assert manager.busy and gatt.is_connected
                with pytest.raises(ObservationError, match='already_running'):
                    manager.open()
                release.set()
            until(lambda: not manager.busy)
            assert manager.snapshot()['state'] == 'closed'
            assert not manager.owns_link and gatt.exited and not network.connected
            if point == 'activation':
                assert not peer.requests
    finally:
        release.set()


def test_cancel_during_ble_write_never_starts_wifi_or_udp(tmp_path, network, peer, gatt):
    gatt.mode = 'stall'
    with owner(tmp_path, network, peer, gatt) as manager:
        manager.open()
        until(lambda: gatt.is_connected)
        manager.close()
        until(lambda: not manager.busy)
        assert manager.snapshot()['state'] == 'closed' and gatt.exited
        assert not any('load' in call or 'up' in call for call in network.calls)
        assert not peer.requests


@pytest.mark.parametrize('problem,reason', [
    ('default', 'interface_carries_default_route'),
    ('busy', 'interface_must_be_disconnected_wifi'),
    ('root', 'wifi_activation_requires_root'),
])
def test_unavailable_interface_is_refused_before_bluetooth(tmp_path, network, peer, gatt, monkeypatch, problem, reason):
    if problem == 'default':
        network.default_interface = 'wlanusb0'
    elif problem == 'busy':
        network.device_state = 'wifi\n100 (connected)\n'
    else:
        monkeypatch.setattr(wifi.os, 'geteuid', lambda: 1000)
    with owner(tmp_path, network, peer, gatt) as manager:
        manager.open()
        until(lambda: not manager.busy)
        assert manager.snapshot()['observation']['error'] == reason
        assert not gatt.exited and not gatt.is_connected and not peer.requests
        assert not any('up' in c for c in network.calls)


@pytest.mark.parametrize('problem,reason', [
    ('deny', 'ble_verification_device_busy'),
    ('disconnect', 'ble_bluetooth_disconnected'),
    ('write', 'ble_bluetooth_error'),
])
def test_setup_failure_prevents_wifi_activation(tmp_path, network, peer, gatt, problem, reason):
    if problem == 'deny':
        gatt.transform = lambda req, reply: replace(reply, payload=b'\0\3').encode()
    elif problem == 'disconnect':
        gatt.mode = 'disconnect'
    else:
        gatt.mode = 'write_error'
    with owner(tmp_path, network, peer, gatt) as manager:
        manager.open()
        until(lambda: not manager.busy)
        assert manager.snapshot()['observation']['error'] == reason
        assert gatt.exited and not peer.requests
        assert not any('load' in c or 'up' in c for c in network.calls)


def test_late_ble_loss_keeps_wifi_session_and_reports_disconnect(tmp_path, network, peer, gatt, monkeypatch):
    original = gatt.start_notify
    async def notify(*args):
        await original(*args)
        gatt.loop = asyncio.get_running_loop()
    monkeypatch.setattr(gatt, 'start_notify', notify)
    with owner(tmp_path, network, peer, gatt) as manager:
        manager.open({'duration': 3})
        until(lambda: manager.snapshot()['connected'])
        def drop():
            gatt.is_connected = False
            gatt.disconnected(gatt)
        gatt.loop.call_soon_threadsafe(drop)
        until(lambda: manager.snapshot()['observation']['ble']['disconnected_during_hold'])
        assert manager.snapshot()['connected'] and network.connected
        manager.close()
        until(lambda: not manager.busy)
        assert manager.snapshot()['state'] == 'closed'


def test_unconfirmed_ble_cleanup_latches_owner_and_refuses_reopen(tmp_path, network, peer, gatt):
    async def broken_exit(*args):
        raise RuntimeError('private backend detail')
    async def broken_disconnect():
        pass
    gatt.__class__ = type('LeakingGatt', (type(gatt),), {'__aexit__': broken_exit})
    gatt.disconnect = broken_disconnect
    with owner(tmp_path, network, peer, gatt) as manager:
        manager.open({'duration': .15})
        until(lambda: not manager.busy)
        final = manager.snapshot()
        assert final['state'] == 'failed'
        assert final['observation']['error'] == 'ble_cleanup_unconfirmed'
        assert manager.owns_link and manager.ble_cleanup_pending
        assert not network.connected and final['observation']['ble']['connected']
        with pytest.raises(ObservationError, match='ble_cleanup_unconfirmed'):
            manager.open()
        with pytest.raises(ObservationError, match='ble_cleanup_unconfirmed'):
            manager.shutdown()
        state = service.ServiceState(tmp_path, tmp_path, True, observation=manager)
        with pytest.raises(ValueError, match='owns the UDP link'):
            state.submit('udp_probe', {})


def test_new_open_uses_fresh_credentials_and_same_persisted_identities(tmp_path, network, peer, gatt):
    with owner(tmp_path, network, peer, gatt) as manager:
        credentials, identities = [], []
        for _ in range(2):
            manager.open({'duration': .15})
            until(lambda: not manager.busy)
            assert manager.snapshot()['state'] == 'closed'
            credentials.append(manager._credentials_path)
            identities.append(next(p for p in reversed(gatt.packets) if p.cmd_id == 0x93).payload[4:24])
        assert credentials[0] != credentials[1]
        assert all(p.exists() for p in credentials)
        assert identities[0] == identities[1]


def test_connect_cli_uses_continuous_owner_and_restores_sigterm(tmp_path, network, peer, gatt, monkeypatch, capsys):
    original_signal = signal.getsignal(signal.SIGTERM)
    monkeypatch.setattr(obs, 'run_session', peer.runner)
    assert cli.main(['connect-session', ADDRESS, '--interface', 'wlanusb0',
        '--ble-identity-file', str(tmp_path / 'ble-id.json'),
        '--wlm-identity-file', str(tmp_path / 'wlm-id.json'),
        '--capture-dir', str(tmp_path / 'captures'), '--duration', '.2']) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['session']['wlm_bootstrap']['reports_transport_acknowledged'] == 3
    assert result['observation']['ble']['cleanup_verified']
    assert signal.getsignal(signal.SIGTERM) == original_signal and gatt.exited


@pytest.mark.parametrize('options', [
    {'observe_ble_address': ADDRESS},
    {'observe_interface': 'wlanusb0', 'observe_ble_address': ADDRESS},
    {'observe_interface': 'wlanusb0', 'allow_active_syn': True, 'observe_ble_address': ADDRESS},
    {'observe_interface': 'wlanusb0', 'allow_active_syn': True, 'drone_mode': 'simulate',
     'observe_ble_address': ADDRESS, 'observe_ble_identity_file': '/tmp/ble', 'observe_wlm_identity_file': '/tmp/wlm'},
    {'observe_ble_identity_file': '/tmp/ble'},
    {'observe_ble_setup_timeout': 10},
])
def test_invalid_service_configuration_precedes_files(tmp_path, options):
    with pytest.raises(ValueError):
        service.serve(listen='127.0.0.1', port=0, token='x' * 32,
            capture_dir=tmp_path / 'captures', report_dir=tmp_path / 'reports', **options)
    assert not list(tmp_path.iterdir())


def test_wrong_identity_format_precedes_radio_or_wifi(tmp_path, network, peer, gatt):
    load_or_create_ble_identity(tmp_path / 'wlm-id.json')
    with owner(tmp_path, network, peer, gatt) as manager:
        manager.open()
        until(lambda: not manager.busy)
        assert manager.snapshot()['observation']['error'] == 'wlm_identity_invalid_or_unavailable'
        assert not network.calls and not gatt.exited


def test_wifi_availability_is_rechecked_after_credentials(tmp_path, network, peer, gatt):
    def transform(request, response):
        if request.cmd_id == 0x0E:
            network.device_state = 'wifi\n100 (connected)\n'
        return response.encode()
    gatt.transform = transform
    with owner(tmp_path, network, peer, gatt) as manager:
        manager.open()
        until(lambda: not manager.busy)
        assert manager.snapshot()['observation']['error'] == 'interface_must_be_disconnected_wifi'
        assert gatt.exited and not any('load' in c or 'up' in c for c in network.calls)
        assert not peer.requests


@pytest.mark.parametrize('problem', ['alias', 'hardlink'])
def test_identity_alias_is_rejected_before_network_or_bluetooth(tmp_path, network, peer, gatt, problem):
    ble_identity = tmp_path / 'ble-id.json'
    load_or_create_ble_identity(ble_identity)
    wlm_identity = ble_identity if problem == 'alias' else tmp_path / 'wlm-id.json'
    if problem == 'hardlink':
        wlm_identity.hardlink_to(ble_identity)
    before = ble_identity.read_bytes()
    with owner(tmp_path, network, peer, gatt, wlm_identity_file=wlm_identity) as manager:
        manager.open()
        until(lambda: not manager.busy)
        assert manager.snapshot()['state'] == 'failed'
        assert not network.calls and not gatt.exited and not peer.requests
        assert ble_identity.read_bytes() == before


def test_serve_retains_ble_and_sigterm_waits_for_full_cleanup(tmp_path, network, peer, gatt, monkeypatch):
    monkeypatch.setattr(obs, 'run_session', peer.runner)
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
                assert request('/v1/drone/session/open', {'duration': 3})[0] == 202
                until(lambda: captured[0].observation.snapshot()['session'].get(
                    'wlm_bootstrap', {}).get('reports_transport_acknowledged') == 3)
                assert gatt.is_connected and network.connected
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
    monkeypatch.setattr(service, 'ThreadingHTTPServer', Server)
    previous = signal.getsignal(signal.SIGTERM)
    with pytest.raises(KeyboardInterrupt):
        service.serve(listen='127.0.0.1', port=0, token='x' * 32,
            capture_dir=tmp_path / 'captures', report_dir=tmp_path / 'reports',
            allow_active_syn=True, observe_interface='wlanusb0',
            observe_ble_address=ADDRESS, observe_ble_identity_file=tmp_path / 'ble-id.json',
            observe_wlm_identity_file=tmp_path / 'wlm-id.json', observe_ble_setup_timeout=.5)
    final = captured[0].observation.snapshot()
    assert final['state'] == 'closed' and gatt.exited and not network.connected
    assert final['observation']['ble']['cleanup_verified'] and not captured[0].observation.busy
    assert signal.getsignal(signal.SIGTERM) == previous


def test_connect_cli_sigterm_requests_cleanup_once(tmp_path, network, peer, gatt, monkeypatch):
    manager_type, captured = cli.ManagedObservation, []
    def create(*args, **options):
        manager = manager_type(*args, **options, session_runner=peer.runner)
        captured.append(manager)
        return manager
    monkeypatch.setattr(cli, 'ManagedObservation', create)
    real_sleep = cli.time.sleep
    signalled = []
    def wait(seconds):
        if captured[0].snapshot()['connected'] and not signalled:
            signalled.append(True)
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        real_sleep(seconds)
    monkeypatch.setattr(cli.time, 'sleep', wait)
    previous = signal.getsignal(signal.SIGTERM)
    assert cli.main(['connect-session', ADDRESS, '--interface', 'wlanusb0',
        '--ble-identity-file', str(tmp_path / 'ble-id.json'),
        '--wlm-identity-file', str(tmp_path / 'wlm-id.json'),
        '--capture-dir', str(tmp_path / 'captures'), '--duration', '3']) == 130
    assert gatt.exited and not network.connected and not captured[0].busy
    assert signal.getsignal(signal.SIGTERM) == previous


def test_broken_parent_journal_still_latches_ble_cleanup_fault(tmp_path, network, peer, gatt):
    async def broken_exit(*args):
        raise RuntimeError('backend failure')
    async def broken_disconnect():
        pass
    gatt.__class__ = type('LeakingGatt', (type(gatt),), {'__aexit__': broken_exit})
    gatt.disconnect = broken_disconnect
    with owner(tmp_path, network, peer, gatt) as manager:
        def journal(name, **details):
            if name == 'ble_handoff_event' and details['phase_event'] == 'ble_cleanup_result':
                raise OSError('full disk')
        manager.event_sink = journal
        manager.open({'duration': .15})
        until(lambda: not manager.busy)
        assert manager.ble_cleanup_pending and manager.owns_link
        assert manager.snapshot()['observation']['error'] == 'ble_cleanup_unconfirmed'
