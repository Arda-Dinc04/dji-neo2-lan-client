from contextlib import contextmanager
import json
from pathlib import Path
import signal
import threading

import pytest

from neo2_control import observation as observation_module, service, wifi_link as wifi
from neo2_control.ble_bootstrap import WifiCredentials
from neo2_control.controller import ControlSupervisor, DecisionJournal
from neo2_control.observation import ManagedObservation, ObservationConfig, ObservationError
from neo2_control.service import ServiceState
from test_observation import LoopbackDrone, until
from test_observation_service import api
from test_wifi_link import PASSWORD, SSID, network


@contextmanager
def managed(tmp_path, network, peer, *, event_override=None):
    source = tmp_path / 'wifi-private.json'
    WifiCredentials(bytes.fromhex('020000000002'), SSID, PASSWORD).save_new(source)
    journal = DecisionJournal(tmp_path / 'controller.jsonl')
    controller = ControlSupervisor(journal)
    def event(name, **metadata):
        if event_override:
            event_override(name, **metadata)
        controller.observation_event(name, **metadata)
    def runner(**options):
        assert network.connected and network.directory.exists()
        return peer.runner(**options)
    manager = ManagedObservation(ObservationConfig('wlanusb0', maximum_duration=10,
        record_video=True, credentials_file=source), tmp_path / 'captures',
        session_runner=runner, event_sink=event)
    controller.attach_observation_source(manager.snapshot)
    try:
        yield controller, manager, source
    finally:
        try:
            manager.shutdown()
        except ObservationError:
            if not manager.wifi_cleanup_pending:
                raise
        journal.close()


@pytest.fixture
def peer():
    drone = LoopbackDrone()
    yield drone
    drone.close()


def test_http_open_owns_temporary_wifi_until_udp_closes_and_cleanup_is_verified(tmp_path, network, peer):
    with managed(tmp_path, network, peer) as (controller, manager, source):
        state = ServiceState(tmp_path / 'captures', tmp_path, True,
                             controller=controller, observation=manager)
        with api(state) as request:
            assert request('/v1/drone/session/open', {'credentials_file': '/tmp/other'})[0] == 400
            assert request('/v1/drone/session/open', {'duration': 3})[0] == 202
            until(lambda: (request('/v1/drone/status')[1]['telemetry'] or {}).get('available'))
            _, active = request('/v1/drone/status')
            assert active['state'] == 'observing' and network.connected
            assert active['observation']['wifi']['stage'] == 'connected'
            assert active['observation']['wifi']['profile_created']
            assert not active['observation']['wifi']['cleanup_verified']
            assert request('/v1/drone/session/close', {})[0] == 202
            until(lambda: not manager.busy)
            final = request('/v1/drone/status')[1]
            assert final['state'] == 'closed' and not final['connected'] and not network.connected
            assert final['observation']['wifi']['cleanup_verified']
            assert final['observation']['wifi']['profile_removed']
            assert final['observation']['wifi']['deactivated']
            assert not list(network.directory.iterdir())
            assert not manager.wifi_cleanup_pending
            assert Path(final['observation']['video_path']).read_bytes() == peer.video
            wifi_journal = Path(final['observation']['wifi_journal_path'])
            assert wifi_journal.stat().st_mode & 0o777 == 0o600
            public = json.dumps(final) + wifi_journal.read_text() + controller.journal.path.read_text()
            assert PASSWORD.decode() not in public and SSID.decode() not in public
            assert str(source) not in json.dumps(final)
            # An asynchronous producer cannot resurrect a terminated owner.
            manager._publish(manager.identifier, {'state': 'closed'})
            assert not manager.busy and manager.snapshot()['state'] == 'closed'


def test_cancel_during_activation_cleans_partial_profile_without_starting_udp(tmp_path, network, peer, monkeypatch):
    entered = threading.Event()
    original = network.run
    def command(*args, **options):
        if 'up' in args:
            network.calls.append(args)
            network.connected = True  # NM may already be activating when nmcli is cancelled.
            entered.set()
            assert options['cancel_event'].wait(2)
            raise wifi.WifiLinkCancelled('wifi_connection_cancelled')
        if 'down' in args or 'delete' in args:
            assert options.get('cancel_event') is None
        return original(*args, **options)
    monkeypatch.setattr(wifi, '_run', command)
    with managed(tmp_path, network, peer) as (_, manager, _):
        manager.open()
        assert entered.wait(1)
        manager.close()
        until(lambda: not manager.busy)
        final = manager.snapshot()
        assert final['state'] == 'closed' and final['observation']['error'] is None
        assert final['observation']['wifi']['cleanup_verified']
        assert not network.connected and not peer.requests
        assert not Path(final['observation']['capture_path']).exists()


@pytest.mark.parametrize('attribute,value,reason', [
    ('default_interface', 'wlanusb0', 'interface_carries_default_route'),
    ('device_state', 'wifi\n100 (connected)\n', 'interface_must_be_disconnected_wifi'),
    ('fail', 'up', 'wifi_activation_failed'),
    ('bad_route', True, 'drone_route_is_not_direct_on_selected_interface'),
    ('active_uuid', 'foreign-connection', 'selected_profile_is_not_active'),
])
def test_network_failures_precede_transport_and_report_owned_profile_cleanup(tmp_path, network, peer, attribute, value, reason):
    setattr(network, attribute, value)
    with managed(tmp_path, network, peer) as (_, manager, _):
        manager.open()
        until(lambda: not manager.busy)
        status = manager.snapshot()
        assert status['state'] == 'failed' and status['observation']['error'] == reason
        assert not peer.requests
        assert status['observation']['wifi']['cleanup_verified']
        if attribute not in ('device_state', 'default_interface'):
            assert not network.connected and status['observation']['wifi']['profile_removed']


def test_cleanup_stays_visible_and_exclusive_until_networkmanager_finishes(tmp_path, network, peer, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = network.run
    def command(*args, **options):
        if 'delete' in args:
            entered.set()
            assert release.wait(2)
        return original(*args, **options)
    monkeypatch.setattr(wifi, '_run', command)
    try:
        with managed(tmp_path, network, peer) as (controller, manager, _):
            state = ServiceState(tmp_path, tmp_path, True, controller=controller, observation=manager)
            manager.open()
            until(lambda: (manager.snapshot()['telemetry'] or {}).get('available'))
            manager.close()
            assert entered.wait(1)
            closing = manager.snapshot()
            assert closing['state'] == 'stopping' and closing['observation']['worker_alive']
            assert closing['observation']['worker_state'] == 'cleaning_up'
            assert not closing['telemetry']['fields']['motors_on']['valid']
            with pytest.raises(ObservationError, match='already_running'):
                manager.open()
            with pytest.raises(ValueError, match='owns the UDP link'):
                state.submit('udp_probe', {})
            release.set()
            until(lambda: not manager.busy)
            assert manager.snapshot()['state'] == 'closed'
    finally:
        release.set()


def test_failed_cleanup_latches_ownership_and_shutdown_error(tmp_path, network, peer):
    with managed(tmp_path, network, peer) as (controller, manager, _):
        manager.open()
        until(lambda: (manager.snapshot()['telemetry'] or {}).get('available'))
        network.fail_delete = True
        manager.close()
        until(lambda: not manager.busy)
        status = manager.snapshot()
        assert status['state'] == 'failed' and status['observation']['error'] == 'wifi_cleanup_failed'
        assert manager.owns_link and manager.wifi_cleanup_pending
        assert not status['observation']['wifi']['cleanup_verified']
        with pytest.raises(ObservationError, match='wifi_cleanup_unconfirmed'):
            manager.open()
        state = ServiceState(tmp_path, tmp_path, True, controller=controller, observation=manager)
        with pytest.raises(ValueError, match='owns the UDP link'):
            state.submit('udp_probe', {})
        with pytest.raises(ObservationError, match='wifi_cleanup_unconfirmed'):
            manager.shutdown()


def test_lifecycle_journal_failure_after_file_creation_still_removes_profile_and_secrets(tmp_path, network, peer):
    def failed_event(name, **metadata):
        if name == 'runtime_profile_created':
            raise OSError(PASSWORD.decode())
    with managed(tmp_path, network, peer, event_override=failed_event) as (_, manager, _):
        manager.open()
        until(lambda: not manager.busy)
        status = manager.snapshot()
        assert status['state'] == 'failed' and status['observation']['error'] == 'observation_io_error'
        assert status['observation']['wifi']['cleanup_verified']
        assert not list(network.directory.iterdir()) and not peer.requests
        assert PASSWORD.decode() not in json.dumps(status)


def test_invalid_credentials_and_nonroot_never_activate_or_send_packets(tmp_path, network, peer, monkeypatch):
    with managed(tmp_path, network, peer) as (_, manager, source):
        before = source.read_bytes()
        monkeypatch.setattr(wifi.os, 'geteuid', lambda: 1000)
        manager.open()
        until(lambda: not manager.busy)
        assert manager.snapshot()['observation']['error'] == 'wifi_activation_requires_root'
        assert not any('load' in call or 'up' in call for call in network.calls)
        assert not peer.requests
        source.write_bytes(b'not valid credentials')
        manager.open()
        until(lambda: not manager.busy)
        assert manager.snapshot()['observation']['error'] == 'wifi_credentials_invalid_or_unavailable'
        assert not peer.requests
        source.write_bytes(before)


def test_sigterm_handler_joins_receive_worker_and_wifi_cleanup_before_return(tmp_path, network, peer, monkeypatch):
    source = tmp_path / 'credentials.json'
    WifiCredentials(bytes.fromhex('020000000002'), SSID, PASSWORD).save_new(source)
    monkeypatch.setattr(observation_module, 'run_session', peer.runner)
    captured = []
    original_handler = service.build_handler
    def handler(state, token):
        captured.append(state)
        return original_handler(state, token)
    monkeypatch.setattr(service, 'build_handler', handler)
    class Server:
        server_port = 0
        def __init__(self, *args): pass
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def serve_forever(self):
            state = captured[0]
            state.open_observation({'duration': 3})
            until(lambda: (state.drone_status()['telemetry'] or {}).get('available'))
            assert network.connected
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
    monkeypatch.setattr(service, 'ThreadingHTTPServer', Server)
    previous = signal.getsignal(signal.SIGTERM)
    with pytest.raises(KeyboardInterrupt):
        service.serve(listen='127.0.0.1', port=0, token='x' * 32,
            capture_dir=tmp_path / 'captures', report_dir=tmp_path / 'reports',
            allow_active_syn=True, observe_interface='wlanusb0', observe_credentials_file=source)
    assert signal.getsignal(signal.SIGTERM) == previous
    manager = captured[0].observation
    assert not manager.thread.is_alive() and manager.snapshot()['observation']['wifi']['cleanup_verified']
    assert not network.connected and not list(network.directory.iterdir())
    assert captured[0].controller.journal.stream.closed
