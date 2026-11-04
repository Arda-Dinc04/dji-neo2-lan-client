from contextlib import contextmanager
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from neo2_control.observation import ObservationError
from neo2_control.service import ServiceState, build_handler, serve
from test_observation import LoopbackDrone, observed, until


@contextmanager
def api(state):
    server = ThreadingHTTPServer(('127.0.0.1', 0), build_handler(state, 'x' * 32))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    def request(path, payload=None, authorized=True):
        headers = {'Authorization': 'Bearer ' + 'x' * 32} if authorized else {}
        req = Request(f'http://127.0.0.1:{server.server_port}' + path, headers=headers,
                      data=None if payload is None else json.dumps(payload).encode())
        try:
            with urlopen(req, timeout=2) as response:
                return response.status, json.load(response)
        except HTTPError as response:
            return response.code, json.load(response)
    try:
        yield request
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_real_http_service_observes_udp_telemetry_and_stop_is_not_a_flight_command(tmp_path):
    peer = LoopbackDrone()
    try:
        with observed(tmp_path, peer, record_video=True) as (controller, manager):
            state = ServiceState(tmp_path / 'captures', tmp_path, True,
                                 controller=controller, observation=manager)
            with api(state) as request:
                assert request('/v1/drone/session/open', {}, authorized=False)[0] == 401
                assert request('/v1/drone/session/open', {'host': '127.0.0.1'})[0] == 400
                assert request('/v1/drone/session/open', {'duration': 3})[0] == 202
                until(lambda: request('/v1/drone/status')[1]['connected'])
                until(lambda: (request('/v1/drone/telemetry')[1]['telemetry'] or {}).get('available'))
                code, status = request('/v1/drone/status')
                assert code == 200 and status['mode'] == 'observe' and status['state'] == 'observing'
                assert not status['live_flight_enabled'] and not status['flight_ready']
                assert status['telemetry']['fields']['motors_on']['value']
                assert status['session']['transport_accepted'] and not status['session']['application_ready']
                assert request('/v1/drone/session/open', {})[0] == 409
                assert request('/v1/commands', {'action': 'udp_probe'})[0] == 400
                assert not state.jobs
                assert request('/v1/drone/simulation/intent', {'action': 'open'})[0] == 403
                assert request('/v1/drone/flight/takeoff', {})[0] == 404
                code, result = request('/v1/drone/session/close', {})
                assert code == 202 and result['accepted'] and not result['connected']
                until(lambda: not manager.busy)
                code, status = request('/v1/drone/status')
                assert status['state'] == 'closed' and not status['connected']
                assert not status['telemetry']['fields']['motors_on']['valid']
                capture = Path(status['observation']['capture_path'])
                code, _ = request('/v1/drone/session/open', {'duration': True})
                assert code == 400
                assert capture.is_file() and Path(str(capture) + '.session.jsonl').is_file()
    finally:
        peer.close()


def test_local_opt_in_is_required_even_in_observe_mode(tmp_path):
    state = ServiceState(tmp_path, tmp_path, False)
    with api(state) as request:
        assert request('/v1/drone/session/open', {})[0] == 403
        assert request('/v1/drone/session/close', {})[0] == 403


def test_udp_probe_and_managed_session_exclusively_own_the_link_in_both_directions(tmp_path, monkeypatch):
    entered, release, route_entered, route_release = [threading.Event() for _ in range(4)]
    def probe(**options):
        entered.set()
        assert release.wait(2)
        return SimpleNamespace(as_dict=lambda: {})
    monkeypatch.setattr('neo2_control.service.run_probe', probe)
    monkeypatch.setattr('neo2_control.service.generate_report', lambda *args: tmp_path / 'report.html')
    peer = LoopbackDrone()
    try:
        with observed(tmp_path, peer) as (controller, manager):
            def guard(interface):
                route_entered.set()
                assert route_release.wait(2)
            manager.guard = guard
            state = ServiceState(tmp_path, tmp_path, True, controller=controller, observation=manager)
            job = state.submit('udp_probe', {'duration': .1})
            assert entered.wait(1)
            with pytest.raises(ObservationError, match='udp_probe_owns_link'):
                state.open_observation({})
            release.set()
            until(lambda: job.state == 'completed')
            state.open_observation({})
            assert route_entered.wait(1)
            with pytest.raises(ValueError, match='owns the UDP link'):
                state.submit('udp_probe', {'duration': .1})
            manager.close()
            route_release.set()
    finally:
        release.set()
        route_release.set()
        peer.close()


@pytest.mark.parametrize('options', [
    {'observe_interface': 'test0'},
    {'observe_interface': 'test0', 'drone_mode': 'simulate'},
    {'observe_video': True}, {'observe_heartbeat': True},
    {'observe_max_duration': 10},
    {'observe_credentials_file': '/tmp/local-private.json'},
    {'observe_interface': '../bad', 'allow_active_syn': True},
    {'observe_interface': 'test0', 'observe_max_duration': float('nan'), 'allow_active_syn': True},
])
def test_bad_local_configuration_precedes_threads_files_and_network(tmp_path, options):
    with pytest.raises(ValueError):
        serve(listen='127.0.0.1', port=0, token='x' * 32,
              capture_dir=tmp_path / 'captures', report_dir=tmp_path / 'reports', **options)
    assert not list(tmp_path.iterdir())
