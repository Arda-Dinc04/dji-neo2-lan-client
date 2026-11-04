from contextlib import contextmanager
from http.server import ThreadingHTTPServer
import json
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from neo2_control.controller_runtime import ControllerRuntime
from neo2_control.service import ServiceState, build_handler


@contextmanager
def api(tmp_path, *, simulate=False):
    runtime = ControllerRuntime(tmp_path / 'http-controller.jsonl', mode='simulate') if simulate else None
    state = ServiceState(tmp_path, tmp_path, False, controller=runtime.controller if runtime else None)
    server = ThreadingHTTPServer(('127.0.0.1', 0), build_handler(state, 'x' * 32))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def request(path, body=None, *, authorized=True, raw=None):
        headers = {'Authorization': 'Bearer ' + 'x' * 32} if authorized else {}
        data = raw if raw is not None else (json.dumps(body).encode() if body is not None else None)
        req = Request(f'http://127.0.0.1:{server.server_port}' + path, data=data, headers=headers)
        try:
            with urlopen(req, timeout=2) as response:
                return response.status, json.load(response)
        except HTTPError as response:
            return response.code, json.load(response)

    try:
        yield request, state, runtime
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        if runtime:
            runtime.close()


def test_observation_is_default_and_remote_requests_cannot_enable_flight(tmp_path):
    with api(tmp_path) as (request, _, _):
        assert request('/v1/drone/status', authorized=False)[0] == 401
        code, status = request('/v1/drone/status')
        assert code == 200 and status['mode'] == 'observe' and not status['connected']
        assert not status['live_flight_enabled']
        assert request('/v1/drone/simulation/intent', {'action': 'open'})[0] == 403
        assert request('/v1/drone/flight/takeoff', {})[0] == 404


def test_authenticated_http_cycle_uses_supervisor_and_tokens_are_not_returned_in_status(tmp_path, monkeypatch):
    with api(tmp_path, simulate=True) as (request, state, runtime):
        path = '/v1/drone/simulation/intent'

        def intent(action, **parameters):
            code, result = request(path, {'action': action, 'parameters': parameters})
            assert code == 200, result
            return result

        assert request(path, {'action': 'open'}, authorized=False)[0] == 401
        intent('open')
        token = intent('prepare', operation='arm')['confirmation_token']
        lease = intent('arm', confirmation_token=token)['lease']
        token = intent('prepare', operation='takeoff', lease=lease)['confirmation_token']
        intent('takeoff', confirmation_token=token, lease=lease)
        assert request(path, {'action': 'takeoff', 'parameters': {'confirmation_token': token, 'lease': lease}})[0] == 409
        deadline = time.monotonic() + 1
        while True:
            _, status = request('/v1/drone/status')
            if status['state'] == 'flying' or time.monotonic() > deadline:
                break
            threading.Event().wait(.01)
        assert status['state'] == 'flying'
        assert lease not in json.dumps(status) and token not in json.dumps(status)
        assert status['telemetry']['synthetic'] and status['telemetry']['valid']
        intent('control', lease=lease, sequence=1, expires_at=status['now'] + .2, channels=[20, 0, 0, 0])
        intent('land')
        deadline = time.monotonic() + 1
        while True:
            _, status = request('/v1/drone/status')
            if status['state'] == 'ready' or time.monotonic() > deadline:
                break
            threading.Event().wait(.01)
        assert status['state'] == 'ready' and status['landing_confirmed']
        intent('close')
        assert runtime.failure is None


def test_simulated_service_blocks_radio_jobs_and_unknown_or_nonfinite_request_fields(tmp_path, monkeypatch):
    with api(tmp_path, simulate=True) as (request, state, _):
        def forbidden(*args, **kwargs):
            raise AssertionError('simulation dispatched a radio job')
        monkeypatch.setattr('neo2_control.service.run_probe', forbidden)
        monkeypatch.setattr('neo2_control.service.scan_sync', forbidden)
        _, status = request('/v1/status')
        assert status['mode'] == 'simulate' and status['allowed_actions'] == []
        assert not status['active_syn_enabled']
        for action in ('udp_probe', 'ble_scan'):
            assert request('/v1/commands', {'action': action})[0] == 400
        assert not state.jobs
        path = '/v1/drone/simulation/intent'
        assert request(path, {'action': 'open', 'mode': 'flight'})[0] == 400
        assert request(path, raw=b'{"action":"control","parameters":{"expires_at":NaN}}')[0] == 400
        assert request(path, {'action': 'open'})[0] == 200
        code, _ = request('/v1/drone/telemetry')
        assert code == 200
