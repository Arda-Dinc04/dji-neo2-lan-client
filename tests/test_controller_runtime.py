import json
import socket
import select
import signal
import subprocess
import sys
import threading
import time
from urllib.request import Request, urlopen

import pytest

from neo2_control.controller_runtime import ControllerRuntime, SCENARIOS, run_simulation
from neo2_control.duml import DumlPacket
from neo2_control.service import serve
from test_controller import authorize, takeoff


@pytest.mark.parametrize('scenario', SCENARIOS)
def test_virtual_time_scenarios_execute_real_supervisor_without_socket_or_device(tmp_path, monkeypatch, scenario):
    def forbidden(*args, **kwargs):
        raise AssertionError('simulation opened a network socket')
    monkeypatch.setattr(socket, 'socket', forbidden)
    path = tmp_path / f'{scenario}.jsonl'
    result = run_simulation(path, scenario)
    assert result['scenario_passed'] and result['journal_healthy']
    assert not result['live_flight_enabled']
    assert result['landing_confirmed'] == (scenario in ('normal', 'control-loss'))
    events = [json.loads(line) for line in path.read_text().splitlines()]
    packets = [DumlPacket.decode(bytes.fromhex(event['raw_hex'])) for event in events if event['event'] == 'packet_intent']
    actions = [packet.payload[0] for packet in packets if packet.cmd_id == 0x2A]
    assert actions.count(1) == 1
    assert actions.count(2) == (0 if scenario == 'link-loss' else 1)
    assert all(event['mode'] == 'simulate' for event in events)
    assert any(event['state'] == 'taking_off' for event in result['transitions'])
    assert path.stat().st_mode & 0o777 == 0o600


def test_real_watchdog_handles_operator_loss_without_any_http_or_status_calls(tmp_path):
    runtime = ControllerRuntime(tmp_path / 'watchdog.jsonl', mode='simulate')
    try:
        c = runtime.controller
        lease = authorize(c)
        takeoff(c, lease)
        deadline = time.monotonic() + 1
        while c.state != 'flying' and time.monotonic() < deadline:
            threading.Event().wait(.01)
        assert c.state == 'flying'
        # No controller calls: the local thread must progress by itself.
        threading.Event().wait(.7)
        with c.lock:
            assert c.state == 'fault' and c.landing_confirmed
            assert c.fault_reason == 'operator_lease_expired'
            actions = [packet.payload[0] for _, packet in runtime.backend.sent if packet.cmd_id == 0x2A]
            assert actions == [1, 2]
        assert runtime.failure is None
    finally:
        runtime.close()
    assert not runtime.thread.is_alive() and runtime.journal.stream.closed


def test_normal_shutdown_keeps_watchdog_running_through_synthetic_landing(tmp_path):
    runtime = ControllerRuntime(tmp_path / 'shutdown.jsonl', mode='simulate')
    c = runtime.controller
    try:
        lease = authorize(c)
        takeoff(c, lease)
        deadline = time.monotonic() + 1
        while c.state != 'flying' and time.monotonic() < deadline:
            threading.Event().wait(.01)
        assert c.state == 'flying'
    finally:
        runtime.close()
    assert c.landing_confirmed and not c.flight_started
    assert c.fault_reason == 'controller_stopped'
    assert not runtime.thread.is_alive()


def test_live_and_simulated_radio_probe_interlocks_precede_files_and_threads(tmp_path):
    journal = tmp_path / 'not-created.jsonl'
    with pytest.raises(ValueError, match='not verified'):
        ControllerRuntime(journal, mode='flight')
    with pytest.raises(ValueError, match='not verified'):
        serve(listen='127.0.0.1', port=0, token='x' * 32,
              capture_dir=tmp_path / 'captures', report_dir=tmp_path / 'reports', drone_mode='flight')
    with pytest.raises(ValueError, match='radio probes'):
        serve(listen='127.0.0.1', port=0, token='x' * 32,
              capture_dir=tmp_path / 'captures', report_dir=tmp_path / 'reports',
              drone_mode='simulate', allow_active_syn=True)
    assert not journal.exists() and not (tmp_path / 'captures').exists()


def test_cli_reports_verified_scenario_and_preserves_existing_journal(tmp_path):
    path = tmp_path / 'cli.jsonl'
    command = [sys.executable, '-m', 'neo2_control', 'simulate', '--scenario', 'control-loss', '--journal', str(path)]
    run = subprocess.run(command, capture_output=True, text=True, timeout=5)
    assert run.returncode == 0, run.stderr
    result = json.loads(run.stdout)
    assert result['scenario_passed'] and result['fault_reason'] == 'control_expired'
    before = path.read_bytes()
    run = subprocess.run(command, capture_output=True, text=True, timeout=5)
    assert run.returncode == 1 and path.read_bytes() == before


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
def test_serve_cli_binds_loopback_enables_simulation_and_closes_journal_on_stop_signal(tmp_path, stop_signal):
    token = tmp_path / 'api.token'
    token.write_text('x' * 32)
    journal = tmp_path / 'server.jsonl'
    process = subprocess.Popen([
        sys.executable, '-m', 'neo2_control', 'serve', '--port', '0',
        '--token-file', str(token), '--capture-dir', str(tmp_path / 'captures'),
        '--report-dir', str(tmp_path / 'reports'), '--drone-mode', 'simulate',
        '--controller-journal', str(journal),
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        ready, _, _ = select.select([process.stdout], [], [], 5)
        assert ready, 'service startup was not observed'
        startup = json.loads(process.stdout.readline())
        assert startup['listen'] == '127.0.0.1' and startup['drone_mode'] == 'simulate'
        base = f"http://127.0.0.1:{startup['port']}"
        headers = {'Authorization': 'Bearer ' + 'x' * 32}
        with urlopen(Request(base + '/v1/drone/status', headers=headers), timeout=2) as response:
            status = json.load(response)
        assert status['mode'] == 'simulate' and not status['live_flight_enabled']
        req = Request(base + '/v1/drone/simulation/intent', headers=headers,
                      data=b'{"action":"open"}')
        with urlopen(req, timeout=2) as response:
            assert json.load(response)['state'] == 'ready'
        process.send_signal(stop_signal)
        _, stderr = process.communicate(timeout=4)
        assert process.returncode == 130, stderr
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=2)
    events = [json.loads(line) for line in journal.read_text().splitlines()]
    assert any(event['event'] == 'controller_stop_requested' for event in events)
    assert events[-1]['event'] == 'controller_owner_stopped'
    assert not events[-1]['possible_airborne']
