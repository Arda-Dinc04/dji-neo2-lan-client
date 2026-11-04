"""Real OS process failures, with synthetic aircraft/production DUML codecs."""
import contextlib
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from neo2_control.control_owner import (
    ProcessControllerRuntime, OwnerUnavailable, _decode, _encode, _receive, _send,
)
from neo2_control.controller import IntentRejected
from neo2_control.duml import DumlPacket
from test_controller import authorize, takeoff


def events(path):
    try:
        return [json.loads(line) for line in path.read_text().splitlines()]
    except (FileNotFoundError, json.JSONDecodeError):
        return []


def wait_for(predicate, timeout=4):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(.02)
    raise AssertionError('expected bounded process outcome was not observed')


def packets(path):
    return [DumlPacket.decode(bytes.fromhex(item['raw_hex']))
            for item in events(path) if item['event'] == 'packet_intent']


def verify_landing(path):
    log = events(path)
    sent = packets(path)
    assert [p.payload[0] for p in sent if (p.cmd_set, p.cmd_id) == (3, 0x2A)] == [1, 2]
    neutral = [p for p in sent if (p.cmd_set, p.cmd_id) == (1, 0x0A)]
    assert len(neutral) == 1
    word = int.from_bytes(neutral[0].payload[2:10], 'little')
    assert [(word >> shift) & 0x7FF for shift in (8, 19, 30, 41)] == [1024] * 4
    final = log[-1]
    assert final['event'] == 'controller_owner_stopped'
    assert final['landing_confirmed'] and not final['possible_airborne']
    ack = next(i for i, item in enumerate(log)
               if item['event'] == 'action_response' and item['request_action'] == 'land')
    grounded = next(i for i, item in enumerate(log)
                    if item['event'] == 'state_changed' and item['reason'] == 'grounded_observed_after_ack')
    assert grounded > ack
    return final


@contextlib.contextmanager
def owner(tmp_path):
    runtime = ProcessControllerRuntime(tmp_path / 'owner.jsonl')
    try:
        yield runtime
    finally:
        try:
            runtime.close()
        except RuntimeError:
            # Test cleanup may kill an explicitly synthetic, already failed
            # child. Production close deliberately refuses this shortcut.
            if runtime.process.poll() is None:
                runtime.process.send_signal(signal.SIGCONT)
                runtime.process.kill()
                runtime.process.wait(timeout=2)


def test_owner_process_keeps_watchdog_and_journal_outside_http_process(tmp_path):
    with owner(tmp_path) as runtime:
        assert runtime.process.pid != os.getpid()
        lease = authorize(runtime.controller)
        takeoff(runtime.controller, lease)
        status = wait_for(lambda: (s if (s := runtime.status())['state'] == 'flying' else None))
        assert status['controller_owner']['independent_watchdog']
        assert status['controller_owner']['scope'] == 'simulation'
        assert not status['live_flight_enabled']
        # Parent heartbeat remains active, but cannot refresh operator lease.
        time.sleep(.8)
        status = runtime.status()
        assert status['state'] == 'fault' and status['landing_confirmed']
        assert status['fault_reason'] == 'operator_lease_expired'
        assert lease not in json.dumps(status)
        runtime.close()
        assert runtime.cleanup_verified and runtime.final_received
        assert not runtime.process.poll()
        assert not runtime.thread.is_alive()
    verify_landing(tmp_path / 'owner.jsonl')
    assert (tmp_path / 'owner.jsonl').stat().st_mode & 0o777 == 0o600


def test_normal_shutdown_waits_for_landing_telemetry_and_one_land(tmp_path):
    with owner(tmp_path) as runtime:
        lease = authorize(runtime)
        takeoff(runtime, lease)
        runtime.close()
        assert runtime.cleanup_verified and runtime.last_status['landing_confirmed']
        assert runtime.status()['telemetry']['valid'] is False
        assert runtime.last_status['fault_reason'] == 'controller_stopped'
        before = (tmp_path / 'owner.jsonl').read_bytes()
        runtime.close()
        assert (tmp_path / 'owner.jsonl').read_bytes() == before
    verify_landing(tmp_path / 'owner.jsonl')


@pytest.mark.parametrize('mode', ['observe'])
def test_owner_rejects_unsupported_modes_before_files_or_processes(tmp_path, mode):
    with pytest.raises(ValueError, match='simulation only'):
        ProcessControllerRuntime(tmp_path / 'absent.jsonl', mode=mode)
    assert not list(tmp_path.iterdir())


def test_flight_owner_requires_explicit_configuration_before_files_or_processes(tmp_path):
    with pytest.raises(ValueError,match='FlightConfig'):
        ProcessControllerRuntime(tmp_path/'absent.jsonl',mode='flight')
    assert not list(tmp_path.iterdir())


def test_explicit_flight_owner_starts_inert_and_closes_without_radio(tmp_path):
    from neo2_control.flight_connection import FlightConfig
    config=FlightConfig('wlanusb0',tmp_path/'wlm.json',tmp_path/'ble.json',
                        capture_directory=tmp_path/'flight-sessions',incident_directory=tmp_path/'incidents')
    runtime=ProcessControllerRuntime(tmp_path/'flight-owner.jsonl',mode='flight',flight_config=config)
    try:
        status=runtime.status()
        assert status['mode']=='flight' and not status['synthetic']
        assert status['state']=='disconnected' and not status['connected']
        assert status['connection']['phase']=='idle' and not status['connection']['worker_alive']
        assert status['controller_owner']['scope']=='flight'
        assert not Path(config.capture_directory).exists()
    finally:runtime.close()
    assert runtime.cleanup_verified and runtime.final_received
    assert not Path(config.capture_directory).exists()


def test_proxy_ages_battery_motion_and_control_independently():
    from types import SimpleNamespace
    from neo2_control.controller import ControllerMode
    now=time.monotonic()
    runtime=object.__new__(ProcessControllerRuntime)
    runtime.epoch='e'*48
    runtime.mode=ControllerMode.FLIGHT
    runtime.process=SimpleNamespace(poll=lambda:None,pid=123)
    runtime.closed=False;runtime.failure=None;runtime.cleanup_verified=False
    runtime.final_received=False;runtime.last_response=now;runtime.possible_airborne=False
    runtime.last_status={'mode':'flight','state':'ready','connected':True,
        'limits':{'telemetry_age':.5,'battery_age':3.},
        'connection':{'phase':'session_open','worker_alive':True,'cleanup_verified':False},
        'telemetry':{'received_at':now-2.5,'motion_received_at':now-.1,
                     'battery_received_at':now-2.5,'control_received_at':now-.1,'valid':True}}
    assert runtime._snapshot()['telemetry']['valid']
    assert runtime._snapshot()['telemetry']['received_at']==now-2.5
    runtime.last_status['telemetry']['motion_received_at']=now-.6
    assert not runtime._snapshot()['telemetry']['valid']
    runtime.last_status['telemetry']['motion_received_at']=now-.1
    for component,field in (('motion','motion_received_at'),('battery','battery_received_at'),
                            ('control','control_received_at')):
        original=runtime.last_status['telemetry'][field]
        runtime.last_status['telemetry'][field]=None
        sample=runtime._snapshot()['telemetry']
        assert sample['component_age_seconds'][component] is None
        assert sample[field] is None and not sample['valid']
        runtime.last_status['telemetry'][field]=original
    runtime.failure='controller_owner_unavailable'
    snapshot=runtime._snapshot()
    assert snapshot['connection']['phase']=='unavailable'
    assert snapshot['connection']['worker_alive'] is None
    assert not snapshot['connection']['cleanup_verified']


@pytest.mark.parametrize('component',['request','motion','control','route','owner_route','missing','owner'])
def test_cached_landing_confirmation_expires_independently_of_navigation(monkeypatch,component):
    from types import SimpleNamespace
    from neo2_control.controller import ControllerMode
    monkeypatch.setattr('neo2_control.control_owner.time.monotonic',lambda:100.)
    runtime=object.__new__(ProcessControllerRuntime)
    runtime.epoch='e'*48
    runtime.mode=ControllerMode.FLIGHT
    runtime.process=SimpleNamespace(poll=lambda:None,pid=123)
    runtime.closed=False;runtime.failure=None;runtime.cleanup_verified=False
    runtime.final_received=False;runtime.last_response=100.;runtime.possible_airborne=True
    confirmation={'needed':True,'available':True,'evidence_available':True,'reason':None,
        'received_at':99.9,'motion_received_at':99.9,'control_received_at':99.9,
        'route_seen_at':99.9,'ready_at':99.9}
    runtime.last_status={'mode':'flight','state':'landing','connected':True,
        'limits':{'telemetry_age':.5,'battery_age':3.},
        'landing_confirmation':confirmation,'telemetry':{
            'received_at':99.9,'motion_received_at':99.9,'battery_received_at':None,
            'control_received_at':99.9,'valid':False}}
    fresh=runtime._snapshot()
    assert fresh['landing_confirmation']['available']
    assert not fresh['telemetry']['valid']  # GPS/battery validity is unrelated
    fields={'request':'received_at','motion':'motion_received_at','control':'control_received_at',
            'route':'route_seen_at','owner_route':'ready_at'}
    if component=='owner':runtime.failure='controller_owner_unavailable'
    elif component=='missing':del confirmation['received_at']
    else:confirmation[fields[component]]=98. if component=='route' else 99.49
    stale=runtime._snapshot()['landing_confirmation']
    assert not stale['available'] and not stale['evidence_available']
    if component in ('request','missing','owner'):assert stale['needed'] is None
    # Projection cannot renew the original evidence or mutate cached status.
    assert runtime.last_status['landing_confirmation']==confirmation


def test_owner_preserves_existing_journal_and_reports_startup_failure(tmp_path):
    path = tmp_path / 'old.jsonl'
    path.write_text('preserve')
    with pytest.raises(OwnerUnavailable, match='startup'):
        ProcessControllerRuntime(path)
    assert path.read_text() == 'preserve'


def test_parent_channel_loss_causes_neutral_and_landing(tmp_path):
    with owner(tmp_path) as runtime:
        lease = authorize(runtime)
        takeoff(runtime, lease)
        # The backend and journal remain owned by the child after the parent
        # closes its sole local descriptor.
        runtime.stopped.set()
        runtime.thread.join(timeout=1)
        runtime.channel.close()
        wait_for(lambda: events(tmp_path / 'owner.jsonl')[-1].get('event') == 'controller_owner_stopped')
        final = verify_landing(tmp_path / 'owner.jsonl')
        assert final['fault_reason'] == 'controller_parent_disconnected'
        status = runtime.status()
        assert not status['connected'] and not status['telemetry']['valid']
        assert not status['landing_confirmed']  # final IPC proof was unavailable
        assert status['possible_airborne'] is True
        with pytest.raises(IntentRejected, match='owner_unavailable'):
            runtime.handle('open')
        with pytest.raises(RuntimeError, match='cleanup unverified'):
            runtime.close()


@pytest.mark.parametrize('failure', ['kill', 'stop', 'after_reply'])
def test_owner_loss_never_repeats_stale_connected_or_landed_status(tmp_path, monkeypatch, failure):
    with owner(tmp_path) as runtime:
        lease = authorize(runtime)
        takeoff(runtime, lease)
        if failure == 'kill':
            runtime.process.kill()
            runtime.process.wait(timeout=2)
        elif failure == 'stop':
            runtime.process.send_signal(signal.SIGSTOP)
        else:
            # Kill the actual owner after a successful fresh reply, before
            # the proxy constructs the public status snapshot.
            original = runtime._request
            def reply_then_die(*args, **kwargs):
                result = original(*args, **kwargs)
                runtime.process.kill()
                runtime.process.wait(timeout=2)
                return result
            monkeypatch.setattr(runtime, '_request', reply_then_die)
        start = time.monotonic()
        status = runtime.status()
        assert time.monotonic() - start < 1
        assert status['state'] == 'fault' and status['fault_reason'] == 'controller_owner_unavailable'
        assert status['possible_airborne'] is True and not status['landing_confirmed']
        assert not status['connected'] and not status['telemetry']['valid']
        with pytest.raises(IntentRejected, match='owner_unavailable'):
            runtime.handle('keepalive', {'lease': lease})
        if failure == 'stop':
            # Production cleanup does not kill an unresponsive owner.
            import neo2_control.control_owner as module
            old = module.CLOSE_SECONDS
            module.CLOSE_SECONDS = .05
            try:
                with pytest.raises(RuntimeError, match='cleanup unverified'):
                    runtime.close()
                assert runtime.process.poll() is None
            finally:
                module.CLOSE_SECONDS = old
                runtime.process.send_signal(signal.SIGCONT)
        wait_for(lambda: runtime.process.poll() is not None)


@pytest.mark.parametrize('payload', [
    b'{"x":NaN}', b'{"x":1,"x":2}', b'[]', b'x' * 32769,
])
def test_ipc_decoder_rejects_nonfinite_duplicate_nonobject_and_oversized(payload):
    with pytest.raises((ValueError, json.JSONDecodeError)):
        _decode(payload)


def start_raw_owner(tmp_path):
    parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    process = subprocess.Popen([sys.executable, '-m', 'neo2_control.control_owner',
                                str(child.fileno()), str(tmp_path / 'raw.jsonl')],
                               pass_fds=(child.fileno(),), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    child.close()
    parent.settimeout(2)
    ready = _receive(parent)
    return parent, process, ready


@pytest.mark.parametrize('fault', ['expired', 'replay', 'epoch', 'too_future', 'huge_expiry', 'unknown', 'oversized'])
def test_child_rejects_stale_replayed_and_malformed_ipc_before_open(tmp_path, fault):
    channel, process, ready = start_raw_owner(tmp_path)
    request = {'kind': 'request', 'epoch': ready['epoch'], 'id': 1,
               'expires_at': time.monotonic() + .25,
               'operation': 'action', 'parameters': {'action': 'open', 'parameters': None}}
    try:
        if fault == 'expired':
            request['expires_at'] = time.monotonic() - 1
        elif fault == 'replay':
            _send(channel, {**request, 'operation': 'status', 'parameters': {}})
            assert _receive(channel)['kind'] == 'response'
        elif fault == 'epoch':
            request['epoch'] = 'stale'
        elif fault == 'too_future':
            request['expires_at'] += 10
        elif fault == 'huge_expiry':
            request['expires_at'] = 10 ** 1000
        elif fault == 'unknown':
            request['operation'] = 'unknown'
        if fault == 'oversized':
            channel.send(b'x' * 32769)
        else:
            _send(channel, request)
        process.wait(timeout=2)
        log = events(tmp_path / 'raw.jsonl')
        assert not any(item['event'] == 'intent_received' for item in log)
        assert log[-1]['fault_reason'] == 'controller_ipc_failed'
        assert not packets(tmp_path / 'raw.jsonl')
        assert process.returncode == 0
    finally:
        channel.close()
        if process.poll() is None:
            process.kill()
        stdout, stderr = process.communicate(timeout=2)
        assert not stdout and not stderr


@contextlib.contextmanager
def service_process(tmp_path):
    token = tmp_path / 'api.token'
    token.write_text('x' * 32)
    process = subprocess.Popen([
        sys.executable, '-m', 'neo2_control', 'serve', '--port', '0',
        '--token-file', str(token), '--capture-dir', str(tmp_path / 'captures'),
        '--report-dir', str(tmp_path / 'reports'), '--drone-mode', 'simulate',
        '--controller-journal', str(tmp_path / 'http.jsonl')],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert select.select([process.stdout], [], [], 5)[0]
        ready = json.loads(process.stdout.readline())
        base = f"http://127.0.0.1:{ready['port']}"
        def request(action=None, parameters=None):
            data = None if action is None else json.dumps({'action': action, 'parameters': parameters or {}}).encode()
            path = '/v1/drone/status' if action is None else '/v1/drone/simulation/intent'
            req = Request(base + path, data=data, headers={'Authorization': 'Bearer ' + 'x' * 32})
            with urlopen(req, timeout=2) as response:
                return json.load(response)
        yield process, request
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGCONT)
            process.send_signal(signal.SIGTERM)
        process.communicate(timeout=8)


@pytest.mark.parametrize('failure', ['sigstop', 'sigkill'])
def test_actual_http_parent_stop_or_death_keeps_child_alive_to_confirm_landing(tmp_path, failure):
    with service_process(tmp_path) as (process, request):
        status = request()
        child_pid = status['controller_owner']['pid']
        assert child_pid != process.pid
        request('open')
        confirm = request('prepare', {'operation': 'arm'})['confirmation_token']
        lease = request('arm', {'confirmation_token': confirm})['lease']
        confirm = request('prepare', {'operation': 'takeoff', 'lease': lease})['confirmation_token']
        request('takeoff', {'confirmation_token': confirm, 'lease': lease})
        process.send_signal(signal.SIGSTOP if failure == 'sigstop' else signal.SIGKILL)
        logpath = tmp_path / 'http.jsonl'
        wait_for(lambda: events(logpath)[-1].get('event') == 'controller_owner_stopped')
        final = verify_landing(logpath)
        assert final['fault_reason'] in ('controller_parent_timeout', 'controller_parent_disconnected')
        assert lease not in logpath.read_text() and confirm not in logpath.read_text()
        if failure == 'sigstop':
            process.send_signal(signal.SIGCONT)
            status = request()
            assert status['stopping'] and not status['connected']
            assert status['landing_confirmed']
            with pytest.raises(HTTPError) as rejected:
                request('keepalive', {'lease': lease})
            assert rejected.value.code == 409
        else:
            process.wait(timeout=2)


def test_lost_takeoff_reply_expires_without_retry_and_preserves_uncertainty(tmp_path, monkeypatch):
    import neo2_control.control_owner as module
    with owner(tmp_path) as runtime:
        runtime.stopped.set()
        runtime.thread.join(timeout=1)
        lease = authorize(runtime)
        confirmation = runtime.handle('prepare', {'operation': 'takeoff', 'lease': lease})['confirmation_token']
        receive = module._receive
        def delayed(channel):
            time.sleep(.35)
            return receive(channel)
        with monkeypatch.context() as patch:
            patch.setattr(module, '_receive', delayed)
            with pytest.raises(IntentRejected, match='owner_unavailable'):
                runtime.handle('takeoff', {'confirmation_token': confirmation, 'lease': lease})
        status = runtime.status()
        assert status['possible_airborne'] is True and not status['landing_confirmed']
        assert not status['connected']
        wait_for(lambda: events(tmp_path / 'owner.jsonl')[-1].get('event') == 'controller_owner_stopped')
        verify_landing(tmp_path / 'owner.jsonl')


def test_synthetic_control_crosses_ipc_and_expired_input_neutralizes_once(tmp_path):
    with owner(tmp_path) as runtime:
        lease = authorize(runtime)
        takeoff(runtime, lease)
        wait_for(lambda: runtime.status()['state'] == 'flying')
        runtime.handle('control', {'lease': lease, 'sequence': 1,
                                  'expires_at': time.monotonic() + .2,
                                  'channels': [100, -100, 50, -50]})
        time.sleep(.7)
        status = runtime.status()
        assert status['fault_reason'] == 'control_expired' and status['landing_confirmed']
        runtime.close()
        sent = packets(tmp_path / 'owner.jsonl')
        sticks = [p for p in sent if p.cmd_id == 0x0A]
        words = [int.from_bytes(p.payload[2:10], 'little') for p in sticks]
        values = [[(w >> shift) & 0x7FF for shift in (8, 19, 30, 41)] for w in words]
        assert values[0] == [1090, 958, 1057, 991]
        assert values[-1] == [1024] * 4
        assert sum(v == [1024] * 4 for v in values) == 1
        assert 1 <= len(values) - 1 <= 4
        assert [p.payload[0] for p in sent if p.cmd_id == 0x2A] == [1, 2]


def test_child_never_blocks_watchdog_on_full_parent_reply_queue(tmp_path):
    channel, process, ready = start_raw_owner(tmp_path)
    sequence = 0
    def rpc(action, parameters=None):
        nonlocal sequence
        sequence += 1
        _send(channel, {'kind': 'request', 'epoch': ready['epoch'], 'id': sequence,
                       'expires_at': time.monotonic() + .25, 'operation': 'action',
                       'parameters': {'action': action, 'parameters': parameters or {}}})
        reply = _receive(channel)
        assert reply['error'] is None
        return reply['result']
    try:
        rpc('open')
        confirmation = rpc('prepare', {'operation': 'arm'})['confirmation_token']
        lease = rpc('arm', {'confirmation_token': confirmation})['lease']
        confirmation = rpc('prepare', {'operation': 'takeoff', 'lease': lease})['confirmation_token']
        rpc('takeoff', {'confirmation_token': confirmation, 'lease': lease})
        # Deliberately stop reading. Bounded nonblocking replies must trigger
        # the stop path rather than trapping the owner in send().
        channel.setblocking(False)
        end = time.monotonic() + .15
        while time.monotonic() < end:
            sequence += 1
            try:
                _send(channel, {'kind': 'request', 'epoch': ready['epoch'], 'id': sequence,
                               'expires_at': time.monotonic() + .25,
                               'operation': 'heartbeat', 'parameters': {}})
            except (BlockingIOError, BrokenPipeError, ConnectionResetError):
                break
        process.wait(timeout=3)
        final = verify_landing(tmp_path / 'raw.jsonl')
        assert final['fault_reason'] == 'controller_ipc_failed'
    finally:
        channel.close()
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=2)


def test_close_consumes_already_queued_final_without_losing_confirmed_outcome(tmp_path):
    with owner(tmp_path) as runtime:
        runtime.stopped.set()
        runtime.thread.join(timeout=1)
        lease = authorize(runtime)
        takeoff(runtime, lease)
        # The parent sends no more heartbeat; final is queued before close().
        wait_for(lambda: events(tmp_path / 'owner.jsonl')[-1].get('event') == 'controller_owner_stopped')
        assert not runtime.closed and not runtime.final_received
        runtime.close()
        assert runtime.cleanup_verified and runtime.final_received
        assert runtime.failure is None
        status = runtime.status()
        assert status['landing_confirmed'] and not status['connected']
        assert not status['telemetry']['valid']
        assert status['fault_reason'] == 'controller_parent_timeout'
        verify_landing(tmp_path / 'owner.jsonl')


def test_expired_inflight_reply_preserves_already_queued_final_landing_proof(tmp_path, monkeypatch):
    import neo2_control.control_owner as module
    with owner(tmp_path) as runtime:
        runtime.stopped.set()
        runtime.thread.join(timeout=1)
        lease = authorize(runtime)
        takeoff(runtime, lease)
        receive = module._receive
        delayed_responses = []
        def suspended_parent(channel):
            reply = receive(channel)
            if channel is runtime.channel and reply.get('kind') == 'response':
                # Model SIGSTOP after an in-flight reply was produced: the
                # independent child completes landing and queues its final
                # before the parent's outstanding RPC can resume.
                delayed_responses.append(reply)
                wait_for(lambda: events(tmp_path / 'owner.jsonl')[-1].get('event')
                         == 'controller_owner_stopped')
                wait_for(lambda: runtime.process.poll() is not None)
            return reply
        with monkeypatch.context() as patch:
            patch.setattr(module, '_receive', suspended_parent)
            status = runtime.status()
        assert len(delayed_responses) == 1
        assert runtime.final_received and runtime.failure is None
        assert status['landing_confirmed'] and not status['possible_airborne']
        assert status['stopping'] and not status['connected'] and not status['telemetry']['valid']
        assert status['fault_reason'] == 'controller_parent_timeout'
        with pytest.raises(IntentRejected, match='owner_unavailable'):
            runtime.handle('keepalive', {'lease': lease})
        runtime.close()
        assert runtime.cleanup_verified
        verify_landing(tmp_path / 'owner.jsonl')
