"""Fake MediaMTX/process/HTTP boundaries; no aircraft or network association."""
import json
import os
import signal
import stat
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

from neo2_control import webrtc_bridge as module
from neo2_control.process_limits import LimitPolicyError


class Limits:
    def __init__(self):
        self.commands = []
        self.fail = False

    def wrap(self, command):
        self.commands.append(command)
        return ['bounded-launcher', *command]

    def check_memory(self, processes):
        if self.fail:
            raise LimitPolicyError('private synthetic monitor error')

    def snapshot(self):
        return {'memory_scope': 'each_owned_process'}


class Process:
    pid = 987654321

    def __init__(self):
        reader, self.writer = os.pipe()
        self.stdout = os.fdopen(reader, 'rb', buffering=0)
        self.returncode = None
        self.waits = []

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        self.waits.append(timeout)
        if self.returncode is None:
            raise subprocess.TimeoutExpired('fake-mediamtx', timeout)
        return self.returncode

    def finish(self, code=0):
        self.returncode = code
        if self.writer is not None:
            os.close(self.writer)
            self.writer = None


def until(predicate):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    assert predicate()


def configured(tmp_path, **options):
    return module.WebRtcBridge(tmp_path / 'bridge', process_limits=Limits(), **options)


def running(tmp_path):
    bridge = configured(tmp_path)
    bridge._ports = (10001, 10002, 10003)
    bridge._process = SimpleNamespace(poll=lambda: None)
    bridge._ready = True
    return bridge


def launch(tmp_path, monkeypatch, *, ignore_term=False, limits=None):
    binary = tmp_path / 'mediamtx'
    binary.write_text('synthetic executable placeholder')
    binary.chmod(0o700)
    process = Process()
    calls = []
    def factory(command, **options):
        calls.append((command, options))
        return process
    signals = []
    def killpg(pid, signum):
        assert pid == process.pid
        signals.append(signum)
        if signum != signal.SIGTERM or not ignore_term:
            process.finish(-signum)
    monkeypatch.setattr(module.subprocess, 'Popen', factory)
    monkeypatch.setattr(module.os, 'killpg', killpg)
    bridge = module.WebRtcBridge(tmp_path / 'bridge', binary=binary,
                                 process_limits=limits or Limits())
    monkeypatch.setattr(bridge, '_reserve_ports', lambda: ([], (10001, 10002, 10003)))
    monkeypatch.setattr(bridge, '_listeners_open', lambda: True)
    return bridge, process, calls, signals


def test_construction_and_snapshot_are_inert(tmp_path):
    bridge = configured(tmp_path)
    snapshot = bridge.snapshot()
    assert not bridge.directory.exists()
    assert snapshot['state'] == 'idle' and snapshot['cleanup_verified']
    assert not snapshot['ready'] and len(snapshot['generation']) == 24
    with pytest.raises(module.WebRtcBridgeError):
        _ = bridge.rtsp_url
    assert bridge.close()['cleanup_verified']


@pytest.mark.parametrize('ports', [(True, 0, 0), (-1, 0, 0), (65536, 0, 0),
    (8554, 8554, 8189), (8554, 8889, 8554)])
def test_invalid_or_conflicting_ports_rejected(tmp_path, ports):
    with pytest.raises(ValueError):
        configured(tmp_path, rtsp_port=ports[0], whep_port=ports[1], ice_udp_port=ports[2])


def test_config_is_loopback_only_with_no_other_protocols_or_external_services(tmp_path):
    bridge = configured(tmp_path)
    bridge._ports = (10001, 10002, 10003)
    config = bridge._configuration()
    assert config['rtspAddress'] == '127.0.0.1:10001'
    assert config['webrtcAddress'] == '127.0.0.1:10002'
    assert config['webrtcLocalUDPAddress'] == '127.0.0.1:10003'
    assert config['rtspTransports'] == ['tcp'] and config['rtspEncryption'] == 'no'
    assert config['webrtcLocalTCPAddress'] == ''
    assert not config['webrtcIPsFromInterfaces']
    assert config['webrtcAdditionalHosts'] == ['127.0.0.1']
    assert config['webrtcICEServers2'] == [] and config['webrtcAllowOrigins'] == []
    assert all(config[key] is False for key in ('api', 'metrics', 'pprof', 'playback',
                                               'rtmp', 'hls', 'srt', 'moq'))
    assert list(config['paths']) == ['preview']
    assert config['paths']['preview'] == {'source': 'publisher', 'maxReaders': 4}
    assert config['pathDefaults']['record'] is False and config['pathDefaults']['overridePublisher'] is False
    assert config['authInternalUsers'][0]['ips'] == ['127.0.0.1']
    assert config['authInternalUsers'][0]['permissions'] == [
        {'action': 'publish', 'path': 'preview'}, {'action': 'read', 'path': 'preview'}]


def test_start_creates_private_config_launches_bounded_process_and_reaps(tmp_path, monkeypatch):
    bridge, process, calls, signals = launch(tmp_path, monkeypatch)
    assert bridge.start() is bridge and bridge.ready
    assert bridge.rtsp_url == 'rtsp://127.0.0.1:10001/preview'
    path = bridge.directory / 'mediamtx.yml'
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert stat.S_IMODE(bridge.directory.stat().st_mode) == 0o700
    assert json.loads(path.read_text()) == bridge._configuration()
    command, options = calls[0]
    assert command == ['bounded-launcher', str(bridge.binary), str(path)]
    assert options['start_new_session'] and options['umask'] == 0o077
    assert options['stdin'] == subprocess.DEVNULL
    assert options['stderr'] == subprocess.STDOUT
    assert options['env'] == {'PATH': '/usr/bin:/bin', 'LC_ALL': 'C'}
    result = bridge.close()
    assert result['cleanup_verified'] and not result['ready']
    assert signals == [signal.SIGTERM] and process.stdout.closed
    assert bridge.close()['cleanup_verified']
    with pytest.raises(module.WebRtcBridgeError):
        bridge.start()


def test_missing_binary_fails_before_output_and_does_not_claim_running(tmp_path):
    bridge = configured(tmp_path, binary=tmp_path / 'missing')
    with pytest.raises(module.WebRtcBridgeError, match='binary_unavailable') as error:
        bridge.start()
    assert error.value.cleanup_verified and bridge.snapshot()['cleanup_verified']
    assert not bridge.directory.exists()


def test_cleanup_escalates_only_owned_process_after_bounded_wait(tmp_path, monkeypatch):
    bridge, process, _, signals = launch(tmp_path, monkeypatch, ignore_term=True)
    bridge.start()
    assert bridge.close()['cleanup_verified']
    assert signals == [signal.SIGTERM, signal.SIGKILL]
    assert process.waits[:2] == [1.5, 1]


def test_stdout_ring_is_bounded_and_never_exposed_in_snapshot(tmp_path, monkeypatch):
    bridge, process, _, _ = launch(tmp_path, monkeypatch)
    bridge.start()
    secret = b'private-sdp-or-session-location'
    data = secret * 4096
    writer = threading.Thread(target=lambda: os.write(process.writer, data))
    writer.start()
    until(lambda: bridge.snapshot()['log_bytes_received'] >= len(data))
    writer.join(1)
    snapshot = bridge.snapshot()
    assert snapshot['log_retained_bytes'] == module.MAX_LOG_BYTES
    assert secret.decode() not in json.dumps(snapshot)
    bridge.close()


def test_memory_failure_invalidates_bridge_and_reaps_process(tmp_path, monkeypatch):
    limits = Limits()
    bridge, process, _, _ = launch(tmp_path, monkeypatch, limits=limits)
    bridge.start()
    limits.fail = True
    until(lambda: process.poll() is not None)
    assert not bridge.ready
    result = bridge.close()
    assert result['cleanup_verified'] and result['error'] == 'webrtc_bridge_memory_limit_or_monitor_failed'


def test_offer_and_delete_use_only_owned_path_and_opaque_ids(tmp_path, monkeypatch):
    bridge = running(tmp_path)
    calls = []
    def request(method, path, body=None):
        calls.append((method, path, body))
        if method == 'POST':
            return 201, 'application/sdp; charset=utf-8', '/preview/whep/upstream-id', b'v=0\r\nprivate-answer'
        return 204, '', None, b''
    monkeypatch.setattr(bridge, '_request', request)
    result = bridge.offer(b'v=0\r\nprivate-offer')
    assert result['sdp'] == b'v=0\r\nprivate-answer'
    assert len(result['session_id']) == 48 and 'upstream-id' not in result['session_id']
    assert bridge.snapshot()['session_count'] == 1
    assert 'private' not in json.dumps(bridge.snapshot())
    bridge.delete(result['session_id'])
    assert bridge.snapshot()['session_count'] == 0
    assert calls == [('POST', '/preview/whep', b'v=0\r\nprivate-offer'),
                     ('DELETE', '/preview/whep/upstream-id', None)]


@pytest.mark.parametrize('location', ['http://example.com/preview/whep/id',
    'https://127.0.0.1:10002/preview/whep/id', '//example.com/preview/whep/id',
    'http://127.0.0.1:9999/preview/whep/id', 'http://user@127.0.0.1:10002/preview/whep/id',
    '/other/whep/id', '/preview/whep/id?target=external', '/preview/whep/id#fragment',
    '/preview/whep/../admin', '/preview/whep/%2fadmin', '/preview/whep/id/extra', None])
def test_rejects_untrusted_upstream_locations_before_registry(tmp_path, monkeypatch, location):
    bridge = running(tmp_path)
    monkeypatch.setattr(bridge, '_request', lambda *a: (201, 'application/sdp', location, b'v=0'))
    with pytest.raises(module.WebRtcBridgeError, match='invalid_whep_session_location'):
        bridge.offer(b'v=0')
    assert bridge.snapshot()['session_count'] == bridge.snapshot()['pending_offers'] == 0


def test_same_origin_absolute_location_is_normalized_to_owned_path(tmp_path):
    bridge = running(tmp_path)
    assert bridge._location('http://127.0.0.1:10002/preview/whep/a-b_c') == '/preview/whep/a-b_c'


@pytest.mark.parametrize('offer', [b'', b'x' * (module.MAX_SDP_BYTES + 1), b'v=0\0secret', 'v=0'])
def test_offer_bounds_fail_before_http(tmp_path, monkeypatch, offer):
    bridge = running(tmp_path)
    monkeypatch.setattr(bridge, '_request', lambda *a: pytest.fail('invalid offer reached HTTP'))
    with pytest.raises(module.WebRtcBridgeError, match='invalid_whep_offer'):
        bridge.offer(offer)


@pytest.mark.parametrize('status,kind,body', [(302, 'application/sdp', b'v=0'),
    (200, 'application/sdp', b'v=0'), (201, 'text/html', b'private-error'),
    (201, 'application/sdp', b''), (201, 'application/sdp', b'\0')])
def test_rejected_answers_never_create_session_or_echo_payload(tmp_path, monkeypatch, status, kind, body):
    bridge = running(tmp_path)
    monkeypatch.setattr(bridge, '_request', lambda *a: (status, kind, '/preview/whep/id', body))
    with pytest.raises(module.WebRtcBridgeError) as error:
        bridge.offer(b'v=0')
    assert 'private-error' not in str(error.value)
    assert bridge.snapshot()['session_count'] == bridge.snapshot()['pending_offers'] == 0


def test_maximum_sessions_and_wrong_generation_delete_are_local_only(tmp_path, monkeypatch):
    bridge = running(tmp_path)
    calls = []
    def request(*args):
        calls.append(args)
        return 201, 'application/sdp', f'/preview/whep/id-{len(calls)}', b'v=0'
    monkeypatch.setattr(bridge, '_request', request)
    ids = [bridge.offer(b'v=0')['session_id'] for _ in range(4)]
    with pytest.raises(module.WebRtcBridgeError, match='limit_reached'):
        bridge.offer(b'v=0')
    other = running(tmp_path)
    with pytest.raises(module.WebRtcBridgeError, match='not_found'):
        other.delete(ids[0])
    assert len(calls) == 4
    assert bridge.snapshot()['maximum_sessions'] == 4


def test_failed_delete_retains_owned_session_for_retry(tmp_path, monkeypatch):
    bridge = running(tmp_path)
    monkeypatch.setattr(bridge, '_request', lambda *a: (201, 'application/sdp', '/preview/whep/id', b'v=0'))
    identifier = bridge.offer(b'v=0')['session_id']
    monkeypatch.setattr(bridge, '_request', lambda *a: (500, '', None, b''))
    with pytest.raises(module.WebRtcBridgeError, match='delete_failed'):
        bridge.delete(identifier)
    assert bridge.snapshot()['session_count'] == 1


class Response:
    def __init__(self, *, status=201, body=b'v=0', headers=None):
        self.status, self.body = status, body
        self.headers = headers or {'Content-Type': 'application/sdp', 'Location': '/preview/whep/id'}
        self.read_sizes = []

    def getheader(self, name, default=None):
        return self.headers.get(name, default)

    def read(self, maximum):
        self.read_sizes.append(maximum)
        return self.body[:maximum]

    def close(self): pass


def test_http_connection_is_fixed_loopback_bounded_no_redirect_or_proxy(tmp_path, monkeypatch):
    bridge = running(tmp_path)
    response = Response()
    connections = []
    class Connection:
        sock = None
        def __init__(self, host, port, timeout):
            self.args = host, port, timeout
            self.requests = []
            self.closed = False
            connections.append(self)
        def request(self, method, path, body, headers):
            self.requests.append((method, path, body, headers))
        def connect(self): pass
        def getresponse(self): return response
        def close(self): self.closed = True
    monkeypatch.setattr(module.http.client, 'HTTPConnection', Connection)
    answer = bridge.offer(b'private-offer')
    assert answer['sdp'] == b'v=0'
    assert connections[0].args == ('127.0.0.1', 10002, 3.0)
    assert connections[0].requests == [('POST', '/preview/whep', b'private-offer', {'Content-Type': 'application/sdp'})]
    assert connections[0].closed and response.read_sizes == [module.MAX_SDP_BYTES + 1]


@pytest.mark.parametrize('length,body', [('65537', b''), ('-1', b''), ('bad', b''),
    (None, b'x' * (module.MAX_SDP_BYTES + 1))])
def test_http_response_bound_checked_without_logging_body(tmp_path, monkeypatch, length, body):
    bridge = running(tmp_path)
    headers = {'Content-Type': 'application/sdp', 'Location': '/preview/whep/id'}
    if length is not None: headers['Content-Length'] = length
    response = Response(headers=headers, body=body)
    class Connection:
        sock = None
        def __init__(self, *a, **k): pass
        def connect(self): pass
        def request(self, *a, **k): pass
        def getresponse(self): return response
        def close(self): pass
    monkeypatch.setattr(module.http.client, 'HTTPConnection', Connection)
    with pytest.raises(module.WebRtcBridgeError, match='too_large'):
        bridge.offer(b'v=0')


def test_ready_false_rejects_offer_before_network(tmp_path, monkeypatch):
    bridge = configured(tmp_path)
    monkeypatch.setattr(bridge, '_request', lambda *a: pytest.fail('not ready'))
    with pytest.raises(module.WebRtcBridgeError, match='not_ready'):
        bridge.offer(b'v=0')
