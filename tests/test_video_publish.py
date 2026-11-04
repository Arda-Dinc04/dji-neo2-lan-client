"""Private config, atomic HLS reads and real loopback HTTPS publication."""
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hashlib
import json
import os
from pathlib import Path
import ssl
import subprocess
import threading
from urllib.parse import parse_qs, urlsplit

import pytest

from neo2_control import video_publish as module
from neo2_control.video_publish import (FORMAT, HttpsVideoClient, ManagedPublisher,
    PublishConfig, PublishError, manifest_segments, publication_succeeded)
from test_observation import until

TOKEN = 'synthetic-local-publisher-token-' + 'a' * 32


@pytest.fixture(scope='module')
def certificate(tmp_path_factory):
    root = tmp_path_factory.mktemp('publisher-tls')
    cert, key = root / 'ca.pem', root / 'key.pem'
    subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
        '-keyout', str(key), '-out', str(cert), '-days', '1', '-subj', '/CN=localhost',
        '-addext', 'subjectAltName=DNS:localhost,IP:127.0.0.1'],
        check=True, capture_output=True, timeout=10)
    return cert, key


class HttpsPeer:
    def __init__(self, certificate):
        self.calls = []
        self.response = lambda method, path, body: (200, {'sessionId': 'local-session', 'live': True}, {})
        self.lock = threading.Lock()
        owner = self
        class Handler(BaseHTTPRequestHandler):
            def handle_request(self):
                body = self.rfile.read(int(self.headers.get('Content-Length', '0')))
                with owner.lock:
                    owner.calls.append({'method': self.command, 'path': self.path, 'body': body,
                        'authorization': self.headers.get('Authorization')})
                status, data, headers = owner.response(self.command, self.path, body)
                raw = data if isinstance(data, bytes) else json.dumps(data).encode()
                try:
                    self.send_response(status)
                    for key, value in headers.items():
                        self.send_header(key, value)
                    self.send_header('Content-Length', str(len(raw)))
                    self.end_headers()
                    self.wfile.write(raw)
                except (BrokenPipeError, ConnectionResetError, ssl.SSLError):
                    pass
            do_POST = handle_request
            do_PUT = handle_request
            def log_message(self, *args): pass
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.server.daemon_threads = True
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(str(certificate[0]), str(certificate[1]))
        self.server.socket = context.wrap_socket(self.server.socket, server_side=True)
        self.url = f'https://localhost:{self.server.server_port}/api'
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)


@pytest.fixture
def remote(certificate):
    instance = HttpsPeer(certificate)
    yield instance
    instance.close()


def profile(tmp_path, remote, certificate, **changes):
    path = tmp_path / 'publish-private.json'
    record = {'format': FORMAT, 'server': remote.url, 'agent_id': 'pc-test',
              'token': TOKEN, 'ca_file': str(certificate[0])} | changes
    path.write_text(json.dumps(record))
    path.chmod(0o600)
    return path


def playlist(directory, *, count=4, start=0, duration=1.0):
    directory.mkdir()
    lines = ['#EXTM3U', '#EXT-X-VERSION:6', '#EXT-X-TARGETDURATION:1',
             '#EXT-X-MEDIA-SEQUENCE:' + str(start)]
    for sequence in range(start, start + count):
        name = f'segment-{sequence:08d}.ts'
        (directory / name).write_bytes(b'bounded-test-segment-' + str(sequence).encode())
        lines.extend([f'#EXTINF:{duration},', name])
    (directory / 'index.m3u8').write_text('\n'.join(lines) + '\n')
    return directory


def test_actual_https_contract_and_private_error_strings(tmp_path, remote, certificate):
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    client = HttpsVideoClient(config)
    assert client.context.check_hostname and client.context.verify_mode == ssl.CERT_REQUIRED
    client.start(30)
    client.upload(3, .4, b'actual-HTTPS-body')
    client.stop()
    assert client.remote_stop_acknowledged
    assert [c['method'] for c in remote.calls] == ['POST', 'PUT', 'POST']
    assert remote.calls[1]['path'].endswith('/segments/3?duration=0.400')
    assert all(c['authorization'] == 'Bearer ' + TOKEN for c in remote.calls)
    assert TOKEN not in repr(config) and TOKEN not in json.dumps(config.metadata())
    assert all(TOKEN not in c['path'] for c in remote.calls)


def test_atomic_manifest_worker_uploads_recent_window_and_stops(tmp_path, remote, certificate):
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    directory = playlist(tmp_path / 'hls')
    publisher = ManagedPublisher(config, directory).start()
    try:
        until(lambda: publisher.poll()['segments_uploaded'] == 3, timeout=5)
        final = publisher.close()
    finally:
        publisher.close()
    assert publication_succeeded(final)
    assert final['segments_dropped'] == 1 and final['last_sequence'] == 3
    assert final['remote_session_sha256'] == hashlib.sha256(b'local-session').hexdigest()
    assert final['last_segment_sha256'] == hashlib.sha256((directory / 'segment-00000003.ts').read_bytes()).hexdigest()
    assert 'local-session' not in json.dumps(final)
    assert not final['worker_alive'] and final['cleanup_verified']
    uploads = [c for c in remote.calls if c['method'] == 'PUT']
    assert [int(urlsplit(c['path']).path.rsplit('/', 1)[-1]) for c in uploads] == [1, 2, 3]
    assert remote.calls[-1]['path'].endswith('/stop')
    assert TOKEN not in json.dumps(final) and str(config.path) not in json.dumps(final)
    with pytest.raises(PublishError, match='cannot_be_reused'):
        publisher.start()


def test_no_media_opens_no_remote_session(tmp_path, remote, certificate):
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    directory = tmp_path / 'hls'
    directory.mkdir()
    publisher = ManagedPublisher(config, directory).start()
    final = publisher.close()
    assert not remote.calls
    assert final['cleanup_verified'] and final['remote_stop_acknowledged']
    assert not publication_succeeded(final)


@pytest.mark.parametrize('change', [
    {'server': 'http://localhost/api'}, {'server': 'https://user:secret@localhost/api'},
    {'server': 'https://localhost/api?secret=1'}, {'server': 'https://localhost/api#fragment'},
    {'server': 'https://localhost:99999/api'}, {'token': 'line\nbreak'}, {'token': ''},
    {'agent_id': ''}, {'format': 'other'}, {'unexpected': True},
])
def test_invalid_private_config_is_sanitized(tmp_path, remote, certificate, change):
    path = profile(tmp_path, remote, certificate, **change)
    with pytest.raises(PublishError) as failure:
        PublishConfig.load(path)
    assert str(failure.value) == 'publish_config_invalid_or_unavailable'
    assert not remote.calls


@pytest.mark.parametrize('problem', ['public', 'symlink', 'directory', 'fifo', 'oversized'])
def test_nonprivate_or_nonregular_config_is_refused(tmp_path, remote, certificate, problem):
    path = profile(tmp_path, remote, certificate)
    if problem == 'public':
        path.chmod(0o644)
    elif problem == 'symlink':
        alias = tmp_path / 'alias'
        alias.symlink_to(path)
        path = alias
    elif problem == 'directory':
        path = tmp_path
    elif problem == 'fifo':
        path.unlink()
        os.mkfifo(path)
    else:
        path.write_bytes(b'x' * (module.MAX_CONFIG_BYTES + 1))
    with pytest.raises(PublishError):
        PublishConfig.load(path)
    assert not remote.calls


@pytest.mark.parametrize('hardlink', [False, True])
def test_config_aliases_are_rejected_before_any_network(tmp_path, remote, certificate, hardlink):
    path = profile(tmp_path, remote, certificate)
    output = tmp_path / 'capture' if hardlink else path
    if hardlink:
        output.hardlink_to(path)
    with pytest.raises(PublishError):
        PublishConfig.load(path, protected_paths=[output])
    assert not remote.calls


def test_config_cannot_be_nested_in_hls_output(tmp_path, remote, certificate):
    directory = tmp_path / 'hls'
    directory.mkdir()
    path = profile(directory, remote, certificate)
    with pytest.raises(PublishError):
        PublishConfig.load(path, protected_paths=[directory])
    assert not remote.calls


def test_validated_config_cannot_change_before_worker_start(tmp_path, remote, certificate):
    path = profile(tmp_path, remote, certificate)
    config = PublishConfig.load(path)
    record = json.loads(path.read_text())
    record['agent_id'] = 'changed-local-target'
    path.write_text(json.dumps(record))
    publisher = ManagedPublisher(config, playlist(tmp_path / 'hls')).start()
    try:
        until(lambda: publisher.poll()['state'] == 'failed', timeout=5)
    finally:
        final = publisher.close()
    assert final['error'] == 'publish_config_changed' and not remote.calls


@pytest.mark.parametrize('lines', [
    ['garbage'],
    ['#EXTM3U', '#EXT-X-MEDIA-SEQUENCE:0', '#EXTINF:nan,', 'segment-00000000.ts'],
    ['#EXTM3U', '#EXT-X-MEDIA-SEQUENCE:0', '#EXTINF:6,', 'segment-00000000.ts'],
    ['#EXTM3U', '#EXT-X-MEDIA-SEQUENCE:0', '#EXTINF:1,', '../secret'],
    ['#EXTM3U', '#EXT-X-MEDIA-SEQUENCE:0', '#EXTINF:1,', 'https://foreign/secret.ts'],
    ['#EXTM3U', '#EXT-X-MEDIA-SEQUENCE:0', '#EXTINF:1,', 'segment-00000001.ts'],
    ['#EXTM3U', '#EXT-X-MEDIA-SEQUENCE:0', '#EXTINF:1,', '#EXTINF:2,', 'segment-00000000.ts'],
    ['#EXTM3U', '#EXT-X-MEDIA-SEQUENCE:0', '#EXT-X-MEDIA-SEQUENCE:1'],
])
def test_manifest_rejects_unbounded_durations_uris_or_inconsistent_sequence(tmp_path, lines):
    (tmp_path / 'index.m3u8').write_text('\n'.join(lines) + '\n')
    with pytest.raises(PublishError):
        manifest_segments(tmp_path)


def test_symlink_segment_is_never_exfiltrated(tmp_path, remote, certificate):
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    directory = playlist(tmp_path / 'hls', count=1)
    (directory / 'segment-00000000.ts').unlink()
    (directory / 'segment-00000000.ts').symlink_to(config.path)
    publisher = ManagedPublisher(config, directory).start()
    try:
        until(lambda: publisher.poll()['state'] == 'failed', timeout=5)
    finally:
        final = publisher.close()
    assert final['error'] == 'publish_input_or_worker_error' and not remote.calls


@pytest.mark.parametrize('status', [302, 401, 500])
def test_response_bodies_and_redirects_cannot_leak_token(tmp_path, remote, certificate, status):
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    remote.response = lambda *a: (status, {'echoed_secret': TOKEN}, {'Location': 'https://foreign.invalid/'})
    client = HttpsVideoClient(config)
    with pytest.raises(PublishError) as failure:
        client.start(30)
    assert str(failure.value) == 'publish_http_' + str(status)
    assert TOKEN not in str(failure.value) and len(remote.calls) == 1
    assert not client.remote_stop_acknowledged


def test_untrusted_certificate_sends_no_authenticated_request(tmp_path, remote, certificate):
    path = profile(tmp_path, remote, certificate)
    record = json.loads(path.read_text())
    del record['ca_file']
    path.write_text(json.dumps(record))
    with pytest.raises(PublishError, match='publish_tls_verification_failed'):
        HttpsVideoClient(PublishConfig.load(path)).start(30)
    assert not remote.calls


def test_retry_only_put_with_same_sequence_and_real_manifest_duration(tmp_path, remote, certificate):
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    attempts = []
    def response(method, path, body):
        if method == 'PUT':
            attempts.append((path, body))
            if len(attempts) == 1:
                return 503, {'echo': TOKEN}, {}
        return 200, {'sessionId': 'local-session'}, {}
    remote.response = response
    publisher = ManagedPublisher(config, playlist(tmp_path / 'hls', count=1, duration=.4)).start()
    try:
        until(lambda: publisher.poll()['segments_uploaded'] == 1, timeout=5)
    finally:
        final = publisher.close()
    assert final['retries'] == 1 and attempts[0] == attempts[1]
    assert attempts[0][0].endswith('duration=0.400')
    assert sum(c['path'].endswith('/drone/sessions') for c in remote.calls) == 1
    assert publication_succeeded(final)


def test_uncertain_creation_is_never_retried_and_remote_stop_is_not_claimed(tmp_path, remote, certificate):
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    remote.response = lambda *a: (200, b'not JSON', {})
    publisher = ManagedPublisher(config, playlist(tmp_path / 'hls')).start()
    try:
        until(lambda: publisher.poll()['state'] == 'failed', timeout=5)
    finally:
        final = publisher.close()
    assert len(remote.calls) == 1 and final['cleanup_verified']
    assert not final['remote_stop_acknowledged'] and not publication_succeeded(final)


def test_hung_request_is_killed_with_bounded_close_and_no_false_stop_ack(tmp_path, remote, certificate, monkeypatch):
    release = threading.Event()
    def response(*args):
        release.wait(5)
        return 200, {'sessionId': 'late-session'}, {}
    remote.response = response
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    publisher = ManagedPublisher(config, playlist(tmp_path / 'hls')).start()
    try:
        until(lambda: bool(remote.calls), timeout=5)
        monkeypatch.setattr(module, 'CLOSE_TIMEOUT', .1)
        final = publisher.close()
        assert final['error'] == 'publish_close_timeout'
        assert final['cleanup_verified'] and not final['remote_stop_acknowledged']
        assert publisher.process.poll() is not None
    finally:
        release.set()
        publisher.close()


def test_pipe_startup_failure_closes_owned_descriptors(tmp_path, remote, certificate, monkeypatch):
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    original = module.os.pipe
    opened = []
    def pipe():
        if opened:
            raise OSError('descriptor limit')
        pair = original()
        opened.extend(pair)
        return pair
    monkeypatch.setattr(module.os, 'pipe', pipe)
    publisher = ManagedPublisher(config, tmp_path / 'hls').start()
    assert publisher.snapshot()['error'] == 'publish_worker_start_failed'
    assert publisher.snapshot()['cleanup_verified']
    for descriptor in opened:
        with pytest.raises(OSError):
            os.fstat(descriptor)


def test_large_response_is_bounded_without_echoing_remote_content(tmp_path, remote, certificate):
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    remote.response = lambda *a: (200, b'x' * (module.MAX_RESPONSE_BYTES + 1), {})
    with pytest.raises(PublishError, match='publish_response_too_large'):
        HttpsVideoClient(config).start(30)


def test_stop_failure_is_separate_from_local_process_cleanup(tmp_path, remote, certificate):
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    def response(method, path, body):
        return (503, {'echo': TOKEN}, {}) if path.endswith('/stop') else (200, {'sessionId': 'local-session'}, {})
    remote.response = response
    publisher = ManagedPublisher(config, playlist(tmp_path / 'hls', count=1)).start()
    try:
        until(lambda: publisher.poll()['segments_uploaded'] == 1, timeout=5)
    finally:
        final = publisher.close()
    assert final['cleanup_verified'] and not final['remote_stop_acknowledged']
    assert final['error'] == 'publish_http_503' and not publication_succeeded(final)
    assert sum(c['path'].endswith('/stop') for c in remote.calls) == 1


def test_worker_does_not_inherit_account_tokens_or_put_them_in_argv(tmp_path, remote, certificate, monkeypatch):
    monkeypatch.setenv('HOME_AGENT_REMOTE_TOKEN', 'unrelated-account-secret')
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    publisher = ManagedPublisher(config, playlist(tmp_path / 'hls', count=1)).start()
    try:
        until(lambda: publisher.poll()['segments_uploaded'] == 1, timeout=5)
        proc = Path('/proc') / str(publisher.process.pid)
        environment = (proc / 'environ').read_bytes()
        command = (proc / 'cmdline').read_bytes()
        assert TOKEN.encode() not in command + environment
        assert b'HOME_AGENT_REMOTE_TOKEN' not in environment
        limits = (proc / 'limits').read_text()
        assert '402653184' in limits
    finally:
        publisher.close()


def test_pending_history_is_bounded_and_old_media_is_counted_as_dropped(tmp_path, remote, certificate):
    config = PublishConfig.load(profile(tmp_path, remote, certificate))
    publisher = ManagedPublisher(config, playlist(tmp_path / 'hls', start=10000, count=8)).start()
    try:
        until(lambda: publisher.poll()['segments_uploaded'] == 3, timeout=5)
    finally:
        final = publisher.close()
    assert final['segments_dropped'] == 10005
    assert final['last_sequence'] == 10007
    assert len([c for c in remote.calls if c['method'] == 'PUT']) == 3


def test_parent_death_terminates_owned_network_worker(tmp_path, remote, certificate):
    config = profile(tmp_path, remote, certificate)
    directory = playlist(tmp_path / 'hls', count=1)
    script = (
        "import sys,time\n"
        "from neo2_control.video_publish import PublishConfig,ManagedPublisher\n"
        "p=ManagedPublisher(PublishConfig.load(sys.argv[1]),sys.argv[2]).start()\n"
        "print(p.process.pid,flush=True)\n"
        "time.sleep(30)\n"
    )
    owner = subprocess.Popen([os.sys.executable, '-c', script, str(config), str(directory)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert select_readable(owner.stdout, 5)
        worker_pid = int(owner.stdout.readline())
        until(lambda: any(c['method'] == 'PUT' for c in remote.calls), timeout=5)
        owner.kill()
        owner.wait(timeout=3)
        def terminated():
            try:
                status = (Path('/proc') / str(worker_pid) / 'stat').read_text()
            except FileNotFoundError:
                return True
            return status.rsplit(')', 1)[1].split()[0] in ('Z', 'X')
        until(terminated, timeout=5)
    finally:
        if owner.poll() is None:
            owner.kill()
            owner.wait(timeout=3)
        owner.stdout.close()
        owner.stderr.close()


def select_readable(stream, timeout):
    import select
    return bool(select.select([stream], [], [], timeout)[0])
