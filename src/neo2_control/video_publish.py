"""Session-owned HTTPS publication of finalized HLS segments.

The network worker is a separate owned process: DNS, TLS or a trickling
response cannot indefinitely block the UDP owner or its cleanup. Tokens stay
in a private local config file and HTTPS headers, outside argv and snapshots.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import ctypes
import hashlib
import http.client
import json
import math
import os
from pathlib import Path
import re
import select
import shutil
import signal
import ssl
import stat
import subprocess
import sys
import time
from urllib.parse import quote, urlencode, urlsplit

MAX_SEGMENT_BYTES = 2_500_000
MAX_RESPONSE_BYTES = 1_048_576
MAX_CONFIG_BYTES = 16384
MAX_MANIFEST_BYTES = 65536
FORMAT = 'neo2-control/video-publish/v1'
REQUEST_TIMEOUT = 2.0
CLOSE_TIMEOUT = 8.0
SEGMENT_RE = re.compile(r'segment-([0-9]{8})\.ts')


class PublishError(RuntimeError):
    def __init__(self, reason, *, retryable=False, cleanup_verified=True):
        super().__init__(reason)
        self.retryable, self.cleanup_verified = retryable, cleanup_verified


def publication_succeeded(snapshot):
    return (snapshot.get('enabled') is True and snapshot.get('error') is None
        and snapshot.get('segments_uploaded', 0) > 0 and snapshot.get('cleanup_verified') is True
        and snapshot.get('remote_stop_acknowledged') is True)


def _read_regular(path, limit, *, private=False):
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(descriptor, 'rb') as source:
        info = os.fstat(source.fileno())
        if (not stat.S_ISREG(info.st_mode) or info.st_size > limit
            or (private and info.st_mode & 0o077)):
            raise ValueError('invalid private publication input')
        data = source.read(limit + 1)
    if len(data) > limit:
        raise ValueError('publication input exceeds bound')
    return data


@dataclass(frozen=True, repr=False)
class PublishConfig:
    path: Path
    server: str
    agent_id: str
    token: str = field(repr=False)
    fingerprint: str
    ca_data: str | None = field(default=None, repr=False)
    ca_path: Path | None = None

    @classmethod
    def load(cls, path, *, protected_paths=()):
        path = Path(path).absolute()
        try:
            raw = _read_regular(path, MAX_CONFIG_BYTES, private=True)
            record = json.loads(raw)
            if (not isinstance(record, dict) or set(record) - {'format', 'server', 'agent_id', 'token', 'ca_file'}
                or record.get('format') != FORMAT):
                raise ValueError
            server, agent, token = (record[key] for key in ('server', 'agent_id', 'token'))
            if (not all(isinstance(v, str) for v in (server, agent, token))
                or not 1 <= len(agent) <= 100 or not 1 <= len(token) <= 4096
                or any(not 33 <= ord(c) <= 126 for c in token)
                or len(server) > 2048 or token in server or token in agent):
                raise ValueError
            agent.encode('utf-8')
            parsed = urlsplit(server)
            if (parsed.scheme != 'https' or not parsed.hostname or parsed.username is not None
                or parsed.password is not None or parsed.query or parsed.fragment
                or not re.fullmatch(r'[A-Za-z0-9_.:-]+', parsed.hostname)
                or not re.fullmatch(r'/[A-Za-z0-9_/-]*|', parsed.path)
                or (parsed.port is not None and not 1 <= parsed.port <= 65535)):
                raise ValueError
            ca_path = record.get('ca_file')
            ca_data = None
            if ca_path is not None:
                if not isinstance(ca_path, str) or not ca_path:
                    raise ValueError
                ca_path = Path(ca_path)
                if not ca_path.is_absolute():
                    ca_path = path.parent / ca_path
                ca_path = ca_path.absolute()
                ca_data = _read_regular(ca_path, 262144).decode('ascii')
            config = cls(path, server.rstrip('/'), agent, token,
                hashlib.sha256(raw + (ca_data or '').encode()).hexdigest(), ca_data, ca_path)
            config.validate_paths(protected_paths)
            config.context()  # Reject invalid CA material before any radio.
            return config
        except (OSError, ValueError, TypeError, KeyError, UnicodeError, ssl.SSLError, RecursionError):
            raise PublishError('publish_config_invalid_or_unavailable') from None

    def validate_paths(self, protected_paths):
        inputs = [self.path] + ([self.ca_path] if self.ca_path is not None else [])
        for source in inputs:
            for protected in protected_paths:
                target = Path(protected)
                if (source.resolve().is_relative_to(target.resolve())
                    or target.resolve().is_relative_to(source.resolve())
                    or (source.exists() and target.exists() and source.samefile(target))):
                    raise ValueError('publication config must be separate from session outputs')

    def context(self):
        return ssl.create_default_context(cadata=self.ca_data)

    def metadata(self):
        return {'server_sha256': hashlib.sha256(self.server.encode()).hexdigest(),
                'agent_sha256': hashlib.sha256(self.agent_id.encode()).hexdigest()}


class HttpsVideoClient:
    """Existing agent endpoint contract, without redirects or response-body errors."""
    def __init__(self, config):
        self.config, self.context = config, config.context()
        self.session_id = None
        self.start_attempted = False
        self.remote_stop_acknowledged = True

    def _request(self, method, suffix, body, content_type, timeout=REQUEST_TIMEOUT):
        parsed = urlsplit(self.config.server)
        connection = http.client.HTTPSConnection(parsed.hostname, parsed.port,
            context=self.context, timeout=max(.05, min(REQUEST_TIMEOUT, timeout)))
        try:
            connection.request(method, parsed.path + suffix, body=body,
                headers={'Authorization': 'Bearer ' + self.config.token,
                    'Content-Type': content_type, 'User-Agent': 'neo2-control/video-publisher'})
            response = connection.getresponse()
            if not 200 <= response.status < 300:
                raise PublishError('publish_http_' + str(response.status),
                    retryable=response.status in (408, 429) or response.status >= 500)
            data = response.read(MAX_RESPONSE_BYTES + 1)
            if len(data) > MAX_RESPONSE_BYTES:
                raise PublishError('publish_response_too_large')
            result = json.loads(data or b'{}')
            if not isinstance(result, dict):
                raise ValueError
            return result
        except ssl.SSLCertVerificationError:
            raise PublishError('publish_tls_verification_failed') from None
        except (OSError, http.client.HTTPException):
            raise PublishError('publish_network_error', retryable=True) from None
        except (ValueError, RecursionError, UnicodeError):
            raise PublishError('publish_response_invalid') from None
        finally:
            connection.close()

    def start(self, fps):
        if self.start_attempted:
            raise PublishError('publish_session_already_attempted')
        if type(fps) not in (float, int) or not math.isfinite(fps) or not 1 <= fps <= 60:
            raise PublishError('publish_nominal_fps_invalid')
        self.start_attempted, self.remote_stop_acknowledged = True, False
        response = self._request('POST', '/agents/' + quote(self.config.agent_id, safe='')
            + '/drone/sessions', json.dumps({'codec': 'h264', 'width': None,
                'height': None, 'nominal_fps': fps}).encode(), 'application/json')
        identifier = response.get('sessionId')
        if (not isinstance(identifier, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', identifier)
            or self.config.token in identifier):
            raise PublishError('publish_session_response_invalid')
        self.session_id = identifier

    def upload(self, sequence, duration, body, *, timeout=REQUEST_TIMEOUT):
        if (self.session_id is None or type(sequence) is not int or sequence < 0
            or not math.isfinite(duration) or not 0 < duration <= 5
            or not 0 < len(body) <= MAX_SEGMENT_BYTES):
            raise PublishError('publish_segment_invalid')
        suffix = ('/agents/' + quote(self.config.agent_id, safe='') + '/drone/sessions/'
            + self.session_id + '/segments/' + str(sequence) + '?'
            + urlencode({'duration': f'{duration:.3f}'}))
        self._request('PUT', suffix, body, 'video/mp2t', timeout)

    def stop(self):
        if self.remote_stop_acknowledged:
            return
        if self.session_id is not None:
            self._request('POST', '/agents/' + quote(self.config.agent_id, safe='')
                + '/drone/sessions/' + self.session_id + '/stop', b'', 'application/octet-stream')
            self.remote_stop_acknowledged = True
        elif not self.start_attempted:
            self.remote_stop_acknowledged = True


def manifest_segments(directory):
    """Only committed MPEG-TS entries from our atomic, bounded HLS playlist."""
    try:
        text = _read_regular(Path(directory) / 'index.m3u8', MAX_MANIFEST_BYTES).decode('utf-8')
    except FileNotFoundError:
        return []
    entries, duration, expected = [], None, None
    lines = text.splitlines()
    if not lines or lines[0] != '#EXTM3U':
        raise PublishError('publish_manifest_invalid')
    try:
        for line in lines[1:]:
            if line.startswith('#EXT-X-MEDIA-SEQUENCE:'):
                if expected is not None or entries:
                    raise ValueError
                expected = int(line.split(':', 1)[1])
                if not 0 <= expected <= 99_999_999:
                    raise ValueError
            elif line.startswith('#EXTINF:'):
                if duration is not None:
                    raise ValueError
                duration = float(line.split(':', 1)[1].split(',', 1)[0])
                if not math.isfinite(duration) or not 0 < duration <= 5:
                    raise ValueError
            elif line and not line.startswith('#'):
                match = SEGMENT_RE.fullmatch(line)
                if match is None or duration is None or expected is None or int(match[1]) != expected:
                    raise ValueError
                entries.append((expected, duration, line))
                duration, expected = None, expected + 1
                if len(entries) > 8:
                    raise ValueError
        if duration is not None:
            raise ValueError
        return entries
    except ValueError:
        raise PublishError('publish_manifest_invalid') from None


def _guard_parent(expected):
    # Called inside the new process, never in a threaded preexec_fn.
    if sys.platform != 'linux' or ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
        raise PublishError('publish_parent_guard_unavailable')
    if os.getppid() != expected:
        raise PublishError('publish_parent_exited')


def worker(config_path, fingerprint, directory, fps, status_fd, stop_fd, parent_pid):
    os.set_blocking(status_fd, False)
    os.set_blocking(stop_fd, False)
    state = {'state': 'connecting', 'error': None, 'last_error': None,
        'segments_uploaded': 0, 'segments_dropped': 0, 'retries': 0,
        'bytes_uploaded': 0, 'last_sequence': None, 'last_uploaded_at': None,
        'remote_stop_acknowledged': True, 'remote_session_sha256': None,
        'last_segment_sha256': None}
    client = None
    def emit(*, final=False):
        state['observed_at'] = time.monotonic()
        state['worker_finished'] = final
        data = json.dumps(state, separators=(',', ':'), allow_nan=False).encode() + b'\n'
        if len(data) > 4096:
            raise PublishError('publish_worker_status_limit')
        deadline = time.monotonic() + (1 if final else 0)
        while True:
            try:
                os.write(status_fd, data)
                return
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    return
                select.select([], [status_fd], [], .025)
    try:
        _guard_parent(parent_pid)
        if not math.isfinite(fps) or not 1 <= fps <= 60:
            raise PublishError('publish_nominal_fps_invalid')
        config = PublishConfig.load(config_path)
        if config.fingerprint != fingerprint:
            raise PublishError('publish_config_changed')
        client = HttpsVideoClient(config)
        closing_at, high_watermark = None, -1
        emit()
        while True:
            if closing_at is None and select.select([stop_fd], [], [], 0)[0]:
                os.read(stop_fd, 16)
                closing_at = time.monotonic()
            entries = [e for e in manifest_segments(directory) if e[0] > high_watermark]
            if not entries and closing_at is not None:
                break
            if closing_at is not None and time.monotonic() >= closing_at + 3:
                break
            # Keep recent video; never accumulate an unbounded history/backlog.
            entries = entries[-3:]
            for sequence, duration, name in entries:
                state['segments_dropped'] += sequence - high_watermark - 1
                high_watermark = sequence
                try:
                    body = _read_regular(Path(directory) / name, MAX_SEGMENT_BYTES)
                except FileNotFoundError:
                    state['segments_dropped'] += 1
                    continue
                if client.session_id is None:
                    # A POST with an uncertain result is never retried.
                    client.start(fps)
                    state['remote_session_sha256'] = hashlib.sha256(client.session_id.encode()).hexdigest()
                for attempt in range(2):
                    try:
                        client.upload(sequence, duration, body)
                        state.update(state='live', error=None, segments_uploaded=state['segments_uploaded'] + 1,
                            bytes_uploaded=state['bytes_uploaded'] + len(body), last_sequence=sequence,
                            last_uploaded_at=time.monotonic(), last_segment_sha256=hashlib.sha256(body).hexdigest())
                        break
                    except PublishError as failure:
                        state['last_error'] = str(failure)
                        if not failure.retryable:
                            raise
                        if attempt == 0:
                            state['retries'] += 1
                            select.select([stop_fd], [], [], .15)
                        else:
                            state.update(state='stale', segments_dropped=state['segments_dropped'] + 1)
                state['remote_stop_acknowledged'] = client.remote_stop_acknowledged
                emit()
                if closing_at is not None and time.monotonic() >= closing_at + 3:
                    break
            select.select([stop_fd], [], [], .1)
    except PublishError as failure:
        state.update(state='failed', error=str(failure))
    except (OSError, ValueError, UnicodeError, RecursionError):
        state.update(state='failed', error='publish_input_or_worker_error')
    except BaseException:
        state.update(state='failed', error='publish_worker_failed')
    finally:
        if client is not None:
            try:
                client.stop()
            except PublishError as failure:
                state.update(state='failed', error=state['error'] or str(failure))
            state['remote_stop_acknowledged'] = client.remote_stop_acknowledged
        if state['state'] != 'failed':
            state['state'] = 'offline'
        try:
            emit(final=True)
        except (OSError, PublishError):
            pass
    return 0 if state['state'] != 'failed' else 2


class ManagedPublisher:
    """One bounded process and two small nonblocking control/status pipes."""
    def __init__(self, config, directory, *, nominal_fps=30):
        if (type(nominal_fps) not in (float, int) or not math.isfinite(nominal_fps)
            or not 1 <= nominal_fps <= 60):
            raise ValueError('publication nominal FPS must be in [1,60]')
        self.config, self.directory, self.fps = config, Path(directory).absolute(), nominal_fps
        self.process = None
        self.status_fd = self.stop_fd = None
        self.buffer = bytearray()
        self.closed = False
        self.closing = False
        self.data = {'enabled': True, 'state': 'connecting', 'error': None,
            'last_error': None, 'segments_uploaded': 0, 'segments_dropped': 0,
            'bytes_uploaded': 0, 'retries': 0, 'last_sequence': None,
            'last_uploaded_at': None, 'remote_stop_acknowledged': True, **config.metadata()}
        self.data.update(remote_session_sha256=None, last_segment_sha256=None)

    def start(self):
        if self.closed or self.process is not None:
            raise PublishError('publish_worker_cannot_be_reused')
        status_write = stop_read = None
        try:
            status_read, status_write = os.pipe()
            self.status_fd = status_read
            stop_read, stop_write = os.pipe()
            self.stop_fd = stop_write
            os.set_blocking(status_read, False)
            os.set_blocking(stop_write, False)
            limiter = shutil.which('prlimit')
            if limiter is None:
                raise OSError
            command = [limiter, '--as=402653184', '--nofile=64', '--', sys.executable,
                '-m', 'neo2_control.video_publish', '--worker-config', str(self.config.path),
                '--fingerprint', self.config.fingerprint, '--directory', str(self.directory),
                '--fps', str(self.fps), '--status-fd', str(status_write),
                '--stop-fd', str(stop_read), '--parent-pid', str(os.getpid())]
            self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                pass_fds=(status_write, stop_read), start_new_session=True,
                env={'PATH': os.environ.get('PATH', '/usr/bin:/bin'), 'LC_ALL': 'C'})
        except (OSError, ValueError):
            self.data.update(state='failed', error='publish_worker_start_failed')
            self.close()
        finally:
            if status_write is not None:
                os.close(status_write)
            if stop_read is not None:
                os.close(stop_read)
        return self

    def poll(self):
        if self.status_fd is not None:
            try:
                while True:
                    chunk = os.read(self.status_fd, 4096)
                    if not chunk:
                        break
                    self.buffer.extend(chunk)
                    while b'\n' in self.buffer:
                        line, _, tail = self.buffer.partition(b'\n')
                        self.buffer[:] = tail
                        record = json.loads(line)
                        if not isinstance(record, dict) or len(line) > 4096:
                            raise ValueError
                        self.data.update(record)
                    if len(self.buffer) > 4096:
                        raise ValueError
            except BlockingIOError:
                pass
            except (OSError, ValueError, UnicodeError, RecursionError):
                self.data.update(state='failed', error='publish_worker_status_invalid')
        if self.process is not None and self.process.poll() is not None:
            if self.process.returncode != 0 and self.data['error'] is None:
                self.data.update(state='failed', error='publish_worker_failed', remote_stop_acknowledged=False)
        return self.snapshot()

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        data = dict(self.data)
        age = None if data['last_uploaded_at'] is None else max(0, now - data['last_uploaded_at'])
        if data['state'] == 'live' and age is not None and age > 3:
            data['state'] = 'stale'
        alive = self.process is not None and self.process.poll() is None
        if self.closing and alive and data['state'] != 'failed':
            data['state'] = 'stopping'
        return {**data, 'observed_at': now, 'last_uploaded_age_seconds': age,
            'worker_alive': alive, 'worker_pid': self.process.pid if self.process else None,
            'cleanup_verified': self.closed and not alive,
            'application_ready': False, 'flight_ready': False}

    def close(self):
        if self.closed:
            return self.poll()
        self.closing = True
        if self.stop_fd is not None:
            try:
                os.write(self.stop_fd, b'1')
            except OSError:
                pass
        deadline = time.monotonic() + CLOSE_TIMEOUT
        forced = False
        if self.process is not None:
            while self.process.poll() is None and time.monotonic() < deadline:
                self.poll()
                time.sleep(.025)
            if self.process.poll() is None:
                forced = True
                self.data.update(state='failed', error='publish_close_timeout', remote_stop_acknowledged=False)
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    self.process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=1)
                except ProcessLookupError:
                    self.process.wait(timeout=1)
            self.poll()
            if forced:
                self.data.update(state='failed', error='publish_close_timeout', remote_stop_acknowledged=False)
            elif not self.data.get('worker_finished'):
                self.data.update(state='failed', error=self.data['error'] or 'publish_worker_status_lost',
                                 remote_stop_acknowledged=False)
        for descriptor in (self.status_fd, self.stop_fd):
            if descriptor is not None:
                os.close(descriptor)
        self.status_fd = self.stop_fd = None
        self.closed = True
        if self.data['state'] != 'failed':
            self.data['state'] = 'offline'
        if self.process is not None and self.process.poll() is None:
            raise PublishError('publish_cleanup_unconfirmed', cleanup_verified=False)
        return self.snapshot()


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker-config', required=True, type=Path)
    parser.add_argument('--fingerprint', required=True)
    parser.add_argument('--directory', required=True, type=Path)
    parser.add_argument('--fps', required=True, type=float)
    parser.add_argument('--status-fd', required=True, type=int)
    parser.add_argument('--stop-fd', required=True, type=int)
    parser.add_argument('--parent-pid', required=True, type=int)
    args = parser.parse_args(argv)
    return worker(args.worker_config, args.fingerprint, args.directory, args.fps,
        args.status_fd, args.stop_fd, args.parent_pid)


if __name__ == '__main__':
    raise SystemExit(main())
