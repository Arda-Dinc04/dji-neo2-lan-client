"""Owned loopback MediaMTX v1.21.3 and a bounded private WHEP proxy.

Readiness covers the process and its listeners, never camera freshness. SDP
and upstream session locations remain in memory and are omitted from logs and
snapshots. Camera/UDP owners need not wait for the browser or this process.
"""
from __future__ import annotations

import http.client
import json
import os
from pathlib import Path
import re
import secrets
import selectors
import shutil
import signal
import socket
import subprocess
import threading
import time
from urllib.parse import urlsplit

from .process_limits import (LimitPolicyError, VideoProcessLimits, MEMORY_CHECK_SECONDS,
                             signal_owned_child)

MAX_SDP_BYTES = 64 * 1024
MAX_LOG_BYTES = 64 * 1024
MAX_SESSIONS = 4
HTTP_TIMEOUT = 3.0
START_TIMEOUT = 5.0
DEFAULT_BINARY = Path(__file__).resolve().parents[2] / 'build/mediamtx/mediamtx'
_SESSION_PATH = re.compile(r'/preview/whep/[A-Za-z0-9_-]{1,128}\Z')
_LOCAL_ID = re.compile(r'[0-9a-f]{48}\Z')


class WebRtcBridgeError(RuntimeError):
    def __init__(self, reason, *, cleanup_verified=True):
        super().__init__(reason)
        self.cleanup_verified = cleanup_verified


class WebRtcBridge:
    def __init__(self, directory, *, binary=None, rtsp_port=0, whep_port=0,
                 ice_udp_port=0, process_limits=None, clock=time.monotonic):
        ports = (rtsp_port, whep_port, ice_udp_port)
        if any(type(p) is not int or not 0 <= p <= 65535 for p in ports):
            raise ValueError('invalid_webrtc_loopback_port')
        nonzero = [p for p in ports if p]
        if len(nonzero) != len(set(nonzero)):
            raise ValueError('webrtc_ports_must_be_distinct')
        self.directory = Path(directory).absolute()
        self.binary = Path(binary or DEFAULT_BINARY).absolute()
        self.clock = clock
        self.generation = secrets.token_hex(12)
        self._requested_ports = ports
        self._ports = None
        self.process_limits = process_limits or VideoProcessLimits(linux_limiter=shutil.which('prlimit'))
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self._process = None
        self._sessions = {}
        self._pending_offers = 0
        self._ready = False
        self._started = self._closing = self._closed = False
        self._cleanup_verified = True
        self._error = None
        self._log = bytearray()
        self._log_received = 0

    @property
    def ready(self):
        with self._lock:
            return bool(self._ready and not self._closing and not self._closed
                        and self._process is not None and self._process.poll() is None)

    @property
    def rtsp_url(self):
        with self._lock:
            if self._ports is None:
                raise WebRtcBridgeError('webrtc_bridge_not_started')
            return f'rtsp://127.0.0.1:{self._ports[0]}/preview'

    def _reserve_ports(self):
        sockets = []
        ports = []
        try:
            for requested, kind in zip(self._requested_ports,
                    (socket.SOCK_STREAM, socket.SOCK_STREAM, socket.SOCK_DGRAM)):
                sock = socket.socket(socket.AF_INET, kind)
                sockets.append(sock)
                sock.bind(('127.0.0.1', requested))
                ports.append(sock.getsockname()[1])
            return sockets, tuple(ports)
        except BaseException:
            for sock in sockets:
                sock.close()
            raise

    def _configuration(self):
        rtsp, whep, ice = self._ports
        # JSON is a YAML subset; no YAML emitter/dependency or interpolation.
        return {
            'logLevel': 'error', 'logDestinations': ['stdout'], 'dumpPackets': False,
            'readTimeout': '3s', 'writeTimeout': '3s', 'writeQueueSize': 256,
            'authMethod': 'internal',
            'authInternalUsers': [{'user': 'any', 'pass': '', 'ips': ['127.0.0.1'],
                'permissions': [{'action': 'publish', 'path': 'preview'},
                                {'action': 'read', 'path': 'preview'}]}],
            'api': False, 'metrics': False, 'pprof': False, 'playback': False,
            'rtsp': True, 'rtspTransports': ['tcp'], 'rtspEncryption': 'no',
            'rtspAddress': f'127.0.0.1:{rtsp}',
            'rtmp': False, 'hls': False, 'srt': False, 'moq': False,
            'webrtc': True, 'webrtcAddress': f'127.0.0.1:{whep}',
            'webrtcEncryption': False, 'webrtcAllowOrigins': [],
            'webrtcTrustedProxies': [],
            'webrtcLocalUDPAddress': f'127.0.0.1:{ice}', 'webrtcLocalTCPAddress': '',
            'webrtcIPsFromInterfaces': False, 'webrtcIPsFromInterfacesList': [],
            'webrtcIPsFromInterfacesExcludeList': [],
            'webrtcAdditionalHosts': ['127.0.0.1'], 'webrtcICEServers2': [],
            'webrtcHandshakeTimeout': '10s', 'webrtcTrackGatherTimeout': '2s',
            'pathDefaults': {'source': 'publisher', 'sourceOnDemand': False,
                'maxReaders': MAX_SESSIONS, 'record': False, 'overridePublisher': False},
            'paths': {'preview': {'source': 'publisher', 'maxReaders': MAX_SESSIONS}},
        }

    def start(self):
        with self._lock:
            if self._started or self._closed:
                raise WebRtcBridgeError('webrtc_bridge_cannot_restart')
            self._started = True
        reservations = []
        try:
            if not self.binary.is_file() or not os.access(self.binary, os.X_OK):
                raise WebRtcBridgeError('mediamtx_binary_unavailable')
            reservations, ports = self._reserve_ports()
            self._ports = ports
            self.directory.mkdir(mode=0o700, parents=True, exist_ok=False)
            config = self.directory / 'mediamtx.yml'
            descriptor = os.open(config, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
            with os.fdopen(descriptor, 'w') as stream:
                json.dump(self._configuration(), stream)
                stream.write('\n')
            command = self.process_limits.wrap([str(self.binary), str(config)])
            for sock in reservations:
                sock.close()
            reservations = []
            self._process = subprocess.Popen(command, cwd=self.directory,
                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                env={'PATH': '/usr/bin:/bin', 'LC_ALL': 'C'},
                start_new_session=True, umask=0o077, bufsize=0)
            self._cleanup_verified = False
            self._thread = threading.Thread(target=self._watch, name='neo2-webrtc-bridge', daemon=True)
            self._thread.start()
            deadline = self.clock() + START_TIMEOUT
            while self.clock() < deadline:
                if self._process.poll() is not None or self._stop.is_set():
                    break
                if self._listeners_open():
                    # Let a conflicting bind fail before accepting readiness.
                    if self._stop.wait(.025) or self._process.poll() is not None:
                        break
                    with self._lock:
                        self._ready = True
                    return self
                self._stop.wait(.025)
            raise WebRtcBridgeError('webrtc_bridge_listeners_unavailable')
        except BaseException as error:
            with self._lock:
                self._error = (str(error) if isinstance(error, WebRtcBridgeError)
                               else 'webrtc_bridge_start_failed')
            self.close()
            if isinstance(error, (KeyboardInterrupt, SystemExit)):
                raise
            raise WebRtcBridgeError(self._error, cleanup_verified=self._cleanup_verified) from None
        finally:
            for sock in reservations:
                sock.close()

    def _listeners_open(self):
        for port in self._ports[:2]:
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.1):
                    pass
            except OSError:
                return False
        return True

    def _watch(self):
        process = self._process
        selector = selectors.DefaultSelector()
        next_check = 0.0
        try:
            os.set_blocking(process.stdout.fileno(), False)
            selector.register(process.stdout, selectors.EVENT_READ)
            while not self._stop.is_set():
                for _, _events in selector.select(.05):
                    data = os.read(process.stdout.fileno(), 4096)
                    if data:
                        with self._lock:
                            self._log_received += len(data)
                            self._log.extend(data)
                            if len(self._log) > MAX_LOG_BYTES:
                                del self._log[:-MAX_LOG_BYTES]
                    else:
                        selector.unregister(process.stdout)
                if process.poll() is not None:
                    with self._lock:
                        if not self._closing:
                            self._error = self._error or 'webrtc_bridge_process_exited'
                    break
                if self.clock() >= next_check:
                    self.process_limits.check_memory([process])
                    next_check = self.clock() + MEMORY_CHECK_SECONDS
        except LimitPolicyError:
            with self._lock:
                self._error = 'webrtc_bridge_memory_limit_or_monitor_failed'
            self._stop.set()
        except Exception:
            with self._lock:
                if not self._closing:
                    self._error = 'webrtc_bridge_monitor_failed'
            self._stop.set()
        finally:
            selector.close()
            with self._lock:
                self._ready = False
            # An error must release the listener even before owner close().
            if self._stop.is_set() and not self._closing:
                self._terminate_process()

    def _terminate_process(self):
        process = self._process
        if process is None:
            self._cleanup_verified = True
            return
        try:
            if process.poll() is None:
                signal_owned_child(process, signal.SIGTERM)
                try:
                    process.wait(timeout=1.5)
                except subprocess.TimeoutExpired:
                    signal_owned_child(process, signal.SIGKILL)
                    process.wait(timeout=1)
            else:
                process.wait(timeout=.1)
            self._cleanup_verified = process.poll() is not None
        except Exception:
            self._cleanup_verified = False

    def snapshot(self):
        with self._lock:
            running = bool(self._process is not None and self._process.poll() is None)
            return {'enabled': True, 'generation': self.generation,
                'ready': self.ready, 'running': running,
                'state': 'closed' if self._closed else 'failed' if self._error else
                         'ready' if self.ready else 'starting' if self._started else 'idle',
                'error': self._error, 'session_count': len(self._sessions),
                'pending_offers': self._pending_offers, 'maximum_sessions': MAX_SESSIONS,
                'log_retained_bytes': len(self._log), 'maximum_log_bytes': MAX_LOG_BYTES,
                'log_bytes_received': self._log_received,
                'cleanup_verified': self._cleanup_verified,
                'process_limits': self.process_limits.snapshot()}

    def _location(self, value):
        if (not isinstance(value, str) or not 1 <= len(value) <= 512
                or any(ord(char) <= 32 or ord(char) >= 127 for char in value)):
            raise WebRtcBridgeError('invalid_whep_session_location')
        try:
            parsed = urlsplit(value)
            expected = f'127.0.0.1:{self._ports[1]}'
            if (parsed.scheme not in ('', 'http') or parsed.fragment or parsed.query
                    or parsed.username is not None or parsed.password is not None
                    or (parsed.netloc and parsed.netloc != expected)
                    or (parsed.scheme and parsed.netloc != expected)
                    or not _SESSION_PATH.fullmatch(parsed.path)):
                raise ValueError
        except ValueError:
            raise WebRtcBridgeError('invalid_whep_session_location') from None
        return parsed.path

    def _request(self, method, path, body=None):
        """Fixed numeric loopback peer, no redirects, proxies, or payload logs."""
        connection = http.client.HTTPConnection('127.0.0.1', self._ports[1], timeout=HTTP_TIMEOUT)
        expired = threading.Event()
        transport = [None]
        response = None
        def timeout():
            expired.set()
            try:
                sock = transport[0] or connection.sock
                if sock is not None:
                    sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        timer = threading.Timer(HTTP_TIMEOUT, timeout)
        timer.daemon = True
        timer.start()
        try:
            connection.connect()
            transport[0] = connection.sock
            if expired.is_set():
                raise WebRtcBridgeError('whep_request_timed_out')
            headers = {'Content-Type': 'application/sdp'} if body is not None else {}
            connection.request(method, path, body=body, headers=headers)
            response = connection.getresponse()
            length = response.getheader('Content-Length')
            if length is not None and (not length.isdecimal() or int(length) > MAX_SDP_BYTES):
                raise WebRtcBridgeError('whep_response_too_large')
            payload = response.read(MAX_SDP_BYTES + 1)
            if expired.is_set():
                raise WebRtcBridgeError('whep_request_timed_out')
            if len(payload) > MAX_SDP_BYTES:
                raise WebRtcBridgeError('whep_response_too_large')
            return response.status, response.getheader('Content-Type', ''), response.getheader('Location'), payload
        except (OSError, http.client.HTTPException, ValueError):
            raise WebRtcBridgeError('whep_request_failed') from None
        finally:
            timer.cancel()
            if response is not None:
                response.close()
            connection.close()

    def offer(self, sdp: bytes):
        if not isinstance(sdp, bytes) or not 1 <= len(sdp) <= MAX_SDP_BYTES or b'\0' in sdp:
            raise WebRtcBridgeError('invalid_whep_offer')
        with self._lock:
            if not self.ready:
                raise WebRtcBridgeError('webrtc_bridge_not_ready')
            if len(self._sessions) + self._pending_offers >= MAX_SESSIONS:
                raise WebRtcBridgeError('webrtc_session_limit_reached')
            self._pending_offers += 1
        try:
            status, content_type, location, answer = self._request('POST', '/preview/whep', sdp)
            if status != 201 or content_type.split(';', 1)[0].strip().lower() != 'application/sdp':
                raise WebRtcBridgeError('whep_offer_rejected')
            target = self._location(location)
            if not answer or b'\0' in answer:
                raise WebRtcBridgeError('invalid_whep_answer')
            with self._lock:
                if not self.ready:
                    raise WebRtcBridgeError('webrtc_bridge_not_ready')
                identifier = secrets.token_hex(24)
                self._sessions[identifier] = target
            return {'sdp': answer, 'session_id': identifier}
        finally:
            with self._lock:
                self._pending_offers -= 1

    def delete(self, session_id):
        if not isinstance(session_id, str) or not _LOCAL_ID.fullmatch(session_id):
            raise WebRtcBridgeError('webrtc_session_not_found')
        with self._lock:
            target = self._sessions.get(session_id)
            if target is None or not self.ready:
                raise WebRtcBridgeError('webrtc_session_not_found')
        status, _, _, _ = self._request('DELETE', target)
        if status not in (200, 204, 404):
            raise WebRtcBridgeError('whep_delete_failed')
        with self._lock:
            if self._sessions.get(session_id) == target:
                del self._sessions[session_id]

    def close(self):
        with self._lock:
            if self._closed:
                return self.snapshot()
            self._closing = True
            self._ready = False
            self._sessions.clear()
        self._stop.set()
        self._terminate_process()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                self._cleanup_verified = False
        if self._process is not None and self._cleanup_verified and self._process.stdout is not None:
            self._process.stdout.close()
        with self._lock:
            self._closed = True
        return self.snapshot()

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.close()
