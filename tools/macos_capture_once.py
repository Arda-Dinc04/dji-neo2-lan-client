#!/usr/bin/env python3
"""Bounded local camera capture with automatic Wi-Fi restoration; plan by default."""
import argparse
from html.parser import HTMLParser
import json
import math
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from neo2_control.macos_wifi import wifi_identity

NETWORKSETUP = '/usr/sbin/networksetup'
INTERFACE = 'en0'


class CaptureFailure(RuntimeError):
    """A fixed reason code, never a subprocess, HTTP, or credential payload."""


class _TokenParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.tokens = []

    def handle_starttag(self, tag, attrs):
        values = dict(attrs)
        if tag == 'meta' and values.get('name') == 'neo2-token':
            self.tokens.append(values.get('content', ''))


class _NoRedirects(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


class LocalCamera:
    """Only fixed loopback URLs; disable environment proxies and redirects."""
    def __init__(self, port=8793):
        if type(port) is not int or not 1 <= port <= 65535:
            raise CaptureFailure('invalid_local_port')
        self.base = f'http://127.0.0.1:{port}'
        self.token = None
        self.opener = build_opener(ProxyHandler({}), _NoRedirects())
        page = self._request('/', raw=True)
        parser = _TokenParser()
        parser.feed(page.decode('utf-8'))
        if len(parser.tokens) != 1 or not re.fullmatch('[0-9a-f]{64}', parser.tokens[0]):
            raise CaptureFailure('local_camera_token_unavailable')
        self.token = parser.tokens[0]

    def _request(self, path, *, post=False, raw=False, missing_ok=False):
        headers = {'Content-Type': 'application/json'}
        if self.token is not None:
            headers['X-Neo2-Token'] = self.token
        request = Request(self.base + path, data=b'{}' if post else None,
                          headers=headers, method='POST' if post else 'GET')
        try:
            with self.opener.open(request, timeout=3) as response:
                body = response.read(65537)
                if len(body) > 65536:
                    raise CaptureFailure('local_camera_response_too_large')
        except HTTPError as error:
            if missing_ok and error.code == 404:
                return None
            raise CaptureFailure('local_camera_http_failed') from None
        except (URLError, OSError, TimeoutError):
            raise CaptureFailure('local_camera_unreachable') from None
        if raw:
            return body
        try:
            result = json.loads(body)
        except (UnicodeError, ValueError):
            raise CaptureFailure('local_camera_response_invalid') from None
        if not isinstance(result, dict):
            raise CaptureFailure('local_camera_response_invalid')
        return result

    def status(self):
        return self._request('/api/camera/status')

    def network(self):
        return self._request('/api/camera/network', missing_ok=True)

    def connect(self):
        return self._request('/api/camera/connect', post=True)

    def disconnect(self):
        return self._request('/api/camera/disconnect', post=True)


class Journal:
    def __init__(self, directory):
        root = Path(directory)
        root.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.directory = root / ('run-' + secrets.token_hex(8))
        self.directory.mkdir(mode=0o700)
        self.events = self._open('events.jsonl')

    def _open(self, name):
        fd = os.open(self.directory / name,
                     os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
        return os.fdopen(fd, 'w')

    def event(self, stage, **details):
        value = {'stage': stage, **details}
        line = json.dumps(value, sort_keys=True)
        self.events.write(line + '\n')
        self.events.flush()
        print(line, flush=True)

    def finish(self, result):
        try:
            with self._open('result.json') as stream:
                json.dump(result, stream, sort_keys=True)
                stream.write('\n')
        finally:
            self.events.close()


def _authorized(status):
    if (status.get('preview_only') is not True
            or status.get('session_setup_authorized') is not True
            or status.get('flight_commands_available') is not False):
        raise CaptureFailure('preview_session_not_authorized')


def _ssid_text(value):
    if not isinstance(value, bytes) or not 1 <= len(value) <= 32 or b'\0' in value:
        raise CaptureFailure('wifi_identity_invalid')
    try:
        return value.decode('utf-8')
    except UnicodeError:
        raise CaptureFailure('wifi_ssid_not_utf8') from None


def _credentials(network, generation):
    if (not isinstance(network, dict) or not generation
            or network.get('session_generation') != generation):
        raise CaptureFailure('camera_network_generation_changed')
    ssid, password = network.get('ssid'), network.get('password')
    if not isinstance(ssid, str) or not isinstance(password, str):
        raise CaptureFailure('camera_network_details_invalid')
    try:
        ssid_bytes, password_bytes = ssid.encode('utf-8'), password.encode('ascii')
        _ssid_text(ssid_bytes)
        raw_bssid = network.get('bssid', network.get('bssid_hex', ''))
        if not isinstance(raw_bssid, str):
            raise ValueError
        bssid = bytes.fromhex(raw_bssid.replace(':', ''))
        if len(bssid) != 6 or bssid == bytes(6) or bssid == b'\xff' * 6:
            raise ValueError
        if not ((8 <= len(password_bytes) <= 63 and all(32 <= c <= 126 for c in password_bytes))
                or (len(password_bytes) == 64 and re.fullmatch(b'[0-9a-fA-F]{64}', password_bytes))):
            raise ValueError
    except (ValueError, UnicodeError):
        raise CaptureFailure('camera_network_details_invalid') from None
    return ssid_bytes, bssid, password_bytes


def _networksetup(ssid, *, password=None, runner=subprocess.run):
    arguments = [NETWORKSETUP, '-setairportnetwork', INTERFACE, _ssid_text(ssid)]
    options = {'stdout': subprocess.DEVNULL, 'stderr': subprocess.DEVNULL,
               'timeout': 25, 'check': False}
    if password is not None:
        arguments.append('-')
        options['input'] = password + b'\n'
    else:
        # Omit the password parameter: macOS may use its saved network settings.
        options['stdin'] = subprocess.DEVNULL
    try:
        return runner(arguments, **options).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def capture_once(api, *, duration=60, observer=wifi_identity, runner=subprocess.run,
                 clock=time.monotonic, sleep=time.sleep, event=lambda *a, **k: None):
    """Own one clean camera session and restore Wi-Fi even after partial joins."""
    if type(duration) not in (int, float) or not math.isfinite(duration) or not 0 < duration <= 60:
        raise CaptureFailure('capture_duration_invalid')
    result = {'preview_seen': False, 'frames_encoded': 0, 'target_network_verified': False,
              'camera_cleanup_verified': None, 'restore_attempted': False,
              'restored_original_ssid': None, 'success': False, 'reason': None}
    owns_session = network_attempted = False
    original_ssid = None
    generation = None
    sink = event
    def event(stage, **details):
        # Disk/Terminal failure must never prevent the restoration finally block.
        try:
            sink(stage, **details)
        except Exception:
            pass
    try:
        event('preflight')
        initial = api.status()
        _authorized(initial)
        if initial.get('state') != 'disconnected' or initial.get('cleanup_verified') is not True:
            raise CaptureFailure('camera_must_be_clean_and_disconnected')
        original_ssid, original_bssid = observer(INTERFACE)
        _ssid_text(original_ssid)
        if not isinstance(original_bssid, bytes) or len(original_bssid) != 6:
            raise CaptureFailure('original_wifi_identity_unavailable')
        event('camera_connecting')
        owns_session = True  # A timed-out POST may still have started the session.
        api.connect()
        deadline = clock() + 65
        next_notice = clock()
        pending_notice = False
        while clock() < deadline:
            if clock() >= next_notice:
                event('awaiting_current_camera_network')
                next_notice = clock() + 5
            status = api.status()
            _authorized(status)
            if status.get('state') in ('failed', 'disconnected') or status.get('error'):
                raise CaptureFailure('camera_setup_ended')
            current = status.get('session_generation')
            if generation is None:
                if not isinstance(current, str) or not current:
                    sleep(.5)
                    continue
                generation = current
            if current != generation:
                raise CaptureFailure('camera_session_generation_changed')
            if status.get('verification_pending') and not pending_notice:
                event('aircraft_permission_pending')
                pending_notice = True
            network = api.network()
            if network is not None:
                ssid, bssid, password = _credentials(network, generation)
                # Do not act on an endpoint response from a superseded session.
                latest = api.status()
                _authorized(latest)
                if latest.get('session_generation') != generation:
                    raise CaptureFailure('camera_session_generation_changed')
                if latest.get('state') not in ('connecting', 'connected') or latest.get('error'):
                    raise CaptureFailure('camera_setup_ended')
                break
            sleep(.5)
        else:
            raise CaptureFailure('camera_network_details_timeout')
        event('joining_aircraft_wifi')
        network_attempted = True
        if not _networksetup(ssid, password=password, runner=runner):
            raise CaptureFailure('aircraft_wifi_join_failed')
        del password, network
        deadline = clock() + 30
        while clock() < deadline:
            try:
                actual_ssid, actual_bssid = observer(INTERFACE)
                if (actual_ssid, actual_bssid) == (ssid, bssid):
                    result['target_network_verified'] = True
                    break
            except Exception:
                pass
            sleep(.5)
        if not result['target_network_verified']:
            raise CaptureFailure('aircraft_wifi_identity_not_verified')
        event('aircraft_wifi_verified')
        deadline = clock() + duration
        next_notice = clock()
        while clock() < deadline:
            status = api.status()
            _authorized(status)
            if status.get('session_generation') != generation:
                raise CaptureFailure('camera_session_generation_changed')
            video = status.get('video', {})
            if isinstance(video, dict):
                count = video.get('frames_encoded', 0)
                if type(count) is int and count >= 0:
                    result['frames_encoded'] = max(result['frames_encoded'], count)
                if video.get('live') is True and not result['preview_seen']:
                    result['preview_seen'] = True
                    event('preview_received')
            if status.get('state') == 'failed' or status.get('error'):
                raise CaptureFailure('camera_session_failed')
            if status.get('state') == 'disconnected':
                break
            if clock() >= next_notice:
                event('capturing', preview_seen=result['preview_seen'], frames_encoded=result['frames_encoded'])
                next_notice = clock() + 5
            sleep(.5)
        result['reason'] = 'capture_complete' if result['preview_seen'] else 'preview_unavailable'
    except CaptureFailure as error:
        result['reason'] = str(error)
    except KeyboardInterrupt:
        result['reason'] = 'interrupted'
    except Exception:
        result['reason'] = 'local_capture_failed'
    finally:
        if owns_session:
            event('disconnecting_camera')
            result['camera_cleanup_verified'] = False
            try:
                try:
                    current = api.status()
                except Exception:
                    current = None
                if current is not None and generation is not None and current.get('session_generation') != generation:
                    raise CaptureFailure('camera_session_generation_changed')
                api.disconnect()
                deadline = clock() + 20
                while clock() < deadline:
                    status = api.status()
                    if (status.get('session_generation') != generation and generation is not None):
                        break
                    if status.get('cleanup_verified') is True and status.get('state') in ('disconnected', 'failed'):
                        result['camera_cleanup_verified'] = True
                        break
                    sleep(.5)
            except Exception:
                pass
        if network_attempted and original_ssid is not None:
            event('restoring_original_wifi')
            result['restore_attempted'] = True
            result['restored_original_ssid'] = False
            _networksetup(original_ssid, runner=runner)
            deadline = clock() + 30
            while clock() < deadline:
                try:
                    if observer(INTERFACE)[0] == original_ssid:
                        result['restored_original_ssid'] = True
                        break
                except Exception:
                    pass
                sleep(.5)
            event('wifi_restoration_checked', restored=result['restored_original_ssid'])
        result['success'] = bool(result['reason'] == 'capture_complete'
            and result['preview_seen'] and result['camera_cleanup_verified'] is True
            and result['restored_original_ssid'] is True)
        if result['restore_attempted'] and not result['restored_original_ssid']:
            result['recovery_notice'] = 'Wi-Fi restoration was not verified. Rejoin your original network in macOS Wi-Fi settings; camera status remains on the local camera page.'
            event('manual_wifi_recovery_needed', local_page_available=True)
        event('finished', success=result['success'], reason=result['reason'])
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Authorize one local camera/Wi-Fi capture and restoration attempt')
    parser.add_argument('--port', type=int, default=8793)
    parser.add_argument('--duration', type=float, default=60)
    args = parser.parse_args(argv)
    if not args.execute:
        print(json.dumps({'execute': False, 'plan': ['verify_preview_only_server', 'remember_original_wifi',
            'connect_camera', 'join_aircraft_using_password_stdin', 'capture_up_to_60_seconds',
            'disconnect_camera', 'restore_saved_original_wifi', 'verify_restoration'],
            'local_page': f'http://127.0.0.1:{args.port}/', 'writes': 0}), flush=True)
        return 0
    if sys.platform != 'darwin':
        parser.error('automatic Wi-Fi capture requires macOS')
    if not 0 < args.duration <= 60 or not math.isfinite(args.duration):
        parser.error('duration must be finite and at most60seconds')
    journal = Journal(ROOT / 'private/macos/auto-capture')
    previous = signal.getsignal(signal.SIGTERM)
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        journal.event('local_capture_starting', local_page=f'http://127.0.0.1:{args.port}/')
        try:
            result = capture_once(LocalCamera(args.port), duration=args.duration, event=journal.event)
        except (Exception, KeyboardInterrupt):
            result = {'success': False, 'reason': 'local_camera_unavailable', 'restore_attempted': False}
        journal.finish(result)
        return 0 if result['success'] else 2
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == '__main__':
    raise SystemExit(main())
