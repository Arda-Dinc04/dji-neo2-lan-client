"""Mocked orchestration: never associate, request BLE, or open a real socket."""
import importlib.util
import io
import json
from pathlib import Path
import stat
import subprocess
from types import SimpleNamespace

import pytest


SPEC = importlib.util.spec_from_file_location('macos_capture_once',
    Path(__file__).resolve().parents[1] / 'tools/macos_capture_once.py')
capture = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture)

ORIGINAL = (b'synthetic-home', b'\x02\x11\x22\x33\x44\x55')
TARGET = (b'synthetic-aircraft', b'\x02\xaa\xbb\xcc\xdd\xee')
PASSWORD = 'synthetic-private-password'


class Clock:
    def __init__(self): self.now = 0
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


class Camera:
    def __init__(self):
        self.connected = self.joined = False
        self.connects = self.disconnects = 0
        self.generation = None
        self.network_value = {'ssid': TARGET[0].decode(), 'password': PASSWORD,
            'bssid_hex': TARGET[1].hex(), 'session_generation': 'generation-test'}
        self.unavailable = self.generation_changed = False

    def status(self):
        if self.unavailable: raise capture.CaptureFailure('local_camera_unreachable')
        return {'preview_only': True, 'session_setup_authorized': True,
            'flight_commands_available': False,
            'session_generation': 'different-generation' if self.generation_changed else self.generation,
            'state': ('connected' if self.joined else 'connecting') if self.connected else 'disconnected',
            'cleanup_verified': not self.connected,
            'video': {'live': self.joined, 'frames_encoded': 4 if self.joined else 0}}

    def network(self): return self.network_value
    def connect(self):
        self.connects += 1
        self.connected = True
        self.generation = 'generation-test'
    def disconnect(self):
        self.disconnects += 1
        self.connected = False
        self.unavailable = False


class Bench:
    def __init__(self):
        self.api = Camera()
        self.clock = Clock()
        self.wifi = ORIGINAL
        self.calls = []
        self.events = []
        self.join_failure = self.restore_failure = self.wrong_bssid = False
        self.after_join = lambda: None

    def observe(self, interface):
        assert interface == 'en0'
        return self.wifi

    def run(self, argv, **options):
        self.calls.append((argv, options))
        if len(self.calls) == 1:
            self.wifi = (TARGET[0], ORIGINAL[1]) if self.wrong_bssid else TARGET
            self.api.joined = True
            self.after_join()
            if self.join_failure: raise subprocess.TimeoutExpired(argv, options['timeout'])
        elif not self.restore_failure:
            self.wifi = ORIGINAL
        return SimpleNamespace(returncode=0)

    def capture(self, **kwargs):
        return capture.capture_once(self.api, duration=1, observer=self.observe,
            runner=self.run, clock=self.clock, sleep=self.clock.sleep,
            event=lambda stage, **details: self.events.append({'stage': stage, **details}), **kwargs)


def test_join_password_is_stdin_only_and_restore_uses_saved_settings():
    bench = Bench()
    result = bench.capture()
    assert result['success'] and result['target_network_verified'] and result['restored_original_ssid']
    assert result['camera_cleanup_verified'] and bench.api.disconnects == 1
    join, restore = bench.calls
    assert join[0] == ['/usr/sbin/networksetup', '-setairportnetwork', 'en0', TARGET[0].decode(), '-']
    assert join[1]['input'] == PASSWORD.encode() + b'\n'
    assert restore[0] == ['/usr/sbin/networksetup', '-setairportnetwork', 'en0', ORIGINAL[0].decode()]
    assert 'input' not in restore[1] and restore[1]['stdin'] == subprocess.DEVNULL
    for argv, options in bench.calls:
        assert PASSWORD not in str(argv)
        assert options['stdout'] == options['stderr'] == subprocess.DEVNULL
        assert options['timeout'] == 25 and not options.get('shell', False)
    public = json.dumps([result, bench.events])
    for secret in (PASSWORD, ORIGINAL[0].decode(), TARGET[0].decode(), TARGET[1].hex(), 'generation-test'):
        assert secret not in public


def test_join_timeout_still_disconnects_and_restores_partial_association():
    bench = Bench()
    bench.join_failure = True
    result = bench.capture()
    assert result['reason'] == 'aircraft_wifi_join_failed' and not result['success']
    assert bench.api.disconnects == 1 and result['restored_original_ssid']
    assert len(bench.calls) == 2 and bench.wifi == ORIGINAL


def test_http_loss_after_join_still_attempts_disconnect_then_restores():
    bench = Bench()
    bench.after_join = lambda: setattr(bench.api, 'unavailable', True)
    result = bench.capture()
    assert result['reason'] == 'local_camera_unreachable'
    assert bench.api.disconnects == 1 and result['restored_original_ssid']


def test_keyboard_interrupt_after_partial_join_runs_restoration():
    bench = Bench()
    def interrupt(): raise KeyboardInterrupt
    bench.after_join = interrupt
    result = bench.capture()
    assert result['reason'] == 'interrupted' and result['restored_original_ssid']
    assert bench.api.disconnects == 1


def test_event_logging_failure_cannot_prevent_cleanup_or_restoration():
    bench = Bench()
    def broken_event(*args, **kwargs): raise OSError('disk failure')
    result = capture.capture_once(bench.api, duration=1, observer=bench.observe,
        runner=bench.run, clock=bench.clock, sleep=bench.clock.sleep, event=broken_event)
    assert result['success'] and bench.wifi == ORIGINAL


def test_target_bssid_mismatch_refuses_preview_and_restores():
    bench = Bench()
    bench.wrong_bssid = True
    result = bench.capture()
    assert result['reason'] == 'aircraft_wifi_identity_not_verified'
    assert not result['preview_seen'] and not result['target_network_verified']
    assert result['restored_original_ssid'] and bench.api.disconnects == 1
    assert bench.clock.now <= 31


def test_restore_failure_is_reported_with_sanitized_local_recovery_notice():
    bench = Bench()
    bench.restore_failure = True
    result = bench.capture()
    assert result['restore_attempted'] and result['restored_original_ssid'] is False
    assert not result['success'] and 'macOS Wi-Fi settings' in result['recovery_notice']
    assert any(item['stage'] == 'manual_wifi_recovery_needed' for item in bench.events)
    assert bench.clock.now <= 32


@pytest.mark.parametrize('field,value', [('preview_only', False),
    ('session_setup_authorized', False), ('flight_commands_available', True),
    ('state', 'connected'), ('cleanup_verified', False)])
def test_refuses_unapproved_or_existing_sessions_before_mutation(field, value):
    bench = Bench()
    original_status = bench.api.status
    bench.api.status = lambda: {**original_status(), field: value}
    result = bench.capture()
    assert not result['success'] and not bench.calls and bench.api.connects == bench.api.disconnects == 0


@pytest.mark.parametrize('changes', [{'session_generation': 'old'}, {'bssid_hex': ''},
    {'bssid_hex': 'ff' * 6}, {'password': 'short'}, {'password': 'secret\npassword'}])
def test_rejects_stale_or_malformed_credentials_before_wifi_change(changes):
    bench = Bench()
    bench.api.network_value.update(changes)
    result = bench.capture()
    assert not bench.calls and bench.api.disconnects == 1 and not result['restore_attempted']


def test_no_original_wifi_identity_stops_before_connect():
    bench = Bench()
    bench.wifi = (b'', None)
    result = bench.capture()
    assert not bench.calls and bench.api.connects == 0 and not result['restore_attempted']


def test_session_generation_change_after_join_does_not_disconnect_new_worker():
    bench = Bench()
    bench.after_join = lambda: setattr(bench.api, 'generation_changed', True)
    result = bench.capture()
    assert result['reason'] == 'camera_session_generation_changed'
    assert bench.api.disconnects == 0 and result['restored_original_ssid']
    assert result['camera_cleanup_verified'] is False


def test_network_details_wait_has_deadline_and_no_association():
    bench = Bench()
    bench.api.network_value = None
    result = bench.capture()
    assert result['reason'] == 'camera_network_details_timeout' and bench.clock.now == 65
    assert not bench.calls and bench.api.disconnects == 1


def test_default_plan_does_no_http_identity_command_or_file_writes(monkeypatch, capsys):
    def forbidden(*args, **kwargs): raise AssertionError('plan must be inert')
    monkeypatch.setattr(capture, 'LocalCamera', forbidden)
    monkeypatch.setattr(capture, 'Journal', forbidden)
    monkeypatch.setattr(capture, 'wifi_identity', forbidden)
    assert capture.main([]) == 0
    assert json.loads(capsys.readouterr().out)['writes'] == 0


def test_journal_files_are_private_and_contain_only_sanitized_stage_data(tmp_path, capsys):
    journal = capture.Journal(tmp_path / 'auto-capture')
    journal.event('testing', joined=False)
    journal.finish({'success': False, 'reason': 'synthetic'})
    assert stat.S_IMODE(journal.directory.stat().st_mode) == 0o700
    for path in journal.directory.iterdir():
        assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert json.loads((journal.directory / 'result.json').read_text())['reason'] == 'synthetic'


class Response(io.BytesIO):
    def __enter__(self): return self
    def __exit__(self, *args): self.close()


def test_local_http_token_auth_without_proxy_or_redirect_following(monkeypatch):
    token = 'a' * 64
    calls = []
    bodies = [f'<meta name="neo2-token" content="{token}">'.encode(), b'{"preview_only":true}', b'{"accepted":true}']
    class Opener:
        def open(self, request, timeout):
            calls.append((request, timeout))
            return Response(bodies.pop(0))
    handlers = []
    def opener(*values):
        handlers.extend(values)
        return Opener()
    monkeypatch.setattr(capture, 'build_opener', opener)
    api = capture.LocalCamera(8793)
    assert api.status()['preview_only']
    api.connect()
    assert all(call[1] == 3 and call[0].full_url.startswith('http://127.0.0.1:8793/') for call in calls)
    assert calls[0][0].get_header('X-neo2-token') is None
    assert calls[1][0].get_header('X-neo2-token') == token
    assert calls[2][0].method == 'POST' and calls[2][0].data == b'{}'
    assert handlers[0].proxies == {} and handlers[1].redirect_request(None) is None
