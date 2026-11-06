"""Console resource ownership with fake BLE/NM/session, never RF or ffmpeg.

JPEG files and HLS snapshots here are deliberate fixtures. These tests prove
console gating and freshness, not encoder cleanup or real frame decoding;
the session's video_cleanup_sink remains the cleanup authority.
"""
import asyncio
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import queue
import threading
import time
from types import SimpleNamespace

import pytest

from neo2_control.ble_bootstrap import WifiCredentials
from neo2_control.camera_console import CameraConfig, CameraConsole
from neo2_control.device_lease import DeviceLease


CREDS = WifiCredentials(bytes.fromhex('020000000002'), b'private-camera-test', b'private-camera-password')


def test_camera_duration_allows_startup_grace_for_ten_minute_observation(tmp_path):
    config=CameraConfig('en0',tmp_path/'wlm',tmp_path/'ble',preview_only=True,duration=660)
    assert config.duration==660
    with pytest.raises(ValueError,match='camera_duration'):
        replace(config,duration=661)


@pytest.mark.parametrize('preview_only,reason,committed,warning',[
    (True,'peer_beacon_expired',True,True),(False,'peer_beacon_expired',True,False),
    (True,'other_failure',True,False),(True,'peer_beacon_expired',False,False)])
def test_discovery_expiry_does_not_abort_fresh_committed_preview(harness,preview_only,reason,committed,warning):
    h,console=harness,harness.console
    console.config=replace(h.config,preview_only=preview_only)
    call=h.connect()
    preview=Path(call.kwargs['hls_directory'])/'preview.jpg'
    preview.parent.mkdir(mode=0o700);preview.write_bytes(b'current-jpeg-fixture')
    snapshot=call.snapshot(video={'last_encoded_age_seconds':0,'preview_age_seconds':0,'frames_encoded':5})
    snapshot['wlm_mode_follow']={'state':'failed','reason':reason,'wifi_service_configured':committed,
        'service_switch':{'state':'complete' if committed else 'failed','committed':committed}}
    console._publish(snapshot,generation=console.generation)
    state=console.status()
    assert state['video']['live']
    if warning:
        assert state['error'] is None
        assert state['warnings']==['peer_beacon_expired_while_video_live']
        assert not any(state['capabilities'].values())
    else:assert state['error']=='camera_link_interrupted'
    h.clock.advance(4)
    assert not console.status()['video']['live']
    assert console.status()['error']=='camera_link_interrupted'
    h.stop()


@pytest.mark.parametrize('value', [1, 'yes', None])
def test_startup_refresh_requires_explicit_boolean_preview_policy(tmp_path,value):
    with pytest.raises(ValueError,match='startup_refresh_requires_preview_only'):
        CameraConfig('en0',tmp_path/'wlm',tmp_path/'ble',preview_only=True,startup_keyframe_refresh=value)
    with pytest.raises(ValueError,match='startup_refresh_requires_preview_only'):
        CameraConfig('en0',tmp_path/'wlm',tmp_path/'ble',startup_keyframe_refresh=True)


class Clock:
    def __init__(self):
        self.value = 100.
        self.lock = threading.Lock()

    def __call__(self):
        with self.lock:
            return self.value

    def advance(self, seconds):
        with self.lock:
            self.value += seconds


def wait_for(predicate, timeout=2):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        threading.Event().wait(.005)
    assert predicate(), 'fake resource did not reach the requested state'


def result(state='closed', reason='requested', **extra):
    values = dict(state=state, reason=reason, **extra)
    return SimpleNamespace(**values, as_dict=lambda: deepcopy(values))


class Session:
    def __init__(self, harness, kwargs):
        self.h, self.kwargs = harness, kwargs
        self.operations, self.sent = queue.Queue(), []
        self.started, self.finished = threading.Event(), threading.Event()
        self.pause_exit, self.release_exit = False, threading.Event()
        self.ready, self.sequence = False, 0x8123

    def snapshot(self, *, video=None):
        return {'state': 'open', 'last_received_age_seconds': 0,
            'wlm_mode_follow': {'wifi_service_configured': self.ready},
            'video': {'enabled': True, 'cleanup_verified': False, 'frames_encoded': 0,
                'last_encoded_age_seconds': None, 'preview_age_seconds': None,
                'preview_available': False, **(video or {})},
            'battery': {'observations': [{'age_seconds': 0,
                'fields': {'percent': {'value': 61, 'valid': True}}}]}}

    def _publish(self, ready=None, video=None):
        if ready is not None:
            self.ready = ready
        self.kwargs['camera_channel'].set_ready(self.ready, self.h.clock())
        self.kwargs['snapshot_sink'](self.snapshot(video=video))

    def post(self, operation):
        done, response = threading.Event(), []
        self.operations.put((operation, done, response))
        assert done.wait(2), 'fake socket owner did not handle an operation'
        if response and isinstance(response[0], BaseException):
            raise response[0]
        return response[0] if response else None

    def publish(self, *, ready=None, video=None):
        self.post(lambda: self._publish(ready, video))

    def send_once(self, *, acknowledge=False):
        def send():
            channel = self.kwargs['camera_channel']
            packet = channel.next_packet(self.h.clock(), lambda: self.sequence)
            if packet is not None:
                self.sequence += 1
                self.sent.append(packet)
                channel.sent(packet.seq, self.h.clock())
                if acknowledge:
                    channel.receive(replace(packet, sender=packet.receiver, receiver=packet.sender,
                        cmd_type=0x80, payload=b'\0'), self.h.clock())
            return packet
        return self.post(send)

    def run(self):
        self.h.trace.append('session.enter')
        try:
            if self.h.failure == 'session_before_snapshot':
                self.started.set()
                raise RuntimeError('synthetic_session_before_snapshot')
            self._publish()
            self.started.set()
            if self.h.failure == 'session_after_snapshot':
                raise RuntimeError('synthetic_session_after_snapshot')
            while not self.kwargs['stop_event'].is_set():
                try:
                    operation, done, response = self.operations.get(timeout=.01)
                except queue.Empty:
                    continue
                try:
                    response.append(operation())
                except BaseException as error:
                    response.append(error)
                finally:
                    done.set()
            if self.pause_exit:
                assert self.release_exit.wait(2), 'test must release the fake session owner'
            return result()
        finally:
            if not self.h.omit_video_cleanup:
                self.kwargs['video_cleanup_sink']({'enabled': True,
                    'cleanup_verified': self.h.video_cleanup})
            self.h.trace.append('session.exit')
            self.finished.set()


class Wifi:
    def __init__(self, harness, credentials, interface, **kwargs):
        self.h, self.cleanup = harness, False
        assert credentials == CREDS and interface == 'wlanusb0'
        assert kwargs['stop_event'] is harness.console.stop_event
        harness.trace.append('wifi.create')
        if harness.failure == 'wifi_constructor':
            raise RuntimeError('synthetic_wifi_constructor')

    async def __aenter__(self):
        self.h.trace.append('wifi.enter')
        if self.h.failure == 'wifi_enter':
            self.cleanup = self.h.wifi_cleanup
            self.h.trace.append('wifi.failed_enter_cleanup')
            raise RuntimeError('synthetic_wifi_enter')
        return self

    async def __aexit__(self, *args):
        self.h.trace.append('wifi.exit')
        self.cleanup = self.h.wifi_cleanup

    def snapshot(self):
        return {'cleanup_verified': self.cleanup, 'state': 'closed' if self.cleanup else 'unknown'}


class Harness:
    def __init__(self, tmp_path):
        self.clock, self.trace, self.sessions = Clock(), [], []
        self.failure = None
        self.wifi_cleanup = self.ble_cleanup = self.video_cleanup = True
        self.omit_video_cleanup = False
        self.pause_ble, self.release_ble = False, threading.Event()
        self.ble_calls, self.guard_calls = [], []
        self.config = CameraConfig('wlanusb0', tmp_path/'wlm.json', tmp_path/'ble.json',
            ble_address='02:00:00:00:00:01', duration=30, capture_directory=tmp_path/'captures',
            last_frame=tmp_path/'previous.jpg')
        self.console = CameraConsole(self.config, clock=self.clock, session_runner=self.run_session,
            ble_runner=self.ble, wifi_factory=lambda *a, **kw: Wifi(self, *a, **kw), route_guard=self.guard,
            lease_factory=lambda i,a:DeviceLease(i,a,directory=tmp_path/'leases'))

    def guard(self, interface, **kwargs):
        self.guard_calls.append(interface)
        assert interface == 'wlanusb0'
        if self.failure == 'route':
            raise RuntimeError('synthetic_route_guard')

    async def ble(self, **kwargs):
        self.trace.append('ble.enter')
        self.ble_calls.append(kwargs)
        try:
            while self.pause_ble and not self.release_ble.is_set():
                if kwargs['stop_event'].is_set():
                    raise asyncio.CancelledError
                await asyncio.sleep(.005)
            if self.failure == 'ble_before_credentials':
                raise RuntimeError('synthetic_ble_before_credentials')
            CREDS.save_new(kwargs['credentials_path'])
            self.trace.append('credentials.saved')
            await kwargs['while_connected']()
            if self.failure == 'ble_after_retention':
                raise RuntimeError('synthetic_ble_after_retention')
            return result('complete', 'credentials_saved', connection_cleanup_verified=self.ble_cleanup)
        finally:
            self.trace.append('ble.exit')
            if self.ble_cleanup is not None:
                kwargs['event_sink']('ble_cleanup_result', cleanup_verified=self.ble_cleanup)

    def run_session(self, **kwargs):
        call = Session(self, kwargs)
        self.sessions.append(call)
        return call.run()

    def connect(self):
        count = len(self.sessions)
        answer = self.console.connect()
        assert answer == {'accepted': True, 'state': 'connecting'}
        wait_for(lambda: len(self.sessions) > count and self.sessions[-1].started.is_set())
        return self.sessions[-1]

    def join(self):
        self.console.thread.join(2)
        assert not self.console.thread.is_alive()

    def stop(self):
        self.console.disconnect()
        self.join()


@pytest.fixture
def harness(tmp_path, monkeypatch):
    # No default backend may accidentally start BLE/NM/UDP in these tests.
    monkeypatch.setattr('neo2_control.camera_console.run_ble_wifi_setup',
        lambda **kw: pytest.fail('unexpected real BLE runner'))
    monkeypatch.setattr('neo2_control.camera_console.run_session',
        lambda **kw: pytest.fail('unexpected real session runner'))
    h = Harness(tmp_path)
    yield h
    h.release_ble.set()
    for call in h.sessions:
        call.release_exit.set()
    h.console.stop_event.set()
    if h.console.thread is not None:
        h.console.thread.join(2)
        assert not h.console.thread.is_alive(), 'fake console leaked its worker'


def test_camera_rejects_an_already_owned_drone_before_ble_or_network_work(harness):
    h=harness
    directory=Path(h.config.capture_directory).parent/'leases'
    with DeviceLease(h.config.interface,h.config.ble_address,directory=directory):
        h.console.connect();h.join()
        assert not h.ble_calls and not h.guard_calls
        assert h.console.status()['error']=='drone_or_wifi_interface_already_owned'


def test_construction_and_last_capture_are_inert_and_never_live(harness):
    h, console = harness, harness.console
    Path(h.config.last_frame).write_bytes(b'previous-private-image')
    status = console.status()
    assert status['state'] == 'disconnected' and not status['connected']
    assert status['frame']['available'] and not status['video']['live']
    assert not status['camera_ready'] and not status['flight_commands_available']
    assert 'takeoff' not in status['capabilities'] and 'land' not in status['capabilities']
    assert not h.trace and not Path(h.config.capture_directory).exists()
    with pytest.raises(RuntimeError, match='not_connected'):
        console.command('record', {})


@pytest.mark.parametrize(('component','failure','expected'),[
    ('video',{'state':'failed','error':'video_normalizer_failed'},'camera_video_unavailable'),
    ('wlm_mode_follow',{'state':'failed','reason':'peer_beacon_expired'},'camera_link_interrupted'),
    ('wlm_mode_follow',{'capability_read':{'state':'timed_out'}},'camera_setup_incomplete'),
    ('wlm_mode_follow',{'service_switch':{'state':'failed'}},'camera_setup_incomplete'),
])
def test_failed_camera_preparation_is_visible_while_udp_transport_remains_open(harness,component,failure,expected):
    call=harness.connect()
    snapshot=call.snapshot()
    snapshot[component].update(failure)
    harness.console._publish(snapshot)
    status=harness.console.status()
    assert status['connected'] and not status['camera_ready']
    assert status['state_label']=='Préparation caméra…'
    assert status['error']==expected and not status['video']['live']
    with pytest.raises(RuntimeError,match='camera_not_ready'):
        harness.console.command('photo',{})


def test_connect_gates_mailbox_on_service_and_disconnect_closes_resources_in_order(harness):
    h, console = harness, harness.console
    call = h.connect()
    assert console.status()['connected'] and not console.status()['camera_ready']
    with pytest.raises(RuntimeError, match='camera_not_ready'):
        console.command('pitch', {'degrees': -10})
    with pytest.raises(RuntimeError, match='already_running'):
        console.connect()
    call.publish(ready=True)
    queued = console.command('pitch', {'degrees': -10})
    assert queued['accepted'] and queued['state'] == 'queued' and not call.sent
    assert console.status()['last_command']['state'] == 'queued'
    with pytest.raises(RuntimeError, match='command_pending'):
        console.command('record', {})
    packet = call.send_once()
    assert (packet.cmd_set, packet.cmd_id) == (4, 0x14)
    assert console.status()['last_command']['state'] == 'sent_unconfirmed'
    assert not console.status()['last_command']['execution_confirmed']
    console.command('record', {})
    h.stop()
    status = console.status()
    assert status['state'] == 'disconnected' and status['cleanup_verified']
    assert not status['connected'] and not status['camera_ready'] and not status['video']['live']
    assert status['last_command']['state'] == 'cancelled' and len(call.sent) == 1
    assert h.trace.index('ble.enter') < h.trace.index('wifi.enter') < h.trace.index('session.enter')
    assert h.trace.index('session.exit') < h.trace.index('wifi.exit') < h.trace.index('ble.exit')
    assert h.ble_calls[0]['hold_timeout'] == h.config.duration+60
    assert call.kwargs['preview_jpeg'] and call.kwargs['video_codec'] == 'hevc'
    assert call.kwargs['wlm_wifi_liveview'] and call.kwargs['camera_channel'] is console.channel
    assert console.paths['credentials'].stat().st_mode & 0o777 == 0o600
    assert console.paths['directory'].stat().st_mode & 0o777 == 0o700
    public = json.dumps(status)
    assert CREDS.ssid.decode() not in public and CREDS.passphrase.decode() not in public


@pytest.mark.parametrize('resource', ['wifi', 'ble', 'video'])
def test_cleanup_unknown_for_any_resource_forbids_reconnect(harness, resource):
    h, console = harness, harness.console
    setattr(h, resource+'_cleanup', False)
    h.connect()
    h.stop()
    assert console.status()['state'] == 'failed' and not console.status()['cleanup_verified']
    with pytest.raises(RuntimeError, match='cleanup_unconfirmed'):
        console.connect()


@pytest.mark.parametrize('failure', ['route', 'ble_before_credentials', 'wifi_constructor',
    'wifi_enter', 'session_before_snapshot', 'session_after_snapshot', 'ble_after_retention'])
def test_failure_at_each_resource_stage_preserves_verified_cleanup(harness, failure):
    h, console = harness, harness.console
    h.failure = failure
    console.connect()
    if failure == 'ble_after_retention':
        wait_for(lambda: h.sessions and h.sessions[0].started.is_set())
        console.disconnect()
    h.join()
    status = console.status()
    assert status['state'] == 'failed' and status['error'].startswith('synthetic_')
    assert status['cleanup_verified'] and not status['connected']
    if failure == 'route':
        assert not h.ble_calls and 'wifi.create' not in h.trace
    if failure in ('route', 'ble_before_credentials', 'wifi_constructor', 'wifi_enter'):
        assert not h.sessions
    if failure == 'wifi_enter':
        assert 'wifi.failed_enter_cleanup' in h.trace
    assert (console.paths['directory']/'console-result.json').exists()


@pytest.mark.parametrize('before_snapshot', [False, True])
def test_missing_hls_cleanup_callback_blocks_reconnect_even_without_first_snapshot(harness, before_snapshot):
    h, console = harness, harness.console
    h.omit_video_cleanup = True
    h.failure = 'session_before_snapshot' if before_snapshot else 'session_after_snapshot'
    console.connect()
    h.join()
    assert not console.status()['cleanup_verified']
    with pytest.raises(RuntimeError, match='cleanup_unconfirmed'):
        console.connect()


def test_cancel_during_ble_setup_does_not_escape_worker_or_leave_unknown_cleanup(harness, monkeypatch):
    h, console = harness, harness.console
    uncaught = []
    monkeypatch.setattr(threading, 'excepthook', lambda args: uncaught.append(args.exc_value))
    h.pause_ble = True
    console.connect()
    wait_for(lambda: h.ble_calls)
    console.disconnect()
    h.join()
    assert not uncaught
    assert console.status()['cleanup_verified'] and not console.status()['connected']
    assert not h.sessions and 'wifi.create' not in h.trace


def test_missing_ble_cleanup_event_blocks_reconnect(harness):
    h, console = harness, harness.console
    h.failure, h.ble_cleanup = 'ble_before_credentials', None
    console.connect()
    h.join()
    assert not console.status()['cleanup_verified']
    with pytest.raises(RuntimeError, match='cleanup_unconfirmed'):
        console.connect()


def test_current_preview_is_live_only_while_both_video_and_receipt_are_fresh(harness):
    h, console = harness, harness.console
    Path(h.config.last_frame).write_bytes(b'reference-fixture')
    call = h.connect()
    fresh = {'last_encoded_age_seconds': 0, 'preview_age_seconds': 0,
             'frames_encoded': 1, 'preview_available': True}
    call.publish(ready=True, video=fresh)
    assert console.status()['frame']['available'] and not console.status()['video']['live']
    preview = Path(call.kwargs['hls_directory'])/'preview.jpg'
    preview.parent.mkdir(mode=0o700)
    preview.write_bytes(b'current-jpeg-fixture')
    call.publish(video={**fresh, 'preview_path': str(preview)})
    assert console.status()['video']['live']
    preview.unlink()
    assert console.frame_path() == Path(h.config.last_frame)
    assert not console.status()['video']['live'], 'fallback reference is never a live preview'
    preview.write_bytes(b'current-jpeg-fixture')
    h.clock.advance(2.1)
    assert not console.status()['connected'] and not console.status()['video']['live']
    call.publish(video={**fresh, 'last_encoded_age_seconds': 3})
    assert console.status()['connected'] and not console.status()['video']['live']
    call.publish(video={**fresh, 'preview_age_seconds': 4})
    assert not console.status()['video']['live']
    h.stop()
    assert not console.status()['video']['live']


def test_expired_browser_intent_never_emits_and_reconnect_starts_with_empty_channel(harness):
    h, console = harness, harness.console
    first = h.connect()
    first.publish(ready=True)
    old_channel, old_paths = console.channel, dict(console.paths)
    console.command('record', {})
    h.clock.advance(3)
    assert console.status()['last_command']['state'] == 'expired'
    assert first.send_once() is None and not first.sent
    h.stop()
    second = h.connect()
    assert console.channel is not old_channel and console.paths['directory'] != old_paths['directory']
    assert console.status()['last_command'] is None and not console.status()['camera_ready']
    with pytest.raises(RuntimeError, match='camera_not_ready'):
        old_channel.submit('record', {})
    second.publish(ready=True)
    assert second.send_once() is None
    h.stop()


def test_late_snapshot_from_old_session_cannot_connect_or_refresh_new_generation(harness):
    h, console = harness, harness.console
    first = h.connect()
    old_sink = first.kwargs['snapshot_sink']
    first.publish(ready=True)
    h.stop()
    h.pause_ble = True
    console.connect()
    wait_for(lambda: len(h.ble_calls) == 2)
    old_sink(first.snapshot(video={'last_encoded_age_seconds': 0, 'preview_age_seconds': 0}))
    status = console.status()
    assert status['state'] == 'connecting' and not status['connected']
    assert console.latest is None and status['last_command'] is None
    console.disconnect()
    h.release_ble.set()
    h.join()


def test_disconnect_racing_with_status_cannot_enqueue_after_stop(harness, monkeypatch):
    h, console = harness, harness.console
    call = h.connect()
    call.publish(ready=True)
    call.pause_exit = True
    original, raced = console.status, False
    def status_then_stop():
        nonlocal raced
        snapshot = original()
        if not raced:
            raced = True
            console.disconnect()
        return snapshot
    monkeypatch.setattr(console, 'status', status_then_stop)
    try:
        with pytest.raises(RuntimeError):
            console.command('record', {})
        assert not console.channel.snapshot()['pending'] and not call.sent
    finally:
        call.release_exit.set()
        h.join()


def test_command_in_progress_cannot_be_retargeted_to_a_reconnected_session(harness, monkeypatch):
    h, console = harness, harness.console
    first = h.connect()
    first.publish(ready=True)
    old_channel, old_generation = console.channel, console.generation
    captured, release, decided = threading.Event(), threading.Event(), threading.Event()
    errors, outcomes = [], []
    original = console.status

    def gated_status():
        snapshot = original()
        if threading.current_thread() is command_worker:
            captured.set()
            assert release.wait(2), 'test must release the command status read'
        return snapshot

    monkeypatch.setattr(console, 'status', gated_status)
    def command():
        try:
            outcomes.append(console.command('record', {}))
        except RuntimeError as error:
            outcomes.append(str(error))
        except BaseException as error:
            errors.append(error)

    def reconnect():
        try:
            # If command holds the lifecycle lock, release its status read
            # before asking the public disconnect/reconnect methods to run.
            # Otherwise finish the restart first: this recreates the original
            # stale-status→new-channel window without a timing-based sleep.
            acquired = console.lock.acquire(blocking=False)
            if acquired:
                console.lock.release()
            else:
                decided.set()
            h.stop()
            second = h.connect()
            second.publish(ready=True)
        except BaseException as error:
            errors.append(error)
        finally:
            decided.set()

    command_worker = threading.Thread(target=command)
    reconnect_worker = threading.Thread(target=reconnect)
    try:
        command_worker.start()
        assert captured.wait(1)
        reconnect_worker.start()
        assert decided.wait(2)
    finally:
        release.set()
        command_worker.join(2)
        if reconnect_worker.ident is not None:
            reconnect_worker.join(2)
    assert not command_worker.is_alive() and not reconnect_worker.is_alive() and not errors, errors
    assert outcomes and len(h.sessions) == 2
    assert console.generation != old_generation and console.channel is not old_channel
    assert not console.channel.snapshot()['pending']
    assert console.channel.snapshot()['last_command'] is None
    assert not old_channel.snapshot()['pending'] and not first.sent
    assert h.sessions[1].send_once() is None
    h.stop()


def test_close_is_final_and_keeps_completed_resources_closed(harness):
    h, console = harness, harness.console
    h.connect()
    console.close()
    assert console.status()['cleanup_verified'] and not console.thread.is_alive()
    with pytest.raises(RuntimeError, match='console_stopping'):
        console.connect()


def test_startup_refresh_is_forwarded_only_by_explicit_preview_configuration(harness):
    h,console=harness,harness.console
    console.config=replace(h.config,preview_only=True,startup_keyframe_refresh=True)
    call=h.connect()
    assert call.kwargs['preview_only'] is True
    assert call.kwargs['startup_keyframe_refresh'] is True
    assert console.status()['startup_keyframe_refresh'] is True
    assert not any(console.status()['capabilities'].values())
    h.stop()
