"""Retained GATT lifetime with owned asynchronous cleanup; no radio."""
import asyncio
import threading
from types import SimpleNamespace

import pytest

from neo2_control import ble_bootstrap as setup
from neo2_control.duml import DumlPacket
from test_ble_bootstrap import Peer, paths


def run(monkeypatch, paths, peer, callback, *, hold_timeout=1, stop=None, events=None):
    monkeypatch.setattr(setup, '_load_bleak_client', lambda: peer.factory)
    return asyncio.run(setup.run_ble_wifi_setup(**paths, while_connected=callback,
        hold_timeout=hold_timeout, stop_event=stop or threading.Event(),
        event_sink=lambda name, **details: events.append((name, details)) if events is not None else None))


def test_context_remains_open_past_setup_deadline_and_acks_late_decisions(monkeypatch, paths):
    paths.update(timeout=.04, request_timeout=.02)
    peer, events = Peer(), []
    async def connected():
        assert peer.is_connected and paths['credentials_path'].exists()
        peer.emit(DumlPacket(7, 2, 0xFFFF, 0, 7, 0x46, b'\0').encode())
        await asyncio.sleep(.12)
        assert peer.is_connected
        assert peer.packets[-1].cmd_id == 0x46
        assert peer.packets[-1].seq == 0xFFFF
        assert peer.packets[-1].payload == b'\0'
    result = run(monkeypatch, paths, peer, connected, events=events)
    assert result.state == 'complete' and result.connection_retained
    assert result.verification_acks == 1 and result.connection_cleanup_verified
    assert peer.exited and not peer.is_connected
    assert [name for name, _ in events][-3:] == [
        'ble_retention_finished', 'ble_cleanup_result', 'setup_result']


@pytest.mark.parametrize('trigger', ['cancel', 'stop', 'timeout', 'notification_error'])
def test_stop_joins_thread_cleanup_before_leaving_ble_context(monkeypatch, paths, trigger):
    peer, stop, cleanup = Peer(), threading.Event(), threading.Event()
    async def connected():
        def worker():
            assert stop.wait(1)
            # Cleanup is deliberately asynchronous with respect to cancellation.
            threading.Event().wait(.08)
            assert peer.is_connected and not peer.exited
            cleanup.set()
        await asyncio.to_thread(worker)
    async def scenario():
        monkeypatch.setattr(setup, '_load_bleak_client', lambda: peer.factory)
        task = asyncio.create_task(setup.run_ble_wifi_setup(**paths,
            while_connected=connected, hold_timeout=.04 if trigger == 'timeout' else 1,
            stop_event=stop))
        while not paths['credentials_path'].exists():
            await asyncio.sleep(.001)
        await asyncio.sleep(.01)
        if trigger == 'cancel':
            task.cancel()
        elif trigger == 'stop':
            stop.set()
        elif trigger == 'notification_error':
            peer.callback(peer.notifier, bytes(513))
        if trigger in ('cancel', 'stop'):
            with pytest.raises(asyncio.CancelledError):
                await task
        else:
            result = await task
            assert result.reason == ('handoff_timeout' if trigger == 'timeout' else 'notification_limit_exceeded')
        assert cleanup.is_set()
    asyncio.run(scenario())
    assert peer.exited and not peer.is_connected


def test_post_credentials_disconnect_is_recorded_while_child_continues(monkeypatch, paths):
    peer = Peer()
    async def connected():
        peer.is_connected = False
        peer.disconnected(peer)
        await asyncio.sleep(.08)
    result = run(monkeypatch, paths, peer, connected)
    assert result.state == 'complete' and result.disconnected_during_hold
    assert result.connection_cleanup_verified and peer.exited


@pytest.mark.parametrize('recover', [True, False])
def test_context_exit_failure_attempts_disconnect_and_verifies_ownership(monkeypatch, paths, recover):
    peer, calls = Peer(), []
    async def broken_exit(*args):
        peer.exited = True
        raise RuntimeError('private backend detail')
    async def disconnect():
        calls.append('disconnect')
        peer.is_connected = not recover
    peer.__class__ = type('BrokenExitPeer', (Peer,), {'__aexit__': broken_exit})
    peer.disconnect = disconnect
    async def connected():
        pass
    result = run(monkeypatch, paths, peer, connected)
    assert calls == ['disconnect']
    assert result.connection_cleanup_verified is recover
    assert result.state == 'failed'
    assert result.reason == ('bluetooth_error' if recover else 'cleanup_unconfirmed')


def test_internal_stop_cannot_mask_failure_during_slow_ble_exit(monkeypatch, paths):
    peer, stop = Peer(), threading.Event()
    async def slow_exit(self, *args):
        await asyncio.sleep(.08)
        self.is_connected = False
    peer.__class__ = type('SlowExitPeer', (Peer,), {'__aexit__': slow_exit})
    async def connected():
        peer.callback(peer.notifier, bytes(513))
        while not stop.is_set():
            await asyncio.sleep(.001)
    result = run(monkeypatch, paths, peer, connected, stop=stop)
    assert result.reason == 'notification_limit_exceeded'
    assert result.connection_cleanup_verified and stop.is_set()
    assert result.state == 'failed'


def test_hold_notification_limits_reset_by_window_but_keep_cumulative_counters():
    recorder = SimpleNamespace(record=lambda **metadata: None)
    exchange = setup._Exchange(recorder, lambda *a, **k: None, 1)
    exchange.holding = True
    exchange.notify_characteristic = SimpleNamespace(handle=1)
    now = [0]
    exchange.notification_clock = lambda: now[0]
    exchange.window_started = 0
    for minute in range(3):
        now[0] = minute * 60
        for _ in range(3000):
            exchange.notification(exchange.notify_characteristic, b'x')
        assert exchange.failure is None
    assert exchange.notifications_received == exchange.received_bytes == 9000
    for _ in range(1097):
        exchange.notification(exchange.notify_characteristic, b'x')
    assert exchange.failure == 'notification_limit_exceeded'


def test_pre_cancelled_setup_never_constructs_backend(monkeypatch, paths):
    stop = threading.Event()
    stop.set()
    monkeypatch.setattr(setup, '_load_bleak_client', lambda: pytest.fail('Bluetooth loaded'))
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(setup.run_ble_wifi_setup(**paths, stop_event=stop))


@pytest.mark.parametrize('options', [
    {'while_connected': lambda: None},
    {'hold_timeout': 1},
    {'while_connected': lambda: None, 'hold_timeout': float('inf'), 'stop_event': threading.Event()},
])
def test_invalid_retention_options_precede_files(paths, options):
    with pytest.raises(ValueError):
        asyncio.run(setup.run_ble_wifi_setup(**paths, **options))
    assert not paths['identity_file'].exists()
    assert not paths['capture_path'].exists()


@pytest.mark.parametrize('recover', [True, False])
def test_hung_context_exit_is_bounded_before_explicit_disconnect(monkeypatch, paths, recover):
    peer, calls = Peer(), []
    async def hung_exit(*args):
        await asyncio.Event().wait()
    async def disconnect():
        calls.append('disconnect')
        if recover:
            peer.is_connected = False
        else:
            await asyncio.Event().wait()
    peer.__class__ = type('HungExitPeer', (Peer,), {'__aexit__': hung_exit})
    peer.disconnect = disconnect
    monkeypatch.setattr(setup, 'BLE_DISCONNECT_TIMEOUT', .03)
    async def connected():
        pass
    result = run(monkeypatch, paths, peer, connected)
    assert calls == ['disconnect']
    assert result.reason == ('disconnect_timeout' if recover else 'cleanup_unconfirmed')
    assert result.connection_cleanup_verified is recover
