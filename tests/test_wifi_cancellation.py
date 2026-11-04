import subprocess
import shutil
import sys
import threading

import pytest

from neo2_control import wifi_link as wifi
from neo2_control.ble_bootstrap import WifiCredentials
from test_wifi_link import network, plan


def test_cancellable_command_retries_communication_without_resending_stdin():
    stop = threading.Event()
    value = 'synthetic-private-input\n' * 100
    result = wifi._run(sys.executable, '-c',
        'import sys,time; data=sys.stdin.read(); time.sleep(.15); sys.stdout.write(data)',
        input_text=value, cancel_event=stop, timeout=2)
    assert result.stdout == value and result.returncode == 0


def test_cancellation_reaps_the_owned_command_and_does_not_expose_stderr(monkeypatch):
    stop, started = threading.Event(), threading.Event()
    processes = []
    original = subprocess.Popen
    def spawn(*args, **kwargs):
        process = original(*args, **kwargs)
        processes.append(process)
        started.set()
        return process
    monkeypatch.setattr(subprocess, 'Popen', spawn)
    def cancel():
        assert started.wait(1)
        stop.set()
    worker = threading.Thread(target=cancel)
    worker.start()
    try:
        with pytest.raises(wifi.WifiLinkCancelled, match='connection_cancelled') as error:
            wifi._run(sys.executable, '-c', 'import time;time.sleep(30)',
                      cancel_event=stop, timeout=5)
        assert str(error.value) == 'wifi_connection_cancelled'
    finally:
        worker.join(timeout=1)
    assert len(processes) == 1 and processes[0].poll() is not None


def test_timeout_reaps_command_and_uses_local_error_code(monkeypatch):
    processes = []
    original = subprocess.Popen
    def spawn(*args, **kwargs):
        process = original(*args, **kwargs)
        processes.append(process)
        return process
    monkeypatch.setattr(subprocess, 'Popen', spawn)
    with pytest.raises(wifi.WifiLinkError, match='bounded_operation_failed'):
        wifi._run(sys.executable, '-c', 'import time;time.sleep(30)', cancel_event=threading.Event(),
                  timeout=.05, reason='bounded_operation_failed')
    assert len(processes) == 1 and processes[0].poll() is not None


def test_precancel_never_starts_a_subprocess(monkeypatch):
    stop = threading.Event()
    stop.set()
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **kw: pytest.fail('no subprocess after cancellation'))
    with pytest.raises(wifi.WifiLinkCancelled):
        wifi._run('nmcli', cancel_event=stop)


@pytest.mark.skipif(shutil.which('nmcli') is None, reason='real NetworkManager offline parser unavailable')
def test_real_offline_parser_accepts_plan_through_cancellable_stdin_path():
    import configparser
    credentials = WifiCredentials(bytes.fromhex('020000000002'), b'NEO2-CANCEL-TEST', b'synthetic-passphrase')
    plan = wifi.prepare_wifi_plan(credentials, 'wlantest0', cancel_event=threading.Event())
    parsed = configparser.ConfigParser(interpolation=None)
    parsed.read_string(plan.keyfile)
    assert parsed['wifi-security']['psk'] == 'synthetic-passphrase'
    assert parsed['ipv4']['never-default'] == 'true'
    assert parsed['ipv4']['ignore-auto-dns'] == 'true'
    assert parsed['ipv6']['method'] == 'disabled'


def test_cancellation_after_profile_load_still_deactivates_deletes_and_verifies(network):
    stop = threading.Event()
    def event(name, **metadata):
        if name == 'wifi_activated':
            stop.set()
    link = wifi.DedicatedWifiLink(plan(), event, cancel_event=stop)
    with pytest.raises(wifi.WifiLinkCancelled):
        with link:
            pytest.fail('cancellation must precede transport')
    assert link.ever_created and link.cleanup_verified and link.removed and link.deactivated
    assert not network.connected and not link.path.exists()
    assert any('down' in call for call in network.calls)
    assert any('delete' in call for call in network.calls)


def test_cleanup_audit_failure_cannot_skip_deactivation_and_profile_removal(network):
    def event(name, **metadata):
        if name == 'wifi_cleanup_started':
            raise OSError('synthetic journal failure')
    link = wifi.DedicatedWifiLink(plan(), event)
    with pytest.raises(OSError, match='journal failure'):
        with link:
            pass
    assert link.cleanup_verified and link.removed and link.deactivated
    assert not network.connected and not link.path.exists()


@pytest.mark.parametrize('active', ['still_active', 'malformed'])
def test_cleanup_does_not_claim_success_when_active_uuid_check_is_bad(network, monkeypatch, active):
    original = network.run
    link = wifi.DedicatedWifiLink(plan(), lambda *a, **kw: None)
    def command(*args, **kwargs):
        result = original(*args, **kwargs)
        if '--active' in args:
            result.stdout = link.plan.connection_uuid if active == 'still_active' else 'unparseable\x1b'
        return result
    monkeypatch.setattr(wifi, '_run', command)
    with pytest.raises(wifi.WifiLinkError, match='wifi_cleanup_failed'):
        with link:
            pass
    assert link.removed and not link.cleanup_verified and not link.deactivated
