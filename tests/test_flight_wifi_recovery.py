"""The real owner wrapper recovers only before BLE, under its device lease."""
from dataclasses import replace
from pathlib import Path
import time

import pytest

from neo2_control.device_lease import DeviceLease
from neo2_control.wifi_link import WifiLinkError
import neo2_control.wifi_memory as wifi
from test_flight_connection import Harness,wait_for


def test_default_wifi_uses_one_stable_store_and_recovers_before_guard_or_ble(tmp_path,monkeypatch):
    h=Harness(tmp_path)
    h.backend.connection.config=replace(h.config,incident_directory=tmp_path/'incidents')
    h.backend.connection.wifi_factory=None
    expected=tmp_path/'incidents'/'wifi'
    seen=[]
    async def recover(interface,directory,**kwargs):
        assert interface==h.config.interface and directory==expected
        with pytest.raises(RuntimeError):
            with DeviceLease(interface,h.config.ble_address,directory=tmp_path/'leases'):pass
        assert h.backend.connection.cleanup['wifi'] is None
        h.trace.append('recover');return {'cleanup_verified':True,'recovered':True}
    def link(credentials,interface,**kwargs):
        seen.append(kwargs.pop('ownership_directory'))
        return h.wifi(credentials,interface,**kwargs)
    monkeypatch.setattr(wifi,'recover_memory_wifi',recover)
    monkeypatch.setattr(wifi,'MemoryWifiLink',link)
    try:
        h.backend.open(time.monotonic());wait_for(lambda:bool(h.sessions))
        assert h.trace[:3]==['recover','guard','ble.enter'] and seen==[expected]
    finally:
        h.backend.close(time.monotonic());h.backend.finish_cleanup()
    assert h.backend.connection.status()['cleanup_verified']
    path=h.backend.connection.paths['directory']/'wifi-recovery.json'
    assert path.is_file() and path.stat().st_mode&0o777==0o600


def test_unresolved_orphan_never_enters_ble_and_is_not_reported_clean(tmp_path,monkeypatch):
    h=Harness(tmp_path);h.backend.connection.wifi_factory=None
    async def refuse(*args,**kwargs):raise WifiLinkError('wifi_recovery_unconfirmed')
    monkeypatch.setattr(wifi,'recover_memory_wifi',refuse)
    h.backend.open(time.monotonic())
    wait_for(lambda:not h.backend.connection.status()['worker_alive'])
    status=h.backend.connection.status()
    assert status['error']=='wifi_recovery_unconfirmed' and status['phase']=='failed'
    assert status['cleanup']['wifi'] is not True and not status['cleanup_verified']
    assert not h.trace and not h.sessions
    with pytest.raises(RuntimeError,match='cleanup_unconfirmed'):h.backend.finish_cleanup()


def test_custom_test_backend_never_runs_default_nm_recovery(tmp_path,monkeypatch):
    h=Harness(tmp_path)
    async def forbidden(*args,**kwargs):raise AssertionError('unexpected default WiFi recovery')
    monkeypatch.setattr(wifi,'recover_memory_wifi',forbidden)
    try:
        h.backend.open(time.monotonic());wait_for(lambda:bool(h.sessions))
        assert h.trace[:2]==['guard','ble.enter']
    finally:
        h.backend.close(time.monotonic());h.backend.finish_cleanup()
