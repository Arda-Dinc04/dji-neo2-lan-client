import asyncio
import threading
import pytest
from neo2_control.ble_bootstrap import WifiCredentials
from neo2_control.macos_wifi import ManualMacWifiLink


def test_manual_link_requires_exact_ssid_and_bssid_without_changing_network():
    credentials = WifiCredentials(b'123456', b'synthetic-drone', b'private-password')
    async def exercise():
        link = ManualMacWifiLink(credentials, 'en0', observer=lambda _: (credentials.ssid, credentials.bssid))
        async with link:assert link.matched
        assert link.snapshot()['cleanup_verified']
        assert not link.snapshot()['network_changed_by_client']
        assert 'private-password' not in str(link.snapshot())
        times = iter((0, 1))
        with pytest.raises(RuntimeError, match='manually_join'):
            async with ManualMacWifiLink(credentials, 'en0', timeout=.1,
                                        observer=lambda _: (b'other', b'654321'), clock=lambda:next(times)):pass
    asyncio.run(exercise())


def test_network_guard_expires_without_a_new_identity_observation():
    credentials=WifiCredentials(b'123456',b'synthetic',b'private-password')
    now=[0.0]
    async def exercise():
        link=ManualMacWifiLink(credentials,'en0',observer=lambda _:(credentials.ssid,credentials.bssid),
                              clock=lambda:now[0])
        async with link:
            assert link.ready()
            now[0]=1.6
            assert not link.ready()
        assert not link.ready()
    asyncio.run(exercise())


def test_manual_join_cancellation_and_privacy_failure_stop_before_session():
    credentials = WifiCredentials(b'123456', b'synthetic', b'private-password')
    async def exercise():
        stop = threading.Event();stop.set()
        with pytest.raises(asyncio.CancelledError):
            async with ManualMacWifiLink(credentials, 'en0', stop_event=stop):pass
        def unavailable(_):raise RuntimeError('macos_wifi_identity_requires_location_permission')
        with pytest.raises(RuntimeError, match='location_permission'):
            async with ManualMacWifiLink(credentials, 'en0', observer=unavailable):pass
    asyncio.run(exercise())
