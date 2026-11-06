import asyncio
import threading
import pytest
from neo2_control.ble_bootstrap import WifiCredentials
from neo2_control.macos_wifi import ManualMacWifiLink, aircraft_ipv4_ready
from types import SimpleNamespace


def test_manual_link_requires_exact_ssid_and_bssid_without_changing_network():
    credentials = WifiCredentials(b'123456', b'synthetic-drone', b'private-password')
    async def exercise():
        link = ManualMacWifiLink(credentials, 'en0', observer=lambda _: (credentials.ssid, credentials.bssid),
                                 ipv4_checker=lambda _:True)
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
                              clock=lambda:now[0], ipv4_checker=lambda _:True)
        async with link:
            assert link.ready()
            now[0]=1.6
            assert not link.ready()
        assert not link.ready()
    asyncio.run(exercise())


@pytest.mark.parametrize('address,mask,router,expected', [
    ('192.168.2.2','255.255.255.0','192.168.2.1',True),
    ('192.168.1.10','255.255.255.0','192.168.1.1',False),
    ('192.168.2.2','255.255.255.0','192.168.1.1',False),
    ('169.254.1.1','255.255.0.0','192.168.2.1',False),
    ('192.168.2.1','255.255.255.0','192.168.2.1',False),
    ('192.168.2.255','255.255.255.0','192.168.2.1',False),
    ('invalid','255.255.255.0','192.168.2.1',False),
])
def test_ipv4_gate_rejects_stale_or_incomplete_dhcp(address,mask,router,expected):
    outputs=iter((address,mask,router)); calls=[]
    def runner(argv,**kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=0,stdout=next(outputs))
    assert aircraft_ipv4_ready('en0',runner=runner) is expected
    assert all(call[0]=='/usr/sbin/ipconfig' for call in calls)


def test_matching_wifi_identity_does_not_start_session_without_ipv4():
    credentials=WifiCredentials(b'123456',b'synthetic',b'private-password')
    async def exercise():
        times=iter((0,0,0,1))
        with pytest.raises(RuntimeError,match='manually_join'):
            async with ManualMacWifiLink(credentials,'en0',timeout=.1,
                observer=lambda _:(credentials.ssid,credentials.bssid),
                ipv4_checker=lambda _:False,clock=lambda:next(times)):pass
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
