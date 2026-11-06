"""Verify an operator-joined Wi-Fi network using CoreWLAN; never associate."""
import asyncio
import time
import subprocess
import json
import threading
import ipaddress
from pathlib import Path

from .ble_bootstrap import WifiCredentials
from .wifi_link import DRONE_HOST


def aircraft_ipv4_ready(interface, *, runner=subprocess.run):
    """Read local DHCP state; never probe an address or change routing."""
    try:
        values = []
        for arguments in (['getifaddr', interface], ['getoption', interface, 'subnet_mask'],
                          ['getoption', interface, 'router']):
            result = runner(['/usr/sbin/ipconfig', *arguments], capture_output=True,
                            text=True, timeout=1, check=False)
            if result.returncode:return False
            values.append(result.stdout.strip())
        address, mask, router = values
        host = ipaddress.IPv4Address(DRONE_HOST)
        local = ipaddress.IPv4Address(address)
        network = ipaddress.IPv4Network(f'{address}/{mask}', strict=False)
        return (ipaddress.IPv4Address(router) == host and host in network
                and local not in (host, network.network_address, network.broadcast_address)
                and not local.is_link_local and not local.is_loopback
                and not local.is_multicast and not local.is_unspecified)
    except (OSError, subprocess.SubprocessError, ValueError):
        return False


def wifi_identity(interface):
    try:
        import CoreWLAN
    except ImportError:
        raise RuntimeError('corewlan_dependency_missing') from None
    device = CoreWLAN.CWWiFiClient.sharedWiFiClient().interfaceWithName_(interface)
    if device is None or not device.powerOn():
        raise RuntimeError('macos_wifi_interface_unavailable')
    ssid, bssid = device.ssidData(), device.bssid()
    if ssid is None or bssid is None:
        helper = Path(__file__).resolve().parents[2]/'build/Neo2WifiCheck.app/Contents/MacOS/Neo2WifiCheck'
        if not helper.is_file():raise RuntimeError('macos_wifi_identity_requires_location_permission_helper')
        try:
            process = subprocess.run([str(helper),interface],capture_output=True,timeout=30,check=False)
            result = json.loads(process.stdout)
            if process.returncode or set(result) != {'ssid_hex','bssid'}:
                raise ValueError
            return bytes.fromhex(result['ssid_hex']), bytes.fromhex(result['bssid'].replace(':',''))
        except (OSError,subprocess.SubprocessError,ValueError,TypeError,KeyError):
            raise RuntimeError('macos_wifi_identity_requires_location_permission') from None
    try:
        return bytes(ssid), bytes.fromhex(str(bssid).replace(':', ''))
    except (TypeError, ValueError):
        raise RuntimeError('macos_wifi_identity_unavailable') from None


def require_wifi_interface(interface, **kwargs):
    # Identity metadata must be available before sending any BLE setup requests.
    wifi_identity(interface)


class ManualMacWifiLink:
    def __init__(self, credentials, interface, *, stop_event=None, timeout=600,
                 observer=wifi_identity, clock=time.monotonic, ipv4_checker=aircraft_ipv4_ready):
        if not isinstance(credentials, WifiCredentials):
            raise ValueError('wifi_credentials_required')
        self.credentials, self.interface, self.stop_event = credentials, interface, stop_event
        self.timeout, self.observer, self.clock = timeout, observer, clock
        self.ipv4_checker = ipv4_checker
        self.matched = False
        self._last_verified = None
        self._stop = threading.Event()
        self._monitor = None

    async def __aenter__(self):
        deadline = self.clock() + self.timeout
        while self.clock() < deadline:
            if self.stop_event is not None and self.stop_event.is_set():
                raise asyncio.CancelledError
            started = self.clock()
            ssid, bssid = self.observer(self.interface)
            if (ssid == self.credentials.ssid and bssid == self.credentials.bssid
                    and self.ipv4_checker(self.interface)):
                self.matched = True
                self._last_verified = started
                self._monitor = threading.Thread(target=self._watch, daemon=True, name='neo2-mac-wifi-identity')
                self._monitor.start()
                return self
            await asyncio.sleep(.25)
        raise RuntimeError('manually_join_aircraft_wifi_before_session')

    async def __aexit__(self, *args):
        # No Wi-Fi profile, association, default route or credentials were changed.
        self._stop.set()
        if self._monitor is not None:
            await asyncio.to_thread(self._monitor.join, 5)

    def _watch(self):
        while not self._stop.wait(.5):
            started = self.clock()
            try:
                ssid, bssid = self.observer(self.interface)
                matched = (ssid == self.credentials.ssid and bssid == self.credentials.bssid
                           and self.ipv4_checker(self.interface))
            except Exception:matched = False
            self.matched = matched
            if not matched:return
            self._last_verified = started

    def ready(self):
        return (self.matched and not self._stop.is_set() and self._last_verified is not None
                and 0 <= self.clock() - self._last_verified <= 1.5)

    def snapshot(self):
        return {'backend': 'macos_manual_association', 'matched_aircraft_network': self.matched,
                'network_changed_by_client': False,
                'cleanup_verified': self._monitor is None or not self._monitor.is_alive(),
                'ipv4_gate': 'dhcp_subnet_and_aircraft_gateway',
                'identity_check_interval_seconds': .5, 'maximum_identity_age_seconds': 1.5}
