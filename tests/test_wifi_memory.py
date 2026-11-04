"""Memory-only NM ownership using fake D-Bus and strictly read-only commands."""
import asyncio
import builtins
from copy import deepcopy
import importlib.util
import json
import subprocess
import threading
from types import SimpleNamespace

import pytest

from neo2_control import wifi_link, wifi_memory as wifi
from neo2_control.ble_bootstrap import WifiCredentials


SSID, PASSWORD = b'PRIVATE-SSID-TEST', b'private-test-passphrase'
CREDS = WifiCredentials(bytes.fromhex('020000000002'), SSID, PASSWORD)
FOREIGN = '00000000-0000-4000-8000-000000000001'
ROOT = wifi.MANAGER
OWN_PATH, ACTIVE = ROOT+'/Settings/2', ROOT+'/ActiveConnection/2'
OLD_PATH, OLD_ACTIVE = ROOT+'/Settings/1', ROOT+'/ActiveConnection/1'
DEVICE, AP = ROOT+'/Devices/9', ROOT+'/AccessPoint/9'


class Variant:
    def __init__(self, signature, value):
        self.signature, self.value = signature, value


class Network:
    def __init__(self):
        self.commands, self.calls, self.events = [], [], []
        self.settings = None
        self.profile_exists = self.active = False
        self.default_device = 'eth0'
        self.device_type, self.idle_state = 'wifi', '30 (disconnected)'
        self.activation_state = 100
        self.bad_route = self.change_before = self.change_after = False
        self.bad_bssid = self.bad_ssid = self.bad_active_uuid = False
        self.fail_add = self.fail_activation = self.fail_delete = False
        self.return_existing = self.proxy_failure = self.drop_foreign = False
        self.unsaved, self.disconnected = True, 0
        self.corrupt = None
        self.stop = None
        self.pause_add = self.pause_activate = self.pause_delete = False
        self.add_started, self.release_add = asyncio.Event(), asyncio.Event()
        self.activation_started, self.release_activation = asyncio.Event(), asyncio.Event()
        self.delete_started, self.release_delete = asyncio.Event(), asyncio.Event()

    @property
    def uuid(self):
        return self.settings['connection']['uuid'].value

    def command(self, *args, **kwargs):
        self.commands.append(args)
        joined = ' '.join(args)
        assert PASSWORD.decode() not in joined and SSID.decode() not in joined
        assert args[0] in ('ip', 'nmcli')
        assert not {'up', 'down', 'delete', 'modify', 'load', 'disconnect', 'sudo'} & set(args)
        if args[:3] == ('nmcli', '--get-values', 'UUID'):
            values = [] if self.drop_foreign else [FOREIGN]
            if self.settings is not None and (self.active if '--active' in args else self.profile_exists):
                values.append(self.uuid)
            return SimpleNamespace(stdout='\n'.join(values), returncode=0)
        if args[:3] == ('nmcli', '--get-values', 'GENERAL.TYPE,GENERAL.STATE'):
            state = '100 (connected)' if self.active else self.idle_state
            return SimpleNamespace(stdout=self.device_type+'\n'+state+'\n', returncode=0)
        if args[:3] == ('nmcli', '--get-values', 'GENERAL.CON-UUID'):
            return SimpleNamespace(stdout=FOREIGN if self.bad_active_uuid else self.uuid, returncode=0)
        if args[0] == 'ip' and 'default' in args:
            rows = [{'dev': self.default_device, 'gateway': '192.0.2.1'}] if '-4' in args else []
            if (self.change_before and self.profile_exists) or (self.change_after and self.active):
                rows.append({'dev': 'changed0', 'gateway': '192.0.2.9'})
            return SimpleNamespace(stdout=json.dumps(rows), returncode=0)
        if args[0] == 'ip' and 'get' in args:
            return SimpleNamespace(stdout=json.dumps([{'dst': '192.168.2.1',
                'dev': 'eth0' if self.bad_route else 'wlanusb0'}]), returncode=0)
        raise AssertionError(args)

    async def connect(self):
        self.calls.append(('connect',))
        return self

    def disconnect(self):
        self.disconnected += 1

    async def introspect(self, service, path):
        assert service == wifi.NM
        return path

    def get_proxy_object(self, service, path, tree):
        if self.proxy_failure and path == OWN_PATH:
            raise RuntimeError(PASSWORD.decode())
        network = self
        class Proxy:
            def get_interface(self, interface):
                class Methods:
                    def __getattr__(self, name):
                        async def call(*args):
                            return await network.handle(path, name, args)
                        return call
                return Methods()
        return Proxy()

    async def call(self, message):
        self.calls.append(('raw', message.path, message.member))
        snake = []
        for i, char in enumerate(message.member):
            snake.append(('_' if char.isupper() and i else '')+char.lower())
        if message.member == 'Get':
            member = message.body[1]
            snake = []
            for i, char in enumerate(member):
                snake.append(('_' if char.isupper() and i else '')+char.lower())
            value = await self.handle(message.path, 'get_'+''.join(snake), ())
            return SimpleNamespace(error_name=None, body=[Variant('v', value)])
        value = await self.handle(message.path, 'call_'+''.join(snake), tuple(message.body))
        return SimpleNamespace(error_name=None, body=[] if value is None else [value])

    async def handle(self, path, method, args):
        self.calls.append((path, method))
        if method == 'call_list_connections':
            return ([] if self.drop_foreign else [OLD_PATH])+([OWN_PATH] if self.profile_exists else [])
        if method == 'call_get_connection_by_uuid':
            if self.profile_exists and args == (self.uuid,):
                return OWN_PATH
            raise RuntimeError('unknown synthetic UUID')
        if method == 'call_add_connection_unsaved':
            self.settings = deepcopy(args[0])
            self.profile_exists = not self.return_existing
            self.add_started.set()
            if self.pause_add:
                await self.release_add.wait()
            if self.fail_add:
                raise RuntimeError(PASSWORD.decode())
            return OLD_PATH if self.return_existing else OWN_PATH
        if method == 'call_get_settings':
            assert path == OWN_PATH
            values = deepcopy(self.settings)
            values['802-11-wireless-security'].pop('psk', None)
            if self.corrupt:
                self.corrupt(values)
            return values
        if method == 'get_unsaved':
            return self.unsaved
        if method == 'get_active_connections':
            return [OLD_ACTIVE]+([ACTIVE] if self.active else [])
        if method == 'call_get_device_by_ip_iface':
            assert args == ('wlanusb0',)
            return DEVICE
        if method == 'call_activate_connection':
            assert args == (OWN_PATH, DEVICE, '/')
            self.active = True
            self.activation_started.set()
            if self.pause_activate:
                await self.release_activation.wait()
            if self.stop:
                self.stop.set()
            if self.fail_activation:
                raise RuntimeError(PASSWORD.decode())
            return ACTIVE
        if method == 'get_state':
            return self.activation_state
        if method == 'get_interface':
            return 'wlanusb0'
        if method == 'get_device_type':
            return 2
        if method == 'get_uuid':
            return self.uuid if path == ACTIVE else FOREIGN
        if method == 'get_connection':
            return OWN_PATH if path == ACTIVE else OLD_PATH
        if method == 'get_devices':
            return [DEVICE] if path == ACTIVE else [ROOT+'/Devices/1']
        if method == 'get_active_access_point':
            return AP
        if method == 'get_hw_address':
            return '02:00:00:00:00:03' if self.bad_bssid else CREDS.bssid.hex(':')
        if method == 'get_ssid':
            return b'other-private-network' if self.bad_ssid else SSID
        if method == 'call_deactivate_connection':
            assert args == (ACTIVE,), 'never deactivate a baseline connection'
            self.active = False
            return None
        if method == 'call_delete':
            assert path == OWN_PATH, 'never delete a baseline profile'
            self.delete_started.set()
            if self.pause_delete:
                await self.release_delete.wait()
            if self.fail_delete:
                raise RuntimeError(PASSWORD.decode())
            self.profile_exists = False
            return None
        raise AssertionError((path, method, args))


@pytest.fixture
def network(monkeypatch):
    system = Network()
    monkeypatch.setattr(wifi, '_new_bus', lambda: system)
    monkeypatch.setattr(wifi, '_variant', Variant)
    monkeypatch.setattr(wifi, '_message', lambda **kw: SimpleNamespace(**kw))
    monkeypatch.setattr(wifi, '_run', system.command)
    monkeypatch.setattr(wifi_link, '_run', system.command)
    monkeypatch.setattr(subprocess, 'run', lambda *a, **k: pytest.fail('unexpected real subprocess'))
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **k: pytest.fail('unexpected real subprocess'))
    return system


def make_link(network, **kwargs):
    return wifi.MemoryWifiLink(CREDS, 'wlanusb0',
        event_sink=lambda name, **detail: network.events.append((name, detail)), **kwargs)


def test_construction_snapshot_and_double_exit_are_inert_and_redacted(network):
    link = make_link(network)
    assert link.snapshot()['state'] == 'new' and not network.calls and not network.commands
    assert SSID.decode() not in repr(link)+json.dumps(link.snapshot())
    assert PASSWORD.decode() not in repr(link)+json.dumps(link.snapshot())
    async def run():
        await link.__aexit__(None, None, None)
        await link.__aexit__(None, None, None)
    asyncio.run(run())
    assert not network.calls and not network.commands


def test_fresh_module_import_and_construction_do_not_import_dbus_or_run_commands(network, monkeypatch):
    original = builtins.__import__
    def load(name, *args, **kwargs):
        if name == 'dbus_fast' or name.startswith('dbus_fast.'):
            pytest.fail('D-Bus must remain lazy until async entry')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', load)
    spec = importlib.util.spec_from_file_location('neo2_control._memory_inert_test', wifi.__file__)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.MemoryWifiLink(CREDS, 'wlanusb0').snapshot()['state'] == 'new'
    assert not network.calls and not network.commands


def test_owned_memory_profile_association_and_baseline_are_verified_then_removed(network):
    link = make_link(network)
    async def run():
        async with link as entered:
            assert entered is link and link.connected and network.active
            snapshot = link.snapshot()
            assert snapshot['profile_verified'] and snapshot['route_verified']
            assert snapshot['drone_route'] == '192.168.2.1/32' and not snapshot['ipv6']
            assert network.settings['connection']['autoconnect'].value is False
            assert network.settings['ipv4']['ignore-auto-routes'].value is True
        calls, commands = len(network.calls), len(network.commands)
        await link.__aexit__(None, None, None)
        assert (len(network.calls), len(network.commands)) == (calls, commands)
        with pytest.raises(wifi.WifiLinkError, match='cannot_be_reused'):
            await link.__aenter__()
    asyncio.run(run())
    result = link.snapshot()
    assert result['cleanup_verified'] and result['profile_removed'] and result['deactivated']
    assert result['default_routes_preserved'] and result['existing_profiles_preserved']
    assert result['existing_active_profiles_preserved'] and result['interface_idle_after_cleanup']
    assert not link.connected and not network.profile_exists and not network.active
    assert network.disconnected == 1
    output = json.dumps(result)+json.dumps(network.events)
    assert SSID.decode() not in output and PASSWORD.decode() not in output


@pytest.mark.parametrize('change', ['active', 'ethernet', 'default', 'nexthop'])
def test_interface_and_default_route_guards_precede_any_dbus_creation(network, monkeypatch, change):
    if change == 'active':
        network.idle_state = '100 (connected)'
    elif change == 'ethernet':
        network.device_type = 'ethernet'
    elif change == 'default':
        network.default_device = 'wlanusb0'
    else:
        original = network.command
        def command(*args, **kwargs):
            if args[0] == 'ip' and 'default' in args:
                return SimpleNamespace(stdout=json.dumps([{'nexthops': [{'dev': 'wlanusb0'}]}]))
            return original(*args, **kwargs)
        monkeypatch.setattr(wifi_link, '_run', command)
    link = make_link(network)
    with pytest.raises(wifi.WifiLinkError):
        asyncio.run(link.__aenter__())
    assert not network.calls and not network.profile_exists


@pytest.mark.parametrize('attribute', ['bad_route', 'bad_bssid', 'bad_ssid',
    'bad_active_uuid', 'change_before', 'change_after', 'fail_activation'])
def test_failed_activation_or_verification_cleans_only_the_owned_objects(network, attribute):
    setattr(network, attribute, True)
    link = make_link(network)
    with pytest.raises(wifi.WifiLinkError) as error:
        asyncio.run(link.__aenter__())
    assert PASSWORD.decode() not in str(error.value)
    assert not network.profile_exists and not network.active
    assert link.profile_removed and link.deactivated
    if attribute != 'change_before':
        assert link.cleanup_verified


@pytest.mark.parametrize('corruption', ['bssid', 'ssid', 'autoconnect', 'never_default',
    'dns', 'ipv6', 'route_prefix', 'route_gateway', 'unsaved'])
def test_stored_profile_is_verified_before_activation(network, corruption):
    def change(settings):
        if corruption in ('bssid', 'ssid'):
            settings['802-11-wireless'][corruption].value = b'changed'
        elif corruption == 'autoconnect':
            settings['connection']['autoconnect'].value = True
        elif corruption == 'never_default':
            settings['ipv4']['never-default'].value = False
        elif corruption == 'dns':
            settings['ipv4']['dns'] = Variant('au', [1])
        elif corruption == 'ipv6':
            settings['ipv6']['method'].value = 'auto'
        elif corruption == 'route_prefix':
            settings['ipv4']['route-data'].value[0]['prefix'].value = 24
        elif corruption == 'route_gateway':
            settings['ipv4']['route-data'].value[0]['next-hop'] = Variant('s', '192.168.2.1')
    network.corrupt = change
    if corruption == 'unsaved':
        network.unsaved = False
    link = make_link(network)
    with pytest.raises(wifi.WifiLinkError):
        asyncio.run(link.__aenter__())
    assert not any(len(call) == 2 and call[1] == 'call_activate_connection' for call in network.calls)
    assert link.cleanup_verified and not network.profile_exists


def test_proxy_creation_failure_uses_direct_dbus_for_owned_verification_and_cleanup(network):
    network.proxy_failure = True
    link = make_link(network)
    async def run():
        async with link:
            assert link.connected
    asyncio.run(run())
    assert link.cleanup_verified
    assert ('raw', OWN_PATH, 'GetSettings') in network.calls
    assert ('raw', OWN_PATH, 'Delete') in network.calls


def test_failed_add_reply_recovers_only_the_new_matching_uuid_for_cleanup(network):
    network.fail_add = True
    link = make_link(network)
    with pytest.raises(wifi.WifiLinkError, match='wifi_memory_connection_failed'):
        asyncio.run(link.__aenter__())
    assert link.cleanup_verified and not network.profile_exists


def test_existing_object_returned_by_add_is_never_deleted_or_activated(network):
    network.return_existing = True
    link = make_link(network)
    with pytest.raises(wifi.WifiLinkError, match='not_owned'):
        asyncio.run(link.__aenter__())
    assert not any(len(call) == 2 and call[1] in ('call_delete', 'call_activate_connection') for call in network.calls)
    assert not link.cleanup_verified


def test_existing_uuid_collision_never_creates_or_deletes_a_profile(network):
    link = make_link(network)
    link.connection_uuid = FOREIGN
    with pytest.raises(wifi.WifiLinkError, match='already_exists'):
        asyncio.run(link.__aenter__())
    assert not network.calls and not network.profile_exists


def test_changed_profile_identity_is_not_deleted_as_if_still_owned(network):
    network.corrupt = lambda settings: setattr(settings['connection']['uuid'], 'value', FOREIGN)
    link = make_link(network)
    with pytest.raises(wifi.WifiLinkError, match='identity_changed'):
        asyncio.run(link.__aenter__())
    assert not any(len(call) == 2 and call[1] == 'call_delete' for call in network.calls)
    assert not link.cleanup_verified and not network.active


def test_foreign_active_object_is_never_deactivated_even_when_returned_by_activation(network):
    original = network.handle
    async def handle(path, method, args):
        if path == ACTIVE and method == 'get_uuid':
            return FOREIGN
        return await original(path, method, args)
    network.handle = handle
    link = make_link(network)
    with pytest.raises(wifi.WifiLinkError, match='selected_profile_is_not_active'):
        asyncio.run(link.__aenter__())
    assert not any(len(call) == 2 and call[1] == 'call_deactivate_connection' for call in network.calls)
    assert not link.cleanup_verified and not link.deactivated


@pytest.mark.parametrize('during', ['add', 'activate'])
def test_task_cancellation_waits_for_owned_handle_then_removes_profile(network, during):
    setattr(network, 'pause_'+during, True)
    link = make_link(network)
    async def run():
        task = asyncio.create_task(link.__aenter__())
        started = network.add_started if during == 'add' else network.activation_started
        release = network.release_add if during == 'add' else network.release_activation
        await asyncio.wait_for(started.wait(), 1)
        task.cancel()
        await asyncio.sleep(.02)
        assert not task.done(), 'cleanup must wait for the pending ownership handle'
        release.set()
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(task, 1)
    asyncio.run(run())
    assert link.cleanup_verified and not network.active and not network.profile_exists


def test_precancel_and_stop_during_activation_have_no_orphaned_profile(network):
    stop = threading.Event()
    stop.set()
    first = make_link(network, stop_event=stop)
    with pytest.raises(wifi.WifiLinkCancelled):
        asyncio.run(first.__aenter__())
    assert not network.calls and not network.commands
    stop.clear()
    network.stop = stop
    second = make_link(network, stop_event=stop)
    with pytest.raises(wifi.WifiLinkCancelled):
        asyncio.run(second.__aenter__())
    assert second.cleanup_verified and not network.active and not network.profile_exists


def test_repeated_cancellation_during_exit_cannot_interrupt_owned_deletion(network):
    network.pause_delete = True
    link = make_link(network)
    async def run():
        await link.__aenter__()
        task = asyncio.create_task(link.__aexit__(None, None, None))
        await asyncio.wait_for(network.delete_started.wait(), 1)
        task.cancel()
        await asyncio.sleep(.01)
        task.cancel()
        network.release_delete.set()
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(task, 1)
        await link.__aexit__(None, None, None)
    asyncio.run(run())
    assert link.cleanup_verified and not network.profile_exists and not network.active


def test_cleanup_waits_for_transient_deactivating_state_before_confirming_idle(network, monkeypatch):
    original_handle, original_command = network.handle, network.command
    cleanup_started = threading.Event()
    loop, states = [], []

    async def handle(path, method, args):
        value = await original_handle(path, method, args)
        if method == 'call_deactivate_connection':
            network.idle_state = '110 (deactivating)'
            cleanup_started.set()
        return value

    def command(*args, **kwargs):
        response = original_command(*args, **kwargs)
        if (cleanup_started.is_set()
                and args[:3] == ('nmcli', '--get-values', 'GENERAL.TYPE,GENERAL.STATE')):
            state = response.stdout.splitlines()[1].split()[0]
            states.append(state)
            if state == '110':
                # Complete deactivation only after the first idle read has
                # captured the transient state, independently of wall time.
                loop[0].call_soon_threadsafe(setattr, network, 'idle_state', '30 (disconnected)')
        return response

    network.handle = handle
    monkeypatch.setattr(wifi, '_run', command)
    monkeypatch.setattr(wifi_link, '_run', command)
    link = make_link(network)
    async def run():
        loop.append(asyncio.get_running_loop())
        async with link:
            assert link.connected
    asyncio.run(run())
    assert states == ['110', '30']
    assert link.cleanup_verified and link.interface_idle_after_cleanup
    assert link.profile_removed and link.deactivated and not network.profile_exists and not network.active
    assert link.default_routes_preserved and link.existing_profiles_preserved
    assert link.existing_active_profiles_preserved


def test_unanswered_creation_has_bounded_cancellation_and_never_claims_verified_cleanup(network, monkeypatch):
    network.pause_add = True
    monkeypatch.setattr(wifi, 'MUTATION_SETTLE_TIMEOUT', .01)
    link = make_link(network)
    async def run():
        task = asyncio.create_task(link.__aenter__())
        await asyncio.wait_for(network.add_started.wait(), 1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(task, 1)
    asyncio.run(run())
    assert not link.cleanup_verified and link.snapshot()['mutation_outcome_unknown']
    assert not network.profile_exists and not network.active
    assert network.disconnected == 1


def test_activation_deadline_cleans_created_profile_before_body(network, monkeypatch):
    monkeypatch.setattr(wifi, 'ACTIVATION_TIMEOUT', .01)
    network.activation_state = 40
    link = make_link(network)
    with pytest.raises(wifi.WifiLinkError, match='activation_timeout'):
        asyncio.run(link.__aenter__())
    assert link.cleanup_verified and not network.profile_exists and not network.active


@pytest.mark.parametrize('stage', ['body', 'connected_event', 'cleanup_event'])
def test_body_and_event_sink_failures_cannot_skip_cleanup(network, stage):
    def event(name, **detail):
        network.events.append((name, detail))
        if (stage == 'connected_event' and name == 'wifi_memory_connected'
                or stage == 'cleanup_event' and name == 'wifi_memory_cleanup_started'):
            raise RuntimeError(PASSWORD.decode())
    link = wifi.MemoryWifiLink(CREDS, 'wlanusb0', event_sink=event)
    async def run():
        async with link:
            if stage == 'body':
                raise RuntimeError('synthetic body failure')
    with pytest.raises(RuntimeError) as error:
        asyncio.run(run())
    assert PASSWORD.decode() not in str(error.value)
    assert link.cleanup_verified and not network.profile_exists and not network.active


def test_cleanup_failures_and_foreign_profile_loss_are_reported_without_false_success(network):
    network.fail_delete = True
    link = make_link(network)
    async def run():
        async with link:
            network.drop_foreign = True
    with pytest.raises(wifi.WifiLinkError, match='cleanup_failed'):
        asyncio.run(run())
    assert not link.cleanup_verified and link.deactivated and not link.profile_removed
    assert not link.existing_profiles_preserved and not link.existing_active_profiles_preserved
    assert not network.active


@pytest.mark.parametrize('kwargs', [{'interface': 'bad;command'}, {'stop_event': True},
    {'event_sink': True}, {'credentials': None},
    {'credentials': WifiCredentials(CREDS.bssid, SSID, b'short')}])
def test_invalid_construction_has_no_backend_effects(network, kwargs):
    options = dict(credentials=CREDS, interface='wlanusb0')
    options.update(kwargs)
    with pytest.raises(ValueError):
        wifi.MemoryWifiLink(**options)
    assert not network.calls and not network.commands
