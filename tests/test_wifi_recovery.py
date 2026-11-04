"""Orphan-profile recovery against fake NM only; no actual D-Bus or commands."""
import asyncio
from copy import deepcopy
import json
import threading
from types import SimpleNamespace

import pytest

from neo2_control import wifi_memory as wifi
from neo2_control.wifi_ownership import WifiOwnershipStore
from test_wifi_memory import (network, CREDS, SSID, PASSWORD, FOREIGN, OWN_PATH,
    ACTIVE, OLD_PATH, OLD_ACTIVE, DEVICE, ROOT, Variant)
from test_wifi_ownership import IDENTITY


@pytest.fixture
def durable(network,tmp_path,monkeypatch):
    network.identity=deepcopy(IDENTITY)
    network.owner=':1.42'
    network.directory=tmp_path/'wifi'
    network.secrets_denied=False
    monkeypatch.setattr(wifi,'ownership_identity',lambda interface:deepcopy(network.identity))
    original_handle,original_call=network.handle,network.call
    original_introspect,original_proxy=network.introspect,network.get_proxy_object
    async def introspect(service,path):
        assert service in (wifi.NM,network.owner)
        return await original_introspect(wifi.NM,path)
    def proxy(service,path,tree):
        assert service in (wifi.NM,network.owner)
        return original_proxy(wifi.NM,path,tree)
    async def call(message):
        if message.destination==wifi.DBUS:
            assert message.member=='GetNameOwner' and message.body==[wifi.NM]
            return SimpleNamespace(error_name=None,body=[network.owner])
        assert message.destination==network.owner
        return await original_call(message)
    async def handle(path,method,args):
        if method=='call_get_settings' and path==OLD_PATH:
            network.calls.append((path,method))
            return {'connection':{'uuid':Variant('s',FOREIGN)}}
        if method=='call_get_secrets':
            network.calls.append((path,method))
            assert path==OWN_PATH and args==('802-11-wireless-security',)
            if network.secrets_denied:raise RuntimeError(PASSWORD.decode())
            return {'802-11-wireless-security':{'psk':deepcopy(network.settings['802-11-wireless-security']['psk'])}}
        if method in ('call_add_connection_unsaved','call_activate_connection'):
            records=list(network.directory.glob('*.json'))
            assert len(records)==1
            active=json.loads(records[0].read_text())['active']
            expected='create_pending' if method=='call_add_connection_unsaved' else 'activate_pending'
            assert active['stage']==expected
            assert active['uuid']==(args[0]['connection']['uuid'].value if method=='call_add_connection_unsaved' else network.uuid)
        return await original_handle(path,method,args)
    monkeypatch.setattr(network,'introspect',introspect)
    monkeypatch.setattr(network,'get_proxy_object',proxy)
    monkeypatch.setattr(network,'call',call)
    monkeypatch.setattr(network,'handle',handle)
    return network


def link(system):
    return wifi.MemoryWifiLink(CREDS,'wlanusb0',ownership_directory=system.directory,
        event_sink=lambda name,**details:system.events.append((name,details)))


def read_active(system):
    records=list(system.directory.glob('*.json'))
    return None if not records else json.loads(records[0].read_text())['active']


async def orphan(system):
    connection=link(system)
    await connection.__aenter__()
    assert read_active(system)['stage']=='active'
    connection._bus.disconnect()
    connection._journal.close()  # The kernel releases exactly this on SIGKILL.
    connection._bus=None
    return connection


def mutations(system):
    return [c for c in system.calls if len(c)>1 and c[1] in
        ('call_add_connection_unsaved','call_activate_connection','call_deactivate_connection','call_delete')]


def test_opt_in_constructor_and_empty_recovery_do_not_touch_nm(durable):
    connection=link(durable)
    assert not durable.directory.exists() and not durable.calls and not durable.commands
    result=asyncio.run(wifi.recover_memory_wifi('wlanusb0',durable.directory))
    assert result['cleanup_verified'] and not result['recovered']
    assert not durable.calls and not durable.commands
    assert connection.state=='new'


def test_journal_precedes_both_mutations_and_clean_exit_durably_clears(durable):
    async def run():
        async with link(durable) as connection:
            assert read_active(durable)['uuid']==connection.connection_uuid
            with pytest.raises(RuntimeError):
                with WifiOwnershipStore(durable.directory,'wlanusb0',durable.identity):pass
        assert connection.cleanup_verified
    asyncio.run(run())
    assert read_active(durable) is None and not durable.profile_exists and not durable.active
    assert SSID.decode() not in next(durable.directory.glob('*.json')).read_text()
    assert PASSWORD.decode() not in json.dumps(durable.events)


def test_orphan_active_exact_profile_is_recovered_then_new_connection_can_open(durable):
    async def run():
        previous=await orphan(durable)
        old_uuid=previous.connection_uuid
        durable.calls.clear()
        with pytest.raises(wifi.WifiLinkError,match='recovery_required'):
            await link(durable).__aenter__()
        assert not mutations(durable) and read_active(durable)['uuid']==old_uuid
        result=await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert result['recovered'] and result['cleanup_verified'] and result['profile_removed']
        assert mutations(durable)==[(wifi.MANAGER,'call_deactivate_connection'),(OWN_PATH,'call_delete')]
        assert read_active(durable) is None
        async with link(durable) as next_connection:
            assert next_connection.connection_uuid!=old_uuid
    asyncio.run(run())
    assert read_active(durable) is None


@pytest.mark.parametrize('change',['saved','psk','ssid','id','permissions','route','added','secrets','device','ifindex','machine','user'])
def test_modified_saved_foreign_or_unverifiable_profile_is_never_mutated(durable,change):
    async def run():
        await orphan(durable)
        if change=='saved':durable.unsaved=False
        elif change=='secrets':durable.secrets_denied=True
        elif change in ('device','ifindex','machine','user'):
            key={'device':'device_id_sha256','ifindex':'ifindex','machine':'machine_id_sha256','user':'uid'}[change]
            durable.identity[key]=99 if key in ('uid','ifindex') else 'f'*64
        elif change=='psk':durable.settings['802-11-wireless-security']['psk']=Variant('s','changed-password')
        elif change=='ssid':durable.settings['802-11-wireless']['ssid']=Variant('ay',b'changed')
        elif change=='id':durable.settings['connection']['id']=Variant('s','someone-else')
        elif change=='permissions':durable.settings['connection']['permissions']=Variant('as',[])
        elif change=='route':durable.settings['ipv4']['route-data'].value[0]['next-hop']=Variant('s','192.0.2.1')
        elif change=='added':durable.settings['connection']['zone']=Variant('s','trusted')
        durable.calls.clear()
        with pytest.raises(wifi.WifiLinkError):await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert not mutations(durable)
        assert read_active(durable) is not None and durable.profile_exists and durable.active
    asyncio.run(run())


def pending(system,stage,*,effect=False,active=False):
    connection=link(system)
    settings=connection._settings()
    with WifiOwnershipStore(system.directory,'wlanusb0',system.identity) as journal:
        journal.begin(connection.connection_uuid,wifi._profile_digest(settings),system.owner)
        if stage!='create_pending':journal.advance(connection.connection_uuid,'created')
        if stage in ('activate_pending','active'):journal.advance(connection.connection_uuid,'activate_pending')
        if stage=='active':journal.advance(connection.connection_uuid,'active')
    if effect:
        system.settings=deepcopy(settings);system.profile_exists=True;system.active=active
    return connection


@pytest.mark.parametrize('stage,effect,active,success',[
    ('create_pending',False,False,False),('create_pending',True,False,True),
    ('created',True,False,True),('activate_pending',True,False,False),
    ('activate_pending',True,True,True),('active',True,True,True),('created',False,False,True)])
def test_pending_outcomes_need_observable_exact_effect_or_known_settled_absence(durable,stage,effect,active,success):
    pending(durable,stage,effect=effect,active=active)
    if success:
        result=asyncio.run(wifi.recover_memory_wifi('wlanusb0',durable.directory))
        assert result['cleanup_verified'] and read_active(durable) is None
        assert not durable.profile_exists and not durable.active
    else:
        with pytest.raises(wifi.WifiLinkError,match='pending_outcome_unknown'):
            asyncio.run(wifi.recover_memory_wifi('wlanusb0',durable.directory))
        assert not mutations(durable) and read_active(durable)['stage']==stage


@pytest.mark.parametrize('restart',['daemon','boot'])
def test_proven_daemon_or_boot_restart_can_clear_absent_pending_request(durable,restart):
    pending(durable,'create_pending')
    if restart=='daemon':durable.owner=':1.77'
    else:durable.identity['boot_id']='00000000-0000-4000-8000-000000000004'
    result=asyncio.run(wifi.recover_memory_wifi('wlanusb0',durable.directory))
    assert result['cleanup_verified'] and read_active(durable) is None and not mutations(durable)


def test_matching_profile_from_another_daemon_is_not_deleted(durable):
    pending(durable,'active',effect=True,active=True)
    durable.owner=':1.77'
    with pytest.raises(wifi.WifiLinkError,match='identity_changed'):
        asyncio.run(wifi.recover_memory_wifi('wlanusb0',durable.directory))
    assert not mutations(durable) and read_active(durable) is not None


def test_saved_or_changed_profile_during_same_context_is_preserved(durable):
    async def run():
        connection=link(durable)
        await connection.__aenter__()
        durable.unsaved=False
        durable.calls.clear()
        with pytest.raises(wifi.WifiLinkError,match='cleanup_failed'):
            await connection.__aexit__(None,None,None)
        assert not connection.cleanup_verified and not mutations(durable)
    asyncio.run(run())
    assert durable.profile_exists and durable.active and read_active(durable) is not None


def test_default_route_on_dedicated_interface_is_refused_before_mutation(durable):
    async def run():
        await orphan(durable)
        durable.default_device='wlanusb0';durable.calls.clear()
        with pytest.raises(wifi.WifiLinkError,match='default_route'):
            await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert not mutations(durable) and read_active(durable) is not None
    asyncio.run(run())


def test_failed_delete_retains_journal_and_explicit_next_attempt_can_finish(durable):
    async def run():
        await orphan(durable)
        durable.fail_delete=True
        with pytest.raises(wifi.WifiLinkError):await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert read_active(durable)['stage']=='cleanup_pending' and durable.profile_exists
        assert not durable.active
        durable.fail_delete=False
        result=await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert result['cleanup_verified'] and read_active(durable) is None
    asyncio.run(run())


def test_cleanup_observes_async_deactivation_after_device_settles(durable,monkeypatch):
    async def run():
        await orphan(durable)
        original_handle,original_command=durable.handle,durable.command
        deactivating=False
        async def handle(path,method,args):
            nonlocal deactivating
            result=await original_handle(path,method,args)
            if method=='call_deactivate_connection':
                deactivating=True;durable.active=True
            return result
        def command(*args,**kwargs):
            nonlocal deactivating
            if deactivating and args[:3]==('nmcli','--get-values','GENERAL.TYPE,GENERAL.STATE'):
                durable.active=False;deactivating=False
            return original_command(*args,**kwargs)
        monkeypatch.setattr(durable,'handle',handle)
        monkeypatch.setattr(wifi,'_run',command)
        from neo2_control import wifi_link
        monkeypatch.setattr(wifi_link,'_run',command)
        result=await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert result['cleanup_verified'] and result['deactivated'] and read_active(durable) is None
    asyncio.run(run())


def test_write_ahead_failure_does_not_create_or_activate(durable,monkeypatch):
    def fail(*args,**kwargs):raise OSError('injected disk error')
    monkeypatch.setattr(WifiOwnershipStore,'begin',fail)
    with pytest.raises(wifi.WifiLinkError):asyncio.run(link(durable).__aenter__())
    assert not mutations(durable) and not durable.profile_exists


def test_clear_failure_does_not_report_cleanup_or_remove_record(durable,monkeypatch):
    def fail(*args,**kwargs):raise OSError('injected disk error')
    async def run():
        await orphan(durable)
        monkeypatch.setattr(WifiOwnershipStore,'clear',fail)
        with pytest.raises(wifi.WifiLinkError):await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert read_active(durable) is not None
        assert not durable.profile_exists and not durable.active
    asyncio.run(run())


@pytest.mark.parametrize('stage',['create','activate'])
def test_stop_after_write_ahead_but_before_send_has_local_proof_of_no_request(durable,monkeypatch,stage):
    stop=threading.Event()
    original=WifiOwnershipStore.begin if stage=='create' else WifiOwnershipStore.advance
    def persist_then_stop(self,*args,**kwargs):
        result=original(self,*args,**kwargs)
        if stage=='create' or args[1]=='activate_pending':stop.set()
        return result
    monkeypatch.setattr(WifiOwnershipStore,'begin' if stage=='create' else 'advance',persist_then_stop)
    connection=wifi.MemoryWifiLink(CREDS,'wlanusb0',ownership_directory=durable.directory,stop_event=stop)
    with pytest.raises(wifi.WifiLinkCancelled):asyncio.run(connection.__aenter__())
    assert not any(c[1]=='call_activate_connection' for c in mutations(durable))
    if stage=='create':assert not mutations(durable)
    assert connection.cleanup_verified and read_active(durable) is None
    assert not durable.profile_exists and not durable.active


@pytest.mark.parametrize('stage',['create','activate'])
def test_unsettled_mutation_retains_journal_until_explicit_recovery(durable,monkeypatch,stage):
    monkeypatch.setattr(wifi,'MUTATION_SETTLE_TIMEOUT',.01)
    async def run():
        if stage=='create':durable.pause_add=True
        else:durable.pause_activate=True
        connection=link(durable)
        task=asyncio.create_task(connection.__aenter__())
        await (durable.add_started if stage=='create' else durable.activation_started).wait()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):await task
        assert not connection.cleanup_verified and connection._mutation_unknown
        assert read_active(durable)['stage']==('create_pending' if stage=='create' else 'activate_pending')
        assert durable.profile_exists
        durable.pause_add=durable.pause_activate=False
        # A real process restart gets this same journal. Here the exact created
        # object (and active object for Activate) supplies positive effect proof.
        result=await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert result['cleanup_verified'] and read_active(durable) is None
    asyncio.run(run())


@pytest.mark.parametrize('kind',['another_active','multiple_devices','wrong_connection'])
def test_foreign_or_ambiguous_active_owner_on_interface_is_preserved(durable,monkeypatch,kind):
    async def run():
        await orphan(durable)
        original=durable.handle
        async def changed(path,method,args):
            if kind=='another_active' and path==OLD_ACTIVE and method=='get_devices':return [DEVICE]
            if kind=='multiple_devices' and path==ACTIVE and method=='get_devices':return [DEVICE,ROOT+'/Devices/10']
            if kind=='wrong_connection' and path==ACTIVE and method=='get_connection':return OLD_PATH
            return await original(path,method,args)
        monkeypatch.setattr(durable,'handle',changed)
        durable.calls.clear()
        with pytest.raises(wifi.WifiLinkError):await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert not mutations(durable) and read_active(durable) is not None
        assert durable.active and durable.profile_exists
    asyncio.run(run())


def test_second_identity_check_prevents_deleting_profile_modified_after_deactivation(durable,monkeypatch):
    async def run():
        await orphan(durable)
        original=durable.handle
        async def changed(path,method,args):
            result=await original(path,method,args)
            if method=='call_deactivate_connection':durable.settings['connection']['autoconnect']=Variant('b',True)
            return result
        monkeypatch.setattr(durable,'handle',changed)
        durable.calls.clear()
        with pytest.raises(wifi.WifiLinkError):await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert mutations(durable)==[(wifi.MANAGER,'call_deactivate_connection')]
        assert not durable.active and durable.profile_exists and read_active(durable) is not None
    asyncio.run(run())


def test_pinned_daemon_replacement_fails_without_mutating_replacement(durable,monkeypatch):
    async def run():
        await orphan(durable)
        original_introspect,original_proxy=durable.introspect,durable.get_proxy_object
        checked=[]
        async def changed(service,path):
            if path==wifi.SETTINGS and not checked:
                checked.append(service)
                durable.owner=':1.99'
            # No call ever falls back to the well-known NM destination. The
            # old unique destination no longer exists; a new NM may reuse paths.
            assert service!='org.freedesktop.NetworkManager'
            return await original_introspect(service,path)
        def proxy(service,path,tree):return original_proxy(service,path,tree)
        monkeypatch.setattr(durable,'introspect',changed)
        monkeypatch.setattr(durable,'get_proxy_object',proxy)
        durable.calls.clear()
        with pytest.raises(wifi.WifiLinkError):await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert checked and not mutations(durable) and read_active(durable) is not None
    asyncio.run(run())


def test_known_nm_normalization_and_timestamp_do_not_mask_changed_required_property(durable):
    async def run():
        await orphan(durable)
        durable.settings['connection']['timestamp']=Variant('t',123456)
        durable.settings['ipv4']['addresses']=Variant('aau',[])
        durable.settings['proxy']={'method':Variant('i',0)}
        result=await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert result['cleanup_verified'] and read_active(durable) is None
    asyncio.run(run())


def test_real_nm_empty_secret_groups_and_normalized_settings_can_be_recovered(durable,monkeypatch):
    """Match the real NM response observed before any yaw-flight motor command."""
    async def run():
        await orphan(durable)
        durable.settings['802-11-wireless']['security']=Variant('s','802-11-wireless-security')
        durable.settings['802-11-wireless']['seen-bssids']=Variant('as',[
            ':'.join(f'{v:02X}' for v in CREDS.bssid)])
        durable.settings['ipv4']['routes']=Variant('aau',deepcopy(wifi._LEGACY_DRONE_ROUTE))
        durable.settings['ipv6']['route-data']=Variant('aa{sv}',[])
        durable.settings['proxy']={}
        original=durable.handle
        async def normalized(path,method,args):
            result=await original(path,method,args)
            if method=='call_get_settings' and path==OWN_PATH:
                assert 'psk' not in result['802-11-wireless-security']
            if method=='call_get_secrets':
                result={group:result.get(group,{}) for group in durable.settings}
            return result
        monkeypatch.setattr(durable,'handle',normalized)
        result=await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert result['cleanup_verified'] and read_active(durable) is None
        assert not durable.profile_exists and not durable.active
    asyncio.run(run())


@pytest.mark.parametrize('change',['foreign_secret_group','nonempty_secret_group','missing_psk',
    'extra_secret','security_pointer','ipv6_route','legacy_destination','legacy_prefix',
    'legacy_gateway','legacy_metric','foreign_seen_bssid','multiple_seen_bssids'])
def test_normalization_does_not_authorize_unknown_secrets_or_changed_settings(durable,monkeypatch,change):
    async def run():
        await orphan(durable)
        if change=='security_pointer':
            durable.settings['802-11-wireless']['security']=Variant('s','other-security')
        if change=='ipv6_route':
            durable.settings['ipv6']['route-data']=Variant('aa{sv}',[{'dest':Variant('s','::')}])
        if change.startswith('legacy_'):
            route=deepcopy(wifi._LEGACY_DRONE_ROUTE)
            route[0][{'legacy_destination':0,'legacy_prefix':1,'legacy_gateway':2,'legacy_metric':3}[change]]+=1
            durable.settings['ipv4']['routes']=Variant('aau',route)
        if change in ('foreign_seen_bssid','multiple_seen_bssids'):
            seen=['02:00:00:00:00:03']
            if change=='multiple_seen_bssids':seen.insert(0,':'.join(f'{v:02X}' for v in CREDS.bssid))
            durable.settings['802-11-wireless']['seen-bssids']=Variant('as',seen)
        original=durable.handle
        async def changed(path,method,args):
            result=await original(path,method,args)
            if method=='call_get_secrets':
                if change=='foreign_secret_group':result['unknown']={}
                elif change=='nonempty_secret_group':result['ipv4']={'method':Variant('s','auto')}
                elif change=='missing_psk':result['802-11-wireless-security']={}
                elif change=='extra_secret':result['802-11-wireless-security']['wep-key0']=Variant('s','x')
            return result
        monkeypatch.setattr(durable,'handle',changed)
        durable.calls.clear()
        with pytest.raises(wifi.WifiLinkError):
            await wifi.recover_memory_wifi('wlanusb0',durable.directory)
        assert not mutations(durable) and read_active(durable) is not None
        assert durable.profile_exists and durable.active
    asyncio.run(run())


def test_crash_window_after_durable_intent_before_add_stays_uncertain_on_reopen(durable,monkeypatch):
    original=WifiOwnershipStore.begin
    def persist_then_fail(self,*args,**kwargs):
        original(self,*args,**kwargs)
        raise OSError('injected failure after durable intent')
    monkeypatch.setattr(WifiOwnershipStore,'begin',persist_then_fail)
    with pytest.raises(wifi.WifiLinkError):asyncio.run(link(durable).__aenter__())
    assert not mutations(durable) and read_active(durable)['stage']=='create_pending'
    with pytest.raises(wifi.WifiLinkError,match='pending_outcome_unknown'):
        asyncio.run(wifi.recover_memory_wifi('wlanusb0',durable.directory))
    assert not mutations(durable) and read_active(durable) is not None


def test_external_journal_rewrite_cannot_authorize_add(durable,monkeypatch):
    original=WifiOwnershipStore.begin
    def persist_then_rewrite(self,*args,**kwargs):
        result=original(self,*args,**kwargs)
        path=self.directory/self.filename
        value=json.loads(path.read_text());value['active']['profile_sha256']='f'*64
        path.write_text(json.dumps(value))
        return result
    monkeypatch.setattr(WifiOwnershipStore,'begin',persist_then_rewrite)
    with pytest.raises(wifi.WifiLinkError):asyncio.run(link(durable).__aenter__())
    assert not mutations(durable) and read_active(durable) is not None


def test_unresponsive_inventory_is_bounded_and_retains_ownership(durable,monkeypatch):
    async def run():
        await orphan(durable)
        original=durable.handle
        async def blocked(path,method,args):
            if path==OLD_PATH and method=='call_get_settings':await asyncio.Event().wait()
            return await original(path,method,args)
        monkeypatch.setattr(durable,'handle',blocked)
        monkeypatch.setattr(wifi,'DURABLE_CLEANUP_TIMEOUT',.02)
        durable.calls.clear()
        with pytest.raises(wifi.WifiLinkError,match='cleanup_timeout'):
            await asyncio.wait_for(wifi.recover_memory_wifi('wlanusb0',durable.directory),1)
        assert not mutations(durable) and read_active(durable) is not None
        # The failed recovery also released the lifetime lock for inspection.
        with WifiOwnershipStore(durable.directory,'wlanusb0',durable.identity) as opened:
            assert opened.active is not None
    asyncio.run(run())
