"""One owned, unsaved NetworkManager profile, with optional durable ownership.

Construction is inert. D-Bus is imported and connected only on async entry.
Every operation which changes NM state addresses an owned object, never a
device-wide disconnect or an existing profile. Public diagnostics are redacted.
"""
from __future__ import annotations

import asyncio
from copy import deepcopy
import hashlib
import json
import os
import pwd
import re
import sys
import time
import uuid

from .ble_bootstrap import WifiCredentials
from .wifi_link import (DRONE_HOST, WifiLinkCancelled, WifiLinkError, _routes, _run,
                        _require_idle_wifi_interface, require_dedicated_drone_route)
from .wifi_ownership import WifiOwnershipStore, daemon_owner, ownership_identity


NM = 'org.freedesktop.NetworkManager'
MANAGER = '/org/freedesktop/NetworkManager'
SETTINGS = MANAGER+'/Settings'
READ_TIMEOUT = 10
MUTATION_TIMEOUT = 10
MUTATION_SETTLE_TIMEOUT = 10
ACTIVATION_TIMEOUT = 22
DEACTIVATION_TIMEOUT = 3
DURABLE_CLEANUP_TIMEOUT = 30
DBUS = 'org.freedesktop.DBus'

# NM may supply these empty defaults when serializing a normalized connection.
# All other additions are refused. Timestamp is mutable NM bookkeeping, never
# an authority or ownership identifier. The PSK remains part of the digest.
_NORMALIZED_DEFAULTS = {
    'connection': {'read-only': ('b', False)},
    '802-11-wireless': {'security': ('s', '802-11-wireless-security')},
    'ipv4': {'addresses': ('aau', []), 'routes': ('aau', []), 'dns': ('au', []),
        'dns-search': ('as', []), 'dns-options': ('as', []), 'dns-data': ('as', []),
        'address-data': ('aa{sv}', []), 'routing-rules': ('aa{sv}', []), 'gateway': ('s', '')},
    'ipv6': {'addresses': ('a(ayuay)', []), 'routes': ('a(ayuayu)', []), 'dns': ('aay', []),
        'dns-search': ('as', []), 'dns-options': ('as', []), 'dns-data': ('as', []),
        'address-data': ('aa{sv}', []), 'route-data': ('aa{sv}', []),
        'routing-rules': ('aa{sv}', []), 'gateway': ('s', '')},
    'proxy': {'method': ('i', 0), 'browser-only': ('b', False),
        'pac-url': ('s', ''), 'pac-script': ('s', '')},
}
_PROFILE_KEYS = {
    'connection': {'id', 'uuid', 'type', 'interface-name', 'autoconnect', 'permissions'},
    '802-11-wireless': {'mode', 'ssid', 'bssid'},
    '802-11-wireless-security': {'key-mgmt', 'psk'},
    'ipv4': {'method', 'never-default', 'ignore-auto-dns', 'ignore-auto-routes',
        'dhcp-send-hostname', 'may-fail', 'dhcp-timeout', 'route-data'},
    'ipv6': {'method', 'never-default'},
}
_LEGACY_DRONE_ROUTE = [[int.from_bytes(bytes(map(int, DRONE_HOST.split('.'))), sys.byteorder), 32, 0, 0]]


def _profile_digest(settings):
    """Hash the exact requested properties, including secrets, without storing them."""
    def encode(value, depth=0):
        if depth > 8:raise ValueError
        if hasattr(value, 'signature') and hasattr(value, 'value'):
            raw=value.value
            if value.signature=='ay':
                if not isinstance(raw,(bytes,bytearray,list,tuple)) or len(raw)>1024:raise ValueError
                raw=bytes(raw)
            return [str(value.signature), encode(raw, depth+1)]
        if isinstance(value, bytes):return {'bytes':value.hex()}
        if isinstance(value, dict):
            if len(value)>64 or any(not isinstance(k,str) or len(k)>128 for k in value):raise ValueError
            return {k:encode(v,depth+1) for k,v in value.items()}
        if isinstance(value, (list,tuple)):
            if len(value)>64:raise ValueError
            return [encode(v,depth+1) for v in value]
        if type(value) in (str,int,bool) and (not isinstance(value,str) or len(value)<=1024):return value
        raise ValueError
    try:
        values=deepcopy(settings)
        if not isinstance(values,dict):raise ValueError
        for group,props in list(values.items()):
            if not isinstance(props,dict):raise ValueError
            for key,item in list(props.items()):
                if key in _PROFILE_KEYS.get(group,set()):continue
                if group=='connection' and key=='timestamp':
                    if item.signature!='t' or type(item.value) is not int or not 0<=item.value<2**64:raise ValueError
                elif group=='802-11-wireless' and key=='seen-bssids':
                    bssid=props['bssid']
                    octets=bytes(bssid.value)
                    if (bssid.signature!='ay' or len(octets)!=6 or item.signature!='as'
                            or item.value not in ([], [':'.join(f'{v:02X}' for v in octets)])):
                        raise ValueError
                elif (group=='ipv4' and key=='routes' and item.signature=='aau'
                        and item.value==_LEGACY_DRONE_ROUTE):
                    # NM serializes the requested host route in both modern
                    # route-data and legacy native-byte-order aau form.
                    pass
                else:
                    default=_NORMALIZED_DEFAULTS.get(group,{}).get(key)
                    if default is None or (item.signature,item.value)!=default:raise ValueError
                del props[key]
            if not props and group not in _PROFILE_KEYS:del values[group]
        if set(values)!=set(_PROFILE_KEYS):raise ValueError
        if any(set(values[k])!=required for k,required in _PROFILE_KEYS.items()):raise ValueError
        data=json.dumps(encode(values),sort_keys=True,separators=(',',':'),allow_nan=False).encode()
        if len(data)>16384:raise ValueError
        return hashlib.sha256(data).hexdigest()
    except (TypeError,ValueError,AttributeError,RecursionError):
        raise WifiLinkError('wifi_owned_profile_not_exact') from None


def _new_bus():
    try:
        from dbus_fast import BusType
        from dbus_fast.aio import MessageBus
    except ImportError:
        raise WifiLinkError('networkmanager_dbus_unavailable') from None
    return MessageBus(bus_type=BusType.SYSTEM, auth_timeout=READ_TIMEOUT)


def _variant(signature, value):
    from dbus_fast import Variant
    return Variant(signature, value)


def _message(**values):
    from dbus_fast import Message
    return Message(**values)


def _value(value):
    return value.value


def _object_path(path, group):
    return isinstance(path, str) and re.fullmatch(re.escape(MANAGER+'/'+group)+r'/[0-9]+', path)


class MemoryWifiLink:
    """Async context owning a temporary Wi-Fi profile and a direct drone route.

    event_sink receives ``(name, **redacted_metadata)``. stop_event can be a
    threading.Event or asyncio.Event; it cancels setup, not the caller's body.
    Entry is single-use; successful cleanup is idempotent. D-Bus mutations are
    shielded until their returned ownership handles have been collected.
    ownership_directory enables a private write-ahead journal/lifetime lock;
    an existing unresolved journal requires explicit recover_memory_wifi first.
    """

    def __init__(self, credentials: WifiCredentials, interface: str, *, stop_event=None,
                 event_sink=None, ownership_directory=None):
        if not isinstance(credentials, WifiCredentials):
            raise ValueError('Wi-Fi link requires a credential record')
        if not isinstance(interface, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', interface):
            raise ValueError('invalid Linux network interface name')
        password = credentials.passphrase
        if not ((8 <= len(password) <= 63 and all(32 <= c <= 126 for c in password))
                or (len(password) == 64 and re.fullmatch(rb'[0-9a-fA-F]{64}', password))):
            raise ValueError('unsupported Wi-Fi PSK format')
        if stop_event is not None and not callable(getattr(stop_event, 'is_set', None)):
            raise ValueError('stop_event must expose is_set()')
        if event_sink is not None and not callable(event_sink):
            raise ValueError('event_sink must be callable')
        self._initialize(credentials, interface, stop_event, event_sink, ownership_directory)

    def _initialize(self, credentials, interface, stop_event, event_sink, ownership_directory):
        self._credentials, self.interface = credentials, interface
        self.stop_event, self.event_sink = stop_event, event_sink
        self.connection_uuid = str(uuid.uuid4())
        self.connection_id = 'neo2-control-'+self.connection_uuid
        self.state, self.reason = 'new', None
        self.connected = self.cleanup_verified = self.profile_removed = self.deactivated = False
        self.default_routes_preserved = self.existing_profiles_preserved = None
        self.existing_active_profiles_preserved = None
        self.interface_idle_after_cleanup = None
        self.route_verified = self.profile_verified = False
        self.cleanup_errors = []
        self._bus = self._owned = self._active = self._device = self._pending = None
        self._cleanup_task = None
        self._baseline = self._profiles = self._active_profiles = None
        self._paths = self._active_paths = set()
        self._proxies = {}
        self._creation_attempted = self._ownership_conflict = False
        self._activation_attempted = False
        self._mutation_unknown = False
        self.ownership_directory = ownership_directory
        self._journal = None
        self._journal_active = False
        self._destination = NM
        self._recovering = False
        self._cleanup_deadline = None

    def snapshot(self):
        return {'state': self.state, 'reason': self.reason, 'interface': self.interface,
                'connection_uuid': self.connection_uuid,
                'network': self._credentials.metadata() if self._credentials is not None else None,
                'profile_method': 'NetworkManager.AddConnectionUnsaved',
                'autoconnect': False, 'default_route': False, 'use_drone_dns': False,
                'ipv6': False, 'drone_route': DRONE_HOST+'/32',
                'connected': self.connected, 'profile_verified': self.profile_verified,
                'route_verified': self.route_verified, 'profile_removed': self.profile_removed,
                'deactivated': self.deactivated, 'cleanup_verified': self.cleanup_verified,
                'default_routes_preserved': self.default_routes_preserved,
                'existing_profiles_preserved': self.existing_profiles_preserved,
                'existing_active_profiles_preserved': self.existing_active_profiles_preserved,
                'interface_idle_after_cleanup': self.interface_idle_after_cleanup,
                'mutation_outcome_unknown': self._mutation_unknown,
                'durable_ownership': self.ownership_directory is not None,
                'cleanup_errors': list(self.cleanup_errors)}

    def _event(self, name):
        if self.event_sink is not None:
            try:
                self.event_sink(name, **self.snapshot())
            except Exception:
                raise WifiLinkError('wifi_event_sink_failed') from None

    def _check_stop(self):
        if self.stop_event is not None and self.stop_event.is_set():
            raise WifiLinkCancelled('wifi_connection_cancelled')

    async def _read(self, operation, *, cleanup=False, timeout=READ_TIMEOUT):
        task = asyncio.ensure_future(operation)
        deadline = time.monotonic()+timeout
        if cleanup and self._cleanup_deadline is not None:
            deadline=min(deadline,self._cleanup_deadline)
        try:
            while not task.done():
                if not cleanup:
                    self._check_stop()
                if time.monotonic() >= deadline:
                    reason=('wifi_ownership_cleanup_timeout' if cleanup
                        and self._cleanup_deadline is not None and time.monotonic()>=self._cleanup_deadline
                        else 'networkmanager_read_timeout')
                    raise WifiLinkError(reason)
                await asyncio.wait((task,), timeout=.05)
            result = task.result()
            if not cleanup:
                self._check_stop()
            return result
        except BaseException:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            raise

    async def _proxy(self, path, interface):
        key = path, interface
        if key not in self._proxies:
            tree = await self._bus.introspect(self._destination, path)
            self._proxies[key] = self._bus.get_proxy_object(self._destination, path, tree).get_interface(interface)
        return self._proxies[key]

    async def _invoke(self, path, interface, member, snake, args=(), signature='', *, property=False):
        try:
            proxy = await self._proxy(path, interface)
        except Exception:
            # Proxy/introspection construction may fail after AddConnection.
            # Fall back before invoking the method, never retry a failed
            # mutation whose remote side effects are uncertain.
            values = dict(destination=self._destination, path=path, interface=interface,
                          member=member, signature=signature, body=list(args))
            if property:
                values.update(interface='org.freedesktop.DBus.Properties', member='Get',
                              signature='ss', body=[interface, member])
            reply = await self._bus.call(_message(**values))
            if reply is None or getattr(reply, 'error_name', None):
                raise WifiLinkError('networkmanager_dbus_call_failed')
            result = reply.body[0] if len(reply.body) == 1 else None
            return _value(result) if property else result
        method = getattr(proxy, ('get_' if property else 'call_')+snake)
        return await method(*args)

    async def _get(self, path, interface, member, snake):
        return await self._invoke(path, interface, member, snake, property=True)

    async def _pin_daemon(self):
        reply=await self._read(self._bus.call(_message(destination=DBUS,
            path='/org/freedesktop/DBus', interface=DBUS, member='GetNameOwner',
            signature='s', body=[NM])))
        if (reply is None or getattr(reply,'error_name',None) or len(reply.body)!=1
                or not daemon_owner(reply.body[0])):
            raise WifiLinkError('networkmanager_owner_unavailable')
        self._destination=reply.body[0]
        self._proxies.clear()

    async def _verified_device(self, *, cleanup=False):
        path=await self._read(self._invoke(MANAGER,NM,'GetDeviceByIpIface',
            'get_device_by_ip_iface',(self.interface,),'s'),cleanup=cleanup)
        if not _object_path(path,'Devices'):
            raise WifiLinkError('invalid_networkmanager_device')
        if (await self._read(self._get(path,NM+'.Device','Interface','interface'),cleanup=cleanup)!=self.interface
                or await self._read(self._get(path,NM+'.Device','DeviceType','device_type'),cleanup=cleanup)!=2):
            raise WifiLinkError('networkmanager_device_does_not_match_interface')
        if self._device is not None and self._device!=path:
            raise WifiLinkError('wifi_ownership_device_changed')
        self._device=path
        return path

    def _check_identity(self):
        if self._journal is None or ownership_identity(self.interface)!=self._journal.identity:
            raise WifiLinkError('wifi_ownership_identity_changed')

    async def _verify_exact_profile(self, *, cleanup=False):
        self._check_identity()
        await self._verified_device(cleanup=cleanup)
        stored=await self._read(self._stored(),cleanup=cleanup)
        if not self._owns_settings(stored):raise WifiLinkError('owned_profile_identity_changed')
        permissions=stored.get('connection',{}).get('permissions')
        if (getattr(permissions,'signature',None)!='as'
                or permissions.value!=[f'user:{pwd.getpwuid(os.getuid()).pw_name}:']):
            raise WifiLinkError('wifi_owned_profile_user_changed')
        if await self._read(self._get(self._owned,NM+'.Settings.Connection','Unsaved','unsaved'),cleanup=cleanup) is not True:
            raise WifiLinkError('profile_is_not_unsaved')
        secrets=await self._read(self._invoke(self._owned,NM+'.Settings.Connection',
            'GetSecrets','get_secrets',('802-11-wireless-security',),'s'),cleanup=cleanup)
        # NM can return every connection setting here, with empty dictionaries
        # for settings that have no secrets. Permit only these known empty
        # groups; the sole secret must still be the PSK hashed in our journal.
        if (not isinstance(secrets,dict)
                or set(secrets)-set(_PROFILE_KEYS)-{'proxy'}
                or any(not isinstance(values,dict) or (group!='802-11-wireless-security' and values)
                    for group,values in secrets.items())
                or '802-11-wireless-security' not in secrets
                or not isinstance(secrets['802-11-wireless-security'],dict)
                or set(secrets['802-11-wireless-security'])!={'psk'}):
            raise WifiLinkError('wifi_owned_profile_secrets_unavailable')
        stored=deepcopy(stored)
        stored['802-11-wireless-security']['psk']=secrets['802-11-wireless-security']['psk']
        active=self._journal.active
        if (active is None or active['uuid']!=self.connection_uuid
                or _profile_digest(stored)!=active['profile_sha256']):
            raise WifiLinkError('wifi_owned_profile_not_exact')
        self.profile_verified=True

    async def _inventory(self):
        """Read a bounded inventory from the pinned daemon, not a replacement."""
        paths=await self._read(self._invoke(SETTINGS,NM+'.Settings',
            'ListConnections','list_connections'),cleanup=True)
        if (not isinstance(paths,list) or len(paths)>512 or len(set(paths))!=len(paths)
                or any(not _object_path(p,'Settings') for p in paths)):
            raise WifiLinkError('invalid_networkmanager_profile_list')
        profiles={}
        for path in paths:
            settings=await self._read(self._invoke(path,NM+'.Settings.Connection',
                'GetSettings','get_settings'),cleanup=True)
            try:
                connection_uuid=settings['connection']['uuid'].value
                if str(uuid.UUID(connection_uuid))!=connection_uuid or connection_uuid in profiles:raise ValueError
            except (KeyError,AttributeError,TypeError,ValueError):
                raise WifiLinkError('invalid_networkmanager_profile_list') from None
            profiles[connection_uuid]=path
        paths=await self._read(self._get(MANAGER,NM,'ActiveConnections','active_connections'),cleanup=True)
        if (not isinstance(paths,list) or len(paths)>256 or len(set(paths))!=len(paths)
                or any(not _object_path(p,'ActiveConnection') for p in paths)):
            raise WifiLinkError('invalid_networkmanager_active_list')
        active={}
        for path in paths:
            interface=NM+'.Connection.Active'
            item=(await self._read(self._get(path,interface,'Uuid','uuid'),cleanup=True),
                await self._read(self._get(path,interface,'Connection','connection'),cleanup=True),
                await self._read(self._get(path,interface,'Devices','devices'),cleanup=True))
            try:
                if (str(uuid.UUID(item[0]))!=item[0]
                        or (item[1]!='/' and not _object_path(item[1],'Settings'))
                        or not isinstance(item[2],list) or len(item[2])>64
                        or len(set(item[2]))!=len(item[2])
                        or any(not _object_path(p,'Devices') for p in item[2])):raise ValueError
            except (ValueError,TypeError,AttributeError):
                raise WifiLinkError('invalid_networkmanager_active_list') from None
            active[path]=item
        return profiles,active

    @staticmethod
    def _dedicated_default_guard(routes,interface):
        for _,route in routes:
            if interface in [route.get('dev')]+[hop.get('dev') for hop in route.get('nexthops',[])]:
                raise WifiLinkError('interface_carries_default_route')

    async def _durable_cleanup(self):
        """Clean only a durably identified unsaved object; uncertainty survives."""
        self._cleanup_deadline=time.monotonic()+DURABLE_CLEANUP_TIMEOUT
        try:
            if self._pending is not None:
                try:
                    await asyncio.wait_for(asyncio.shield(self._pending),MUTATION_SETTLE_TIMEOUT)
                except TimeoutError:
                    self._mutation_unknown=True
                    self._pending.cancel()
                    await asyncio.gather(self._pending,return_exceptions=True)
                    raise WifiLinkError('networkmanager_mutation_outcome_unknown') from None
                except Exception:
                    pass  # An observable exact effect may still establish ownership below.
                finally:self._pending=None
            record=self._journal.active
            if not self._journal_active:
                if record is not None:raise WifiLinkError('wifi_ownership_recovery_required')
                self.cleanup_verified=True
                return
            if record is None or record['uuid']!=self.connection_uuid:
                raise WifiLinkError('wifi_ownership_record_changed')
            identity=self._journal.identity
            if any(record['identity'][k]!=identity[k] for k in ('uid','machine_id_sha256')):
                raise WifiLinkError('wifi_ownership_identity_changed')
            self._check_identity()
            if self._bus is None:raise WifiLinkError('networkmanager_owner_unavailable')
            await self._verified_device(cleanup=True)
            routes=await self._read(asyncio.to_thread(_routes),cleanup=True)
            self._dedicated_default_guard(routes,self.interface)
            if self._baseline is not None and routes!=self._baseline:
                raise WifiLinkError('default_routes_changed_before_cleanup')
            profiles,active=await self._inventory()
            self._owned=profiles.get(self.connection_uuid)
            owned_active=[]
            for path,(connection_uuid,connection,devices) in active.items():
                if connection_uuid==self.connection_uuid:
                    if (self._owned is None or connection!=self._owned or devices!=[self._device]):
                        raise WifiLinkError('wifi_owned_active_identity_changed')
                    owned_active.append(path)
                elif self._device in devices:
                    raise WifiLinkError('wifi_interface_has_foreign_active_profile')
            if len(owned_active)>1:raise WifiLinkError('wifi_owned_active_identity_changed')
            pending=record['stage'] in ('create_pending','activate_pending')
            restarted=(record['identity']['boot_id']!=identity['boot_id']
                or record['daemon_owner']!=self._destination)
            if self._owned is None:
                locally_unsent=(not self._recovering and record['stage']=='create_pending'
                    and not self._creation_attempted)
                if pending and not restarted and not locally_unsent:
                    raise WifiLinkError('wifi_ownership_pending_outcome_unknown')
            else:
                if record['identity']!=identity or record['daemon_owner']!=self._destination:
                    raise WifiLinkError('wifi_ownership_identity_changed')
                await self._verify_exact_profile(cleanup=True)
                locally_unsent=(not self._recovering and not self._activation_attempted)
                if record['stage']=='activate_pending' and not owned_active and not locally_unsent:
                    raise WifiLinkError('wifi_ownership_pending_outcome_unknown')
                self._journal.advance(self.connection_uuid,'cleanup_pending')
                for path in owned_active:
                    # Revalidate both the entire profile and active object just
                    # before each precise mutation. Never disconnect a device.
                    await self._verify_exact_profile(cleanup=True)
                    if not await self._read(self._active_is_owned(path),cleanup=True):
                        raise WifiLinkError('wifi_owned_active_identity_changed')
                    await self._read(self._invoke(MANAGER,NM,'DeactivateConnection',
                        'deactivate_connection',(path,),'o'),cleanup=True)
                await self._verify_exact_profile(cleanup=True)
                await self._read(self._invoke(self._owned,NM+'.Settings.Connection',
                    'Delete','delete'),cleanup=True)
            # DeactivateConnection acknowledges a request before NM removes
            # its active object. Inspect the final inventory after the device
            # has reached disconnected, rather than freezing an interim view.
            deadline=min(time.monotonic()+DEACTIVATION_TIMEOUT,self._cleanup_deadline)
            self.interface_idle_after_cleanup=False
            while time.monotonic()<deadline:
                try:
                    await self._read(asyncio.to_thread(_require_idle_wifi_interface,self.interface),
                        cleanup=True,timeout=max(.01,deadline-time.monotonic()))
                    self.interface_idle_after_cleanup=True
                    break
                except Exception:
                    if time.monotonic()<deadline:await asyncio.sleep(.1)
            remaining,remaining_active=await self._inventory()
            foreign={u:p for u,p in profiles.items() if u!=self.connection_uuid}
            foreign_active={p:v for p,v in active.items() if p not in owned_active}
            self.profile_removed=self.connection_uuid not in remaining
            self.deactivated=all(v[0]!=self.connection_uuid for v in remaining_active.values())
            self.existing_profiles_preserved=all(remaining.get(u)==p for u,p in foreign.items())
            self.existing_active_profiles_preserved=all(remaining_active.get(p)==v for p,v in foreign_active.items())
            if self._profiles is not None:
                self.existing_profiles_preserved &= self._profiles<=set(remaining)
                self.existing_active_profiles_preserved &= self._active_profiles<={v[0] for v in remaining_active.values()}
                self.existing_profiles_preserved &= self._paths<=set(remaining.values())
                self.existing_active_profiles_preserved &= self._active_paths<=set(remaining_active)
            self.default_routes_preserved=routes==await self._read(asyncio.to_thread(_routes),cleanup=True)
            if not all((self.profile_removed,self.deactivated,self.existing_profiles_preserved,
                    self.existing_active_profiles_preserved,self.default_routes_preserved,
                    self.interface_idle_after_cleanup)):
                raise WifiLinkError('wifi_cleanup_verification_failed')
            self._check_identity()
            self._journal.clear(self.connection_uuid)
            self.cleanup_verified=True
        except Exception as error:
            self.cleanup_verified=False
            reason=str(error) if isinstance(error,WifiLinkError) else 'wifi_ownership_cleanup_not_verified'
            self.reason=reason
            self.cleanup_errors.append(reason)
        finally:
            self._cleanup_deadline=None
            self.connected=False
            if self._bus is not None:
                try:self._bus.disconnect()
                except Exception:
                    self.cleanup_verified=False
                    self.cleanup_errors.append('networkmanager_bus_disconnect_failed')
                self._bus=None
                self._proxies.clear()
            if self._journal is not None:self._journal.close()
            self.state='closed' if self.cleanup_verified else 'cleanup_failed'
            try:self._event('wifi_memory_removed' if self.cleanup_verified else 'wifi_memory_cleanup_failed')
            except WifiLinkError:self.cleanup_errors.append('wifi_event_sink_failed')

    async def _mutate(self, operation):
        try:
            self._check_stop()
        except BaseException:
            operation.close()
            raise
        self._pending = asyncio.create_task(operation)
        try:
            result = await asyncio.wait_for(asyncio.shield(self._pending), MUTATION_TIMEOUT)
        except TimeoutError:
            raise WifiLinkError('networkmanager_mutation_timeout') from None
        finally:
            if self._pending.done():
                self._pending = None
        self._check_stop()
        return result

    async def _command(self, *args, cleanup=False):
        return await self._read(asyncio.to_thread(_run, *args,
            cancel_event=None if cleanup else self.stop_event,
            reason='cannot_inspect_wifi_state'), cleanup=cleanup)

    async def _uuids(self, *, active=False, cleanup=False):
        args = ('nmcli', '--get-values', 'UUID', 'connection', 'show')
        result = await self._command(*args, *(('--active',) if active else ()), cleanup=cleanup)
        try:
            return {str(uuid.UUID(line.strip())) for line in result.stdout.splitlines() if line.strip()}
        except (ValueError, AttributeError):
            raise WifiLinkError('invalid_networkmanager_profile_list') from None

    def _settings(self):
        user = pwd.getpwuid(os.getuid()).pw_name
        v, credentials = _variant, self._credentials
        return {
            'connection': {'id': v('s', self.connection_id), 'uuid': v('s', self.connection_uuid),
                'type': v('s', '802-11-wireless'), 'interface-name': v('s', self.interface),
                'autoconnect': v('b', False), 'permissions': v('as', [f'user:{user}:'])},
            '802-11-wireless': {'mode': v('s', 'infrastructure'),
                'ssid': v('ay', credentials.ssid), 'bssid': v('ay', credentials.bssid)},
            '802-11-wireless-security': {'key-mgmt': v('s', 'wpa-psk'),
                'psk': v('s', credentials.passphrase.decode('ascii'))},
            'ipv4': {'method': v('s', 'auto'), 'never-default': v('b', True),
                'ignore-auto-dns': v('b', True), 'ignore-auto-routes': v('b', True),
                'dhcp-send-hostname': v('b', False), 'may-fail': v('b', False),
                'dhcp-timeout': v('i', 15), 'route-data': v('aa{sv}', [
                    {'dest': v('s', DRONE_HOST), 'prefix': v('u', 32)}])},
            'ipv6': {'method': v('s', 'disabled'), 'never-default': v('b', True)},
        }

    async def _stored(self):
        return await self._invoke(self._owned, NM+'.Settings.Connection', 'GetSettings', 'get_settings')

    def _owns_settings(self, stored):
        try:
            group = stored['connection']
            return all(_value(group[key]) == expected for key, expected in (
                ('uuid', self.connection_uuid), ('id', self.connection_id),
                ('interface-name', self.interface)))
        except (KeyError, TypeError, AttributeError):
            return False

    async def _verify_settings(self, expected):
        stored = await self._read(self._stored())
        if not self._owns_settings(stored):
            self._ownership_conflict = True
            raise WifiLinkError('owned_profile_identity_changed')
        try:
            for group, values in expected.items():
                for key, value in values.items():
                    if key not in ('psk', 'route-data'):
                        actual = stored[group][key]
                        if actual.signature != value.signature or _value(actual) != _value(value):
                            raise ValueError
            routes = _value(stored['ipv4']['route-data'])
            if (len(routes) != 1 or _value(routes[0]['dest']) != DRONE_HOST
                    or _value(routes[0]['prefix']) != 32
                    or ('next-hop' in routes[0] and _value(routes[0]['next-hop']))):
                raise ValueError
            for key in ('dns', 'dns-data', 'dns-search', 'gateway'):
                if key in stored['ipv4'] and _value(stored['ipv4'][key]):
                    raise ValueError
        except (KeyError, TypeError, AttributeError, ValueError):
            raise WifiLinkError('unsaved_profile_changed_required_properties') from None
        if await self._read(self._get(self._owned, NM+'.Settings.Connection', 'Unsaved', 'unsaved')) is not True:
            raise WifiLinkError('profile_is_not_unsaved')
        self.profile_verified = True

    async def _create_profile(self, settings):
        if self._journal is not None:
            self._check_identity()
            active=self._journal.active
            if (active is None or active['uuid']!=self.connection_uuid
                    or active['stage']!='create_pending'
                    or active['profile_sha256']!=_profile_digest(settings)):
                raise WifiLinkError('wifi_ownership_record_changed')
            self._check_stop()
        self._creation_attempted = True
        path = await self._invoke(SETTINGS, NM+'.Settings', 'AddConnectionUnsaved',
                                  'add_connection_unsaved', (settings,), 'a{sa{sv}}')
        if not _object_path(path, 'Settings') or path in self._paths:
            self._ownership_conflict = True
            raise WifiLinkError('new_profile_object_is_not_owned')
        self._owned = path
        if self._journal is not None:
            self._journal.advance(self.connection_uuid,'created')

    async def _activate(self):
        if self._journal is not None:
            self._check_identity()
            self._journal.advance(self.connection_uuid,'activate_pending')
            await self._verify_exact_profile()
            await self._read(asyncio.to_thread(_require_idle_wifi_interface,
                self.interface,cancel_event=self.stop_event))
            if await self._read(asyncio.to_thread(_routes,cancel_event=self.stop_event))!=self._baseline:
                raise WifiLinkError('default_routes_changed_before_activation')
            self._check_identity()
            self._check_stop()
        self._activation_attempted=True
        path = await self._invoke(MANAGER, NM, 'ActivateConnection', 'activate_connection',
                                  (self._owned, self._device, '/'), 'ooo')
        if not _object_path(path, 'ActiveConnection') or path in self._active_paths:
            raise WifiLinkError('new_active_object_is_not_owned')
        self._active = path
        if self._journal is not None:
            self._journal.advance(self.connection_uuid,'active')

    async def _active_is_owned(self, path):
        interface = NM+'.Connection.Active'
        if not (path not in self._active_paths
            and await self._get(path, interface, 'Uuid', 'uuid') == self.connection_uuid
            and await self._get(path, interface, 'Connection', 'connection') == self._owned):return False
        devices=await self._get(path, interface, 'Devices', 'devices')
        return devices==[self._device] if self._journal is not None else self._device in devices

    async def __aenter__(self):
        if self.state != 'new':
            raise WifiLinkError('wifi_link_context_cannot_be_reused')
        self.state = 'connecting'
        try:
            self._check_stop()
            if self.ownership_directory is not None:
                self._journal=WifiOwnershipStore(self.ownership_directory,self.interface,
                    ownership_identity(self.interface))
                self._journal.__enter__()
                if self._journal.active is not None:
                    raise WifiLinkError('wifi_ownership_recovery_required')
            self._baseline = await self._read(asyncio.to_thread(_routes, cancel_event=self.stop_event))
            for _, route in self._baseline:
                if self.interface in [route.get('dev')]+[hop.get('dev') for hop in route.get('nexthops', [])]:
                    raise WifiLinkError('interface_carries_default_route')
            await self._read(asyncio.to_thread(_require_idle_wifi_interface,
                                              self.interface, cancel_event=self.stop_event))
            self._profiles = await self._uuids()
            self._active_profiles = await self._uuids(active=True)
            if self.connection_uuid in self._profiles:
                raise WifiLinkError('connection_uuid_already_exists')
            self._bus = _new_bus()
            await self._read(self._bus.connect())
            if self._journal is not None:await self._pin_daemon()
            self._paths = set(await self._read(self._invoke(SETTINGS, NM+'.Settings',
                                                            'ListConnections', 'list_connections')))
            self._active_paths = set(await self._read(self._get(MANAGER, NM, 'ActiveConnections', 'active_connections')))
            self._device = await self._read(self._invoke(MANAGER, NM, 'GetDeviceByIpIface',
                                               'get_device_by_ip_iface', (self.interface,), 's'))
            if not _object_path(self._device, 'Devices'):
                raise WifiLinkError('invalid_networkmanager_device')
            if (await self._read(self._get(self._device, NM+'.Device', 'Interface', 'interface')) != self.interface
                    or await self._read(self._get(self._device, NM+'.Device', 'DeviceType', 'device_type')) != 2):
                raise WifiLinkError('networkmanager_device_does_not_match_interface')
            settings = self._settings()
            if self._journal is not None:
                self._check_identity()
                self._journal.begin(self.connection_uuid,_profile_digest(settings),self._destination)
                self._journal_active=True
            await self._mutate(self._create_profile(settings))
            await self._verify_settings(settings)
            if self._journal is not None:await self._verify_exact_profile()
            self._event('wifi_memory_profile_verified')
            await self._read(asyncio.to_thread(_require_idle_wifi_interface,
                                              self.interface, cancel_event=self.stop_event))
            if await self._read(asyncio.to_thread(_routes, cancel_event=self.stop_event)) != self._baseline:
                raise WifiLinkError('default_routes_changed_before_activation')
            if self._journal is not None:
                await self._verify_exact_profile()
            await self._mutate(self._activate())
            deadline = time.monotonic()+ACTIVATION_TIMEOUT
            while True:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise WifiLinkError('wifi_activation_timeout')
                state = await self._read(self._get(self._device, NM+'.Device', 'State', 'state'),
                                         timeout=min(READ_TIMEOUT, remaining))
                if state == 100:
                    break
                if state == 120:
                    raise WifiLinkError('wifi_activation_failed')
                if time.monotonic() >= deadline:
                    raise WifiLinkError('wifi_activation_timeout')
                await self._read(asyncio.sleep(.1))
            current = await self._command('nmcli', '--get-values', 'GENERAL.CON-UUID',
                                          'device', 'show', self.interface)
            if current.stdout.strip() != self.connection_uuid or not await self._read(self._active_is_owned(self._active)):
                raise WifiLinkError('selected_profile_is_not_active')
            ap = await self._read(self._get(self._device, NM+'.Device.Wireless',
                                            'ActiveAccessPoint', 'active_access_point'))
            if not _object_path(ap, 'AccessPoint'):
                raise WifiLinkError('wifi_access_point_is_not_verified')
            bssid = await self._read(self._get(ap, NM+'.AccessPoint', 'HwAddress', 'hw_address'))
            ssid = await self._read(self._get(ap, NM+'.AccessPoint', 'Ssid', 'ssid'))
            try:
                matches = (bytes.fromhex(bssid.replace(':', '')) == self._credentials.bssid
                           and bytes(ssid) == self._credentials.ssid)
            except (ValueError, TypeError, AttributeError):
                matches = False
            if not matches:
                raise WifiLinkError('associated_network_does_not_match_credentials')
            if await self._read(asyncio.to_thread(_routes, cancel_event=self.stop_event)) != self._baseline:
                raise WifiLinkError('default_routes_changed_after_activation')
            await self._read(asyncio.to_thread(require_dedicated_drone_route,
                                               self.interface, cancel_event=self.stop_event))
            self.route_verified = self.connected = True
            self.state = 'connected'
            self._event('wifi_memory_connected')
            self._check_stop()
            return self
        except BaseException as error:
            self.reason = str(error) if isinstance(error, WifiLinkError) else (
                'wifi_connection_cancelled' if isinstance(error, asyncio.CancelledError)
                else 'wifi_memory_connection_failed')
            await self._finish_cleanup()
            if isinstance(error, (WifiLinkError, asyncio.CancelledError, KeyboardInterrupt, SystemExit)):
                raise
            raise WifiLinkError(self.reason) from None

    async def _cleanup(self):
        if self._journal is not None:
            await self._durable_cleanup()
            return
        if self._pending is not None:
            try:
                await asyncio.wait_for(asyncio.shield(self._pending), MUTATION_SETTLE_TIMEOUT)
            except (TimeoutError, asyncio.CancelledError):
                # NM can have processed a request whose reply never arrived.
                # Recover matching objects below, but never claim that an
                # unresolved remote mutation has been completely cleaned up.
                self._mutation_unknown = True
                self.cleanup_errors.append('networkmanager_mutation_outcome_unknown')
                self._pending.cancel()
                await asyncio.gather(self._pending, return_exceptions=True)
            except Exception:
                pass
            self._pending = None
        if (self._owned is None and self._creation_attempted and not self._ownership_conflict
                and self._bus is not None):
            try:
                path = await self._read(self._invoke(SETTINGS, NM+'.Settings',
                    'GetConnectionByUuid', 'get_connection_by_uuid',
                    (self.connection_uuid,), 's'), cleanup=True)
                if _object_path(path, 'Settings') and path not in self._paths:
                    stored = await self._read(self._invoke(path, NM+'.Settings.Connection',
                        'GetSettings', 'get_settings'), cleanup=True)
                    if self._owns_settings(stored):
                        self._owned = path
            except Exception:
                pass  # Absence is confirmed below from the actual profile list.
        try:
            self._event('wifi_memory_cleanup_started')
        except WifiLinkError:
            self.cleanup_errors.append('wifi_event_sink_failed')
        if self._owned is not None:
            try:
                stored = await self._read(self._stored(), cleanup=True)
                if not self._owns_settings(stored) or self._ownership_conflict:
                    raise WifiLinkError('owned_profile_identity_changed')
                # An activation can fail after NM changed state but before it
                # returned the active path. Discover only a matching owned
                # UUID+profile+device; never disconnect the selected device.
                try:
                    paths = await self._read(self._get(MANAGER, NM, 'ActiveConnections',
                                                       'active_connections'), cleanup=True)
                except Exception:
                    self.cleanup_errors.append('active_profile_list_unavailable')
                    paths = [self._active] if self._active is not None else []
                for path in paths:
                    try:
                        if await self._read(self._active_is_owned(path), cleanup=True):
                            await self._read(self._invoke(MANAGER, NM, 'DeactivateConnection',
                                'deactivate_connection', (path,), 'o'), cleanup=True)
                    except Exception:
                        self.cleanup_errors.append('owned_profile_deactivation_failed')
                try:
                    await self._read(self._invoke(self._owned, NM+'.Settings.Connection',
                        'Delete', 'delete'), cleanup=True)
                except Exception:
                    self.cleanup_errors.append('owned_profile_deletion_failed')
            except Exception:
                self.cleanup_errors.append('owned_profile_cleanup_not_verified')
        try:
            remaining = await self._uuids(cleanup=True) if self._profiles is not None else set()
            active = await self._uuids(active=True, cleanup=True) if self._active_profiles is not None else set()
            self.profile_removed = self.connection_uuid not in remaining and not self._ownership_conflict
            self.deactivated = self.connection_uuid not in active
            self.existing_profiles_preserved = self._profiles is None or self._profiles <= remaining
            self.existing_active_profiles_preserved = self._active_profiles is None or self._active_profiles <= active
            self.default_routes_preserved = (self._baseline is None or self._baseline ==
                await self._read(asyncio.to_thread(_routes), cleanup=True))
            if self._owned is not None:
                paths = await self._read(self._invoke(SETTINGS, NM+'.Settings',
                    'ListConnections', 'list_connections'), cleanup=True)
                self.profile_removed = self.profile_removed and self._owned not in paths
                self.existing_profiles_preserved = self.existing_profiles_preserved and self._paths <= set(paths)
                active_paths = await self._read(self._get(MANAGER, NM, 'ActiveConnections',
                    'active_connections'), cleanup=True)
                self.existing_active_profiles_preserved = (self.existing_active_profiles_preserved
                    and self._active_paths <= set(active_paths))
                deadline=time.monotonic()+DEACTIVATION_TIMEOUT
                self.interface_idle_after_cleanup=False
                while time.monotonic()<deadline:
                    try:
                        await self._read(asyncio.to_thread(_require_idle_wifi_interface,
                            self.interface),cleanup=True,timeout=max(.01,deadline-time.monotonic()))
                        self.interface_idle_after_cleanup=True
                        break
                    except Exception:
                        if time.monotonic()<deadline:await asyncio.sleep(.1)
            self.cleanup_verified = all((self.profile_removed, self.deactivated,
                self.existing_profiles_preserved, self.existing_active_profiles_preserved,
                self.default_routes_preserved, self.interface_idle_after_cleanup is not False,
                not self._mutation_unknown))
        except Exception:
            self.cleanup_errors.append('wifi_cleanup_verification_failed')
            self.cleanup_verified = False
        finally:
            self.connected = False
            if self._bus is not None:
                try:
                    self._bus.disconnect()
                except Exception:
                    self.cleanup_errors.append('networkmanager_bus_disconnect_failed')
                    self.cleanup_verified = False
                self._bus = None
                self._proxies.clear()
            self.state = 'closed' if self.cleanup_verified else 'cleanup_failed'
        try:
            self._event('wifi_memory_removed' if self.cleanup_verified else 'wifi_memory_cleanup_failed')
        except WifiLinkError:
            self.cleanup_errors.append('wifi_event_sink_failed')

    async def _finish_cleanup(self):
        if self.cleanup_verified:
            return
        if self._cleanup_task is None:
            self._cleanup_task = asyncio.create_task(self._cleanup())
        cancelled = False
        while True:
            try:
                await asyncio.shield(self._cleanup_task)
                break
            except asyncio.CancelledError:
                if self._cleanup_task.cancelled():
                    self.cleanup_verified = False
                    self.state = 'cleanup_failed'
                    raise WifiLinkError('wifi_memory_cleanup_failed') from None
                cancelled = True
        if cancelled:
            raise asyncio.CancelledError

    async def __aexit__(self, exc_type, exc, traceback):
        await self._finish_cleanup()
        if not self.cleanup_verified:
            raise WifiLinkError('wifi_memory_cleanup_failed') from None
        if 'wifi_event_sink_failed' in self.cleanup_errors and exc is None:
            raise WifiLinkError('wifi_event_sink_failed') from None
        return False


async def recover_memory_wifi(interface, ownership_directory, *, stop_event=None, event_sink=None):
    """Explicitly recover the exact orphan profile, under the caller's DeviceLease.

    Call before the idle-interface guard and BLE bootstrap. The returned snapshot
    contains no credentials. Success is a durable clean journal and an idle
    interface; failure leaves ownership unresolved. In particular an unobserved
    pending Add/Activate cannot be cancelled by deleting unrelated connections.
    The journal-to-send crash window cannot be distinguished from a delayed NM
    request. Absence in that window stays blocked until the exact effect is
    observed or the original daemon/boot is proved to have ended. Modified or
    saved profiles, denied GetSecrets, unrecognized NM normalization and a
    changed owner/device with a remaining profile are also refused. The caller
    must keep other interface owners out with DeviceLease; NM has no atomic
    compare-settings-and-delete operation against unrelated external clients.
    No connection is created or activated here.
    """
    if not isinstance(interface,str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}',interface):
        raise ValueError('invalid Linux network interface name')
    if stop_event is not None and not callable(getattr(stop_event,'is_set',None)):
        raise ValueError('stop_event must expose is_set()')
    if event_sink is not None and not callable(event_sink):raise ValueError('event_sink must be callable')
    if ownership_directory is None:raise ValueError('wifi_ownership_directory_required')
    link=MemoryWifiLink.__new__(MemoryWifiLink)
    link._initialize(None,interface,stop_event,event_sink,ownership_directory)
    link._recovering=True
    link.state='recovering'
    try:
        link._check_stop()
        link._journal=WifiOwnershipStore(ownership_directory,interface,ownership_identity(interface))
        link._journal.__enter__()
        record=link._journal.active
        if record is None:
            return {'state':'clean','interface':interface,'durable_ownership':True,
                'cleanup_verified':True,'recovered':False}
        link.connection_uuid=record['uuid']
        link.connection_id='neo2-control-'+record['uuid']
        link._journal_active=True
        link._bus=_new_bus()
        await link._read(link._bus.connect())
        await link._pin_daemon()
        link._check_stop()
        await link._finish_cleanup()
        if not link.cleanup_verified:raise WifiLinkError(link.reason or 'wifi_ownership_recovery_failed')
        return dict(link.snapshot(),recovered=True)
    except (WifiLinkError,asyncio.CancelledError,KeyboardInterrupt,SystemExit):raise
    except Exception:raise WifiLinkError('wifi_ownership_recovery_failed') from None
    finally:
        if link._bus is not None:
            try:link._bus.disconnect()
            except Exception:pass
        if link._journal is not None:link._journal.close()
