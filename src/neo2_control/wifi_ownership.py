"""Private write-ahead ownership of one unsaved NM profile, without secrets.

The existing incident store supplies descriptor-based private I/O, lifetime
flock, bounded strict JSON and atomic file/directory fsync. Its flight-specific
public mutations are deliberately not used here.
"""
from copy import deepcopy
import hashlib
import os
from pathlib import Path
import re
import uuid

from .flight_incident import FlightIncidentStore


FORMAT = 'neo2-control/wifi-ownership/v1'
STAGES = ('create_pending','created','activate_pending','active','cleanup_pending')
IDENTITY_KEYS = {'machine_id_sha256','boot_id','uid','device_id_sha256','ifindex','network_namespace'}
_TRANSITIONS = {
    'create_pending': {'created', 'cleanup_pending'},
    'created': {'activate_pending', 'cleanup_pending'},
    'activate_pending': {'active', 'cleanup_pending'},
    'active': {'cleanup_pending'},
    'cleanup_pending': set(),
}


def daemon_owner(value):
    return isinstance(value, str) and re.fullmatch(r':[0-9]+\.[0-9]+', value) is not None


def _uuid(value):
    return isinstance(value,str) and str(uuid.UUID(value)) == value


def _digest(value):
    return isinstance(value,str) and re.fullmatch('[0-9a-f]{64}',value) is not None


def _identity(value):
    if not isinstance(value,dict) or set(value)!=IDENTITY_KEYS:return False
    try:
        return (_digest(value['machine_id_sha256']) and _digest(value['device_id_sha256'])
            and _uuid(value['boot_id']) and all(type(value[k]) is int for k in ('uid','ifindex','network_namespace'))
            and 0 <= value['uid'] < 2**32 and 0 < value['ifindex'] < 2**31
            and 0 < value['network_namespace'] < 2**64)
    except (TypeError,ValueError,AttributeError):return False


def ownership_identity(interface):
    """Read local kernel identity only; never connect or change an interface."""
    if not isinstance(interface,str) or re.fullmatch('[A-Za-z0-9_.-]{1,15}',interface) is None:
        raise ValueError('invalid_wifi_ownership_interface')
    try:
        machine=Path('/etc/machine-id').read_text().strip()
        if re.fullmatch('[0-9a-f]{32}',machine) is None:raise ValueError
        device=Path('/sys/class/net')/interface
        identity={'machine_id_sha256':hashlib.sha256(machine.encode()).hexdigest(),
            'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'uid':os.getuid(),'ifindex':int((device/'ifindex').read_text()),
            'device_id_sha256':hashlib.sha256(str((device/'device').resolve(strict=True)).encode()).hexdigest(),
            'network_namespace':os.stat('/proc/self/ns/net').st_ino}
        if not _identity(identity):raise ValueError
        return identity
    except (OSError,ValueError):raise RuntimeError('wifi_ownership_identity_unavailable') from None


class WifiOwnershipStore(FlightIncidentStore):
    """One private journal/lock per interface; construction is inert."""
    def __init__(self,directory,interface,identity):
        if not isinstance(interface,str) or re.fullmatch('[A-Za-z0-9_.-]{1,15}',interface) is None:
            raise ValueError('invalid_wifi_ownership_interface')
        if not _identity(identity):raise ValueError('invalid_wifi_ownership_identity')
        super().__init__(directory,'00:00:00:00:00:00')
        self.interface,self.identity=interface,deepcopy(identity)
        self.key=hashlib.sha256(('wifi:'+interface).encode()).hexdigest()
        self.filename=self.key+'.json'

    def _validate(self,value):
        try:
            if (not isinstance(value,dict) or set(value)!={'format','owner_key','interface','active'}
                    or value['format']!=FORMAT or value['owner_key']!=self.key
                    or value['interface']!=self.interface):raise ValueError
            active=value['active']
            if active is not None and (not isinstance(active,dict)
                    or set(active)!={'uuid','identity','profile_sha256','stage','daemon_owner'}
                    or not _uuid(active['uuid']) or not _identity(active['identity'])
                    or not _digest(active['profile_sha256']) or active['stage'] not in STAGES
                    or not daemon_owner(active['daemon_owner'])):raise ValueError
            return value
        except (ValueError,TypeError,AttributeError):raise ValueError('invalid_wifi_ownership_record') from None

    def _record_with(self,active):
        return {'format':FORMAT,'owner_key':self.key,'interface':self.interface,'active':active}

    def begin(self,connection_uuid,profile_sha256,networkmanager_owner):
        with self._mutex:
            self._require_open();self._check_unchanged()
            if self.active is not None:raise RuntimeError('wifi_ownership_recovery_required')
            active={'uuid':connection_uuid,'profile_sha256':profile_sha256,
                    'identity':deepcopy(self.identity),'stage':'create_pending',
                    'daemon_owner':networkmanager_owner}
            self._persist(self._record_with(active))
            return deepcopy(active)

    def advance(self,connection_uuid,stage):
        if stage not in STAGES:raise ValueError('invalid_wifi_ownership_stage')
        with self._mutex:
            self._require_open();self._check_unchanged()
            active=self.active
            if active is None or active['uuid']!=connection_uuid:raise RuntimeError('wifi_ownership_record_changed')
            if active['identity']!=self.identity:raise RuntimeError('wifi_ownership_identity_changed')
            if stage != active['stage'] and stage not in _TRANSITIONS[active['stage']]:
                raise ValueError('invalid_wifi_ownership_transition')
            active['stage']=stage
            self._persist(self._record_with(active))

    def clear(self,connection_uuid):
        with self._mutex:
            self._require_open();self._check_unchanged()
            active=self.active
            if active is None or active['uuid']!=connection_uuid:raise RuntimeError('wifi_ownership_record_changed')
            if any(active['identity'][key]!=self.identity[key] for key in ('uid','machine_id_sha256')):
                raise RuntimeError('wifi_ownership_identity_changed')
            self._persist(self._record_with(None))

    def mark_airborne(self,*args,**kwargs):raise RuntimeError('not_a_flight_incident_store')
    def resolve_ground(self,*args,**kwargs):raise RuntimeError('not_a_flight_incident_store')
