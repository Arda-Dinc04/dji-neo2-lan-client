"""Durable, exclusively owned uncertainty about one aircraft's ground state.

This store records local decisions, not aircraft telemetry. Its caller must
establish fresh, matching physical ground evidence before resolving an incident.
Construction performs no filesystem work; ownership lasts for the context.
"""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import stat
import threading


_FORMAT = 'neo2-control/flight-incident/v1'
_MAXIMUM_BYTES = 4096
_REASONS = frozenset(('takeoff_requested', 'airborne_observed'))
_NOFOLLOW = os.O_NOFOLLOW | os.O_CLOEXEC


def _identity(peer_id_sha256, peer_hash):
    if (not isinstance(peer_id_sha256, str)
            or re.fullmatch(r'[0-9a-f]{64}', peer_id_sha256) is None
            or type(peer_hash) is not int or not 0 <= peer_hash < 2**32):
        raise ValueError('invalid_flight_incident_peer')


def _utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='microseconds').replace('+00:00', 'Z')


def _valid_utc(value):
    if not isinstance(value, str) or re.fullmatch(
            r'[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{6}Z', value) is None:
        return False
    try:
        datetime.strptime(value, '%Y-%m-%dT%H:%M:%S.%fZ')
    except ValueError:
        return False
    return True


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('invalid_flight_incident_record')
        result[key] = value
    return result


def _constant(_value):
    raise ValueError('invalid_flight_incident_record')


class FlightIncidentStore:
    """Per-BLE-address private incident file and nonblocking lifetime lock.

    An I/O or validation failure poisons this instance. Reopen explicitly to
    inspect the surviving on-disk state; a failed mutation never reports clean.
    """

    def __init__(self, directory, address):
        if not isinstance(address, str) or re.fullmatch(
                r'(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}', address) is None:
            raise ValueError('invalid_flight_incident_address')
        self.directory = Path(directory)
        if '..' in self.directory.parts or '\0' in str(self.directory):
            raise ValueError('unsafe_flight_incident_directory')
        self.key = hashlib.sha256(address.upper().encode('ascii')).hexdigest()
        self.filename = self.key + '.json'
        self._root = self._lock_fd = None
        self._record = None
        self._used = self._failed = False
        self._mutex = threading.RLock()

    @staticmethod
    def _private_file(fd):
        info = os.fstat(fd)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                or stat.S_IMODE(info.st_mode) != 0o600 or info.st_nlink != 1):
            raise RuntimeError('unsafe_flight_incident_file')

    def _open_directory(self):
        # Walk with directory FDs so symlinks in parent components cannot be
        # followed, including while constructing a new private directory.
        parts = self.directory.absolute().parts
        fd = os.open(parts[0], os.O_RDONLY | os.O_DIRECTORY | _NOFOLLOW)
        try:
            for part in parts[1:]:
                try:
                    child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | _NOFOLLOW, dir_fd=fd)
                except FileNotFoundError:
                    try:
                        os.mkdir(part, 0o700, dir_fd=fd)
                        os.fsync(fd)
                    except FileExistsError:
                        pass
                    child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | _NOFOLLOW, dir_fd=fd)
                os.close(fd)
                fd = child
            info = os.fstat(fd)
            if info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o700:
                raise RuntimeError('unsafe_flight_incident_directory')
            return fd
        except BaseException:
            os.close(fd)
            raise

    def __enter__(self):
        with self._mutex:
            if self._used:
                raise RuntimeError('flight_incident_store_cannot_be_reused')
            self._used = True
            try:
                self._root = self._open_directory()
                self._lock_fd = os.open(self.key + '.lock',
                    os.O_CREAT | os.O_RDWR | os.O_NONBLOCK | _NOFOLLOW, 0o600, dir_fd=self._root)
                self._private_file(self._lock_fd)
                try:
                    fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    raise RuntimeError('flight_incident_already_owned') from None
                self._record = self._load()
                return self
            except BaseException:
                self.close()
                raise

    def _require_open(self):
        if self._root is None or self._lock_fd is None:
            raise RuntimeError('flight_incident_store_not_open')
        if self._failed:
            raise RuntimeError('flight_incident_store_failed')
        try:
            info = os.fstat(self._root)
            if info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o700:
                raise RuntimeError('unsafe_flight_incident_directory')
            self._private_file(self._lock_fd)
        except BaseException:
            self._failed = True
            raise

    def _validate(self, value):
        if (not isinstance(value, dict)
                or set(value) != {'format', 'aircraft_key', 'active', 'resolution'}
                or value['format'] != _FORMAT or value['aircraft_key'] != self.key):
            raise ValueError('invalid_flight_incident_record')
        for name in ('active', 'resolution'):
            item = value[name]
            if item is None:
                continue
            expected = ({'incident_id', 'peer_id_sha256', 'peer_hash', 'reason', 'created_at'}
                if name == 'active' else {'incident_id', 'peer_id_sha256', 'peer_hash', 'resolved_at'})
            if (not isinstance(item, dict) or set(item) != expected
                    or not isinstance(item['incident_id'], str)
                    or re.fullmatch(r'[0-9a-f]{32}', item['incident_id']) is None):
                raise ValueError('invalid_flight_incident_record')
            _identity(item['peer_id_sha256'], item['peer_hash'])
            timestamp = item['created_at'] if name == 'active' else item['resolved_at']
            if not _valid_utc(timestamp) or (name == 'active' and item['reason'] not in _REASONS):
                raise ValueError('invalid_flight_incident_record')
        return value

    def _load(self):
        try:
            fd = os.open(self.filename, os.O_RDONLY | os.O_NONBLOCK | _NOFOLLOW, dir_fd=self._root)
        except FileNotFoundError:
            return None
        try:
            self._private_file(fd)
            with os.fdopen(fd, 'rb', closefd=False) as stream:
                data = stream.read(_MAXIMUM_BYTES + 1)
        finally:
            os.close(fd)
        if len(data) > _MAXIMUM_BYTES:
            raise ValueError('invalid_flight_incident_record')
        try:
            value = json.loads(data, object_pairs_hook=_object, parse_constant=_constant)
            return self._validate(value)
        except (ValueError, TypeError, RecursionError):
            raise ValueError('invalid_flight_incident_record') from None

    def _check_unchanged(self):
        try:
            if self._load() != self._record:
                raise RuntimeError('flight_incident_record_changed')
        except BaseException:
            self._failed = True
            raise

    @property
    def active(self):
        with self._mutex:
            self._require_open()
            self._check_unchanged()
            return deepcopy(None if self._record is None else self._record['active'])

    def _persist(self, record):
        self._validate(record)
        data = (json.dumps(record, allow_nan=False, sort_keys=True, separators=(',', ':')) + '\n').encode('ascii')
        if len(data) > _MAXIMUM_BYTES:
            raise ValueError('invalid_flight_incident_record')
        temporary = '.' + self.key + '.' + secrets.token_hex(12) + '.tmp'
        fd = None
        try:
            fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | _NOFOLLOW,
                         0o600, dir_fd=self._root)
            os.fchmod(fd, 0o600)
            self._private_file(fd)
            remaining = memoryview(data)
            while remaining:
                written = os.write(fd, remaining)
                if written <= 0:
                    raise OSError('flight_incident_write_failed')
                remaining = remaining[written:]
            os.fsync(fd)
            os.close(fd)
            fd = None
            os.replace(temporary, self.filename, src_dir_fd=self._root, dst_dir_fd=self._root)
            os.fsync(self._root)
            self._record = record
        except BaseException:
            self._failed = True
            raise
        finally:
            if fd is not None:
                os.close(fd)
            try:
                os.unlink(temporary, dir_fd=self._root)
            except FileNotFoundError:
                pass

    def mark_airborne(self, peer_id_sha256, peer_hash, *, reason='takeoff_requested'):
        _identity(peer_id_sha256, peer_hash)
        if not isinstance(reason, str) or reason not in _REASONS:
            raise ValueError('invalid_flight_incident_reason')
        with self._mutex:
            self._require_open()
            self._check_unchanged()
            active = None if self._record is None else self._record['active']
            if active is not None:
                if (active['peer_id_sha256'], active['peer_hash']) != (peer_id_sha256, peer_hash):
                    raise ValueError('flight_incident_peer_mismatch')
                return deepcopy(active)
            active = {'incident_id': secrets.token_hex(16), 'peer_id_sha256': peer_id_sha256,
                      'peer_hash': peer_hash, 'reason': reason, 'created_at': _utc_now()}
            self._persist({'format': _FORMAT, 'aircraft_key': self.key, 'active': active,
                'resolution': None if self._record is None else self._record['resolution']})
            return deepcopy(active)

    def resolve_ground(self, peer_id_sha256, peer_hash):
        _identity(peer_id_sha256, peer_hash)
        with self._mutex:
            self._require_open()
            self._check_unchanged()
            active = None if self._record is None else self._record['active']
            if active is None:
                raise ValueError('flight_incident_not_active')
            if (active['peer_id_sha256'], active['peer_hash']) != (peer_id_sha256, peer_hash):
                raise ValueError('flight_incident_peer_mismatch')
            resolution = {key: active[key] for key in ('incident_id', 'peer_id_sha256', 'peer_hash')}
            resolution['resolved_at'] = _utc_now()
            self._persist({'format': _FORMAT, 'aircraft_key': self.key,
                           'active': None, 'resolution': resolution})

    def close(self):
        with self._mutex:
            for name in ('_lock_fd', '_root'):
                fd = getattr(self, name)
                if fd is not None:
                    setattr(self, name, None)
                    os.close(fd)

    def __exit__(self, *args):
        self.close()
