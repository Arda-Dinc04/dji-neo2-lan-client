"""Private durable uncertainty, tested without devices or network access."""
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys

import pytest

import neo2_control.flight_incident as module
from neo2_control.flight_incident import FlightIncidentStore


ADDRESS = '02:00:00:00:00:01'
PEER = 'a' * 64
PEER_HASH = 927510506


@pytest.fixture
def directory(tmp_path):
    path = tmp_path / 'incidents'
    path.mkdir(mode=0o700)
    return path


def record_path(directory):
    return directory / (hashlib.sha256(ADDRESS.encode('ascii')).hexdigest() + '.json')


def child(script, directory, *, expected=0):
    environment = dict(os.environ)
    environment['PYTHONPATH'] = str(Path(module.__file__).resolve().parents[1])
    result = subprocess.run([sys.executable, '-c', script, str(directory), ADDRESS],
                            env=environment, capture_output=True, text=True, timeout=10)
    assert result.returncode == expected, result.stderr
    return result


def test_constructor_is_inert_and_context_lifetime_is_explicit(tmp_path):
    path = tmp_path / 'new' / 'private'
    store = FlightIncidentStore(path, ADDRESS.lower())
    assert not path.exists()
    with pytest.raises(RuntimeError, match='not_open'):
        _ = store.active
    with store:
        assert path.is_dir() and stat.S_IMODE(path.stat().st_mode) == 0o700
        assert store.active is None and not record_path(path).exists()
    store.close()
    with pytest.raises(RuntimeError, match='not_open'):
        store.mark_airborne(PEER, PEER_HASH)
    with pytest.raises(RuntimeError, match='cannot_be_reused'):
        store.__enter__()


def test_mark_survives_abrupt_process_exit_and_reopen_without_ground_assumption(directory):
    child("""
import os, sys
from neo2_control.flight_incident import FlightIncidentStore
store = FlightIncidentStore(sys.argv[1], sys.argv[2])
store.__enter__()
store.mark_airborne('a' * 64, 927510506)
os._exit(23)
""", directory, expected=23)
    with FlightIncidentStore(directory, ADDRESS.lower()) as store:
        active = store.active
        assert active['peer_id_sha256'] == PEER and active['peer_hash'] == PEER_HASH
        assert active['reason'] == 'takeoff_requested'
        assert active['created_at'].endswith('Z')
        assert len(active['incident_id']) == 32
        first_bytes = record_path(directory).read_bytes()
        assert store.mark_airborne(PEER, PEER_HASH, reason='airborne_observed') == active
        assert record_path(directory).read_bytes() == first_bytes
        active['reason'] = 'changed_by_caller'
        assert store.active['reason'] == 'takeoff_requested'
    assert ADDRESS.encode('ascii') not in record_path(directory).read_bytes()
    assert stat.S_IMODE(record_path(directory).stat().st_mode) == 0o600


def test_exact_peer_resolution_is_persistent_and_keeps_a_resolution_record(directory):
    with FlightIncidentStore(directory, ADDRESS) as store:
        active = store.mark_airborne(PEER, PEER_HASH, reason='airborne_observed')
        for peer, routing_hash in [('b' * 64, PEER_HASH), (PEER, PEER_HASH + 1)]:
            before = record_path(directory).read_bytes()
            with pytest.raises(ValueError, match='peer_mismatch'):
                store.mark_airborne(peer, routing_hash)
            with pytest.raises(ValueError, match='peer_mismatch'):
                store.resolve_ground(peer, routing_hash)
            assert record_path(directory).read_bytes() == before
            assert store.active == active
        assert store.resolve_ground(PEER, PEER_HASH) is None
        assert store.active is None
        saved = json.loads(record_path(directory).read_bytes())
        assert saved['resolution']['incident_id'] == active['incident_id']
        assert saved['resolution']['resolved_at'].endswith('Z')
        with pytest.raises(ValueError, match='not_active'):
            store.resolve_ground(PEER, PEER_HASH)
    with FlightIncidentStore(directory, ADDRESS) as store:
        assert store.active is None
        another = store.mark_airborne(PEER, PEER_HASH)
        assert another['incident_id'] != active['incident_id']
        assert json.loads(record_path(directory).read_bytes())['resolution'] == saved['resolution']


def test_absence_cannot_be_resolved_as_a_ground_confirmation(directory):
    with FlightIncidentStore(directory, ADDRESS) as store:
        with pytest.raises(ValueError, match='not_active'):
            store.resolve_ground(PEER, PEER_HASH)
        assert not record_path(directory).exists()


def test_per_drone_lock_excludes_other_process_and_survives_failed_second_owner(directory):
    attempt = """
import sys
from neo2_control.flight_incident import FlightIncidentStore
try:
    with FlightIncidentStore(sys.argv[1], sys.argv[2]):
        raise SystemExit(4)
except RuntimeError as error:
    assert str(error) == 'flight_incident_already_owned'
"""
    with FlightIncidentStore(directory, ADDRESS) as owner:
        owner.mark_airborne(PEER, PEER_HASH)
        child(attempt, directory)
        child(attempt, directory)
        with FlightIncidentStore(directory, '02:00:00:00:00:02') as other:
            assert other.active is None
    with FlightIncidentStore(directory, ADDRESS) as reopened:
        assert reopened.active['peer_id_sha256'] == PEER


@pytest.mark.parametrize('change', [
    'bad-json', 'too-large', 'duplicate-key', 'nan', 'unknown-field', 'wrong-aircraft',
    'bool-hash', 'unknown-reason', 'bad-time', 'bad-incident', 'bad-resolution', 'nested',
])
def test_invalid_records_fail_closed_and_are_not_replaced(directory, change):
    with FlightIncidentStore(directory, ADDRESS) as store:
        store.mark_airborne(PEER, PEER_HASH)
    path = record_path(directory)
    value = json.loads(path.read_bytes())
    if change == 'bad-json': data = b'{'
    elif change == 'too-large': data = b' ' * 4097
    elif change == 'duplicate-key': data = path.read_bytes().replace(b'"active":', b'"active":null,"active":', 1)
    elif change == 'nan': data = path.read_bytes().replace(str(PEER_HASH).encode(), b'NaN', 1)
    elif change == 'nested': data = b'[' * 2000
    else:
        if change == 'unknown-field': value['extra'] = True
        elif change == 'wrong-aircraft': value['aircraft_key'] = 'b' * 64
        elif change == 'bool-hash': value['active']['peer_hash'] = True
        elif change == 'unknown-reason': value['active']['reason'] = 'land_acknowledged'
        elif change == 'bad-time': value['active']['created_at'] = '2026-02-30T01:02:03.000000Z'
        elif change == 'bad-incident': value['active']['incident_id'] = '../bad'
        elif change == 'bad-resolution': value['resolution'] = {'resolved_at': 'now'}
        data = json.dumps(value).encode()
    path.write_bytes(data)
    with pytest.raises(ValueError, match='invalid_flight_incident_record'):
        with FlightIncidentStore(directory, ADDRESS):
            pytest.fail('corrupt record was reported as clean')
    assert path.read_bytes() == data


@pytest.mark.parametrize('target', ['directory', 'parent-directory', 'record', 'lock'])
def test_symlinks_are_rejected_without_touching_their_target(tmp_path, target):
    real = tmp_path / 'real'
    real.mkdir(mode=0o700)
    if target in ('directory', 'parent-directory'):
        alias = tmp_path / 'alias'
        alias.symlink_to(real, target_is_directory=True)
        directory = alias if target == 'directory' else alias / 'child'
        with pytest.raises(OSError):
            with FlightIncidentStore(directory, ADDRESS): pass
        assert list(real.iterdir()) == []
    else:
        protected = tmp_path / 'protected'
        protected.write_bytes(b'unchanged')
        protected.chmod(0o600)
        name = record_path(real) if target == 'record' else real / (hashlib.sha256(ADDRESS.encode()).hexdigest() + '.lock')
        name.symlink_to(protected)
        with pytest.raises(OSError):
            with FlightIncidentStore(real, ADDRESS): pass
        assert protected.read_bytes() == b'unchanged'


@pytest.mark.parametrize('target', ['directory', 'record', 'lock', 'hardlink', 'fifo'])
def test_nonprivate_or_nonregular_storage_is_rejected(directory, tmp_path, target):
    with FlightIncidentStore(directory, ADDRESS) as store:
        store.mark_airborne(PEER, PEER_HASH)
    path = record_path(directory)
    if target == 'directory': directory.chmod(0o755)
    elif target == 'record': path.chmod(0o644)
    elif target == 'lock': next(directory.glob('*.lock')).chmod(0o644)
    elif target == 'hardlink': os.link(path, tmp_path / 'linked-record')
    else:
        path.unlink()
        os.mkfifo(path, mode=0o600)
    with pytest.raises(RuntimeError, match='unsafe_flight_incident'):
        with FlightIncidentStore(directory, ADDRESS): pass


@pytest.mark.parametrize('tamper', ['delete', 'corrupt', 'change', 'permissions', 'directory-permissions'])
def test_open_owner_never_clears_or_reports_clean_after_storage_tampering(directory, tamper):
    with FlightIncidentStore(directory, ADDRESS) as store:
        store.mark_airborne(PEER, PEER_HASH)
        path = record_path(directory)
        if tamper == 'delete': path.unlink()
        elif tamper == 'corrupt': path.write_bytes(b'{')
        elif tamper == 'change':
            value = json.loads(path.read_bytes());value['active'] = None
            path.write_text(json.dumps(value))
        elif tamper == 'permissions': path.chmod(0o644)
        else: directory.chmod(0o755)
        with pytest.raises((RuntimeError, ValueError)):
            store.resolve_ground(PEER, PEER_HASH)
        with pytest.raises(RuntimeError, match='store_failed'):
            _ = store.active


def test_mark_and_clear_sync_file_then_replace_then_sync_directory_before_return(directory, monkeypatch):
    with FlightIncidentStore(directory, ADDRESS) as store:
        events = []
        sync, replace = os.fsync, os.replace
        def fsync(fd):
            events.append('directory_sync' if stat.S_ISDIR(os.fstat(fd).st_mode) else 'file_sync')
            return sync(fd)
        def atomic_replace(*args, **kwargs):
            events.append('replace')
            return replace(*args, **kwargs)
        monkeypatch.setattr(module.os, 'fsync', fsync)
        monkeypatch.setattr(module.os, 'replace', atomic_replace)
        store.mark_airborne(PEER, PEER_HASH)
        assert events == ['file_sync', 'replace', 'directory_sync']
        events.clear()
        store.resolve_ground(PEER, PEER_HASH)
        assert events == ['file_sync', 'replace', 'directory_sync']


@pytest.mark.parametrize('operation', ['mark', 'resolve'])
@pytest.mark.parametrize('failure', ['file-sync', 'replace', 'directory-sync'])
def test_failed_durable_mutation_poisoned_owner_cannot_report_clean(directory, monkeypatch, operation, failure):
    with FlightIncidentStore(directory, ADDRESS) as store:
        if operation == 'resolve': store.mark_airborne(PEER, PEER_HASH)
        sync = os.fsync
        def fsync(fd):
            directory_fd = stat.S_ISDIR(os.fstat(fd).st_mode)
            if directory_fd == (failure == 'directory-sync'):
                raise OSError('injected sync failure')
            return sync(fd)
        def replace(*args, **kwargs):
            raise OSError('injected replace failure')
        with monkeypatch.context() as patch:
            if failure == 'replace': patch.setattr(module.os, 'replace', replace)
            else: patch.setattr(module.os, 'fsync', fsync)
            with pytest.raises(OSError, match='injected'):
                if operation == 'mark': store.mark_airborne(PEER, PEER_HASH)
                else: store.resolve_ground(PEER, PEER_HASH)
            with pytest.raises(RuntimeError, match='store_failed'):
                _ = store.active
            with pytest.raises(RuntimeError, match='store_failed'):
                store.mark_airborne(PEER, PEER_HASH)
        assert not list(directory.glob('*.tmp'))
    # A fresh owner reads the actual surviving record. Directory fsync failure
    # occurs after rename; this check is not a claim of reboot durability.
    expected_active = ((operation == 'mark' and failure == 'directory-sync')
                       or (operation == 'resolve' and failure != 'directory-sync'))
    with FlightIncidentStore(directory, ADDRESS) as reopened:
        assert (reopened.active is not None) is expected_active


@pytest.mark.parametrize('peer,routing_hash', [('bad', PEER_HASH), ('A' * 64, PEER_HASH),
    (PEER, True), (PEER, -1), (PEER, 2**32)])
def test_invalid_peer_never_creates_an_incident(directory, peer, routing_hash):
    with FlightIncidentStore(directory, ADDRESS) as store:
        with pytest.raises(ValueError, match='invalid_flight_incident_peer'):
            store.mark_airborne(peer, routing_hash)
        assert store.active is None and not record_path(directory).exists()


@pytest.mark.parametrize('reason', ['landing_confirmed', '', None, ['airborne_observed']])
def test_invalid_reason_never_creates_an_incident(directory, reason):
    with FlightIncidentStore(directory, ADDRESS) as store:
        with pytest.raises(ValueError, match='invalid_flight_incident_reason'):
            store.mark_airborne(PEER, PEER_HASH, reason=reason)
        assert store.active is None
