"""The private Wi-Fi write-ahead journal survives process death, never radio I/O."""
from copy import deepcopy
import json
import os
import select
import subprocess
import sys

import pytest

from neo2_control import flight_incident
from neo2_control.wifi_ownership import WifiOwnershipStore


IDENTITY={'machine_id_sha256':'a'*64,'boot_id':'00000000-0000-4000-8000-000000000001',
    'uid':os.getuid(),'device_id_sha256':'b'*64,'ifindex':19,'network_namespace':12345}
UUID='00000000-0000-4000-8000-000000000002'
DAEMON=':1.42'


def store(directory,identity=None):
    return WifiOwnershipStore(directory,'wlanusb0',IDENTITY if identity is None else identity)


def test_construction_is_inert_and_crash_reopen_retains_record(tmp_path):
    directory=tmp_path/'ownership'
    item=store(directory)
    assert not directory.exists()
    script='''import json,os,sys
from neo2_control.wifi_ownership import WifiOwnershipStore
with WifiOwnershipStore(sys.argv[1], 'wlanusb0', json.loads(sys.argv[2])) as s:
    s.begin(sys.argv[3], 'c'*64, ':1.42')
    s.advance(sys.argv[3], 'created')
    os._exit(0)
'''
    result=subprocess.run([sys.executable,'-c',script,str(directory),json.dumps(IDENTITY),UUID],
        capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr
    with item as opened:
        assert opened.active['stage']=='created'
        assert opened.active['uuid']==UUID and opened.active['daemon_owner']==DAEMON
        assert opened.active['identity']==IDENTITY
        with pytest.raises(RuntimeError,match='recovery_required'):opened.begin(UUID,'c'*64,DAEMON)
        opened.clear(UUID)
    with store(directory) as reopened:assert reopened.active is None
    contents=(directory/item.filename).read_text()
    assert 'psk' not in contents and 'password' not in contents


def test_lifetime_lock_excludes_another_process_until_close(tmp_path):
    script='''import json,sys
from neo2_control.wifi_ownership import WifiOwnershipStore
try:
    with WifiOwnershipStore(sys.argv[1], 'wlanusb0', json.loads(sys.argv[2])): pass
except RuntimeError: sys.exit(19)
'''
    directory=tmp_path/'ownership'
    with store(directory):
        result=subprocess.run([sys.executable,'-c',script,str(directory),json.dumps(IDENTITY)],
            capture_output=True,text=True,timeout=10)
        assert result.returncode==19,result.stderr
    result=subprocess.run([sys.executable,'-c',script,str(directory),json.dumps(IDENTITY)],
        capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr


def test_sigkill_releases_only_lock_and_never_clears_ownership(tmp_path):
    directory=tmp_path/'ownership'
    script='''import json,signal,sys
from neo2_control.wifi_ownership import WifiOwnershipStore
with WifiOwnershipStore(sys.argv[1], 'wlanusb0', json.loads(sys.argv[2])) as s:
    s.begin(sys.argv[3], 'c'*64, ':1.42')
    s.advance(sys.argv[3], 'created')
    s.advance(sys.argv[3], 'activate_pending')
    print('READY', flush=True)
    signal.pause()
'''
    process=subprocess.Popen([sys.executable,'-c',script,str(directory),json.dumps(IDENTITY),UUID],
        stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        assert select.select([process.stdout],[],[],10)[0]
        assert process.stdout.readline()=='READY\n'
        with pytest.raises(RuntimeError):
            with store(directory):pass
        process.kill()
        process.communicate(timeout=10)
        assert process.returncode==-9
    finally:
        if process.poll() is None:process.kill()
        process.communicate(timeout=10)
    with store(directory) as reopened:
        assert reopened.active['uuid']==UUID
        assert reopened.active['stage']=='activate_pending'


def test_strict_identity_stage_and_exact_uuid_cannot_be_replaced(tmp_path):
    directory=tmp_path/'ownership'
    with store(directory) as opened:
        record=opened.begin(UUID,'c'*64,DAEMON)
        assert record['stage']=='create_pending'
        record['identity']['ifindex']=900
        assert opened.active['identity']['ifindex']==19
        with pytest.raises(RuntimeError,match='record_changed'):opened.clear('00000000-0000-4000-8000-000000000003')
        with pytest.raises(ValueError,match='transition'):opened.advance(UUID,'active')
        opened.advance(UUID,'created')
        opened.advance(UUID,'activate_pending')
        opened.advance(UUID,'active')
        opened.advance(UUID,'cleanup_pending')
        with pytest.raises(ValueError,match='transition'):opened.advance(UUID,'created')
    changed=deepcopy(IDENTITY);changed['ifindex']+=1
    with store(directory,changed) as opened:
        with pytest.raises(RuntimeError,match='identity_changed'):opened.advance(UUID,'cleanup_pending')
        assert opened.active['identity']==IDENTITY


@pytest.mark.parametrize('bad',['symlink','mode','json','duplicate','wrong_interface','oversized'])
def test_invalid_or_unsafe_record_fails_closed(tmp_path,bad):
    directory=tmp_path/'ownership'
    with store(directory) as opened:opened.begin(UUID,'c'*64,DAEMON)
    path=directory/store(directory).filename
    if bad=='symlink':
        target=tmp_path/'foreign';target.write_text('foreign');path.unlink();path.symlink_to(target)
    elif bad=='mode':path.chmod(0o644)
    elif bad=='json':path.write_text('[')
    elif bad=='duplicate':path.write_text('{"active":null,"active":null}')
    elif bad=='oversized':path.write_bytes(b' '*4097)
    else:
        value=json.loads(path.read_text());value['interface']='eth0';path.write_text(json.dumps(value))
    with pytest.raises((OSError,ValueError,RuntimeError)):
        with store(directory):pass


@pytest.mark.parametrize('operation',['mark','clear'])
def test_directory_fsync_failure_never_reports_clean(tmp_path,monkeypatch,operation):
    directory=tmp_path/'ownership'
    with store(directory) as opened:
        if operation=='clear':opened.begin(UUID,'c'*64,DAEMON)
        original=flight_incident.os.fsync
        def fail(fd):
            if fd==opened._root:raise OSError('injected directory fsync')
            return original(fd)
        monkeypatch.setattr(flight_incident.os,'fsync',fail)
        with pytest.raises(OSError):
            if operation=='mark':opened.begin(UUID,'c'*64,DAEMON)
            else:opened.clear(UUID)
        with pytest.raises(RuntimeError):_ = opened.active
    # A new explicit context inspects what survived; the failed call itself
    # never gave permission for AddConnection or reported successful cleanup.
    monkeypatch.setattr(flight_incident.os,'fsync',original)
    with store(directory) as reopened:
        assert (reopened.active is None)==(operation=='clear')
