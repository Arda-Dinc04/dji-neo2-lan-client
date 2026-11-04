import os
from pathlib import Path
import select
import subprocess
import sys

import pytest

from neo2_control.device_lease import DeviceLease


ADDRESS='02:00:00:00:00:01'


@pytest.mark.parametrize('interface,address',[('wlanusb0',ADDRESS),('other0',ADDRESS),('wlanusb0','02:00:00:00:00:02')])
def test_drone_and_adapter_each_require_exclusive_ownership(tmp_path,interface,address):
    with DeviceLease('wlanusb0',ADDRESS,directory=tmp_path/'locks'):
        with pytest.raises(RuntimeError,match='already_owned'):
            with DeviceLease(interface,address,directory=tmp_path/'locks'):pass
    with DeviceLease(interface,address,directory=tmp_path/'locks'):pass


def test_actual_other_process_holds_ownership_until_exit(tmp_path):
    directory=tmp_path/'locks'
    script="from neo2_control.device_lease import DeviceLease; import sys; lease=DeviceLease('wlanusb0',sys.argv[2],directory=sys.argv[1]); lease.__enter__(); print('owned',flush=True); sys.stdin.readline(); lease.close()"
    proc=subprocess.Popen([sys.executable,'-c',script,str(directory),ADDRESS],stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,env={**os.environ,'PYTHONPATH':str(Path.cwd()/'src')})
    try:
        assert select.select([proc.stdout],[],[],3)[0]
        assert proc.stdout.readline()==b'owned\n'
        with pytest.raises(RuntimeError,match='already_owned'):
            with DeviceLease('wlanusb0',ADDRESS,directory=directory):pass
        proc.communicate(b'close\n',timeout=2)
        with DeviceLease('wlanusb0',ADDRESS,directory=directory):pass
    finally:
        if proc.poll() is None:proc.kill();proc.wait(timeout=2)


def test_symlink_and_shared_permissions_cannot_replace_lease_storage(tmp_path):
    target=tmp_path/'target';target.mkdir(mode=0o700)
    alias=tmp_path/'alias';alias.symlink_to(target)
    with pytest.raises(RuntimeError,match='unsafe_device_lease_directory'):
        with DeviceLease('wlanusb0',ADDRESS,directory=alias):pass
    target.chmod(0o777)
    with pytest.raises(RuntimeError,match='unsafe_device_lease_directory'):
        with DeviceLease('wlanusb0',ADDRESS,directory=target):pass
