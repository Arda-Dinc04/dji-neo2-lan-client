"""Per-user, cross-process exclusion for an owned drone and Wi-Fi adapter.

Kernel locks prove concurrent ownership only, not ground contact or cleanup.
The connection retains its separate BLE, network and socket cleanup evidence.
"""
import fcntl
import hashlib
import os
from pathlib import Path
import re
import stat
import tempfile


class DeviceLease:
    def __init__(self, interface, address, *, directory=None):
        if not isinstance(interface,str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}',interface):
            raise ValueError('invalid_owned_wifi_interface')
        if not isinstance(address,str) or not re.fullmatch(r'(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}',address):
            raise ValueError('invalid_owned_drone_address')
        self.directory=Path(directory) if directory is not None else Path(tempfile.gettempdir())/f'neo2-control-{os.getuid()}'
        self.keys=sorted(hashlib.sha256(value.encode()).hexdigest() for value in
                         ('wifi:'+interface,'drone:'+address.upper()))
        self.fds=[]
        self.used=False

    def __enter__(self):
        if self.used:raise RuntimeError('device_lease_cannot_be_reused')
        self.used=True
        self.directory.mkdir(mode=0o700,parents=True,exist_ok=True)
        info=self.directory.lstat()
        if not stat.S_ISDIR(info.st_mode) or info.st_uid!=os.getuid() or info.st_mode & 0o077:
            raise RuntimeError('unsafe_device_lease_directory')
        root=os.open(self.directory,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC)
        try:
            for key in self.keys:
                fd=os.open(key+'.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW|os.O_CLOEXEC,0o600,dir_fd=root)
                self.fds.append(fd)
                info=os.fstat(fd)
                if not stat.S_ISREG(info.st_mode) or info.st_uid!=os.getuid() or info.st_mode & 0o077:
                    raise RuntimeError('unsafe_device_lease_file')
                try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:raise RuntimeError('drone_or_wifi_interface_already_owned') from None
        except BaseException:
            self.close()
            raise
        finally:os.close(root)
        return self

    def close(self):
        while self.fds:os.close(self.fds.pop())

    def __exit__(self,*args):
        self.close()
