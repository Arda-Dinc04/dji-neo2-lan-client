"""Prepare one private SSH socket; never unlink a live or foreign endpoint.

This small module uses only Python's standard library so its exact source can
also run on the SSH host without installing the aircraft control package.
"""
import errno
import os
from pathlib import Path
import re
import socket
import stat


def prepare_socket(directory='/run/neo2-control',name='flight.sock',*,expected_uid=0):
    root=Path(directory)
    if (not root.is_absolute() or '..' in root.parts or type(expected_uid) is not int
            or expected_uid<0 or not isinstance(name,str)
            or re.fullmatch(r'[A-Za-z0-9_-]+\.sock',name) is None):
        raise ValueError('invalid_tunnel_socket_path')
    for parent in reversed((root,*root.parents)):
        try:metadata=parent.lstat()
        except FileNotFoundError:
            if parent!=root:raise RuntimeError('tunnel_parent_missing') from None
            continue
        if not stat.S_ISDIR(metadata.st_mode):raise RuntimeError('tunnel_directory_invalid')
    try:root.mkdir(mode=0o700)
    except FileExistsError:pass
    directory_stat=root.lstat()
    if (not stat.S_ISDIR(directory_stat.st_mode) or directory_stat.st_uid!=expected_uid
            or stat.S_IMODE(directory_stat.st_mode)!=0o700):
        raise RuntimeError('tunnel_directory_not_private')
    path=root/name
    try:before=path.lstat()
    except FileNotFoundError:return 'socket_absent'
    if (not stat.S_ISSOCK(before.st_mode) or before.st_uid!=expected_uid
            or stat.S_IMODE(before.st_mode)!=0o600):
        raise RuntimeError('tunnel_socket_not_owned')
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as probe:
        probe.settimeout(1.)
        try:probe.connect(str(path))
        except OSError as error:
            if error.errno!=errno.ECONNREFUSED:
                raise RuntimeError('tunnel_socket_state_unknown') from None
        else:raise RuntimeError('tunnel_socket_already_live')
    # Only an explicitly private, refused socket from this exact inode may be
    # removed. BindUnlink=no still prevents replacing a later competing bind.
    after=path.lstat()
    def identity(value):return value.st_dev,value.st_ino,value.st_ctime_ns,value.st_uid,value.st_mode
    if identity(before)!=identity(after):raise RuntimeError('tunnel_socket_changed')
    path.unlink()
    return 'stale_socket_removed'
