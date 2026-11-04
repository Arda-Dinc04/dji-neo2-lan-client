"""Real Unix socket ownership/liveness guards, without SSH or aircraft."""
import os
from pathlib import Path
import socket

import pytest

from neo2_control.tunnel_socket import prepare_socket


def prepare(path):return prepare_socket(path,expected_uid=os.getuid())


def test_new_private_directory_and_absent_endpoint(tmp_path):
    path=tmp_path/'private'
    assert prepare(path)=='socket_absent'
    assert path.stat().st_mode&0o777==0o700
    assert not (path/'flight.sock').exists()


def test_refused_owned_socket_is_the_only_removed_artifact(tmp_path):
    root=tmp_path/'private';root.mkdir(mode=0o700)
    path=root/'flight.sock'
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as endpoint:
        endpoint.bind(str(path));path.chmod(0o600)
    assert prepare(root)=='stale_socket_removed'
    assert not path.exists()


def test_live_listener_is_never_removed(tmp_path):
    root=tmp_path/'private';root.mkdir(mode=0o700)
    path=root/'flight.sock'
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as endpoint:
        endpoint.bind(str(path));path.chmod(0o600);endpoint.listen(1)
        before=path.stat().st_ino
        with pytest.raises(RuntimeError,match='already_live'):prepare(root)
        assert path.stat().st_ino==before
        connection,_=endpoint.accept();connection.close()


@pytest.mark.parametrize('kind',['file','symlink','public_socket','foreign_owner','public_directory'])
def test_unknown_artifacts_and_public_directories_are_preserved(tmp_path,kind):
    root=tmp_path/'private';root.mkdir(mode=0o700)
    path=root/'flight.sock'
    if kind=='file':path.write_text('preserve')
    elif kind=='symlink':path.symlink_to(tmp_path/'other')
    else:
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as endpoint:endpoint.bind(str(path))
        path.chmod(0o666 if kind=='public_socket' else 0o600)
        if kind=='public_directory':root.chmod(0o755)
    before=path.lstat()
    with pytest.raises(RuntimeError):
        prepare_socket(root,expected_uid=os.getuid()+1 if kind=='foreign_owner' else os.getuid())
    assert path.lstat().st_ino==before.st_ino


def test_directory_symlink_is_refused(tmp_path):
    target=tmp_path/'real';target.mkdir(mode=0o700)
    alias=tmp_path/'alias';alias.symlink_to(target,target_is_directory=True)
    with pytest.raises(RuntimeError,match='directory_invalid'):prepare(alias)
    assert alias.is_symlink()


def test_replaced_inode_during_probe_is_preserved(tmp_path,monkeypatch):
    root=tmp_path/'private';root.mkdir(mode=0o700);path=root/'flight.sock'
    original=socket.socket
    with original(socket.AF_UNIX,socket.SOCK_STREAM) as endpoint:endpoint.bind(str(path))
    path.chmod(0o600)
    class RacingSocket:
        def __init__(self,*args):self.inner=original(*args)
        def __enter__(self):return self
        def __exit__(self,*_):self.inner.close()
        def settimeout(self,value):self.inner.settimeout(value)
        def connect(self,address):
            try:self.inner.connect(address)
            except ConnectionRefusedError:
                path.unlink()
                with original(socket.AF_UNIX,socket.SOCK_STREAM) as replacement:replacement.bind(str(path))
                path.chmod(0o600)
                raise
    monkeypatch.setattr(socket,'socket',RacingSocket)
    with pytest.raises(RuntimeError,match='socket_changed'):prepare(root)
    assert path.is_socket()
