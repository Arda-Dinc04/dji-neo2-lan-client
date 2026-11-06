"""Local archive fixtures only; never download, run MediaMTX or access a drone."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile

import pytest

SPEC = importlib.util.spec_from_file_location('install_mediamtx',
    Path(__file__).resolve().parents[1] / 'tools/install_mediamtx.py')
installer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(installer)


def archive_bytes(extra=(), *, omit=()):
    output = io.BytesIO()
    with tarfile.open(fileobj=output, mode='w:gz') as package:
        for name, body in [('mediamtx', b'synthetic binary; never executed'),
                           ('LICENSE', b'synthetic license'), ('mediamtx.yml', b'rtsp: true')]:
            if name in omit:
                continue
            member = tarfile.TarInfo(name)
            member.size = len(body)
            package.addfile(member, io.BytesIO(body))
        for member in extra:
            package.addfile(member, io.BytesIO(b'x' * member.size) if member.isfile() else None)
    return output.getvalue()


def downloads(monkeypatch, archive):
    archive_hash = hashlib.sha256(archive).hexdigest()
    checksums = f'{archive_hash} *{installer.ASSET}\n'.encode()
    monkeypatch.setattr(installer, 'ARCHIVE_SHA256', archive_hash)
    monkeypatch.setattr(installer, 'CHECKSUM_SHA256', hashlib.sha256(checksums).hexdigest())
    monkeypatch.setattr(installer.platform, 'system', lambda: 'Darwin')
    monkeypatch.setattr(installer.platform, 'machine', lambda: 'arm64')
    calls = []

    def fetch(url, path, maximum):
        calls.append(url)
        path.write_bytes(checksums if url == installer.CHECKSUM_URL else archive)
    return fetch, calls


def test_default_plan_has_no_network_or_files(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(installer, 'ROOT', tmp_path)
    monkeypatch.setattr(installer, 'install', lambda *a, **k: pytest.fail('installation requested'))
    assert installer.main([]) == 0
    plan = json.loads(capsys.readouterr().out)
    assert plan['mode'] == 'plan' and plan['network_requested'] is False
    assert plan['starts_server'] is False
    assert not list(tmp_path.iterdir())


def test_verified_install_copies_only_binary_and_license(monkeypatch, tmp_path):
    fetch, calls = downloads(monkeypatch, archive_bytes())
    target = tmp_path / 'build' / 'mediamtx'
    binary = installer.install(target, fetch=fetch)
    assert binary == target / 'mediamtx'
    assert sorted(p.name for p in target.iterdir()) == ['LICENSE', 'mediamtx']
    assert binary.stat().st_mode & 0o777 == 0o755
    assert (target / 'LICENSE').stat().st_mode & 0o777 == 0o644
    assert calls == [installer.CHECKSUM_URL, installer.ARCHIVE_URL]
    assert sorted(p.name for p in target.parent.iterdir()) == ['mediamtx']


@pytest.mark.parametrize('corruption', ['archive', 'checksums', 'checksum_entry'])
def test_integrity_failure_leaves_no_install(monkeypatch, tmp_path, corruption):
    fetch, calls = downloads(monkeypatch, archive_bytes())
    if corruption == 'archive':
        monkeypatch.setattr(installer, 'ARCHIVE_SHA256', '0' * 64)
        expected = 'official_archive_checksum_missing_or_mismatched'
    elif corruption == 'checksums':
        monkeypatch.setattr(installer, 'CHECKSUM_SHA256', '0' * 64)
        expected = 'official_checksum_file_digest_mismatch'
    else:
        original_fetch = fetch
        def fetch(url, path, maximum):
            original_fetch(url, path, maximum)
            if url == installer.ARCHIVE_URL:
                path.write_bytes(path.read_bytes() + b'corruption')
        expected = 'archive_digest_mismatch'
    target = tmp_path / 'mediamtx'
    with pytest.raises(installer.InstallError, match=expected):
        installer.install(target, fetch=fetch)
    assert not target.exists() and not list(tmp_path.iterdir())


@pytest.mark.parametrize('name,kind', [('../escape', 'file'), ('/tmp/escape', 'file'),
    ('nested/../escape', 'file'), ('nested\\escape', 'file'), ('mediamtx', 'file'),
    ('link', 'symlink'), ('hardlink', 'hardlink'), ('fifo', 'fifo')])
def test_unsafe_archive_is_rejected_before_copy(tmp_path, name, kind):
    member = tarfile.TarInfo(name)
    member.type = {'file': tarfile.REGTYPE, 'symlink': tarfile.SYMTYPE,
                   'hardlink': tarfile.LNKTYPE, 'fifo': tarfile.FIFOTYPE}[kind]
    member.linkname = '/tmp/escape'
    archive = tmp_path / 'package.tar.gz'
    archive.write_bytes(archive_bytes([member]))
    target = tmp_path / 'selected'
    with pytest.raises(installer.InstallError):
        installer.extract_selected(archive, target)
    assert not target.exists()


def test_archive_requires_license(tmp_path):
    archive = tmp_path / 'package.tar.gz'
    archive.write_bytes(archive_bytes(omit={'LICENSE'}))
    with pytest.raises(installer.InstallError, match='requires_binary_and_license'):
        installer.extract_selected(archive, tmp_path / 'selected')


def test_existing_target_and_wrong_platform_never_download(monkeypatch, tmp_path):
    fetch, calls = downloads(monkeypatch, archive_bytes())
    target = tmp_path / 'existing'
    target.mkdir()
    with pytest.raises(installer.InstallError, match='already_exists'):
        installer.install(target, fetch=fetch)
    monkeypatch.setattr(installer.platform, 'system', lambda: 'Linux')
    with pytest.raises(installer.InstallError, match='requires_apple_silicon_macos'):
        installer.install(tmp_path / 'new', fetch=fetch)
    assert not calls


def test_symlink_parent_rejected_before_download(monkeypatch, tmp_path):
    fetch, calls = downloads(monkeypatch, archive_bytes())
    real = tmp_path / 'real'
    real.mkdir()
    link = tmp_path / 'link'
    link.symlink_to(real, target_is_directory=True)
    with pytest.raises(installer.InstallError, match='must_not_be_symlink'):
        installer.install(link / 'mediamtx', fetch=fetch)
    assert not calls and not list(real.iterdir())


def test_download_is_bounded(monkeypatch, tmp_path):
    class Response(io.BytesIO):
        def geturl(self):
            return 'https://release-assets.githubusercontent.com/fixture'
    monkeypatch.setattr(installer.urllib.request, 'urlopen', lambda *a, **k: Response(b'12345'))
    with pytest.raises(installer.InstallError, match='size_limit_exceeded'):
        installer.download(installer.ARCHIVE_URL, tmp_path / 'archive', 4)
