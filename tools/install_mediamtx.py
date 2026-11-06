#!/usr/bin/env python3
"""Install a verified, pinned MediaMTX binary locally; never launch it."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import sys
import tarfile
import tempfile
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
VERSION = 'v1.21.3'
ASSET = f'mediamtx_{VERSION}_darwin_arm64.tar.gz'
RELEASE = f'https://github.com/bluenviron/mediamtx/releases/tag/{VERSION}'
BASE_URL = f'https://github.com/bluenviron/mediamtx/releases/download/{VERSION}'
ARCHIVE_URL = f'{BASE_URL}/{ASSET}'
CHECKSUM_URL = f'{BASE_URL}/checksums.sha256'
ARCHIVE_SHA256 = 'bc1a0762fd18ffb413bcd81dcb9a072ac159fbd1f27e79ac6f14598aaac5ed3f'
CHECKSUM_SHA256 = '7df899490a0941533d55eab3c31048d365f55c9ed519a962394118d609183431'
MAX_ARCHIVE_BYTES = 128 * 1024 * 1024
MAX_CHECKSUM_BYTES = 1024 * 1024
MAX_LICENSE_BYTES = 1024 * 1024


class InstallError(Exception):
    pass


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def download(url, destination, maximum):
    request = urllib.request.Request(url, headers={'User-Agent': 'neo2-local-mediamtx-installer'})
    with urllib.request.urlopen(request, timeout=30) as response, destination.open('xb') as output:
        if urllib.parse.urlsplit(response.geturl()).scheme != 'https':
            raise InstallError('download_redirect_requires_https')
        total = 0
        while chunk := response.read(1024 * 1024):
            total += len(chunk)
            if total > maximum:
                raise InstallError('download_size_limit_exceeded')
            output.write(chunk)


def verify_downloads(checksums, archive):
    if digest(checksums) != CHECKSUM_SHA256:
        raise InstallError('official_checksum_file_digest_mismatch')
    matches = []
    for line in checksums.read_text(encoding='ascii').splitlines():
        match = re.fullmatch(r'([a-fA-F0-9]{64}) [ *](.+)', line)
        if match and match[2] == ASSET:
            matches.append(match[1].lower())
    if matches != [ARCHIVE_SHA256]:
        raise InstallError('official_archive_checksum_missing_or_mismatched')
    if digest(archive) != ARCHIVE_SHA256:
        raise InstallError('archive_digest_mismatch')


def extract_selected(archive, destination):
    """Inspect every member, then copy two regular files without tar extraction."""
    with tarfile.open(archive, mode='r:gz') as package:
        selected, names, expanded = {}, set(), 0
        for member in package:
            name = member.name
            parts = PurePosixPath(name).parts
            if (not parts or name.startswith('/') or '\\' in name or
                    any(part in ('.', '..') for part in name.split('/')) or name in names):
                raise InstallError('unsafe_or_duplicate_archive_path')
            names.add(name)
            expanded += member.size
            if len(names) > 64 or expanded > MAX_ARCHIVE_BYTES:
                raise InstallError('archive_expansion_limit_exceeded')
            if not (member.isfile() or member.isdir()):
                raise InstallError('archive_links_or_special_files_rejected')
            if member.size < 0 or member.size > MAX_ARCHIVE_BYTES:
                raise InstallError('archive_member_size_limit_exceeded')
            if name in ('mediamtx', 'LICENSE'):
                if not member.isfile() or member.size == 0:
                    raise InstallError('required_archive_member_not_regular_file')
                if name == 'LICENSE' and member.size > MAX_LICENSE_BYTES:
                    raise InstallError('license_size_limit_exceeded')
                selected[name] = member
        if set(selected) != {'mediamtx', 'LICENSE'}:
            raise InstallError('archive_requires_binary_and_license')
        destination.mkdir()
        for name, member in selected.items():
            with package.extractfile(member) as source, (destination / name).open('xb') as output:
                shutil.copyfileobj(source, output, length=1024 * 1024)
            (destination / name).chmod(0o755 if name == 'mediamtx' else 0o644)


def install(target, *, fetch=download):
    target = Path(target)
    if platform.system() != 'Darwin' or platform.machine().lower() not in ('arm64', 'aarch64'):
        raise InstallError('this_pinned_binary_requires_apple_silicon_macos')
    if target.exists() or target.is_symlink():
        raise InstallError('installation_target_already_exists')
    if target.parent.is_symlink():
        raise InstallError('installation_parent_must_not_be_symlink')
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.mediamtx-install-', dir=target.parent) as temporary:
        stage = Path(temporary)
        checksums, archive = stage / 'checksums.sha256', stage / ASSET
        fetch(CHECKSUM_URL, checksums, MAX_CHECKSUM_BYTES)
        fetch(ARCHIVE_URL, archive, MAX_ARCHIVE_BYTES)
        if checksums.stat().st_size > MAX_CHECKSUM_BYTES or archive.stat().st_size > MAX_ARCHIVE_BYTES:
            raise InstallError('download_size_limit_exceeded')
        verify_downloads(checksums, archive)
        prepared = stage / 'verified'
        extract_selected(archive, prepared)
        if target.exists() or target.is_symlink():
            raise InstallError('installation_target_already_exists')
        prepared.rename(target)
    return target / 'mediamtx'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', action='store_true', help='download and install; does not start MediaMTX')
    args = parser.parse_args(argv)
    target = ROOT / 'build' / 'mediamtx'
    plan = {'version': VERSION, 'release': RELEASE, 'archive_url': ARCHIVE_URL,
            'checksums_url': CHECKSUM_URL, 'archive_sha256': ARCHIVE_SHA256,
            'checksums_sha256': CHECKSUM_SHA256, 'target': str(target),
            'files': ['mediamtx', 'LICENSE'], 'starts_server': False,
            'network_requested': args.install}
    if not args.install:
        print(json.dumps({'mode': 'plan', **plan}, indent=2))
        return 0
    try:
        binary = install(target)
    except (InstallError, OSError, ValueError, tarfile.TarError) as error:
        print(f'MediaMTX install failed: {error}', file=sys.stderr)
        return 1
    print(json.dumps({'mode': 'installed', 'binary': str(binary), **plan}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
