"""Caller-owned WLM identity and the native identification-setter projection.

The PC stores are a neo2-control design choice. They never import a phone's
identity. DJI Fly's BLE fast-connect and WLM paths use distinct UUID stores.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import secrets
import tempfile
from typing import Callable, TypeVar
import uuid

from .duml import DumlPacket, neo2_link_device_info, neo2_wifi_device_info


IDENTITY_STORE_FORMAT = "neo2-control/wlm-identity/v1"
BLE_IDENTITY_STORE_FORMAT = "neo2-control/ble-wifi-identity/v1"
NATIVE_IDENTIFICATION_LIMIT = 255
NATIVE_DEVICE_SN_PREFIX_LENGTH = 19


@dataclass(frozen=True, repr=False)
class WlmIdentity:
    """One client identification projected through the verified native setter.

    This client selects an ASCII profile. The SDK's 16-byte copy overlapping
    with four bytes at offset 15 retains 19 bytes, then writes NUL at index 19.
    Both identity-report formats must consume that same normalized snapshot.
    """

    identification_id: str

    def __post_init__(self) -> None:
        try:
            value = self.identification_id.encode("ascii")
        except UnicodeEncodeError as error:
            raise ValueError("client identification must be ASCII") from error
        if not 1 <= len(value) <= NATIVE_IDENTIFICATION_LIMIT or b"\0" in value:
            raise ValueError("client identification must contain 1..255 non-NUL ASCII bytes")

    @property
    def device_sn(self) -> str:
        return self.identification_id[:NATIVE_DEVICE_SN_PREFIX_LENGTH]

    @property
    def fingerprint(self) -> str:
        return hashlib.sha256(self.device_sn.encode("ascii")).hexdigest()

    @property
    def wlm_device_id(self) -> bytes:
        """Native own23byte WLM ID: APP/type4, index2, model0 and SN20."""
        return b'\x04\x02\0' + self.device_sn.encode('ascii').ljust(20,b'\0')

    @property
    def wlm_device_hash(self) -> int:
        """Native WLM routing hash of our normalized SN, not authentication."""
        value = 0
        for byte in self.device_sn.encode('ascii') + b'_0':
            value = (value * 256 + byte) % 0xfffffffb
        return value

    def metadata(self) -> dict[str, str | int]:
        return {
            "identity_sha256": self.fingerprint,
            "device_sn_length": len(self.device_sn),
            "normalization": "native-identification-setter-prefix19",
        }

    def link_report(self, sequence: int, *, physical_port: int) -> DumlPacket:
        return neo2_link_device_info(
            sequence, physical_port=physical_port, device_id=self.device_sn,
        )

    def wifi_report(self, sequence: int, *, physical_port: int) -> DumlPacket:
        return neo2_wifi_device_info(
            sequence, physical_port=physical_port, device_id=self.device_sn,
        )


@dataclass(frozen=True, repr=False)
class BleClientIdentity:
    """Own 32-byte identity for the observed BLE 07/45 exchange.

    DJI Fly's fast-connect UUID is stored separately from its WLM UUID.
    This PC store likewise leaves the existing WLM identity untouched.
    """

    identification_id: str

    def __post_init__(self) -> None:
        try:
            raw = self.identification_id.encode('ascii')
        except UnicodeEncodeError as error:
            raise ValueError('BLE client identity must be ASCII') from error
        if len(raw) != 32 or b'\0' in raw:
            raise ValueError('BLE client identity must contain exactly 32 non-NUL ASCII bytes')

    def __repr__(self) -> str:
        return 'BleClientIdentity(<private>)'

    def metadata(self) -> dict[str, str | int]:
        return {'identity_sha256': hashlib.sha256(self.identification_id.encode('ascii')).hexdigest(),
                'identification_length': 32, 'purpose': 'ble-wifi-setup'}


IdentityT = TypeVar('IdentityT', WlmIdentity, BleClientIdentity)


def _load_identity(path: Path, model: type[IdentityT], store_format: str) -> IdentityT:
    # Bound a local configuration read; never interpolate its private content
    # in an error or silently regenerate an established client identity.
    with path.open("rb") as stream:
        data = stream.read(4097)
    if len(data) > 4096:
        raise ValueError("client identity store is too large")
    try:
        value = json.loads(data)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("invalid client identity store") from error
    if (
        not isinstance(value, dict)
        or set(value) != {"format", "identification_id"}
        or value["format"] != store_format
        or not isinstance(value["identification_id"], str)
    ):
        raise ValueError("invalid client identity store format")
    return model(value["identification_id"])


def _load_or_create_identity(
    path: str | Path,
    model: type[IdentityT],
    store_format: str,
    generate: Callable[[], str],
) -> IdentityT:
    """Atomically establish a stable identity without replacing an existing one.

    A completely written private temporary file is published with link(),
    which cannot overwrite a concurrently established identity. Only the
    winning identity is returned, including to concurrent creators.
    """

    target = Path(path)
    try:
        return _load_identity(target, model, store_format)
    except FileNotFoundError:
        pass
    target.parent.mkdir(parents=True, exist_ok=True)
    candidate = generate()
    model(candidate)  # Validate completely before publishing the private record.
    payload = json.dumps({"format": store_format, "identification_id": candidate}) + "\n"
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=target.parent, prefix=".neo2-wlm-", mode="w", encoding="utf-8", delete=False,
        ) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, target)
        except FileExistsError:
            pass
        return _load_identity(target, model, store_format)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def validate_identity_path(path: str | Path, protected_paths=()) -> None:
    target = Path(path)
    for protected in protected_paths:
        other = Path(protected)
        if (target.resolve() == other.resolve()
            or (target.exists() and other.exists() and target.samefile(other))):
            raise ValueError('WLM identity must be distinct from credentials and session outputs')


def load_or_create_identity(path: str | Path, *, protected_paths=()) -> WlmIdentity:
    validate_identity_path(path, protected_paths)
    return _load_or_create_identity(path, WlmIdentity, IDENTITY_STORE_FORMAT,
                                    lambda: secrets.token_hex(10)[:NATIVE_DEVICE_SN_PREFIX_LENGTH])


def load_or_create_ble_identity(path: str | Path) -> BleClientIdentity:
    return _load_or_create_identity(path, BleClientIdentity, BLE_IDENTITY_STORE_FORMAT,
                                    lambda: str(uuid.uuid4())[:32])
