"""Read-only BLE discovery for DJI Neo 2 protocol identification."""

from __future__ import annotations

import asyncio
from contextlib import contextmanager
from dataclasses import dataclass, asdict
import os
from pathlib import Path
import time
from typing import Any


DJI_CANDIDATE_SERVICE = "0000fff0-0000-1000-8000-00805f9b34fb"


@dataclass(frozen=True)
class BleDevice:
    address: str
    name: str
    rssi: int | None
    service_uuids: list[str]
    manufacturer_data: dict[str, str]
    candidate: bool

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def _load_bleak():
    try:
        from bleak import BleakScanner
    except ImportError as error:
        raise RuntimeError("BLE support requires the 'bleak' package") from error
    return BleakScanner


def _load_bleak_client():
    try:
        from bleak import BleakClient
    except ImportError as error:
        raise RuntimeError("BLE support requires the 'bleak' package") from error
    return BleakClient


async def scan(duration: float = 10.0) -> list[BleDevice]:
    """Scan advertisements only; this function performs no GATT writes."""
    if duration <= 0 or duration > 300:
        raise ValueError("BLE scan duration must be between 0 and 300 seconds")
    scanner_type = _load_bleak()
    discovered = await scanner_type.discover(timeout=duration, return_adv=True)
    devices: list[BleDevice] = []
    for _, (device, advertisement) in discovered.items():
        name = device.name or advertisement.local_name or ""
        uuids = [value.lower() for value in (advertisement.service_uuids or [])]
        manufacturer = {
            f"0x{company:04x}": payload.hex()
            for company, payload in (advertisement.manufacturer_data or {}).items()
        }
        candidate = (
            DJI_CANDIDATE_SERVICE in uuids
            or "DJI" in name.upper()
            or "NEO" in name.upper()
        )
        devices.append(
            BleDevice(
                address=device.address,
                name=name,
                rssi=getattr(advertisement, "rssi", None),
                service_uuids=uuids,
                manufacturer_data=manufacturer,
                candidate=candidate,
            )
        )
    return sorted(devices, key=lambda item: (not item.candidate, -(item.rssi or -999)))


def scan_sync(duration: float = 10.0) -> list[BleDevice]:
    return asyncio.run(scan(duration))


async def probe_gatt(address: str, timeout: float = 20.0) -> dict[str, Any]:
    """Enumerate GATT metadata without app-requested custom value access.

    BlueZ may still enable the standard Service Changed indication and read the
    standard Generic Access device name as part of its own discovery workflow.
    """
    if timeout <= 0 or timeout > 120:
        raise ValueError("GATT timeout must be between 0 and 120 seconds")
    client_type = _load_bleak_client()
    services = []
    async with asyncio.timeout(timeout):
        async with client_type(address, timeout=timeout, pair=False) as client:
            for service in client.services:
                characteristics = []
                for characteristic in service.characteristics:
                    characteristics.append(
                        {
                            "uuid": characteristic.uuid.lower(),
                            "handle": characteristic.handle,
                            "properties": sorted(characteristic.properties),
                            "descriptors": [
                                {"uuid": descriptor.uuid.lower(), "handle": descriptor.handle}
                                for descriptor in characteristic.descriptors
                            ],
                        }
                    )
                services.append(
                    {
                        "uuid": service.uuid.lower(),
                        "handle": service.handle,
                        "characteristics": characteristics,
                    }
                )
            return {"address": address.upper(), "connected": client.is_connected, "services": services}


def probe_gatt_sync(address: str, timeout: float = 20.0) -> dict[str, Any]:
    return asyncio.run(probe_gatt(address, timeout))


def probe_gatt_with_intervals(
    address: str,
    *,
    minimum_units: int,
    maximum_units: int,
    timeout: float = 20.0,
    debug_directory: str | Path = "/sys/kernel/debug/bluetooth/hci0",
) -> dict[str, Any]:
    """Temporarily match advertised BLE intervals and restore them unconditionally."""
    with temporary_connection_intervals(
        minimum_units=minimum_units,
        maximum_units=maximum_units,
        debug_directory=debug_directory,
    ) as experiment:
        result = probe_gatt_sync(address, timeout)
        result["interval_experiment"] = experiment
        return result


@contextmanager
def temporary_connection_intervals(
    *,
    minimum_units: int,
    maximum_units: int,
    debug_directory: str | Path = "/sys/kernel/debug/bluetooth/hci0",
):
    if not 6 <= minimum_units <= maximum_units <= 3200:
        raise ValueError("BLE interval units must satisfy 6 <= minimum <= maximum <= 3200")
    directory = Path(debug_directory)
    minimum_path = directory / "conn_min_interval"
    maximum_path = directory / "conn_max_interval"
    if not minimum_path.is_file() or not maximum_path.is_file():
        raise RuntimeError("Bluetooth connection interval controls are unavailable")
    if hasattr(os, "geteuid") and os.geteuid() != 0 and str(directory).startswith("/sys/"):
        raise PermissionError("temporary Bluetooth interval control requires root")
    previous_minimum = minimum_path.read_text(encoding="ascii").strip()
    previous_maximum = maximum_path.read_text(encoding="ascii").strip()
    try:
        minimum_path.write_text(str(minimum_units), encoding="ascii")
        maximum_path.write_text(str(maximum_units), encoding="ascii")
        yield {
            "previous_minimum_units": int(previous_minimum),
            "previous_maximum_units": int(previous_maximum),
            "test_minimum_units": minimum_units,
            "test_maximum_units": maximum_units,
        }
    finally:
        maximum_path.write_text(previous_maximum, encoding="ascii")
        minimum_path.write_text(previous_minimum, encoding="ascii")


async def read_gatt_values(address: str, uuids: list[str], timeout: float = 20.0) -> dict[str, Any]:
    """Explicitly read only the requested characteristics and return lossless hex."""
    if not uuids:
        raise ValueError("at least one characteristic UUID is required")
    if timeout <= 0 or timeout > 120:
        raise ValueError("GATT timeout must be between 0 and 120 seconds")
    client_type = _load_bleak_client()
    values = []
    async with asyncio.timeout(timeout):
        async with client_type(address, timeout=timeout, pair=False) as client:
            for uuid in uuids:
                raw = bytes(await client.read_gatt_char(uuid))
                values.append({"uuid": uuid.lower(), "length": len(raw), "value_hex": raw.hex()})
            return {"address": address.upper(), "connected": client.is_connected, "values": values}


def read_gatt_values_with_intervals(
    address: str,
    uuids: list[str],
    *,
    minimum_units: int,
    maximum_units: int,
    timeout: float = 20.0,
    debug_directory: str | Path = "/sys/kernel/debug/bluetooth/hci0",
) -> dict[str, Any]:
    with temporary_connection_intervals(
        minimum_units=minimum_units,
        maximum_units=maximum_units,
        debug_directory=debug_directory,
    ) as experiment:
        result = asyncio.run(read_gatt_values(address, uuids, timeout))
        result["interval_experiment"] = experiment
        return result


async def observe_gatt_notifications(
    address: str,
    uuids: list[str],
    *,
    duration: float = 10.0,
    timeout: float = 30.0,
) -> dict[str, Any]:
    """Enable notifications only; never write an application payload."""
    if not uuids:
        raise ValueError("at least one characteristic UUID is required")
    if duration <= 0 or duration > 300:
        raise ValueError("notification duration must be between 0 and 300 seconds")
    if timeout <= duration or timeout > 360:
        raise ValueError("timeout must be greater than duration and at most 360 seconds")
    client_type = _load_bleak_client()
    events: list[dict[str, Any]] = []
    started_ns = time.monotonic_ns()

    def notification(characteristic: Any, data: bytearray) -> None:
        raw = bytes(data)
        events.append(
            {
                "offset_ns": time.monotonic_ns() - started_ns,
                "uuid": characteristic.uuid.lower(),
                "handle": characteristic.handle,
                "length": len(raw),
                "value_hex": raw.hex(),
            }
        )

    async with asyncio.timeout(timeout):
        async with client_type(address, timeout=timeout, pair=False) as client:
            for uuid in uuids:
                await client.start_notify(uuid, notification)
            await asyncio.sleep(duration)
            return {
                "address": address.upper(),
                "connected": client.is_connected,
                "duration_seconds": duration,
                "subscribed_uuids": [uuid.lower() for uuid in uuids],
                "events": events,
            }


def observe_gatt_notifications_with_intervals(
    address: str,
    uuids: list[str],
    *,
    duration: float,
    minimum_units: int,
    maximum_units: int,
    timeout: float = 30.0,
    debug_directory: str | Path = "/sys/kernel/debug/bluetooth/hci0",
) -> dict[str, Any]:
    with temporary_connection_intervals(
        minimum_units=minimum_units,
        maximum_units=maximum_units,
        debug_directory=debug_directory,
    ) as experiment:
        result = asyncio.run(
            observe_gatt_notifications(
                address,
                uuids,
                duration=duration,
                timeout=timeout,
            )
        )
        result["interval_experiment"] = experiment
        return result
