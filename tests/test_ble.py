import asyncio
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from neo2_control.ble import (
    observe_gatt_notifications,
    probe_gatt,
    probe_gatt_with_intervals,
    read_gatt_values,
)
from pathlib import Path
import tempfile


class FakeClient:
    def __init__(self, address, timeout, pair):
        self.address = address
        self.is_connected = False
        descriptor = SimpleNamespace(uuid="00002902-0000-1000-8000-00805F9B34FB", handle=4)
        characteristic = SimpleNamespace(
            uuid="0000FFF4-0000-1000-8000-00805F9B34FB",
            handle=3,
            properties=["notify"],
            descriptors=[descriptor],
        )
        self.services = [SimpleNamespace(uuid="0000FFF0-0000-1000-8000-00805F9B34FB", handle=1, characteristics=[characteristic])]

    async def __aenter__(self):
        self.is_connected = True
        return self

    async def __aexit__(self, exc_type, exc, tb):
        self.is_connected = False

    async def read_gatt_char(self, uuid):
        return bytearray.fromhex("010203")

    async def start_notify(self, uuid, callback):
        characteristic = SimpleNamespace(uuid=uuid, handle=9)
        callback(characteristic, bytearray.fromhex("aabbcc"))


class BleTests(unittest.TestCase):
    def test_gatt_probe_enumerates_metadata_only(self):
        with patch("neo2_control.ble._load_bleak_client", return_value=FakeClient):
            result = asyncio.run(probe_gatt("02:00:00:00:00:01"))
        self.assertTrue(result["connected"])
        service = result["services"][0]
        self.assertEqual(service["uuid"], "0000fff0-0000-1000-8000-00805f9b34fb")
        self.assertEqual(service["characteristics"][0]["properties"], ["notify"])

    def test_interval_experiment_always_restores_values(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            minimum = root / "conn_min_interval"
            maximum = root / "conn_max_interval"
            minimum.write_text("24", encoding="ascii")
            maximum.write_text("40", encoding="ascii")
            with patch("neo2_control.ble.probe_gatt_sync", side_effect=RuntimeError("probe failed")):
                with self.assertRaisesRegex(RuntimeError, "probe failed"):
                    probe_gatt_with_intervals(
                        "02:00:00:00:00:01",
                        minimum_units=12,
                        maximum_units=20,
                        debug_directory=root,
                    )
            self.assertEqual(minimum.read_text(encoding="ascii"), "24")
            self.assertEqual(maximum.read_text(encoding="ascii"), "40")

    def test_explicit_read_returns_lossless_hex(self):
        with patch("neo2_control.ble._load_bleak_client", return_value=FakeClient):
            result = asyncio.run(read_gatt_values("02:00:00:00:00:01", ["0000fff3-0000-1000-8000-00805f9b34fb"]))
        self.assertEqual(result["values"][0]["length"], 3)
        self.assertEqual(result["values"][0]["value_hex"], "010203")

    def test_notification_observer_preserves_payload_and_timing(self):
        with patch("neo2_control.ble._load_bleak_client", return_value=FakeClient):
            result = asyncio.run(
                observe_gatt_notifications(
                    "02:00:00:00:00:01",
                    ["0000fff3-0000-1000-8000-00805f9b34fb"],
                    duration=0.001,
                    timeout=1,
                )
            )
        self.assertEqual(result["events"][0]["value_hex"], "aabbcc")
        self.assertGreaterEqual(result["events"][0]["offset_ns"], 0)


if __name__ == "__main__":
    unittest.main()
