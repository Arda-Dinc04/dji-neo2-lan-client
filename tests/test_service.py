import unittest

from neo2_control.service import ServiceState, bounded_float, bounded_int, build_handler
from neo2_control.udp_probe import run_probe
from pathlib import Path
import tempfile
from http.server import ThreadingHTTPServer
import json
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen


class ServiceTests(unittest.TestCase):
    def test_bounds(self):
        self.assertEqual(bounded_float("8", "duration", 0.1, 300), 8.0)
        self.assertEqual(bounded_int("9003", "port", 1, 65535), 9003)
        with self.assertRaises(ValueError):
            bounded_float(0, "duration", 0.1, 300)
        with self.assertRaises(ValueError):
            bounded_int(70000, "port", 1, 65535)

    def test_action_allowlist_rejects_motor_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            state = ServiceState(Path(directory), Path(directory), False)
            with self.assertRaisesRegex(ValueError, "not allowed"):
                state.submit("takeoff", {})
            snapshot = state.snapshot()
            self.assertFalse(snapshot["motor_commands_present"])
            self.assertNotIn("takeoff", snapshot["allowed_actions"])

    def test_idle_window_cannot_be_requested_without_session_syn(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "requires the session SYN"):
                run_probe(
                    capture_path=Path(directory) / "unused.jsonl",
                    send_idle_window=True,
                )

    def test_heartbeat_cannot_be_requested_without_session_syn(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "requires the session SYN"):
                run_probe(
                    capture_path=Path(directory) / "unused.jsonl",
                    send_heartbeat=True,
                )

    def test_application_probes_are_single_and_require_session_syn(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / "unused.jsonl"
            with self.assertRaisesRegex(ValueError, "requires the session SYN"):
                run_probe(
                    capture_path=capture,
                    send_wifi_device_info=True,
                )
            with self.assertRaisesRegex(ValueError, "only one"):
                run_probe(
                    capture_path=capture,
                    send_syn=True,
                    send_heartbeat=True,
                    send_wifi_device_info=True,
                )
            with self.assertRaisesRegex(ValueError, "requires the session SYN"):
                run_probe(
                    capture_path=capture,
                    send_wifi_then_heartbeat=True,
                )
            with self.assertRaisesRegex(ValueError, "exclusive"):
                run_probe(
                    capture_path=capture,
                    send_syn=True,
                    send_heartbeat=True,
                    send_wifi_then_heartbeat=True,
                )

    def test_static_capability_requires_the_stateful_sequence(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / "unused.jsonl"
            with self.assertRaisesRegex(ValueError, "extends the stateful"):
                run_probe(
                    capture_path=capture,
                    send_syn=True,
                    send_static_capability=True,
                )
            with self.assertRaisesRegex(ValueError, "extends the stateful"):
                run_probe(
                    capture_path=capture,
                    send_syn=True,
                    send_heartbeat=True,
                    send_static_capability=True,
                )

    def test_idle_window_bounds_are_enforced_before_network_access(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / "unused.jsonl"
            with self.assertRaisesRegex(ValueError, "count"):
                run_probe(
                    capture_path=capture,
                    send_syn=True,
                    send_idle_window=True,
                    idle_window_count=101,
                )
            with self.assertRaisesRegex(ValueError, "interval"):
                run_probe(
                    capture_path=capture,
                    send_syn=True,
                    send_idle_window=True,
                    idle_window_interval=0.01,
                )
            with self.assertRaisesRegex(ValueError, "mode"):
                run_probe(
                    capture_path=capture,
                    send_syn=True,
                    send_idle_window=True,
                    idle_window_mode="unknown",
                )

    def test_http_api_requires_auth_and_rejects_takeoff(self):
        with tempfile.TemporaryDirectory() as directory:
            state = ServiceState(Path(directory), Path(directory), False)
            server = ThreadingHTTPServer(("127.0.0.1", 0), build_handler(state, "x" * 32))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            base = f"http://127.0.0.1:{server.server_port}"
            try:
                with self.assertRaises(HTTPError) as unauthorized:
                    urlopen(base + "/v1/status", timeout=2)
                self.assertEqual(unauthorized.exception.code, 401)

                status_request = Request(base + "/v1/status", headers={"Authorization": "Bearer " + "x" * 32})
                with urlopen(status_request, timeout=2) as response:
                    status = json.load(response)
                self.assertFalse(status["motor_commands_present"])

                body = json.dumps({"action": "takeoff", "parameters": {}}).encode()
                command_request = Request(
                    base + "/v1/commands",
                    data=body,
                    method="POST",
                    headers={"Authorization": "Bearer " + "x" * 32, "Content-Type": "application/json"},
                )
                with self.assertRaises(HTTPError) as rejected:
                    urlopen(command_request, timeout=2)
                self.assertEqual(rejected.exception.code, 400)
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
