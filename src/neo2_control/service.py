"""Loopback diagnostic API intended for access through the Scaleway SSH tunnel."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import hmac
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import signal
import threading
from typing import Any
from urllib.parse import urlparse

from .ble import scan_sync
from .controller import ControllerMode, ControlSupervisor, IntentRejected
from .controller_runtime import ControllerRuntime
from .control_owner import ProcessControllerRuntime
from .observation import ManagedObservation, ObservationConfig, ObservationError
from .report import generate_report
from .udp_probe import run_probe


ALLOWED_ACTIONS = frozenset(("ble_scan", "udp_probe"))


@dataclass
class Job:
    id: str
    action: str
    state: str
    created_at: str
    parameters: dict[str, Any]
    result: Any = None
    error: str = ""


@dataclass
class ServiceState:
    capture_dir: Path
    report_dir: Path
    allow_active_syn: bool
    jobs: dict[str, Job] = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock)
    controller: ControlSupervisor | None = None
    observation: ManagedObservation | None = None

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            jobs = [asdict(job) for job in reversed(list(self.jobs.values())[-50:])]
        return {
            "service": "neo2-control",
            "mode": "simulate" if self.controller and self.controller.mode is ControllerMode.SIMULATE else "diagnostic-only",
            "allowed_actions": [] if self.controller and self.controller.mode is ControllerMode.SIMULATE else sorted(ALLOWED_ACTIONS),
            "active_syn_enabled": self.allow_active_syn and not (self.controller and self.controller.mode is ControllerMode.SIMULATE),
            "motor_commands_present": False,
            'observation_enabled': self.observation is not None and self.allow_active_syn,
            "drone": self.drone_status(),
            "jobs": jobs,
        }

    def submit(self, action: str, parameters: dict[str, Any]) -> Job:
        if self.controller and self.controller.mode is ControllerMode.SIMULATE:
            raise ValueError('radio diagnostic jobs are disabled in simulate mode')
        if action not in ALLOWED_ACTIONS:
            raise ValueError("action is not allowed")
        job = Job(
            id=secrets.token_hex(12),
            action=action,
            state="pending",
            created_at=datetime.now(timezone.utc).isoformat(),
            parameters=parameters,
        )
        with self.lock:
            if action == 'udp_probe' and self.observation and self.observation.owns_link:
                raise ValueError('managed observation owns the UDP link')
            self.jobs[job.id] = job
        threading.Thread(target=self._execute, args=(job,), daemon=True).start()
        return job

    def open_observation(self, parameters):
        if (not self.observation or not self.allow_active_syn
            or not self.controller or self.controller.mode is not ControllerMode.OBSERVE):
            raise ObservationError('observation_not_enabled')
        with self.lock:
            if any(job.action == 'udp_probe' and job.state in ('pending', 'running') for job in self.jobs.values()):
                raise ObservationError('udp_probe_owns_link')
            return self.observation.open(parameters)

    def drone_status(self):
        if self.controller:
            return self.controller.status()
        return {'mode': 'observe', 'state': 'disconnected', 'live_flight_enabled': False,
                'synthetic': False, 'connected': False, 'telemetry': None}

    def _execute(self, job: Job) -> None:
        with self.lock:
            job.state = "running"
        try:
            if job.action == "ble_scan":
                duration = bounded_float(job.parameters.get("duration", 10), "duration", 0.1, 300)
                result = {"devices": [item.as_dict() for item in scan_sync(duration)]}
            else:
                duration = bounded_float(job.parameters.get("duration", 8), "duration", 0.1, 300)
                port = bounded_int(job.parameters.get("port", 9003), "port", 1, 65535)
                host = str(job.parameters.get("host", "192.168.2.1"))
                if host != "192.168.2.1":
                    raise ValueError("remote API restricts UDP probes to the Neo 2 candidate address 192.168.2.1")
                send_syn = job.parameters.get("send_syn", False) is True
                send_heartbeat = job.parameters.get("send_heartbeat", False) is True
                send_wifi_device_info = (
                    job.parameters.get("send_wifi_device_info", False) is True
                )
                send_wifi_then_heartbeat = (
                    job.parameters.get("send_wifi_then_heartbeat", False) is True
                )
                send_static_capability = (
                    job.parameters.get("send_static_capability", False) is True
                )
                send_idle_window = job.parameters.get("send_idle_window", False) is True
                idle_window_count = bounded_int(
                    job.parameters.get("idle_window_count", 1),
                    "idle_window_count",
                    1,
                    100,
                )
                idle_window_interval = bounded_float(
                    job.parameters.get("idle_window_interval", 0.1),
                    "idle_window_interval",
                    0.05,
                    2,
                )
                idle_window_mode = str(
                    job.parameters.get("idle_window_mode", "periodic")
                )
                interface = str(job.parameters.get("interface", "wlan1"))
                if (
                    send_syn
                    or send_heartbeat
                    or send_wifi_device_info
                    or send_wifi_then_heartbeat
                    or send_static_capability
                    or send_idle_window
                ) and not self.allow_active_syn:
                    raise ValueError("active transport probes are disabled by service configuration")
                capture = self.capture_dir / f"udp-{job.id}.jsonl"
                probe = run_probe(
                    capture_path=capture,
                    host=host,
                    port=port,
                    duration=duration,
                    send_syn=send_syn,
                    send_heartbeat=send_heartbeat,
                    send_wifi_device_info=send_wifi_device_info,
                    send_wifi_then_heartbeat=send_wifi_then_heartbeat,
                    send_static_capability=send_static_capability,
                    send_idle_window=send_idle_window,
                    idle_window_count=idle_window_count,
                    idle_window_interval=idle_window_interval,
                    idle_window_mode=idle_window_mode,
                    interface=interface,
                )
                report = generate_report(capture, self.report_dir / f"udp-{job.id}.html")
                result = {
                    **probe.as_dict(),
                    "capture_url": f"/v1/captures/{capture.name}",
                    "report_path": str(report),
                    "report_url": f"/v1/reports/{report.name}",
                }
        except Exception as error:  # job boundary: preserve the error for remote inspection
            with self.lock:
                job.state = "failed"
                job.error = str(error)
            return
        with self.lock:
            job.state = "completed"
            job.result = result


def bounded_float(value: Any, name: str, minimum: float, maximum: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if not minimum <= parsed <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return parsed


def bounded_int(value: Any, name: str, minimum: int, maximum: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be an integer") from error
    if not minimum <= parsed <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return parsed


def build_handler(state: ServiceState, token: str):
    class Handler(BaseHTTPRequestHandler):
        server_version = "neo2-control/0.1"

        def _authorized(self) -> bool:
            supplied = self.headers.get("Authorization", "")
            expected = "Bearer " + token
            return bool(token) and hmac.compare_digest(supplied.encode(), expected.encode())

        def _json(self, status: int, payload: Any) -> None:
            body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def _auth_or_reject(self) -> bool:
            if self._authorized():
                return True
            self._json(HTTPStatus.UNAUTHORIZED, {"error": "unauthorized"})
            return False

        def do_GET(self) -> None:
            if not self._auth_or_reject():
                return
            path = urlparse(self.path).path
            if path == "/v1/status":
                self._json(HTTPStatus.OK, state.snapshot())
                return
            if path in ('/v1/drone/status', '/v1/drone/telemetry'):
                result = state.drone_status()
                if path.endswith('/telemetry'):
                    result = {key: result[key] for key in ('mode', 'synthetic', 'telemetry', 'wifi_control', 'battery')
                              if key in result}
                self._json(HTTPStatus.OK, result)
                return
            if path.startswith("/v1/reports/"):
                self._artifact(path.removeprefix("/v1/reports/"), state.report_dir, "text/html; charset=utf-8")
                return
            if path.startswith("/v1/captures/"):
                self._artifact(path.removeprefix("/v1/captures/"), state.capture_dir, "application/x-ndjson")
                return
            self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        def _artifact(self, name: str, directory: Path, content_type: str) -> None:
            if not name or name != Path(name).name:
                self._json(HTTPStatus.BAD_REQUEST, {"error": "invalid artifact name"})
                return
            target = directory / name
            if not target.is_file():
                self._json(HTTPStatus.NOT_FOUND, {"error": "artifact not found"})
                return
            body = target.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self) -> None:
            if not self._auth_or_reject():
                return
            path = urlparse(self.path).path
            simulation_intent = path == '/v1/drone/simulation/intent'
            observation_intent = path in ('/v1/drone/session/open', '/v1/drone/session/close')
            if path != "/v1/commands" and not simulation_intent and not observation_intent:
                self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
                return
            if simulation_intent and (not state.controller or state.controller.mode is not ControllerMode.SIMULATE):
                self._json(HTTPStatus.FORBIDDEN, {'error': 'simulate_mode_not_enabled'})
                return
            if observation_intent and (not state.observation or not state.allow_active_syn
                or not state.controller or state.controller.mode is not ControllerMode.OBSERVE):
                self._json(HTTPStatus.FORBIDDEN, {'error': 'observation_not_enabled'})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 16_384:
                    raise ValueError("invalid request size")
                def reject_constant(value):
                    raise ValueError('nonfinite JSON numbers are not accepted')
                payload = json.loads(self.rfile.read(length), parse_constant=reject_constant)
                if not isinstance(payload, dict):
                    raise ValueError("JSON object required")
                if observation_intent:
                    try:
                        if path.endswith('/open'):
                            result = state.open_observation(payload)
                        else:
                            if payload:
                                raise ValueError('session close takes no parameters')
                            result = state.observation.close()
                    except ObservationError as error:
                        self._json(HTTPStatus.CONFLICT, {'error': str(error)})
                        return
                    self._json(HTTPStatus.ACCEPTED, {'accepted': True, 'mode': 'observe', **result})
                    return
                action = payload.get("action")
                parameters = payload.get("parameters", {})
                if not isinstance(action, str) or not isinstance(parameters, dict):
                    raise ValueError("action and parameters are required")
                if simulation_intent:
                    if set(payload) - {'action', 'parameters'}:
                        raise ValueError('unknown request fields')
                    try:
                        result = state.controller.handle(action, parameters)
                    except IntentRejected as error:
                        self._json(HTTPStatus.CONFLICT, {'error': str(error)})
                        return
                    self._json(HTTPStatus.OK, result)
                    return
                job = state.submit(action, parameters)
            except (ValueError, json.JSONDecodeError) as error:
                self._json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
                return
            self._json(HTTPStatus.ACCEPTED, asdict(job))

        def log_message(self, format: str, *args: Any) -> None:
            return

    return Handler


def serve(
    *,
    listen: str,
    port: int,
    token: str,
    capture_dir: str | Path,
    report_dir: str | Path,
    allow_active_syn: bool = False,
    drone_mode: str = 'observe',
    controller_journal: str | Path | None = None,
    observe_interface: str | None = None,
    observe_max_duration: float = 600,
    observe_heartbeat: bool = False,
    observe_video: bool = False,
    observe_credentials_file: str | Path | None = None,
    observe_wlm_identity_file: str | Path | None = None,
    observe_ble_address: str | None = None,
    observe_ble_identity_file: str | Path | None = None,
    observe_ble_setup_timeout: float = 30,
    observe_hls: bool = False,
    observe_video_fps: float = 30,
    observe_video_codec: str = 'h264',
    observe_publish_config: str | Path | None = None,
) -> None:
    if listen not in ("127.0.0.1", "::1"):
        raise ValueError("diagnostic API must listen on an explicit loopback address")
    if len(token) < 32:
        raise ValueError("API token must contain at least 32 characters")
    mode = ControllerMode(drone_mode)
    if mode is ControllerMode.FLIGHT:
        raise ValueError('live flight adapter is not verified')
    if mode is ControllerMode.SIMULATE and allow_active_syn:
        raise ValueError('simulate mode cannot enable active radio probes')
    observation_config = None
    if observe_interface is not None:
        if mode is not ControllerMode.OBSERVE or not allow_active_syn:
            raise ValueError('observation requires observe mode and --allow-active-syn')
        observation_config = ObservationConfig(observe_interface, observe_max_duration, observe_heartbeat,
            observe_video, observe_credentials_file, observe_wlm_identity_file,
            observe_ble_address, observe_ble_identity_file, observe_ble_setup_timeout,
            observe_hls, observe_video_fps, observe_video_codec, observe_publish_config)
    elif (observe_heartbeat or observe_video or observe_max_duration != 600
          or observe_credentials_file is not None or observe_wlm_identity_file is not None
          or observe_ble_address is not None or observe_ble_identity_file is not None
          or observe_ble_setup_timeout != 30 or observe_hls
          or observe_video_fps != 30 or observe_video_codec != 'h264' or observe_publish_config is not None):
        raise ValueError('observation options require a locally selected interface')
    state = ServiceState(Path(capture_dir), Path(report_dir), allow_active_syn)
    state.capture_dir.mkdir(parents=True, exist_ok=True)
    state.report_dir.mkdir(parents=True, exist_ok=True)
    journal = controller_journal or state.capture_dir / f'controller-{secrets.token_hex(12)}.jsonl'
    runtime_class = ProcessControllerRuntime if mode is ControllerMode.SIMULATE else ControllerRuntime
    runtime = runtime_class(journal, mode=mode)
    state.controller = runtime.controller
    if observation_config:
        state.observation = ManagedObservation(observation_config, state.capture_dir,
                                               event_sink=runtime.controller.observation_event)
        runtime.controller.attach_observation_source(state.observation.snapshot)
    main_thread = threading.current_thread() is threading.main_thread()
    previous_sigterm = signal.getsignal(signal.SIGTERM) if main_thread else None
    term_requested = False
    def graceful_term(signum, frame):
        nonlocal term_requested
        if not term_requested:
            term_requested = True
            raise KeyboardInterrupt
    if main_thread:
        signal.signal(signal.SIGTERM, graceful_term)
    try:
        with ThreadingHTTPServer((listen, port), build_handler(state, token)) as server:
            print(json.dumps({'event': 'service_started', 'listen': listen,
                              'port': server.server_port, 'drone_mode': mode.value,
                              'observation_enabled': state.observation is not None,
                              'controller_journal': str(journal)}), flush=True)
            server.serve_forever()
    finally:
        try:
            if state.observation:
                state.observation.shutdown()
        finally:
            try:
                runtime.close()
            finally:
                if main_thread:
                    signal.signal(signal.SIGTERM, previous_sigterm)
