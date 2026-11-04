"""Constrained UDP probes that never carry a DUML flight command."""

from __future__ import annotations

from dataclasses import dataclass, asdict
from pathlib import Path
import re
import secrets
import socket
import time
from typing import Any

from .capture import CaptureEvent, CaptureRecorder
from .duml import (
    STATIC_CAPABILITY_DEVELOPER_SETTING,
    DumlSequence,
    neo2_heartbeat,
    neo2_static_capability,
)
from .identity import WlmIdentity, load_or_create_identity
from .transport import (
    ReliableSendState,
    ReliableTransport,
    is_session_accept,
    is_uplink_ack,
)

WIFI_DEVICE_INFO_START_DELAY_SECONDS = 0.019
HEARTBEAT_START_DELAY_SECONDS = 0.797
STATIC_CAPABILITY_START_DELAY_SECONDS = 0.810


@dataclass(frozen=True)
class ProbeResult:
    host: str
    port: int
    duration_seconds: float
    active_syn: bool
    session: int | None
    base_sequence: int | None
    transmitted: int
    received: int
    session_ack_seen: bool
    heartbeat_sent: bool
    heartbeat_sequence: int | None
    heartbeat_transport_ack_seen: bool
    wifi_device_info_sent: bool
    wifi_device_info_sequence: int | None
    wifi_device_info_transport_ack_seen: bool
    wifi_device_info_response_seen: bool
    wifi_device_info_receiver: int | None
    wifi_device_info_identity: dict[str, str | int] | None
    static_capability_sent: bool
    static_capability_sequence: int | None
    static_capability_ids: list[int]
    static_capability_transport_ack_seen: bool
    static_capability_response_seen: bool
    application_sent_after_accept_ms: float | None
    wifi_device_info_sent_after_accept_ms: float | None
    heartbeat_sent_after_accept_ms: float | None
    static_capability_sent_after_accept_ms: float | None
    drone_stream_window_observed: bool
    drone_stream_window_advanced: bool
    drone_video_window_end: int | None
    drone_protocol_window_end: int | None
    stateful_sequence_requested: bool
    stateful_sequence_completed: bool
    idle_window_sent: bool
    idle_windows_sent: int
    capture_path: str

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def _transport_layer(event: CaptureEvent) -> dict[str, Any] | None:
    """Return the decoded reliable-transport layer of a captured datagram."""

    for layer in event.decoded.get("layers", []):
        if layer.get("name") == "dji-reliable-udp":
            return layer
    return None


def _carries_duml(event: CaptureEvent, cmd_set: int, cmd_id: int) -> bool:
    """Return whether a captured datagram carries one CmdSet/CmdId pair."""

    for layer in event.decoded.get("layers", []):
        if layer.get("name") != "duml":
            continue
        if any(
            packet.get("command_set") == cmd_set
            and packet.get("command_id") == cmd_id
            for packet in layer.get("packets", [])
        ):
            return True
    return False


def run_probe(
    *,
    capture_path: str | Path,
    host: str = "192.168.2.1",
    port: int = 9003,
    duration: float = 8.0,
    send_syn: bool = False,
    send_heartbeat: bool = False,
    send_wifi_device_info: bool = False,
    send_wifi_then_heartbeat: bool = False,
    send_static_capability: bool = False,
    send_idle_window: bool = False,
    idle_window_count: int = 1,
    idle_window_interval: float = 0.1,
    idle_window_mode: str = "periodic",
    bind_host: str = "0.0.0.0",
    interface: str | None = None,
    identity_file: str | Path | None = None,
) -> ProbeResult:
    if not 0 < port <= 65535:
        raise ValueError("UDP port must be between 1 and 65535")
    if duration <= 0 or duration > 300:
        raise ValueError("probe duration must be between 0 and 300 seconds")
    if send_idle_window and not send_syn:
        raise ValueError("idle window requires the session SYN")
    if send_heartbeat and not send_syn:
        raise ValueError("heartbeat requires the session SYN")
    if send_wifi_device_info and not send_syn:
        raise ValueError("Wi-Fi device information requires the session SYN")
    if send_wifi_then_heartbeat and not send_syn:
        raise ValueError("stateful Wi-Fi/heartbeat sequence requires the session SYN")
    if send_heartbeat and send_wifi_device_info:
        raise ValueError("only one first reliable application probe can be sent")
    if send_wifi_then_heartbeat and (send_heartbeat or send_wifi_device_info):
        raise ValueError(
            "stateful Wi-Fi/heartbeat sequence is exclusive with single application probes"
        )
    if send_static_capability and not send_wifi_then_heartbeat:
        raise ValueError(
            "static capability request extends the stateful Wi-Fi/heartbeat sequence"
        )
    if not 1 <= idle_window_count <= 100:
        raise ValueError("idle window count must be between 1 and 100")
    if not 0.05 <= idle_window_interval <= 2:
        raise ValueError("idle window interval must be between 0.05 and 2 seconds")
    if idle_window_mode not in ("periodic", "immediate", "alternate"):
        raise ValueError("idle window mode must be periodic, immediate, or alternate")
    if interface and not re.fullmatch(r"[A-Za-z0-9_.-]{1,32}", interface):
        raise ValueError("invalid network interface name")
    if identity_file is not None:
        if not (send_wifi_device_info or send_wifi_then_heartbeat):
            raise ValueError("an identity file requires a Wi-Fi device-information probe")
        identity_path, capture = Path(identity_file), Path(capture_path)
        same_file = identity_path.exists() and capture.exists() and identity_path.samefile(capture)
        if identity_path.resolve() == capture.resolve() or same_file:
            raise ValueError("identity and capture must use distinct paths")
    wifi_identity = (
        (load_or_create_identity(identity_file) if identity_file is not None
         else WlmIdentity(secrets.token_hex(12)))
        if send_wifi_device_info or send_wifi_then_heartbeat else None
    )
    recorder = CaptureRecorder(capture_path)
    transport = ReliableTransport() if send_syn else None
    send_state = (
        ReliableSendState(transport)
        if transport
        and (send_heartbeat or send_wifi_device_info or send_wifi_then_heartbeat)
        else None
    )
    # The native encoder owns one sequence allocator per link: every DUML
    # request of a session draws from it, so the numbers stay consecutive.
    duml_sequence = DumlSequence()
    static_capability_ids = (STATIC_CAPABILITY_DEVELOPER_SETTING,)
    transmitted = 0
    received = 0
    session_ack_seen = False
    heartbeat_sent = False
    heartbeat_sequence: int | None = None
    heartbeat_transport_ack_seen = False
    wifi_device_info_sent = False
    wifi_device_info_sequence: int | None = None
    wifi_device_info_transport_ack_seen = False
    wifi_device_info_response_seen = False
    static_capability_sent = False
    static_capability_sequence: int | None = None
    static_capability_transport_ack_seen = False
    static_capability_response_seen = False
    session_accepted_at: float | None = None
    wifi_device_info_send_at: float | None = None
    heartbeat_send_at: float | None = None
    static_capability_send_at: float | None = None
    application_sent_after_accept_ms: float | None = None
    wifi_device_info_sent_after_accept_ms: float | None = None
    heartbeat_sent_after_accept_ms: float | None = None
    static_capability_sent_after_accept_ms: float | None = None
    wifi_device_info_transport_sequence: int | None = None
    heartbeat_transport_sequence: int | None = None
    static_capability_transport_sequence: int | None = None
    drone_stream_window_observed = False
    drone_stream_window_advanced = False
    drone_video_window_end: int | None = None
    drone_protocol_window_end: int | None = None
    idle_windows_sent = 0
    next_idle_window_at: float | None = None
    deadline = time.monotonic() + duration

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(0.05)
        if interface:
            if not hasattr(socket, "SO_BINDTODEVICE"):
                raise RuntimeError("binding to a network interface is only supported on Linux")
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, interface.encode() + b"\x00")
        if send_syn:
            sock.connect((host, port))
            frame = transport.open_session()
            sock.send(frame)
            recorder.record(
                interface="udp",
                direction="tx",
                peer=f"{host}:{port}",
                raw=frame,
                note="candidate reliable-transport SYN; contains no DUML command",
            )
            transmitted += 1
        else:
            sock.bind((bind_host, port))

        while time.monotonic() < deadline:
            now = time.monotonic()
            if (
                send_state
                and wifi_device_info_send_at is not None
                and now >= wifi_device_info_send_at
                and not wifi_device_info_sent
            ):
                wifi_device_info_sent_after_accept_ms = (
                    (now - session_accepted_at) * 1000
                    if session_accepted_at is not None
                    else None
                )
                if application_sent_after_accept_ms is None:
                    application_sent_after_accept_ms = (
                        wifi_device_info_sent_after_accept_ms
                    )
                wifi_device_info_sequence = duml_sequence.next()
                frame = send_state.send(
                    wifi_identity.wifi_report(
                        wifi_device_info_sequence,
                        physical_port=port,
                    ).encode()
                )
                wifi_device_info_transport_sequence = send_state.waiting_for_ack
                note = (
                    "bounded Neo 2 Wi-Fi software-device information request "
                    "with a normalized PC-owned WLM identity and receiver index zero; "
                    "no motor or flight command"
                )
                wifi_device_info_sent = True
                sock.send(frame)
                recorder.record(
                    interface="udp",
                    direction="tx",
                    peer=f"{host}:{port}",
                    raw=frame,
                    note=note,
                )
                transmitted += 1
                wifi_device_info_send_at = None
            if (
                send_state
                and heartbeat_send_at is not None
                and now >= heartbeat_send_at
                and not heartbeat_sent
                and (
                    not send_wifi_then_heartbeat
                    or wifi_device_info_transport_ack_seen
                )
            ):
                heartbeat_sent_after_accept_ms = (
                    (now - session_accepted_at) * 1000
                    if session_accepted_at is not None
                    else None
                )
                if application_sent_after_accept_ms is None:
                    application_sent_after_accept_ms = heartbeat_sent_after_accept_ms
                heartbeat_sequence = duml_sequence.next()
                frame = send_state.send(
                    neo2_heartbeat(heartbeat_sequence).encode()
                )
                heartbeat_transport_sequence = send_state.waiting_for_ack
                note = (
                    "bounded Neo 2 V1 heartbeat after the native startup "
                    "delay and ACK-gated reliable-window transition; no motor "
                    "or flight command"
                )
                heartbeat_sent = True
                sock.send(frame)
                recorder.record(
                    interface="udp",
                    direction="tx",
                    peer=f"{host}:{port}",
                    raw=frame,
                    note=note,
                )
                transmitted += 1
                heartbeat_send_at = None
            if (
                send_state
                and static_capability_send_at is not None
                and now >= static_capability_send_at
                and not static_capability_sent
                and heartbeat_transport_ack_seen
            ):
                static_capability_sent_after_accept_ms = (
                    (now - session_accepted_at) * 1000
                    if session_accepted_at is not None
                    else None
                )
                if application_sent_after_accept_ms is None:
                    application_sent_after_accept_ms = (
                        static_capability_sent_after_accept_ms
                    )
                static_capability_sequence = duml_sequence.next()
                frame = send_state.send(
                    neo2_static_capability(
                        static_capability_sequence,
                        static_capability_ids,
                    ).encode()
                )
                static_capability_transport_sequence = send_state.waiting_for_ack
                note = (
                    "bounded Neo 2 static capability read after the heartbeat "
                    "transport ACK; declared capability identifiers "
                    f"{list(static_capability_ids)}; no motor or flight command"
                )
                static_capability_sent = True
                sock.send(frame)
                recorder.record(
                    interface="udp",
                    direction="tx",
                    peer=f"{host}:{port}",
                    raw=frame,
                    note=note,
                )
                transmitted += 1
                static_capability_send_at = None
            if (
                transport
                and next_idle_window_at is not None
                and idle_windows_sent < idle_window_count
                and now >= next_idle_window_at
            ):
                periodic = idle_window_mode == "periodic" or (
                    idle_window_mode == "alternate"
                    and idle_windows_sent % 2 == 1
                )
                frame = transport.idle_window(periodic=periodic)
                transport_type = 6 if periodic else 4
                sock.send(frame)
                recorder.record(
                    interface="udp",
                    direction="tx",
                    peer=f"{host}:{port}",
                    raw=frame,
                    note=(
                        f"bounded ground type-{transport_type} idle window "
                        f"{idle_windows_sent + 1}/{idle_window_count}; "
                        "application queue is empty"
                    ),
                )
                transmitted += 1
                idle_windows_sent += 1
                next_idle_window_at += idle_window_interval
            receive_timeout = 0.05
            pending_send_times = [
                send_at
                for send_at in (
                    wifi_device_info_send_at,
                    heartbeat_send_at,
                    static_capability_send_at,
                )
                if send_at is not None and send_at > time.monotonic()
            ]
            if pending_send_times:
                receive_timeout = max(
                    0.001,
                    min(
                        receive_timeout,
                        min(pending_send_times) - time.monotonic(),
                    ),
                )
            sock.settimeout(receive_timeout)
            try:
                raw, peer = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except ConnectionResetError:
                continue
            event = recorder.record(
                interface="udp",
                direction="rx",
                peer=f"{peer[0]}:{peer[1]}",
                raw=raw,
            )
            received += 1
            layer = _transport_layer(event)
            if layer is not None and "drone_video_window_end" in layer:
                drone_stream_window_observed = True
                drone_video_window_end = layer["drone_video_window_end"]
                drone_protocol_window_end = layer["drone_protocol_window_end"]
                if transport and any(
                    layer[name] != transport.base_sequence
                    for name in (
                        "drone_video_window_start",
                        "drone_video_window_end",
                        "drone_protocol_window_start",
                        "drone_protocol_window_end",
                    )
                ):
                    # The drone has queued something on a stream channel: the
                    # observed sessions never leave the link anchor.
                    drone_stream_window_advanced = True
            if (
                transport
                and heartbeat_sent
                and heartbeat_transport_sequence is not None
                and is_uplink_ack(
                    raw,
                    transport.session,
                    heartbeat_transport_sequence,
                )
            ):
                heartbeat_transport_ack_seen = True
            if (
                transport
                and wifi_device_info_sent
                and wifi_device_info_transport_sequence is not None
                and is_uplink_ack(
                    raw,
                    transport.session,
                    wifi_device_info_transport_sequence,
                )
            ):
                wifi_device_info_transport_ack_seen = True
            if (
                transport
                and static_capability_sent
                and static_capability_transport_sequence is not None
                and is_uplink_ack(
                    raw,
                    transport.session,
                    static_capability_transport_sequence,
                )
            ):
                static_capability_transport_ack_seen = True
            if send_state:
                send_state.acknowledge(raw)
            if wifi_device_info_sent and _carries_duml(event, 0x07, 0x93):
                wifi_device_info_response_seen = True
            if static_capability_sent and _carries_duml(event, 0x00, 0xB7):
                static_capability_response_seen = True
            if transport and is_session_accept(raw, transport.session):
                session_ack_seen = True
                if session_accepted_at is None:
                    session_accepted_at = time.monotonic()
                    if send_wifi_device_info or send_wifi_then_heartbeat:
                        wifi_device_info_send_at = (
                            session_accepted_at
                            + WIFI_DEVICE_INFO_START_DELAY_SECONDS
                        )
                    if send_heartbeat or send_wifi_then_heartbeat:
                        heartbeat_send_at = (
                            session_accepted_at + HEARTBEAT_START_DELAY_SECONDS
                        )
                    if send_static_capability:
                        static_capability_send_at = (
                            session_accepted_at
                            + STATIC_CAPABILITY_START_DELAY_SECONDS
                        )
                if send_idle_window and next_idle_window_at is None:
                    next_idle_window_at = time.monotonic()

    return ProbeResult(
        host=host,
        port=port,
        duration_seconds=duration,
        active_syn=send_syn,
        session=transport.session if transport else None,
        base_sequence=transport.base_sequence if transport else None,
        transmitted=transmitted,
        received=received,
        session_ack_seen=session_ack_seen,
        heartbeat_sent=heartbeat_sent,
        heartbeat_sequence=heartbeat_sequence,
        heartbeat_transport_ack_seen=heartbeat_transport_ack_seen,
        wifi_device_info_sent=wifi_device_info_sent,
        wifi_device_info_sequence=wifi_device_info_sequence,
        wifi_device_info_transport_ack_seen=wifi_device_info_transport_ack_seen,
        wifi_device_info_response_seen=wifi_device_info_response_seen,
        wifi_device_info_receiver=0x1B if wifi_identity is not None else None,
        wifi_device_info_identity=wifi_identity.metadata() if wifi_identity is not None else None,
        static_capability_sent=static_capability_sent,
        static_capability_sequence=static_capability_sequence,
        static_capability_ids=(
            list(static_capability_ids) if send_static_capability else []
        ),
        static_capability_transport_ack_seen=static_capability_transport_ack_seen,
        static_capability_response_seen=static_capability_response_seen,
        application_sent_after_accept_ms=application_sent_after_accept_ms,
        wifi_device_info_sent_after_accept_ms=(
            wifi_device_info_sent_after_accept_ms
        ),
        heartbeat_sent_after_accept_ms=heartbeat_sent_after_accept_ms,
        static_capability_sent_after_accept_ms=(
            static_capability_sent_after_accept_ms
        ),
        drone_stream_window_observed=drone_stream_window_observed,
        drone_stream_window_advanced=drone_stream_window_advanced,
        drone_video_window_end=drone_video_window_end,
        drone_protocol_window_end=drone_protocol_window_end,
        stateful_sequence_requested=send_wifi_then_heartbeat,
        stateful_sequence_completed=(
            send_wifi_then_heartbeat
            and wifi_device_info_transport_ack_seen
            and heartbeat_transport_ack_seen
            and (
                not send_static_capability
                or static_capability_transport_ack_seen
            )
        ),
        idle_window_sent=idle_windows_sent > 0,
        idle_windows_sent=idle_windows_sent,
        capture_path=str(Path(capture_path).resolve()),
    )
