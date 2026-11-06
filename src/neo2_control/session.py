"""Persistent SWUDP transport client for the measured Neo 2 link profile.

Transport acceptance is distinct from application/flight readiness. Receive
windows reorder the measured type 2/3 profile before handing bytes to a sink.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from contextlib import ExitStack
from ipaddress import IPv4Address
import json
import math
import os
from pathlib import Path
import re
import socket
import threading
import time
from typing import Any, Callable

from .capture import CaptureRecorder
from .platform_transport import bind_ipv4_socket, validate_interface_binding
from .udp_observation import OwnerCosts, UdpReceiptObserver
from .duml import DumlSequence, neo2_heartbeat
from .receiver import ReliableReceiveState
from .telemetry import TelemetryReceiver
from .wifi_status import WifiControlReceiver
from .battery import BatteryReceiver
from .landing_confirmation import LandingConfirmationReceiver
from .identity import WlmIdentity, load_or_create_identity
from .wlm_bootstrap import WlmWifiBootstrap
from .wlm_discovery import WlmDiscovery
from .wlm_query import WlmCapabilityRead
from .wlm_presence import WlmPresence
from .wlm_link import WlmWifiLink
from .camera import CameraChannel
from .flight_channel import FlightChannel
from .liveview import WifiKeyframeRequests
from .video import HlsVideo
from .video_worker import ManagedHevcDecoder
from .video_publish import ManagedPublisher, PublishConfig
from .transport import (
    ReliableSendState,
    ReliableTransport,
    MAX_DATAGRAM_LENGTH,
    STREAM_HEADER_LENGTH,
    TYPE_1_HEADER_LENGTH,
    describe_transport,
    is_session_accept,
)


@dataclass(frozen=True)
class SessionTiming:
    handshake_timeout: float = 3.0
    remote_timeout: float = 2.0
    acknowledgement_timeout: float = 1.0
    window_interval: float = 0.1
    first_heartbeat_delay: float = 0.797
    heartbeat_interval: float = 0.5

    def __post_init__(self) -> None:
        for name, value in asdict(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class OutboundDatagram:
    raw: bytes
    purpose: str
    application_kind: str | None = None
    duml_sequence: int | None = None
    tos: int = 0


class SwudpSession:
    """Clock-driven session state with bounded, independent receive queues.

    Heartbeats and optional WLM reports share one DUML allocator and sender.
    The direct profile gates each application packet on its transport ACK.
    The negotiated QoS profile drains raw DUML without a reliable ACK or
    automatic replay. Both profiles require fresh incoming traffic for liveness.
    """

    def __init__(
        self,
        *,
        heartbeat: bool = False,
        timing: SessionTiming | None = None,
        transport: ReliableTransport | None = None,
        duml_sequence: DumlSequence | None = None,
        event_sink: Callable[[dict[str, Any]], None] | None = None,
        payload_sink: Callable[[int, bytes], None] | None = None,
        wlm_identity: WlmIdentity | None = None,
        physical_wifi_port: int = 9003,
        wlm_capability_query: bool = False,
        wlm_mode_follow: bool = False,
        wlm_wifi_liveview: bool = False,
        wlm_forwarding_sequence: int = 1,
        wifi_backend_ready: Callable[[], bool] | None = None,
        camera_channel: CameraChannel | None = None,
        flight_channel: FlightChannel | None = None,
        video_keyframe_needed: Callable[[], bool] | None = None,
        decoded_video: ManagedHevcDecoder | None = None,
        preview_only: bool = False,
    ) -> None:
        if type(preview_only) is not bool or (preview_only and (wlm_identity is None or flight_channel is not None)):
            raise ValueError('preview_requires_owned_identity_and_no_flight_channel')
        self.preview_only = preview_only
        self.preview_physical_port = physical_wifi_port
        if type(wlm_capability_query) is not bool or (wlm_capability_query and wlm_identity is None):
            raise ValueError('WLM capability query requires a boolean option and an owned WLM identity')
        if (type(wlm_mode_follow) is not bool or (wlm_mode_follow and wlm_identity is None)
                or (wlm_mode_follow and wlm_capability_query)):
            raise ValueError('WLM mode follow requires owned identity and excludes the separate read experiment')
        if (type(wlm_wifi_liveview) is not bool or (wlm_wifi_liveview and (
                wlm_identity is None or wlm_capability_query or not callable(wifi_backend_ready)))):
            raise ValueError('Wi-Fi liveview requires owned identity and an explicit backend owner')
        if type(wlm_forwarding_sequence) is not int or not 0<=wlm_forwarding_sequence<=0xffffffff:
            raise ValueError('initial WLM forwarding sequence must fit uint32')
        if wlm_forwarding_sequence!=1 and (wlm_identity is None or not (wlm_mode_follow or wlm_wifi_liveview)):
            raise ValueError('nondefault WLM forwarding sequence requires owned persistent WLM')
        self.wifi_backend_ready=wifi_backend_ready
        if camera_channel is not None and (not wlm_wifi_liveview or not isinstance(camera_channel,CameraChannel)):
            raise ValueError('camera commands require the owned Wi-Fi liveview session')
        if flight_channel is not None and (not wlm_wifi_liveview or not isinstance(flight_channel,FlightChannel)):
            raise ValueError('flight commands require the owned Wi-Fi service session')
        if video_keyframe_needed is not None and (not callable(video_keyframe_needed)
                or camera_channel is None or not wlm_wifi_liveview):
            raise ValueError('video keyframe requests require a callable and owned camera liveview')
        self.video_keyframe_needed=video_keyframe_needed
        self.flight_channel=flight_channel
        self._flight_projection_at = None
        self._flight_projection_state = None
        self.video_keyframes=WifiKeyframeRequests() if video_keyframe_needed is not None else None
        self.transport = transport or ReliableTransport()
        self.sender = ReliableSendState(self.transport)
        self.sequence = duml_sequence or DumlSequence()
        self.timing = timing or SessionTiming()
        self.heartbeat = heartbeat
        self.event_sink = event_sink
        self.payload_sink = payload_sink
        if decoded_video is not None and not isinstance(decoded_video,ManagedHevcDecoder):
            raise ValueError('decoded_video_requires_owned_decoder')
        self.decoded_video = decoded_video
        self._decoded_video_accepting = decoded_video is not None
        self.wlm = WlmWifiBootstrap(wlm_identity, physical_wifi_port) if wlm_identity is not None else None
        self.wlm_discovery = WlmDiscovery(maximum_age=self.timing.remote_timeout)
        self.capability_query_enabled = wlm_capability_query
        self.capability_read = None
        self.capability_query_scheduled = False
        self.capability_query_observed_state = None
        self.wlm_presence = None
        self.capability_peer_id = None
        self.capability_peer_seen_at = None
        self.wlm_link = WlmWifiLink(wlm_identity,self.sequence.next,
            # Captured camera sessions contain beacon gaps near4s while
            # camera/FC traffic from the same peer continues. Keep a
            # bounded5s beacon policy for this camera-only owner; ordinary
            # observation and future flight owners retain the transport bound.
            maximum_age=5 if camera_channel is not None and flight_channel is None else min(5,self.timing.remote_timeout),
            event=lambda name,**detail:self._event(self.wlm_link.last_time,name,**detail),
            switch_wifi_service=wlm_wifi_liveview,
            prepare_wifi=self._prepare_wifi_service,
            camera_channel=camera_channel,
            flight_channel=flight_channel,
            **({'initial_forwarding_sequence':wlm_forwarding_sequence}
               if wlm_forwarding_sequence!=1 else {})) if wlm_mode_follow or wlm_wifi_liveview else None
        self.pending_application = None
        # Current SYN: video capacity 100, protocol capacity 256, FEC disabled.
        self.receivers = {
            channel: ReliableReceiveState(
                session=self.transport.session, anchor=self.transport.base_sequence,
                channel=channel, capacity=capacity,
            )
            for channel, capacity in ((2, 100), (3, 256))
        }
        self.video_bytes_delivered = self.protocol_bytes_delivered = 0
        self.telemetry = TelemetryReceiver()
        self.wifi_control = WifiControlReceiver()
        self.battery = BatteryReceiver(maximum_age=(flight_channel.observer.BATTERY_MAXIMUM_AGE
            if flight_channel is not None and flight_channel.observer is not None else 1.))
        self.landing_confirmation = LandingConfirmationReceiver()
        self.state = "new"
        self.reason: str | None = None
        self.started_at: float | None = None
        self.last_received_at: float | None = None
        self.accepted_at: float | None = None
        self.feature_flags: int | None = None
        self.next_window_at = 0.0
        self.window_interval = self.timing.window_interval
        self.next_heartbeat_at = 0.0
        self.ack_deadline: float | None = None
        self.heartbeats_sent = 0
        self.heartbeats_acknowledged = 0
        self.rejected_datagrams = 0

    @property
    def terminal(self) -> bool:
        return self.state in ("closed", "failed")

    def _prepare_wifi_service(self,modes) -> bool:
        # Native Wi-Fi51/07 and51/05 preparation only tests local link state.
        # Here the owner proves the actual connected socket, while this layer
        # verifies its negotiated transport and allocated receive windows.
        return (self.state=='open' and self.qos and self.wifi_backend_ready is not None
            and self.receivers[2].capacity==100 and self.receivers[3].capacity==256
            and self.wifi_backend_ready() is True)

    @property
    def qos(self) -> bool:
        return self.feature_flags is not None and bool(self.feature_flags & 2)

    @property
    def delivery_mode(self) -> str | None:
        return None if self.feature_flags is None else ('qos' if self.qos else 'reliable')

    @property
    def application_tos(self) -> int:
        # Native heartbeat, WLM reports and direct18/47 use category4 with
        # bit3 enabled; otherwise these direct QoS frames use category6.
        return (128 if self.feature_flags & 8 else 192) if self.qos else 0

    def _event(self, now: float, event: str, **detail: Any) -> None:
        if self.event_sink is not None:
            self.event_sink({
                "elapsed_seconds": now - (self.started_at if self.started_at is not None else now),
                "event": event,
                "state": self.state,
                **detail,
            })

    def start(self, now: float) -> OutboundDatagram:
        if self.state != "new":
            raise RuntimeError("a session can only be started once")
        self.started_at = now
        self.state = "connecting"
        self._event(now, "session_opening", session=self.transport.session,
                    base_sequence=self.transport.base_sequence)
        return OutboundDatagram(self.transport.open_session(), "session SYN")

    def fail(self, now: float, reason: str) -> None:
        if not self.terminal:
            self._stop_decoded_video('session_failed:'+reason)
            if self.capability_read is not None:
                self.capability_read.cancel(now,reason)
            if self.wlm_presence is not None:
                self.wlm_presence.cancel(now,reason)
            if self.wlm_link is not None:
                self.wlm_link.close(now,reason,failed=True)
            if self.video_keyframes is not None:self.video_keyframes.close(now,reason)
            self.state = "failed"
            self.reason = reason
            self._event(now, "session_failed", reason=reason)

    def close(self, now: float, reason: str = "requested") -> None:
        if not self.terminal:
            self._stop_decoded_video('session_closed:'+reason)
            if self.capability_read is not None:
                self.capability_read.cancel(now,reason)
            if self.wlm_presence is not None:
                self.wlm_presence.cancel(now,reason)
            if self.wlm_link is not None:
                self.wlm_link.close(now,reason)
            if self.video_keyframes is not None:self.video_keyframes.close(now,reason)
            self.state = "closed"
            self.reason = reason
            self._event(now, "session_closed", reason=reason)

    def _expire(self, now: float) -> None:
        if self.state == "connecting" and self.started_at is not None:
            if now - self.started_at >= self.timing.handshake_timeout:
                self.fail(now, "handshake_timeout")
        elif self.state == "open":
            if self.ack_deadline is not None and now >= self.ack_deadline:
                self.fail(now, "acknowledgement_timeout")
            elif self.last_received_at is not None:
                if now - self.last_received_at >= self.timing.remote_timeout:
                    self.fail(now, "remote_timeout")

    def _reject(self, now: float, reason: str) -> None:
        self.rejected_datagrams += 1
        self._event(now, "datagram_rejected", reason=reason)

    def _wlm_beacon(self,beacon,now):
        if self.wlm_link is not None and self.qos and not self.terminal:
            self.wlm_link.on_beacon(beacon,now)
        if not self.capability_query_enabled or not self.qos or self.terminal:
            return
        metadata=beacon.metadata()
        if self.capability_read is not None:
            if beacon.device_id == self.capability_peer_id:
                if metadata['format_code'] != 5 or not metadata['ability_negotiation_advertised']:
                    self.capability_read.cancel(now,'peer_profile_changed')
                    self.wlm_presence.cancel(now,'peer_profile_changed')
                else:
                    self.capability_peer_seen_at = now
            return
        if metadata['format_code'] != 5 or not metadata['ability_negotiation_advertised']:
            return
        if (metadata['device_type'],metadata['device_index'],metadata['device_model'])!=(0,0,36):
            return
        try:
            self.capability_read = WlmCapabilityRead(self.wlm.identity,beacon,
                sequence=self.sequence.next(),forwarding_sequence=1,
                timeout=min(self.timing.remote_timeout,2.))
            self.wlm_presence = WlmPresence(self.wlm.identity)
            self.capability_peer_id = beacon.device_id
            self.capability_peer_seen_at = now
        except ValueError:
            return
        self._capability_read_event(now)

    def _capability_read_snapshot(self,now):
        if not self.capability_query_enabled:
            return {'enabled':False}
        if self.capability_read is None:
            return {'enabled':True,'state':('unsupported_transport' if self.accepted_at is not None
                and not self.qos else 'awaiting_current_beacon'),'application_ready':False}
        return {**self.capability_read.snapshot(now),'scheduled':self.capability_query_scheduled,
                'presence':self.wlm_presence.snapshot(now)}

    def _capability_read_event(self,now):
        if self.capability_read is None:
            return
        snapshot=self._capability_read_snapshot(now)
        if snapshot['state']!=self.capability_query_observed_state:
            self.capability_query_observed_state=snapshot['state']
            self._event(now,'wlm_capability_read_state',query_state=snapshot['state'],
                sequence=snapshot['sequence'],response_status=snapshot['response_status'],
                capabilities=snapshot['capabilities'],application_ready=False)

    def _stop_decoded_video(self, reason):
        if self.decoded_video is not None:
            self._decoded_video_accepting = False
            self.decoded_video.invalidate(reason)

    def _decoded_video_snapshot(self):
        return ({'enabled':True,'swudp_attached':True,**self.decoded_video.status()}
            if self.decoded_video is not None else {'enabled':False})

    def _deliver(self, channel: int, payload: bytes, now: float, *, received_at=None) -> None:
        if channel == 3:
            if self.capability_read is not None:
                try:
                    self.capability_read.feed(payload,now)
                    self._capability_read_event(now)
                except ValueError:
                    self._event(now,'wlm_capability_read_rejected')
            try:
                self.wlm_discovery.feed(payload, now, lambda name, **detail: self._event(now, name, **detail),
                                        beacon_sink=self._wlm_beacon if self.capability_query_enabled or self.wlm_link is not None else None)
            except ValueError:
                self._event(now, 'wlm_discovery_rejected')
            if self.wlm_link is not None and self.qos:
                try:self.wlm_link.feed(payload,now)
                except ValueError:self._event(now,'wlm_link_input_rejected')
            if self.wlm is not None:
                try:
                    self.wlm.feed(payload, lambda name, **detail: self._event(now, name, **detail))
                except ValueError:
                    self._event(now, 'wlm_protocol_rejected')
            before = self.telemetry.accepted
            direct_context = None
            try:
                route = (self.wlm_link.flight_route(now)
                    if self.flight_channel is not None and self.wlm_link is not None else None)
                direct_context = (route.host_control
                    if route is not None and route.command_ready else None)
                self.telemetry.feed(payload, now, direct_context=direct_context)
            except ValueError:
                self._event(now, 'telemetry_decode_rejected')
            if self.telemetry.accepted != before:
                self._event(now, 'telemetry_updated', telemetry=self.telemetry.snapshot(now))
            before = self.wifi_control.accepted
            try:
                self.wifi_control.feed(payload, now)
            except ValueError:
                self._event(now, 'wifi_control_decode_rejected')
            if self.wifi_control.accepted != before:
                self._event(now, 'wifi_control_updated', wifi_control=self.wifi_control.snapshot(now))
            before = self.battery.accepted
            try:
                self.battery.feed(payload, now, direct_context=direct_context)
            except ValueError:
                self._event(now, 'battery_decode_rejected')
            if self.battery.accepted != before:
                self._event(now, 'battery_updated', battery=self.battery.snapshot(now))
            before = self.landing_confirmation.accepted
            try:
                self.landing_confirmation.feed(payload, now, direct_context=direct_context)
            except ValueError:
                self._event(now, 'landing_confirmation_decode_rejected')
            if self.landing_confirmation.accepted != before:
                self._event(now, 'landing_confirmation_updated',
                            landing_confirmation=self.landing_confirmation.snapshot(now))
            self._project_flight(now)
        if channel == 2 and self._decoded_video_accepting:
            try:
                accepted = self.decoded_video.feed(payload,delivered_at=now,received_at=received_at)
            except Exception:
                self.decoded_video.invalidate('session_video_enqueue_failed')
                accepted = False
            if not accepted:
                self._decoded_video_accepting = False
                self._event(now,'decoded_video_stopped',reason=self.decoded_video.status()['error'])
        if self.payload_sink is not None:
            self.payload_sink(channel, payload)
        if channel == 2:
            self.video_bytes_delivered += len(payload)
            if self.wlm_link is not None:self.wlm_link.video_received(len(payload))
        else:
            self.protocol_bytes_delivered += len(payload)
        self._event(now, "stream_payload_delivered", channel=channel, bytes=len(payload))

    def _project_flight(self, now, *, force=True):
        if self.flight_channel is not None and self.flight_channel.observer is not None:
            if (not force and self._flight_projection_at is not None
                    and self._flight_projection_state==self.state
                    and 0<=now-self._flight_projection_at<.025):
                return
            route=self.wlm_link.flight_route(now) if self.state=='open' and self.wlm_link is not None else None
            self.flight_channel.update_observations(route,self.telemetry.snapshot(now),
                self.battery.snapshot(now),self.wifi_control.snapshot(now),now,
                landing_confirmation=self.landing_confirmation.snapshot(now))
            self._flight_projection_at,self._flight_projection_state=now,self.state

    def receive(self, raw: bytes, now: float) -> None:
        self._expire(now)
        if self.state == "new" or self.terminal:
            return
        if len(raw) > MAX_DATAGRAM_LENGTH:
            self._reject(now, "datagram_exceeds_native_mtu")
            return
        try:
            decoded, _ = describe_transport(raw)
        except ValueError:
            self._reject(now, "malformed_datagram")
            return
        if (
            decoded["declared_length"] != len(raw)
            or decoded["flags"] != 2
            or not decoded["header_checksum_valid"]
            or decoded["session"] != self.transport.session
        ):
            self._reject(now, "invalid_envelope_or_session")
            return

        if decoded["type"] == 0:
            if not is_session_accept(raw, self.transport.session):
                self._reject(now, "invalid_acceptance")
                return
            flags = int(decoded["feature_flags"])
            if self.state == "connecting":
                if flags & ~0x0F:
                    self.fail(now, "unsupported_feature_flags")
                    return
                self.feature_flags = flags
                # Native mode1 emits full ACKs, on an ACK-limit × 10ms
                # timer (limit starts at 2), without intermediate type6 ACKs.
                ack_limit = max(2, int(decoded['ack_limit'])) if flags & 1 else 2
                self.window_interval = min(self.timing.window_interval,
                                           ack_limit * .01)
                self.state = "open"
                self.accepted_at = now
                self.last_received_at = now
                self.next_window_at = now
                self.next_heartbeat_at = now + self.timing.first_heartbeat_delay
                if self.wlm is not None:
                    self.wlm.open()
                self._event(now, "transport_accepted", feature_flags=flags,
                            delivery_mode=self.delivery_mode, application_ready=False)
            # A delayed duplicate must not reset counters, cadence or liveness.
            return
        if self.state != "open":
            self._reject(now, "data_before_acceptance")
            return

        if decoded["type"] in (2, 3):
            if len(raw) < STREAM_HEADER_LENGTH:
                self._reject(now, "truncated_stream")
                return
            channel = int(decoded["type"])
            try:
                blocks = self.receivers[channel].receive(raw,received_at=now)
            except ValueError as error:
                self._reject(now, str(error))
                return
            self.last_received_at = now
            for block,origin in zip(blocks,self.receivers[channel].last_deliveries):
                self._deliver(channel, block, now,received_at=origin.received_at)
            return
        if decoded["type"] != 1 or len(raw) < TYPE_1_HEADER_LENGTH:
            self._reject(now, "unsupported_or_truncated_status")
            return
        if int(decoded['uplink_ack_slot_count']) > 20:
            self._reject(now, "uplink_ack_bitmap_exceeds_send_capacity")
            return
        payload_offset = int(decoded['payload_offset'])
        if decoded["downlink_aggregate_length"] != len(raw) - payload_offset:
            self._reject(now, "invalid_aggregate_length")
            return
        self.last_received_at = now
        for channel, prefix in ((2, 'drone_video'), (3, 'drone_protocol')):
            try:
                self.receivers[channel].observe_window(*(
                    int(decoded[f'{prefix}_{name}'])
                    for name in ('window_start', 'window_end', 'window_flags_0', 'window_flags_1')
                ))
            except ValueError as error:
                # Native Deal_Win rejects each channel independently, before
                # processing the separate uplink ACK and aggregate.
                self._event(now, 'receive_window_rejected', channel=channel, reason=str(error))
        if self.sender.acknowledge(raw):
            self.ack_deadline = None
            pending, self.pending_application = self.pending_application, None
            if pending is not None:
                kind, sequence = pending
                if kind == 'heartbeat':
                    self.heartbeats_acknowledged += 1
                elif self.wlm is not None:
                    self.wlm.acknowledge(sequence)
            self._event(now, "uplink_acknowledged",
                        sequence=self.sender.acknowledged_sequence)
        if len(raw) > payload_offset:
            self._deliver(3, raw[payload_offset:], now)

    def _application_datagram(self, payload: bytes, kind: str, sequence: int, now: float) -> bytes:
        if self.preview_only:
            from .preview_policy import validate_preview_udp
            validate_preview_udp(payload, self.wlm.identity,
                                 peer_id=self.wlm_link.peer_id if self.wlm_link is not None else None,
                                 physical_port=self.preview_physical_port)
        if self.qos:
            raw = self.transport.qos_data(
                payload, video_receive=self.receivers[2].status,
                protocol_receive=self.receivers[3].status,
                send_window_start=self.sender.acknowledged_sequence,
                send_window_end=self.sender.acknowledged_sequence,
            )
            for receiver in self.receivers.values():
                receiver.consume_ack_events()
            return raw
        raw = self.sender.send(payload)
        self.pending_application = kind, sequence
        self.ack_deadline = now + self.timing.acknowledgement_timeout
        return raw

    def datagram_sent(self, datagram: OutboundDatagram, now: float) -> None:
        """Record a completed socket send separately from scheduling and ACKs."""
        if datagram.application_kind is None:
            return
        if datagram.application_kind=='video_keyframe_request':
            self.video_keyframes.sent(datagram.duml_sequence,now)
            self._event(now,'video_keyframe_request_sent',duml_sequence=datagram.duml_sequence,
                        keyframe_requests=self.video_keyframes.snapshot(now))
        elif datagram.application_kind.startswith('wlm_link_'):
            self.wlm_link.sent(datagram.application_kind,now)
        elif datagram.application_kind == 'wlm_capability_query':
            self.capability_read.sent(now)
            self._capability_read_event(now)
        elif datagram.application_kind == 'wlm_presence':
            self.wlm_presence.sent(now)
            self._event(now,'wlm_presence_datagram_sent',
                        datagrams_sent=self.wlm_presence.sent_count,
                        peer_registration_confirmed=False,application_ready=False)
        elif self.wlm is not None and datagram.application_kind != 'heartbeat':
            self.wlm.sent(datagram.duml_sequence, self.delivery_mode)
        self._event(now, 'application_datagram_sent', kind=datagram.application_kind,
                    duml_sequence=datagram.duml_sequence, delivery_mode=self.delivery_mode)

    def _video_keyframe_snapshot(self,now):
        return {'enabled':False} if self.video_keyframes is None else self.video_keyframes.snapshot(now)

    def _next_video_keyframe(self,now):
        requests=self.video_keyframes
        if requests is None or requests.closed:return None
        try:
            needed=self.video_keyframe_needed() is True
            backend_ready=self._prepare_wifi_service(None)
        except Exception:
            requests.close(now,'callback_failed')
            self._event(now,'video_keyframe_requests_cancelled',reason='callback_failed')
            return None
        link=self.wlm_link
        ssfn=None
        if not self.qos:reason='unsupported_transport'
        elif not backend_ready:reason='backend_not_ready'
        elif not self.video_bytes_delivered:reason='awaiting_video_bytes'
        elif link is None or link.state in ('closed','failed'):reason='peer_unavailable'
        elif link.camera_channel is None or link.camera_channel.revoked:reason='camera_revoked'
        elif (link.peer_seen_at is None or now-link.peer_seen_at>=link.maximum_age
                or link.peer_beacon is None or link.peer_beacon.device_id!=link.peer_id):reason='peer_not_current'
        elif link.peer_beacon.device_id[0]!=0:reason='unsupported_peer'
        elif link.peer_modes is None or link.peer_modes.liveview!=4:reason='peer_not_wifi_liveview'
        else:
            reason=None
            ssfn=int.from_bytes(link.peer_beacon.payload[43:47],'little')
        previous=requests.pending
        packet=requests.next_packet(now,needed=needed,ssfn=ssfn,next_sequence=self.sequence.next,blocked_by=reason)
        if previous is not None and requests.pending is None:
            self._event(now,'video_keyframe_request_cancelled',duml_sequence=previous['sequence'],
                        keyframe_requests=requests.snapshot(now))
        if packet is None:return None
        raw=self._application_datagram(packet.encode(),'video_keyframe_request',packet.seq,now)
        self._event(now,'video_keyframe_request_scheduled',duml_sequence=packet.seq,ssfn=ssfn,
                    keyframe_requests=requests.snapshot(now))
        return OutboundDatagram(raw,'bounded direct Wi-Fi keyframe request','video_keyframe_request',
                                packet.seq,self.application_tos)

    def datagram_send_allowed(self,datagram,now):
        if self.preview_only and datagram.application_kind in ('wlm_link_camera', 'wlm_link_flight'):
            return False
        if datagram.application_kind=='wlm_link_camera':
            return self.wlm_link is not None and self.wlm_link.camera_send_allowed(datagram.duml_sequence,now)
        if datagram.application_kind=='wlm_link_flight':
            # Re-evaluate the actual backend and selected peer immediately
            # before the socket call, without renewing aircraft receipt times.
            self._project_flight(now)
            return self.wlm_link is not None and self.wlm_link.flight_send_allowed(datagram.duml_sequence,now)
        if datagram.application_kind!='video_keyframe_request':return True
        requests=self.video_keyframes
        if (self.terminal or requests is None or requests.pending is None
                or requests.pending['sequence']!=datagram.duml_sequence):return False
        # The decoder may finish bootstrap on its own thread after tick().
        # Recheck before the actual UDP send, without allocating another frame.
        self._next_video_keyframe(now)
        return requests.pending is not None and requests.pending['sequence']==datagram.duml_sequence

    def tick(self, now: float) -> list[OutboundDatagram]:
        self._expire(now)
        # Video can cause hundreds of ticks per second. Protocol delivery,
        # explicit snapshots and every final flight send still force a full
        # projection; idle ticks only need to age it at a bounded cadence.
        self._project_flight(now,force=False)
        if self.state != "open":
            return []
        self._capability_read_event(now)
        outgoing = []
        if (self.capability_read is not None
                and now-self.capability_peer_seen_at >= self.timing.remote_timeout):
            self.capability_read.cancel(now,'peer_beacon_expired')
            self.wlm_presence.cancel(now,'peer_beacon_expired')
        if self.wlm is not None and self.wlm.has_pending and self.sender.waiting_for_ack is None:
            sequence = self.sequence.next()
            packet, kind, trigger = self.wlm.next_report(sequence)
            raw = self._application_datagram(packet.encode(), kind, sequence, now)
            self._event(now, 'wlm_report_scheduled', kind=kind, trigger=trigger,
                        duml_sequence=sequence, transport_sequence=self.sender.waiting_for_ack,
                        delivery_mode=self.delivery_mode)
            outgoing.append(OutboundDatagram(raw, f'WLM {kind} report', kind, sequence,
                                            self.application_tos))
        if self.wlm_link is not None and self.qos:
            transmission=self.wlm_link.next_packet(now)
            if transmission is not None:
                raw=self._application_datagram(transmission.packet.encode(),transmission.kind,
                                               transmission.sequence,now)
                outgoing.append(OutboundDatagram(raw,transmission.kind,transmission.kind,
                                                transmission.sequence,128))
        if self.wlm_presence is not None:
            presence = self.wlm_presence.next_packet(now)
            if presence is not None:
                raw=self._application_datagram(presence.encode(),'wlm_presence',0,now)
                outgoing.append(OutboundDatagram(raw,'bounded PC WLM presence',
                                                'wlm_presence',0,128))
        if (self.capability_read is not None and not self.capability_query_scheduled
                and self.capability_read.snapshot(now)['state']=='prepared'
                and self.wlm_presence.snapshot(now)['state']=='complete'):
            self.capability_query_scheduled=True
            query=self.capability_read
            raw=self._application_datagram(query.request.encode(),'wlm_capability_query',query.sequence,now)
            # Native wrapper51/01 stays in category4/TOS128 with or without
            # the featurebit3 used by heartbeat/report classification.
            outgoing.append(OutboundDatagram(raw,'one WLM capability read',
                'wlm_capability_query',query.sequence,128))
        keyframe=self._next_video_keyframe(now)
        if keyframe is not None:outgoing.append(keyframe)
        if (
            self.heartbeat
            and now >= self.next_heartbeat_at
            and self.sender.waiting_for_ack is None
        ):
            sequence = self.sequence.next()
            raw = self._application_datagram(neo2_heartbeat(sequence=sequence).encode(),
                                             'heartbeat', sequence, now)
            self.heartbeats_sent += 1
            self.next_heartbeat_at = now + self.timing.heartbeat_interval
            self._event(now, "heartbeat_scheduled", duml_sequence=sequence,
                        transport_sequence=self.sender.waiting_for_ack,
                        delivery_mode=self.delivery_mode)
            outgoing.append(OutboundDatagram(raw, "V1 heartbeat", 'heartbeat', sequence,
                                            self.application_tos))
        if now >= self.next_window_at:
            outgoing.append(OutboundDatagram(self.sender.idle_window(
                periodic=False,
                video_receive=self.receivers[2].status,
                protocol_receive=self.receivers[3].status,
                video_changes=self.receivers[2].ack_events,
                protocol_changes=self.receivers[3].ack_events,
            ), "periodic transport window", tos=192 if self.qos else 0))
            for receiver in self.receivers.values():
                receiver.consume_ack_events()
            self.next_window_at = now + self.window_interval
        return outgoing

    def snapshot(self, now: float) -> dict[str, Any]:
        """Publish from the owning socket thread; readers age a detached copy."""
        self._project_flight(now)
        return {
            'observed_at': now, 'state': self.state, 'reason': self.reason,
            'session': self.transport.session, 'base_sequence': self.transport.base_sequence,
            'transport_accepted': self.accepted_at is not None, 'application_ready': False,
            'feature_flags': self.feature_flags, 'delivery_mode': self.delivery_mode,
            'ack_mode': 1, 'window_interval_seconds': self.window_interval,
            'last_received_age_seconds': None if self.last_received_at is None else now - self.last_received_at,
            'remote_timeout_seconds': self.timing.remote_timeout,
            'heartbeats_scheduled': self.heartbeats_sent,
            'heartbeats_acknowledged': self.heartbeats_acknowledged,
            'rejected_datagrams': self.rejected_datagrams,
            'video_bytes_delivered': self.video_bytes_delivered,
            'video_keyframe_requests': self._video_keyframe_snapshot(now),
            'decoded_video':self._decoded_video_snapshot(),
            'protocol_bytes_delivered': self.protocol_bytes_delivered,
            'receive_windows': {str(channel): asdict(receiver.status) for channel, receiver in self.receivers.items()},
            'telemetry': self.telemetry.snapshot(now),
            'wifi_control': self.wifi_control.snapshot(now),
            'battery': self.battery.snapshot(now),
            'wlm_bootstrap': self.wlm.snapshot() if self.wlm is not None else {'enabled': False},
            'wlm_discovery': self.wlm_discovery.snapshot(now, session_active=self.state == 'open'),
            'wlm_capability_read': self._capability_read_snapshot(now),
            'wlm_mode_follow': {'enabled':False} if self.wlm_link is None else self.wlm_link.snapshot(now),
        }


@dataclass(frozen=True)
class SessionResult:
    state: str
    reason: str
    transport_accepted: bool
    application_ready: bool
    session: int
    base_sequence: int
    duration_seconds: float
    transmitted: int
    received: int
    rejected_datagrams: int
    heartbeats_sent: int
    heartbeats_acknowledged: int
    video_bytes_delivered: int
    protocol_bytes_delivered: int
    capture_path: str
    journal_path: str
    video_path: str | None
    feature_flags: int | None = None
    delivery_mode: str | None = None
    telemetry: dict[str, Any] = field(default_factory=dict)
    wifi_control: dict[str, Any] = field(default_factory=dict)
    battery: dict[str, Any] = field(default_factory=dict)
    wlm_bootstrap: dict[str, Any] = field(default_factory=dict)
    wlm_discovery: dict[str, Any] = field(default_factory=dict)
    wlm_capability_read: dict[str, Any] = field(default_factory=lambda: {'enabled': False})
    wlm_mode_follow: dict[str, Any] = field(default_factory=lambda: {'enabled': False})
    video: dict[str, Any] = field(default_factory=lambda: {'enabled': False})
    video_publish: dict[str, Any] = field(default_factory=lambda: {'enabled': False})
    decoded_video: dict[str, Any] = field(default_factory=lambda: {'enabled': False})
    socket_cleanup_verified: bool = False
    udp_observation: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def run_session(
    *,
    capture_path: str | Path,
    host: str = "192.168.2.1",
    port: int = 9003,
    interface: str | None = None,
    duration: float = 30.0,
    heartbeat: bool = False,
    video_path: str | Path | None = None,
    stop_event: threading.Event | None = None,
    snapshot_sink: Callable[[dict[str, Any]], None] | None = None,
    wlm_identity_file: str | Path | None = None,
    wlm_capability_query: bool = False,
    wlm_mode_follow: bool = False,
    wlm_wifi_liveview: bool = False,
    wlm_forwarding_sequence: int = 1,
    camera_channel: CameraChannel | None = None,
    flight_channel: FlightChannel | None = None,
    preview_jpeg: bool = False,
    hls_directory: str | Path | None = None,
    video_nominal_fps: float = 30,
    video_codec: str = 'h264',
    video_cleanup_sink: Callable[[dict[str, Any]], None] | None = None,
    publish_config_file: str | Path | None = None,
    publish_cleanup_sink: Callable[[dict[str, Any]], None] | None = None,
    socket_cleanup_sink: Callable[[dict[str, Any]], None] | None = None,
    decoded_video: ManagedHevcDecoder | None = None,
    decoded_video_cleanup_sink: Callable[[dict[str, Any]], None] | None = None,
    preview_only: bool = False,
    network_guard: Callable[[], bool] | None = None,
) -> SessionResult:
    """Run the transport on a PC/Pi Linux socket and journal its decisions.

    Files are created exclusively to keep independent sessions separate.
    Closing is local: no undocumented remote-close packet is synthesized.
    """

    IPv4Address(host)
    if network_guard is not None and not callable(network_guard):raise ValueError('network_guard_must_be_callable')
    if type(preview_only) is not bool or (preview_only and (wlm_identity_file is None or flight_channel is not None)):
        raise ValueError('preview_requires_owned_identity_and_no_flight_channel')
    if socket_cleanup_sink is not None and not callable(socket_cleanup_sink):
        raise ValueError('socket cleanup sink must be callable')
    if decoded_video is not None and (not isinstance(decoded_video,ManagedHevcDecoder)
            or decoded_video.started or decoded_video.closed):
        raise ValueError('decoded_video_requires_unused_owned_decoder')
    if decoded_video_cleanup_sink is not None and (decoded_video is None or not callable(decoded_video_cleanup_sink)):
        raise ValueError('decoded_video_cleanup_requires_decoder_and_callable')
    if decoded_video is not None and hls_directory is not None and video_codec != 'hevc':
        raise ValueError('decoded_video_requires_hevc_source')
    if type(wlm_capability_query) is not bool or (wlm_capability_query and wlm_identity_file is None):
        raise ValueError('WLM capability query requires a boolean option and an identity file')
    if (type(wlm_mode_follow) is not bool or (wlm_mode_follow and wlm_identity_file is None)
            or (wlm_mode_follow and wlm_capability_query)):
        raise ValueError('WLM mode follow requires owned identity and excludes the separate read experiment')
    if (type(wlm_wifi_liveview) is not bool or (wlm_wifi_liveview and (
            wlm_identity_file is None or wlm_capability_query))):
        raise ValueError('Wi-Fi liveview requires owned identity and excludes the separate read experiment')
    if type(wlm_forwarding_sequence) is not int or not 0<=wlm_forwarding_sequence<=0xffffffff:
        raise ValueError('initial WLM forwarding sequence must fit uint32')
    if wlm_forwarding_sequence!=1 and (wlm_identity_file is None or not (wlm_mode_follow or wlm_wifi_liveview)):
        raise ValueError('nondefault WLM forwarding sequence requires owned persistent WLM')
    if camera_channel is not None and (not wlm_wifi_liveview or not isinstance(camera_channel,CameraChannel)):
        raise ValueError('camera commands require the owned Wi-Fi liveview session')
    if flight_channel is not None and (not wlm_wifi_liveview or not isinstance(flight_channel,FlightChannel)):
        raise ValueError('flight commands require the owned Wi-Fi service session')
    if type(preview_jpeg) is not bool or (preview_jpeg and hls_directory is None):
        raise ValueError('JPEG preview requires local HLS activation')
    if type(port) is not int or not 0 < port <= 65535:
        raise ValueError("UDP port must be between 1 and 65535")
    if not math.isfinite(duration) or not 0 < duration <= 86400:
        raise ValueError("session duration must be finite, positive and at most 86400 seconds")
    if interface and not re.fullmatch(r"[A-Za-z0-9_.-]{1,32}", interface):
        raise ValueError("invalid network interface name")
    if interface:
        validate_interface_binding(interface)
    path = Path(capture_path)
    journal_path = path.with_name(path.name + ".session.jsonl")
    video = Path(video_path) if video_path is not None else None
    if hls_directory is None and (video_nominal_fps != 30 or video_codec != 'h264'):
        raise ValueError('video decoding options require an HLS directory')
    if video_cleanup_sink is not None and (hls_directory is None or not callable(video_cleanup_sink)):
        raise ValueError('video cleanup sink requires HLS and a callable')
    if publish_config_file is not None and hls_directory is None:
        raise ValueError('video publication requires an HLS directory')
    if publish_cleanup_sink is not None and (publish_config_file is None or not callable(publish_cleanup_sink)):
        raise ValueError('publication cleanup sink requires publication and a callable')
    publication_config = (PublishConfig.load(publish_config_file,
        protected_paths=[path, journal_path, hls_directory]
            + ([video] if video is not None else [])
            + ([wlm_identity_file] if wlm_identity_file is not None else []))
        if publish_config_file is not None else None)
    if flight_channel is not None:
        flight_channel.bind_transport(host, port)
    publisher = (ManagedPublisher(publication_config, hls_directory, nominal_fps=video_nominal_fps)
        if publication_config is not None else None)
    consumer = (HlsVideo(hls_directory, nominal_fps=video_nominal_fps, input_codec=video_codec,
                        **({'preview_jpeg':True} if preview_jpeg else {}),
        protected_paths=[path, journal_path] + ([video] if video is not None else [])
            + ([wlm_identity_file] if wlm_identity_file is not None else []))
        if hls_directory is not None else None)
    if video is not None:
        if video.resolve() in (path.resolve(), journal_path.resolve()) or video.exists():
            raise ValueError("video output must be a new path distinct from capture and journal")
        video.parent.mkdir(parents=True, exist_ok=True)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() or journal_path.exists():
        raise ValueError("session capture or journal already exists; choose a new capture path")
    identity = (load_or_create_identity(wlm_identity_file,
                protected_paths=[path, journal_path] + ([video] if video is not None else []))
                if wlm_identity_file is not None else None)
    os.close(os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600))
    # The protocol owner already validates/decodes the operational fields.
    # Retain exact bytes and receipt clocks here; report generation expands
    # diagnostic annotations offline, away from UDP/control deadlines.
    transmitted = received = heartbeats_transmitted = 0
    started = time.monotonic()
    costs = OwnerCosts()
    receipt_observer = None
    deadline = started + duration
    peer = f"{host}:{port}"
    with ExitStack() as stack:
        recorder=stack.enter_context(CaptureRecorder(path,decode_annotations=False,persistent=True))
        journal = stack.enter_context(os.fdopen(
            os.open(journal_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600),
            'w', encoding='utf-8',
        ))
        video_file = stack.enter_context(os.fdopen(
            os.open(video, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600), 'wb',
        )) if video is not None else None
        decoded_cleaned = False
        def close_decoded_video():
            nonlocal decoded_cleaned
            if decoded_video is None or decoded_cleaned:return
            try:decoded_video.close()
            finally:
                snapshot=decoded_video.status()
                decoded_cleaned=snapshot['cleanup_verified']
                if decoded_video_cleanup_sink is not None:decoded_video_cleanup_sink(snapshot)
        if decoded_video is not None:
            stack.callback(close_decoded_video)
            decoded_video.start()
        publication_cleaned = False
        def close_publication():
            nonlocal publication_cleaned
            if publication_cleaned or publisher is None:
                return
            try:
                publisher.close()
            finally:
                snapshot = publisher.snapshot()
                publication_cleaned = snapshot['cleanup_verified']
                if publish_cleanup_sink is not None:
                    publish_cleanup_sink(snapshot)
        if publisher is not None:
            # Registered first: decoder closes/finalizes its last segment
            # before publication flushes and acknowledges remote stop.
            stack.callback(close_publication)
        video_cleaned = False
        def close_video():
            nonlocal video_cleaned
            if video_cleaned or consumer is None:
                return
            try:
                consumer.close()
            finally:
                snapshot = consumer.snapshot()
                video_cleaned = snapshot['cleanup_verified']
                if video_cleanup_sink is not None:
                    video_cleanup_sink(snapshot)
        if consumer is not None:
            # Register before starting: startup/cleanup errors must also
            # report ownership independently of later journal failures.
            stack.callback(close_video)
            consumer.__enter__()
        if publisher is not None:
            publisher.start()
        def record_event(event: dict[str, Any]) -> None:
            with costs.measure('protocol_journal'):
                journal.write(json.dumps(event, separators=(",", ":")) + "\n")
                journal.flush()

        def record_payload(channel: int, payload: bytes) -> None:
            if channel == 2 and video_file is not None:
                with costs.measure('raw_video_write'):
                    video_file.write(payload)
            if channel == 2 and consumer is not None:
                with costs.measure('video_enqueue'):
                    consumer.feed(payload)

        socket_connected=False
        socket_owner=threading.get_ident()
        def wifi_backend_ready():
            if not socket_connected or threading.get_ident()!=socket_owner:return False
            try:
                return (sock.fileno()>=0 and sock.getpeername()==(host,port)
                    and (video_file is None or not video_file.closed))
            except OSError:return False

        client = SwudpSession(heartbeat=heartbeat, event_sink=record_event, payload_sink=record_payload,
                              **({'preview_only':True} if preview_only else {}),
                              **({'decoded_video':decoded_video} if decoded_video is not None else {}),
                              wlm_identity=identity, physical_wifi_port=port,
                              **({'wlm_capability_query':True} if wlm_capability_query else {}),
                              **({'wlm_mode_follow':True} if wlm_mode_follow else {}),
                              **({'wlm_forwarding_sequence':wlm_forwarding_sequence}
                                 if wlm_forwarding_sequence!=1 else {}),
                              **({'wlm_wifi_liveview':True,'wifi_backend_ready':wifi_backend_ready}
                                 if wlm_wifi_liveview else {}),
                              **({'camera_channel':camera_channel} if camera_channel is not None else {}),
                              **({'flight_channel':flight_channel} if flight_channel is not None else {}),
                              **({'video_keyframe_needed':lambda:not consumer.hevc_filter.bootstrap_ready}
                                 if consumer is not None and video_codec=='hevc' and camera_channel is not None else {}))
        next_snapshot_at = 0.0
        published_state = None

        def video_snapshot(now=None):
            result=consumer.snapshot(now) if consumer is not None else {'enabled':False}
            if client.video_keyframes is not None:
                result['keyframe_requests']=client._video_keyframe_snapshot(time.monotonic() if now is None else now)
            return result

        def publish(*, force=False):
            nonlocal next_snapshot_at, published_state
            if publisher is not None:
                publisher.poll()
            if snapshot_sink is None:
                return
            now = time.monotonic()
            signature = client.state, client.reason
            if not force and signature == published_state and now < next_snapshot_at:
                return
            snapshot = client.snapshot(now)
            snapshot['video'] = video_snapshot(now)
            snapshot['video_publish'] = publisher.snapshot(now) if publisher is not None else {'enabled': False}
            snapshot['udp_observation'] = {'receive':receipt_observer.snapshot()
                if receipt_observer is not None else {}, 'owner_costs':costs.snapshot()}
            snapshot.update(transmitted=transmitted, received=received,
                            capture_path=str(path), journal_path=str(journal_path),
                            video_path=str(video) if video is not None else None)
            try:
                snapshot_sink(snapshot)
            except Exception:
                if client.state == 'closed':
                    record_event({'event': 'snapshot_consumer_failed', 'state': client.state})
                    raise RuntimeError('snapshot_consumer_failed') from None
                client.fail(time.monotonic(), 'snapshot_consumer_failed')
            next_snapshot_at, published_state = now + .1, signature

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            receipt_observer = UdpReceiptObserver(sock)
            if socket_cleanup_sink is not None:
                stack.callback(lambda:socket_cleanup_sink({'cleanup_verified':sock.fileno()<0}))
            if interface and not (stop_event is not None and stop_event.is_set()):
                bind_ipv4_socket(sock, interface)

            def send(datagram: OutboundDatagram) -> bool:
                nonlocal transmitted, heartbeats_transmitted
                if network_guard is not None and network_guard() is not True:
                    client.fail(time.monotonic(), 'camera_network_identity_changed_or_stale')
                    return False
                if stop_event is not None and stop_event.is_set():
                    client.close(time.monotonic(), 'requested')
                    return False
                with costs.measure('pre_send_guard'):
                    allowed = client.datagram_send_allowed(datagram,time.monotonic())
                if not allowed:
                    return True
                try:
                    with costs.measure('socket_send'):
                        sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, datagram.tos)
                        send_started_ns=time.monotonic_ns()
                        count=sock.send(datagram.raw)
                        send_completed_ns=time.monotonic_ns()
                    if count != len(datagram.raw):
                        client.fail(time.monotonic(), "short_datagram_send")
                        return False
                except OSError:
                    client.fail(time.monotonic(), "network_error")
                    return False
                transmitted += 1
                with costs.measure('send_receipt'):
                    client.datagram_sent(datagram, send_completed_ns/1e9)
                if datagram.application_kind == 'heartbeat':
                    heartbeats_transmitted += 1
                with costs.measure('capture_append'):
                    recorder.record(interface="udp", direction="tx", peer=peer,
                        raw=datagram.raw, note=datagram.purpose,
                        observation={'send_started_monotonic_ns':send_started_ns,
                                     'send_completed_monotonic_ns':send_completed_ns})
                return True

            try:
                if stop_event is not None and stop_event.is_set():
                    client.close(time.monotonic(), 'requested')
                else:
                    sock.connect((host, port))
                    socket_connected=True
                    send(client.start(time.monotonic()))
                publish(force=True)
                while not client.terminal and time.monotonic() < deadline:
                    if stop_event is not None and stop_event.is_set():
                        client.close(time.monotonic(), 'requested')
                        break
                    with costs.measure('tick'):
                        outgoing=client.tick(time.monotonic())
                    for datagram in outgoing:
                        if not send(datagram):
                            break
                    with costs.measure('snapshot'):
                        publish()
                    if client.terminal:
                        break
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    sock.settimeout(min(0.025, remaining))
                    try:
                        raw, received_ns, received_wall_ns, receipt = receipt_observer.receive()
                    except socket.timeout:
                        continue
                    except OSError:
                        client.fail(time.monotonic(), "network_error")
                        break
                    received += 1
                    # Publish protocol state before capture disk I/O,
                    # retaining the actual socket receipt time for both
                    # deadlines and the immutable capture. Annotation is
                    # deferred to report generation.
                    try:
                        with costs.measure('protocol_delivery'):
                            client.receive(raw, received_ns/1e9)
                    finally:
                        with costs.measure('capture_append'):
                            recorder.record(interface="udp", direction="rx", peer=peer, raw=raw,
                                timestamp_ns=received_wall_ns,monotonic_ns=received_ns,
                                observation={**receipt,'protocol_delivery_ns':
                                    costs.values['protocol_delivery']['last_ns']})
                client.close(time.monotonic(), "duration_elapsed")
            except KeyboardInterrupt:
                client.close(time.monotonic(), "interrupted")
            except OSError:
                client.fail(time.monotonic(), "network_error")
                raise
        if consumer is not None:
            close_video()
        if publisher is not None:
            close_publication()
        if decoded_video is not None:
            close_decoded_video()
        result = SessionResult(
            state=client.state,
            reason=client.reason or "unknown",
            transport_accepted=client.accepted_at is not None,
            application_ready=False,
            session=client.transport.session,
            base_sequence=client.transport.base_sequence,
            duration_seconds=time.monotonic() - started,
            transmitted=transmitted,
            received=received,
            rejected_datagrams=client.rejected_datagrams,
            heartbeats_sent=heartbeats_transmitted,
            heartbeats_acknowledged=client.heartbeats_acknowledged,
            video_bytes_delivered=client.video_bytes_delivered,
            protocol_bytes_delivered=client.protocol_bytes_delivered,
            capture_path=str(path),
            journal_path=str(journal_path),
            video_path=str(video) if video is not None else None,
            feature_flags=client.feature_flags,
            delivery_mode=client.delivery_mode,
            telemetry=client.telemetry.snapshot(time.monotonic()),
            wifi_control=client.wifi_control.snapshot(time.monotonic()),
            battery=client.battery.snapshot(time.monotonic()),
            wlm_bootstrap=client.wlm.snapshot() if client.wlm is not None else {'enabled': False},
            wlm_discovery=client.wlm_discovery.snapshot(time.monotonic(), session_active=False),
            wlm_capability_read=client._capability_read_snapshot(time.monotonic()),
            wlm_mode_follow={'enabled':False} if client.wlm_link is None else client.wlm_link.snapshot(time.monotonic()),
            video=video_snapshot(),
            video_publish=publisher.snapshot() if publisher is not None else {'enabled': False},
            decoded_video=client._decoded_video_snapshot(),
            socket_cleanup_verified=sock.fileno()<0,
            udp_observation={'receive':receipt_observer.snapshot(), 'owner_costs':costs.snapshot()},
        )
        record_event({"event": "session_result", **result.as_dict()})
        publish(force=True)
        return result
