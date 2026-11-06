"""Bounded PC implementation of Neo 2 BLE verification and Wi-Fi requests.

FFF5 writes and FFF4 notifications carry ordinary DUML v1 frames. Receiving
network credentials is distinct from Wi-Fi association or application/flight
readiness. An optional WLM identity adds the native raw link report after
permission ALLOW. This standalone setup closes BLE before returning.
"""

from __future__ import annotations

import asyncio
from contextlib import AsyncExitStack, asynccontextmanager, nullcontext
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
import threading
import time
from typing import Any, Awaitable, Callable

from .ble import _load_bleak_client, temporary_connection_intervals
from .platform_transport import normalize_ble_identifier
from .capture import CaptureRecorder
from .duml import DumlPacket, DumlSequence
from .duml_stream import DumlStreamDecoder
from .identity import (BleClientIdentity, load_or_create_ble_identity,
                       load_or_create_identity, validate_identity_path)


SERVICE_UUID = '0000fff0-0000-1000-8000-00805f9b34fb'
NOTIFY_UUID = '0000fff4-0000-1000-8000-00805f9b34fb'
WRITE_UUID = '0000fff5-0000-1000-8000-00805f9b34fb'
STEPS = ((0x45, 'identify_client'), (0x0C, 'read_bssid'),
         (0x07, 'read_ssid'), (0x0E, 'read_passphrase'))
CREDENTIALS_FORMAT = 'neo2-control/wifi-credentials/v1'
# DJIApplicationConfigs.m() -> WIFI_LINK_MC_SERVICE_PORT, DJI Fly1.19.0.
# This is a static provider value, not a measurement or inference from UDP.
DJI_FLY_WLM_SERVICE_PORT = 9003
BLE_DISCONNECT_TIMEOUT = 3


class BleSetupError(RuntimeError):
    """A reason code without peer-supplied or private payload text."""


def setup_request(command: int, sequence: int, identity: BleClientIdentity) -> DumlPacket:
    if command not in {item[0] for item in STEPS}:
        raise ValueError('command is not part of the measured BLE setup')
    payload = (b'\x20' + identity.identification_id.encode('ascii') + b'\x07DJI Fly'
               if command == 0x45 else b'')
    return DumlPacket(sender=2, receiver=7, seq=sequence & 0xFFFF,
                      cmd_type=0x40, cmd_set=7, cmd_id=command, payload=payload)


def _reply_value(command: int, payload: bytes) -> bytes:
    if not payload or payload[0] != 0:
        raise BleSetupError('peer_status_error')
    if command == 0x45:
        if payload == b'\x00\x01':
            return b''
        reasons = {b'\x00\x02': 'verification_needs_request',
                   b'\x00\x03': 'verification_device_busy',
                   b'\x00\x04': 'verification_high_temperature'}
        raise BleSetupError(reasons.get(payload, 'unsupported_identification_reply'))
    if command == 0x0C:
        if len(payload) != 7:
            raise BleSetupError('malformed_bssid_reply')
        return payload[1:]
    maximum = 32 if command == 7 else 255
    if len(payload) < 2 or not 1 <= payload[1] <= maximum or len(payload) != payload[1] + 2:
        raise BleSetupError('malformed_counted_reply')
    return payload[2:]


@dataclass(frozen=True, repr=False)
class WifiCredentials:
    bssid: bytes
    ssid: bytes
    passphrase: bytes

    def __post_init__(self):
        if not (isinstance(self.bssid, bytes) and len(self.bssid) == 6
                and isinstance(self.ssid, bytes) and 1 <= len(self.ssid) <= 32
                and isinstance(self.passphrase, bytes) and 1 <= len(self.passphrase) <= 255):
            raise ValueError('invalid Wi-Fi credential record')

    def __repr__(self):
        return 'WifiCredentials(<private>)'

    def metadata(self):
        return {'network_sha256': hashlib.sha256(self.bssid + self.ssid).hexdigest(),
                'ssid_bytes': len(self.ssid), 'passphrase_bytes': len(self.passphrase)}

    def save_new(self, path: str | Path):
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        record = {'format': CREDENTIALS_FORMAT, 'bssid_hex': self.bssid.hex(),
                  'ssid_hex': self.ssid.hex(), 'passphrase_hex': self.passphrase.hex()}
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=target.parent, prefix='.neo2-wifi-',
                                             mode='w', encoding='utf-8', delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(json.dumps(record) + '\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.link(temporary, target)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)


def load_wifi_credentials(path: str | Path) -> WifiCredentials:
    with Path(path).open('rb') as stream:
        raw = stream.read(4097)
    try:
        record = json.loads(raw)
        if (len(raw) > 4096 or not isinstance(record, dict)
            or set(record) != {'format', 'bssid_hex', 'ssid_hex', 'passphrase_hex'}):
            raise ValueError
        if record['format'] != CREDENTIALS_FORMAT:
            raise ValueError
        return WifiCredentials(*(bytes.fromhex(record[key]) for key in ('bssid_hex', 'ssid_hex', 'passphrase_hex')))
    except (ValueError, TypeError, KeyError, AttributeError) as error:
        raise ValueError('invalid private Wi-Fi credential store') from error


@dataclass(frozen=True)
class BleSetupResult:
    state: str
    reason: str
    stage: str
    requests_queued: int
    responses_received: int
    write_chunks: int
    notifications_received: int
    ignored_frames: int
    discarded_bytes: int
    verification_acks: int
    identity_sha256: str
    network: dict[str, Any] | None
    capture_path: str
    journal_path: str
    credentials_path: str | None
    wlm_report: dict[str, Any] | None = None
    connection_retained: bool = False
    disconnected_during_hold: bool = False
    connection_cleanup_verified: bool = True

    def as_dict(self):
        # Keep raw credentials out of result objects and the CLI entirely.
        return {**self.__dict__, 'application_ready': False}


class _Exchange:
    def __init__(self, recorder, event, request_timeout, packet_guard=None):
        self.packet_guard = packet_guard
        self.recorder, self.event, self.request_timeout = recorder, event, request_timeout
        self.decoder = DumlStreamDecoder()
        self.pending = None
        self.expected = None
        self.failure = None
        self.finished = False
        self.notify_characteristic = None
        self.verification_pushes = asyncio.Queue(maxsize=64)
        self.verification_active = False
        self.verification_result = None
        self.verification_acks = 0
        self.wlm_report = None
        self.holding = self.was_retained = self.disconnected_during_hold = False
        self.notification_clock = time.monotonic
        self.window_started = self.notification_clock()
        self.window_bytes = self.window_notifications = 0
        self.requests_queued = self.responses_received = self.write_chunks = 0
        self.notifications_received = self.ignored_frames = self.received_bytes = 0

    def fail(self, reason):
        if self.failure is None:
            self.failure = reason
        if self.pending is not None and not self.pending.done():
            self.pending.set_exception(BleSetupError(self.failure))

    def disconnected(self, _client):
        if self.finished:
            return
        if self.holding:
            self.disconnected_during_hold = True
            try:
                self.event('ble_disconnected_during_hold')
            except Exception:
                self.fail('notification_capture_or_framing_error')
        else:
            self.fail('bluetooth_disconnected')

    def notification(self, characteristic, data):
        if (self.failure is not None or self.finished
            or (self.holding and self.disconnected_during_hold)):
            return
        self.notifications_received += 1
        self.received_bytes += len(data)
        if self.holding:
            # The setup bounds apply per minute during a long retained link;
            # counters remain cumulative, but normal traffic cannot exhaust a
            # one-shot setup budget simply because the session lasts longer.
            now = self.notification_clock()
            if now - self.window_started >= 60:
                self.window_started, self.window_bytes, self.window_notifications = now, 0, 0
            self.window_bytes += len(data)
            self.window_notifications += 1
            size, count = self.window_bytes, self.window_notifications
        else:
            size, count = self.received_bytes, self.notifications_received
        if len(data) > 512 or size > 1024 * 1024 or count > 4096:
            self.fail('notification_limit_exceeded')
            return
        try:
            self.recorder.record(interface='ble-gatt', direction='rx', peer=NOTIFY_UUID,
                                 raw=bytes(data), note='notification callback bytes')
            target = self.notify_characteristic
            if target is None or characteristic.handle != target.handle:
                self.ignored_frames += 1
                return
            for packet, raw in self.decoder.feed(bytes(data)):
                self.recorder.record(interface='ble-duml', direction='rx', peer=NOTIFY_UUID,
                                     raw=raw, note='reassembled notification frame')
                if ((packet.sender, packet.receiver, packet.version, packet.cmd_set, packet.cmd_id)
                    == (7, 2, 1, 7, 0x46)):
                    # The SDK observer dispatches on version/set/id, without
                    # correlating the request sequence or filtering cmd_type.
                    # Keep writes out of this synchronous notification callback.
                    try:
                        self.verification_pushes.put_nowait((packet, self.verification_active))
                    except asyncio.QueueFull:
                        self.fail('verification_notification_limit_exceeded')
                    continue
                expected = self.expected
                report = self.wlm_report
                if (report is not None and
                    (packet.sender, packet.receiver, packet.seq, packet.cmd_type,
                     packet.cmd_set, packet.cmd_id, packet.version)
                    == (7, 2, report['sequence'], 0xC0, 7, 0x93, 1)):
                    # Native callbacks log body[0]; they neither gate the next
                    # report nor establish application readiness.
                    if not packet.payload:
                        report['malformed_responses'] += 1
                        self.event('wlm_report_response_rejected', reason='missing_status')
                    elif report['response_status'] is None:
                        report['response_status'] = packet.payload[0]
                        self.event('wlm_report_response', sequence=packet.seq,
                                   status=packet.payload[0], application_ready=False)
                    continue
                if (expected is not None and self.pending is not None and not self.pending.done()
                    and (packet.sender, packet.receiver, packet.seq, packet.cmd_type,
                         packet.cmd_set, packet.cmd_id, packet.version)
                    == (7, 2, expected.seq, 0xC0, 7, expected.cmd_id, 1)):
                    self.responses_received += 1
                    self.pending.set_result(packet)
                else:
                    self.ignored_frames += 1
        except Exception:
            self.fail('notification_capture_or_framing_error')

    async def _write(self, client, writer, packet, *, kind='request'):
        if self.packet_guard is not None:self.packet_guard(packet)
        raw = packet.encode()
        self.recorder.record(interface='ble-duml', direction='prepared', peer=WRITE_UUID,
                             raw=raw, note=f'assembled {kind}; GATT writes follow')
        self.event(kind + '_prepared', command=packet.cmd_id, sequence=packet.seq)
        offset = 0
        while offset < len(raw):
            if self.failure is not None:
                raise BleSetupError(self.failure)
            if not client.is_connected:
                raise BleSetupError('bluetooth_disconnected')
            limit = writer.max_write_without_response_size
            if type(limit) is not int or not 20 <= limit <= 512:
                raise BleSetupError('unsupported_write_payload_limit')
            chunk = raw[offset:offset + limit]
            await client.write_gatt_char(writer, chunk, response=False)
            self.write_chunks += 1
            self.recorder.record(interface='ble-gatt', direction='tx', peer=WRITE_UUID,
                                 raw=chunk, note='write returned; radio delivery unconfirmed')
            offset += len(chunk)
        if kind == 'verification_ack':
            self.verification_acks += 1
        elif kind == 'request':
            self.requests_queued += 1
        elif kind == 'wlm_report':
            self.wlm_report['queued'] = True
        self.event(kind + '_queued', command=packet.cmd_id, sequence=packet.seq)

    async def report_wlm_identity(self, client, writer, identity, sequence):
        packet = identity.link_report(sequence, physical_port=DJI_FLY_WLM_SERVICE_PORT)
        self.wlm_report = {'identity_sha256': identity.fingerprint,
                           'physical_port': DJI_FLY_WLM_SERVICE_PORT,
                           'port_source': 'DJI-Fly-1.19.0-compiled-provider',
                           'sequence': packet.seq, 'queued': False,
                           'response_status': None, 'malformed_responses': 0,
                           'application_ready': False}
        try:
            async with asyncio.timeout(self.request_timeout):
                await self._write(client, writer, packet, kind='wlm_report')
                if self.failure is not None:
                    raise BleSetupError(self.failure)
        except TimeoutError as error:
            raise BleSetupError('wlm_report_write_timeout') from error

    async def _verification_push(self, client, writer, packet, active_at_receipt):
        # ACK before delivering the decision, even without a verification
        # callback. Its sequence belongs to the push, including sequence zero.
        ack = DumlPacket(2, 7, packet.seq, 0xC0, 7, 0x46, b'\x00')
        try:
            async with asyncio.timeout(self.request_timeout):
                await self._write(client, writer, ack, kind='verification_ack')
        except TimeoutError as error:
            raise BleSetupError('verification_ack_timeout') from error
        active = active_at_receipt and self.verification_active
        self.event('verification_push', sequence=packet.seq,
                   result=packet.payload[0] if packet.payload else None,
                   area_code_requested=len(packet.payload) > 1 and bool(packet.payload[1] & 1),
                   area_code_action='not_sent')
        if active:
            if not packet.payload:
                raise BleSetupError('malformed_verification_push')
            self.verification_result = packet.payload
            self.verification_active = False

    async def _wait(self, client, writer, *, response):
        while True:
            if self.failure is not None:
                raise BleSetupError(self.failure)
            # Drain pushes before releasing a reply or a decision. This also
            # handles several complete frames in one GATT notification.
            while not self.verification_pushes.empty():
                await self._verification_push(client, writer, *self.verification_pushes.get_nowait())
            if response and self.pending.done():
                return self.pending.result()
            if not response and self.verification_result is not None:
                return self.verification_result
            push = asyncio.create_task(self.verification_pushes.get())
            try:
                await asyncio.wait((self.pending, push), return_when=asyncio.FIRST_COMPLETED)
                if push.done():
                    await self._verification_push(client, writer, *push.result())
            finally:
                if not push.done():
                    push.cancel()
                await asyncio.gather(push, return_exceptions=True)

    def _clear_pending(self):
        future, self.pending = self.pending, None
        self.expected = None
        if future is not None:
            if not future.done():
                future.cancel()
            elif not future.cancelled():
                future.exception()  # Retrieve a disconnect/capture exception on a failed write.

    async def request(self, client, writer, packet):
        if self.failure is not None:
            raise BleSetupError(self.failure)
        if not client.is_connected:
            raise BleSetupError('bluetooth_disconnected')
        self.pending = asyncio.get_running_loop().create_future()
        self.expected = packet
        try:
            async with asyncio.timeout(self.request_timeout):
                await self._write(client, writer, packet)
                return await self._wait(client, writer, response=True)
        except TimeoutError as error:
            raise BleSetupError('response_timeout') from error
        finally:
            self._clear_pending()

    async def verify(self, client, writer, packet):
        self.verification_active = True
        try:
            reply = await self.request(client, writer, packet)
            if reply.payload != b'\x00\x02':
                value = _reply_value(0x45, reply.payload)
                if self.verification_result is not None and self.verification_result[0] != 1:
                    raise BleSetupError('verification_conflicting_result')
                return reply, value
            self.event('verification_requested')
            # Human validation is bounded by the whole setup deadline, rather
            # than the short request/response deadline. No second request45/46.
            self.pending = asyncio.get_running_loop().create_future()
            try:
                result = await self._wait(client, writer, response=False)
            finally:
                self._clear_pending()
            if result[0] != 1:
                reasons = {0: 'verification_denied', 2: 'verification_timeout',
                           3: 'verification_needs_request', 4: 'verification_device_busy',
                           5: 'verification_high_temperature'}
                raise BleSetupError(reasons.get(result[0], 'unsupported_verification_result'))
            return reply, b''
        finally:
            self.verification_active = False

    async def retain(self, client, writer, callback, stop_event, duration):
        """Keep callbacks alive while one owned Wi-Fi task runs to cleanup.

        A callback wrapping a thread must not be cancelled and abandoned. On
        exit we signal the shared stop event and await its resource cleanup.
        """
        self.holding = self.was_retained = True
        self.window_started = self.notification_clock()
        self.window_bytes = self.window_notifications = 0
        self.event('ble_retained')
        task = asyncio.create_task(callback())
        try:
            async with asyncio.timeout(duration):
                while not task.done():
                    if stop_event.is_set():
                        raise asyncio.CancelledError
                    if self.failure is not None:
                        raise BleSetupError(self.failure)
                    while not self.verification_pushes.empty():
                        packet, active = self.verification_pushes.get_nowait()
                        if self.disconnected_during_hold or not client.is_connected:
                            continue
                        try:
                            await self._verification_push(client, writer, packet, active)
                        except Exception:
                            if client.is_connected:
                                raise
                            self.disconnected_during_hold = True
                    # asyncio.wait never transfers cancellation to the task
                    # that owns activation, socket reception and NM cleanup.
                    await asyncio.wait((task,), timeout=.05)
                await task
                if self.failure is not None:
                    raise BleSetupError(self.failure)
        except TimeoutError as error:
            stop_event.set()
            raise BleSetupError('handoff_timeout') from error
        finally:
            if not task.done():
                stop_event.set()
            await asyncio.gather(asyncio.shield(task), return_exceptions=True)
            self.holding = False
            self.event('ble_retention_finished', disconnected=self.disconnected_during_hold)


@asynccontextmanager
async def _bounded_ble_connection(connection):
    client = await connection.__aenter__()
    async with AsyncExitStack() as resources:
        async def close(*exception):
            try:
                async with asyncio.timeout(BLE_DISCONNECT_TIMEOUT):
                    return await connection.__aexit__(*exception)
            except TimeoutError as error:
                raise BleSetupError('disconnect_timeout') from error
        resources.push_async_exit(close)
        yield client


def _check_new_paths(paths, identity_path):
    all_paths = [Path(p) for p in (*paths, identity_path)]
    for index, first in enumerate(all_paths):
        for second in all_paths[index + 1:]:
            if (first.resolve() == second.resolve()
                or (first.exists() and second.exists() and first.samefile(second))):
                raise ValueError('identity, credentials, capture and journal must use distinct paths')
    for path in all_paths[:-1]:
        if path.exists() or path.is_symlink():
            raise ValueError('setup output already exists; choose new paths')


async def run_ble_wifi_setup(
    *, address: str, identity_file: str | Path, credentials_path: str | Path,
    capture_path: str | Path, timeout: float = 30, request_timeout: float = 3,
    interval_min_units: int | None = None, interval_max_units: int | None = None,
    wlm_identity_file: str | Path | None = None,
    while_connected: Callable[[], Awaitable[None]] | None = None,
    hold_timeout: float | None = None,
    stop_event: threading.Event | None = None,
    event_sink: Callable[..., None] | None = None,
    preview_only: bool = False,
) -> BleSetupResult:
    if type(preview_only) is not bool:raise ValueError('preview_only_must_be_boolean')
    address = normalize_ble_identifier(address)
    if not math.isfinite(timeout) or not 0 < timeout <= 120:
        raise ValueError('setup timeout must be finite and between 0 and 120 seconds')
    if not math.isfinite(request_timeout) or not 0 < request_timeout <= timeout:
        raise ValueError('request timeout must be finite, positive and no greater than setup timeout')
    if (interval_min_units is None) != (interval_max_units is None):
        raise ValueError('both BLE interval bounds are required')
    if interval_min_units is not None and not 6 <= interval_min_units <= interval_max_units <= 3200:
        raise ValueError('BLE interval units must satisfy 6 <= minimum <= maximum <= 3200')
    if while_connected is not None:
        if (not callable(while_connected) or stop_event is None or hold_timeout is None
            or not math.isfinite(hold_timeout) or not 0 < hold_timeout <= 86520):
            raise ValueError('retained BLE requires a callback, stop event and bounded hold timeout')
    elif hold_timeout is not None:
        raise ValueError('hold timeout requires a retained BLE callback')
    path, credentials = Path(capture_path), Path(credentials_path)
    journal_path = path.with_name(path.name + '.setup.jsonl')
    _check_new_paths((path, journal_path, credentials), identity_file)
    protected = [path, journal_path, credentials, identity_file]
    if wlm_identity_file is not None:
        validate_identity_path(wlm_identity_file, protected)
    identity = load_or_create_ble_identity(identity_file)
    wlm_identity = (load_or_create_identity(wlm_identity_file, protected_paths=protected)
                    if wlm_identity_file is not None else None)
    path.parent.mkdir(parents=True, exist_ok=True)
    # O_EXCL and mode 0600 apply before any Bluetooth activity.
    with os.fdopen(os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600), 'w'):
        pass
    started = time.monotonic()
    state, reason, stage, network = 'failed', 'bluetooth_error', 'connect', None
    credentials_saved = False
    with os.fdopen(os.open(journal_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600), 'w') as journal:
        def event(name, **detail):
            # Resource ownership must update even when either journal fails.
            if event_sink is not None:
                event_sink(name, **detail)
            journal.write(json.dumps({'event': name, 'elapsed_seconds': time.monotonic() - started,
                                      **detail}, separators=(',', ':')) + '\n')
            journal.flush()

        guard = None
        if preview_only:
            from .preview_policy import validate_preview_ble
            guard = lambda packet: validate_preview_ble(packet, identity, wlm_identity)
        exchange = _Exchange(CaptureRecorder(path), event, request_timeout, packet_guard=guard)
        intervals = (temporary_connection_intervals(minimum_units=interval_min_units,
                                                    maximum_units=interval_max_units)
                     if interval_min_units is not None else nullcontext())
        owner = asyncio.current_task()
        connection = None
        cleanup_verified = True
        async def cancel_on_stop():
            while not stop_event.is_set():
                await asyncio.sleep(.05)
            # Retention owns a worker thread: let it signal and join that
            # worker before leaving the BLE context, including on timeout.
            if not exchange.was_retained:
                owner.cancel()
        watcher = asyncio.create_task(cancel_on_stop()) if stop_event is not None else None
        try:
            if stop_event is not None and stop_event.is_set():
                raise asyncio.CancelledError
            async with asyncio.timeout(timeout) as setup_deadline:
                with intervals:
                    factory = _load_bleak_client()
                    connection = factory(address, timeout=timeout, pair=False,
                                         disconnected_callback=exchange.disconnected)
                    async with _bounded_ble_connection(connection) as client:
                        writer = client.services.get_characteristic(WRITE_UUID)
                        notifier = client.services.get_characteristic(NOTIFY_UUID)
                        if (writer is None or notifier is None
                            or writer.service_uuid.lower() != SERVICE_UUID
                            or notifier.service_uuid.lower() != SERVICE_UUID
                            or 'write-without-response' not in writer.properties
                            or 'notify' not in notifier.properties):
                            raise BleSetupError('unsupported_gatt_profile')
                        exchange.notify_characteristic = notifier
                        await client.start_notify(notifier, exchange.notification)
                        event('notifications_enabled')
                        sequence, values = DumlSequence(), {}
                        for command, stage in STEPS:
                            event('stage_started', stage=stage)
                            packet = setup_request(command, sequence.next(), identity)
                            if command == 0x45:
                                reply, values[command] = await exchange.verify(client, writer, packet)
                            else:
                                reply = await exchange.request(client, writer, packet)
                                values[command] = _reply_value(command, reply.payload)
                            event('reply_accepted', command=command, sequence=reply.seq)
                            if command == 0x45 and wlm_identity is not None:
                                # PC scheduling choice after permission; this
                                # does not assert DJI Fly's radio chronology.
                                stage = 'report_wlm_identity'
                                event('stage_started', stage=stage)
                                await exchange.report_wlm_identity(client, writer, wlm_identity,
                                                                   sequence.next())
                        stage = 'save_credentials'
                        record = WifiCredentials(values[0x0C], values[7], values[0x0E])
                        record.save_new(credentials)
                        credentials_saved = True
                        network = record.metadata()
                        if while_connected is not None:
                            stage = 'wifi_session'
                            setup_deadline.reschedule(None)
                            await exchange.retain(client, writer, while_connected, stop_event, hold_timeout)
                            stage = 'ble_cleanup'
                        exchange.finished = True
                state, reason, stage = 'complete', 'credentials_saved', 'complete'
        except BleSetupError as error:
            reason = str(error)
        except TimeoutError:
            reason = 'setup_timeout'
        except OSError:
            reason = 'io_or_bluetooth_error'
        except asyncio.CancelledError:
            event('setup_cancelled', stage=stage)
            raise
        except Exception as error:
            # Backend error strings can contain peer/private data. Keep a
            # reason code; the raw callbacks remain in the private capture.
            wifi_reasons={'manually_join_aircraft_wifi_before_session',
                          'macos_wifi_identity_requires_location_permission',
                          'macos_wifi_identity_unavailable','macos_wifi_interface_unavailable'}
            if stage=='wifi_session':
                reason=str(error) if isinstance(error,RuntimeError) and str(error) in wifi_reasons else 'wifi_session_failed'
                event('wifi_session_failed',reason=reason,error_type=type(error).__name__)
            else:reason = 'bluetooth_error'
        finally:
            if watcher is not None:
                watcher.cancel()
                await asyncio.gather(watcher, return_exceptions=True)
            exchange.finished = True
            if connection is not None:
                try:
                    if connection.is_connected:
                        async with asyncio.timeout(BLE_DISCONNECT_TIMEOUT):
                            await connection.disconnect()
                    cleanup_verified = not connection.is_connected
                except Exception:
                    cleanup_verified = False
            event('ble_cleanup_result', cleanup_verified=cleanup_verified)
        if not cleanup_verified:
            state, reason = 'failed', 'cleanup_unconfirmed'
        result = BleSetupResult(
            state, reason, stage, exchange.requests_queued, exchange.responses_received,
            exchange.write_chunks, exchange.notifications_received, exchange.ignored_frames,
            exchange.decoder.discarded_bytes, exchange.verification_acks,
            str(identity.metadata()['identity_sha256']),
            network, str(path), str(journal_path), str(credentials) if credentials_saved else None,
            dict(exchange.wlm_report) if exchange.wlm_report is not None else None,
            exchange.was_retained, exchange.disconnected_during_hold,
            cleanup_verified,
        )
        event('setup_result', **result.as_dict())
        return result


def run_ble_wifi_setup_sync(**kwargs):
    return asyncio.run(run_ble_wifi_setup(**kwargs))
