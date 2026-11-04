"""One managed, cancellable PC receive session with detached public snapshots.

The peer and interface are fixed by local configuration. No flight command
producer is available here. Optional BLE credentials acquisition retains its
connection while the temporary Wi-Fi profile and bounded SWUDP client run.
"""

from copy import deepcopy
import asyncio
from contextlib import ExitStack
from dataclasses import dataclass, field
import math
from pathlib import Path
import re
import secrets
import threading
import time

from .session import run_session
from .wifi_status import age_wifi_control
from .battery import age_battery
from .wlm_discovery import age_discovery
from .ble_bootstrap import load_wifi_credentials, run_ble_wifi_setup
from .controller import DecisionJournal
from .identity import load_or_create_identity, load_or_create_ble_identity, validate_identity_path
from .video import HlsVideo, VideoError
from .video_publish import PublishConfig, PublishError
from .wifi_link import (DRONE_HOST, DedicatedWifiLink, WifiLinkCancelled, WifiLinkError,
                        prepare_wifi_plan, require_dedicated_drone_route, require_available_wifi_interface)


class ObservationError(RuntimeError):
    pass


@dataclass(frozen=True)
class ObservationConfig:
    interface: str
    maximum_duration: float = 600.0
    heartbeat: bool = False
    record_video: bool = False
    credentials_file: str | Path | None = field(default=None, repr=False)
    wlm_identity_file: str | Path | None = field(default=None, repr=False)
    ble_address: str | None = field(default=None, repr=False)
    ble_identity_file: str | Path | None = field(default=None, repr=False)
    ble_setup_timeout: float = 30.0
    record_hls: bool = False
    video_nominal_fps: float = 30.0
    video_codec: str = 'h264'
    publish_config_file: str | Path | None = field(default=None, repr=False)

    def __post_init__(self):
        if not isinstance(self.interface, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', self.interface):
            raise ValueError('invalid observation interface')
        if type(self.maximum_duration) not in (float, int) or not math.isfinite(self.maximum_duration) or not 0 < self.maximum_duration <= 86400:
            raise ValueError('observation maximum duration must be in (0,86400] seconds')
        if type(self.heartbeat) is not bool or type(self.record_video) is not bool or type(self.record_hls) is not bool:
            raise ValueError('observation options must be booleans')
        if (type(self.video_nominal_fps) not in (float, int) or not math.isfinite(self.video_nominal_fps)
            or not 1 <= self.video_nominal_fps <= 60 or self.video_codec not in ('h264', 'hevc')):
            raise ValueError('invalid locally selected video codec or nominal FPS')
        if not self.record_hls and (self.video_nominal_fps != 30 or self.video_codec != 'h264'):
            raise ValueError('video decoding options require local HLS activation')
        if self.publish_config_file is not None and (not self.record_hls
            or not isinstance(self.publish_config_file, (str, Path)) or not str(self.publish_config_file)):
            raise ValueError('video publication requires local HLS activation and a private config path')
        if self.credentials_file is not None and (not isinstance(self.credentials_file, (str, Path)) or not str(self.credentials_file)):
            raise ValueError('observation credentials must be a local file path')
        if self.wlm_identity_file is not None and (not isinstance(self.wlm_identity_file, (str, Path)) or not str(self.wlm_identity_file)):
            raise ValueError('observation WLM identity must be a local file path')
        if self.ble_address is not None:
            if (not isinstance(self.ble_address, str)
                or not re.fullmatch(r'(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}', self.ble_address)
                or not isinstance(self.ble_identity_file, (str, Path)) or not str(self.ble_identity_file)
                or self.wlm_identity_file is None or self.credentials_file is not None):
                raise ValueError('BLE observation requires separate local BLE/WLM identities and no credentials input')
        elif self.ble_identity_file is not None or self.ble_setup_timeout != 30:
            raise ValueError('BLE observation options require a local BLE address')
        if type(self.ble_setup_timeout) not in (float, int) or not math.isfinite(self.ble_setup_timeout) or not 0 < self.ble_setup_timeout <= 120:
            raise ValueError('BLE setup timeout must be in (0,120] seconds')

    @property
    def managed_wifi(self):
        return self.credentials_file is not None or self.ble_address is not None


class ManagedObservation:
    def __init__(self, config: ObservationConfig, capture_dir: str | Path, *,
                 event_sink=None, session_runner=None, route_guard=None, clock=time.monotonic):
        self.config, self.capture_dir = config, Path(capture_dir)
        self.event_sink, self.clock = event_sink, clock
        self.runner = run_session if session_runner is None else session_runner
        self.guard = (lambda interface: require_dedicated_drone_route(interface, cancel_event=self.stop_event)) if route_guard is None else route_guard
        self.lock = threading.RLock()
        self.thread = None
        self.stop_event = threading.Event()
        self.worker_state = 'idle'
        self.identifier = self.latest = self.result = self.error = None
        self.paths = {}
        self.duration = None
        self.shutting_down = False
        self.wifi = {'managed': config.managed_wifi}
        self.ble = {'enabled': config.ble_address is not None}
        self.wifi_cleanup_pending = False
        self.ble_cleanup_pending = False
        self.video_cleanup_pending = False
        self.video_final = None
        self.publish_cleanup_pending = False
        self.publication_final = None

    @property
    def busy(self):
        with self.lock:
            return self.worker_state in ('starting', 'running', 'stopping', 'cleaning_up') or bool(self.thread and self.thread.is_alive())

    @property
    def owns_link(self):
        return self.busy or self.wifi_cleanup_pending or self.ble_cleanup_pending or self.video_cleanup_pending or self.publish_cleanup_pending

    def _event(self, name, **detail):
        if self.event_sink:
            self.event_sink(name, **detail)

    def open(self, parameters=None):
        parameters = {} if parameters is None else parameters
        if not isinstance(parameters, dict) or set(parameters) - {'duration'}:
            raise ValueError('only observation duration may be requested remotely')
        duration = parameters.get('duration', min(60, self.config.maximum_duration))
        if type(duration) not in (float, int) or not math.isfinite(duration) or not 0 < duration <= self.config.maximum_duration:
            raise ValueError('observation duration exceeds the locally configured bound')
        with self.lock:
            if self.shutting_down:
                raise ObservationError('service_stopping')
            if self.wifi_cleanup_pending:
                raise ObservationError('wifi_cleanup_unconfirmed')
            if self.ble_cleanup_pending:
                raise ObservationError('ble_cleanup_unconfirmed')
            if self.video_cleanup_pending:
                raise ObservationError('video_cleanup_unconfirmed')
            if self.publish_cleanup_pending:
                raise ObservationError('publish_cleanup_unconfirmed')
            if self.busy:
                raise ObservationError('observation_already_running')
            identifier, stop = secrets.token_hex(12), threading.Event()
            capture = self.capture_dir / f'observe-{identifier}.jsonl'
            video = self.capture_dir / f'observe-{identifier}.video.bin' if self.config.record_video else None
            self.identifier, self.stop_event = identifier, stop
            self.paths = {'capture_path': str(capture), 'journal_path': str(capture) + '.session.jsonl',
                          'video_path': str(video) if video else None,
                          'wifi_journal_path': str(capture) + '.wifi.jsonl' if self.config.managed_wifi else None}
            self.paths['hls_directory'] = str(capture) + '.hls' if self.config.record_hls else None
            self._credentials_path = Path(str(capture) + '.wifi-credentials.json')
            if self.config.ble_address is not None:
                self.paths.update(ble_capture_path=str(capture) + '.ble.jsonl',
                                  ble_journal_path=str(capture) + '.ble.jsonl.setup.jsonl')
            self.ble = {'enabled': self.config.ble_address is not None, 'connected': False,
                        'retained': False, 'disconnected_during_hold': False,
                        'stage': 'preparing' if self.config.ble_address is not None else 'disabled'}
            self.wifi = {'managed': self.config.managed_wifi,
                         'stage': 'preparing' if self.config.managed_wifi else 'not_managed',
                         'profile_created': False, 'profile_removed': False,
                         'deactivated': False, 'cleanup_verified': True}
            self.latest = self.result = self.error = None
            self.video_final = None
            self.publication_final = None
            self.duration, self.worker_state = duration, 'starting'
            self.thread = threading.Thread(target=self._run,
                args=(identifier, stop, duration, capture, video), name='neo2-observation', daemon=True)
        try:
            self._event('observation_requested', identifier=identifier, interface=self.config.interface,
                        duration=duration, heartbeat=self.config.heartbeat, record_video=self.config.record_video)
        except Exception:
            with self.lock:
                self.worker_state, self.error = 'failed', 'observation_audit_unavailable'
            raise ObservationError('observation_audit_unavailable') from None
        with self.lock:
            if stop.is_set() or self.shutting_down:
                self.worker_state = 'finished'
            else:
                try:
                    self.thread.start()
                except RuntimeError:
                    self.worker_state, self.error = 'failed', 'observation_thread_start_failed'
                    raise ObservationError('observation_thread_start_failed') from None
        return self.snapshot()

    def _publish(self, identifier, snapshot):
        with self.lock:
            if (identifier == self.identifier and self.thread and self.thread.is_alive()
                and self.worker_state in ('starting', 'running', 'stopping', 'cleaning_up')):
                self.latest = deepcopy(snapshot)
                if self.config.managed_wifi and snapshot.get('state') in ('closed', 'failed'):
                    self.worker_state = 'cleaning_up'

    def _wifi_event(self, identifier, journal, name, **detail):
        with self.lock:
            if identifier == self.identifier:
                stages = {'runtime_profile_created': 'connecting', 'wifi_activated': 'verifying',
                          'wifi_route_verified': 'connected', 'wifi_cleanup_started': 'cleaning_up',
                          'wifi_profile_removed': 'removed', 'wifi_cleanup_failed': 'failed'}
                if name in stages:
                    self.wifi['stage'] = stages[name]
                if name == 'runtime_profile_created':
                    self.wifi.update(profile_created=True, cleanup_verified=False)
                if name == 'wifi_cleanup_started':
                    self.worker_state = 'cleaning_up'
                for key in ('profile_removed', 'deactivated', 'cleanup_verified'):
                    if key in detail:
                        self.wifi[key] = detail[key]
        journal.record({'event': name, 'time': self.clock(), 'identifier': identifier, **detail})
        self._event(name, identifier=identifier, **detail)

    def _ble_event(self, identifier, name, **detail):
        public = {k: v for k, v in detail.items()
                  if k not in ('credentials_path', 'capture_path', 'journal_path')}
        with self.lock:
            if identifier == self.identifier:
                if name == 'notifications_enabled':
                    self.ble.update(connected=True, stage='credentials')
                elif name == 'stage_started':
                    self.ble['stage'] = public['stage']
                elif name == 'ble_retained':
                    self.ble.update(retained=True, stage='wifi_session')
                elif name == 'ble_disconnected_during_hold':
                    self.ble.update(connected=False, disconnected_during_hold=True)
                elif name == 'ble_retention_finished':
                    self.ble.update(retained=False, stage='closing')
                elif name == 'ble_cleanup_result':
                    self.ble_cleanup_pending = not public['cleanup_verified']
                    self.ble['cleanup_verified'] = public['cleanup_verified']
                    if public['cleanup_verified']:
                        self.ble['connected'] = False
                elif name == 'setup_result':
                    self.ble['result'] = deepcopy(public)
        self._event('ble_handoff_event', identifier=identifier, phase_event=name, detail=public)

    def _video_cleanup(self, identifier, snapshot):
        with self.lock:
            if identifier == self.identifier:
                self.video_final = deepcopy(snapshot)
                self.video_cleanup_pending = not snapshot['cleanup_verified']
        self._event('video_cleanup_result', identifier=identifier,
                    cleanup_verified=snapshot['cleanup_verified'])

    def _publication_cleanup(self, identifier, snapshot):
        with self.lock:
            if identifier == self.identifier:
                self.publication_final = deepcopy(snapshot)
                self.publish_cleanup_pending = not snapshot['cleanup_verified']
        self._event('publish_cleanup_result', identifier=identifier,
                    cleanup_verified=snapshot['cleanup_verified'],
                    remote_stop_acknowledged=snapshot['remote_stop_acknowledged'])

    def _wifi_session(self, identifier, stop, duration, capture, video, state):
        try:
            with ExitStack() as resources:
                if self.config.managed_wifi:
                    source = Path(self.config.credentials_file) if self.config.credentials_file is not None else self._credentials_path
                    if source.resolve() in {Path(path).resolve() for path in self.paths.values() if path}:
                        raise WifiLinkError('wifi_credentials_output_collision')
                    journal = DecisionJournal(str(capture) + '.wifi.jsonl')
                    resources.callback(journal.close)
                    def event(name, **details):
                        self._wifi_event(identifier, journal, name, **details)
                    event('wifi_prepare_started', interface=self.config.interface)
                    try:
                        credentials = load_wifi_credentials(source)
                    except (OSError, ValueError):
                        raise WifiLinkError('wifi_credentials_invalid_or_unavailable') from None
                    plan = prepare_wifi_plan(credentials, self.config.interface, cancel_event=stop)
                    with self.lock:
                        self.wifi.update(connection_uuid=plan.connection_uuid,
                                         network_sha256=plan.network['network_sha256'])
                    event('wifi_plan_prepared', interface=self.config.interface,
                          connection_uuid=plan.connection_uuid, network_sha256=plan.network['network_sha256'])
                    state['link'] = link = DedicatedWifiLink(plan, event, cancel_event=stop)
                    resources.enter_context(link)
                if stop.is_set():
                    return
                self.guard(self.config.interface)
                if stop.is_set():
                    return
                self._event('observation_route_verified', identifier=identifier, interface=self.config.interface)
                if stop.is_set():
                    return
                with self.lock:
                    if identifier == self.identifier and not stop.is_set():
                        self.worker_state = 'running'
                state['result'] = self.runner(capture_path=capture, host=DRONE_HOST, port=9003,
                    interface=self.config.interface, duration=duration, heartbeat=self.config.heartbeat,
                    video_path=video, stop_event=stop,
                    snapshot_sink=lambda snapshot: self._publish(identifier, snapshot),
                    **({'wlm_identity_file': self.config.wlm_identity_file}
                       if self.config.wlm_identity_file is not None else {}),
                    **({'hls_directory': self.paths['hls_directory'],
                        'video_nominal_fps': self.config.video_nominal_fps, 'video_codec': self.config.video_codec,
                        'video_cleanup_sink': lambda snapshot: self._video_cleanup(identifier, snapshot)}
                       if self.config.record_hls else {}),
                    **({'publish_config_file': self.config.publish_config_file,
                        'publish_cleanup_sink': lambda snapshot: self._publication_cleanup(identifier, snapshot)}
                       if self.config.publish_config_file is not None else {}))
        except BaseException as failure:
            state['error'] = failure
            raise

    def _run(self, identifier, stop, duration, capture, video):
        error = None
        state = {'link': None, 'result': None, 'error': None}
        try:
            if stop.is_set():
                return
            protected = [path for path in self.paths.values() if path]
            if self.config.credentials_file is not None:
                protected.append(self.config.credentials_file)
            if self.config.ble_address is not None:
                protected.extend((self._credentials_path, self.config.ble_identity_file))
            if self.config.publish_config_file is not None:
                publication = PublishConfig.load(self.config.publish_config_file, protected_paths=protected
                    + ([self.config.wlm_identity_file] if self.config.wlm_identity_file is not None else []))
                protected.append(publication.path)
                if publication.ca_path is not None:
                    protected.append(publication.ca_path)
            if self.config.record_hls:
                HlsVideo(self.paths['hls_directory'], nominal_fps=self.config.video_nominal_fps,
                    input_codec=self.config.video_codec,
                    protected_paths=[p for p in protected if p != self.paths['hls_directory']]
                        + ([self.config.wlm_identity_file] if self.config.wlm_identity_file is not None else []))
            if self.config.wlm_identity_file is not None:
                try:
                    load_or_create_identity(self.config.wlm_identity_file, protected_paths=protected)
                except (OSError, ValueError):
                    raise ObservationError('wlm_identity_invalid_or_unavailable') from None
            if self.config.ble_address is not None:
                try:
                    validate_identity_path(self.config.ble_identity_file,
                        [p for p in protected if p != self.config.ble_identity_file]
                        + [self.config.wlm_identity_file])
                    load_or_create_ble_identity(self.config.ble_identity_file)
                except (OSError, ValueError):
                    raise ObservationError('ble_identity_invalid_or_unavailable') from None
                # Rechecked by DedicatedWifiLink immediately before activation.
                require_available_wifi_interface(self.config.interface, cancel_event=stop)
                async def connected():
                    await asyncio.to_thread(self._wifi_session, identifier, stop, duration, capture, video, state)
                ble_result = asyncio.run(run_ble_wifi_setup(
                    address=self.config.ble_address, identity_file=self.config.ble_identity_file,
                    wlm_identity_file=self.config.wlm_identity_file,
                    credentials_path=self._credentials_path, capture_path=self.paths['ble_capture_path'],
                    timeout=self.config.ble_setup_timeout,
                    request_timeout=min(3, self.config.ble_setup_timeout),
                    while_connected=connected, hold_timeout=duration + 120,
                    stop_event=stop, event_sink=lambda name, **detail: self._ble_event(identifier, name, **detail)))
                if state['error'] is not None:
                    raise state['error']
                if ble_result.state != 'complete':
                    raise ObservationError('ble_' + ble_result.reason)
            else:
                self._wifi_session(identifier, stop, duration, capture, video, state)
        except asyncio.CancelledError:
            failure = state['error']
            if isinstance(failure, (WifiLinkError, ObservationError)) and not isinstance(failure, WifiLinkCancelled):
                error = str(failure)
            elif failure is not None and not isinstance(failure, WifiLinkCancelled):
                error = 'observation_worker_failed'
            elif not stop.is_set():
                error = 'observation_cancelled'
        except WifiLinkCancelled:
            error = None if stop.is_set() else 'wifi_connection_cancelled'
        except (WifiLinkError, ObservationError) as failure:
            error = str(failure)
        except (VideoError, PublishError) as failure:
            error = str(failure)
        except OSError:
            error = 'observation_io_error'
        except BaseException:
            error = 'observation_worker_failed'
        finally:
            link, result = state['link'], state['result']
            with self.lock:
                if identifier == self.identifier:
                    if (self.video_final is None and isinstance(state['error'], VideoError)
                        and not state['error'].cleanup_verified):
                        self.video_cleanup_pending = True
                    if self.video_cleanup_pending:
                        error = 'video_cleanup_unconfirmed'
                    if (self.publication_final is None and isinstance(state['error'], PublishError)
                        and not state['error'].cleanup_verified):
                        self.publish_cleanup_pending = True
                    if self.publish_cleanup_pending:
                        error = 'publish_cleanup_unconfirmed'
                    if self.config.managed_wifi:
                        created = bool(link and link.ever_created)
                        verified = not created or link.cleanup_verified
                        self.wifi.update(profile_created=created, profile_removed=bool(link and link.removed),
                                         deactivated=bool(link and link.deactivated), cleanup_verified=verified)
                        self.wifi_cleanup_pending = not verified
                        if not created:
                            self.wifi['stage'] = 'cancelled' if stop.is_set() and error is None else 'failed'
                    if not self.ble_cleanup_pending:
                        self.ble['connected'] = False
                    else:
                        error = 'ble_cleanup_unconfirmed'
                    self.ble.update(retained=False,
                                    stage='failed' if error else 'closed')
                    self.result = result.as_dict() if result is not None else None
                    self.error = error or (result.reason if result is not None and result.state == 'failed' else None)
                    self.worker_state = 'failed' if self.error else 'finished'
            try:
                self._event('observation_finished', identifier=identifier,
                            state='failed' if self.error else 'closed', reason=self.error or (result.reason if result else 'requested'))
            except Exception:
                with self.lock:
                    self.worker_state, self.error = 'failed', 'observation_audit_unavailable'

    def close(self):
        with self.lock:
            identifier = self.identifier
            if self.worker_state in ('starting', 'running', 'stopping', 'cleaning_up'):
                self.stop_event.set()
                if self.worker_state != 'cleaning_up':
                    self.worker_state = 'stopping'
        # Cancellation takes effect even if the lifecycle journal is broken.
        try:
            self._event('observation_stop_requested', identifier=identifier)
        except Exception:
            pass
        return self.snapshot()

    def shutdown(self, timeout=None):
        if timeout is None:
            if self.config.publish_config_file is not None:
                timeout = 85.0 if self.config.managed_wifi else 25.0
            else:
                timeout = (65.0 if self.config.record_hls else 50.0) if self.config.managed_wifi else (10.0 if self.config.record_hls else 2.0)
        with self.lock:
            self.shutting_down = True
        self.close()
        with self.lock:
            thread = self.thread
        if thread and thread.is_alive():
            thread.join(timeout=timeout)
            if thread.is_alive():
                raise ObservationError('observation_worker_did_not_stop')
        if self.wifi_cleanup_pending:
            raise ObservationError('wifi_cleanup_unconfirmed')
        if self.ble_cleanup_pending:
            raise ObservationError('ble_cleanup_unconfirmed')
        if self.video_cleanup_pending:
            raise ObservationError('video_cleanup_unconfirmed')
        if self.publish_cleanup_pending:
            raise ObservationError('publish_cleanup_unconfirmed')

    def snapshot(self, now=None):
        now = self.clock() if now is None else now
        if not math.isfinite(now):
            raise ValueError('observation time must be finite')
        with self.lock:
            state, latest = self.worker_state, deepcopy(self.latest)
            alive = bool(self.thread and self.thread.is_alive())
            stopped = self.stop_event.is_set()
            result = deepcopy(self.result)
            video_final = deepcopy(self.video_final)
            publication_final = deepcopy(self.publication_final)
            status = {'enabled': True, 'identifier': self.identifier, 'worker_state': state,
                      'worker_alive': alive, 'stop_requested': stopped,
                      'interface': self.config.interface, 'host': DRONE_HOST, 'port': 9003,
                      'heartbeat': self.config.heartbeat, 'duration_seconds': self.duration,
                      'wlm_bootstrap_enabled': self.config.wlm_identity_file is not None,
                      'maximum_duration_seconds': self.config.maximum_duration,
                      'wifi': deepcopy(self.wifi), 'wifi_cleanup_pending': self.wifi_cleanup_pending,
                      'video_cleanup_pending': self.video_cleanup_pending,
                      'publish_cleanup_pending': self.publish_cleanup_pending,
                      'ble': deepcopy(self.ble),
                      'error': self.error, **self.paths}
        session = latest or {}
        # A concurrent publication may be a few microseconds newer than the
        # caller's captured time. Never create negative ages in that case.
        elapsed = max(0.0, now - session.get('observed_at', now))
        receive_age = session.get('last_received_age_seconds')
        if receive_age is not None:
            session['last_received_age_seconds'] = receive_age + elapsed
        connected = (alive and state == 'running' and not stopped and session.get('state') == 'open'
                     and receive_age is not None and receive_age + elapsed < session.get('remote_timeout_seconds', 0))
        if result:
            session.update({key: result[key] for key in ('state', 'reason', 'transmitted', 'received',
                'heartbeats_sent', 'heartbeats_acknowledged', 'video_bytes_delivered', 'protocol_bytes_delivered')})
            if 'wlm_bootstrap' in result:
                session['wlm_bootstrap'] = deepcopy(result['wlm_bootstrap'])
            if 'wlm_discovery' in result:
                session['wlm_discovery'] = deepcopy(result['wlm_discovery'])
            if 'video' in result:
                session['video'] = deepcopy(result['video'])
            if 'video_publish' in result:
                session['video_publish'] = deepcopy(result['video_publish'])
        if publication_final is not None:
            session['video_publish'] = publication_final
        if session.get('wlm_discovery'):
            session['wlm_discovery'] = age_discovery(session['wlm_discovery'], now,
                                                    session_active=connected)
        delivery = session.get('video_publish', {})
        if delivery.get('enabled'):
            age = delivery.get('last_uploaded_age_seconds')
            if age is not None:
                age += max(0, now - delivery.get('observed_at', now))
                delivery['last_uploaded_age_seconds'] = age
                if delivery.get('state') == 'live' and age > 3:
                    delivery['state'] = 'stale'
            if delivery.get('state') != 'failed':
                if not alive:
                    delivery['state'] = 'offline'
                elif stopped:
                    delivery['state'] = 'stopping'
        video_status = session.get('video', {})
        if video_final is not None:
            video_status = session['video'] = video_final
        if video_status.get('enabled'):
            age = video_status.get('last_encoded_age_seconds')
            if age is not None:
                video_age = age + max(0, now - video_status.get('observed_at', now))
                video_status['last_encoded_age_seconds'] = video_age
                if video_status.get('state') == 'live' and video_age > 2:
                    video_status['state'] = 'stale'
            if (not alive or stopped or state in ('stopping', 'cleaning_up', 'finished')) and video_status.get('state') != 'failed':
                video_status['state'] = 'offline'
        wifi_control = session.pop('wifi_control', None)
        if wifi_control is not None:
            age_wifi_control(wifi_control, elapsed, bool(connected))
        battery = session.pop('battery', None)
        if battery is not None:
            age_battery(battery, elapsed, bool(connected))
        telemetry = session.pop('telemetry', None)
        if telemetry is not None:
            telemetry['synthetic'] = False
            telemetry['flight_ready'] = False
            if telemetry.get('available'):
                age = telemetry['age_seconds'] + elapsed
                fresh = age <= telemetry['maximum_age_seconds']
                telemetry.update(age_seconds=age, fresh=fresh, session_active=bool(connected))
                for value in telemetry['fields'].values():
                    value['age_seconds'] += elapsed
                    value['valid'] = value['valid'] and fresh and bool(connected)
                    if not connected:
                        value['reason'] = 'session_inactive'
                    elif not fresh:
                        value['reason'] = 'stale'
            for sample in telemetry.get('observations', []):
                age = sample['age_seconds'] + elapsed
                fresh = age <= telemetry['maximum_age_seconds']
                sample.update(age_seconds=age, fresh=fresh, session_active=bool(connected), flight_ready=False)
                for value in sample['fields'].values():
                    value['age_seconds'] += elapsed
                    value['valid'] = value['valid'] and fresh and bool(connected)
                    if not connected:
                        value['reason'] = 'session_inactive'
                    elif not fresh:
                        value['reason'] = 'stale'
        public_state = {'idle': 'disconnected', 'starting': 'connecting', 'stopping': 'stopping',
                        'cleaning_up': 'stopping', 'finished': 'stopping' if alive else 'closed',
                        'failed': 'failed'}.get(state,
                        'observing' if connected else ('stopping' if session.get('state') in ('closed', 'failed') else 'connecting'))
        return {'state': public_state, 'connected': bool(connected), 'synthetic': False,
                'flight_ready': False, 'application_ready': False, 'session': session,
                'telemetry': telemetry, 'wifi_control': wifi_control,
                'battery': battery, 'observation': status}
