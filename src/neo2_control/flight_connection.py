"""Owned BLE retention, temporary Wi-Fi and typed flight UDP lifetime.

Construction is inert. The supervisor opens the connection explicitly. Stop
requests are nonblocking; cleanup is awaited separately by the process owner.
"""
from copy import deepcopy
from dataclasses import asdict, dataclass
import asyncio
import json
import math
import os
from pathlib import Path
import re
import secrets
import threading
import time

from .ble_bootstrap import load_wifi_credentials,run_ble_wifi_setup
from .camera import CameraChannel
from .control_transport import FlightChannelBackend
from .controller import IntentRejected
from .device_lease import DeviceLease
from .flight_channel import FlightChannel
from .flight import DEFAULT_YAW_CHANNEL_LIMIT, validate_yaw_channel_limit
from .session import run_session
from .wifi_link import _require_idle_wifi_interface,WifiLinkCancelled


@dataclass(frozen=True)
class FlightConfig:
    interface: str
    wlm_identity_file: str | Path
    ble_identity_file: str | Path
    ble_address: str = '02:00:00:00:00:01'
    duration: float = 300
    capture_directory: str | Path = 'private/flight-sessions'
    camera_preview: bool = False
    vertical_trial: bool = False
    incident_directory: str | Path = Path.home()/'.local/state/neo2-control/flight-incidents'
    yaw_trial: bool = False
    yaw_channel_limit: int = DEFAULT_YAW_CHANNEL_LIMIT
    acquisition_trial: bool = False

    def __post_init__(self):
        if not isinstance(self.interface,str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}',self.interface):
            raise ValueError('invalid_flight_interface')
        if not isinstance(self.ble_address,str) or not re.fullmatch(r'(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}',self.ble_address):
            raise ValueError('invalid_flight_ble_address')
        if type(self.duration) not in (int,float) or not math.isfinite(self.duration) or not 0<self.duration<=600:
            raise ValueError('flight_duration_must_be_in_0_600')
        if not all(isinstance(p,(str,Path)) and str(p) for p in
                   (self.wlm_identity_file,self.ble_identity_file,self.capture_directory,self.incident_directory)):
            raise ValueError('flight_local_paths_required')
        if Path(self.wlm_identity_file).absolute()==Path(self.ble_identity_file).absolute():
            raise ValueError('flight_identities_must_be_separate')
        if type(self.camera_preview) is not bool:raise ValueError('flight_camera_preview_must_be_boolean')
        if type(self.vertical_trial) is not bool:raise ValueError('vertical_trial_must_be_boolean')
        if type(self.yaw_trial) is not bool:raise ValueError('yaw_trial_must_be_boolean')
        if self.vertical_trial and self.yaw_trial:raise ValueError('flight_trial_profiles_are_exclusive')
        if type(self.acquisition_trial) is not bool or self.acquisition_trial and not self.yaw_trial:
            raise ValueError('acquisition_trial_requires_yaw_profile')
        validate_yaw_channel_limit(self.yaw_channel_limit,yaw_trial=self.yaw_trial)

    def as_dict(self):
        paths={'wlm_identity_file','ble_identity_file','capture_directory','incident_directory'}
        return {key:str(Path(value).absolute()) if key in paths else value
                for key,value in asdict(self).items()}


class FlightConnection:
    def __init__(self,config,channel,*,session_runner=None,ble_runner=None,wifi_factory=None,
                 route_guard=None,lease_factory=DeviceLease,clock=time.monotonic):
        if not isinstance(config,FlightConfig) or not isinstance(channel,FlightChannel) or channel.observer is None:
            raise ValueError('flight_connection_requires_config_and_owned_observations')
        self.config,self.channel,self.clock=config,channel,clock
        self.session_runner=session_runner or run_session
        self.ble_runner=ble_runner or run_ble_wifi_setup
        self.wifi_factory=wifi_factory
        self.route_guard=route_guard or _require_idle_wifi_interface
        self.lease_factory=lease_factory
        self.lock=threading.RLock()
        self.stop_event=threading.Event()
        self.thread=None
        self.phase='idle'
        self.error=self.result=self.latest=None
        self.cleanup={'ble':True,'wifi':True,'socket':True}
        self.generation=None
        self.paths={}
        self.camera=CameraChannel(clock=clock) if config.camera_preview else None
        if config.camera_preview:self.cleanup['video']=True
        self.video={}
        self.control_setup={'state':'idle'}

    def start(self):
        with self.lock:
            if self.thread is not None:raise RuntimeError('flight_connection_generation_already_used')
            self.generation=secrets.token_hex(12)
            root=Path(self.config.capture_directory)/self.generation
            self.paths={'directory':root,'capture':root/'swudp.jsonl',
                        'credentials':root/'wifi.json','ble':root/'ble.jsonl',
                        'video':root/'camera.hevc','hls':root/'hls'}
            self.phase='connecting'
            self.thread=threading.Thread(target=self._run,name='neo2-flight-session',daemon=False)
            try:self.thread.start()
            except BaseException:
                self.phase,self.error='failed','flight_worker_start_failed'
                raise

    def request_stop(self):
        self.stop_event.set()
        self.channel.revoke()
        if self.camera is not None:self.camera.revoke()
        with self.lock:
            if self.thread is not None and self.thread.is_alive():self.phase='stopping'

    def join(self,timeout=60):
        thread=self.thread
        if thread is not None and thread.ident is not None:
            thread.join(timeout)
            if thread.is_alive():raise RuntimeError('flight_connection_worker_did_not_stop')
        if not self.status()['cleanup_verified']:raise RuntimeError('flight_connection_cleanup_unconfirmed')

    def _save(self,name,value):
        path=self.paths['directory']/name
        fd=os.open(path,os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
        with os.fdopen(fd,'w') as stream:json.dump(value,stream,indent=2);stream.write('\n')

    def _publish(self,snapshot):
        with self.lock:
            self.latest={'state':snapshot.get('state'),'observed_at':self.clock(),
                'last_received_age_seconds':snapshot.get('last_received_age_seconds')}
            if self.phase!='stopping' and snapshot.get('state')=='open':self.phase='session_open'
            video=snapshot.get('video',{})
            self.video={key:video.get(key) for key in ('state','frames_encoded','last_encoded_age_seconds',
                'preview_age_seconds','preview_available','error')}
            link=snapshot.get('wlm_mode_follow',{})
            switch=link.get('service_switch',{})
            failed=(link.get('state')=='failed' or switch.get('state')=='failed' or (
                link.get('capability_read',{}).get('state') in ('timed_out','response_error')
                and link.get('capability_query_retry_in_seconds') is None))
            self.control_setup={'state':('failed' if failed else 'preparing' if link.get('peer_beacon_stale') else 'ready'
                if link.get('wifi_service_configured') is True else 'preparing')}

    def _video_cleanup(self,snapshot):
        with self.lock:self.cleanup['video']=snapshot.get('cleanup_verified') is True

    def _socket_cleanup(self,snapshot):
        with self.lock:self.cleanup['socket']=snapshot.get('cleanup_verified') is True

    def _ble_event(self,name,**detail):
        if name=='ble_cleanup_result':
            with self.lock:self.cleanup['ble']=detail.get('cleanup_verified') is True

    async def _connect_and_run(self):
        if self.wifi_factory is None:
            from .wifi_memory import MemoryWifiLink
            factory=MemoryWifiLink
        else:factory=self.wifi_factory
        async def connected():
            credentials=load_wifi_credentials(self.paths['credentials'])
            ownership=({} if self.wifi_factory is not None else {
                'ownership_directory':Path(self.config.incident_directory)/'wifi'})
            link=factory(credentials,self.config.interface,stop_event=self.stop_event,**ownership)
            try:
                with self.lock:self.cleanup['wifi']=None;self.phase='wifi_connecting'
                async with link:
                    with self.lock:self.cleanup['socket']=None
                    extra={}
                    if self.camera is not None:
                        with self.lock:self.cleanup['video']=None
                        extra={'camera_channel':self.camera,'preview_jpeg':True,
                            'video_path':self.paths['video'],'hls_directory':self.paths['hls'],
                            'video_codec':'hevc','video_cleanup_sink':self._video_cleanup}
                    try:
                        result=await asyncio.to_thread(self.session_runner,
                            capture_path=self.paths['capture'],interface=self.config.interface,
                            duration=self.config.duration,heartbeat=True,
                            wlm_identity_file=self.config.wlm_identity_file,wlm_wifi_liveview=True,
                            flight_channel=self.channel,snapshot_sink=self._publish,
                            socket_cleanup_sink=self._socket_cleanup,stop_event=self.stop_event,**extra)
                    except Exception as error:
                        # The BLE retention wrapper deliberately suppresses
                        # arbitrary exception strings. Keep a bounded private
                        # diagnostic so a Wi-Fi/decoder failure is identifiable.
                        reason=str(error)
                        self._save('session-error.json',{'type':type(error).__name__,
                            'code':reason if re.fullmatch(r'[a-z][a-z0-9_]{0,95}',reason)
                            else 'detail_suppressed'})
                        raise
                    with self.lock:
                        self.result=result.as_dict()
                        # The callback is authoritative on exceptions; the
                        # successful result independently records its closed FD.
                        if result.socket_cleanup_verified is not True:self.cleanup['socket']=False
                        if result.state=='failed':self.error=result.reason
            finally:
                metadata=link.snapshot()
                with self.lock:self.cleanup['wifi']=metadata.get('cleanup_verified') is True
                self._save('wifi-result.json',metadata)
        with self.lock:self.cleanup['ble']=None;self.phase='ble_connecting'
        result=await self.ble_runner(address=self.config.ble_address,
            identity_file=self.config.ble_identity_file,wlm_identity_file=self.config.wlm_identity_file,
            credentials_path=self.paths['credentials'],capture_path=self.paths['ble'],timeout=45,
            stop_event=self.stop_event,while_connected=connected,hold_timeout=self.config.duration+60,
            event_sink=self._ble_event)
        self._save('ble-result.json',result.as_dict())
        with self.lock:
            self.cleanup['ble']=result.connection_cleanup_verified is True
            if result.state=='failed' and self.error is None:self.error=result.reason

    def _run(self):
        try:
            if self.wifi_factory is None:
                with self.lock:self.cleanup['wifi']=None
            self.paths['directory'].mkdir(mode=0o700,parents=True,exist_ok=False)
            with self.lease_factory(self.config.interface,self.config.ble_address):
                if self.wifi_factory is None:
                    from .wifi_memory import recover_memory_wifi
                    with self.lock:self.cleanup['wifi']=None;self.phase='wifi_recovering'
                    recovery=asyncio.run(recover_memory_wifi(self.config.interface,
                        Path(self.config.incident_directory)/'wifi',stop_event=self.stop_event))
                    with self.lock:self.cleanup['wifi']=recovery.get('cleanup_verified') is True
                    self._save('wifi-recovery.json',recovery)
                self.route_guard(self.config.interface,cancel_event=self.stop_event)
                asyncio.run(self._connect_and_run())
        except asyncio.CancelledError:
            if not self.stop_event.is_set():self.error='flight_connection_cancelled'
        except WifiLinkCancelled:
            if not self.stop_event.is_set():self.error='flight_connection_cancelled'
        except Exception as error:
            reason=str(error)
            with self.lock:
                self.error=(reason if re.fullmatch(r'[a-z][a-z0-9_]{0,95}',reason)
                            else 'flight_connection_failed')
        finally:
            self.channel.cancel(self.clock(),'session_closed')
            if self.camera is not None:self.camera.cancel(self.clock(),'session_closed')
            with self.lock:
                clean=all(value is True for value in self.cleanup.values())
                if not clean and self.error is None:self.error='flight_cleanup_unconfirmed'
                self.phase='failed' if self.error else 'closed'
            try:self._save('connection-result.json',{'phase':self.phase,'error':self.error,
                'cleanup':self.cleanup,'session':self.result})
            except OSError:
                with self.lock:self.phase,self.error='failed','flight_audit_unavailable'

    def status(self):
        with self.lock:
            alive=self.thread is not None and self.thread.is_alive()
            latest=deepcopy(self.latest)
            camera=self.camera.snapshot() if self.camera is not None else {}
            camera={key:camera.get(key) for key in ('ready','capabilities','pitch_limits',
                'commanded_pitch','recording','last_command','pending','fov_status','fov_observations')}
            video=deepcopy(self.video)
            elapsed=max(0,self.clock()-(latest or {}).get('observed_at',self.clock()))
            for key in ('last_encoded_age_seconds','preview_age_seconds'):
                if video.get(key) is not None:video[key]+=elapsed
            video['live']=bool(alive and self.phase=='session_open' and video.get('preview_available')
                and video.get('last_encoded_age_seconds') is not None and video['last_encoded_age_seconds']<=2
                and video.get('preview_age_seconds') is not None and video['preview_age_seconds']<=3)
            return {'phase':self.phase,'error':self.error,'generation':self.generation,
                'worker_alive':alive,'cleanup':dict(self.cleanup),
                'cleanup_verified':not alive and all(value is True for value in self.cleanup.values()),
                'transport_snapshot':latest,'camera':camera,'video':video,
                'control_setup':dict(self.control_setup)}

    def camera_command(self,action,parameters, *, ingress=None):
        if self.camera is None:raise ValueError('flight_camera_not_configured')
        with self.lock:
            if ingress is not None:
                ingress.validate(self.clock(), connection_generation=self.generation)
            if self.phase!='session_open' or self.stop_event.is_set():raise RuntimeError('camera_not_ready')
            options = {} if ingress is None else {'ingress': ingress}
            return self.camera.submit(action,parameters, **options)


class OwnedFlightBackend(FlightChannelBackend):
    """Supervisor adapter whose open/close owns the physical connection too."""
    def __init__(self,config,*,clock=time.monotonic,channel_factory=FlightChannel,
                 connection_factory=FlightConnection,**connection_options):
        if not isinstance(config,FlightConfig):raise ValueError('flight_config_required')
        self.config,self.clock=config,clock
        self.channel_factory,self.connection_factory=channel_factory,connection_factory
        self.connection_options=connection_options
        self.connection=None
        self._new_generation()

    def _new_generation(self):
        options={'vertical_trial':True} if self.config.vertical_trial else {}
        if self.config.yaw_trial:
            options.update(yaw_trial=True,yaw_channel_limit=self.config.yaw_channel_limit)
        if self.config.acquisition_trial:options['acquisition_trial']=True
        super().__init__(self.channel_factory(clock=self.clock,**options),synthetic=False)
        self.connection=self.connection_factory(self.config,self.channel,clock=self.clock,**self.connection_options)

    def open(self,now):
        if self.opened:raise IntentRejected('flight_connection_already_open')
        if self.connection.thread is not None or self.channel.snapshot()['revoked']:
            if not self.connection.status()['cleanup_verified']:
                raise IntentRejected('previous_flight_cleanup_unconfirmed')
            self._new_generation()
        super().open(now)
        try:self.connection.start()
        except Exception:
            super().close(now)
            raise

    def close(self,now):
        super().close(now)
        self.connection.request_stop()

    def finish_cleanup(self,timeout=60):
        self.connection.request_stop()
        self.connection.join(timeout)

    def connection_status(self):
        return self.connection.status()

    def camera_command(self,action,parameters, *, ingress=None):
        options = {} if ingress is None else {'ingress': ingress}
        return self.connection.camera_command(action,parameters, **options)
