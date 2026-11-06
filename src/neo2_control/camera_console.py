"""Owned BLE→temporary Wi-Fi→camera session for the local HTML interface."""
from copy import deepcopy
from dataclasses import dataclass
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
from .session import run_session
from .wifi_link import _require_idle_wifi_interface
from .device_lease import DeviceLease
from .platform_transport import normalize_ble_identifier


@dataclass(frozen=True)
class CameraConfig:
    interface:str
    wlm_identity_file:str|Path
    ble_identity_file:str|Path
    ble_address:str='02:00:00:00:00:01'
    duration:float=300
    capture_directory:str|Path='private/camera-sessions'
    last_frame:str|Path|None=None
    wlm_forwarding_sequence:int=1
    preview_only:bool=False
    manual_macos_wifi:bool=False

    def __post_init__(self):
        if type(self.preview_only) is not bool or type(self.manual_macos_wifi) is not bool:
            raise ValueError('camera_platform_flags_must_be_boolean')
        if not isinstance(self.interface,str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}',self.interface):
            raise ValueError('invalid_camera_interface')
        try:normalize_ble_identifier(self.ble_address)
        except ValueError:raise ValueError('invalid_camera_ble_address') from None
        if type(self.duration) not in (int,float) or not math.isfinite(self.duration) or not 0<self.duration<=600:
            raise ValueError('camera_duration_must_be_in_0_600')
        if type(self.wlm_forwarding_sequence) is not int or not 0<=self.wlm_forwarding_sequence<=0xffffffff:
            raise ValueError('initial WLM forwarding sequence must fit uint32')
        if not all(isinstance(p,(str,Path)) and str(p) for p in (self.wlm_identity_file,self.ble_identity_file,self.capture_directory)):
            raise ValueError('camera_local_paths_required')
        if Path(self.wlm_identity_file).absolute()==Path(self.ble_identity_file).absolute():
            raise ValueError('camera_identities_must_be_separate')
        if self.last_frame is not None and (not isinstance(self.last_frame,(str,Path)) or Path(self.last_frame).suffix.lower() not in ('.png','.jpg','.jpeg')):
            raise ValueError('invalid_last_camera_frame')


class CameraConsole:
    def __init__(self,config,*,session_runner=None,ble_runner=None,wifi_factory=None,route_guard=None,clock=time.monotonic,lease_factory=DeviceLease):
        if not isinstance(config,CameraConfig):raise ValueError('camera_config_required')
        self.config,self.clock=config,clock
        self.session_runner=session_runner or run_session
        self.ble_runner=ble_runner or run_ble_wifi_setup
        self.wifi_factory=wifi_factory
        if config.manual_macos_wifi:
            from .macos_wifi import require_wifi_interface
            self.route_guard=route_guard or require_wifi_interface
        else:self.route_guard=route_guard or _require_idle_wifi_interface
        self.lease_factory=lease_factory
        self.lock=threading.RLock()
        self.channel=CameraChannel(clock=clock)
        self.stop_event=threading.Event()
        self.thread=None
        self.state='disconnected'
        self.error=self.latest=self.result=None
        self.wifi_cleanup_verified=self.ble_cleanup_verified=self.video_cleanup_verified=True
        self.paths={}
        self.closing=False
        self.generation=None
        self.verification_pending=False

    def connect(self):
        with self.lock:
            if self.closing:raise RuntimeError('camera_console_stopping')
            if self.thread is not None and self.thread.is_alive():raise RuntimeError('camera_session_already_running')
            if not all(x is True for x in (self.wifi_cleanup_verified,self.ble_cleanup_verified,self.video_cleanup_verified)):
                raise RuntimeError('camera_cleanup_unconfirmed')
            identifier=secrets.token_hex(12)
            self.generation=identifier
            root=Path(self.config.capture_directory)/identifier
            self.paths={'directory':root,'capture':root/'swudp.jsonl','video':root/'camera.hevc',
                        'hls':root/'hls','credentials':root/'wifi.json','ble':root/'ble.jsonl'}
            self.stop_event=threading.Event()
            self.channel=CameraChannel(clock=self.clock)
            self.state,self.error,self.latest,self.result='connecting',None,None,None
            self.verification_pending=False
            self.wifi_cleanup_verified=self.ble_cleanup_verified=self.video_cleanup_verified=True
            self.thread=threading.Thread(target=self._run,name='neo2-camera-session',daemon=False)
            self.thread.start()
        return {'accepted':True,'state':'connecting'}

    def disconnect(self):
        with self.lock:
            self.stop_event.set()
            self.channel.revoke()
            if self.thread is not None and self.thread.is_alive():self.state='stopping'
        return {'accepted':True,'state':self.status()['state']}

    def _publish(self,snapshot,*,generation=None):
        with self.lock:
            if generation is not None and generation!=self.generation:return
            self.latest=deepcopy(snapshot)
            self.latest['console_observed_at']=self.clock()
            if snapshot.get('video',{}).get('enabled') and not snapshot['video'].get('cleanup_verified'):
                self.video_cleanup_verified=False
            if self.state!='stopping' and snapshot.get('state')=='open':self.state='connected'

    def _ble_event(self,name,*,generation=None,**detail):
        with self.lock:
            if generation is not None and generation!=self.generation:return
            if name=='verification_requested':self.verification_pending=True
            elif name in ('verification_push','ble_cleanup_result','setup_result'):self.verification_pending=False
        if name=='ble_cleanup_result':
            with self.lock:
                if generation is not None and generation!=self.generation:return
                self.ble_cleanup_verified=detail.get('cleanup_verified') is True

    def _video_cleanup(self,snapshot,*,generation=None):
        with self.lock:
            if generation is not None and generation!=self.generation:return
            self.video_cleanup_verified=snapshot.get('cleanup_verified') is True

    def _save(self,name,value):
        target=self.paths['directory']/name
        fd=os.open(target,os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
        with os.fdopen(fd,'w') as stream:json.dump(value,stream,indent=2);stream.write('\n')

    def _run(self):
        try:
            self.paths['directory'].mkdir(mode=0o700,parents=True,exist_ok=False)
            with self.lease_factory(self.config.interface,self.config.ble_address):
                self.route_guard(self.config.interface,cancel_event=self.stop_event)
                asyncio.run(self._connect_and_run())
        except asyncio.CancelledError:
            if not self.stop_event.is_set():
                with self.lock:self.error='camera_connection_cancelled'
        except Exception as error:
            with self.lock:self.error=str(error) if isinstance(error,(ValueError,RuntimeError)) else 'camera_connection_failed'
        finally:
            self.channel.cancel(self.clock(),'session_closed')
            with self.lock:
                clean=all(x is True for x in (self.wifi_cleanup_verified,self.ble_cleanup_verified,self.video_cleanup_verified))
                if not clean and self.error is None:self.error='camera_cleanup_unconfirmed'
                self.state='failed' if self.error else 'disconnected'
            try:self._save('console-result.json',{'state':self.state,'error':self.error,
                'wifi_cleanup_verified':self.wifi_cleanup_verified,'ble_cleanup_verified':self.ble_cleanup_verified,
                'video_cleanup_verified':self.video_cleanup_verified,'session':self.result})
            except OSError:
                with self.lock:self.error='camera_audit_unavailable';self.state='failed'

    async def _connect_and_run(self):
        generation=self.generation
        if self.wifi_factory is None:
            if self.config.manual_macos_wifi:
                from .macos_wifi import ManualMacWifiLink
                factory=ManualMacWifiLink
            else:
                from .wifi_memory import MemoryWifiLink
                factory=MemoryWifiLink
        else:factory=self.wifi_factory
        entered_wifi=False
        session_invoked=False
        async def connected():
            nonlocal entered_wifi,session_invoked
            credentials=load_wifi_credentials(self.paths['credentials'])
            if self.config.manual_macos_wifi:
                self._save('join-network.json', {'ssid':credentials.ssid.decode('utf-8'),
                                                'password':credentials.passphrase.decode('utf-8')})
                print(json.dumps({'event':'manual_wifi_join_required',
                                  'private_instructions_file':str(self.paths['directory']/'join-network.json')}),flush=True)
            link=factory(credentials,self.config.interface,stop_event=self.stop_event)
            try:
                async with link:
                    entered_wifi=True
                    with self.lock:self.wifi_cleanup_verified=False
                    with self.lock:self.video_cleanup_verified=None
                    session_invoked=True
                    result=await asyncio.to_thread(self.session_runner,capture_path=self.paths['capture'],
                        interface=self.config.interface,duration=self.config.duration,heartbeat=True,
                        wlm_identity_file=self.config.wlm_identity_file,wlm_wifi_liveview=True,
                        **({'preview_only':True} if self.config.preview_only else {}),
                        **({'network_guard':link.ready} if self.config.manual_macos_wifi else {}),
                        **({'wlm_forwarding_sequence':self.config.wlm_forwarding_sequence}
                           if self.config.wlm_forwarding_sequence!=1 else {}),
                        camera_channel=self.channel,preview_jpeg=True,
                        video_path=self.paths['video'],hls_directory=self.paths['hls'],video_codec='hevc',
                        snapshot_sink=lambda snapshot:self._publish(snapshot,generation=generation),stop_event=self.stop_event,
                        video_cleanup_sink=lambda snapshot:self._video_cleanup(snapshot,generation=generation))
                    with self.lock:
                        self.result=result.as_dict()
                        self.state='stopping'
                        if result.state=='failed':self.error=result.reason
            finally:
                metadata=link.snapshot()
                with self.lock:self.wifi_cleanup_verified=metadata.get('cleanup_verified') is True
                self._save('wifi-result.json',metadata)
        try:
            with self.lock:self.ble_cleanup_verified=None
            result=await self.ble_runner(address=self.config.ble_address,
                **({'preview_only':True} if self.config.preview_only else {}),
                identity_file=self.config.ble_identity_file,wlm_identity_file=self.config.wlm_identity_file,
                credentials_path=self.paths['credentials'],capture_path=self.paths['ble'],timeout=45,
                stop_event=self.stop_event,while_connected=connected,
                hold_timeout=self.config.duration+(660 if self.config.manual_macos_wifi else 60),
                event_sink=lambda name,**detail:self._ble_event(name,generation=generation,**detail))
            self._save('ble-result.json',result.as_dict())
            with self.lock:
                self.ble_cleanup_verified=result.connection_cleanup_verified
                if result.state=='failed' and self.error is None:self.error=result.reason
        finally:
            with self.lock:
                if not entered_wifi and self.wifi_cleanup_verified is None:self.wifi_cleanup_verified=True
                if not session_invoked and self.video_cleanup_verified is None:self.video_cleanup_verified=True

    def command(self,action,parameters):
        if self.config.preview_only:raise RuntimeError('preview_only_camera_actions_disabled')
        with self.lock:
            generation,channel=self.generation,self.channel
            status=self.status()
            if not status['connected']:raise RuntimeError('drone_not_connected')
            if generation!=self.generation or channel is not self.channel:
                raise RuntimeError('camera_session_changed')
            return channel.submit(action,parameters)

    def frame_path(self):
        with self.lock:
            paths=dict(self.paths)
        preview=paths['hls']/'preview.jpg' if 'hls' in paths else None
        if preview is not None and preview.is_file():return preview
        reference=Path(self.config.last_frame) if self.config.last_frame is not None else None
        return reference if reference is not None and reference.is_file() else None

    def network_instructions(self):
        if not self.config.manual_macos_wifi:return None
        with self.lock:
            path=self.paths.get('credentials')
            generation=self.generation
        if path is None or not path.is_file():return None
        credentials=load_wifi_credentials(path)
        return {'session_generation':generation,'ssid':credentials.ssid.decode('utf-8'),
                'password':credentials.passphrase.decode('utf-8'), 'bssid_hex':credentials.bssid.hex()}

    def status(self):
        now=self.clock()
        with self.lock:
            latest=deepcopy(self.latest) or {}
            worker_alive=self.thread is not None and self.thread.is_alive()
            state,error=self.state,self.error
            session_preview=self.paths['hls']/'preview.jpg' if 'hls' in self.paths else None
            cleanup=all(x is True for x in (self.wifi_cleanup_verified,self.ble_cleanup_verified,self.video_cleanup_verified))
        elapsed=max(0,now-latest.get('console_observed_at',now))
        connected=worker_alive and state=='connected' and latest.get('state')=='open'
        received_age=latest.get('last_received_age_seconds')
        if received_age is not None and received_age+elapsed>2:connected=False
        camera=self.channel.snapshot()
        preview=self.frame_path()
        frame={'available':False,'version':None,'kind':None}
        if preview is not None:
            try:
                stat=preview.stat();frame={'available':True,'version':f'{stat.st_mtime_ns}-{stat.st_size}',
                    'kind':'session' if preview==session_preview else 'last_capture'}
            except OSError:pass
        video=latest.get('video',{})
        wlm=latest.get('wlm_mode_follow',{})
        encoded_age=video.get('last_encoded_age_seconds')
        preview_age=video.get('preview_age_seconds')
        live=bool(connected and frame['available'] and frame['kind']=='session'
                  and encoded_age is not None and encoded_age+elapsed<=2
                  and preview_age is not None and preview_age+elapsed<=3)
        battery=None
        for observation in latest.get('battery',{}).get('observations',[]):
            field=observation.get('fields',{}).get('percent',{})
            if observation.get('age_seconds',float('inf'))+elapsed<=1 and field.get('valid'):
                battery=field.get('value')
        labels={'disconnected':'Au repos','connecting':'Connexion…','connected':'Connecté','stopping':'Fermeture…','failed':'Connexion interrompue'}
        label=labels.get(state,state)
        if connected and not camera['ready']:label='Préparation caméra…'
        if error is None:
            if video.get('state')=='failed':error='camera_video_unavailable'
            elif wlm.get('state')=='failed':error='camera_link_interrupted'
            elif wlm.get('service_switch',{}).get('state')=='failed':error='camera_setup_incomplete'
            elif (wlm.get('capability_read',{}).get('state') in ('timed_out','response_error')
                    and wlm.get('capability_query_retry_in_seconds') is None):
                error='camera_setup_incomplete'
        command=camera['last_command']
        if command is not None:
            command['label']={'queued':'En attente','scheduled':'Prête à envoyer','sent':'Envoyée · attente de réponse',
                'waiting_mode':'Préparation du mode caméra','acknowledged':'Acceptée par la caméra',
                'sent_unconfirmed':'Envoyée · mouvement à vérifier sur l’image','rejected':'Refusée par la caméra',
                'timed_out':'Délai de réponse dépassé','cancelled':'Annulée','expired':'Expirée'}.get(command['state'],command['state'])
        return {'state':state,'state_label':label,'connected':bool(connected),
                'session_generation':self.generation,
                'verification_pending':self.verification_pending,
                'preview_only':self.config.preview_only,
                'camera_ready':bool(connected and camera['ready']),
                'frame':frame,'video':{'live':live,'state':video.get('state'),
                    'frames_encoded':video.get('frames_encoded',0)},
                'battery_percent':battery if connected else None,'error':error,
                'capabilities':({key:False for key in camera['capabilities']} if self.config.preview_only else camera['capabilities']),'pitch_limits':camera['pitch_limits'],
                'commanded_pitch':camera['commanded_pitch'],'recording':camera['recording'],
                'last_command':command,'cleanup_verified':cleanup,
                'flight_commands_available':False}

    def close(self):
        with self.lock:self.closing=True
        self.disconnect()
        thread=self.thread
        if thread is not None:
            thread.join(timeout=60)
            if thread.is_alive():raise RuntimeError('camera_worker_did_not_stop')
        if not self.status()['cleanup_verified']:raise RuntimeError('camera_cleanup_unconfirmed')
