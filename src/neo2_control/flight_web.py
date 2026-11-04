"""Loopback operator interface for the independent owned flight process."""
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from importlib.resources import files
import hmac
import json
import os
from pathlib import Path
import re
import secrets
import signal
import stat
import threading
import time
from urllib.parse import urlsplit

from .control_owner import ProcessControllerRuntime
from .controller import IntentRejected
from .flight_connection import FlightConfig
from .command_ingress import CommandWindows
from .remote_target import RemoteTargetWindows, target_horizon

MAX_FRAME_BYTES=4*1024*1024
CAMERA_ACTIONS=frozenset(('pitch','recenter','photo','record','stop_record'))


class FlightWeb:
    def __init__(self,config,journal_path,*,runtime_factory=ProcessControllerRuntime):
        if not isinstance(config,FlightConfig) or not config.camera_preview:
            raise ValueError('flight web requires camera preview configuration')
        self.config=config
        self.lock=threading.RLock()
        self.runtime_generation=0
        self.runtime=runtime_factory(journal_path,mode='flight',flight_config=config)
        self.windows=CommandWindows(clock=lambda:time.monotonic())
        self.remote_windows=RemoteTargetWindows(clock=lambda:time.monotonic())

    def status(self):
        with self.lock:return self.runtime.status()

    def window(self):
        with self.lock:
            context=self._command_context()
            return self.windows.issue(context['owner_epoch'],context['connection_generation'])

    def remote_window(self,round_trip_seconds=None):
        with self.lock:
            context=self._command_context()
            return self.remote_windows.issue(context['owner_epoch'],context['connection_generation'],
                round_trip_seconds=round_trip_seconds)

    def remote_command(self,action,parameters,*,camera=False,ingress,
                       round_trip_seconds=None,expected_generation=None):
        if not isinstance(action,str) or not isinstance(parameters,dict):raise ValueError('invalid_intent')
        if camera and action not in CAMERA_ACTIONS:raise ValueError('invalid_camera_intent')
        target_horizon(round_trip_seconds)  # Reject an invalid hint before any mutation.
        with self.lock:
            if expected_generation is not None and expected_generation!=self.runtime_generation:
                raise IntentRejected('controller_owner_changed')
            context=self._command_context()
            try:source=self.remote_windows.consume(ingress,owner_epoch=context['owner_epoch'],
                connection_generation=context['connection_generation'])
            except ValueError as error:raise IntentRejected(str(error)) from None
            if not camera and action=='recover_owner':
                if parameters:raise ValueError('recovery_takes_no_parameters')
                self.runtime=self.runtime.recover_owner(ingress=source.local_execution(time.monotonic()))
                self.runtime_generation+=1
                result={'accepted':True}
            else:
                method=self.runtime.remote_camera_command if camera else self.runtime.remote_handle
                result=method(action,parameters,ingress=source)
            # A new window is for the next gesture only. Never renew the source
            # just executed or retry a mutation whose response was lost.
            return {**result,'status':self.runtime.status(),
                'next_window':self.remote_window(round_trip_seconds)}

    def _command_context(self):
        context=getattr(self.runtime,'command_context',None)
        if callable(context):return context()
        return {'owner_epoch':self.runtime.epoch,
            'connection_generation':self.runtime.status().get('connection',{}).get('generation')}

    def _consume_window(self,value):
        if value is None:return None  # Internal, local Python calls only.
        context=self._command_context()
        try:return self.windows.consume(value,owner_epoch=context['owner_epoch'],
            connection_generation=context['connection_generation'])
        except ValueError as error:raise IntentRejected(str(error)) from None

    def intent(self,action,parameters,*,expected_generation=None,ingress=None):
        if not isinstance(action,str) or not isinstance(parameters,dict):raise ValueError('invalid_intent')
        generation=self.runtime_generation if expected_generation is None else expected_generation
        if action=='control':
            if set(parameters)!={'lease','sequence','channels'}:raise ValueError('invalid_control_fields')
            parameters={**parameters,'expires_at':time.monotonic()+.2}
        with self.lock:
            if generation!=self.runtime_generation:raise IntentRejected('controller_owner_changed')
            context=self._consume_window(ingress)
            options={} if context is None else {'ingress':context}
            if context is not None and action=='control':
                parameters={**parameters,'expires_at':min(context.expires_at,context.issued_at+.2)}
            if action=='recover_owner':
                if parameters:raise ValueError('recovery_takes_no_parameters')
                self.runtime=self.runtime.recover_owner(**options)
                self.runtime_generation+=1
                return {'accepted':True,'status':self.runtime.status()}
            return self.runtime.handle(action,parameters,**options)

    def camera_command(self,action,parameters,*,expected_generation=None,ingress=None):
        if not isinstance(action,str) or action not in CAMERA_ACTIONS or not isinstance(parameters,dict):raise ValueError('invalid_camera_intent')
        generation=self.runtime_generation if expected_generation is None else expected_generation
        with self.lock:
            if generation!=self.runtime_generation:raise IntentRejected('controller_owner_changed')
            context=self._consume_window(ingress)
            return self.runtime.camera_command(action,parameters,
                **({} if context is None else {'ingress':context}))

    def frame_path(self):
        with self.lock:
            generation=self.status().get('connection',{}).get('generation')
            if not isinstance(generation,str) or not re.fullmatch('[0-9a-f]{24}',generation):return None
            return Path(self.config.capture_directory)/generation/'hls'/'preview.jpg'

    def read_frame(self):
        with self.lock:
            owner=self.runtime.epoch
            generation=self.runtime.status().get('connection',{}).get('generation')
            if not isinstance(generation,str) or not re.fullmatch('[0-9a-f]{24}',generation):
                raise FileNotFoundError
            target=Path(self.config.capture_directory)/generation/'hls'/'preview.jpg'
            fd=os.open(target,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
            with os.fdopen(fd,'rb') as stream:
                before=os.fstat(stream.fileno())
                started=time.monotonic();wall=time.time_ns()
                if not stat.S_ISREG(before.st_mode) or before.st_mtime_ns>wall:
                    raise FileNotFoundError
                body=stream.read(MAX_FRAME_BYTES+1)
                after=os.fstat(stream.fileno())
                elapsed=time.monotonic()-started
                wall_elapsed=(time.time_ns()-wall)/1e9
            if len(body)>MAX_FRAME_BYTES:raise ValueError('frame_too_large')
            if ((before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns)
                    or elapsed<0 or abs(wall_elapsed-elapsed)>.1):raise FileNotFoundError
            if (owner!=self.runtime.epoch or generation!=
                    self.runtime.status().get('connection',{}).get('generation')):
                raise FileNotFoundError
            age=(wall-before.st_mtime_ns)/1e9+elapsed
            identity=f'{before.st_dev:x}:{before.st_ino:x}:{before.st_mtime_ns:x}:{before.st_size:x}'
            return body,{'X-Neo2-Connection-Generation':generation,
                'X-Neo2-Frame-Age-Seconds':f'{age:.9f}','X-Neo2-Frame-Id':identity}

    def close(self):
        with self.lock:self.runtime.close()


def build_flight_handler(app,token):
    if not isinstance(token,str) or not re.fullmatch('[0-9a-f]{64}',token):raise ValueError('invalid_flight_token')
    assets=files('neo2_control').joinpath('web')
    class Handler(BaseHTTPRequestHandler):
        server_version='neo2-flight/0.1'

        def _reply(self,status,body,kind,extra=None):
            self.send_response(status)
            for key,value in {'Content-Type':kind,'Content-Length':str(len(body)),'Cache-Control':'no-store',
                'X-Content-Type-Options':'nosniff','Referrer-Policy':'no-referrer','X-Frame-Options':'DENY',
                'Content-Security-Policy':"default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'"}.items():
                self.send_header(key,value)
            for key,value in (extra or {}).items():self.send_header(key,value)
            try:self.end_headers();self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass  # Superseded browser fetch; never replay.

        def _json(self,status,value):
            self._reply(status,json.dumps(value,ensure_ascii=False,allow_nan=False).encode(),'application/json; charset=utf-8')

        def _local(self):
            hosts={f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'}
            if self.headers.get('Host') not in hosts or (self.headers.get('Origin') and
                    self.headers['Origin'] not in {'http://'+h for h in hosts}):
                self._json(HTTPStatus.FORBIDDEN,{'error':'invalid_origin_or_host'});return False
            return True

        def _authorized(self):
            if hmac.compare_digest(self.headers.get('X-Neo2-Token','').encode(),token.encode()):return True
            self._json(HTTPStatus.UNAUTHORIZED,{'error':'unauthorized'});return False

        def do_GET(self):
            if not self._local():return
            path=urlsplit(self.path).path
            if path=='/':
                page=assets.joinpath('flight.html').read_text().replace('__NEO2_TOKEN__',token)
                page=page.replace('name="neo2-api-prefix" content="/api/flight"',
                    'name="neo2-api-prefix" content="/api/remote-flight"').replace(
                    'name="neo2-command-mode" content="native"','name="neo2-command-mode" content="remote-target"')
                self._reply(HTTPStatus.OK,page.encode(),'text/html; charset=utf-8');return
            if path in ('/flight.js','/flight.css','/camera.css','/favicon.svg'):
                kind='image/svg+xml' if path.endswith('.svg') else 'text/javascript' if path.endswith('.js') else 'text/css'
                self._reply(HTTPStatus.OK,assets.joinpath(path[1:]).read_bytes(),kind+'; charset=utf-8');return
            if not self._authorized():return
            try:
                if path=='/api/flight/status':self._json(HTTPStatus.OK,app.status())
                elif path=='/api/flight/window':self._json(HTTPStatus.OK,app.window())
                elif path=='/api/remote-flight/status':self._json(HTTPStatus.OK,app.status())
                elif path=='/api/remote-flight/window':self._json(HTTPStatus.OK,app.remote_window())
                elif path in ('/api/flight/frame','/api/remote-flight/frame'):
                    body,metadata=app.read_frame()
                    self._reply(HTTPStatus.OK,body,'image/jpeg',metadata)
                else:self._json(HTTPStatus.NOT_FOUND,{'error':'not_found'})
            except OSError:self._json(HTTPStatus.NOT_FOUND,{'error':'frame_unavailable'})
            except RuntimeError:self._json(HTTPStatus.SERVICE_UNAVAILABLE,{'error':'controller_owner_unavailable'})
            except ValueError as error:self._json(HTTPStatus.BAD_REQUEST,{'error':str(error)})

        def do_POST(self):
            if not self._local() or not self._authorized():return
            generation=app.runtime_generation
            path=urlsplit(self.path).path
            remote=path.startswith('/api/remote-flight/')
            if path not in ('/api/flight/intent','/api/flight/camera','/api/remote-flight/intent','/api/remote-flight/camera'):
                self._json(HTTPStatus.NOT_FOUND,{'error':'not_found'});return
            try:
                if self.headers.get('Content-Type','').split(';')[0]!='application/json':raise ValueError('json_required')
                if self.headers.get('Transfer-Encoding'):raise ValueError('invalid_request_size')
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=8192:raise ValueError('invalid_request_size')
                def constant(_):raise ValueError('nonfinite_number')
                def pairs(items):
                    result={}
                    for key,value in items:
                        if key in result:raise ValueError('duplicate_json_key')
                        result[key]=value
                    return result
                body=json.loads(self.rfile.read(size),parse_constant=constant,object_pairs_hook=pairs)
                fields={'action','parameters','ingress'}|({'round_trip_seconds'} if remote else set())
                if not isinstance(body,dict) or set(body)!=fields:raise ValueError('invalid_request_fields')
                if not isinstance(body['ingress'],dict):raise ValueError('operator_window_required')
                action,parameters=body['action'],body['parameters']
                if not isinstance(action,str) or not isinstance(parameters,dict):raise ValueError('invalid_intent')
                if remote:
                    result=app.remote_command(action,parameters,camera=path.endswith('/camera'),
                        expected_generation=generation,ingress=body['ingress'],round_trip_seconds=body['round_trip_seconds'])
                else:
                    result=(app.intent(action,parameters,expected_generation=generation,ingress=body['ingress']) if path.endswith('/intent')
                        else app.camera_command(action,parameters,expected_generation=generation,ingress=body['ingress']))
                self._json(HTTPStatus.ACCEPTED,{**result,'server_time':time.monotonic()})
            except IntentRejected as error:self._json(HTTPStatus.CONFLICT,{'error':str(error)})
            except (ValueError,TypeError) as error:self._json(HTTPStatus.BAD_REQUEST,{'error':str(error)})
            except RuntimeError:self._json(HTTPStatus.SERVICE_UNAVAILABLE,{'error':'controller_owner_unavailable'})

        def log_message(self,*args):pass
    return Handler


def serve_flight_web(app,*,port=8794):
    if type(port) is not int or not 0<=port<=65535:raise ValueError('invalid_flight_port')
    token=secrets.token_hex(32);main=threading.current_thread() is threading.main_thread()
    previous=signal.getsignal(signal.SIGTERM) if main else None
    def stop(*args):raise KeyboardInterrupt
    if main:signal.signal(signal.SIGTERM,stop)
    try:
        with ThreadingHTTPServer(('127.0.0.1',port),build_flight_handler(app,token)) as server:
            server.daemon_threads=True
            print(json.dumps({'event':'flight_web_started','url':f'http://127.0.0.1:{server.server_port}/'}),flush=True)
            try:server.serve_forever()
            except KeyboardInterrupt:pass
    finally:
        try:app.close()
        finally:
            if main:signal.signal(signal.SIGTERM,previous)
