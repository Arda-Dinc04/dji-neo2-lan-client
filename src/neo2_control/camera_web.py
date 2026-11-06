"""Local browser interface for the owned camera connection, without flight intents."""
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
import hmac
import json
from pathlib import Path
import re
import secrets
import signal
import threading
from urllib.parse import urlsplit

CAMERA_ACTIONS=frozenset(('pitch','recenter','photo','record','stop_record'))
MAX_FRAME_BYTES=4*1024*1024


def build_camera_handler(console, token):
    if not isinstance(token,str) or not re.fullmatch('[0-9a-f]{64}',token):
        raise ValueError('camera token requires32random bytes in hexadecimal')
    assets=files('neo2_control').joinpath('web')

    class Handler(BaseHTTPRequestHandler):
        server_version='neo2-camera/0.1'

        def _local(self):
            expected={f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'}
            if self.headers.get('Host') not in expected:
                self._json(HTTPStatus.FORBIDDEN,{'error':'invalid_host'})
                return False
            origin=self.headers.get('Origin')
            if origin and origin not in {f'http://{host}' for host in expected}:
                self._json(HTTPStatus.FORBIDDEN,{'error':'invalid_origin'})
                return False
            return True

        def _reply(self,status,body,content_type):
            self.send_response(status)
            for name,value in {'Content-Type':content_type,'Content-Length':str(len(body)),
                    'Cache-Control':'no-store','X-Content-Type-Options':'nosniff',
                    'Referrer-Policy':'no-referrer','X-Frame-Options':'DENY',
                    'Content-Security-Policy':"default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'"}.items():
                self.send_header(name,value)
            self.end_headers()
            self.wfile.write(body)

        def _json(self,status,value):
            self._reply(status,json.dumps(value,ensure_ascii=False,allow_nan=False).encode(),'application/json; charset=utf-8')

        def _authorized(self):
            candidate=self.headers.get('X-Neo2-Token','')
            if hmac.compare_digest(candidate.encode(),token.encode()):return True
            self._json(HTTPStatus.UNAUTHORIZED,{'error':'unauthorized'})
            return False

        def do_GET(self):
            if not self._local():return
            path=urlsplit(self.path).path
            if path=='/':
                body=assets.joinpath('camera.html').read_text().replace('__NEO2_TOKEN__',token)
                self._reply(HTTPStatus.OK,body.encode(),'text/html; charset=utf-8')
                return
            if path in ('/camera.css','/camera.js','/favicon.svg'):
                name=path[1:]
                kind={'camera.css':'text/css','camera.js':'text/javascript','favicon.svg':'image/svg+xml'}[name]
                self._reply(HTTPStatus.OK,assets.joinpath(name).read_bytes(),kind+'; charset=utf-8')
                return
            if not self._authorized():return
            if path=='/api/camera/status':
                self._json(HTTPStatus.OK,console.status())
            elif path=='/api/camera/network':
                instructions=console.network_instructions()
                if instructions is None:self._json(HTTPStatus.NOT_FOUND,{'error':'network_details_unavailable'})
                else:self._json(HTTPStatus.OK,instructions)
            elif path=='/api/camera/frame':
                target=console.frame_path()
                if target is None:
                    self._json(HTTPStatus.NOT_FOUND,{'error':'frame_unavailable'})
                    return
                target=Path(target)
                if target.suffix.lower() not in ('.jpg','.jpeg','.png'):
                    self._json(HTTPStatus.NOT_FOUND,{'error':'frame_unavailable'})
                    return
                try:
                    with target.open('rb') as stream:body=stream.read(MAX_FRAME_BYTES+1)
                except OSError:
                    self._json(HTTPStatus.NOT_FOUND,{'error':'frame_unavailable'})
                    return
                if len(body)>MAX_FRAME_BYTES:
                    self._json(HTTPStatus.CONFLICT,{'error':'frame_exceeds_limit'})
                    return
                self._reply(HTTPStatus.OK,body,'image/png' if target.suffix.lower()=='.png' else 'image/jpeg')
            else:self._json(HTTPStatus.NOT_FOUND,{'error':'not_found'})

        def do_POST(self):
            if not self._local() or not self._authorized():return
            path=urlsplit(self.path).path
            if path not in ('/api/camera/connect','/api/camera/disconnect','/api/camera/command'):
                self._json(HTTPStatus.NOT_FOUND,{'error':'not_found'})
                return
            try:
                if self.headers.get('Content-Type','').split(';',1)[0]!='application/json':
                    raise ValueError('json_content_type_required')
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=16384:raise ValueError('invalid_request_size')
                def invalid(value):raise ValueError('nonfinite_number')
                payload=json.loads(self.rfile.read(size),parse_constant=invalid)
                if not isinstance(payload,dict):raise ValueError('json_object_required')
                if path.endswith('/command'):
                    if set(payload)-{'action','parameters'}:raise ValueError('unknown_request_fields')
                    action,parameters=payload.get('action'),payload.get('parameters',{})
                    if not isinstance(action,str) or action not in CAMERA_ACTIONS:
                        raise ValueError('unsupported_camera_action')
                    if not isinstance(parameters,dict):raise ValueError('parameters_object_required')
                    result=console.command(action,parameters)
                else:
                    if payload:raise ValueError('connection_takes_no_parameters')
                    result=console.connect() if path.endswith('/connect') else console.disconnect()
            except ValueError as error:
                self._json(HTTPStatus.BAD_REQUEST,{'error':str(error)})
            except RuntimeError as error:
                self._json(HTTPStatus.CONFLICT,{'error':str(error)})
            else:self._json(HTTPStatus.ACCEPTED,result)

        def log_message(self,*args):pass
    return Handler


def serve_camera_web(console, *, port=8793):
    if type(port) is not int or not 0<=port<=65535:raise ValueError('invalid_camera_port')
    token=secrets.token_hex(32)
    main=threading.current_thread() is threading.main_thread()
    previous=signal.getsignal(signal.SIGTERM) if main else None
    def stop(*args):raise KeyboardInterrupt
    if main:signal.signal(signal.SIGTERM,stop)
    try:
        with ThreadingHTTPServer(('127.0.0.1',port),build_camera_handler(console,token)) as server:
            server.daemon_threads=True
            print(json.dumps({'event':'camera_web_started','url':f'http://127.0.0.1:{server.server_port}/'}),flush=True)
            try:server.serve_forever()
            except KeyboardInterrupt:pass
    finally:
        try:console.close()
        finally:
            if main:signal.signal(signal.SIGTERM,previous)
