"""Real local HTTP checks for camera-only requests and private frame serving."""
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
import json
import threading
from urllib.error import HTTPError
from urllib.request import Request,urlopen

import pytest

from neo2_control.camera_web import build_camera_handler

TOKEN='a'*64


class Console:
    def __init__(self,path):self.path=path;self.calls=[]
    def status(self):return {'state':'disconnected','connected':False,'camera_ready':False}
    def frame_path(self):return self.path
    def connect(self):self.calls.append('connect');return {'state':'connecting'}
    def disconnect(self):self.calls.append('disconnect');return {'state':'stopping'}
    def command(self,action,parameters):self.calls.append((action,parameters));return {'accepted':True}


@contextmanager
def server(tmp_path):
    frame=tmp_path/'frame.jpg';frame.write_bytes(b'\xff\xd8camera-unit-fixture\xff\xd9')
    console=Console(frame)
    http=ThreadingHTTPServer(('127.0.0.1',0),build_camera_handler(console,TOKEN))
    worker=threading.Thread(target=http.serve_forever,daemon=True);worker.start()
    try:yield f'http://127.0.0.1:{http.server_port}',console
    finally:http.shutdown();http.server_close();worker.join(1)
    assert not worker.is_alive()


def request(base,path,*,payload=None,headers=None,auth=True):
    h={'X-Neo2-Token':TOKEN} if auth else {}
    if payload is not None:h['Content-Type']='application/json'
    if headers:h.update(headers)
    data=json.dumps(payload).encode() if payload is not None else None
    req=Request(base+path,data=data,headers=h)
    try:response=urlopen(req,timeout=2)
    except HTTPError as error:response=error
    with response:return response.status,dict(response.headers),response.read()


def test_page_and_packaged_assets_serve_with_real_offline_status_and_no_secret_in_logs(tmp_path):
    with server(tmp_path) as (base,console):
        status,headers,body=request(base,'/',auth=False)
        assert status==200 and TOKEN.encode() in body and b'Cam\xc3\xa9ra frontale' in body
        assert '__NEO2_TOKEN__' not in body.decode()
        assert headers['Cache-Control']=='no-store' and "frame-ancestors 'none'" in headers['Content-Security-Policy']
        for path,kind in (('/camera.css','text/css'),('/camera.js','text/javascript')):
            status,headers,body=request(base,path,auth=False)
            assert status==200 and headers['Content-Type'].startswith(kind) and len(body)>100
        status,headers,body=request(base,'/api/camera/status')
        assert status==200 and json.loads(body)['camera_ready'] is False
        assert not console.calls


def test_frames_are_served_only_after_local_authentication_and_never_by_requested_path(tmp_path):
    with server(tmp_path) as (base,console):
        assert request(base,'/api/camera/frame',auth=False)[0]==401
        status,headers,body=request(base,'/api/camera/frame')
        assert status==200 and headers['Content-Type']=='image/jpeg'
        assert body==console.path.read_bytes()
        assert request(base,'/../../etc/passwd')[0]==404
        console.path=None
        assert request(base,'/api/camera/frame')[0]==404


def test_network_instructions_are_local_authenticated_and_never_cached(tmp_path):
    with server(tmp_path) as (base,console):
        console.network_instructions=lambda:{'session_generation':'synthetic','ssid':'synthetic-aircraft',
                                             'password':'synthetic-test-key'}
        assert request(base,'/api/camera/network',auth=False)[0]==401
        assert request(base,'/api/camera/network',headers={'Origin':'https://foreign.invalid'})[0]==403
        status,headers,body=request(base,'/api/camera/network')
        assert status==200 and json.loads(body)['password']=='synthetic-test-key'
        assert headers['Cache-Control']=='no-store'
        assert not console.calls
        console.network_instructions=lambda:None
        assert request(base,'/api/camera/network')[0]==404


@pytest.mark.parametrize('headers',[{'Host':'foreign.invalid'},{'Origin':'https://foreign.invalid'},
                                      {'X-Neo2-Token':'incorrect'}])
def test_foreign_host_origin_and_invalid_token_cannot_trigger_effects(tmp_path,headers):
    with server(tmp_path) as (base,console):
        assert request(base,'/api/camera/connect',payload={},headers=headers)[0] in (401,403)
        assert not console.calls


def test_only_camera_actions_can_reach_console_and_connection_configuration_is_local(tmp_path):
    with server(tmp_path) as (base,console):
        for action in ('takeoff','land','arm','raw_duml','update_firmware'):
            assert request(base,'/api/camera/command',payload={'action':action})[0]==400
        assert request(base,'/api/camera/connect',payload={'interface':'foreign0'})[0]==400
        assert not console.calls
        assert request(base,'/api/camera/connect',payload={})[0]==202
        assert request(base,'/api/camera/command',payload={'action':'pitch','parameters':{'degrees':-10}})[0]==202
        assert request(base,'/api/camera/disconnect',payload={})[0]==202
        assert console.calls==['connect',('pitch',{'degrees':-10}),'disconnect']


def test_cross_site_form_and_nonfinite_parameters_are_rejected_before_effect(tmp_path):
    with server(tmp_path) as (base,console):
        assert request(base,'/api/camera/connect',payload={},headers={'Content-Type':'text/plain'})[0]==400
        assert request(base,'/api/camera/command',payload={'action':'pitch','parameters':{'degrees':float('nan')}})[0]==400
        assert request(base,'/api/camera/command',payload={'action':'photo','raw_hex':'1234'})[0]==400
        assert not console.calls
