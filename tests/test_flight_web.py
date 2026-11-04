"""Real loopback HTTP with an inert runtime; no aircraft connection."""
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
import json
import secrets
import threading
import time
from types import SimpleNamespace

import pytest

from neo2_control.controller import IntentRejected
from neo2_control.flight_connection import FlightConfig
from neo2_control.flight_web import FlightWeb,build_flight_handler
from test_camera_web import request as raw_request

TOKEN='a'*64


def request(base,path,*,payload=None,headers=None,auth=True):
    # Exercise the actual two-request browser protocol for existing scenarios.
    if payload is not None and isinstance(payload,dict) and set(payload)=={'action','parameters'}:
        code,_,body=raw_request(base,'/api/flight/window',headers=headers,auth=auth)
        if code==200:
            window=json.loads(body)
            payload={**payload,'ingress':window_ingress(window)}
    return raw_request(base,path,payload=payload,headers=headers,auth=auth)


def window_ingress(window):
    return {'ticket':window['ticket'],'mutation_sequence':window['mutation_sequence'],
        'expected_owner_epoch':window['owner_epoch'],
        'expected_connection_generation':window['connection_generation']}


class Runtime:
    def __init__(self,*args,**kwargs):
        self.calls=[];self.closed=False;self.epoch=secrets.token_hex(24);self.contexts=[]
    def status(self):return {'state':'disconnected','connection':{'generation':'b'*24},
        'controller_owner':{'epoch':self.epoch}}
    def handle(self,action,parameters,*,ingress=None):
        if ingress is not None:ingress.validate(time.monotonic());self.contexts.append(ingress)
        if action=='rejected':raise IntentRejected('invalid_state')
        self.calls.append((action,parameters));return {}
    def camera_command(self,action,parameters,*,ingress=None):
        if ingress is not None:ingress.validate(time.monotonic());self.contexts.append(ingress)
        self.calls.append((action,parameters));return {'accepted':True}
    def close(self):self.closed=True


@contextmanager
def server(tmp_path,*,runtime_factory=Runtime):
    config=FlightConfig('fixture0',tmp_path/'wlm.json',tmp_path/'ble.json',
        capture_directory=tmp_path/'captures',camera_preview=True,incident_directory=tmp_path/'incidents')
    app=FlightWeb(config,tmp_path/'journal.jsonl',runtime_factory=runtime_factory)
    http=ThreadingHTTPServer(('127.0.0.1',0),build_flight_handler(app,TOKEN))
    worker=threading.Thread(target=http.serve_forever,daemon=True);worker.start()
    try:yield f'http://127.0.0.1:{http.server_port}',app
    finally:http.shutdown();http.server_close();worker.join(1);app.close()
    assert not worker.is_alive()


def test_page_load_and_polling_never_start_connection_or_renew_operator(tmp_path):
    with server(tmp_path) as (base,app):
        code,headers,body=request(base,'/',auth=False)
        assert code==200 and b'Contr\xc3\xb4le PC' in body and TOKEN.encode() in body
        assert b'name="neo2-command-mode" content="remote-target"' in body
        assert b'name="neo2-api-prefix" content="/api/remote-flight"' in body
        assert "frame-ancestors 'none'" in headers['Content-Security-Policy']
        for asset in ('flight.js','flight.css','camera.css','favicon.svg'):
            assert request(base,'/'+asset,auth=False)[0]==200
        assert request(base,'/api/flight/status')[0]==200
        assert not app.runtime.calls and not app.config.capture_directory.exists()


@pytest.mark.parametrize('headers',[{'Host':'foreign.invalid'},{'Origin':'https://foreign.invalid'},
    {'X-Neo2-Token':'invalid'}])
def test_foreign_pages_cannot_open_or_send_flight_commands(tmp_path,headers):
    with server(tmp_path) as (base,app):
        assert request(base,'/api/flight/intent',payload={'action':'open','parameters':{}},headers=headers)[0] in (401,403)
        assert not app.runtime.calls


def test_control_validity_is_assigned_by_server_and_camera_is_typed(tmp_path):
    with server(tmp_path) as (base,app):
        before=time.monotonic()
        p={'lease':'fixture','sequence':1,'channels':[0,0,0,0]}
        assert request(base,'/api/flight/intent',payload={'action':'control','parameters':p})[0]==202
        action,parameters=app.runtime.calls[-1]
        assert action=='control' and before+.19<=parameters['expires_at']<=time.monotonic()+.2
        p['expires_at']=time.monotonic()+999
        assert request(base,'/api/flight/intent',payload={'action':'control','parameters':p})[0]==400
        assert request(base,'/api/flight/camera',payload={'action':'takeoff','parameters':{}})[0]==400
        assert len(app.runtime.calls)==1
        assert request(base,'/api/flight/camera',payload={'action':'pitch','parameters':{'degrees':-10}})[0]==202
        assert app.runtime.calls[-1]==('pitch',{'degrees':-10})
        assert request(base,'/api/flight/intent',payload={'action':'rejected','parameters':{}})[0]==409


def test_only_current_generation_frame_can_be_read_and_symlinks_are_rejected(tmp_path):
    with server(tmp_path) as (base,app):
        target=app.frame_path();target.parent.mkdir(parents=True);target.write_bytes(b'\xff\xd8fixture\xff\xd9')
        assert request(base,'/api/flight/frame',auth=False)[0]==401
        assert request(base,'/api/flight/frame')[2]==target.read_bytes()
        assert request(base,'/../../etc/passwd')[0]==404
        target.unlink();target.symlink_to(tmp_path/'wlm.json')
        assert request(base,'/api/flight/frame')[0]==404
        app.runtime.status=lambda:{'connection':{'generation':'../../etc'}}
        assert request(base,'/api/flight/frame')[0]==404


def test_malformed_and_nonfinite_requests_have_no_effects(tmp_path):
    with server(tmp_path) as (base,app):
        for payload in ({'action':'open'}, {'action':'open','parameters':{},'extra':1},
            {'action':'control','parameters':{'channels':[float('nan')]}},
            {'action':'open','parameters':[]}):
            assert request(base,'/api/flight/intent',payload=payload)[0]==400
        assert request(base,'/api/flight/intent',payload={'action':'open','parameters':{}},
                       headers={'Content-Type':'text/plain'})[0]==400
        assert not app.runtime.calls


def test_real_owner_stays_inert_and_rejects_unconfirmed_motor_intents(tmp_path):
    config=FlightConfig('fixture0',tmp_path/'wlm.json',tmp_path/'ble.json',
        capture_directory=tmp_path/'sessions',camera_preview=True,incident_directory=tmp_path/'incidents')
    app=FlightWeb(config,tmp_path/'owner.jsonl')
    try:
        for _ in range(3):
            s=app.status();assert s['state']=='disconnected'
            assert s['connection']['phase']=='idle' and not s['connection']['worker_alive']
        for action in ('takeoff','raw_duml'):
            with pytest.raises(IntentRejected):app.intent(action,{})
        with pytest.raises(IntentRejected):app.camera_command('recenter',{})
        assert not config.capture_directory.exists()
    finally:app.close()
    assert app.runtime.cleanup_verified


def test_real_owner_window_checks_do_not_add_status_rpcs_or_bypass_owner_validation(tmp_path,monkeypatch):
    config=FlightConfig('fixture0',tmp_path/'wlm.json',tmp_path/'ble.json',
        capture_directory=tmp_path/'sessions',camera_preview=True,incident_directory=tmp_path/'incidents')
    app=FlightWeb(config,tmp_path/'owner.jsonl')
    requests=[];original=app.runtime._request
    def traced(operation,*args,**kwargs):
        if operation!='heartbeat':requests.append(operation)
        return original(operation,*args,**kwargs)
    monkeypatch.setattr(app.runtime,'_request',traced)
    try:
        window=app.window()
        assert requests==[]
        with pytest.raises(IntentRejected):
            app.intent('takeoff',{},ingress=window_ingress(window))
        assert requests==['action']
        status=app.status()
        assert status['state']=='disconnected' and not status['connection']['worker_alive']
        assert not config.capture_directory.exists()
    finally:app.close()
    assert app.runtime.cleanup_verified


class RecoverableRuntime(Runtime):
    def __init__(self,*args,dead=True,**kwargs):
        super().__init__(*args,**kwargs)
        self.dead=dead;self.recovery_calls=0;self.generation=('b' if dead else 'c')*24

    def status(self):
        return {'state':'fault' if self.dead else 'disconnected','connected':False,
            'possible_airborne':True,'connection':{'generation':self.generation,'worker_alive':False},
            'controller_owner':{'epoch':self.epoch,'available':not self.dead,'restart_available':self.dead},
            'recovery':{'required':True,'incident_id':'private-fixture','peer_match':None,
                        'reason':'unresolved_previous_flight'}}

    def recover_owner(self,*,ingress=None):
        if ingress is not None:ingress.validate(time.monotonic())
        self.recovery_calls+=1
        if not self.dead:raise IntentRejected('controller_owner_still_running')
        return RecoverableRuntime(dead=False)


def test_explicit_http_owner_recovery_replaces_runtime_without_opening_or_reusing_frames(tmp_path):
    with server(tmp_path,runtime_factory=RecoverableRuntime) as (base,app):
        old=app.runtime
        target=app.frame_path();target.parent.mkdir(parents=True);target.write_bytes(b'old-owner-frame')
        for _ in range(2):
            assert request(base,'/api/flight/status')[0]==200
        assert not old.recovery_calls and not old.calls
        assert request(base,'/api/flight/intent',payload={'action':'recover_owner','parameters':{'open':True}})[0]==400
        assert not old.recovery_calls
        code,_,body=request(base,'/api/flight/intent',payload={'action':'recover_owner','parameters':{}})
        assert code==202
        result=json.loads(body)
        assert result['accepted'] and result['status']['recovery']['required']
        assert result['status']['state']=='disconnected' and result['status']['possible_airborne']
        assert app.runtime is not old and old.recovery_calls==1
        assert not old.closed and not old.calls and not app.runtime.calls
        assert request(base,'/api/flight/frame')[0]==404
        assert request(base,'/api/flight/intent',payload={'action':'recover_owner','parameters':{}})[0]==409
        assert request(base,'/api/flight/intent',payload={'action':'open','parameters':{}})[0]==202
        assert app.runtime.calls==[('open',{})] and not old.calls
        new=app.runtime
    assert new.closed and not old.closed


@pytest.mark.parametrize('headers',[{'Host':'foreign.invalid'},{'Origin':'https://foreign.invalid'},
    {'X-Neo2-Token':'invalid'}])
def test_recovery_preserves_authentication_host_and_origin_checks(tmp_path,headers):
    with server(tmp_path,runtime_factory=RecoverableRuntime) as (base,app):
        old=app.runtime
        assert request(base,'/api/flight/intent',payload={'action':'recover_owner','parameters':{}},headers=headers)[0] in (401,403)
        assert app.runtime is old and old.recovery_calls==0


@pytest.mark.parametrize('operation',['intent','camera','recover_owner'])
def test_requests_waiting_during_replacement_cannot_cross_to_new_runtime(tmp_path,operation):
    config=FlightConfig('fixture0',tmp_path/'wlm.json',tmp_path/'ble.json',camera_preview=True,
        incident_directory=tmp_path/'incidents')
    app=FlightWeb(config,tmp_path/'journal.jsonl',runtime_factory=RecoverableRuntime)
    queued=threading.Event();underlying=threading.RLock();errors=[]
    class Lock:
        def __enter__(self):
            if threading.current_thread().name=='queued-intent':queued.set()
            underlying.acquire();return self
        def __exit__(self,*args):underlying.release()
    app.lock=Lock();old=app.runtime
    def call():
        try:
            if operation=='camera':app.camera_command('recenter',{})
            else:app.intent('recover_owner' if operation=='recover_owner' else 'open',{})
        except Exception as error:errors.append(error)
    with app.lock:
        worker=threading.Thread(target=call,name='queued-intent');worker.start()
        assert queued.wait(1)
        app.intent('recover_owner',{})
    worker.join(1)
    try:
        assert not worker.is_alive() and len(errors)==1
        assert isinstance(errors[0],IntentRejected) and str(errors[0])=='controller_owner_changed'
        assert old.recovery_calls==1 and not app.runtime.calls and not old.calls
    finally:app.close()


def test_failed_owner_replacement_keeps_previous_runtime(tmp_path):
    config=FlightConfig('fixture0',tmp_path/'wlm.json',tmp_path/'ble.json',camera_preview=True,
        incident_directory=tmp_path/'incidents')
    app=FlightWeb(config,tmp_path/'journal.jsonl',runtime_factory=RecoverableRuntime);old=app.runtime
    def failed():raise RuntimeError('startup_failed')
    old.recover_owner=failed
    try:
        with pytest.raises(RuntimeError):app.intent('recover_owner',{})
        assert app.runtime is old and not old.closed and app.runtime_generation==0
    finally:app.close()


@pytest.mark.parametrize('endpoint,action',[('intent','open'),('intent','recover_owner'),('camera','recenter')])
def test_http_body_delayed_across_owner_replacement_cannot_target_new_owner(tmp_path,endpoint,action):
    config=FlightConfig('fixture0',tmp_path/'wlm.json',tmp_path/'ble.json',camera_preview=True,
        incident_directory=tmp_path/'incidents')
    app=FlightWeb(config,tmp_path/'journal.jsonl',runtime_factory=RecoverableRuntime)
    reading=threading.Event();resume=threading.Event();results=[]
    class Handler(build_flight_handler(app,TOKEN)):
        def do_POST(self):
            stream=self.rfile
            class Reader:
                def read(self,*args):
                    reading.set();assert resume.wait(3);return stream.read(*args)
                def __getattr__(self,name):return getattr(stream,name)
            self.rfile=Reader()
            super().do_POST()
    http=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    worker=threading.Thread(target=http.serve_forever,daemon=True);worker.start()
    call=threading.Thread(target=lambda:results.append(request(f'http://127.0.0.1:{http.server_port}',
        '/api/flight/'+endpoint,payload={'action':action,'parameters':{}})))
    try:
        call.start();assert reading.wait(2)
        old=app.runtime;app.intent('recover_owner',{});resume.set();call.join(2)
        assert not call.is_alive() and results[0][0]==409
        assert json.loads(results[0][2])['error']=='controller_owner_changed'
        assert old.recovery_calls==1 and not old.calls and not app.runtime.calls
    finally:
        resume.set();call.join(2);http.shutdown();http.server_close();worker.join(1);app.close()


def test_control_expiry_is_not_renewed_while_waiting_for_the_app_lock(tmp_path,monkeypatch):
    import neo2_control.flight_web as web
    clock=[100.];ready=threading.Event();underlying=threading.RLock()
    monkeypatch.setattr(web,'time',SimpleNamespace(monotonic=lambda:clock[0]))
    config=FlightConfig('fixture0',tmp_path/'wlm.json',tmp_path/'ble.json',camera_preview=True,
        incident_directory=tmp_path/'incidents')
    app=FlightWeb(config,tmp_path/'journal.jsonl',runtime_factory=Runtime)
    class Lock:
        def __enter__(self):
            if threading.current_thread().name=='queued-control':ready.set()
            underlying.acquire();return self
        def __exit__(self,*args):underlying.release()
    app.lock=Lock()
    with app.lock:
        worker=threading.Thread(target=lambda:app.intent('control',
            {'lease':'old','sequence':1,'channels':[0,0,0,0]}),name='queued-control')
        worker.start();assert ready.wait(1);clock[0]=101.
    worker.join(1)
    try:
        assert not worker.is_alive()
        assert app.runtime.calls[0][1]['expires_at']==100.2
    finally:app.close()


def test_http_recovery_requires_exited_real_inert_owner_and_creates_a_fresh_one(tmp_path):
    from neo2_control.control_owner import ProcessControllerRuntime
    with server(tmp_path,runtime_factory=ProcessControllerRuntime) as (base,app):
        old=app.runtime
        assert request(base,'/api/flight/intent',payload={'action':'recover_owner','parameters':{}})[0]==409
        assert app.runtime is old and old.process.poll() is None
        old.process.kill();old.process.wait(timeout=3)
        code,_,body=request(base,'/api/flight/status')
        assert code==200 and json.loads(body)['controller_owner']['restart_available']
        code,_,body=request(base,'/api/flight/intent',payload={'action':'recover_owner','parameters':{}})
        assert code==202
        new=app.runtime;result=json.loads(body)['status']
        assert new is not old and new.process.pid!=old.process.pid and new.epoch!=old.epoch
        assert new.journal_path!=old.journal_path and new.journal_path.exists()
        assert result['state']=='disconnected' and not result['connected']
        assert result['connection']['phase']=='idle' and not result['connection']['worker_alive']
        assert not old.thread.is_alive() and not app.config.capture_directory.exists()
    assert new.cleanup_verified
