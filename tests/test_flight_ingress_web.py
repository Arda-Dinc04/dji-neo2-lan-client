"""Real HTTP ingress windows and image age, without aircraft or radio access."""
from http.server import ThreadingHTTPServer
import json
import os
import threading
import time

import pytest

from neo2_control.controller import IntentRejected
from neo2_control.flight_web import build_flight_handler
from test_flight_web import server,raw_request,TOKEN,window_ingress


def window(base):
    code,_,body=raw_request(base,'/api/flight/window')
    assert code==200
    value=json.loads(body)
    return window_ingress(value)


@pytest.mark.parametrize('endpoint,action',[
    ('intent','open'),('intent','prepare'),('intent','takeoff'),
    ('intent','keepalive'),('intent','land'),('intent','confirm_ground'),
    ('intent','recover_owner'),('intent','close'),('camera','recenter')])
def test_every_http_mutation_requires_a_pc_window(tmp_path,endpoint,action):
    with server(tmp_path) as (base,app):
        code,_,_=raw_request(base,'/api/flight/'+endpoint,
            payload={'action':action,'parameters':{}})
        assert code==400 and not app.runtime.calls


def test_all_mutations_share_one_monotonic_sequence_even_on_business_refusal(tmp_path):
    with server(tmp_path) as (base,app):
        old=window(base);new=window(base)
        code,_,_=raw_request(base,'/api/flight/camera',
            payload={'action':'recenter','parameters':{},'ingress':new})
        assert code==202
        for ticket in (old,new):
            code,_,body=raw_request(base,'/api/flight/intent',
                payload={'action':'open','parameters':{},'ingress':ticket})
            assert code==409 and json.loads(body)['error']=='operator_request_replayed'
        rejected=window(base)
        for _ in range(2):
            code,_,body=raw_request(base,'/api/flight/intent',
                payload={'action':'rejected','parameters':{},'ingress':rejected})
            assert code==409
        assert json.loads(body)['error']=='operator_request_replayed'
        assert app.runtime.calls==[('recenter',{})]


def test_control_uses_original_issue_time_and_never_caller_or_arrival_time(tmp_path):
    with server(tmp_path) as (base,app):
        ticket=window(base)
        code,_,_=raw_request(base,'/api/flight/intent',payload={
            'action':'control','parameters':{'lease':'local','sequence':1,'channels':[0,0,0,0]},
            'ingress':ticket})
        assert code==202
        context=app.runtime.contexts[-1]
        assert app.runtime.calls[-1][1]['expires_at']==context.issued_at+.2
        assert context.expires_at==context.issued_at+.25


def test_expiry_during_app_lock_wait_rejects_without_business_call(tmp_path):
    with server(tmp_path) as (_,app):
        clock=[100.];app.windows.clock=lambda:clock[0]
        issued=app.window();ticket=window_ingress(issued)
        queued=threading.Event();original=threading.RLock();errors=[]
        class Lock:
            def __enter__(self):queued.set();original.acquire();return self
            def __exit__(self,*args):original.release()
        app.lock=Lock()
        def call():
            try:app.intent('open',{},ingress=ticket)
            except IntentRejected as error:errors.append(str(error))
        with original:
            worker=threading.Thread(target=call);worker.start();assert queued.wait(1)
            clock[0]=100.251
        worker.join(1)
        assert not worker.is_alive() and errors==['operator_request_expired']
        assert not app.runtime.calls


def test_slow_http_body_cannot_get_a_new_deadline_when_finally_read(tmp_path):
    with server(tmp_path) as (_,app):
        clock=[100.];app.windows.clock=lambda:clock[0]
        started=threading.Event();resume=threading.Event();results=[]
        class Handler(build_flight_handler(app,TOKEN)):
            def do_POST(self):
                stream=self.rfile
                class Reader:
                    def read(self,*args):
                        started.set();assert resume.wait(2);return stream.read(*args)
                    def __getattr__(self,name):return getattr(stream,name)
                self.rfile=Reader();super().do_POST()
        http=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
        base=f'http://127.0.0.1:{http.server_port}'
        ticket=window(base)
        request=threading.Thread(target=lambda:results.append(raw_request(base,'/api/flight/intent',
            payload={'action':'open','parameters':{},'ingress':ticket})))
        try:
            request.start();assert started.wait(1);clock[0]=100.251;resume.set();request.join(2)
            assert not request.is_alive() and results[0][0]==409
            assert json.loads(results[0][2])['error']=='operator_request_expired'
            assert not app.runtime.calls
        finally:
            resume.set();request.join(2);http.shutdown();http.server_close();thread.join(1)


def test_get_window_and_status_do_not_change_authority_and_old_generation_is_rejected(tmp_path):
    with server(tmp_path) as (base,app):
        tickets=[window(base) for _ in range(3)]
        assert raw_request(base,'/api/flight/status')[0]==200 and not app.runtime.calls
        app.runtime.status=lambda:{'connection':{'generation':None}}
        code,_,body=raw_request(base,'/api/flight/intent',
            payload={'action':'open','parameters':{},'ingress':tickets[-1]})
        assert code==409 and json.loads(body)['error']=='operator_request_generation_changed'
        assert not app.runtime.calls


def test_frame_age_and_identity_come_from_the_read_file_and_future_dates_are_refused(tmp_path):
    with server(tmp_path) as (base,app):
        target=app.frame_path();target.parent.mkdir(parents=True);target.write_bytes(b'jpeg-fixture')
        old=time.time()-20;os.utime(target,(old,old))
        code,headers,body=raw_request(base,'/api/flight/frame')
        assert code==200 and body==b'jpeg-fixture'
        assert float(headers['X-Neo2-Frame-Age-Seconds'])>=20
        assert headers['X-Neo2-Connection-Generation']=='b'*24
        code,again,_=raw_request(base,'/api/flight/frame')
        assert code==200 and again['X-Neo2-Frame-Id']==headers['X-Neo2-Frame-Id']
        assert float(again['X-Neo2-Frame-Age-Seconds'])>=float(headers['X-Neo2-Frame-Age-Seconds'])
        future=time.time()+60;os.utime(target,(future,future))
        assert raw_request(base,'/api/flight/frame')[0]==404


def test_frame_read_cannot_cross_a_connection_generation_change(tmp_path):
    with server(tmp_path) as (_,app):
        target=app.frame_path();target.parent.mkdir(parents=True);target.write_bytes(b'old-frame')
        calls=[]
        def status():
            calls.append(True)
            return {'connection':{'generation':('b' if len(calls)==1 else 'c')*24}}
        app.runtime.status=status
        with pytest.raises(FileNotFoundError):app.read_frame()
