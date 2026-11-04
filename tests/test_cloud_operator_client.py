"""Real TLS and HTTP/IPC, authenticating a fixture bearer before synthetic PC."""
from contextlib import contextmanager
import http.client
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import shutil
import socket
import ssl
import subprocess
import threading
import time
from urllib.parse import urlsplit

import pytest

from neo2_control.cloud_operator_client import CloudFlightOperatorClient,read_bearer_token
from neo2_control.control_owner import ProcessControllerRuntime
from neo2_control.operator_client import OperatorError
from test_flight_web import server,TOKEN

BEARER='fixture.header.signature'
PREFIX='/home-server/api/drone/control/'


@pytest.fixture(scope='module')
def certificate(tmp_path_factory):
    if shutil.which('openssl') is None:pytest.skip('TLS fixture needs openssl')
    root=tmp_path_factory.mktemp('cloud-client-tls');key=root/'key.pem';cert=root/'cert.pem'
    subprocess.run(['openssl','req','-x509','-newkey','rsa:2048','-sha256','-nodes','-days','1',
        '-keyout',str(key),'-out',str(cert),'-subj','/CN=localhost',
        '-addext','subjectAltName=DNS:localhost,IP:127.0.0.1'],
        check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    return cert,key


class Bridge:
    def __init__(self,upstream):
        self.upstream=urlsplit(upstream);self.requests=[];self.delay=0.
        self.get_mode=None;self.post_mode=None;self.redirect=None
        self.block_action=None;self.blocked=threading.Event();self.release=threading.Event()
        self.block_window=False
        self.last_status=None;self.frozen=None
        self.transform=None

    def reply(self,method,path,headers,body):
        self.requests.append({'method':method,'path':path,'authorization':headers.get('Authorization'),
            'pc_token':headers.get('X-Neo2-Token'),'body':None if body is None else json.loads(body)})
        if headers.get('Authorization')!='Bearer '+BEARER:return 401,b'{"error":"Unauthorized"}'
        if method=='GET' and self.get_mode:
            if self.get_mode=='freeze':return 200,self.frozen
            if self.get_mode=='redirect':return 307,b''
            if self.get_mode=='pc401':return 401,b'{"error":"unauthorized"}'
            if self.get_mode=='duplicate':return 200,b'{"now":1,"now":2}'
            if self.get_mode=='nonfinite':return 200,b'{"now":NaN}'
            if self.get_mode=='oversize':return 200,b' '*262145
        assert path.startswith(PREFIX)
        time.sleep(self.delay)
        connection=http.client.HTTPConnection(self.upstream.hostname,self.upstream.port,timeout=3)
        try:
            connection.request(method,'/api/remote-flight/'+path[len(PREFIX):],body=body,
                headers={'X-Neo2-Token':TOKEN,**({} if body is None else {'Content-Type':'application/json'})})
            response=connection.getresponse();code=response.status;data=response.read(262145)
        finally:connection.close()
        if self.transform is not None:data=json.dumps(self.transform(json.loads(data))).encode()
        if method=='GET' and path.endswith('/status'):self.last_status=data
        if method=='GET' and path.endswith('/window') and self.block_window:
            self.block_window=False;self.blocked.set();assert self.release.wait(5)
        if method=='POST':
            value=json.loads(body)
            if self.block_action==value['action']:
                self.blocked.set();assert self.release.wait(5)
            if self.post_mode=='drop':return None,None
        time.sleep(self.delay)
        return code,data


@contextmanager
def gateway(tmp_path,certificate,*,runtime_factory=None):
    def synthetic(path,**_):return ProcessControllerRuntime(path)
    with server(tmp_path,runtime_factory=runtime_factory or synthetic) as (upstream,app):
        bridge=Bridge(upstream)
        class Handler(BaseHTTPRequestHandler):
            def handle_request(self):
                size=int(self.headers.get('Content-Length','0'))
                body=self.rfile.read(size) if self.command=='POST' else None
                code,data=bridge.reply(self.command,self.path,self.headers,body)
                if code is None:
                    self.close_connection=True;self.connection.shutdown(socket.SHUT_RDWR);return
                self.send_response(code)
                self.send_header('Content-Type','application/json')
                if code==307:self.send_header('Location',bridge.redirect)
                self.send_header('Content-Length',str(len(data)));self.end_headers()
                try:self.wfile.write(data)
                except (BrokenPipeError,ssl.SSLError):pass
            do_GET=do_POST=handle_request
            def log_message(self,*_):pass
        http=ThreadingHTTPServer(('127.0.0.1',0),Handler);http.daemon_threads=True
        context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER);context.load_cert_chain(*map(str,certificate))
        http.socket=context.wrap_socket(http.socket,server_side=True)
        worker=threading.Thread(target=http.serve_forever,daemon=True);worker.start()
        trust=ssl.create_default_context(cafile=str(certificate[0]))
        url='https://localhost:'+str(http.server_port)+PREFIX[:-1]
        try:yield url,trust,bridge,app
        finally:
            bridge.release.set();http.shutdown();http.server_close();worker.join(2)


def make(url,trust):return CloudFlightOperatorClient(url,BEARER,ssl_context=trust)


def test_real_https_reads_only_api_and_does_not_send_pc_secret(tmp_path,certificate):
    with gateway(tmp_path,certificate) as (url,trust,bridge,app):
        client=make(url,trust)
        assert app.runtime.status()['state']=='disconnected'
        assert [r['path'] for r in bridge.requests]==[PREFIX+'status']
        assert all(r['pc_token'] is None and r['authorization']=='Bearer '+BEARER for r in bridge.requests)
        client.intent('open');client.intent('prepare',{'operation':'arm'})
        assert sum(r['path'].endswith('/window') for r in bridge.requests)==1
        assert app.runtime.status()['state']=='ready'


def test_full_program_crosses_tls_and_real_owner_with_measured_network_delay(tmp_path,certificate):
    with gateway(tmp_path,certificate) as (url,trust,bridge,app):
        bridge.delay=.13
        client=make(url,trust)
        client.intent('open')
        token=client.intent('prepare',{'operation':'arm'})['confirmation_token']
        lease=client.intent('arm',{'confirmation_token':token})['lease']
        token=client.intent('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
        client.intent('takeoff',{'confirmation_token':token,'lease':lease})
        for _ in range(3):
            status=client.intent('keepalive',{'lease':lease})['status']
            if status['state']=='flying':break
        assert status['state']=='flying' and status['_client_round_trip_seconds']>=.26
        result=client.intent('program_start',{'lease':lease,'program':{'version':1,'finish':'land','steps':[
            {'kind':'translate','seconds':1,'right':50,'forward':0,'up':0},
            {'kind':'hold','seconds':.2}]}})
        status=result['status']
        for _ in range(10):
            report=status['remote_control']['program']
            if report['phase']!='running':break
            try:status=client.intent('program_keepalive',{'lease':lease,'program_id':result['program_id']})['status']
            except OperatorError as error:
                assert str(error) in ('operator_lease_invalid','motion_program_changed')
                status=client.status()
        assert report['phase']=='motion_complete'
        for _ in range(5):
            status=client.status()
            if status['landing_confirmed']:break
        assert status['landing_confirmed'] and status['possible_airborne'] is False
        assert all(r['pc_token'] is None for r in bridge.requests)


def test_post_response_loss_is_never_retried(tmp_path,certificate):
    with gateway(tmp_path,certificate) as (url,trust,bridge,app):
        client=make(url,trust);bridge.post_mode='drop'
        with pytest.raises(OperatorError,match='uncertain'):client.intent('open')
        assert len([r for r in bridge.requests if r['method']=='POST'])==1
        assert app.runtime.status()['state']=='ready'
        assert client._next_window is None


def test_tls_certificate_and_bearer_are_verified(tmp_path,certificate):
    with gateway(tmp_path,certificate) as (url,trust,bridge,app):
        with pytest.raises(OperatorError):CloudFlightOperatorClient(url,BEARER)
        assert not bridge.requests
        with pytest.raises(OperatorError,match='cloud_authentication_failed'):
            CloudFlightOperatorClient(url,'invalid',ssl_context=trust)
        assert app.runtime.status()['state']=='disconnected'


@pytest.mark.parametrize('mode,reason',[('pc401','controller_owner_unavailable'),
    ('duplicate','operator_response_invalid'),('nonfinite','operator_response_invalid'),
    ('oversize','operator_response_too_large')])
def test_invalid_responses_never_become_authority(tmp_path,certificate,mode,reason):
    with gateway(tmp_path,certificate) as (url,trust,bridge,app):
        client=make(url,trust);bridge.get_mode=mode
        with pytest.raises(OperatorError,match=reason):client.status()
        assert not any(r['method']=='POST' for r in bridge.requests)


def test_redirect_and_proxy_cannot_export_bearer(tmp_path,certificate,monkeypatch):
    hits=[]
    class Collector(BaseHTTPRequestHandler):
        def do_GET(self):hits.append(self.path);self.send_response(500);self.end_headers()
        def log_message(self,*_):pass
    collector=ThreadingHTTPServer(('127.0.0.1',0),Collector)
    worker=threading.Thread(target=collector.serve_forever,daemon=True);worker.start()
    try:
        target='http://127.0.0.1:'+str(collector.server_port)+'/steal'
        monkeypatch.setenv('HTTPS_PROXY',target);monkeypatch.setenv('NO_PROXY','')
        with gateway(tmp_path,certificate) as (url,trust,bridge,app):
            client=make(url,trust)
            bridge.get_mode='redirect';bridge.redirect=target
            with pytest.raises(OperatorError,match='cloud_redirect_refused'):client.status()
        assert not hits
    finally:collector.shutdown();collector.server_close();worker.join(2)


@pytest.mark.parametrize('url',['http://remote.invalid/api/control','https://user:password@remote.invalid/api/control',
    'https://remote.invalid/api/control?token=secret','https://remote.invalid/api/control#fragment',
    'https://@remote.invalid/api/control','https://remote.invalid/api/control?',
    'https://remote.invalid/api/../control','https://remote.invalid/api/control\r\nInjected:x',
    'https://remote.invalid:70000/api/control','https://remote.invalid/'])
def test_cloud_endpoint_validation_precedes_network(url):
    with pytest.raises(ValueError,match='cloud_api_url_invalid'):CloudFlightOperatorClient(url,BEARER)


@pytest.mark.parametrize('token',['','secret\r\nX-Other: value','é','a'*8186])
def test_invalid_bearer_never_enters_a_request(token):
    with pytest.raises(ValueError,match='cloud_bearer_token_invalid'):
        CloudFlightOperatorClient('https://server.invalid/api/control',token)


def test_private_token_file_and_no_symlink(tmp_path):
    path=tmp_path/'token';path.write_text(BEARER+'\n');path.chmod(0o600)
    assert read_bearer_token(path)==BEARER
    path.chmod(0o644)
    with pytest.raises(ValueError,match='not_private'):read_bearer_token(path)
    path.chmod(0o600);alias=tmp_path/'alias';alias.symlink_to(path)
    with pytest.raises(ValueError,match='unavailable'):read_bearer_token(alias)


@pytest.mark.parametrize('body',[b'',b'a\nb',bytes([255]),b'a'*8193])
def test_bad_token_files_are_rejected_without_returning_their_contents(tmp_path,body):
    path=tmp_path/'token';path.write_bytes(body);path.chmod(0o600)
    with pytest.raises(ValueError):read_bearer_token(path)


def airborne(client):
    client.intent('open')
    token=client.intent('prepare',{'operation':'arm'})['confirmation_token']
    lease=client.intent('arm',{'confirmation_token':token})['lease']
    token=client.intent('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
    status=client.intent('takeoff',{'confirmation_token':token,'lease':lease})['status']
    for _ in range(30):
        if status['state']=='flying':return lease
        status=client.intent('keepalive',{'lease':lease})['status'];time.sleep(.03)
    raise AssertionError('synthetic takeoff did not complete')


@pytest.mark.parametrize('block',['window','control','camera','program_keepalive'])
def test_land_overtakes_request_and_old_reply_cannot_restore_cached_authority(tmp_path,certificate,block):
    with gateway(tmp_path,certificate) as (url,trust,bridge,app):
        client=make(url,trust);lease=airborne(client);errors=[]
        program_id=None
        if block=='program_keepalive':
            program_id=client.intent('program_start',{'lease':lease,'program':{
                'version':1,'finish':'land','steps':[{'kind':'hold','seconds':3}]}})['program_id']
        if block=='window':client._next_window=None;bridge.block_window=True
        else:bridge.block_action='photo' if block=='camera' else block
        def old_control():
            try:
                if block=='camera':client.camera('photo')
                elif block=='program_keepalive':client.intent('program_keepalive',{'lease':lease,'program_id':program_id})
                else:client.intent('control',{'lease':lease,'sequence':0,'channels':[0,0,0,100]})
            except OperatorError as error:errors.append(str(error))
        worker=threading.Thread(target=old_control);worker.start()
        assert bridge.blocked.wait(2)
        result=client.intent('land')
        assert result['status']['state']=='landing'
        saved=client._next_window
        bridge.release.set();worker.join(3)
        assert not worker.is_alive() and errors==['operator_mutation_superseded']
        assert client._next_window==saved
        posts=[r['body']['action'] for r in bridge.requests if r['method']=='POST']
        assert posts.count('land')==1
        assert posts.count('control')==(1 if block=='control' else 0)


def test_read_only_polling_never_renews_a_loaded_program(tmp_path,certificate):
    with gateway(tmp_path,certificate) as (url,trust,bridge,app):
        client=make(url,trust);lease=airborne(client)
        result=client.intent('program_start',{'lease':lease,'program':{
            'version':1,'finish':'land','steps':[{'kind':'hold','seconds':3}]}})
        expiry=result['status']['remote_control']['expires_at']
        while time.monotonic()<expiry+.1:client.status();time.sleep(.03)
        report=client.status()['remote_control']['program']
        assert report['phase']=='cancelled'
        assert [r['body']['action'] for r in bridge.requests if r['method']=='POST'].count('program_keepalive')==0


def test_insecure_tls_context_is_refused_before_network():
    context=ssl._create_unverified_context()
    with pytest.raises(ValueError,match='tls_verification_required'):
        CloudFlightOperatorClient('https://server.invalid/api/control',BEARER,ssl_context=context)


def test_repeated_pc_snapshot_keeps_original_client_receipt(tmp_path,certificate):
    with gateway(tmp_path,certificate) as (url,trust,bridge,app):
        client=make(url,trust);first=client.status()
        bridge.get_mode='freeze';bridge.frozen=bridge.last_status
        time.sleep(.05)
        repeated=client.status()
        assert repeated['now']==first['now']
        assert repeated['_client_received_at']==first['_client_received_at']
        old=json.loads(bridge.frozen);old['now']-=1
        bridge.frozen=json.dumps(old).encode()
        old=client.status()
        assert old['now']==first['now'] and old['_client_observation_reused'] is True
        assert old['_client_received_at']==first['_client_received_at']


def test_parallel_observation_does_not_turn_a_valid_command_receipt_into_failure(tmp_path,certificate):
    with gateway(tmp_path,certificate) as (url,trust,bridge,app):
        client=make(url,trust);lease=airborne(client);bridge.block_action='control'
        results=[];errors=[]
        def command():
            try:results.append(client.intent('control',{'lease':lease,'sequence':0,'channels':[0,0,0,50]}))
            except Exception as error:errors.append(str(error))
        worker=threading.Thread(target=command);worker.start();assert bridge.blocked.wait(2)
        newer=client.status()
        bridge.release.set();worker.join(3)
        assert not worker.is_alive() and not errors
        assert results[0]['status']['now']==newer['now']
        assert results[0]['status']['_client_received_at']==newer['_client_received_at']
        assert results[0]['status']['_client_observation_reused'] is True
        client.intent('land')
