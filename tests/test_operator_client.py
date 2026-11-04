import pytest

from neo2_control.operator_client import FlightOperatorClient,OperatorError
from test_flight_web import server


def test_real_http_client_gets_one_pc_window_for_each_mutation(tmp_path):
    with server(tmp_path) as (base,app):
        client=FlightOperatorClient(base)
        assert client.status()['state']=='disconnected' and not app.runtime.calls
        client.intent('open');client.camera('recenter')
        assert app.runtime.calls==[('open',{}),('recenter',{})]
        assert [c.mutation_sequence for c in app.runtime.contexts]==[1,2]


def test_rejected_or_uncertain_mutations_are_never_retried(tmp_path):
    with server(tmp_path) as (base,app):
        client=FlightOperatorClient(base)
        with pytest.raises(OperatorError,match='invalid_state'):client.intent('rejected')
        assert len(app.runtime.contexts)==1
        original=client._json;posts=[]
        def response(path,payload=None):
            if payload is not None:posts.append(payload);raise OperatorError('operator_request_uncertain')
            return original(path)
        client._json=response
        with pytest.raises(OperatorError,match='uncertain'):client.intent('land')
        assert len(posts)==1 and not app.runtime.calls


@pytest.mark.parametrize('action,parameters',[('prepare',{'operation':'ground'}),('land',{})])
def test_mission_pin_refuses_new_generation_window_before_any_post(tmp_path,action,parameters):
    with server(tmp_path) as (base,app):
        client=FlightOperatorClient(base);client.pin_connection(client.status())
        app.runtime.status=lambda:{'state':'disconnected','connection':{'generation':'c'*24},
            'controller_owner':{'epoch':app.runtime.epoch}}
        with pytest.raises(OperatorError,match='generation_changed'):client.intent(action,parameters)
        assert not app.runtime.calls


@pytest.mark.parametrize('url',['https://remote.invalid','http://127.0.0.1/other',
    'http://username:password@127.0.0.1','http://127.0.0.1?secret=1'])
def test_local_monotonic_client_does_not_export_credentials_or_clock_to_another_origin(url):
    with pytest.raises(ValueError,match='loopback'):FlightOperatorClient(url)


def test_preview_uses_larger_binary_limit_and_requires_explicit_connection_pin(tmp_path):
    with server(tmp_path) as (base,app):
        target=app.frame_path();target.parent.mkdir(parents=True)
        body=b'\xff\xd8'+b'x'*(300*1024)+b'\xff\xd9'
        target.write_bytes(body)
        client=FlightOperatorClient(base)
        with pytest.raises(OperatorError,match='requires_pinned_connection'):client.read_frame()
        client.pin_connection(client.status())
        frame=client.read_frame()
        assert frame.data==body and frame.connection_generation=='b'*24
        assert int(frame.frame_id.rsplit(':',1)[1],16)==len(body)
        assert frame.encoded_age_seconds>=0 and frame.read_finished_at>=frame.read_started_at
        assert not app.runtime.calls and not app.runtime.contexts
        target.write_bytes(b'\xff\xd8'+b'x'*FlightOperatorClient.MAX_FRAME_BYTES+b'\xff\xd9')
        with pytest.raises(OperatorError,match='frame_too_large'):client.read_frame()


@pytest.mark.parametrize('change,reason',[
    ({'X-Neo2-Connection-Generation':'c'*24},'generation_changed'),
    ({'X-Neo2-Frame-Age-Seconds':'nan'},'metadata_invalid'),
    ({'X-Neo2-Frame-Age-Seconds':'-1'},'metadata_invalid'),
    ({'X-Neo2-Frame-Age-Seconds':'invalid'},'metadata_invalid'),
    ({'X-Neo2-Frame-Id':'1:2:3:1'},'metadata_invalid'),
    ({'X-Neo2-Frame-Id':'unstructured'},'metadata_invalid'),
])
def test_preview_rejects_foreign_or_invalid_frame_metadata_without_mutations(tmp_path,change,reason):
    with server(tmp_path) as (base,app):
        body=b'\xff\xd8fixture\xff\xd9'
        headers={'X-Neo2-Connection-Generation':'b'*24,'X-Neo2-Frame-Age-Seconds':'0.1',
            'X-Neo2-Frame-Id':f'1:2:3:{len(body):x}',**change}
        app.read_frame=lambda:(body,headers)
        client=FlightOperatorClient(base);client.pin_connection(client.status())
        with pytest.raises(OperatorError,match=reason):client.read_frame()
        assert not app.runtime.calls and not app.runtime.contexts


def test_inherited_preview_never_uses_cloud_transport_or_credentials():
    from neo2_control.cloud_operator_client import CloudFlightOperatorClient
    client=object.__new__(CloudFlightOperatorClient)
    with pytest.raises(OperatorError,match='requires_local_client'):client.read_frame()
