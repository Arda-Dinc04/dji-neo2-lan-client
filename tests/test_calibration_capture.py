"""Capture real detected board images over HTTP, with an inert aircraft runtime.

The views are rendered fixtures, not evidence of physical camera calibration.
"""
import hashlib
import json
import time

import pytest

cv=pytest.importorskip('cv2')
np=pytest.importorskip('numpy')

from neo2_control.calibration import calibrate_charuco
from neo2_control.calibration_capture import capture_calibration
from neo2_control.camera import decode_camera_fov
from neo2_control.operator_client import FlightOperatorClient,OperatorError,PreviewFrame
from test_flight_web import server
from test_metric_reference import BOARD,MODEL,render_calibration_views


def stationary(now,epoch='a'*48):
    return {'state':'ready','connected':True,'possible_airborne':False,'synthetic':False,
        'connection':{'generation':'b'*24,'video':{'live':True}},'controller_owner':{'epoch':epoch},
        'telemetry':{'flying':False,'motors_on':False,'motion_received_at':now}}


@pytest.fixture(scope='module')
def images(tmp_path_factory):
    cv.setNumThreads(2)
    paths=render_calibration_views(tmp_path_factory.mktemp('capture-rendered-views'))
    return [cv.imencode('.jpg',cv.imread(str(path)),[cv.IMWRITE_JPEG_QUALITY,95])[1].tobytes()
        for path in paths]


class Client:
    pin_connection=FlightOperatorClient.pin_connection

    def __init__(self,images):
        self.images=images;self.now=10.;self.reads=0;self.status_reads=0;self._expected_context=None
        self.change=lambda s,n:s;self.age=0.;self.frame_generation='b'*24

    def status(self):
        self.status_reads+=1
        return self.change(stationary(self.now),self.status_reads)

    def read_frame(self):
        body=self.images[self.reads%len(self.images)];self.reads+=1
        return PreviewFrame(body,self.frame_generation,f'1:2:{self.reads:x}:{len(body):x}',
            self.age,self.now,self.now)

    def wait(self,seconds):self.now+=seconds


def capture(client,path,**options):
    return capture_calibration(client,BOARD,path,views=10,duration=5.,interval=1.,
        clock=lambda:client.now,wait=client.wait,**options)


def test_real_local_http_capture_produces_images_that_calibrate_without_any_intents(tmp_path,images):
    with server(tmp_path) as (base,app):
        app.runtime.status=lambda:with_fov(stationary(time.monotonic(),app.runtime.epoch))
        frames=iter(enumerate(images))
        def next_frame():
            i,body=next(frames)
            return body,{'X-Neo2-Connection-Generation':'b'*24,'X-Neo2-Frame-Age-Seconds':'0',
                'X-Neo2-Frame-Id':f'1:2:{i:x}:{len(body):x}'}
        app.read_frame=next_frame
        output=tmp_path/'capture';events=[]
        report=capture_calibration(FlightOperatorClient(base),BOARD,output,views=12,
            duration=20.,interval=.1,event_sink=events.append)
        assert not app.runtime.calls and not app.runtime.contexts
    assert report['target_reached'] and report['accepted_views']==12
    assert report['reason']=='target_views_reached'
    assert report['context']['fov_info']['sdk_fields_raw']['width']==6400
    assert not any(report[k] for k in ('optical_calibration_verified','physical_scale_verified',
        'acquisition_synchronization_verified'))
    assert json.loads((output/'capture.json').read_text())==report
    assert output.stat().st_mode&0o777==0o700
    for source,body in zip(report['sources'],images):
        path=output/source['file']
        assert path.read_bytes()==body and source['sha256']==hashlib.sha256(body).hexdigest()
        assert path.stat().st_mode&0o777==0o600
        assert source['stationary_before']['motors_on'] is False
        assert source['stationary_after']['flying'] is False
        assert len(source['corner_ids'])>=12
    model,fit=calibrate_charuco([output/s['file'] for s in report['sources']],BOARD)
    assert model.fx==pytest.approx(MODEL.fx,rel=.02)
    assert model.fy==pytest.approx(MODEL.fy,rel=.02)
    assert fit['accepted_views']==12 and fit['metric_scale_verified'] is False
    assert events[-1]['event']=='finished'


@pytest.mark.parametrize('problem',[
    'motors_on','flying','stale_motion','synthetic','video_missing','generation','owner',
])
@pytest.mark.parametrize('changed_status',[3,5])
def test_session_or_motion_change_between_gets_stops_before_saving_image(tmp_path,images,problem,changed_status):
    client=Client(images)
    def change(s,n):
        if n!=changed_status:return s
        if problem in ('motors_on','flying'):s['telemetry'][problem]=True
        if problem=='stale_motion':s['telemetry']['motion_received_at']-=1
        if problem=='synthetic':s['synthetic']=True
        if problem=='video_missing':s['connection']['video']['live']=False
        if problem=='generation':s['connection']['generation']='c'*24
        if problem=='owner':s['controller_owner']['epoch']='c'*48
        return s
    client.change=change
    report=capture(client,tmp_path/'capture')
    expected=0 if changed_status==3 else 1
    assert report['accepted_views']==expected and not report['target_reached'] and client.reads==expected+1
    assert report['reason'] in ('calibration_requires_live_stationary_session','operator_request_generation_changed')
    assert len(list((tmp_path/'capture').glob('*.jpg')))==expected


def test_disconnected_session_fails_before_creating_output(tmp_path,images):
    client=Client(images)
    client.change=lambda s,n:{**s,'connected':False}
    with pytest.raises(ValueError,match='requires_live_stationary_session'):
        capture(client,tmp_path/'capture')
    assert not (tmp_path/'capture').exists() and not client.reads


@pytest.mark.parametrize('problem,expected',[
    ('stale','encoded_preview_stale'),('duplicate','duplicate_frame'),
    ('similar','view_too_similar'),('missing','pattern_not_found'),
    ('unavailable','frame_unavailable'),
])
def test_unusable_or_repeated_views_do_not_fill_calibration_set(tmp_path,images,problem,expected):
    client=Client(images);events=[]
    if problem=='stale':client.age=1.
    if problem=='duplicate':client.images=[images[0]]
    if problem=='similar':
        image=cv.imdecode(np.frombuffer(images[0],np.uint8),cv.IMREAD_COLOR)
        client.images=[cv.imencode('.jpg',image,[cv.IMWRITE_JPEG_QUALITY,q])[1].tobytes()
            for q in (89,90,91,92,93)]
    if problem=='missing':
        client.images=[cv.imencode('.jpg',np.full((720,1280),180,np.uint8))[1].tobytes()]
    if problem=='unavailable':
        def unavailable():raise OperatorError('frame_unavailable')
        client.read_frame=unavailable
    report=capture(client,tmp_path/'capture',event_sink=events.append)
    journal=[json.loads(line) for line in (tmp_path/'capture'/'events.jsonl').read_text().splitlines()]
    assert report['reason']=='capture_deadline' and not report['target_reached']
    assert report['accepted_views']<=1 and any(e.get('reason')==expected for e in journal)


@pytest.mark.parametrize('problem,reason',[
    ('generation','calibration_frame_generation_changed'),
    ('invalid_image','calibration_capture_image_invalid'),
    ('mode_change','calibration_capture_mode_changed'),
])
def test_bad_frame_or_changed_camera_mode_aborts_and_retains_partial_evidence(tmp_path,images,problem,reason):
    client=Client(images)
    if problem=='generation':client.frame_generation='c'*24
    if problem=='invalid_image':client.images=[b'not-a-jpeg']
    if problem=='mode_change':
        other=cv.imdecode(np.frombuffer(images[1],np.uint8),cv.IMREAD_COLOR)
        client.images=[images[0],cv.imencode('.jpg',cv.resize(other,(640,360)))[1].tobytes()]
    report=capture(client,tmp_path/'capture')
    assert report['reason']==reason and not report['target_reached']
    assert report['accepted_views']==(1 if problem=='mode_change' else 0)


def test_cli_reports_disconnected_without_creating_capture_or_sending_intents(tmp_path,monkeypatch,capsys):
    from neo2_control.cli import main
    import neo2_control.operator_client as operator
    with server(tmp_path) as (base,app):
        actual=FlightOperatorClient
        monkeypatch.setattr(operator,'FlightOperatorClient',lambda url:actual(base))
        output=tmp_path/'capture'
        assert main(['capture-calibration','--board','examples/calibration/charuco-7x5.json',
            '--output',str(output)])==1
        assert not app.runtime.calls and not app.runtime.contexts
    assert 'calibration_requires_live_stationary_session' in capsys.readouterr().err
    assert not output.exists()


def with_fov(status):
    status['connection']['camera']={'fov_status':{
        **decode_camera_fov(bytes.fromhex('00190000190e000001000000ea0b0000')),
        'received_at':status['telemetry']['motion_received_at'],'fresh':True,'revision':1,
        'source':{'version':1,'sender':1,'receiver':2,'sequence':59783,
            'cmd_type':0,'cmd_set':2,'cmd_id':0x8a}}}
    return status


def test_raw_fov_is_retained_around_each_image_without_claiming_complete_optical_mode(tmp_path,images):
    client=Client(images);client.change=lambda s,n:with_fov(s)
    report=capture(client,tmp_path/'capture')
    assert report['accepted_views']>1 and report['reason']=='capture_deadline'
    original=report['context']['fov_info']
    assert original['sdk_fields_raw']['lensFocalLength']==3050
    assert not report['complete_optical_mode_verified']
    for record in report['sources']:
        for key in ('stationary_before','stationary_after'):
            retained=record[key]['fov_info']
            assert retained['payload_sha256']==original['payload_sha256']
            assert retained['source']==original['source']
            assert retained['received_at']==record[key]['checked_at']


@pytest.mark.parametrize('problem',[
    'changed','lost','stale','appeared','invalid','changed_back',
])
@pytest.mark.parametrize('changed_status',[4,5])
def test_fov_change_at_same_raster_stops_before_accepting_another_image(tmp_path,images,problem,changed_status):
    client=Client(images)
    def change(s,n):
        if problem=='appeared' and n<changed_status:return s
        s=with_fov(s)
        if n!=changed_status:return s
        fov=s['connection']['camera']['fov_status']
        if problem=='changed':
            fov.update(decode_camera_fov(bytes.fromhex('00190000190e000002000000ea0b0000')))
        if problem=='lost':s['connection']['camera']['fov_status']=None
        if problem=='stale':fov['received_at']-=2
        if problem=='invalid':fov['payload_sha256']='0'*64
        if problem=='changed_back':fov['revision']+=2
        return s
    client.change=change
    report=capture(client,tmp_path/'capture')
    assert report['accepted_views']==1 and not report['target_reached']
    assert report['reason']==('calibration_fov_metadata_unavailable' if problem=='stale' else
        'calibration_fov_metadata_invalid' if problem=='invalid' else 'calibration_fov_metadata_changed')
    assert len(list((tmp_path/'capture').glob('*.jpg')))==1
    assert not report['complete_optical_mode_verified']


def test_stale_initial_fov_fails_before_creating_capture(tmp_path,images):
    client=Client(images)
    def change(s,n):
        s=with_fov(s);s['connection']['camera']['fov_status']['fresh']=False
        return s
    client.change=change
    with pytest.raises(ValueError,match='calibration_fov_metadata_unavailable'):
        capture(client,tmp_path/'capture')
    assert not (tmp_path/'capture').exists() and client.reads==0
