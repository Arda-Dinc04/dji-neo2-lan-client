"""Read-only HTTP pose acquisition and fail-closed source/time transitions.

Rendered images and a controlled receipt clock are fixtures, not hardware
exposure timing, physical calibration or evidence of actual body navigation.
"""
from dataclasses import replace
import hashlib
import json

import pytest

cv=pytest.importorskip('cv2')
np=pytest.importorskip('numpy')

from neo2_control.metric_reference import anchor_scan
from neo2_control.operator_client import FlightOperatorClient,PreviewFrame
from neo2_control.preview_tracking import PreviewPoseTracker,track_preview
from neo2_control.reconstruction import reconstruct_pair
from room_scene import RoomScene
from test_calibration_capture import stationary
from test_flight_web import server
from test_metric_reference import MODEL,detect


@pytest.fixture(scope='module')
def scene(tmp_path_factory):
    cv.setNumThreads(2);cv.setRNGSeed(31)
    folder=tmp_path_factory.mktemp('preview-tracking')
    room=RoomScene(folder);paths=room.pair(0)
    seed=anchor_scan(reconstruct_pair(*paths,MODEL),*[detect(p) for p in paths])
    images=[]
    for i,d in enumerate([0.,.05,.1,.2]):
        path=folder/f'query-{i}.png';room.render(path,0,room.center+np.array([d,0.,0.]))
        images.append(cv.imencode('.jpg',cv.imread(str(path)),[cv.IMWRITE_JPEG_QUALITY,95])[1].tobytes())
    blank=cv.imencode('.jpg',np.full((MODEL.height,MODEL.width),160,np.uint8))[1].tobytes()
    return seed,images,blank


class Client:
    pin_connection=FlightOperatorClient.pin_connection

    def __init__(self,images):
        self.images=images;self.now=10.;self._expected_context=None
        self.reads=0;self.status_reads=0;self.age=0.;self.generation='b'*24
        self.change=lambda s,n:s;self.identity=None

    def status(self):
        self.status_reads+=1
        return self.change(stationary(self.now),self.status_reads)

    def read_frame(self):
        body=self.images[self.reads%len(self.images)];self.reads+=1
        return PreviewFrame(body,self.generation,self.identity or f'1:2:{self.reads:x}:{len(body):x}',
            self.age,self.now,self.now)

    def wait(self,seconds):self.now+=seconds


def test_actual_http_receives_camera_motion_and_saves_source_bound_frames_without_commands(tmp_path,scene):
    seed,images,_=scene;clock=Client(images);observations=[]
    with server(tmp_path) as (base,app):
        app.runtime.status=lambda:stationary(clock.now,app.runtime.epoch)
        frames=iter(enumerate(images))
        def next_frame():
            i,body=next(frames)
            return body,{'X-Neo2-Connection-Generation':'b'*24,'X-Neo2-Frame-Age-Seconds':'0',
                'X-Neo2-Frame-Id':f'1:2:{i:x}:{len(body):x}'}
        app.read_frame=next_frame
        client=FlightOperatorClient(base);real_read=client.read_frame;requests=[];transport=client._read_response
        def trace(path,payload=None,**kw):
            requests.append((path,payload));return transport(path,payload,**kw)
        client._read_response=trace
        # Keep real HTTP/header/body validation; rebase only receipt times to
        # the fixture clock so slow CI scheduling cannot fake exposure expiry.
        client.read_frame=lambda:replace(real_read(),read_started_at=clock.now,read_finished_at=clock.now)
        result=track_preview(client,seed,tmp_path/'capture',duration=5.,observations=4,interval=.1,
            clock=lambda:clock.now,wait=clock.wait,event_sink=observations.append)
        assert not app.runtime.calls and not app.runtime.contexts
        assert all(path in ('/api/flight/status','/api/flight/frame') and payload is None
            for path,payload in requests)
    assert result['localized_count']==4 and result['lost_count']==result['waiting_count']==0
    assert result['reason']=='observation_limit'
    assert result['samples'][-1]['distance_from_first']==pytest.approx(.2,abs=.02)
    assert result['distance_unit']=='m' and result['flight_authorized'] is False
    for record,body in zip(result['samples'],images):
        image=tmp_path/'capture'/record['file']
        assert image.read_bytes()==body and image.stat().st_mode&0o777==0o600
        assert record['source']['image_sha256']==hashlib.sha256(body).hexdigest()
        assert record['stationary_after_localization']['motors_on'] is False
    assert (tmp_path/'capture').stat().st_mode&0o777==0o700
    assert json.loads((tmp_path/'capture'/'tracking.json').read_text())==result
    assert observations[-1]['event']=='finished'
    assert not any(result[k] for k in ('physical_accuracy_verified','acquisition_synchronization_verified',
        'optical_mode_verified','current_body_pose_verified'))


@pytest.mark.parametrize('problem',['owner','generation','motors_on','flying','disconnect','synthetic','stale_motion'])
def test_source_or_motion_change_during_localization_invalidates_current_pose_and_stops(tmp_path,scene,problem):
    client=Client(scene[1])
    def change(s,n):
        if n!=4:return s
        if problem=='owner':s['controller_owner']['epoch']='c'*48
        if problem=='generation':s['connection']['generation']='c'*24
        if problem in ('motors_on','flying'):s['telemetry'][problem]=True
        if problem=='disconnect':s['connected']=False
        if problem=='synthetic':s['synthetic']=True
        if problem=='stale_motion':s['telemetry']['motion_received_at']-=1
        return s
    client.change=change
    tracker=PreviewPoseTracker(client,scene[0],clock=lambda:client.now)
    with pytest.raises((ValueError,RuntimeError)):tracker.sample()
    assert tracker.current_pose is None and tracker.localizer._previous is None
    with pytest.raises(ValueError,match='tracking_stopped'):tracker.sample()


def test_expiry_during_computation_discards_pose_and_clears_flow(scene):
    client=Client(scene[1]);tracker=PreviewPoseTracker(client,scene[0],clock=lambda:client.now)
    solve=tracker.localizer.localize_bytes
    def delayed(raw):
        result=solve(raw);client.now+=1.;return result
    tracker.localizer.localize_bytes=delayed
    observation=tracker.sample()
    assert observation.record['state']=='expired'
    assert observation.record['reason']=='preview_expired_during_localization'
    assert tracker.current_pose is None and tracker.localizer._previous is None


def test_repeated_poll_never_reuses_pose_as_current_but_fresh_identical_image_can_be_observed(scene):
    client=Client([scene[1][0]]);tracker=PreviewPoseTracker(client,scene[0],clock=lambda:client.now)
    first=tracker.sample();client.identity=first.record['source']['frame_id']
    repeated=tracker.sample()
    assert repeated.record['state']=='waiting' and repeated.record['reason']=='duplicate_frame'
    assert tracker.current_pose is None and repeated.image is None
    client.identity=None;client.now+=.1
    fresh=tracker.sample()
    assert fresh.record['state']=='localized' and fresh.record['source']['same_encoded_image_as_previous']
    assert fresh.record['distance_from_first']<.01


def test_blank_view_has_no_pose_and_recovery_uses_saved_map_instead_of_old_position(scene):
    client=Client([scene[1][0],scene[2],scene[1][-1]])
    tracker=PreviewPoseTracker(client,scene[0],clock=lambda:client.now)
    assert tracker.sample().record['state']=='localized'
    missing=tracker.sample()
    assert missing.record['state']=='lost' and missing.record['pose'] is None
    assert tracker.current_pose is None
    recovered=tracker.sample()
    assert recovered.record['pose']['tracking_method']=='descriptors'
    assert recovered.record['distance_from_first']==pytest.approx(.2,abs=.02)


def test_baseline_map_never_reports_camera_distance_in_metres(scene):
    client=Client([scene[1][0],scene[1][-1]])
    tracker=PreviewPoseTracker(client,scene[0]['scan'],clock=lambda:client.now)
    tracker.sample();sample=tracker.sample().record
    assert sample['distance_unit']==sample['pose']['distance_unit']=='baseline'
    assert sample['distance_from_first']==pytest.approx(.2/scene[0]['metres_per_baseline'],abs=.15)


def test_disconnected_capture_fails_before_creating_directory(tmp_path,scene):
    client=Client(scene[1]);client.change=lambda s,n:{**s,'connected':False}
    with pytest.raises(ValueError,match='preview_requires_live_stationary_session'):
        track_preview(client,scene[0],tmp_path/'never',clock=lambda:client.now)
    assert not (tmp_path/'never').exists() and client.reads==0


def test_clock_failure_preserves_partial_report_and_does_not_emit_last_pose_as_live(tmp_path,scene):
    client=Client(scene[1])
    def invalid_time(seconds):client.now=float('nan')
    result=track_preview(client,scene[0],tmp_path/'partial',duration=5.,observations=3,
        clock=lambda:client.now,wait=invalid_time)
    assert result['localized_count']==1 and result['reason']=='preview_tracking_clock_invalid'
    assert json.loads((tmp_path/'partial'/'tracking.json').read_text())==result
    assert result['flight_authorized'] is False


def test_changed_mode_stops_without_saving_the_incompatible_frame(tmp_path,scene):
    client=Client([cv.imencode('.jpg',np.full((120,160),180,np.uint8))[1].tobytes()])
    result=track_preview(client,scene[0],tmp_path/'changed',clock=lambda:client.now,wait=client.wait)
    assert result['reason']=='preview_image_or_mode_invalid' and not result['samples']
    assert not list((tmp_path/'changed').glob('*.jpg'))


def test_repeated_file_that_has_expired_cannot_keep_old_flow_alive(scene):
    client=Client([scene[1][0]]);tracker=PreviewPoseTracker(client,scene[0],clock=lambda:client.now)
    first=tracker.sample();client.identity=first.record['source']['frame_id'];client.age=1.
    expired=tracker.sample()
    assert expired.record['state']=='expired' and tracker.current_pose is None
    assert tracker.localizer._previous is None
    client.identity=None;client.age=0.;client.now+=.1
    assert tracker.sample().record['pose']['tracking_method']=='descriptors'


def test_long_receipt_gap_requires_relocalization_even_when_new_file_is_fresh(scene):
    client=Client([scene[1][0]]);tracker=PreviewPoseTracker(client,scene[0],clock=lambda:client.now)
    tracker.sample();client.now+=1.
    next_observation=tracker.sample()
    assert next_observation.record['pose']['tracking_method']=='descriptors'


def test_pose_expires_while_a_consumer_is_not_polling(scene):
    client=Client([scene[1][0]]);tracker=PreviewPoseTracker(client,scene[0],clock=lambda:client.now)
    tracker.sample();assert tracker.current_pose is not None
    client.now+=1.
    assert tracker.current_pose is None and tracker.localizer._previous is None
    assert tracker.sample().record['pose']['tracking_method']=='descriptors'


def test_late_session_change_keeps_only_previously_completed_records(tmp_path,scene):
    client=Client(scene[1])
    def change(s,n):
        if n==7:s['controller_owner']['epoch']='c'*48
        return s
    client.change=change
    result=track_preview(client,scene[0],tmp_path/'partial-session',duration=5.,observations=3,
        clock=lambda:client.now,wait=client.wait)
    assert result['reason']=='operator_request_generation_changed'
    assert result['localized_count']==1 and len(result['samples'])==1
    assert len(list((tmp_path/'partial-session').glob('*.jpg')))==1


@pytest.mark.parametrize('loss',[False,True])
def test_cli_keeps_partial_measurement_distinct_from_complete_observations(tmp_path,scene,monkeypatch,capsys,loss):
    from neo2_control.cli import main
    import neo2_control.operator_client as operator
    import neo2_control.preview_tracking as preview
    images=[scene[1][0],scene[2] if loss else scene[1][-1]]
    client=Client(images);run=preview.track_preview
    monkeypatch.setattr(operator,'FlightOperatorClient',lambda url:client)
    monkeypatch.setattr(preview,'track_preview',lambda c,s,o,**kw:run(c,s,o,
        clock=lambda:client.now,wait=client.wait,**kw))
    path=tmp_path/'map.json';path.write_text(json.dumps(scene[0]))
    output=tmp_path/'capture'
    result=main(['track-preview',str(path),'--output',str(output),
        '--duration','5','--observations','2','--interval','.1'])
    captured=capsys.readouterr();report=json.loads(captured.out)
    assert result==(2 if loss else 0)
    assert report['localized_count']==(1 if loss else 2) and report['lost_count']==int(loss)
    assert 'samples' not in report and report['flight_authorized'] is False
    assert report['reason']=='observation_limit'
