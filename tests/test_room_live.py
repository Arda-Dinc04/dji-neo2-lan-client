import hashlib,json,threading,urllib.request,urllib.error
import pytest
from neo2_control.room_live import read_state,create_server


@pytest.fixture
def acquisition(tmp_path):
    folder=tmp_path/'capture';(folder/'map/revision-0001').mkdir(parents=True)
    (folder/'map/frames').mkdir()
    snapshot={'format':'neo2-control/progressive-room/v1','revision':1,'context':'fixture',
        'navigation_eligible':False,'state':'reconstructed','newest_geometry_observation_received_at':10.,
        'points':[[0,0,3]],'patches':[],'poses':[]}
    raw=json.dumps(snapshot).encode();(folder/'map/revision-0001/snapshot.json').write_bytes(raw)
    head={k:v for k,v in snapshot.items() if k not in ('points','patches','poses')}
    head.update(format='neo2-control/progressive-room-head/v1',
        snapshot_file='revision-0001/snapshot.json',snapshot_sha256=hashlib.sha256(raw).hexdigest())
    (folder/'map/latest.json').write_text(json.dumps(head))
    (folder/'capture-configuration.json').write_text(json.dumps({'source':'fixture'}))
    image=b'\xff\xd8fixture\xff\xd9';(folder/'map/frames/frame-00000.jpg').write_bytes(image)
    (folder/'map/inputs.json').write_text(json.dumps({'images':{'frame-00000.jpg':{
        'sha256':hashlib.sha256(image).hexdigest()}}}))
    return folder


def test_reader_checks_immutable_snapshot_and_computes_current_age(acquisition):
    state=read_state(acquisition,clock=lambda:14.)
    assert state['source_age_seconds']==4 and state['snapshot']['state']=='reconstructed'
    assert read_state(acquisition,clock=lambda:20.)['snapshot']['state']=='stale_geometry'
    (acquisition/'map/revision-0001/snapshot.json').write_text('{}')
    with pytest.raises(ValueError,match='snapshot_changed'):read_state(acquisition,clock=lambda:20.)


def test_snapshot_pointer_cannot_escape_the_acquisition(acquisition):
    path=acquisition/'map/latest.json';head=json.loads(path.read_text())
    head['snapshot_file']='../../other.json';path.write_text(json.dumps(head))
    with pytest.raises(ValueError,match='path_invalid'):read_state(acquisition)


def test_completed_capture_remains_viewable_after_a_pc_reboot(acquisition):
    (acquisition/'capture-result.json').write_text('{}')
    state=read_state(acquisition,clock=lambda:1.)
    assert state['finished'] and state['source_age_seconds'] is None
    assert state['snapshot']['state']=='archived_geometry'


def test_previous_boot_without_a_completion_record_is_interrupted(acquisition):
    (acquisition/'capture-configuration.json').write_text(json.dumps({'pc_boot_id':'previous-boot'}))
    state=read_state(acquisition,clock=lambda:1.)
    assert state['failed'] and not state['finished'] and state['source_age_seconds'] is None


def test_failure_before_any_geometry_is_reported(tmp_path):
    (tmp_path/'worker-failure.json').write_text('{}')
    state=read_state(tmp_path)
    assert state['snapshot'] is None and state['failed']


def test_loopback_server_exposes_only_fixed_read_routes_and_verified_images(acquisition,tmp_path):
    bundle=tmp_path/'bundle.js';bundle.write_text('const NeoRoom={};')
    with create_server(acquisition,bundle) as server:
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        base=f'http://127.0.0.1:{server.server_port}'
        try:
            with urllib.request.urlopen(base+'/state.json') as response:
                state=json.load(response)
                assert state['snapshot']['state']=='stale_geometry'
                assert response.headers['Cross-Origin-Resource-Policy']=='same-origin'
            with urllib.request.urlopen(base+'/frames/frame-00000.jpg') as response:
                assert response.read()==b'\xff\xd8fixture\xff\xd9'
            for request,code in [(urllib.request.Request(base+'/state.json',headers={'Origin':'https://example.com'}),403),
                    (urllib.request.Request(base+'/state.json',headers={'Host':'example.com'}),403),
                    (urllib.request.Request(base+'/api/flight/intent',data=b'{}'),405),
                    (urllib.request.Request(base+'/capture-configuration.json'),404)]:
                with pytest.raises(urllib.error.HTTPError) as error:urllib.request.urlopen(request)
                assert error.value.code==code
            (acquisition/'map/frames/frame-00000.jpg').write_bytes(b'changed')
            with pytest.raises(urllib.error.HTTPError) as error:
                urllib.request.urlopen(base+'/frames/frame-00000.jpg')
            assert error.value.code==409
        finally:server.shutdown();thread.join(3)
