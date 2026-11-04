import hashlib,json,threading
from urllib.error import HTTPError
from urllib.request import Request,urlopen

import pytest

from neo2_control.operator_client import FlightOperatorClient
from neo2_control.preview_acquisition import ReadOnlyPreviewFeed
from tools.replay_room_preview import create_preview_replay


@pytest.fixture
def cache(tmp_path):
    cv=pytest.importorskip('cv2');np=pytest.importorskip('numpy')
    (tmp_path/'frames').mkdir()
    ok,jpeg=cv.imencode('.jpg',np.zeros((96,128,3),np.uint8));assert ok
    raw=jpeg.tobytes();(tmp_path/'frames/frame-00000.jpg').write_bytes(raw)
    (tmp_path/'preview.json').write_text(json.dumps({
        'format':'neo2-control/recorded-room-preview/v1','source':{'sha256':'d'*64},
        'poses_supplied':False,'frames':[{'file':'frame-00000.jpg','recorded_available_at':100.,
            'sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw),'decoded_index':7}]}))
    return tmp_path,raw


def test_preview_fixture_serves_real_bytes_and_retains_source_availability_without_control_routes(cache):
    folder,raw=cache;server,state=create_preview_replay(folder)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        url=f'http://127.0.0.1:{server.server_port}'
        client=FlightOperatorClient(url);client.pin_connection(client.status())
        feed=ReadOnlyPreviewFeed(client,client._expected_context)
        frame=feed.read();assert frame['raw']==raw
        assert frame['metadata']['read_started_at']-frame['metadata']['encoded_age_seconds']<=state['origin_live_monotonic']
        assert feed.read() is None  # Reading the same old image never creates a new frame.
        with pytest.raises(HTTPError) as error:
            urlopen(Request(url+'/api/flight/intent',method='POST',data=b'{}'),timeout=2)
        assert error.value.code==405
        served=[row for row in state['requests'] if row['path']=='/api/flight/frame']
        assert len(served)==2 and all(row['decoded_index']==7 for row in served)
        assert served[0]['available_at']==served[1]['available_at']==state['origin_live_monotonic']
        assert served[1]['encoded_age_seconds']>=served[0]['encoded_age_seconds']
    finally:server.shutdown();server.server_close();thread.join(3)


def test_changed_photograph_is_rejected_before_a_replay_can_start(cache):
    folder,raw=cache;(folder/'frames/frame-00000.jpg').write_bytes(raw+b'changed')
    with pytest.raises(ValueError,match='image_changed'):create_preview_replay(folder)
