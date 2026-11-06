"""Same-origin local WHEP proxy has no arbitrary URL or aircraft command input."""
from urllib.request import Request,urlopen
from urllib.error import HTTPError

import pytest

from test_camera_web import server,TOKEN


def call(base,path,*,method='POST',body=b'v=0\r\n',headers=None,auth=True):
    values={'Content-Type':'application/sdp',**({'X-Neo2-Token':TOKEN} if auth else {}),**(headers or {})}
    try:r=urlopen(Request(base+path,data=body if method=='POST' else None,
        headers=values,method=method),timeout=3)
    except HTTPError as error:r=error
    with r:return r.status,dict(r.headers),r.read()


def test_offer_and_delete_are_authenticated_local_and_never_expose_backend_url(tmp_path):
    with server(tmp_path) as (base,console):
        identifier='a'*48
        console.webrtc_offer=lambda body:{'session_id':identifier,'sdp':b'v=0\r\nanswer'}
        deleted=[];console.webrtc_delete=deleted.append
        assert call(base,'/api/camera/webrtc',auth=False)[0]==401
        assert call(base,'/api/camera/webrtc',headers={'Origin':'https://foreign.invalid'})[0]==403
        code,headers,body=call(base,'/api/camera/webrtc')
        assert code==201 and headers['Content-Type']=='application/sdp'
        assert headers['Location']=='/api/camera/webrtc/'+identifier
        assert headers['Cache-Control']=='no-store' and body==b'v=0\r\nanswer'
        assert call(base,headers['Location'],method='DELETE',auth=False)[0]==401
        assert not deleted
        assert call(base,headers['Location'],method='DELETE')[0]==204
        assert deleted==[identifier]
        assert not console.calls


@pytest.mark.parametrize('body,headers',[(b'x'*65537,{}),(b'',{}),
    (b'v=0\r\n',{'Content-Type':'application/json'})])
def test_invalid_offers_never_reach_the_bridge(tmp_path,body,headers):
    with server(tmp_path) as (base,console):
        console.webrtc_offer=lambda body:pytest.fail('invalid offer forwarded')
        assert call(base,'/api/camera/webrtc',body=body,headers=headers)[0]==400


def test_peer_location_cannot_be_selected_by_the_browser_or_backend(tmp_path):
    with server(tmp_path) as (base,console):
        console.webrtc_offer=lambda body:{'session_id':'http://foreign.invalid/','sdp':b'v=0\r\n'}
        assert call(base,'/api/camera/webrtc')[0]==409
        console.webrtc_delete=lambda identifier:pytest.fail('arbitrary delete forwarded')
        assert call(base,'/api/camera/webrtc/http://foreign.invalid/',method='DELETE')[0]==404


def test_disabled_feature_and_packaged_module(tmp_path):
    with server(tmp_path) as (base,console):
        assert call(base,'/api/camera/webrtc')[0]==409
        code,headers,body=call(base,'/webrtc.js',method='GET',auth=False)
        assert code==200 and headers['Content-Type'].startswith('text/javascript')
        assert b'RTCPeerConnection' in body
