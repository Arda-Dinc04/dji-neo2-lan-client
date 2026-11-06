"""Camera ownership, graceful local bridge failure, and generation-safe offers."""
from dataclasses import replace
from pathlib import Path

import pytest

from neo2_control.camera_console import CameraConfig
from test_camera_console import harness


class Bridge:
    def __init__(self,*args,**kwargs):
        self.ready=False;self.fail=False;self.clean=True;self.offers=[];self.deleted=[]
        self.rtsp_url='rtsp://127.0.0.1:12345/preview'
    def start(self):
        if self.fail:raise RuntimeError('synthetic_start_failure')
        self.ready=True;return self
    def snapshot(self):return {'ready':self.ready,'generation':'synthetic','error':None,'cleanup_verified':not self.ready and self.clean}
    def close(self):self.ready=False;return self.snapshot()
    def offer(self,sdp):self.offers.append(sdp);return {'session_id':'a'*48,'sdp':b'v=0\r\n'}
    def delete(self,identifier):self.deleted.append(identifier)


def configure(h,*,fail=False,clean=True):
    h.console.config=replace(h.config,preview_only=True,webrtc=True,mediamtx_binary='/synthetic/local-mediamtx')
    bridge=Bridge();bridge.fail=fail;bridge.clean=clean
    h.console.webrtc_factory=lambda *a,**k:bridge
    return bridge


def fresh_frame(h,call):
    path=Path(call.kwargs['hls_directory'])/'preview.jpg';path.parent.mkdir();path.write_bytes(b'synthetic-jpeg')
    call.publish(video={'last_encoded_age_seconds':0,'preview_age_seconds':0,'frames_encoded':3})


def test_owned_bridge_is_wired_to_preview_and_cleaned_with_camera(harness):
    h=harness;bridge=configure(h);call=h.connect()
    assert call.kwargs['rtsp_preview_url']==bridge.rtsp_url and call.kwargs['preview_only']
    assert not h.console.status()['webrtc']['ready']
    fresh_frame(h,call)
    assert h.console.status()['webrtc']['ready']
    assert h.console.webrtc_offer(b'v=0\r\n')['session_id']=='a'*48
    h.stop()
    assert not bridge.ready and h.console.status()['cleanup_verified']


def test_missing_bridge_falls_back_without_stopping_camera(harness):
    h=harness;bridge=configure(h,fail=True);call=h.connect()
    assert 'rtsp_preview_url' not in call.kwargs
    fresh_frame(h,call)
    state=h.console.status()
    assert state['video']['live'] and state['error'] is None
    assert state['webrtc']['error']=='webrtc_start_failed'
    h.stop();assert h.console.status()['cleanup_verified']


def test_incomplete_bridge_cleanup_blocks_a_new_connection(harness):
    h=harness;configure(h,clean=False);h.connect();h.stop()
    assert not h.console.status()['cleanup_verified']
    with pytest.raises(RuntimeError,match='cleanup_unconfirmed'):h.console.connect()


def test_start_failure_without_verified_cleanup_cannot_start_aircraft_session(harness):
    h=harness;configure(h,fail=True,clean=False)
    h.console.connect();h.join()
    assert not h.sessions
    assert h.console.status()['error']=='webrtc_cleanup_unconfirmed'
    assert not h.console.status()['cleanup_verified']


def test_offer_must_use_fresh_video_and_same_generation(harness):
    h=harness;bridge=configure(h);call=h.connect()
    with pytest.raises(RuntimeError,match='webrtc_not_ready'):h.console.webrtc_offer(b'v=0\r\n')
    fresh_frame(h,call)
    original=bridge.offer
    def replaced(sdp):
        result=original(sdp);h.console.generation='replacement';return result
    bridge.offer=replaced
    with pytest.raises(RuntimeError,match='camera_session_changed'):h.console.webrtc_offer(b'v=0\r\n')
    assert bridge.deleted==['a'*48]
    h.stop()


def test_webrtc_cannot_enable_general_camera_or_flight_mode(tmp_path):
    with pytest.raises(ValueError,match='webrtc_requires_preview_only'):
        CameraConfig('en0',tmp_path/'wlm',tmp_path/'ble',webrtc=True,mediamtx_binary='/synthetic')
