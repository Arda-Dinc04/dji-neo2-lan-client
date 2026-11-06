"""Local preview fan-out never changes aircraft traffic; fallback stays usable."""
import socket
import subprocess

import pytest

from neo2_control.video import HlsVideo
from neo2_control.session import run_session
from test_video_hevc_pipeline import hevc_source


@pytest.mark.parametrize('url',['rtsp://remote.invalid:8554/preview',
    'rtsp://127.0.0.1:8554/other','rtsp://127.0.0.1:65536/preview',
    'rtsp://127.0.0.1:8554/preview?target=remote','rtsp://user:key@127.0.0.1:8554/preview',42])
def test_remote_or_arbitrary_output_is_rejected_before_effects(tmp_path,url):
    with pytest.raises(ValueError,match='owned loopback'):
        HlsVideo(tmp_path/'hls',rtsp_output=url)
    assert not (tmp_path/'hls').exists()


def test_rtsp_activation_requires_preview_only_before_capture(tmp_path):
    target=tmp_path/'capture.jsonl'
    with pytest.raises(ValueError,match='owned preview-only'):
        run_session(capture_path=target,rtsp_preview_url='rtsp://127.0.0.1:8554/preview')
    assert not target.exists()


def test_bridge_unavailable_preserves_real_fixture_hls_and_jpeg(tmp_path,hevc_source):
    with socket.socket() as listener:
        listener.bind(('127.0.0.1',0));port=listener.getsockname()[1]
    with HlsVideo(tmp_path/'hls',input_codec='hevc',nominal_fps=8,preview_jpeg=True,
                  rtsp_output=f'rtsp://127.0.0.1:{port}/preview') as video:
        for offset in range(0,len(hevc_source),1024):assert video.feed(hevc_source[offset:offset+1024])
    snapshot=video.snapshot()
    assert snapshot['error'] is None and snapshot['frames_encoded']==16
    assert snapshot['preview_available'] and snapshot['cleanup_verified']
    assert snapshot['rtsp_preview']['enabled']
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-i',str(video.directory/'index.m3u8'),
        '-f','null','-'],check=True,capture_output=True,timeout=10)
