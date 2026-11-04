"""Actual local FFmpeg decoding plus bounded process failure scenarios."""
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import pytest

from neo2_control import video as module
from neo2_control.video import HlsVideo, VideoError
from test_observation import until


@pytest.fixture(scope='module')
def elementary():
    return subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error',
        '-f', 'lavfi', '-i', 'testsrc2=size=160x96:rate=10', '-t', '12',
        '-c:v', 'libx264', '-preset', 'ultrafast', '-g', '10',
        '-f', 'h264', 'pipe:1'], check=True, capture_output=True, timeout=10).stdout


def test_real_decoder_produces_bounded_private_playable_hls(tmp_path, elementary):
    with HlsVideo(tmp_path / 'hls', nominal_fps=10) as video:
        for offset in range(0, len(elementary), 1452):
            assert video.feed(elementary[offset:offset + 1452])
    final = video.snapshot()
    assert final['state'] == 'offline' and final['error'] is None
    assert final['frames_encoded'] == 120
    assert final['input_bytes'] == final['written_bytes'] == len(elementary)
    assert not final['worker_alive'] and video.process.poll() == 0
    assert video.directory.stat().st_mode & 0o777 == 0o700
    for path in video.directory.iterdir():
        assert path.stat().st_mode & 0o777 == 0o600
    playlist = video.directory / 'index.m3u8'
    text = playlist.read_text()
    assert '#EXT-X-ENDLIST' in text and '#EXT-X-INDEPENDENT-SEGMENTS' in text
    assert text.count('#EXTINF:') == 6
    assert len(list(video.directory.glob('*.ts'))) <= 8
    for path in video.directory.glob('*.ts'):
        assert path.stat().st_size < 2_500_000
    decoded = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
        '-show_entries', 'stream=codec_name,width,height', '-of', 'json', str(playlist)],
        check=True, capture_output=True, text=True, timeout=5)
    stream = json.loads(decoded.stdout)['streams'][0]
    assert stream == {'codec_name': 'h264', 'width': 160, 'height': 96}
    # Demuxing metadata alone is insufficient: decode the retained playlist.
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-i', str(playlist),
        '-f', 'null', '-'], check=True, capture_output=True, timeout=5)


def test_status_becomes_live_stale_then_offline_from_local_progress(tmp_path, elementary):
    clock = [1.0]
    with HlsVideo(tmp_path / 'hls', nominal_fps=10, clock=lambda: clock[0]) as video:
        assert video.snapshot()['state'] == 'connecting'
        for offset in range(0, len(elementary), 1452):
            assert video.feed(elementary[offset:offset + 1452])
            time.sleep(.001)
        until(lambda: video.snapshot()['frames_encoded'] > 0)
        until(lambda: (video.directory / 'index.m3u8').exists())
        assert video.snapshot()['state'] == 'live'
        clock[0] = 4.0
        assert video.snapshot()['state'] == 'stale'
        assert video.snapshot()['timestamp_source'] == 'local_nominal_fps'
    assert video.snapshot()['state'] == 'offline'


def test_absent_video_is_distinct_from_corrupt_video_and_does_not_claim_ready(tmp_path):
    with HlsVideo(tmp_path / 'empty') as empty:
        pass
    assert empty.snapshot()['state'] == 'offline' and empty.snapshot()['error'] is None
    with HlsVideo(tmp_path / 'corrupt') as corrupt:
        corrupt.feed(b'not elementary video' * 50)
    final = corrupt.snapshot()
    assert final['state'] == 'failed' and final['frames_encoded'] == 0
    assert final['error'] in ('video_decoder_failed', 'video_decoder_io_error')
    assert not final['application_ready'] and not final['flight_ready']


@pytest.mark.parametrize('fps', [0, 61, True, float('nan'), float('inf'), '30'])
def test_invalid_local_config_precedes_files(tmp_path, fps):
    with pytest.raises(ValueError):
        HlsVideo(tmp_path / 'hls', nominal_fps=fps)
    assert not list(tmp_path.iterdir())


def test_existing_directory_or_symlink_is_not_reused(tmp_path):
    with pytest.raises(ValueError, match='must be new'):
        HlsVideo(tmp_path)
    alias = tmp_path / 'alias'
    alias.symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(ValueError, match='must be new'):
        HlsVideo(alias)


def test_missing_dependencies_precede_output_creation(tmp_path, monkeypatch):
    monkeypatch.setattr(module.shutil, 'which', lambda name: None)
    with pytest.raises(VideoError, match='requires_ffmpeg'):
        HlsVideo(tmp_path / 'hls')
    assert not list(tmp_path.iterdir())


def fake_decoder(monkeypatch, tmp_path, body):
    executable = tmp_path / 'decoder'
    executable.write_text('#!/usr/bin/python3\n' + body)
    executable.chmod(0o700)
    original = module.shutil.which
    monkeypatch.setattr(module.shutil, 'which', lambda name: str(executable) if name == 'ffmpeg' else original(name))


@pytest.mark.parametrize('kind', ['bytes', 'chunks'])
def test_stalled_decoder_has_bounded_nonblocking_input_and_is_reaped(tmp_path, monkeypatch, kind):
    fake_decoder(monkeypatch, tmp_path, 'import time\ntime.sleep(30)\n')
    monkeypatch.setattr(module, 'CLOSE_TIMEOUT', .1)
    video = HlsVideo(tmp_path / 'hls').__enter__()
    pid = video.process.pid
    started = time.monotonic()
    payload = bytes(65536) if kind == 'bytes' else b'x'
    for _ in range(10000):
        if not video.feed(payload):
            break
    else:
        pytest.fail('queue remained unbounded')
    assert time.monotonic() - started < 1
    final = video.close()
    assert final['error'] == 'video_input_queue_full'
    assert final['queued_bytes'] == 0 and not final['worker_alive']
    assert video.process.poll() is not None
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)


def test_close_timeout_terminates_owned_decoder_and_keeps_diagnostic_bounded(tmp_path, monkeypatch):
    fake_decoder(monkeypatch, tmp_path,
        "import os, time\nos.write(2, b'x' * 200000)\ntime.sleep(30)\n")
    monkeypatch.setattr(module, 'CLOSE_TIMEOUT', .15)
    with HlsVideo(tmp_path / 'hls') as video:
        until(lambda: len(video._stderr) > 0)
    final = video.snapshot()
    assert final['error'] == 'video_close_timeout' and not final['worker_alive']
    assert (video.directory / 'decoder.log').stat().st_size <= module.MAX_STDERR_BYTES
    assert len(video._line) <= 4096
    assert video.process.poll() is not None


def test_output_limit_stops_decoder_without_blocking_producer(tmp_path, monkeypatch):
    fake_decoder(monkeypatch, tmp_path,
        "import time\nfrom pathlib import Path\nPath('unexpected').write_bytes(b'x' * 5000)\ntime.sleep(30)\n")
    monkeypatch.setattr(module, 'MAX_DISK_BYTES', 4096)
    with HlsVideo(tmp_path / 'hls') as video:
        until(lambda: video.error is not None)
        assert not video.feed(b'ignored')
    assert video.snapshot()['error'] == 'video_output_limit_exceeded'
    assert not video.snapshot()['worker_alive']


def test_process_start_error_and_reuse_are_explicit(tmp_path, monkeypatch):
    with HlsVideo(tmp_path / 'one') as video:
        pass
    with pytest.raises(VideoError, match='cannot_be_reused'):
        video.__enter__()
    original = module.subprocess.Popen
    def unavailable(*args, **kwargs):
        raise OSError('private backend detail')
    monkeypatch.setattr(module.subprocess, 'Popen', unavailable)
    failed = HlsVideo(tmp_path / 'failed')
    with pytest.raises(OSError):
        failed.__enter__()
    assert failed.snapshot()['error'] == 'video_decoder_start_failed'
    assert failed.closed and failed.process is None


def test_hevc_input_is_decoded_and_transcoded_to_h264(tmp_path):
    raw = subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error',
        '-f', 'lavfi', '-i', 'testsrc2=size=128x96:rate=5', '-t', '1',
        '-c:v', 'libx265', '-x265-params', 'pools=none:frame-threads=1:wpp=0:log-level=error',
        '-f', 'hevc', 'pipe:1'], check=True, capture_output=True, timeout=10).stdout
    with HlsVideo(tmp_path / 'hevc', nominal_fps=5, input_codec='hevc') as video:
        for index in range(0, len(raw), 17):
            assert video.feed(raw[index:index+17])
    result = video.snapshot()
    assert result['input_codec'] == 'hevc' and result['codec'] == 'h264'
    assert result['frames_encoded'] == 5 and result['error'] is None
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error',
        '-i', str(video.directory / 'index.m3u8'), '-f', 'null', '-'],
        check=True, capture_output=True, timeout=5)
