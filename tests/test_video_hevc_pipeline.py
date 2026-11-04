"""Owned HEVC preprocessing/decoding processes, real images and bounded failure."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from neo2_control import video as module
from neo2_control.hevc import HevcAnnexBFilter
from neo2_control.video import HlsVideo
from test_hevc import incompatible_sei, nal
from test_observation import until


def units(data):
    starts = []
    index = 0
    while index < len(data) - 2:
        if data[index:index+4] == b'\0\0\0\1':
            starts.append((index, 4));index += 4
        elif data[index:index+3] == b'\0\0\1':
            starts.append((index, 3));index += 3
        else:index += 1
    return [((data[start+prefix] >> 1) & 63,
             data[start:starts[i+1][0] if i+1 < len(starts) else len(data)])
            for i,(start,prefix) in enumerate(starts)]


@pytest.fixture(scope='module')
def hevc_source():
    return subprocess.run([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'lavfi',
        '-i', 'testsrc2=size=128x96:rate=8', '-frames:v', '16', '-c:v', 'libx265',
        '-x265-params', 'pools=none:frame-threads=1:wpp=0:log-level=error:keyint=100:min-keyint=100:scenecut=0:bframes=0',
        '-f', 'hevc', 'pipe:1'], check=True, capture_output=True, timeout=10).stdout


@pytest.fixture(scope='module')
def hevc_with_timing_change(hevc_source):
    source=hevc_source
    changed = subprocess.run([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'hevc', '-i', 'pipe:0',
        '-map', '0:v:0', '-c:v', 'copy', '-bsf:v', 'hevc_metadata=tick_rate=4',
        '-f', 'hevc', 'pipe:1'], input=source, check=True, capture_output=True, timeout=10).stdout

    replacement = b''.join(body for kind,body in units(changed) if kind in (32,33))
    out, frames = [], 0
    for kind, body in units(source):
        if kind < 32:
            if frames == 8:out.append(replacement)
            frames += 1
        out.append(body)
        if frames == 1 and kind < 32:
            out.append(bytes.fromhex('000000015001f5070300020000030080'))
    assert frames == 16
    return b''.join(out)


def assert_reaped(video):
    assert video.snapshot()['cleanup_verified']
    for process in (video.normalizer, video.process):
        assert process is not None and process.poll() is not None
        with pytest.raises(ProcessLookupError):os.kill(process.pid, 0)


def test_real_timing_normalization_preserves_all_frames_and_produces_atomic_jpeg(tmp_path, hevc_with_timing_change):
    raw = hevc_with_timing_change
    with HlsVideo(tmp_path/'hls', input_codec='hevc', nominal_fps=8, preview_jpeg=True) as video:
        assert video.process.stdin.closed and video.normalizer.stdout is None
        assert os.getpgid(video.process.pid) == video.process.pid
        assert os.getpgid(video.normalizer.pid) == video.normalizer.pid
        for offset in range(0, len(raw), 71):assert video.feed(raw[offset:offset+71])
    result = video.snapshot()
    assert result['error'] is None and result['frames_encoded'] == 16
    assert result['input_bytes'] == len(raw)
    dropped = [body for kind,body in units(raw) if kind in (39,40)]
    assert result['written_bytes'] + result['hevc_bootstrap']['discarded_bytes'] + sum(map(len,dropped)) == len(raw)
    assert result['hevc_bootstrap']['ready'] and result['hevc_bootstrap']['opening_nal_type'] in range(16,24)
    assert (result['dropped_sei_nals'], result['dropped_sei_bytes']) == (len(dropped),sum(map(len,dropped)))
    assert result['hevc_normalization']['sei_policy'] == 'strip_all_for_preview'
    assert result['hevc_normalization']['enabled'] and result['hevc_normalization']['returncode'] == 0
    assert result['hevc_normalization']['tick_rate'] == 8
    assert result['timestamp_source'] == 'local_nominal_fps'
    assert result['preview_available'] and result['preview_age_seconds'] >= 0
    assert result['preview_timestamp_source'] == 'local_file_mtime'
    image = Path(result['preview_path'])
    assert image.read_bytes().startswith(b'\xff\xd8') and image.read_bytes().endswith(b'\xff\xd9')
    probe = subprocess.run(['ffprobe','-v','error','-show_entries','stream=codec_name,width,height',
        '-of','json',str(image)],check=True,capture_output=True,text=True,timeout=5)
    assert json.loads(probe.stdout)['streams'][0] == {'codec_name':'mjpeg','width':128,'height':96}
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-i',
        str(video.directory/'index.m3u8'),'-f','null','-'],check=True,capture_output=True,timeout=5)
    assert len(list(video.directory.iterdir())) <= 16
    assert all(path.stat().st_mode & 0o777 == 0o600 for path in video.directory.iterdir())
    assert 'Could not find ref' not in (video.directory/'decoder.log').read_text()
    assert_reaped(video)


def test_ffmpeg_incompatible_sei_does_not_drop_preview_pictures_or_change_raw_input(tmp_path,hevc_source):
    offending = incompatible_sei()
    raw = b''.join(body + (offending if kind < 32 else b'') for kind,body in units(hevc_source))
    # This is a parser compatibility failure beyond the default bounds filter.
    f = HevcAnnexBFilter()
    assert f.feed(raw)+f.finish() == raw and f.dropped_sei_nals == 0
    baseline = subprocess.run(['ffmpeg','-hide_banner','-loglevel','error',
        '-f','hevc','-i','pipe:0','-c:v','copy','-bsf:v','hevc_metadata=tick_rate=8',
        '-f','hevc','pipe:1'],input=raw,capture_output=True,timeout=10)
    assert b'payload_size too large (25 bytes)' in baseline.stderr
    before = bytes(raw)
    with HlsVideo(tmp_path/'preview',input_codec='hevc',nominal_fps=8,preview_jpeg=True) as video:
        for offset in range(0,len(raw),17):assert video.feed(raw[offset:offset+17])
    result=video.snapshot()
    assert raw == before
    assert result['error'] is None and result['frames_encoded'] == 16
    assert result['input_bytes'] == len(raw)
    assert result['written_bytes']+result['dropped_sei_bytes'] == len(raw)
    assert result['dropped_sei_nals'] == sum(kind in (39,40) for kind,_ in units(raw))
    assert result['preview_available'] and result['hevc_normalization']['returncode'] == 0
    assert 'Invalid SEI' not in (video.directory/'normalizer.log').read_text()
    assert 'Could not find ref' not in (video.directory/'decoder.log').read_text()
    assert video.hevc_filter.output_bytes == len(b''.join(body for kind,body in units(raw) if kind not in (39,40)))
    assert_reaped(video)


def fake_normalizer(monkeypatch, body):
    monkeypatch.setattr(HlsVideo, '_normalizer_command', lambda self: [sys.executable,'-u','-c',body])


def test_killed_first_stage_fails_only_video_and_reaps_both_processes(tmp_path, monkeypatch):
    fake_normalizer(monkeypatch, 'import time; time.sleep(30)')
    video = HlsVideo(tmp_path/'hls',input_codec='hevc').__enter__()
    os.kill(video.normalizer.pid, signal.SIGKILL)
    until(lambda: video.closed)
    result = video.close()
    assert result['error'] == 'video_normalizer_failed' or result['error'] == 'video_normalizer_exited'
    assert not video.feed(b'ignored')
    assert_reaped(video)


@pytest.mark.parametrize('limit_kind', ['bytes','chunks'])
def test_stalled_first_stage_cannot_block_producer_or_grow_its_queue(tmp_path,monkeypatch,limit_kind):
    fake_normalizer(monkeypatch,'import time; time.sleep(30)')
    monkeypatch.setattr(module,'CLOSE_TIMEOUT',.1)
    if limit_kind == 'bytes':monkeypatch.setattr(module,'MAX_INPUT_BYTES',65536)
    else:monkeypatch.setattr(module,'MAX_QUEUED_CHUNKS',8)
    video = HlsVideo(tmp_path/'hls',input_codec='hevc').__enter__()
    assert video.feed(nal(32)+nal(33)+nal(34)+nal(20)+nal(35))
    until(lambda: video.snapshot()['hevc_bootstrap']['ready'])
    raw = b'\0\0\0\1\x02\x01' + b'X'*4090
    started = time.monotonic()
    for _ in range(10000):
        if not video.feed(raw):break
    else:pytest.fail('HEVC queue remained unbounded')
    assert time.monotonic()-started < 1
    result = video.close()
    assert result['error'] == 'video_input_queue_full'
    assert result['queued_bytes'] == result['filtered_pending_bytes'] == 0
    assert_reaped(video)


def test_first_stage_stderr_and_close_timeout_are_bounded_and_reap_both(tmp_path,monkeypatch):
    fake_normalizer(monkeypatch,"import os,time; os.write(2,b'x'*200000); time.sleep(30)")
    monkeypatch.setattr(module,'CLOSE_TIMEOUT',.15)
    with HlsVideo(tmp_path/'hls',input_codec='hevc') as video:
        until(lambda: len(video._normalizer_stderr) > 0)
    assert video.snapshot()['error'] == 'video_close_timeout'
    assert (video.directory/'normalizer.log').stat().st_size <= module.MAX_STDERR_BYTES
    assert_reaped(video)


def test_second_process_start_failure_reaps_already_owned_decoder(tmp_path,monkeypatch):
    original = module.subprocess.Popen
    calls = []
    def start(*args,**kwargs):
        if calls:raise OSError('synthetic first-stage start failure')
        process=original(*args,**kwargs);calls.append(process);return process
    monkeypatch.setattr(module.subprocess,'Popen',start)
    video = HlsVideo(tmp_path/'hls',input_codec='hevc')
    with pytest.raises(OSError):video.__enter__()
    assert video.closed and video.snapshot()['cleanup_verified']
    assert calls[0].poll() is not None
    with pytest.raises(ProcessLookupError):os.kill(calls[0].pid,0)


def test_empty_hevc_and_invalid_framing_are_distinct_and_all_processes_reaped(tmp_path):
    with HlsVideo(tmp_path/'empty',input_codec='hevc') as empty:pass
    assert empty.snapshot()['error'] is None and not empty.snapshot()['preview_available']
    assert_reaped(empty)
    with HlsVideo(tmp_path/'corrupt',input_codec='hevc') as corrupt:
        assert corrupt.feed(b'not Annex-B')
    assert corrupt.snapshot()['error'] == 'video_hevc_framing_error'
    assert_reaped(corrupt)


@pytest.mark.parametrize('option',[1,None,'yes'])
def test_preview_configuration_fails_before_output_creation(tmp_path,option):
    with pytest.raises(ValueError):HlsVideo(tmp_path/'hls',preview_jpeg=option)
    assert not list(tmp_path.iterdir())


def test_current_stream_config_and_irap_can_arrive_after_orphan_frames_without_restarting_processes(tmp_path,hevc_source):
    orphan=nal(1,b'dependent-picture')+nal(35,b'\x30')
    with HlsVideo(tmp_path/'late-bootstrap',input_codec='hevc',nominal_fps=8,preview_jpeg=True) as video:
        processes=(video.normalizer.pid,video.process.pid)
        for _ in range(40):assert video.feed(orphan)
        until(lambda: video.snapshot()['queued_bytes']==0)
        state=video.snapshot()
        assert state['state']=='waiting_for_codec_config'
        assert not state['hevc_bootstrap']['ready'] and state['written_bytes']==0
        assert state['hevc_bootstrap']['waiting_seconds']>=0
        assert not state['error'] and not state['preview_available']
        assert all(p.poll() is None for p in (video.normalizer,video.process))
        for offset in range(0,len(hevc_source),71):
            assert video.feed(hevc_source[offset:offset+71])
    result=video.snapshot()
    assert result['error'] is None and result['frames_encoded']==16
    assert result['hevc_bootstrap']['ready'] and result['preview_available']
    assert result['hevc_bootstrap']['discarded_bytes']>=40*len(orphan)
    assert result['hevc_bootstrap']['buffered_bytes']==0
    assert (video.normalizer.pid,video.process.pid)==processes
    assert 'PPS id out of range' not in (video.directory/'normalizer.log').read_text()
    assert 'Could not find ref' not in (video.directory/'decoder.log').read_text()
    assert_reaped(video)


@pytest.mark.parametrize('headers',[False,True])
def test_missing_current_stream_bootstrap_stays_alive_until_explicit_eof(tmp_path,headers):
    source=(nal(32)+nal(33)+nal(34) if headers else b'')+nal(1)+nal(35)
    with HlsVideo(tmp_path/'waiting',input_codec='hevc',preview_jpeg=True) as video:
        assert video.feed(source)
        until(lambda: video.snapshot()['queued_bytes']==0)
        state=video.snapshot()
        assert state['state']==('waiting_for_keyframe' if headers else 'waiting_for_codec_config')
        assert not state['error'] and not state['hevc_bootstrap']['ready']
        assert state['written_bytes']==0 and state['frames_encoded']==0
        assert all(p.poll() is None for p in (video.normalizer,video.process))
    state=video.snapshot()
    assert state['error']=='video_no_codec_bootstrap' and state['frames_encoded']==0
    assert state['hevc_bootstrap']['discarded_bytes']==len(source)
    assert not state['preview_available']
    waiting=state['hevc_bootstrap']['waiting_seconds']
    assert video.snapshot(now=state['observed_at']+100)['hevc_bootstrap']['waiting_seconds']==waiting
    assert_reaped(video)
