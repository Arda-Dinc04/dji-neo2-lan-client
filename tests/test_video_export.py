"""Real offline codecs, nominal clocks, unchanged raw input and bounded failure."""
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys

import pytest

from neo2_control import video_export as export
from neo2_control.video import HlsVideo
from test_video_hevc_pipeline import hevc_source, hevc_with_timing_change


@pytest.fixture(scope='module')
def h264_source():
    return subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-f','lavfi',
        '-i','testsrc2=size=160x96:rate=10','-frames:v','20','-c:v','libx264',
        '-threads','1','-preset','ultrafast','-f','h264','pipe:1'],
        check=True,capture_output=True,timeout=10).stdout


def assert_clean(directory):
    assert not list(directory.glob('.video-export-*'))


@pytest.mark.parametrize('codec',['h264','hevc'])
def test_real_pc_mp4_preserves_raw_and_explicitly_uses_nominal_clock(tmp_path,h264_source,hevc_with_timing_change,codec):
    raw = h264_source if codec=='h264' else hevc_with_timing_change
    source, target = tmp_path/('raw.'+codec), tmp_path/'flight.mp4'
    source.write_bytes(raw)
    original = source.stat()
    fps, frames, dimensions = (10,20,(160,96)) if codec=='h264' else (8,16,(128,96))
    metadata = export.export_video(source,target,input_codec=codec,nominal_fps=fps,timeout=20)
    assert metadata['source']['unchanged_verified']
    assert metadata['source']['sha256']==hashlib.sha256(raw).hexdigest()
    assert metadata['source']['size_bytes']==len(raw) and source.read_bytes()==raw
    assert export._stamp(source.stat())==export._stamp(original)
    assert metadata['output']['sha256']==hashlib.sha256(target.read_bytes()).hexdigest()
    assert metadata['output']['frames']==frames
    assert metadata['output']['duration_seconds']==pytest.approx(frames/fps,abs=.01)
    assert (metadata['output']['width'],metadata['output']['height'])==dimensions
    assert metadata['output']['audio_streams']==0 and metadata['output']['pixel_format']=='yuv420p'
    assert metadata['timing']=={'source':'local_nominal_fps','nominal_fps':fps,
        'presentation_timestamp_formula':'zero_based_decoded_frame_index / nominal_fps',
        'camera_timestamps_available':False,'telemetry_alignment_verified':False,
        'scope':'entire_recorded_connection_stream','chronological_trim':False}
    assert metadata['cleanup_verified'] and target.stat().st_mode&0o777==0o600
    data = target.read_bytes()
    assert data.index(b'moov')<data.index(b'mdat')
    if codec=='hevc':
        assert metadata['preprocessing']['dropped_sei_nals']>0
        assert metadata['preprocessing']['normalization']=='hevc_metadata=tick_rate=8:num_ticks_poc_diff_one=1'
        assert metadata['preprocessing']['normalization_before_decode']
        assert [p['stage'] for p in metadata['processes']]==['normalizer','encoder','probe']
    else:assert not metadata['preprocessing']['hevc_filter']
    json.dumps(metadata,allow_nan=False)
    subprocess.run(['ffmpeg','-v','error','-i',str(target),'-f','null','-'],
        check=True,capture_output=True,timeout=10)
    assert_clean(tmp_path)


def test_hevc_normalizer_is_the_existing_exact_command_except_offline_file_bound(tmp_path):
    live = HlsVideo(tmp_path/'live',input_codec='hevc',nominal_fps=30)
    expected = live._normalizer_command()
    actual = export._normalizer(live.limiter,live.ffmpeg,30,tmp_path/'offline')
    assert [v for v in actual if not v.startswith(('--fsize=','--core='))]==[v for v in expected if not v.startswith('--fsize=')]
    assert f'--fsize={export.MAX_INTERMEDIATE_BYTES}' in actual
    assert '--core=0' in actual


@pytest.mark.parametrize('option,value',[
    ('input_codec','unknown'),('nominal_fps',True),('nominal_fps',0),('nominal_fps',61),
    ('nominal_fps',float('nan')),('timeout',0),('timeout',601),('timeout',float('inf')),('timeout',True)])
def test_invalid_options_are_inert(tmp_path,option,value):
    with pytest.raises(ValueError):export.export_video(tmp_path/'missing',tmp_path/'output.mp4',**{option:value})
    assert not list(tmp_path.iterdir())


def test_missing_tools_precede_files_or_processes(tmp_path,monkeypatch):
    monkeypatch.setattr(export.shutil,'which',lambda name:None)
    with pytest.raises(export.VideoExportError,match='requires_ffmpeg'):
        export.export_video(tmp_path/'missing',tmp_path/'created'/'output.mp4')
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('kind',['empty','large','directory','fifo'])
def test_non_regular_empty_or_oversized_source_never_starts_decoder(tmp_path,monkeypatch,kind):
    source = tmp_path/'raw'
    if kind=='directory':source.mkdir()
    elif kind=='fifo':os.mkfifo(source)
    else:
        with source.open('wb') as stream:stream.truncate(export.MAX_SOURCE_BYTES+1 if kind=='large' else 0)
    monkeypatch.setattr(export.subprocess,'Popen',lambda *a,**k:pytest.fail('decoder started'))
    with pytest.raises((ValueError,OSError)):export.export_video(source,tmp_path/'flight.mp4')
    assert not (tmp_path/'flight.mp4').exists()
    assert_clean(tmp_path)


@pytest.mark.parametrize('kind',['existing','output_symlink','source_symlink','source_parent_symlink','output_parent_symlink'])
def test_existing_outputs_and_symlinks_are_never_followed(tmp_path,h264_source,kind):
    source, target = tmp_path/'raw.h264',tmp_path/'flight.mp4'
    source.write_bytes(h264_source)
    foreign = tmp_path/'foreign';foreign.write_bytes(b'leave unchanged')
    if kind=='existing':target.write_bytes(b'leave unchanged')
    elif kind=='output_symlink':target.symlink_to(foreign)
    elif kind=='source_symlink':source.unlink();source.symlink_to(foreign)
    else:
        alias = tmp_path/'alias';alias.symlink_to(tmp_path,target_is_directory=True)
        if kind=='source_parent_symlink':source=alias/'raw.h264'
        else:target=alias/'flight.mp4'
    with pytest.raises((ValueError,OSError)):export.export_video(source,target,input_codec='h264')
    assert foreign.read_bytes()==b'leave unchanged'
    if kind in ('existing','output_symlink'):assert target.read_bytes()==b'leave unchanged'
    assert_clean(tmp_path)


def test_timeout_kills_and_reaps_only_owned_process_and_cleans_files(tmp_path,h264_source,monkeypatch):
    source, target = tmp_path/'raw.h264',tmp_path/'flight.mp4'
    source.write_bytes(h264_source)
    processes = []
    popen, limited = export.subprocess.Popen,export._limited
    def record(*args,**kwargs):
        child = popen(*args,**kwargs);processes.append(child);return child
    def stalled(limiter,executable,**kwargs):
        return limited(limiter,sys.executable,**kwargs)+['-u','-c',
            "import sys,time;sys.stderr.write('x'*200000);sys.stderr.flush();time.sleep(30)"]
    monkeypatch.setattr(export.subprocess,'Popen',record)
    monkeypatch.setattr(export,'_limited',stalled)
    with pytest.raises(export.VideoExportError,match='timeout') as caught:
        export.export_video(source,target,input_codec='h264',timeout=.3)
    assert processes and all(p.poll() is not None for p in processes)
    for process in processes:
        with pytest.raises(ProcessLookupError):os.kill(process.pid,0)
    assert caught.value.cleanup_verified
    assert caught.value.diagnostics['stderr_truncated']
    assert len(caught.value.diagnostics['stderr_tail'])<=export.MAX_LOG_BYTES
    assert source.read_bytes()==h264_source and not target.exists()
    assert_clean(tmp_path)


def test_corrupt_hevc_has_no_bootstrap_and_leaves_no_output(tmp_path):
    source = tmp_path/'raw.hevc';source.write_bytes(b'not annex b')
    with pytest.raises(ValueError):export.export_video(source,tmp_path/'flight.mp4')
    assert source.read_bytes()==b'not annex b'
    assert not (tmp_path/'flight.mp4').exists()
    assert_clean(tmp_path)


@pytest.mark.parametrize('change',['append','replace','same_bytes_rewritten'])
def test_source_must_stay_frozen_through_processing(tmp_path,h264_source,monkeypatch,change):
    source, target = tmp_path/'raw.h264',tmp_path/'flight.mp4'
    source.write_bytes(h264_source)
    original = export._run
    def run(*args,**kwargs):
        result=original(*args,**kwargs)
        if kwargs['stage']=='encoder':
            if change=='append':
                with source.open('ab') as stream:stream.write(b'changed')
            elif change=='replace':
                source.rename(tmp_path/'original-raw');source.write_bytes(h264_source)
            else:source.write_bytes(h264_source)
        return result
    monkeypatch.setattr(export,'_run',run)
    with pytest.raises(export.VideoExportError,match='source_changed'):
        export.export_video(source,target,input_codec='h264',timeout=20)
    assert not target.exists()
    assert_clean(tmp_path)


def test_output_created_during_processing_cannot_be_overwritten(tmp_path,h264_source,monkeypatch):
    source, target = tmp_path/'raw.h264',tmp_path/'flight.mp4'
    source.write_bytes(h264_source)
    original = export._run
    def run(*args,**kwargs):
        result=original(*args,**kwargs)
        if kwargs['stage']=='probe':target.write_bytes(b'foreign output')
        return result
    monkeypatch.setattr(export,'_run',run)
    with pytest.raises(FileExistsError):export.export_video(source,target,input_codec='h264',timeout=20)
    assert target.read_bytes()==b'foreign output' and source.read_bytes()==h264_source
    assert_clean(tmp_path)


@pytest.mark.parametrize('limit',['intermediate','output'])
def test_disk_limits_remove_partial_files_and_preserve_raw(tmp_path,h264_source,monkeypatch,limit):
    source, target = tmp_path/'raw.h264',tmp_path/'flight.mp4'
    source.write_bytes(h264_source)
    monkeypatch.setattr(export,'MAX_INTERMEDIATE_BYTES' if limit=='intermediate' else 'MAX_OUTPUT_BYTES',1024)
    with pytest.raises(export.VideoExportError):export.export_video(source,target,input_codec='h264',timeout=10)
    assert source.read_bytes()==h264_source and not target.exists()
    assert_clean(tmp_path)


def test_probe_stdout_limit_is_bounded_and_reaped(tmp_path,h264_source,monkeypatch):
    source, target = tmp_path/'raw.h264',tmp_path/'flight.mp4'
    source.write_bytes(h264_source)
    limited, popen = export._limited,export.subprocess.Popen
    processes = []
    def command(limiter,executable,**kwargs):
        if Path(executable).name=='ffprobe':
            return limited(limiter,sys.executable,**kwargs)+['-c',"print('x'*200000)"]
        return limited(limiter,executable,**kwargs)
    def record(*args,**kwargs):
        process = popen(*args,**kwargs);processes.append(process);return process
    monkeypatch.setattr(export,'_limited',command)
    monkeypatch.setattr(export.subprocess,'Popen',record)
    with pytest.raises(export.VideoExportError,match='probe_limit'):
        export.export_video(source,target,input_codec='h264',timeout=10)
    assert all(process.poll() is not None for process in processes)
    assert not target.exists() and source.read_bytes()==h264_source
    assert_clean(tmp_path)


def test_failed_directory_fsync_rolls_back_only_our_published_output(tmp_path,h264_source,monkeypatch):
    source, target = tmp_path/'raw.h264',tmp_path/'flight.mp4'
    source.write_bytes(h264_source)
    original = export.os.fsync
    def fail(fd):
        if stat.S_ISDIR(os.fstat(fd).st_mode):raise OSError('injected fsync failure')
        return original(fd)
    monkeypatch.setattr(export.os,'fsync',fail)
    with pytest.raises(OSError):export.export_video(source,target,input_codec='h264',timeout=10)
    assert not target.exists() and source.read_bytes()==h264_source
    assert_clean(tmp_path)


def test_temporary_cleanup_failure_is_explicit_and_does_not_return_output(tmp_path,h264_source,monkeypatch):
    source, target = tmp_path/'raw.h264',tmp_path/'flight.mp4'
    source.write_bytes(h264_source)
    original, failed = export.os.unlink,[]
    def fail(name,*args,**kwargs):
        if name=='input.annexb' and not failed:
            failed.append(True)
            raise OSError('injected cleanup failure')
        return original(name,*args,**kwargs)
    monkeypatch.setattr(export.os,'unlink',fail)
    with pytest.raises(export.VideoExportError,match='temporary_cleanup_failed') as caught:
        export.export_video(source,target,input_codec='h264',timeout=10)
    assert not caught.value.cleanup_verified
    assert not target.exists() and source.read_bytes()==h264_source
    remaining = list(tmp_path.glob('.video-export-*'))
    assert len(remaining)==1
    for file in remaining[0].iterdir():file.unlink()
    remaining[0].rmdir()


def test_temporary_output_replaced_with_symlink_is_not_published_or_followed(tmp_path,h264_source,monkeypatch):
    source, target = tmp_path/'raw.h264',tmp_path/'flight.mp4'
    source.write_bytes(h264_source)
    original = export._run
    def run(*args,**kwargs):
        result = original(*args,**kwargs)
        if kwargs['stage']=='probe':
            fd = kwargs['directory_fd']
            os.rename('export.mp4','old.mp4',src_dir_fd=fd,dst_dir_fd=fd)
            os.symlink(str(source),'export.mp4',dir_fd=fd)
        return result
    monkeypatch.setattr(export,'_run',run)
    with pytest.raises(export.VideoExportError,match='output_changed'):
        export.export_video(source,target,input_codec='h264',timeout=10)
    assert not target.exists() and source.read_bytes()==h264_source
    assert_clean(tmp_path)


def test_cached_directory_iterator_is_reset_even_with_reported_offset_zero(tmp_path,h264_source,monkeypatch):
    source, target = tmp_path/'raw.h264',tmp_path/'flight.mp4'
    source.write_bytes(h264_source)
    original, listing, seek = export._run,export.os.listdir,export.os.lseek
    inspected, stale = [],[]
    def run(*args,**kwargs):
        result = original(*args,**kwargs)
        if kwargs['stage']=='probe':
            fd = kwargs['directory_fd']
            assert seek(fd,0,os.SEEK_CUR)==0
            inspected.append(fd)
            stale.append(True)
        return result
    def rewind(fd,offset,whence):
        if fd in inspected and offset==0 and whence==os.SEEK_SET:stale.clear()
        return seek(fd,offset,whence)
    def guarded(fd):
        if fd in inspected and stale:return []  # Old empty Btrfs iterator state.
        return listing(fd)
    monkeypatch.setattr(export,'_run',run)
    monkeypatch.setattr(export.os,'lseek',rewind)
    monkeypatch.setattr(export.os,'listdir',guarded)
    metadata = export.export_video(source,target,input_codec='h264',timeout=10)
    assert inspected and metadata['cleanup_verified']
    assert target.exists() and source.read_bytes()==h264_source
    assert_clean(tmp_path)
