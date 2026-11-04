"""Bounded offline conversion of a frozen raw stream; original bytes stay intact.

The MP4 clock is reconstructed at a requested nominal FPS. It is not a camera
clock or an alignment with telemetry. HEVC decoder copies use the same bootstrap,
SEI removal and timing normalizer as the live preview, before decoding.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import selectors
import shutil
import signal
import stat
import subprocess
import sys
import time

from .hevc import HevcAnnexBFilter
from .video import HlsVideo, VideoError

MAX_SOURCE_BYTES = 512 * 1024 * 1024
MAX_INTERMEDIATE_BYTES = 512 * 1024 * 1024
MAX_OUTPUT_BYTES = 512 * 1024 * 1024
MAX_LOG_BYTES = 64 * 1024
MAX_PROBE_BYTES = 64 * 1024
CHUNK_BYTES = 65536
_NOFOLLOW = os.O_NOFOLLOW | os.O_CLOEXEC
_ENV = {'PATH': os.environ.get('PATH', '/usr/bin:/bin'), 'LC_ALL': 'C'}


class VideoExportError(VideoError):
    def __init__(self, reason, *, cleanup_verified=True, diagnostics=None):
        super().__init__(reason, cleanup_verified=cleanup_verified)
        self.diagnostics = diagnostics


def _path(value):
    path = Path(value).absolute()
    if '..' in path.parts or '\0' in str(path) or not path.name:
        raise ValueError('unsafe_video_export_path')
    return path


def _parent(path, *, create=False):
    """Open every component with O_NOFOLLOW; keep the actual directory pinned."""
    parts = path.parent.parts
    fd = os.open(parts[0], os.O_RDONLY | os.O_DIRECTORY | _NOFOLLOW)
    try:
        for part in parts[1:]:
            try:
                child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | _NOFOLLOW, dir_fd=fd)
            except FileNotFoundError:
                if not create:raise
                try:os.mkdir(part, 0o700, dir_fd=fd)
                except FileExistsError:pass
                child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | _NOFOLLOW, dir_fd=fd)
            os.close(fd)
            fd = child
        return fd
    except BaseException:
        os.close(fd)
        raise


def _same_parent(path, expected):
    fd = _parent(path)
    try:
        current, original = os.fstat(fd), os.fstat(expected)
        if (current.st_dev, current.st_ino) != (original.st_dev, original.st_ino):
            raise VideoExportError('video_export_parent_changed')
    finally:os.close(fd)


def _stamp(info):
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns)


def _discard_published(parent, name, identity):
    """Rollback only our inode, never a replacement or preexisting output."""
    try:info = os.stat(name,dir_fd=parent,follow_symlinks=False)
    except FileNotFoundError:return
    if (info.st_dev,info.st_ino)==identity:os.unlink(name,dir_fd=parent)


def _deadline(deadline):
    if time.monotonic() >= deadline:raise VideoExportError('video_export_timeout')


def _hash(fd, deadline):
    os.lseek(fd, 0, os.SEEK_SET)
    digest = hashlib.sha256()
    while True:
        _deadline(deadline)
        data = os.read(fd, CHUNK_BYTES)
        if not data:break
        digest.update(data)
    return digest.hexdigest()


def _write(fd, data, total, maximum):
    if total + len(data) > maximum:raise VideoExportError('video_export_intermediate_limit')
    remaining = memoryview(data)
    while remaining:
        count = os.write(fd, remaining)
        if count <= 0:raise VideoExportError('video_export_write_failed')
        remaining = remaining[count:]
    return total + len(data)


def _limited(limiter, executable, *, maximum=None):
    maximum = MAX_OUTPUT_BYTES if maximum is None else maximum
    return [limiter, '--as=1610612736', '--nofile=64', '--core=0', f'--fsize={maximum}', '--', executable]


def _run(command, *, directory_fd, deadline, stage, stdin=None, stdout=None):
    """Drain bounded diagnostic pipes and always reap the owned process group."""
    process = None
    error = None
    output, log = bytearray(), bytearray()
    log_bytes = 0
    try:
        _deadline(deadline)
        process = subprocess.Popen(command, stdin=stdin if stdin is not None else subprocess.DEVNULL,
            stdout=subprocess.PIPE if stdout is None else stdout, stderr=subprocess.PIPE,
            cwd=f'/proc/self/fd/{directory_fd}', pass_fds=(directory_fd,),
            env=_ENV, start_new_session=True, umask=0o077, bufsize=0)
        with selectors.DefaultSelector() as poll:
            for pipe in (process.stderr, process.stdout):
                if pipe is not None:
                    os.set_blocking(pipe.fileno(), False)
                    poll.register(pipe, selectors.EVENT_READ)
            while poll.get_map() or process.poll() is None:
                _deadline(deadline)
                for key, _ in poll.select(min(.05, max(0, deadline-time.monotonic()))):
                    data = os.read(key.fileobj.fileno(), 8192)
                    if not data:
                        poll.unregister(key.fileobj)
                    elif key.fileobj is process.stderr:
                        log_bytes += len(data)
                        log.extend(data)
                        if len(log)>MAX_LOG_BYTES:del log[:-MAX_LOG_BYTES]
                    else:
                        output.extend(data)
                        if len(output)>MAX_PROBE_BYTES:raise VideoExportError('video_export_probe_limit')
            process.wait(timeout=max(.001, deadline-time.monotonic()))
        if process.returncode != 0:raise VideoExportError(f'video_export_{stage}_failed')
        return bytes(output), {'stage':stage, 'returncode':process.returncode,
            'stderr_bytes':log_bytes, 'stderr_truncated':log_bytes>MAX_LOG_BYTES,
            'stderr_tail':bytes(log).decode('utf-8','replace')}
    except BaseException as caught:
        error = caught
        if isinstance(caught, VideoExportError) and caught.diagnostics is None:
            caught.diagnostics = {'stage':stage,'returncode':process.poll() if process else None,
                'stderr_bytes':log_bytes,'stderr_truncated':log_bytes>MAX_LOG_BYTES,
                'stderr_tail':bytes(log).decode('utf-8','replace')}
        raise
    finally:
        if process is not None:
            if error is not None or process.poll() is None:
                try:os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:pass
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                raise VideoExportError('video_export_process_cleanup_failed', cleanup_verified=False) from None
            finally:
                for pipe in (process.stdout, process.stderr):
                    if pipe is not None:pipe.close()


def _normalizer(limiter, ffmpeg, fps, directory):
    # Reuse the live decoder's complete timing-normalizer command. Its small
    # per-HLS-file limit changes for the offline file; core dumps are disabled.
    preview = HlsVideo(directory/'unused-preview', input_codec='hevc', nominal_fps=fps)
    preview.limiter, preview.ffmpeg = limiter, ffmpeg
    command = preview._normalizer_command()
    command = [f'--fsize={MAX_INTERMEDIATE_BYTES}' if item.startswith('--fsize=') else item
        for item in command]
    command.insert(command.index('--'),'--core=0')
    return command


def _probe(data, nominal_fps):
    try:
        value = json.loads(data)
        streams = value['streams']
        if not isinstance(streams, list) or len(streams)!=1:raise ValueError
        stream = streams[0]
        frames = int(stream['nb_read_frames'])
        duration = float(value['format']['duration'])
        width, height = stream['width'], stream['height']
        if (stream['codec_type']!='video' or stream['codec_name']!='h264'
                or stream['pix_fmt']!='yuv420p' or frames<=0 or frames>=2**31
                or type(width) is not int or type(height) is not int
                or not 2<=width<=1920 or not 2<=height<=1080 or width%2 or height%2
                or not math.isfinite(duration) or duration<=0
                or abs(duration-frames/nominal_fps)>max(.01,1/nominal_fps)):
            raise ValueError
        return {'frames':frames, 'duration_seconds':duration, 'width':width, 'height':height,
            'codec':'h264', 'pixel_format':'yuv420p', 'profile':stream.get('profile'),
            'audio_streams':0, 'frame_rate':stream.get('avg_frame_rate')}
    except (KeyError,TypeError,ValueError,OverflowError,RecursionError):
        raise VideoExportError('video_export_invalid_probe') from None


def export_video(source: Path, output: Path, *, input_codec='hevc', nominal_fps=30,
                 timeout=180):
    """Publish a new MP4 and return a JSON-compatible provenance manifest.

    The complete raw connection stream is considered. Pre-bootstrap HEVC units
    and SEI are removed from the decoder copy only; corrupt input may decode
    fewer pictures, recorded in process diagnostics. No time interval is trimmed
    and no audio, camera timestamps or telemetry synchronization are invented.
    At most three private 512 MiB temporary files and bounded process logs exist.
    The source must remain unchanged throughout; only a verified result is
    linked atomically into the requested new output path. No manifest is written
    implicitly; the caller can save the returned JSON alongside the result.
    """
    if input_codec not in ('hevc','h264'):raise ValueError('video export codec must be hevc or h264')
    if (type(nominal_fps) not in (int,float) or not math.isfinite(nominal_fps)
            or not 1<=nominal_fps<=60):raise ValueError('nominal video FPS must be in [1,60]')
    if (type(timeout) not in (int,float) or not math.isfinite(timeout)
            or not 0<timeout<=600):raise ValueError('video export timeout must be in (0,600]')
    source, output = _path(source), _path(output)
    if source==output:raise ValueError('video export output must be new')
    ffmpeg, ffprobe, limiter = (shutil.which(name) for name in ('ffmpeg','ffprobe','prlimit'))
    if not all((ffmpeg,ffprobe,limiter)):raise VideoExportError('video_export_requires_ffmpeg_ffprobe_prlimit')
    deadline = time.monotonic()+timeout
    source_parent = output_parent = source_fd = scratch_fd = output_fd = None
    scratch = None
    published = None
    try:
        source_parent = _parent(source)
        source_fd = os.open(source.name, os.O_RDONLY|os.O_NONBLOCK|_NOFOLLOW, dir_fd=source_parent)
        initial = os.fstat(source_fd)
        if not stat.S_ISREG(initial.st_mode) or not 0<initial.st_size<=MAX_SOURCE_BYTES:
            raise ValueError('video export requires a nonempty regular source of at most512MiB')
        output_parent = _parent(output, create=True)
        try:os.stat(output.name, dir_fd=output_parent, follow_symlinks=False)
        except FileNotFoundError:pass
        else:raise ValueError('video export output must be new')
        scratch = '.video-export-'+secrets.token_hex(16)
        os.mkdir(scratch, 0o700, dir_fd=output_parent)
        scratch_fd = os.open(scratch, os.O_RDONLY|os.O_DIRECTORY|_NOFOLLOW, dir_fd=output_parent)
        filtered = HevcAnnexBFilter(wait_for_bootstrap=True,strip_sei=True) if input_codec=='hevc' else None
        digest, source_bytes, copied = hashlib.sha256(), 0, 0
        temporary = os.open('input.annexb',os.O_WRONLY|os.O_CREAT|os.O_EXCL|_NOFOLLOW,0o600,dir_fd=scratch_fd)
        try:
            while True:
                _deadline(deadline)
                data = os.read(source_fd,CHUNK_BYTES)
                if not data:break
                source_bytes += len(data)
                if source_bytes>MAX_SOURCE_BYTES:raise VideoExportError('video_export_source_changed')
                digest.update(data)
                copied = _write(temporary, filtered.feed(data) if filtered else data,
                    copied, MAX_INTERMEDIATE_BYTES)
            if filtered:
                copied = _write(temporary, filtered.finish(),copied,MAX_INTERMEDIATE_BYTES)
                if not filtered.bootstrap_ready:raise VideoExportError('video_export_no_hevc_bootstrap')
        finally:os.close(temporary)
        if source_bytes!=initial.st_size or _stamp(os.fstat(source_fd))!=_stamp(initial):
            raise VideoExportError('video_export_source_changed')
        stages = []
        decode_name = 'input.annexb'
        if filtered:
            incoming = os.open(decode_name,os.O_RDONLY|_NOFOLLOW,dir_fd=scratch_fd)
            normalized = os.open('normalized.hevc',os.O_WRONLY|os.O_CREAT|os.O_EXCL|_NOFOLLOW,0o600,dir_fd=scratch_fd)
            try:
                _, log = _run(_normalizer(limiter,ffmpeg,nominal_fps,output.parent/scratch),
                    directory_fd=scratch_fd, deadline=deadline, stage='normalizer',stdin=incoming,stdout=normalized)
                stages.append(log)
            finally:
                os.close(incoming);os.close(normalized)
            decode_name = 'normalized.hevc'
        incoming = os.open(decode_name,os.O_RDONLY|_NOFOLLOW,dir_fd=scratch_fd)
        try:
            command = _limited(limiter,ffmpeg)+[
                '-hide_banner','-loglevel','error','-nostdin','-n','-max_alloc','67108864','-threads','2',
                '-fflags','+genpts','-probesize','32768','-analyzeduration','100000',
                '-protocol_whitelist','file,pipe','-f',input_codec,'-framerate',str(nominal_fps),'-i','pipe:0',
                '-map','0:v:0','-an','-sn','-dn','-filter_threads','1','-vf',
                f"setpts=N/({nominal_fps}*TB),scale=w='min(1920,iw)':h='min(1080,ih)':force_original_aspect_ratio=decrease:force_divisible_by=2,format=yuv420p",
                '-c:v','libx264','-threads','2','-preset','veryfast','-crf','20','-profile:v','high',
                '-maxrate','5000000','-bufsize','10000000','-bf','0','-g',str(math.ceil(nominal_fps*2)),
                '-r',str(nominal_fps),'-fps_mode','cfr','-movflags','+faststart','-f','mp4','export.mp4']
            _, log = _run(command,directory_fd=scratch_fd,deadline=deadline,stage='encoder',
                stdin=incoming,stdout=subprocess.DEVNULL)
            stages.append(log)
        finally:os.close(incoming)
        output_fd = os.open('export.mp4',os.O_RDONLY|_NOFOLLOW,dir_fd=scratch_fd)
        encoded = os.fstat(output_fd)
        if not stat.S_ISREG(encoded.st_mode) or not 0<encoded.st_size<=MAX_OUTPUT_BYTES:
            raise VideoExportError('video_export_output_limit')
        probe, log = _run(_limited(limiter,ffprobe)+[
            '-v','error','-max_alloc','67108864','-threads','2','-protocol_whitelist','file,pipe',
            '-f','mov','-i','pipe:0','-count_frames','-show_entries',
            'stream=codec_type,codec_name,pix_fmt,profile,width,height,nb_read_frames,avg_frame_rate:format=duration,size',
            '-of','json'],directory_fd=scratch_fd,deadline=deadline,stage='probe',stdin=output_fd)
        stages.append(log)
        media = _probe(probe,nominal_fps)
        output_hash = _hash(output_fd,deadline)
        _same_parent(source,source_parent)
        current = os.stat(source.name,dir_fd=source_parent,follow_symlinks=False)
        if (_stamp(current)!=_stamp(initial) or _stamp(os.fstat(source_fd))!=_stamp(initial)
                or _hash(source_fd,deadline)!=digest.hexdigest()
                or _stamp(os.fstat(source_fd))!=_stamp(initial)
                or _stamp(os.stat(source.name,dir_fd=source_parent,follow_symlinks=False))!=_stamp(initial)):
            raise VideoExportError('video_export_source_changed')
        _same_parent(output,output_parent)
        _deadline(deadline)
        if (_stamp(os.stat('export.mp4',dir_fd=scratch_fd,follow_symlinks=False))!=_stamp(encoded)
                or _stamp(os.fstat(output_fd))!=_stamp(encoded)):
            raise VideoExportError('video_export_output_changed')
        os.fsync(output_fd)
        os.link('export.mp4',output.name,src_dir_fd=scratch_fd,dst_dir_fd=output_parent,follow_symlinks=False)
        linked = os.stat(output.name,dir_fd=output_parent,follow_symlinks=False)
        published = (linked.st_dev,linked.st_ino)
        if (not stat.S_ISREG(linked.st_mode) or published!=(encoded.st_dev,encoded.st_ino)
                or linked.st_size!=encoded.st_size or _hash(output_fd,deadline)!=output_hash):
            raise VideoExportError('video_export_output_changed')
        os.fsync(output_parent)
        return {'format':'neo2-control/video-export/v1',
            'created_at':datetime.now(timezone.utc).isoformat().replace('+00:00','Z'),
            'source':{'path':str(source),'codec':input_codec,'size_bytes':source_bytes,
                'sha256':digest.hexdigest(),'unchanged_verified':True},
            'output':{'path':str(output),'container':'mp4','size_bytes':encoded.st_size,
                'sha256':output_hash,'encoder':'libx264','faststart':True,**media},
            'timing':{'source':'local_nominal_fps','nominal_fps':nominal_fps,
                'presentation_timestamp_formula':'zero_based_decoded_frame_index / nominal_fps',
                'camera_timestamps_available':False,'telemetry_alignment_verified':False,
                'scope':'entire_recorded_connection_stream','chronological_trim':False},
            'preprocessing':{'hevc_filter':filtered is not None,'wait_for_bootstrap':bool(filtered),
                'strip_sei':bool(filtered),'bootstrap_discarded_bytes':filtered.bootstrap_discarded_bytes if filtered else 0,
                'dropped_sei_nals':filtered.dropped_sei_nals if filtered else 0,
                'dropped_sei_bytes':filtered.dropped_sei_bytes if filtered else 0,
                'normalization':f'hevc_metadata=tick_rate={nominal_fps}:num_ticks_poc_diff_one=1' if filtered else None,
                'normalization_before_decode':bool(filtered),'maximum_dimensions':[1920,1080]},
            'processes':stages,'cleanup_verified':True}
    except BaseException:
        if published is not None and output_parent is not None:
            _discard_published(output_parent,output.name,published)
        raise
    finally:
        prior_error = sys.exception()
        for fd in (source_fd,output_fd):
            if fd is not None:os.close(fd)
        try:
            if scratch_fd is not None:
                # Reset cached directory iteration even if its reported cursor
                # is already zero. Btrfs may otherwise reuse an empty listing
                # obtained before the encoder created its files.
                os.lseek(scratch_fd,0,os.SEEK_SET)
                for name in os.listdir(scratch_fd):os.unlink(name,dir_fd=scratch_fd)
                expected = os.fstat(scratch_fd)
                info = os.stat(scratch,dir_fd=output_parent,follow_symlinks=False)
                if (info.st_dev,info.st_ino)!=(expected.st_dev,expected.st_ino):
                    raise OSError('scratch directory changed')
                os.rmdir(scratch,dir_fd=output_parent)
        except OSError as error:
            if published is not None:
                _discard_published(output_parent,output.name,published)
            raise VideoExportError('video_export_temporary_cleanup_failed',cleanup_verified=False,
                diagnostics={'stage':'temporary_cleanup','errno':error.errno,'detail':str(error),
                    'previous_error':str(prior_error) if prior_error is not None else None}) from error
        finally:
            for fd in (scratch_fd,source_parent,output_parent):
                if fd is not None:os.close(fd)
