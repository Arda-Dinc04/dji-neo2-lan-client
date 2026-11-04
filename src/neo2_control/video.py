"""Bounded local Annex-B decoder, HEVC timing normalizer and HLS encoder.

Input is elementary H.264/HEVC, after any DJI framing has been removed. SWUDP
acceptance and byte receipt do not imply a valid elementary video stream.
"""
from __future__ import annotations

from collections import deque
import math
import os
from pathlib import Path
import selectors
import shutil
import signal
import subprocess
import threading
import time

from .hevc import HevcAnnexBFilter

MAX_INPUT_BYTES = 4 * 1024 * 1024
MAX_QUEUED_CHUNKS = 4096
MAX_DISK_BYTES = 16 * 1024 * 1024
MAX_STDERR_BYTES = 64 * 1024
CLOSE_TIMEOUT = 5.0


class VideoError(RuntimeError):
    def __init__(self, reason, *, cleanup_verified=True):
        super().__init__(reason)
        self.cleanup_verified = cleanup_verified


class HlsVideo:
    """Own the decoder and optional HEVC normalizer; producers never wait.

    A slow or failed decoder stops this consumer. It cannot stop transport
    acknowledgements or grow an unbounded queue. All output is local/private.
    Frame ages refer to local encoding progress, not to a camera timestamp.
    """
    def __init__(self, directory: str | Path, *, nominal_fps=30.0,
                 input_codec='h264', protected_paths=(), clock=time.monotonic,
                 preview_jpeg=False):
        if (type(nominal_fps) not in (int, float) or not math.isfinite(nominal_fps)
                or not 1 <= nominal_fps <= 60):
            raise ValueError('nominal video FPS must be in [1,60]')
        self.directory = Path(directory).absolute()
        if input_codec not in ('h264', 'hevc'):
            raise ValueError('video codec must be h264 or hevc')
        if type(preview_jpeg) is not bool:
            raise ValueError('JPEG preview option must be boolean')
        for path in protected_paths:
            other, directory = Path(path).resolve(), self.directory.resolve()
            if other.is_relative_to(directory) or directory.is_relative_to(other):
                raise ValueError('video directory must be separate from session inputs and outputs')
        if self.directory.exists() or self.directory.is_symlink():
            raise ValueError('video directory must be new')
        self.ffmpeg, self.limiter = shutil.which('ffmpeg'), shutil.which('prlimit')
        if self.ffmpeg is None or self.limiter is None:
            raise VideoError('video_requires_ffmpeg_and_prlimit')
        self.nominal_fps, self.input_codec, self.clock = nominal_fps, input_codec, clock
        self.preview_jpeg = preview_jpeg
        self.lock = threading.RLock()
        self.queue = deque()
        self.queued_bytes = self.input_bytes = self.written_bytes = 0
        self.frames_encoded = 0
        self.last_frame_at = None
        self.error = None
        self.returncode = None
        self.normalizer_returncode = None
        self.accepting = self.closing = self.closed = False
        self.process = self.thread = None
        self.normalizer = None
        # Preview does not consume SEI metadata. Some camera SEI pass generic
        # length checks but use escaping rejected by FFmpeg's metadata parser;
        # losing that access unit also loses the following reference pictures.
        # Only this decoder copy is stripped: transport/raw capture is separate.
        self.hevc_filter = (HevcAnnexBFilter(wait_for_bootstrap=True, strip_sei=True)
            if input_codec == 'hevc' else None)
        self.bootstrap_started_at = self.bootstrap_ready_at = self.bootstrap_finished_at = None
        self.filtered_pending_bytes = 0
        self._stop = threading.Event()
        self._stderr = bytearray()
        self._line = bytearray()
        self._normalizer_stderr = bytearray()

    def _command(self):
        command = [
            self.limiter, '--as=1610612736', '--nofile=64', '--fsize=4194304', '--',
            self.ffmpeg, '-hide_banner', '-loglevel', 'error', '-nostdin',
            '-max_alloc', '67108864', '-threads', '2',
            '-fflags', '+genpts', '-probesize', '32768', '-analyzeduration', '100000',
            '-f', self.input_codec, '-framerate', str(self.nominal_fps), '-i', 'pipe:0',
            '-map', '0:v:0', '-an', '-sn', '-dn', '-filter_threads', '1',
            '-vf', "scale=w='min(1280,iw)':h='min(720,ih)':force_original_aspect_ratio=decrease:force_divisible_by=2,format=yuv420p",
            '-c:v', 'libx264', '-threads', '2', '-preset', 'ultrafast', '-tune', 'zerolatency',
            '-b:v', '2000000', '-maxrate', '2500000', '-bufsize', '5000000',
            '-g', str(math.ceil(self.nominal_fps)), '-sc_threshold', '0',
            '-force_key_frames', 'expr:gte(t,n_forced*1)',
            '-f', 'hls', '-hls_time', '1', '-hls_list_size', '6',
            '-hls_delete_threshold', '1', '-hls_flags', 'delete_segments+temp_file+independent_segments',
            '-hls_segment_filename', 'segment-%08d.ts',
            '-progress', 'pipe:2', '-stats_period', '0.1', 'index.m3u8',
        ]
        if self.preview_jpeg:
            command.extend([
                '-map', '0:v:0', '-an', '-sn', '-dn',
                '-vf', "fps=5,scale=w='min(960,iw)':h=-2",
                '-c:v', 'mjpeg', '-threads', '1', '-f', 'image2',
                '-update', '1', '-atomic_writing', '1', 'preview.jpg',
            ])
        return command

    def _normalizer_command(self):
        return [
            self.limiter, '--as=1610612736', '--nofile=64', '--fsize=4194304', '--',
            self.ffmpeg, '-hide_banner', '-loglevel', 'error', '-nostdin',
            '-max_alloc', '67108864', '-threads', '1',
            '-probesize', '32768', '-analyzeduration', '100000',
            '-f', 'hevc', '-framerate', str(self.nominal_fps), '-i', 'pipe:0',
            '-map', '0:v:0', '-an', '-sn', '-dn', '-c:v', 'copy',
            '-bsf:v', f'hevc_metadata=tick_rate={self.nominal_fps}:num_ticks_poc_diff_one=1',
            '-f', 'hevc', 'pipe:1',
        ]

    def __enter__(self):
        if self.process is not None or self.closed:
            raise VideoError('video_consumer_cannot_be_reused')
        self.directory.mkdir(mode=0o700, parents=True, exist_ok=False)
        try:
            self.process = subprocess.Popen(self._command(), cwd=self.directory,
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                env={'PATH': os.environ.get('PATH', '/usr/bin:/bin'), 'LC_ALL': 'C'},
                start_new_session=True, umask=0o077, bufsize=0)
            if self.hevc_filter is not None:
                self.normalizer = subprocess.Popen(self._normalizer_command(), cwd=self.directory,
                    stdin=subprocess.PIPE, stdout=self.process.stdin, stderr=subprocess.PIPE,
                    env={'PATH': os.environ.get('PATH', '/usr/bin:/bin'), 'LC_ALL': 'C'},
                    start_new_session=True, umask=0o077, bufsize=0)
                # Only the normalizer owns the decoder's input pipe writer.
                self.process.stdin.close()
                os.set_blocking(self.normalizer.stdin.fileno(), False)
                os.set_blocking(self.normalizer.stderr.fileno(), False)
            else:
                os.set_blocking(self.process.stdin.fileno(), False)
            os.set_blocking(self.process.stderr.fileno(), False)
            self.accepting = True
            worker = self._run_hevc if self.hevc_filter is not None else self._run
            self.thread = threading.Thread(target=worker, name='neo2-video', daemon=True)
            self.thread.start()
            return self
        except BaseException:
            self._fail('video_normalizer_start_failed' if self.hevc_filter is not None
                and self.process is not None and self.normalizer is None else 'video_decoder_start_failed')
            try:
                self._reap()
            except Exception:
                self._fail('video_cleanup_unconfirmed')
            self.closed = True
            if any(p is not None and p.poll() is None for p in (self.process, self.normalizer)):
                raise VideoError('video_cleanup_unconfirmed', cleanup_verified=False) from None
            raise

    def _fail(self, reason):
        with self.lock:
            self.error = self.error or reason
            self.accepting = False
        self._stop.set()

    def feed(self, data: bytes):
        if not isinstance(data, bytes) or len(data) > 65536:
            raise ValueError('video input must be bytes of at most 65536 bytes')
        with self.lock:
            if not self.accepting:
                return False
            if self.queued_bytes + len(data) > MAX_INPUT_BYTES or len(self.queue) >= MAX_QUEUED_CHUNKS:
                self._fail('video_input_queue_full')
                return False
            if data:
                self.queue.append(data)
                self.queued_bytes += len(data)
                self.input_bytes += len(data)
                if self.hevc_filter is not None and self.bootstrap_started_at is None:
                    self.bootstrap_started_at = self.clock()
            return True

    def _progress(self, data):
        self._stderr.extend(data)
        if len(self._stderr) > MAX_STDERR_BYTES:
            del self._stderr[:-MAX_STDERR_BYTES]
        # Keep an unterminated hostile/error line bounded as well.
        self._line.extend(data)
        while b'\n' in self._line:
            raw, _, tail = self._line.partition(b'\n')
            self._line[:] = tail
            if raw.startswith(b'frame='):
                value = raw[6:].strip()
                if value.isdigit() and len(value) <= 12:
                    frames = int(value)
                    with self.lock:
                        if frames > self.frames_encoded:
                            self.frames_encoded, self.last_frame_at = frames, self.clock()
        if len(self._line) > 4096:
            self._line.clear()

    def _check_disk(self):
        files = list(self.directory.iterdir())
        size = 0
        for path in files:
            try:
                size += path.stat().st_size
            except FileNotFoundError:
                pass  # The HLS muxer atomically renames/removes its own files.
        if len(files) > 16 or size > MAX_DISK_BYTES:
            self._fail('video_output_limit_exceeded')

    def _run(self):
        process = self.process
        current, offset = None, 0
        input_open = True
        next_disk_check = self.clock()
        try:
            with selectors.DefaultSelector() as poll:
                poll.register(process.stderr, selectors.EVENT_READ)
                write_registered = False
                while not self._stop.is_set():
                    with self.lock:
                        if current is None and self.queue:
                            current, offset = self.queue.popleft(), 0
                        finish = self.closing and current is None
                    if finish and input_open:
                        if write_registered:
                            poll.unregister(process.stdin)
                            write_registered = False
                        process.stdin.close()
                        input_open = False
                    if current is not None and not write_registered:
                        poll.register(process.stdin, selectors.EVENT_WRITE)
                        write_registered = True
                    if current is None and write_registered:
                        poll.unregister(process.stdin)
                        write_registered = False
                    for key, events in poll.select(.025):
                        if key.fileobj is process.stderr:
                            chunk = os.read(process.stderr.fileno(), 8192)
                            if chunk:
                                self._progress(chunk)
                            else:
                                poll.unregister(process.stderr)
                        elif current is not None:
                            try:
                                count = os.write(process.stdin.fileno(), current[offset:])
                            except BlockingIOError:
                                continue
                            offset += count
                            with self.lock:
                                self.written_bytes += count
                                self.queued_bytes -= count
                            if offset == len(current):
                                current = None
                    if self.clock() >= next_disk_check:
                        self._check_disk()
                        next_disk_check = self.clock() + .25
                    if process.poll() is not None:
                        # Drain the last progress block before classifying exit.
                        while True:
                            chunk = os.read(process.stderr.fileno(), 8192)
                            if not chunk:
                                break
                            self._progress(chunk)
                        if process.returncode != 0 and self.input_bytes:
                            self._fail('video_decoder_failed')
                        elif not self.closing:
                            self._fail('video_decoder_exited')
                        elif self.input_bytes and not self.frames_encoded:
                            self._fail('video_no_decodable_frames')
                        break
        except (BrokenPipeError, OSError, ValueError):
            self._fail('video_decoder_io_error')
        except BaseException:
            self._fail('video_worker_failed')
        finally:
            self._worker_finished()

    def _normalizer_progress(self, data):
        self._normalizer_stderr.extend(data)
        if len(self._normalizer_stderr) > MAX_STDERR_BYTES:
            del self._normalizer_stderr[:-MAX_STDERR_BYTES]

    def _drain_stderr(self, process, *, normalizer=False):
        consume = self._normalizer_progress if normalizer else self._progress
        while True:
            try:
                chunk = os.read(process.stderr.fileno(), 8192)
            except BlockingIOError:
                break
            if not chunk:
                break
            consume(chunk)

    def _run_hevc(self):
        decoder, normalizer = self.process, self.normalizer
        current, offset = None, 0
        input_open, filter_finished = True, False
        next_disk_check = self.clock()
        try:
            with selectors.DefaultSelector() as poll:
                poll.register(decoder.stderr, selectors.EVENT_READ, 'decoder')
                poll.register(normalizer.stderr, selectors.EVENT_READ, 'normalizer')
                write_registered = False
                while not self._stop.is_set():
                    raw = None
                    if current is None:
                        with self.lock:
                            if self.queue:
                                raw = self.queue.popleft()
                                self.queued_bytes -= len(raw)
                            finish = self.closing and raw is None
                        if raw is not None:
                            current = self.hevc_filter.feed(raw) or None
                            offset = 0
                        elif finish and not filter_finished:
                            current = self.hevc_filter.finish() or None
                            offset = 0
                            filter_finished = True
                            if self.input_bytes and not self.hevc_filter.bootstrap_ready:
                                self._fail('video_no_codec_bootstrap')
                                break
                    with self.lock:
                        if self.hevc_filter.bootstrap_ready and self.bootstrap_ready_at is None:
                            self.bootstrap_ready_at = self.clock()
                        self.filtered_pending_bytes = 0 if current is None else len(current) - offset
                    if filter_finished and current is None and input_open:
                        if write_registered:
                            poll.unregister(normalizer.stdin)
                            write_registered = False
                        normalizer.stdin.close()
                        input_open = False
                    if current is not None and not write_registered:
                        poll.register(normalizer.stdin, selectors.EVENT_WRITE, 'input')
                        write_registered = True
                    elif current is None and write_registered:
                        poll.unregister(normalizer.stdin)
                        write_registered = False
                    # Consume queued pieces of an incomplete NAL without an
                    # artificial25ms delay, while still draining both stderr pipes.
                    for key, _ in poll.select(0 if raw is not None else .025):
                        if key.data in ('decoder', 'normalizer'):
                            chunk = os.read(key.fileobj.fileno(), 8192)
                            if chunk:
                                if key.data == 'normalizer':self._normalizer_progress(chunk)
                                else:self._progress(chunk)
                            else:
                                poll.unregister(key.fileobj)
                        elif current is not None:
                            try:
                                count = os.write(normalizer.stdin.fileno(),
                                    memoryview(current)[offset:offset + 65536])
                            except BlockingIOError:
                                continue
                            if not count:
                                raise BrokenPipeError('HEVC normalizer accepted no bytes')
                            offset += count
                            with self.lock:
                                self.written_bytes += count
                                self.filtered_pending_bytes = len(current) - offset
                            if offset == len(current):
                                current = None
                    if self.clock() >= next_disk_check:
                        self._check_disk()
                        next_disk_check = self.clock() + .25
                    if normalizer.poll() is not None:
                        self._drain_stderr(normalizer, normalizer=True)
                        if normalizer.returncode != 0 and self.input_bytes:
                            self._fail('video_normalizer_failed')
                            break
                        if not self.closing or input_open:
                            self._fail('video_normalizer_exited')
                            break
                    if decoder.poll() is not None:
                        self._drain_stderr(decoder)
                        if decoder.returncode != 0 and self.input_bytes:
                            self._fail('video_decoder_failed')
                        elif not self.closing:
                            self._fail('video_decoder_exited')
                        elif self.input_bytes and not self.frames_encoded:
                            self._fail('video_no_decodable_frames')
                        break
        except ValueError:
            self._fail('video_hevc_framing_error')
        except (BrokenPipeError, OSError):
            self._fail('video_normalizer_io_error')
        except BaseException:
            self._fail('video_worker_failed')
        finally:
            self._worker_finished()

    def _worker_finished(self):
        with self.lock:
            self.accepting = False
        try:
            self._reap()
        except Exception:
            self._fail('video_cleanup_unconfirmed')
        logs = [('decoder.log', self._stderr)]
        if self.normalizer is not None:
            logs.append(('normalizer.log', self._normalizer_stderr))
        for name, data in logs:
            try:
                descriptor = os.open(self.directory / name,
                    os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
                with os.fdopen(descriptor, 'wb') as output:
                    output.write(data)
            except OSError:
                self._fail('video_journal_unavailable')
        with self.lock:
            self.queue.clear()
            self.queued_bytes = self.filtered_pending_bytes = 0
            self.closed = True
            self.bootstrap_finished_at = self.clock()

    def _reap(self):
        errors = []
        for process in (self.normalizer, self.process):
            if process is None:
                continue
            try:
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        process.wait(timeout=.5)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        process.wait(timeout=1)
                else:
                    process.wait(timeout=1)
            except Exception as error:
                errors.append(error)
            finally:
                if process is self.normalizer:self.normalizer_returncode = process.returncode
                else:self.returncode = process.returncode
                for stream in (process.stdin, process.stdout, process.stderr):
                    if stream is not None and not stream.closed:
                        try:stream.close()
                        except Exception as error:errors.append(error)
        if errors:
            raise errors[0]

    def close(self):
        with self.lock:
            self.accepting, self.closing = False, True
        if self.thread is not None:
            # Two serial HEVC stages need a bounded drain period each. The
            # existing single-stage H.264 deadline remains unchanged.
            self.thread.join(timeout=CLOSE_TIMEOUT * (2 if self.normalizer is not None else 1))
            if self.thread.is_alive():
                self._fail('video_close_timeout')
                self.thread.join(timeout=2)
            if self.thread.is_alive():
                raise VideoError('video_worker_did_not_stop', cleanup_verified=False)
        else:
            self.closed = True
        if any(p is not None and p.poll() is None for p in (self.process, self.normalizer)):
            raise VideoError('video_cleanup_unconfirmed', cleanup_verified=False)
        return self.snapshot()

    def __exit__(self, *exception):
        self.close()

    def snapshot(self, now=None):
        now = self.clock() if now is None else now
        preview_path = self.directory / 'preview.jpg'
        preview_available, preview_age = False, None
        if self.preview_jpeg:
            try:
                if preview_path.is_file() and not preview_path.is_symlink():
                    elapsed=time.time()-preview_path.stat().st_mtime
                    if math.isfinite(elapsed) and elapsed>=0:
                        preview_age,preview_available=elapsed,True
            except FileNotFoundError:
                pass
        with self.lock:
            age = None if self.last_frame_at is None else max(0, now - self.last_frame_at)
            bootstrap = self.hevc_filter
            bootstrap_state = ('disabled' if bootstrap is None else 'ready' if bootstrap.bootstrap_ready
                else 'waiting_for_keyframe' if len(bootstrap.bootstrap_seen_parameter_sets) == 3
                else 'waiting_for_codec_config')
            bootstrap_end = (self.bootstrap_ready_at if self.bootstrap_ready_at is not None
                else self.bootstrap_finished_at if self.bootstrap_finished_at is not None else now)
            bootstrap_wait = (None if self.bootstrap_started_at is None else
                              max(0, bootstrap_end - self.bootstrap_started_at))
            if self.error:
                state = 'failed'
            elif self.closed:
                state = 'offline'
            elif bootstrap is not None and self.input_bytes and not bootstrap.bootstrap_ready:
                state = bootstrap_state
            elif age is None:
                state = 'connecting'
            else:
                state = ('live' if age <= 2 else 'stale') if (self.directory / 'index.m3u8').is_file() else 'connecting'
            return {'enabled': True, 'state': state, 'error': self.error,
                'observed_at': now,
                'codec': 'h264', 'input_codec': self.input_codec, 'input_format': 'annex_b',
                'input_codec_source': 'local_configuration',
                'nominal_fps': self.nominal_fps, 'timestamp_source': 'local_nominal_fps',
                'frames_encoded': self.frames_encoded, 'last_encoded_age_seconds': age,
                'input_bytes': self.input_bytes, 'written_bytes': self.written_bytes,
                'queued_bytes': self.queued_bytes, 'maximum_queued_bytes': MAX_INPUT_BYTES,
                'filtered_pending_bytes': self.filtered_pending_bytes,
                'hevc_bootstrap': {'enabled': bootstrap is not None,
                    'ready': bool(bootstrap is not None and bootstrap.bootstrap_ready),
                    'state': bootstrap_state, 'waiting_seconds': bootstrap_wait,
                    'seen_parameter_sets': [] if bootstrap is None else sorted(bootstrap.bootstrap_seen_parameter_sets),
                    'opening_nal_type': None if bootstrap is None else bootstrap.bootstrap_opening_nal_type,
                    'discarded_nals': 0 if bootstrap is None else bootstrap.bootstrap_discarded_nals,
                    'discarded_bytes': 0 if bootstrap is None else bootstrap.bootstrap_discarded_bytes,
                    'buffered_bytes': 0 if bootstrap is None else bootstrap.bootstrap_buffered_bytes,
                    'maximum_buffered_bytes': None if bootstrap is None else bootstrap.MAX_BOOTSTRAP_BYTES},
                'hevc_normalization': {'enabled': self.hevc_filter is not None,
                    'method': 'ffmpeg_hevc_metadata' if self.hevc_filter is not None else None,
                    'sei_policy': 'strip_all_for_preview' if self.hevc_filter is not None else None,
                    'tick_rate': self.nominal_fps if self.hevc_filter is not None else None,
                    'num_ticks_poc_diff_one': 1 if self.hevc_filter is not None else None,
                    'timestamp_source': 'local_nominal_fps',
                    'returncode': self.normalizer_returncode,
                    'process_running': bool(self.normalizer and self.normalizer.poll() is None)},
                'dropped_sei_nals': 0 if self.hevc_filter is None else self.hevc_filter.dropped_sei_nals,
                'dropped_sei_bytes': 0 if self.hevc_filter is None else self.hevc_filter.dropped_sei_bytes,
                'preprocessing_buffered_bytes': 0 if self.hevc_filter is None else self.hevc_filter.buffered_bytes,
                'maximum_preprocessing_nal_bytes': None if self.hevc_filter is None else self.hevc_filter.max_nal_bytes,
                'preview_enabled': self.preview_jpeg,
                'preview_path': str(preview_path) if self.preview_jpeg else None,
                'preview_available': preview_available, 'preview_age_seconds': preview_age,
                'preview_timestamp_source': 'local_file_mtime' if self.preview_jpeg else None,
                'directory': str(self.directory), 'playlist_path': str(self.directory / 'index.m3u8'),
                'returncode': self.returncode, 'worker_alive': bool(self.thread and self.thread.is_alive()),
                'cleanup_verified': bool(self.closed and not (self.thread and self.thread.is_alive())
                    and all(p is None or p.poll() is not None for p in (self.process, self.normalizer))),
                'application_ready': False, 'flight_ready': False}
