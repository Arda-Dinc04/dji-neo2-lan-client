"""Owned native decoder, bounded nonblocking ingress and expiring RGB output.

This consumer does not open a radio or change the existing HTTP/HLS preview.
Only its monitor thread parses/hashes decoded frames; feed never decodes or
waits for the child. A full kernel input queue terminally invalidates it.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import selectors
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

from .video import VideoError
from .video_export import _limited, _ENV
from .video_stream import DecodedObservation, IncrementalHevcDecoder, LatestDecodedFrame, _context, _finite

INPUT_HEADER = struct.Struct('!QddB')
OUTPUT_HEADER = struct.Struct('!II')
MAX_INPUT = 65536
MAX_METADATA = 64 * 1024
MAX_RGB = 1920 * 1080 * 3
MAX_LOG = 64 * 1024
PROVENANCE_KEYS = ('decoded_index', 'raw_picture_ordinal', 'pixels',
    'source_first_slice_byte_offset', 'source_vcl_body_sha256',
    'candidate_fields', 'encoded_interval_checksum', 'normalized_packet_sha256')


def provenance_bytes(record):
    """Stable rolling proof, independent of replay wall-clock scheduling."""
    return json.dumps({k: record[k] for k in PROVENANCE_KEYS}, sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode() + b'\n'


def _emit(record, rgb=b''):
    raw = json.dumps(record, separators=(',', ':'), allow_nan=False).encode()
    if not 0 < len(raw) <= MAX_METADATA or len(rgb) > MAX_RGB:
        raise ValueError('worker_output_size_invalid')
    stream = sys.stdout.buffer
    for data in (OUTPUT_HEADER.pack(len(raw), len(rgb)), raw, rgb):
        stream.write(data)
    stream.flush()


def _child(args):
    def receive(sample):
        if decoder.latest.peek() is sample:
            _emit({'type': 'frame', 'observation': sample.record()}, sample.rgb)
        else:
            # Already too old on the child: retain the provenance proof, but
            # do not copy multi-megabyte pixels that can never be offered.
            _emit({'type': 'stale', 'observation': sample.record()})
    decoder = IncrementalHevcDecoder(args.owner, args.generation,
        nominal_fps=args.fps, maximum_age=args.maximum_age, on_frame=receive)
    _emit({'type': 'ready'})
    sequence = 0
    with socket.socket(fileno=args.input_fd) as ingress:
        while True:
            packet = ingress.recv(INPUT_HEADER.size + MAX_INPUT + 1)
            if not INPUT_HEADER.size <= len(packet) <= INPUT_HEADER.size + MAX_INPUT:
                raise ValueError('worker_input_packet_size_invalid')
            number, delivered, received, kind = INPUT_HEADER.unpack_from(packet)
            if number != sequence or kind not in (0, 1, 2):
                raise ValueError('worker_input_sequence_invalid')
            if kind == 2:
                if len(packet) != INPUT_HEADER.size:
                    raise ValueError('worker_end_packet_invalid')
                decoder.finish()
                _emit({'type': 'finished', 'summary': decoder.status(), 'consumed_chunks': sequence})
                return 0
            decoder.feed(packet[INPUT_HEADER.size:], delivered_at=delivered,
                received_at=received if kind else None)
            _emit({'type': 'consumed', 'sequence': sequence})
            sequence += 1


class ManagedHevcDecoder:
    """One process/session, bounded socket ingress, one expiring observation.

    Construct/start/finish/wait/close outside the UDP callback. feed is a small
    validation and MSG_DONTWAIT send; overflow never discards an arbitrary HEVC
    chunk and continues. It terminally invalidates this consumer instead.
    """
    def __init__(self, owner_epoch, connection_generation, *, nominal_fps=30.,
            maximum_age=.35, input_buffer_bytes=4*1024*1024, stall_timeout=2., startup_timeout=5.):
        self.context = _context(owner_epoch, connection_generation)
        if (not _finite(nominal_fps) or not 1 <= nominal_fps <= 60
                or type(input_buffer_bytes) is not int or not 128*1024 <= input_buffer_bytes <= 4*1024*1024
                or not _finite(stall_timeout) or not .1 <= stall_timeout <= 10
                or not _finite(startup_timeout) or not .1 <= startup_timeout <= 30):
            raise ValueError('worker_options_invalid')
        self.latest = LatestDecodedFrame(*self.context, maximum_age=maximum_age)
        self.fps, self.maximum_age = nominal_fps, maximum_age
        self.input_buffer_bytes = input_buffer_bytes
        self.stall_timeout, self.startup_timeout = stall_timeout, startup_timeout
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.process = self.thread = self.ingress = None
        self.started = self.ready = self.input_finished = self.finished = self.closed = False
        self.error = None
        self.submitted_chunks = self.submitted_bytes = self.received_frames = 0
        self.stale_without_rgb = 0
        self.last_ack = -1
        self.last_delivery = self.last_clock = None
        self.started_at = self.progress_at = None
        self.input_kernel_buffer_bytes = None
        self.stderr = bytearray()
        self.stderr_bytes = 0
        self.decoder_summary = None
        self.provenance = hashlib.sha256()

    def start(self):
        with self.lock:
            if self.started or self.closed:
                raise ValueError('worker_cannot_be_reused')
            self.started = True
        child = None
        try:
            limiter = shutil.which('prlimit')
            if limiter is None:
                raise ValueError('worker_requires_prlimit')
            self.ingress, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            self.ingress.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.input_buffer_bytes)
            self.input_kernel_buffer_bytes = self.ingress.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
            self.ingress.setblocking(False)
            command = _limited(limiter, sys.executable, maximum=1024*1024) + [
                '-m', 'neo2_control.video_worker', '--input-fd', str(child.fileno()),
                '--owner', self.context[0], '--generation', self.context[1],
                '--fps', str(self.fps), '--maximum-age', str(self.maximum_age)]
            self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, pass_fds=(child.fileno(),),
                env=_ENV, start_new_session=True, umask=0o077, bufsize=0)
            self.started_at = self.progress_at = time.monotonic()
            self.thread = threading.Thread(target=self._monitor, name='neo2-hevc-observations', daemon=True)
            self.thread.start()
            return self
        except BaseException:
            self.invalidate('worker_start_failed')
            self._reap()
            self.closed = True
            raise
        finally:
            if child is not None:
                child.close()

    def invalidate(self, reason='worker_session_invalidated'):
        with self.lock:
            self.error = self.error or reason
            self.latest.invalidate(self.error)
            self.stop.set()

    def feed(self, data, *, delivered_at, received_at=None):
        # No decoding, hashing, blocking socket write or process wait here.
        with self.lock:
            if not self.started or self.closed or self.error or self.input_finished:
                return False
            now = time.monotonic()
            received = delivered_at if received_at is None else received_at
            if (not isinstance(data, bytes) or len(data) > MAX_INPUT
                    or not _finite(delivered_at) or not _finite(received)
                    or not 0 <= received <= delivered_at <= now
                    or (self.last_delivery is not None and delivered_at < self.last_delivery)
                    or (self.last_clock is not None and now < self.last_clock)):
                self.invalidate('worker_input_or_clock_invalid')
                return False
            if not data:
                return True
            packet = INPUT_HEADER.pack(self.submitted_chunks, delivered_at, received, int(received_at is not None)) + data
            try:
                sent = self.ingress.send(packet, socket.MSG_DONTWAIT)
                if sent != len(packet):
                    raise OSError('partial seqpacket write')
            except BlockingIOError:
                self.invalidate('worker_input_queue_full')
                return False
            except OSError:
                self.invalidate('worker_input_channel_failed')
                return False
            if self.last_ack == self.submitted_chunks - 1:
                self.progress_at = now
            self.last_clock, self.last_delivery = now, delivered_at
            self.submitted_chunks += 1
            self.submitted_bytes += len(data)
            return True

    def finish_input(self):
        """Explicit offline drain; never use as a session-loss notification."""
        with self.lock:
            if not self.started or self.closed or self.error or self.input_finished:
                return False
            try:
                packet = INPUT_HEADER.pack(self.submitted_chunks, 0., 0., 2)
                if self.ingress.send(packet, socket.MSG_DONTWAIT) != len(packet):
                    raise OSError('partial finish packet')
            except (BlockingIOError, OSError):
                self.invalidate('worker_finish_channel_failed')
                return False
            self.input_finished = True
            if self.last_ack == self.submitted_chunks - 1:
                self.progress_at = time.monotonic()
            return True

    def _frame(self, record, rgb, *, stale=False):
        if not self.ready or self.finished:
            raise ValueError('worker_frame_outside_stream')
        pixels = record['pixels']
        if (pixels['format'] != 'rgb24' or type(pixels['width']) is not int
                or type(pixels['height']) is not int
                or not 2 <= pixels['width'] <= 1920 or not 2 <= pixels['height'] <= 1080
                or (not stale and (len(rgb) != pixels['width']*pixels['height']*3
                    or hashlib.sha256(rgb).hexdigest() != pixels['sha256']))
                or (stale and rgb)
                or type(record['raw_picture_ordinal']) is not int or record['raw_picture_ordinal'] < 0
                or type(record['decoded_index']) is not int
                or type(record['original_packet_receipt_known']) is not bool
                or any(record[k] is not False for k in ('clock_unit_verified',
                    'exposure_semantics_verified', 'pc_clock_alignment_verified', 'flight_authorized'))):
            raise ValueError('worker_frame_pixels_or_proof_invalid')
        keys = ('owner_epoch', 'connection_generation', 'decoded_index', 'raw_picture_ordinal',
            'pc_first_byte_delivered_at', 'pc_first_byte_received_at',
            'original_packet_receipt_known', 'decoded_at')
        sample = DecodedObservation(*(record[k] for k in keys), pixels['width'], pixels['height'],
            rgb, pixels['sha256'], {k:v for k,v in record.items() if k not in (*keys, 'pixels')})
        now = time.monotonic()
        if ((sample.owner_epoch,sample.connection_generation) != self.context
                or not all(_finite(t) for t in (sample.pc_first_byte_received_at,
                    sample.pc_first_byte_delivered_at,sample.decoded_at))
                or not 0 <= sample.pc_first_byte_received_at <= sample.pc_first_byte_delivered_at <= sample.decoded_at <= now
                or (stale and now-sample.pc_first_byte_received_at <= self.maximum_age)):
            raise ValueError('worker_frame_identity_or_clock_invalid')
        proof = provenance_bytes(record)
        with self.lock:
            if self.error:
                return
            if sample.decoded_index != self.received_frames:
                raise ValueError('worker_frame_order_invalid')
            if stale:
                self.latest.discard()
                self.stale_without_rgb += 1
            else:
                self.latest.publish(sample)
            self.provenance.update(proof)
            self.received_frames += 1

    def _message(self, raw, rgb):
        def invalid_constant(_):raise ValueError('worker_nonfinite_metadata')
        record = json.loads(raw, parse_constant=invalid_constant)
        kind = record['type']
        if kind in ('frame','stale'):
            self._frame(record['observation'], rgb, stale=kind=='stale')
        else:
            if rgb:
                raise ValueError('worker_unexpected_pixels')
            with self.lock:
                if self.error:
                    return
                if kind == 'ready' and not self.ready and not self.finished:
                    self.ready = True
                elif kind == 'consumed' and self.ready and not self.finished:
                    number = record['sequence']
                    if type(number) is not int or number != self.last_ack + 1 or number >= self.submitted_chunks:
                        raise ValueError('worker_ack_sequence_invalid')
                    self.last_ack = number
                elif kind == 'finished' and self.ready and self.input_finished and not self.finished:
                    summary = record['summary']
                    if (record['consumed_chunks'] != self.submitted_chunks
                            or self.last_ack != self.submitted_chunks - 1
                            or summary['decoded_frames'] != self.received_frames
                            or summary['error'] is not None or summary['finished'] is not True):
                        raise ValueError('worker_finish_summary_invalid')
                    self.decoder_summary = summary
                    self.finished = True
                    self.latest.invalidate('worker_input_finished')
                else:
                    raise ValueError('worker_message_order_invalid')
        with self.lock:
            self.progress_at = time.monotonic()

    def _monitor(self):
        buffer = bytearray()
        try:
            with selectors.DefaultSelector() as poll:
                for pipe in (self.process.stdout, self.process.stderr):
                    os.set_blocking(pipe.fileno(), False)
                    poll.register(pipe, selectors.EVENT_READ)
                while poll.get_map() or self.process.poll() is None:
                    with self.lock:
                        if self.stop.is_set():
                            break
                        now = time.monotonic()
                        if not self.ready and now - self.started_at > self.startup_timeout:
                            raise ValueError('worker_startup_timeout')
                        pending = self.last_ack != self.submitted_chunks - 1 or self.input_finished
                        if self.ready and not self.finished and pending and now - self.progress_at > self.stall_timeout:
                            raise ValueError('worker_stall_timeout')
                    for key, _ in poll.select(.025):
                        data = os.read(key.fileobj.fileno(), 65536)
                        if not data:
                            poll.unregister(key.fileobj)
                        elif key.fileobj is self.process.stderr:
                            self.stderr_bytes += len(data)
                            self.stderr.extend(data)
                            if len(self.stderr) > MAX_LOG:del self.stderr[:-MAX_LOG]
                        else:
                            buffer.extend(data)
                            while len(buffer) >= OUTPUT_HEADER.size:
                                meta, size = OUTPUT_HEADER.unpack_from(buffer)
                                if not 0 < meta <= MAX_METADATA or size > MAX_RGB:
                                    raise ValueError('worker_output_header_invalid')
                                end = OUTPUT_HEADER.size + meta + size
                                if len(buffer) < end:break
                                raw = bytes(buffer[OUTPUT_HEADER.size:OUTPUT_HEADER.size+meta])
                                rgb = bytes(buffer[OUTPUT_HEADER.size+meta:end])
                                del buffer[:end]
                                self._message(raw, rgb)
                if not self.stop.is_set() and not self.error and (buffer or not self.finished or self.process.poll() != 0):
                    raise ValueError('worker_exit_without_complete_stream')
        except Exception as error:
            self.invalidate(str(error)[:256])
        finally:
            buffer.clear()
            try:
                self._reap()
            except Exception:
                self.invalidate('worker_cleanup_unconfirmed')

    def _reap(self):
        if self.process is not None:
            if self.process.poll() is None:
                try:os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:pass
            try:self.process.wait(timeout=1.)
            except subprocess.TimeoutExpired:
                raise VideoError('worker_cleanup_unconfirmed', cleanup_verified=False) from None
            for pipe in (self.process.stdout, self.process.stderr):
                if pipe is not None:pipe.close()
        if self.ingress is not None:
            self.ingress.close()

    def wait(self, timeout=5.):
        if not _finite(timeout) or not 0 < timeout <= 600:
            raise ValueError('worker_wait_timeout_invalid')
        if self.thread is not None:
            self.thread.join(timeout)
            if self.thread.is_alive():
                self.invalidate('worker_wait_timeout')
                self.thread.join(1.5)
            if self.thread.is_alive():
                raise VideoError('worker_monitor_did_not_stop', cleanup_verified=False)
        return self.status()

    def close(self):
        if not self.closed:
            with self.lock:
                self.latest.invalidate('worker_closed')
                self.stop.set()
            self.wait(2.)
            self.closed = True
        return self.status()

    def current(self):
        """Recheck process ownership as well as original PC input age."""
        with self.lock:
            if self.error or self.closed or self.finished or not self.ready:
                return None
            if self.process.poll() is not None:
                self.invalidate('worker_unexpected_exit')
                return None
            return self.latest.peek()

    def status(self):
        with self.lock:
            alive = self.process is not None and self.process.poll() is None
            monitor_alive = bool(self.thread and self.thread.is_alive())
            descriptors_closed = ((self.ingress is None or self.ingress.fileno() < 0)
                and (self.process is None or all(p is None or p.closed
                    for p in (self.process.stdout,self.process.stderr))))
            if self.started and not alive and not monitor_alive and not self.finished and not self.error and not self.stop.is_set():
                self.invalidate('worker_unexpected_exit')
            return {'ready':self.ready, 'input_finished':self.input_finished, 'finished':self.finished,
                'closed':self.closed, 'error':self.error, 'worker_pid':self.process.pid if self.process else None,
                'worker_alive':alive, 'monitor_alive':monitor_alive,
                'cleanup_verified':self.started and not alive and not monitor_alive and descriptors_closed,
                'descriptors_closed':descriptors_closed,
                'input_kernel_buffer_bytes':self.input_kernel_buffer_bytes,
                'submitted_chunks':self.submitted_chunks, 'submitted_bytes':self.submitted_bytes,
                'consumed_chunks':self.last_ack+1, 'received_frames':self.received_frames,
                'stale_without_rgb':self.stale_without_rgb,
                'pixel_verified_frames':self.received_frames-self.stale_without_rgb,
                'provenance_sha256':self.provenance.hexdigest(), 'decoder_summary':self.decoder_summary,
                'stderr_bytes':self.stderr_bytes, 'stderr_tail':bytes(self.stderr).decode('utf-8','replace'),
                'latest':self.latest.status(), 'live_transport_integrated':False,
                'exposure_age_bound_verified':False, 'flight_authorized':False}

    def __enter__(self):return self.start()
    def __exit__(self, *args):self.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description='Internal owned decoder worker')
    parser.add_argument('--input-fd', type=int, required=True)
    parser.add_argument('--owner', required=True)
    parser.add_argument('--generation', required=True)
    parser.add_argument('--fps', type=float, required=True)
    parser.add_argument('--maximum-age', type=float, required=True)
    args = parser.parse_args(argv)
    return _child(args)


if __name__ == '__main__':
    raise SystemExit(main())
