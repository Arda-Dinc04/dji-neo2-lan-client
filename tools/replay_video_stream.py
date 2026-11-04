#!/usr/bin/env python3
"""Verify incremental observations against an offline lineage manifest.

Native decoding runs in an owned bounded child. A compressed virtual scheduler
can include actual worker busy time while skipping idle gaps in the recording.
This is a recording replay, not a live device or exposure-clock measurement.
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import sys
import time

from neo2_control.video_stream import IncrementalHevcDecoder
from neo2_control.video_lineage import _json_new
from neo2_control.video_export import _parent, _path, _run, _limited, _stamp


def require(ok, reason):
    if not ok:raise ValueError(reason)


def read_json(path, maximum):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as stream:
        initial = os.fstat(stream.fileno())
        require(stat.S_ISREG(initial.st_mode) and 0 < initial.st_size <= maximum, 'replay_metadata_size_invalid')
        raw = stream.read(maximum + 1)
        require(len(raw) == initial.st_size and _stamp(initial) == _stamp(os.fstat(stream.fileno())), 'replay_metadata_changed')
    return json.loads(raw), hashlib.sha256(raw).hexdigest()


class ReplayClock:
    def __init__(self, processing):
        self.processing = processing
        self.base = 0.
        self.anchor = None

    def __call__(self):
        return self.base + (time.monotonic() - self.anchor if self.anchor is not None else 0.)

    def deliver(self, at):
        self.base = max(at, self())
        self.anchor = time.monotonic() if self.processing else None

    def idle(self, seconds):
        self.base = self() + seconds
        self.anchor = None


def replay(args):
    import av
    reference, reference_hash = read_json(args.reference, 64*1024*1024)
    expected = reference['frames']
    require(isinstance(expected, list) and len(expected) <= 60_000, 'replay_reference_frame_limit')
    by_picture = {f['raw_picture_ordinal']: f for f in expected}
    require(len(by_picture) == len(expected), 'replay_reference_duplicate_picture')
    receipts = receipts_hash = starts = None
    if args.receipts:
        receipts, receipts_hash = read_json(args.receipts, 32*1024*1024)
        require(isinstance(receipts, list) and 0 < len(receipts) <= 400_000, 'replay_receipt_count_invalid')
        starts = [r['stream_offset_start'] for r in receipts]
    clock = ReplayClock(args.time_mode == 'processing')
    rows = []
    max_age = 0.

    def receive(sample):
        nonlocal max_age
        record = sample.record()
        old = by_picture[sample.raw_picture_ordinal]
        require(sample.rgb_sha256 == old['pixels']['sha256'], 'replay_decoded_pixels_differ')
        require(record['candidate_fields'] == old['candidate_fields'], 'replay_source_fields_differ')
        require(record['encoded_interval_checksum'] == old['encoded_interval_checksum'], 'replay_source_checksum_differ')
        require(record['source_vcl_body_sha256'] == old['packet']['source_vcl_body_sha256'], 'replay_vcl_bytes_differ')
        require(record['normalized_packet_sha256'] == old['packet']['sha256'], 'replay_normalized_packet_bytes_differ')
        require(len(rows) == old['decoded_index'], 'replay_decoded_order_differ')
        if receipts:
            original = receipts[bisect.bisect_right(starts, record['source_first_slice_byte_offset']) - 1]
            require(sample.pc_first_byte_received_at == original['original_packet_monotonic_ns']/1e9, 'replay_original_receipt_differ')
            require(sample.pc_first_byte_delivered_at == original['release_trigger_monotonic_ns']/1e9, 'replay_ordered_delivery_differ')
        age = sample.decoded_at - sample.pc_first_byte_received_at
        max_age = max(max_age, age)
        record['pc_input_age_at_decode_seconds'] = age
        rows.append(record)

    started = time.monotonic()
    decoder = IncrementalHevcDecoder('0'*48, '1'*24, clock=clock, on_frame=receive,
        decode_thread_type=args.thread_type)
    digest = hashlib.sha256()
    offset = 0
    fd = os.open(args.source, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as stream:
        initial = os.fstat(stream.fileno())
        require(stat.S_ISREG(initial.st_mode) and 0 < initial.st_size <= 512*1024*1024, 'replay_source_size_invalid')
        if receipts:
            for row in receipts:
                require(row['stream_offset_start'] == offset, 'replay_receipt_offset_gap')
                size = row['stream_offset_end'] - offset
                require(0 < size <= 65536, 'replay_receipt_chunk_size_invalid')
                raw = stream.read(size)
                require(len(raw) == size, 'replay_source_short_read')
                digest.update(raw)
                delivered = row['release_trigger_monotonic_ns']/1e9
                clock.deliver(delivered)
                decoder.feed(raw, delivered_at=delivered, received_at=row['original_packet_monotonic_ns']/1e9)
                offset += size
        else:
            while True:
                raw = stream.read(args.chunk)
                if not raw:break
                digest.update(raw)
                offset += len(raw)
                # This fallback rate is synthetic and is labelled as such.
                delivered = (offset//args.chunk + 1)*.001
                clock.deliver(delivered)
                decoder.feed(raw, delivered_at=delivered)
        decoder.finish()
        require(_stamp(initial) == _stamp(os.fstat(stream.fileno())), 'replay_source_changed')
    require(offset == initial.st_size and digest.hexdigest() == reference['source']['sha256'], 'replay_source_digest_differ')
    require(len(rows) == len(expected), 'replay_missing_decoded_frames')
    status = decoder.status()
    last = decoder.latest.peek()
    clock.idle(.351)
    require(decoder.latest.peek() is None, 'replay_old_frame_did_not_expire')
    result = {'scope': 'Frozen byte replay; virtual scheduler with real worker busy time when requested; idle capture gaps compressed. Not live hardware or exposure timing.',
        'context_origin': 'offline_replay', 'source': reference['source'], 'reference_sha256': reference_hash,
        'receipts_sha256': receipts_hash, 'receipt_times': 'historical' if receipts else 'synthetic_1ms_chunks',
        'time_mode': args.time_mode, 'pyav': av.__version__, 'libraries': av.library_versions,
        'summary': status, 'frames': rows, 'latest_after_idle': decoder.latest.status(),
        'max_pc_input_age_at_decode_seconds': max_age,
        'last_raw_picture': last.raw_picture_ordinal if last else None,
        'wall_seconds': time.monotonic() - started, 'hardware_used': False, 'flight_authorized': False}
    _json_new(args.output, result)
    print(json.dumps({k: v for k, v in result.items() if k not in ('frames', 'libraries')}))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--receipts', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--chunk', type=int, default=1452)
    parser.add_argument('--time-mode', choices=('historical', 'processing'), default='processing')
    parser.add_argument('--thread-type', choices=('FRAME', 'SLICE'), default='FRAME')
    parser.add_argument('--timeout', type=float, default=300.)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    require(1 <= args.chunk <= 65536 and 0 < args.timeout <= 600, 'replay_options_invalid')
    if args.worker:
        replay(args)
        return 0
    output = _path(args.output)
    require(not output.exists() and not output.is_symlink(), 'replay_output_must_be_new')
    limiter = shutil.which('prlimit')
    require(limiter is not None, 'replay_requires_prlimit')
    options = ['--source', str(_path(args.source)), '--reference', str(_path(args.reference)),
        '--output', output.name, '--chunk', str(args.chunk), '--time-mode', args.time_mode,
        '--thread-type', args.thread_type]
    if args.receipts:options.extend(['--receipts', str(_path(args.receipts))])
    parent = _parent(output, create=True)
    try:
        raw, _ = _run(_limited(limiter, sys.executable, maximum=64*1024*1024) +
            [str(Path(__file__).absolute()), '--worker', *options], directory_fd=parent,
            deadline=time.monotonic() + args.timeout, stage='stream_replay')
        print(raw.decode(), end='')
    finally:
        os.close(parent)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
