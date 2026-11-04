#!/usr/bin/env python3
"""Replay a recorded source through the owned decoder at recorded release times.

The capture clocks are translated onto the current PC monotonic clock. The
source is read locally; this does not connect to a radio or measure exposure.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import time

from neo2_control.video_export import _stamp
from neo2_control.video_lineage import _json_new
from neo2_control.video_worker import ManagedHevcDecoder, provenance_bytes
if __package__:
    from .replay_video_stream import read_json, require
else:
    from replay_video_stream import read_json, require


def replay(source, reference, receipts, output, *, input_buffer_bytes=4*1024*1024):
    output = Path(output)
    require(not output.exists() and not output.is_symlink(), 'worker_replay_output_must_be_new')
    expected, reference_hash = read_json(reference,64*1024*1024)
    rows, receipts_hash = read_json(receipts,32*1024*1024)
    require(isinstance(rows,list) and 0 < len(rows) <= 400_000, 'worker_replay_receipts_invalid')
    require(isinstance(expected['frames'],list) and len(expected['frames']) <= 60_000, 'worker_replay_reference_invalid')
    digest = hashlib.sha256()
    proofs = [digest.hexdigest()]
    for frame in expected['frames']:
        record = {**frame,'source_vcl_body_sha256':frame['packet']['source_vcl_body_sha256'],
            'normalized_packet_sha256':frame['packet']['sha256']}
        digest.update(provenance_bytes(record))
        proofs.append(digest.hexdigest())
    expected_proof = digest.hexdigest()
    source_digest = hashlib.sha256()
    feed_durations = []
    offset = 0
    started = time.monotonic()
    worker = ManagedHevcDecoder('0'*48,'1'*24,input_buffer_bytes=input_buffer_bytes).start()
    failure = None
    try:
        deadline = time.monotonic()+10
        while not worker.status()['ready']:
            require(not worker.status()['error'] and time.monotonic() < deadline,'worker_replay_start_failed')
            time.sleep(.005)
        anchor = time.monotonic()
        origin = rows[0]['release_trigger_monotonic_ns']/1e9
        require(0 <= rows[-1]['release_trigger_monotonic_ns']/1e9-origin <= 600,'worker_replay_duration_invalid')
        fd = os.open(source,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
        with os.fdopen(fd,'rb') as stream:
            initial = os.fstat(stream.fileno())
            require(stat.S_ISREG(initial.st_mode) and 0 < initial.st_size <= 512*1024*1024,'worker_replay_source_invalid')
            for row in rows:
                require(row['stream_offset_start']==offset,'worker_replay_source_gap')
                size = row['stream_offset_end']-offset
                require(0 < size <= 65536,'worker_replay_chunk_size_invalid')
                raw = stream.read(size)
                require(len(raw)==size,'worker_replay_short_source')
                delivered = anchor+(row['release_trigger_monotonic_ns']/1e9-origin)
                received = anchor+(row['original_packet_monotonic_ns']/1e9-origin)
                while time.monotonic() < delivered:
                    time.sleep(max(0.,min(.02,delivered-time.monotonic())))
                before = time.monotonic()
                accepted = worker.feed(raw,delivered_at=delivered,received_at=received)
                feed_durations.append(time.monotonic()-before)
                if not accepted:
                    failure = worker.status()['error'] or 'worker_replay_feed_refused'
                    break
                source_digest.update(raw)
                offset += size
            require(_stamp(initial)==_stamp(os.fstat(stream.fileno())),'worker_replay_source_changed')
        if failure is None:
            require(offset==initial.st_size and source_digest.hexdigest()==expected['source']['sha256'],'worker_replay_source_digest_differ')
            require(worker.finish_input(),'worker_replay_finish_refused')
            status = worker.wait(10.)
            failure = status['error']
        else:
            status = worker.wait(3.)
        complete = (failure is None and status['finished']
            and status['received_frames']==len(expected['frames'])
            and status['provenance_sha256']==expected_proof)
        prefix_identical = (status['received_frames'] < len(proofs)
            and status['provenance_sha256']==proofs[status['received_frames']])
        if failure is None and not complete:
            failure = 'worker_replay_frame_identity_differ'
        require(not complete or status['cleanup_verified'],'worker_replay_cleanup_unconfirmed')
        result = {'scope':'Frozen local capture paced by recorded ordered release; clocks translated onto PC monotonic time. No radio or exposure proof.',
            'source':expected['source'],'reference_sha256':reference_hash,'receipts_sha256':receipts_hash,
            'expected_frames':len(expected['frames']),'expected_provenance_sha256':expected_proof,
            'complete_and_identical':complete,'decoded_prefix_identical':prefix_identical,
            'failure':failure,'consumed_source_bytes':offset,
            'submitted_source_sha256':source_digest.hexdigest(), 'input_buffer_requested_bytes':input_buffer_bytes,
            'feed_calls':len(feed_durations),'maximum_feed_call_seconds':max(feed_durations,default=0),
            'worker':status,'wall_seconds':time.monotonic()-started,'hardware_used':False,'flight_authorized':False}
        _json_new(output,result)
        print(json.dumps(result))
        return 0 if complete else 2
    finally:worker.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source','reference','receipts','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--input-buffer-bytes',type=int,default=4*1024*1024)
    args = parser.parse_args(argv)
    return replay(args.source,args.reference,args.receipts,args.output,input_buffer_bytes=args.input_buffer_bytes)


if __name__ == '__main__':
    raise SystemExit(main())
