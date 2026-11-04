"""Real owned-process decoding, stopped consumers, queue pressure and cleanup."""
import hashlib
import json
import os
from pathlib import Path
import signal
import threading
import time

import pytest

from neo2_control.video_lineage import build_video_lineage
from neo2_control.video_worker import ManagedHevcDecoder, provenance_bytes
from test_video_lineage import encoded
from tools.replay_video_worker import replay


OWNER, GENERATION = 'a'*48, 'b'*24


def until(worker, condition, timeout=3.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        status = worker.status()
        if condition(status):return status
        assert status['error'] is None, status
        time.sleep(.005)
    pytest.fail(f'worker condition timed out: {worker.status()}')


@pytest.fixture
def worker():
    pytest.importorskip('av')
    consumer = ManagedHevcDecoder(OWNER, GENERATION,input_buffer_bytes=128*1024).start()
    try:
        until(consumer, lambda s:s['ready'])
        yield consumer
    finally:
        consumer.close()
        assert not consumer.status()['worker_alive']
        assert not consumer.status()['monitor_alive']


def test_owned_decoder_preserves_all_b_picture_provenance_and_enforces_native_limits(encoded, worker, tmp_path):
    source, _ = encoded
    reference = build_video_lineage(source, tmp_path/'reference', timeout=30)
    digest = hashlib.sha256()
    for frame in reference['frames']:
        record = {**frame, 'source_vcl_body_sha256':frame['packet']['source_vcl_body_sha256'],
            'normalized_packet_sha256':frame['packet']['sha256']}
        digest.update(provenance_bytes(record))
    limits = Path(f'/proc/{worker.process.pid}/limits').read_text().splitlines()
    assert any(line.startswith('Max address space') and '1610612736' in line for line in limits)
    assert any(line.startswith('Max open files') and '64' in line for line in limits)
    now = time.monotonic()
    raw = source.read_bytes()
    for offset in range(0,len(raw),1452):
        assert worker.feed(raw[offset:offset+1452],delivered_at=now,received_at=now-.01)
    assert worker.finish_input()
    status = worker.wait(5.)
    assert status['error'] is None and status['finished']
    assert status['received_frames'] == 9
    assert status['provenance_sha256'] == digest.hexdigest()
    assert status['decoder_summary']['peak_pending_pictures'] <= 5
    assert status['submitted_chunks'] == status['consumed_chunks']
    assert status['cleanup_verified']
    assert status['descriptors_closed']
    assert worker.latest.peek() is None
    assert not worker.feed(b'x',delivered_at=time.monotonic())
    assert not status['flight_authorized'] and not status['live_transport_integrated']


def test_live_observation_keeps_original_receipt_and_expires_without_new_packets(encoded, worker):
    source, _ = encoded
    now = time.monotonic()
    assert worker.feed(source.read_bytes(),delivered_at=now,received_at=now-.02)
    until(worker,lambda s:s['consumed_chunks']==1)
    sample = worker.current()
    assert sample is not None
    assert sample.pc_first_byte_received_at == now-.02
    assert sample.pc_first_byte_delivered_at == now
    assert sample.owner_epoch == OWNER and sample.connection_generation == GENERATION
    assert hashlib.sha256(sample.rgb).hexdigest() == sample.rgb_sha256
    time.sleep(.36)
    assert worker.current() is None
    assert worker.status()['latest']['retained_pixel_bytes'] == 0
    assert worker.status()['worker_alive']


def test_queue_full_returns_while_native_child_is_stopped_and_clears_pixels(encoded, worker):
    source, _ = encoded
    now = time.monotonic()
    assert worker.feed(source.read_bytes(),delivered_at=now)
    until(worker,lambda s:s['consumed_chunks']==1)
    assert worker.latest.peek() is not None
    os.kill(worker.process.pid,signal.SIGSTOP)
    outcome = []
    def producer():
        for _ in range(100):
            if not worker.feed(b'x'*65536,delivered_at=time.monotonic()):
                outcome.append(worker.status()['error'])
                return
    producer_thread = threading.Thread(target=producer,daemon=True)
    producer_thread.start()
    producer_thread.join(.5)
    try:
        assert not producer_thread.is_alive(), 'producer waited for stopped native decoder'
        assert outcome == ['worker_input_queue_full']
        assert worker.latest.peek() is None
        assert worker.status()['latest']['retained_pixel_bytes'] == 0
        assert worker.submitted_chunks < 100
        assert not worker.feed(b'x',delivered_at=time.monotonic())
    finally:
        worker.close()
        producer_thread.join(1.)
    assert worker.status()['cleanup_verified']


def test_watchdog_reaps_stopped_native_decoder_with_pending_input(worker):
    worker.stall_timeout = .1
    os.kill(worker.process.pid,signal.SIGSTOP)
    assert worker.feed(b'\0',delivered_at=time.monotonic())
    status = worker.wait(2.)
    assert status['error'] == 'worker_stall_timeout'
    assert status['cleanup_verified']
    assert worker.latest.peek() is None


def test_startup_watchdog_reaps_child_that_never_initializes():
    pytest.importorskip('av')
    consumer = ManagedHevcDecoder(OWNER,GENERATION,startup_timeout=.1).start()
    try:
        os.kill(consumer.process.pid,signal.SIGSTOP)
        status = consumer.wait(2.)
        assert status['error'] == 'worker_startup_timeout'
        assert status['cleanup_verified']
    finally:consumer.close()


def test_malformed_hevc_fails_and_cannot_reuse_consumer(worker):
    assert worker.feed(b'not Annex B',delivered_at=time.monotonic())
    status = worker.wait(2.)
    assert status['error'] == 'worker_exit_without_complete_stream'
    assert 'HEVC Annex-B' in status['stderr_tail']
    assert status['cleanup_verified']
    assert worker.latest.peek() is None
    with pytest.raises(ValueError,match='cannot_be_reused'):worker.start()


def test_session_invalidation_during_pending_native_work_removes_observation(encoded, worker):
    source, _ = encoded
    assert worker.feed(source.read_bytes(),delivered_at=time.monotonic())
    until(worker,lambda s:s['received_frames']>0)
    worker.invalidate('session_generation_changed')
    assert worker.latest.peek() is None
    status = worker.wait(2.)
    assert status['error'] == 'session_generation_changed'
    assert status['cleanup_verified']


@pytest.mark.parametrize('receipt', ['future','backwards','too_large'])
def test_invalid_input_is_terminal_and_does_not_enter_child(worker,receipt):
    now = time.monotonic()
    if receipt == 'future':data,stamp = b'x',now+100
    elif receipt == 'too_large':data,stamp = b'x'*65537,now
    else:
        assert worker.feed(b'\0',delivered_at=now)
        data,stamp = b'\0',now-1
    assert not worker.feed(data,delivered_at=stamp)
    assert worker.latest.peek() is None
    status = worker.wait(2.)
    assert status['error'] == 'worker_input_or_clock_invalid'
    assert status['cleanup_verified']


def test_finish_without_start_and_close_without_start_are_safe():
    consumer = ManagedHevcDecoder(OWNER,GENERATION)
    assert not consumer.feed(b'x',delivered_at=time.monotonic())
    assert not consumer.finish_input()
    status = consumer.close()
    assert status['closed'] and status['error'] is None
    with pytest.raises(ValueError,match='cannot_be_reused'):consumer.start()


def test_terminated_native_process_cannot_leave_a_current_observation(encoded, worker):
    source, _ = encoded
    assert worker.feed(source.read_bytes(),delivered_at=time.monotonic())
    until(worker,lambda s:s['consumed_chunks']==1)
    assert worker.current() is not None
    os.kill(worker.process.pid,signal.SIGKILL)
    worker.process.wait(timeout=1.)
    assert worker.current() is None
    status = worker.wait(2.)
    assert status['error'] and status['cleanup_verified']


def test_old_source_is_stale_even_when_child_and_pipe_just_produced_pixels(encoded, worker):
    source, _ = encoded
    assert worker.feed(source.read_bytes(),delivered_at=time.monotonic()-1)
    until(worker,lambda s:s['consumed_chunks']==1)
    assert worker.status()['received_frames'] > 0
    assert worker.current() is None
    assert worker.status()['stale_without_rgb'] > 0
    assert worker.status()['pixel_verified_frames'] == 0


def test_explicit_close_reaps_idle_child_without_reporting_decode_failure(worker):
    status = worker.close()
    assert status['closed'] and status['cleanup_verified']
    assert status['error'] is None
    assert worker.current() is None
    assert worker.close() == status


def test_worker_replay_uses_owned_process_and_exact_complete_proof(encoded,tmp_path,capsys):
    source, _ = encoded
    build_video_lineage(source,tmp_path/'reference',timeout=30)
    size = source.stat().st_size
    rows = [{'stream_offset_start':start,'stream_offset_end':min(start+1452,size),
        'original_packet_monotonic_ns':100_000_000_000,
        'release_trigger_monotonic_ns':100_000_000_000} for start in range(0,size,1452)]
    receipts = tmp_path/'receipts.json';receipts.write_text(json.dumps(rows))
    output = tmp_path/'result.json'
    assert replay(source,tmp_path/'reference/manifest.json',receipts,output) == 0
    result = json.loads(output.read_text())
    assert result['complete_and_identical'] and result['decoded_prefix_identical']
    assert result['worker']['received_frames'] == 9
    assert result['worker']['cleanup_verified']
    assert not result['hardware_used'] and not result['flight_authorized']
    assert output.stat().st_mode & 0o777 == 0o600
    original = output.read_bytes()
    with pytest.raises(ValueError,match='output_must_be_new'):
        replay(source,tmp_path/'reference/manifest.json',receipts,output)
    assert original == output.read_bytes()
    assert json.loads(capsys.readouterr().out)['complete_and_identical']
