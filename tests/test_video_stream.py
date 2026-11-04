"""Chunking, original PC ages, B-picture identity and bounded latest storage."""
from dataclasses import FrozenInstanceError, replace
import hashlib

import pytest

from neo2_control import video_stream
from neo2_control.video_clock import inventory_hevc_clocks
from neo2_control.video_stream import DecodedObservation, LatestDecodedFrame, IncrementalHevcDecoder
from test_video_lineage import encoded, nal


OWNER, GENERATION = 'a'*48, 'b'*24


@pytest.fixture
def av_available():pytest.importorskip('av')


class Clock:
    now = 0.
    def __call__(self):return self.now


def sample(index=0, first=0., decoded=0., *, generation=GENERATION):
    pixels = bytes(range(12))
    return DecodedObservation(OWNER, generation, index, index, first, first,
        False, decoded, 2, 2, pixels, hashlib.sha256(pixels).hexdigest(), {})


def test_latest_slot_replaces_without_queue_and_expires_when_consumer_stops_polling():
    clock = Clock();slot = LatestDecodedFrame(OWNER, GENERATION, clock=clock)
    for i in range(100):
        clock.now = i*.001
        assert slot.publish(sample(i, clock.now, clock.now))
    assert slot.peek().decoded_index == 99
    status = slot.status()
    assert status['retained_frames'] == 1 and status['retained_pixel_bytes'] == 12
    assert status['overwritten'] == 99
    clock.now += .351
    assert slot.peek() is None
    assert slot.status()['retained_pixel_bytes'] == 0
    assert slot.expired == 1


def test_new_decode_time_does_not_freshen_old_input_or_mutated_metadata():
    clock = Clock();slot = LatestDecodedFrame(OWNER, GENERATION, clock=clock)
    clock.now = 1.
    old = sample(0, first=0., decoded=1.)
    old.details.update(pc_first_byte_received_at=1., flight_authorized=True)
    assert not slot.publish(old)
    assert slot.peek() is None
    assert slot.stale == 1
    record = old.record()
    assert record['pc_first_byte_received_at'] == 0.
    assert not record['flight_authorized']
    with pytest.raises(FrozenInstanceError):old.pc_first_byte_received_at = 1.


@pytest.mark.parametrize('problem', ['generation', 'duplicate', 'clock'])
def test_context_or_clock_change_invalidates_and_cannot_reexpose_previous_frame(problem):
    clock = Clock();slot = LatestDecodedFrame(OWNER, GENERATION, clock=clock)
    clock.now = 1.
    assert slot.publish(sample(0, 1., 1.))
    if problem == 'generation':
        action = lambda:slot.publish(sample(1, 1., 1., generation='c'*24))
    elif problem == 'duplicate':action = lambda:slot.publish(sample(0, 1., 1.))
    else:
        clock.now = .9
        action = slot.peek
    with pytest.raises(ValueError):action()
    assert slot.peek() is None
    clock.now = 2.
    assert not slot.publish(sample(2, 2., 2.))


@pytest.mark.parametrize('chunk_size', [1, 2, 7, 1452, 65536])
@pytest.mark.parametrize('thread_type', ['FRAME', 'SLICE'])
def test_incremental_b_pictures_keep_source_counters_checksums_and_exact_pixels(encoded, chunk_size, thread_type):
    path, expected = encoded
    samples = []
    decoder = IncrementalHevcDecoder(OWNER, GENERATION, clock=lambda:0., on_frame=samples.append,
        decode_thread_type=thread_type)
    raw = path.read_bytes()
    for i in range(0, len(raw), chunk_size):
        decoder.feed(raw[i:i+chunk_size], delivered_at=0.)
    decoder.finish()
    assert [s.rgb_sha256 for s in samples] == expected
    assert [s.raw_picture_ordinal for s in samples] != list(range(9))
    assert sorted(s.raw_picture_ordinal for s in samples) == list(range(9))
    original = inventory_hevc_clocks(path)['pictures']
    for s in samples:
        record = s.record();p = original[s.raw_picture_ordinal]
        assert record['candidate_fields'] == p['candidate_fields']
        assert record['encoded_interval_checksum'] == p['encoded_interval_checksum']
        assert not record['exposure_semantics_verified'] and not record['flight_authorized']
        assert hashlib.sha256(s.rgb).hexdigest() == s.rgb_sha256
    status = decoder.status()
    assert status['decoded_frames'] == 9 and status['peak_pending_pictures'] <= 5
    assert status['decoder_thread_type'] == thread_type
    assert status['receipt_spans'] == status['pending_pictures'] == status['buffered_nal_bytes'] == 0
    assert status['latest']['retained_frames'] == 1
    assert status['latest']['retained_pixel_bytes'] == 128*96*3


def test_old_split_vcl_expires_even_when_final_bytes_and_decode_are_recent(encoded):
    path, _ = encoded;raw = path.read_bytes()
    first = inventory_hevc_clocks(path)['pictures'][0]
    split = first['first_slice_byte_offset'] + 12
    clock = Clock();samples = []
    decoder = IncrementalHevcDecoder(OWNER, GENERATION, clock=clock, on_frame=samples.append)
    decoder.feed(raw[:split], delivered_at=0.)
    assert not samples
    clock.now = .6
    decoder.feed(raw[split:], delivered_at=.6)
    decoder.finish()
    old = next(s for s in samples if s.raw_picture_ordinal == 0)
    assert old.pc_first_byte_delivered_at == old.pc_first_byte_received_at == 0.
    assert old.decoded_at == .6
    assert decoder.latest.stale >= 1
    assert decoder.latest.peek().pc_first_byte_received_at == .6
    clock.now = .951
    assert decoder.latest.peek() is None


@pytest.mark.parametrize('original_known', [False, True])
def test_original_packet_receipt_includes_reassembly_delay_when_supplied(encoded, original_known):
    path, _ = encoded;clock = Clock();clock.now = .5
    samples = []
    decoder = IncrementalHevcDecoder(OWNER, GENERATION, clock=clock, on_frame=samples.append)
    kwargs = {'received_at': .1} if original_known else {}
    raw = path.read_bytes()
    decoder.feed(raw, delivered_at=.5, **kwargs);decoder.finish()
    assert all(s.original_packet_receipt_known is original_known for s in samples)
    assert decoder.latest.stale == (9 if original_known else 0)
    assert (decoder.latest.peek() is None) is original_known


def test_receipt_span_limit_fails_terminally_and_releases_buffers(monkeypatch, av_available):
    monkeypatch.setattr(video_stream, 'MAX_SPANS', 4)
    clock = Clock();decoder = IncrementalHevcDecoder(OWNER, GENERATION, clock=clock)
    raw = nal(32)
    for i in range(4):
        clock.now = i*.001
        decoder.feed(raw[i:i+1], delivered_at=clock.now)
    clock.now = .004
    with pytest.raises(ValueError, match='receipt_span_limit'):
        decoder.feed(raw[4:5], delivered_at=clock.now)
    assert not decoder.receipts.spans and decoder.framer.buffered_bytes == 0
    assert decoder.latest.peek() is None
    with pytest.raises(ValueError, match='finished_or_failed'):
        decoder.feed(b'', delivered_at=clock.now)


def test_access_unit_limit_cannot_leave_previous_frame_available(encoded, monkeypatch):
    path, _ = encoded
    monkeypatch.setattr(video_stream, 'MAX_ACCESS_UNIT_BYTES', 400)
    decoder = IncrementalHevcDecoder(OWNER, GENERATION, clock=lambda:0.)
    with pytest.raises(ValueError, match='access_unit_byte_limit'):
        decoder.feed(path.read_bytes(), delivered_at=0.)
    assert decoder.error and decoder.latest.peek() is None
    assert not decoder._pending and not decoder._codec_nals


def test_frame_sink_failure_invalidates_already_published_pixels(encoded):
    path, _ = encoded
    def fail(_):raise OSError('sink unavailable')
    decoder = IncrementalHevcDecoder(OWNER, GENERATION, clock=lambda:0., on_frame=fail)
    with pytest.raises(OSError, match='sink unavailable'):
        decoder.feed(path.read_bytes(), delivered_at=0.)
    assert decoder.latest.peek() is None
    assert decoder.latest.reason == 'sink unavailable'


@pytest.mark.parametrize('values', [
    {'delivered_at':float('nan')}, {'delivered_at':1.},
    {'delivered_at':0.,'received_at':.1}, {'delivered_at':0.,'received_at':-1.}])
def test_invalid_receipt_domain_never_creates_an_observation(values, av_available):
    decoder = IncrementalHevcDecoder(OWNER, GENERATION, clock=lambda:0.)
    with pytest.raises(ValueError, match='stream_input_or_receipt_invalid'):
        decoder.feed(b'\0', **values)
    assert decoder.latest.peek() is None


def test_invalid_decoder_thread_type_is_explicit(av_available):
    with pytest.raises(ValueError, match='stream_decoder_thread_type_invalid'):
        IncrementalHevcDecoder(OWNER, GENERATION, decode_thread_type='AUTO')
