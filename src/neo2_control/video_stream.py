"""Incremental HEVC observations with bounded state and original PC byte ages.

The synchronous decoder belongs in a worker, never on the UDP/ACK thread.
Its input clock is ordered-byte delivery on the PC, not camera exposure. The
current controller/HLS preview is not changed by importing or using this core.
"""
from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field
import copy
import hashlib
import io
import math
import re
import threading
import time

from .hevc import HevcAnnexBFilter
from .video_clock import _messages, _candidate, _units

MAX_SPANS = 4096
MAX_ACCESS_UNIT_BYTES = 8 * 1024 * 1024
MAX_HEADER_BYTES = 4 * 1024 * 1024
MAX_ACCESS_UNIT_NALS = 256
MAX_PENDING_PICTURES = 64
MAX_PIXELS = 1920 * 1080


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _context(owner_epoch, connection_generation):
    if (not isinstance(owner_epoch, str) or not re.fullmatch('[0-9a-f]{48}', owner_epoch)
            or not isinstance(connection_generation, str)
            or not re.fullmatch('[0-9a-f]{24}', connection_generation)):
        raise ValueError('stream_context_invalid')
    return owner_epoch, connection_generation


@dataclass(frozen=True)
class DecodedObservation:
    owner_epoch: str
    connection_generation: str
    decoded_index: int
    raw_picture_ordinal: int
    pc_first_byte_delivered_at: float
    pc_first_byte_received_at: float
    original_packet_receipt_known: bool
    decoded_at: float
    width: int
    height: int
    rgb: bytes = field(repr=False)
    rgb_sha256: str
    details: dict = field(repr=False)

    def record(self):
        # Timing/context fields cannot be replaced by a mutated details dict.
        return {**copy.deepcopy(self.details), 'owner_epoch': self.owner_epoch,
            'connection_generation': self.connection_generation,
            'decoded_index': self.decoded_index, 'raw_picture_ordinal': self.raw_picture_ordinal,
            'pc_first_byte_delivered_at': self.pc_first_byte_delivered_at,
            'pc_first_byte_received_at': self.pc_first_byte_received_at,
            'original_packet_receipt_known': self.original_packet_receipt_known,
            'decoded_at': self.decoded_at,
            'pixels': {'width': self.width, 'height': self.height,
                'format': 'rgb24', 'sha256': self.rgb_sha256},
            'clock_unit_verified': False, 'exposure_semantics_verified': False,
            'pc_clock_alignment_verified': False, 'flight_authorized': False}


class LatestDecodedFrame:
    """One immutable pixel buffer; expiry uses original input, not publication.

    The retained frame is the most recent decoder output. PC byte receipt
    does not establish which image has the most recent camera exposure.
    """
    def __init__(self, owner_epoch, connection_generation, *, maximum_age=.35, clock=time.monotonic):
        self.context = _context(owner_epoch, connection_generation)
        if not _finite(maximum_age) or not .01 <= maximum_age <= 2:
            raise ValueError('stream_maximum_age_invalid')
        self.maximum_age, self.clock = maximum_age, clock
        self.lock = threading.RLock()
        self.current = None
        self.reason = None
        self._last_time = None
        self._last_index = -1
        self.published = self.overwritten = self.stale = self.expired = 0

    def _now(self):
        now = self.clock()
        if not _finite(now) or (self._last_time is not None and now < self._last_time):
            self.invalidate('stream_clock_invalid')
            raise ValueError('stream_clock_invalid')
        self._last_time = now
        return now

    def invalidate(self, reason):
        with self.lock:
            self.reason = self.reason or reason
            self.current = None

    def discard(self):
        """Remove available pixels without authorizing or reopening anything."""
        with self.lock:
            self.current = None

    def publish(self, sample):
        with self.lock:
            if self.reason is not None:
                return False
            now = self._now()
            if (not isinstance(sample, DecodedObservation)
                    or (sample.owner_epoch, sample.connection_generation) != self.context
                    or type(sample.decoded_index) is not int or sample.decoded_index <= self._last_index
                    or not all(_finite(t) for t in (sample.pc_first_byte_received_at,
                        sample.pc_first_byte_delivered_at, sample.decoded_at))
                    or not 0 <= sample.pc_first_byte_received_at <= sample.pc_first_byte_delivered_at <= sample.decoded_at <= now
                    or not isinstance(sample.rgb, bytes)
                    or not 2 <= sample.width <= 1920 or not 2 <= sample.height <= 1080
                    or len(sample.rgb) != sample.width * sample.height * 3):
                self.invalidate('stream_observation_identity_or_clock_invalid')
                raise ValueError(self.reason)
            self._last_index = sample.decoded_index
            if now - sample.pc_first_byte_received_at > self.maximum_age:
                self.stale += 1
                self.current = None
                return False
            self.overwritten += self.current is not None
            self.published += 1
            self.current = sample
            return True

    def peek(self):
        with self.lock:
            if self.reason is not None:
                return None
            now = self._now()
            if self.current is not None and now - self.current.pc_first_byte_received_at > self.maximum_age:
                self.expired += 1
                self.current = None
            return self.current

    def status(self):
        with self.lock:
            current = self.peek()
            return {'retained_frames': int(current is not None),
                'retained_pixel_bytes': len(current.rgb) if current is not None else 0,
                'maximum_age_seconds': self.maximum_age, 'published': self.published,
                'overwritten': self.overwritten, 'stale_on_publication': self.stale,
                'expired_on_read': self.expired, 'invalidated': self.reason,
                'age_source': 'pc_original_packet_receipt_when_known_else_ordered_delivery',
                'exposure_age_bound_verified': False, 'flight_authorized': False}


class _Receipts:
    def __init__(self):
        self.spans = deque()
        self.end = 0
        self.peak = 0

    def append(self, size, delivered, received, known):
        if not size:
            return
        values = (delivered, received, known)
        if self.spans and self.spans[-1][2:] == values:
            start, _, *old = self.spans.pop()
            self.spans.append((start, self.end + size, *old))
        else:
            if len(self.spans) == MAX_SPANS:
                raise ValueError('stream_receipt_span_limit')
            self.spans.append((self.end, self.end + size, *values))
        self.end += size
        self.peak = max(self.peak, len(self.spans))

    def at(self, offset):
        for start, end, delivered, received, known in self.spans:
            if start <= offset < end:
                return delivered, received, known
        raise ValueError('stream_byte_receipt_missing')

    def discard(self, offset):
        while self.spans and self.spans[0][1] <= offset:
            self.spans.popleft()


class _Framer(HevcAnnexBFilter):
    def __init__(self, owner):
        super().__init__()
        self.owner, self.consumed = owner, 0

    def _preamble(self, raw):
        self.consumed += len(raw)
        self.owner.receipts.discard(self.consumed)
        return b''

    def _unit(self, raw, prefix_length):
        start = self.consumed
        self.consumed += len(raw)
        if not 2 <= len(raw) - prefix_length <= self.max_nal_bytes:
            raise ValueError('stream_nal_size_invalid')
        self.owner._nal(raw, prefix_length, start, self.consumed)
        self.owner.receipts.discard(self.consumed)
        return b''


@dataclass(frozen=True)
class _Identity:
    raw_picture: int
    first_delivered: float
    first_received: float
    receipt_known: bool
    details: dict = field(repr=False)


def _signature(raw):
    result = []
    for _, _, body in _units(io.BytesIO(raw), hashlib.sha256()):
        if len(body) < 2:
            raise ValueError('stream_normalized_header_invalid')
        result.append(((body[0] >> 1) & 63, hashlib.sha256(body).hexdigest()))
        if len(result) > MAX_ACCESS_UNIT_NALS:
            raise ValueError('stream_access_unit_nal_limit')
    return result


class IncrementalHevcDecoder:
    """Worker-only source-bound decoding, requiring observed DJI AUD framing.

    Input may split every start code or SEI. Original receipt is optional;
    without it ages begin at ordered delivery, omitting prior reassembly delay.
    The caller owns any input queue and must keep it bounded separately.
    ``on_frame`` receives every output for archival verification, including
    stale ones. Current observations must be read through ``latest.peek()``.
    """
    def __init__(self, owner_epoch, connection_generation, *, nominal_fps=30.,
            maximum_age=.35, clock=time.monotonic, on_frame=None, decode_thread_type='FRAME'):
        import av
        self.context = _context(owner_epoch, connection_generation)
        if not _finite(nominal_fps) or not 1 <= nominal_fps <= 60:
            raise ValueError('stream_nominal_fps_invalid')
        if decode_thread_type not in ('FRAME', 'SLICE'):
            raise ValueError('stream_decoder_thread_type_invalid')
        if on_frame is not None and not callable(on_frame):
            raise ValueError('stream_frame_sink_invalid')
        self.decode_thread_type = decode_thread_type
        self.av, self.clock, self.on_frame = av, clock, on_frame
        self.latest = LatestDecodedFrame(*self.context, maximum_age=maximum_age, clock=clock)
        self.receipts = _Receipts()
        self.framer = _Framer(self)
        self.normalizer = av.bitstream.BitStreamFilterContext(
            f'hevc_metadata=tick_rate={nominal_fps}:num_ticks_poc_diff_one=1', in_stream='hevc')
        self.decoder = av.CodecContext.create('hevc', 'r')
        self.decoder.thread_count = 2
        self.decoder.thread_type = decode_thread_type
        self.decoder.copy_opaque = True
        self.decoder.options = {'err_detect': 'explode', 'max_pixels': str(MAX_PIXELS)}
        self._last_delivered = self._last_time = None
        self._parameters = {}
        self._codec_nals = []
        self._codec_bytes = 0
        self._picture = None
        self._interval_start = self._interval_sum = 0
        self._checksum_prefix = None
        self._pending = {}
        self.ready = self.finished = False
        self.error = None
        self.raw_pictures = self.decoded_frames = self.discarded_before_bootstrap = 0
        self.malformed_sei = self.rewritten_parameters = 0
        self.peak_access_unit_bytes = self.peak_pending = 0

    def _now(self):
        now = self.clock()
        if not _finite(now) or (self._last_time is not None and now < self._last_time):
            raise ValueError('stream_clock_invalid')
        self._last_time = now
        return now

    def _fail(self, error):
        self.error = self.error or str(error)
        self.latest.invalidate(self.error)
        self._pending.clear()
        self._codec_nals.clear()
        self._parameters.clear()
        self._codec_bytes = 0
        self._picture = None
        self.receipts.spans.clear()
        self.framer._buffer.clear()

    def feed(self, data, *, delivered_at, received_at=None):
        if self.finished or self.error is not None:
            raise ValueError('stream_decoder_finished_or_failed')
        try:
            now = self._now()
            known = received_at is not None
            received = received_at if known else delivered_at
            if (not isinstance(data, bytes) or len(data) > 65536
                    or not _finite(delivered_at) or not _finite(received)
                    or not 0 <= received <= delivered_at <= now
                    or (self._last_delivered is not None and delivered_at < self._last_delivered)):
                raise ValueError('stream_input_or_receipt_invalid')
            self._last_delivered = delivered_at
            self.receipts.append(len(data), delivered_at, received, known)
            self.framer.feed(data)
        except Exception as error:
            self._fail(error)
            raise

    def finish(self):
        if self.finished or self.error is not None:
            raise ValueError('stream_decoder_finished_or_failed')
        try:
            self._now()
            self.framer.finish()
            self._complete()
            if self.normalizer.filter(None):
                raise ValueError('stream_normalizer_delayed_packet')
            if self.ready:
                for frame in self.decoder.decode(None):
                    self._receive(frame)
            if self._pending:
                raise ValueError('stream_missing_decoded_picture')
            self.finished = True
        except Exception as error:
            self._fail(error)
            raise

    def _append_codec(self, raw):
        if self._codec_bytes + len(raw) > MAX_ACCESS_UNIT_BYTES:
            raise ValueError('stream_access_unit_byte_limit')
        if len(self._codec_nals) == MAX_ACCESS_UNIT_NALS:
            raise ValueError('stream_access_unit_nal_limit')
        self._codec_nals.append(raw)
        self._codec_bytes += len(raw)
        self.peak_access_unit_bytes = max(self.peak_access_unit_bytes, self._codec_bytes)

    def _nal(self, raw, prefix, start, end):
        body = raw[prefix:].rstrip(b'\0')
        if len(body) < 2 or body[0] & 128 or not body[1] & 7:
            raise ValueError('stream_nal_header_invalid')
        if ((body[0] & 1) << 5) | (body[1] >> 3):
            raise ValueError('stream_multilayer_not_supported')
        kind = (body[0] >> 1) & 63
        if kind == 35:
            self._complete()
            self._interval_start, self._interval_sum = end, 0
            if self.ready:
                self._append_codec(raw)
            return
        prefix_sum = self._interval_sum
        self._interval_sum = (self._interval_sum + sum(raw)) & 0xffffffff
        if kind in (32, 33, 34):
            if self._picture is not None:
                raise ValueError('stream_parameter_change_inside_picture')
            self._parameters[kind] = raw
            if sum(map(len, self._parameters.values())) > MAX_HEADER_BYTES:
                raise ValueError('stream_parameter_cache_limit')
        if kind <= 31:
            if len(body) < 3:
                raise ValueError('stream_slice_truncated')
            if body[2] & 128:
                if self._picture is not None:
                    raise ValueError('stream_picture_without_aud_boundary')
                delivered, received, known = self.receipts.at(start)
                last_delivered, last_received, last_known = self.receipts.at(end - 1)
                self._picture = {'raw_picture_ordinal': self.raw_pictures,
                    'source_first_slice_byte_offset': start, 'first_slice_nal_end_offset': end,
                    'source_vcl_body_sha256': hashlib.sha256(body).hexdigest(),
                    'first_delivered': delivered, 'first_received': received, 'receipt_known': known,
                    'pc_vcl_last_byte_delivered_at': last_delivered,
                    'pc_vcl_last_byte_received_at': last_received,
                    'last_byte_original_receipt_known': last_known, 'candidate_fields': []}
                self.raw_pictures += 1
                if not self.ready and 16 <= kind <= 23 and len(self._parameters) == 3:
                    self.ready = True
                    self._codec_nals.clear(); self._codec_bytes = 0
                    for k in (32, 33, 34):
                        self._append_codec(self._parameters[k])
            elif self.ready and self._picture is None:
                raise ValueError('stream_continuation_without_picture')
        if self.ready and kind not in (39, 40):
            self._append_codec(raw)
        if kind != 40:
            return
        try:
            messages = _messages(body)
        except ValueError:
            self.malformed_sei += 1
            return
        for message_type, payload in messages:
            value = _candidate(message_type, payload)
            if value is None or self._picture is None:
                continue
            fields = self._picture['candidate_fields']
            if len(fields) == 64:
                raise ValueError('stream_picture_field_limit')
            fields.append({**value, 'nal_offset': start, 'nal_end_offset': end})
            if message_type == 256:
                self._checksum_prefix = (self._interval_start, start, prefix_sum)

    def _complete(self):
        picture, self._picture = self._picture, None
        if picture is None:
            # Retain changed parameter sets across a pictureless AUD interval.
            parameters = [raw for raw in self._codec_nals if (raw[4 if raw[:4] == b'\0\0\0\1' else 3] >> 1) & 63 in (32, 33, 34)]
            self._codec_nals = parameters
            self._codec_bytes = sum(map(len, parameters))
            return
        raws, self._codec_nals = self._codec_nals, []
        self._codec_bytes = 0
        if not self.ready:
            self.discarded_before_bootstrap += 1
            self._checksum_prefix = None
            return
        fields = picture['candidate_fields']
        check = {'available': False, 'reason': 'candidate_pair_missing_or_ambiguous'}
        if Counter(v['payload_type'] for v in fields) == {256: 1, 240: 1}:
            by_type = {v['payload_type']: v for v in fields}
            a, b = by_type[256], by_type[240]
            picture['type240_minus_type256_ticks'] = b['split_tick_candidate'] - a['split_tick_candidate']
            lower, upper, computed = self._checksum_prefix
            if lower <= picture['source_first_slice_byte_offset'] and picture['first_slice_nal_end_offset'] <= upper < b['nal_offset']:
                payload = bytes.fromhex(b['payload_hex'])
                expected = int.from_bytes(payload[9:11], 'little') + (int.from_bytes(payload[12:14], 'little') << 16)
                check = {'available': True, 'algorithm': 'sum_u8_mod_2_32',
                    'byte_offset_start': lower, 'byte_offset_end': upper,
                    'expected_from_sei240': expected, 'computed_from_original_bytes': computed,
                    'native_zero_bypass': expected == 0, 'matched': computed == expected if expected else None,
                    'scope': 'Observed AUD interval before SEI256; not a decoded image or exposure proof'}
            else:
                check = {'available': False, 'reason': 'encoded_interval_boundary_unestablished'}
        picture['encoded_interval_checksum'] = check
        self._checksum_prefix = None
        raw = b''.join(raws)
        original_signature = _signature(raw)
        if sum(kind <= 31 for kind, _ in original_signature) < 1:
            raise ValueError('stream_picture_has_no_vcl')
        if len(self._pending) == MAX_PENDING_PICTURES:
            raise ValueError('stream_pending_picture_limit')
        tag = _Identity(picture['raw_picture_ordinal'], picture.pop('first_delivered'),
            picture.pop('first_received'), picture.pop('receipt_known'), picture)
        packet = self.av.Packet(raw)
        packet.opaque = tag
        outputs = self.normalizer.filter(packet)
        if len(outputs) != 1 or outputs[0].opaque is not tag:
            raise ValueError('stream_normalizer_packet_identity_changed')
        normalized = bytes(outputs[0])
        after = _signature(normalized)
        if len(after) != len(original_signature):
            raise ValueError('stream_normalizer_nal_count_changed')
        for (kind, digest), (new_kind, new_digest) in zip(original_signature, after):
            if kind != new_kind or (digest != new_digest and kind not in (32, 33)):
                raise ValueError('stream_normalizer_changed_picture_or_pps')
            self.rewritten_parameters += digest != new_digest
        picture['normalized_packet_sha256'] = hashlib.sha256(normalized).hexdigest()
        self._pending[tag.raw_picture] = tag
        self.peak_pending = max(self.peak_pending, len(self._pending))
        for frame in self.decoder.decode(outputs[0]):
            self._receive(frame)

    def _receive(self, frame):
        tag = frame.opaque
        if not isinstance(tag, _Identity) or self._pending.pop(tag.raw_picture, None) is not tag:
            raise ValueError('stream_decoded_frame_identity_missing')
        if frame.is_corrupt or not 2 <= frame.width <= 1920 or not 2 <= frame.height <= 1080:
            raise ValueError('stream_corrupt_or_oversized_frame')
        rgb = frame.reformat(format='rgb24')
        plane, stride = memoryview(rgb.planes[0]), rgb.planes[0].line_size
        pixels = b''.join(plane[y*stride:y*stride+3*rgb.width] for y in range(rgb.height))
        details = {**tag.details, 'picture_type': self.av.video.frame.PictureType(frame.pict_type).name}
        sample = DecodedObservation(*self.context, self.decoded_frames, tag.raw_picture,
            tag.first_delivered, tag.first_received, tag.receipt_known, self._now(),
            rgb.width, rgb.height, pixels, hashlib.sha256(pixels).hexdigest(), details)
        self.decoded_frames += 1
        self.latest.publish(sample)
        if self.on_frame is not None:
            self.on_frame(sample)

    def status(self):
        return {'raw_picture_candidates': self.raw_pictures, 'decoded_frames': self.decoded_frames,
            'discarded_before_bootstrap': self.discarded_before_bootstrap,
            'malformed_sei_nals': self.malformed_sei, 'rewritten_parameter_nals': self.rewritten_parameters,
            'pending_pictures': len(self._pending), 'peak_pending_pictures': self.peak_pending,
            'buffered_nal_bytes': self.framer.buffered_bytes,
            'access_unit_bytes': self._codec_bytes, 'peak_access_unit_bytes': self.peak_access_unit_bytes,
            'receipt_spans': len(self.receipts.spans), 'peak_receipt_spans': self.receipts.peak,
            'finished': self.finished, 'error': self.error, 'latest': self.latest.status(),
            'decoder_threads': 2, 'decoder_thread_type': self.decode_thread_type,
            'live_transport_integrated': False, 'exposure_age_bound_verified': False,
            'flight_authorized': False}
