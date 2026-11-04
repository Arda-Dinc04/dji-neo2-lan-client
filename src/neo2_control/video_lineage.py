"""Offline decoded pixels tied to original HEVC pictures via packet opaque refs.

All native decoding runs in a bounded child. Timestamps remain uncalibrated
candidates; this tool never opens a controller, radio or camera connection.
"""
from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from fractions import Fraction
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import stat
import sys
import time

from .hevc import HevcAnnexBFilter
from .video_clock import inventory_hevc_clocks, _units
from .video_export import (_path, _parent, _stamp, _deadline, _write, _run,
    _limited, _normalizer, MAX_SOURCE_BYTES, CHUNK_BYTES)

MAX_JSON_BYTES = 64 * 1024 * 1024
MAX_IMAGE_BYTES = 16 * 1024 * 1024
MAX_IMAGES_BYTES = 128 * 1024 * 1024
MAX_FRAME_PIXELS = 1920 * 1080
MAX_SELECTED = 32
MAX_SESSION_SELECTED = 96
MAX_PACKETS = 60_000


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _json_new(path, value):
    raw = json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode()
    if len(raw) > MAX_JSON_BYTES:
        raise ValueError('video_lineage_json_limit')
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(raw)


class _TrackedFilter(HevcAnnexBFilter):
    """Observe the existing filter's emissions, including its cached bootstrap."""
    def __init__(self):
        super().__init__(wait_for_bootstrap=True, strip_sei=True)
        self.consumed = self.prepared_bytes = 0
        self.records = []
        self._sources = {}

    def _preamble(self, raw):
        self.consumed += len(raw)
        return super()._preamble(raw)

    def _unit(self, raw, prefix_length):
        start = self.consumed
        self.consumed += len(raw)
        ready = self.bootstrap_ready
        kind = (raw[prefix_length] >> 1) & 63 if len(raw) > prefix_length else None
        body = raw[prefix_length:].rstrip(b'\0')
        record = {'source_start': start, 'source_end': self.consumed,
            'nal_type': kind, 'body_sha256': _sha(body), 'raw_sha256': _sha(raw),
            'first_slice': kind is not None and kind <= 31 and len(body) >= 3 and bool(body[2] & 128)}
        emitted = super()._unit(raw, prefix_length)
        if not ready and kind in (32, 33, 34):
            self._sources[kind] = record
        if emitted:
            records = [record] if ready else [self._sources[k] for k in (32, 33, 34)] + [record]
            offset = 0
            for item in records:
                size = item['source_end'] - item['source_start']
                if _sha(emitted[offset:offset + size]) != item['raw_sha256']:
                    raise ValueError('video_lineage_filter_emission_changed')
                self.records.append({**item, 'prepared_start': self.prepared_bytes,
                    'prepared_end': self.prepared_bytes + size})
                self.prepared_bytes += size
                offset += size
            if offset != len(emitted):
                raise ValueError('video_lineage_filter_emission_size_changed')
            if not ready:
                self._sources.clear()
        return emitted


def _normalization_map(path, originals):
    digest = hashlib.sha256()
    mapped = []
    rewritten = []
    with path.open('rb') as stream:
        for i, (start, end, body) in enumerate(_units(stream, digest)):
            if i >= len(originals):
                raise ValueError('video_lineage_normalizer_added_nal')
            old = originals[i]
            kind = (body[0] >> 1) & 63
            if kind != old['nal_type']:
                raise ValueError('video_lineage_normalizer_reordered_nals')
            body_hash = _sha(body)
            if body_hash != old['body_sha256']:
                if kind not in (32, 33):
                    raise ValueError('video_lineage_normalizer_changed_picture_or_pps')
                rewritten.append(i)
            mapped.append({**old, 'normalized_start': start, 'normalized_end': end,
                'normalized_body_sha256': body_hash})
    if len(mapped) != len(originals):
        raise ValueError('video_lineage_normalizer_removed_nal')
    return mapped, {'sha256': digest.hexdigest(), 'bytes': path.stat().st_size,
        'mapped_nals': len(mapped), 'rewritten_parameter_nals': rewritten,
        'all_vcl_bodies_byte_identical': True}


def build_video_lineage(source, output, *, pictures=(), nominal_fps=30, timeout=180,
        maximum_selected=MAX_SELECTED):
    """Write a new private directory with a manifest and selected lossless PNGs.

    ``pictures`` are raw first-slice ordinals, not nominal MP4 frame numbers.
    Only a successful result publishes manifest.json. The original is frozen
    before parsing; all working copies are removed after success or failure.
    An interrupted/failed directory is retained, never reused or overwritten.
    The default selection budget is 32; a complete scan can explicitly request
    up to 96. Pixel, total image-byte and worker resource limits still apply.
    """
    if type(maximum_selected) is not int or not 1<=maximum_selected<=MAX_SESSION_SELECTED:
        raise ValueError('video_lineage_invalid_selection_budget')
    if (not isinstance(pictures, (tuple, list)) or len(pictures) > maximum_selected
            or any(type(p) is not int or not 0 <= p < MAX_PACKETS for p in pictures)
            or len(set(pictures)) != len(pictures)):
        raise ValueError('video_lineage_invalid_picture_selection')
    if type(nominal_fps) not in (int, float) or not math.isfinite(nominal_fps) or not 1 <= nominal_fps <= 60:
        raise ValueError('video_lineage_invalid_nominal_fps')
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 0 < timeout <= 600:
        raise ValueError('video_lineage_invalid_timeout')
    try:
        import av  # Validate the optional dependency before creating output.
    except ImportError:
        raise ValueError('video_lineage_requires_video_analysis_extra') from None
    if not hasattr(av.CodecContext.create('hevc', 'r'), 'copy_opaque'):
        raise ValueError('video_lineage_requires_packet_opaque_support')
    source, output = _path(source), _path(output)
    ffmpeg, limiter = (shutil.which(name) for name in ('ffmpeg', 'prlimit'))
    if not ffmpeg or not limiter:
        raise ValueError('video_lineage_requires_ffmpeg_prlimit')
    deadline = time.monotonic() + timeout
    parent = root = scratch = source_parent = source_fd = None
    made = False
    try:
        source_parent = _parent(source)
        source_fd = os.open(source.name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=source_parent)
        initial = os.fstat(source_fd)
        if not stat.S_ISREG(initial.st_mode) or not 0 < initial.st_size <= MAX_SOURCE_BYTES:
            raise ValueError('video_lineage_source_not_bounded_regular_file')
        parent = _parent(output, create=True)
        os.mkdir(output.name, 0o700, dir_fd=parent)
        root = os.open(output.name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent)
        os.mkdir('work', 0o700, dir_fd=root)
        scratch = os.open('work', os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=root)
        made = True
        # Paths through the pinned directory descriptors do not follow a
        # replacement of the visible output name during this operation.
        work = Path(f'/proc/self/fd/{scratch}')
        digest = hashlib.sha256()
        total = prepared_bytes = 0
        filtered = _TrackedFilter()
        with (work/'source.hevc').open('xb') as frozen, (work/'prepared.hevc').open('xb') as prepared:
            os.fchmod(frozen.fileno(), 0o600)
            os.fchmod(prepared.fileno(), 0o600)
            while True:
                _deadline(deadline)
                data = os.read(source_fd, CHUNK_BYTES)
                if not data:
                    break
                total += len(data)
                if total > MAX_SOURCE_BYTES:
                    raise ValueError('video_lineage_source_grew')
                digest.update(data)
                frozen.write(data)
                prepared_bytes = _write(prepared.fileno(), filtered.feed(data), prepared_bytes, MAX_SOURCE_BYTES)
            prepared_bytes = _write(prepared.fileno(), filtered.finish(), prepared_bytes, MAX_SOURCE_BYTES)
        if (initial.st_size != total or _stamp(initial) != _stamp(os.fstat(source_fd))
                or filtered.consumed != total):
            raise ValueError('video_lineage_source_changed')
        if not filtered.bootstrap_ready:
            raise ValueError('video_lineage_no_bootstrap')
        inventory = inventory_hevc_clocks(work/'source.hevc')
        if inventory['source']['sha256'] != digest.hexdigest():
            raise ValueError('video_lineage_frozen_source_changed')
        with (work/'prepared.hevc').open('rb') as input_stream, (work/'normalized.hevc').open('xb') as normalized:
            os.fchmod(normalized.fileno(), 0o600)
            _, normalization_log = _run(_normalizer(limiter, ffmpeg, nominal_fps, work),
                directory_fd=scratch, deadline=deadline, stage='lineage_normalize',
                stdin=input_stream, stdout=normalized)
        nal_map, normalization = _normalization_map(work/'normalized.hevc', filtered.records)
        config = {'source': {'path': str(source), 'bytes': total, 'sha256': digest.hexdigest()},
            'pictures': inventory['pictures'], 'nal_map': nal_map, 'selected': list(pictures),
            'preparation': {'bootstrap_discarded_bytes': filtered.bootstrap_discarded_bytes,
                'dropped_sei_nals': filtered.dropped_sei_nals, 'prepared_bytes': prepared_bytes,
                'nominal_fps': nominal_fps, 'timestamp_source': 'local_nominal_fps',
                'normalization': normalization, 'normalization_log': normalization_log}}
        _json_new(work/'input.json', config)
        _, worker_log = _run(_limited(limiter, sys.executable) +
            ['-m', 'neo2_control.video_lineage', '--worker'],
            directory_fd=root, deadline=deadline, stage='lineage_decode')
        result_path = Path(f'/proc/self/fd/{root}')/'result.json'
        if result_path.stat().st_size > MAX_JSON_BYTES:
            raise ValueError('video_lineage_json_limit')
        result = json.loads(result_path.read_bytes())
        if result.get('source') != config['source'] or result.get('format') != 'neo2-control/video-frame-lineage/v1':
            raise ValueError('video_lineage_worker_result_mismatch')
        if _stamp(initial) != _stamp(os.fstat(source_fd)):
            raise ValueError('video_lineage_source_changed')
        result['decoder']['process_log'] = worker_log
        os.unlink('result.json', dir_fd=root)
        _json_new(Path(f'/proc/self/fd/{root}')/'manifest.json', result)
        return result
    except Exception as error:
        if made and root is not None:
            try:
                _json_new(Path(f'/proc/self/fd/{root}')/'failure.json', {
                    'success': False, 'error': type(error).__name__, 'reason': str(error),
                    'diagnostics': getattr(error, 'diagnostics', None)})
            except OSError:
                pass  # Preserve the original error if the output became unavailable.
        raise
    finally:
        if scratch is not None:
            os.close(scratch)
        if made and root is not None:
            shutil.rmtree('work', dir_fd=root)
        for fd in (root, parent, source_fd, source_parent):
            if fd is not None:
                os.close(fd)


@dataclass(frozen=True)
class _PacketIdentity:
    index: int
    picture: int


def _rgb(frame):
    rgb = frame.reformat(format='rgb24')
    plane = memoryview(rgb.planes[0])
    stride = rgb.planes[0].line_size
    width = rgb.width * 3
    digest = hashlib.sha256()
    for y in range(rgb.height):
        digest.update(plane[y * stride:y * stride + width])
    return rgb, digest.hexdigest()


def _png(av, frame):
    encoder = av.CodecContext.create('png', 'w')
    encoder.width, encoder.height = frame.width, frame.height
    encoder.pix_fmt = 'rgb24'
    encoder.time_base = Fraction(1, 30)
    encoder.thread_count = 1
    raw = b''.join(bytes(p) for p in [*encoder.encode(frame), *encoder.encode(None)])
    if not raw.startswith(b'\x89PNG\r\n\x1a\n') or len(raw) > MAX_IMAGE_BYTES:
        raise ValueError('video_lineage_png_limit_or_format')
    check = av.CodecContext.create('png', 'r')
    decoded = check.decode(av.Packet(raw)) + check.decode(None)
    if len(decoded) != 1:
        raise ValueError('video_lineage_png_decode_count')
    return raw, _rgb(decoded[0])[1]


def _decode(config):
    import av
    mapping = config['nal_map']
    starts = [n['normalized_start'] for n in mapping]
    by_offset = {p['first_slice_byte_offset']: p for p in config['pictures']}
    wanted = set(config['selected'])
    pending, records, outputs = {}, [], []
    seen_packets = 0
    pictureless = []
    total_images = 0
    with av.open('work/normalized.hevc', format='hevc') as container, open('work/normalized.hevc', 'rb') as raw:
        if len(container.streams) != 1 or container.streams[0].codec_context.name != 'hevc':
            raise ValueError('video_lineage_unexpected_streams')
        ctx = container.streams[0].codec_context
        ctx.thread_count = 2
        ctx.thread_type = 'SLICE'
        ctx.copy_opaque = True
        ctx.options = {'err_detect': 'explode', 'max_pixels': str(MAX_FRAME_PIXELS)}

        def receive(frame):
            nonlocal total_images
            tag = frame.opaque
            if not isinstance(tag, _PacketIdentity) or tag.index not in pending:
                raise ValueError('video_lineage_decoded_frame_has_no_packet_identity')
            original_tag, packet_info = pending.pop(tag.index)
            if tag is not original_tag:
                raise ValueError('video_lineage_packet_identity_changed')
            if frame.is_corrupt or not 2 <= frame.width <= 1920 or not 2 <= frame.height <= 1080:
                raise ValueError('video_lineage_corrupt_or_oversized_frame')
            rgb, rgb_hash = _rgb(frame)
            picture = by_offset[packet_info['source_vcl_start']]
            record = {'decoded_index': len(records), 'raw_picture_ordinal': tag.picture,
                'packet': packet_info, 'source_first_slice_byte_offset': picture['first_slice_byte_offset'],
                'candidate_fields': picture['candidate_fields'],
                'encoded_interval_checksum': picture['encoded_interval_checksum'],
                'pixels': {'width': frame.width, 'height': frame.height,
                    'format': 'rgb24', 'sha256': rgb_hash},
                'picture_type': av.video.frame.PictureType(frame.pict_type).name}
            if tag.picture in wanted:
                png, roundtrip = _png(av, rgb)
                if roundtrip != rgb_hash:
                    raise ValueError('video_lineage_png_pixels_changed')
                total_images += len(png)
                if total_images > MAX_IMAGES_BYTES:
                    raise ValueError('video_lineage_total_image_limit')
                filename = f'picture-{tag.picture:05d}.png'
                fd = os.open(filename, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
                with os.fdopen(fd, 'wb') as stream:
                    stream.write(png)
                record['image'] = {'file': filename, 'bytes': len(png), 'sha256': _sha(png),
                    'decoded_rgb_sha256': roundtrip, 'pixels_equal_decoder_output': True}
                outputs.append(tag.picture)
            records.append(record)

        for packet in container.demux(video=0):
            if not packet.size:
                break
            seen_packets += 1
            if seen_packets > MAX_PACKETS:
                raise ValueError('video_lineage_packet_count_limit')
            position, size = packet.pos, packet.size
            if position is None or position < 0 or not 0 < size <= 16 * 1024 * 1024:
                raise ValueError('video_lineage_packet_position_unavailable')
            data = bytes(packet)
            if data != os.pread(raw.fileno(), size, position):
                raise ValueError('video_lineage_packet_bytes_not_at_recorded_position')
            first = bisect_right(starts, position) - 1
            if first < 0 or starts[first] != position:
                raise ValueError('video_lineage_packet_not_on_nal_boundary')
            last = bisect_left(starts, position + size)
            nals = mapping[first:last]
            if not nals or nals[-1]['normalized_end'] != position + size:
                raise ValueError('video_lineage_packet_ends_inside_nal')
            vcls = [n for n in nals if n['first_slice']]
            if not vcls:
                if any(n['nal_type'] not in (32, 33, 34, 35, 36, 37) for n in nals):
                    raise ValueError('video_lineage_packet_has_no_picture')
                pictureless.append({'position': position, 'bytes': size, 'nal_types': [n['nal_type'] for n in nals]})
                continue
            if len(vcls) != 1:
                raise ValueError('video_lineage_packet_has_multiple_pictures')
            source_vcl = vcls[0]
            picture = by_offset[source_vcl['source_start']]
            tag = _PacketIdentity(seen_packets - 1, picture['raw_picture_ordinal'])
            pending[tag.index] = (tag, {'index': tag.index, 'normalized_position': position,
                'bytes': size, 'sha256': _sha(data), 'source_vcl_start': source_vcl['source_start'],
                'source_vcl_body_sha256': source_vcl['body_sha256']})
            packet.opaque = tag
            for frame in ctx.decode(packet):
                receive(frame)
        for frame in ctx.decode(None):
            receive(frame)
        if pending:
            raise ValueError('video_lineage_pictures_not_decoded')
    expected = {n['source_start'] for n in mapping if n['first_slice']}
    actual = [r['source_first_slice_byte_offset'] for r in records]
    if len(actual) != len(set(actual)) or set(actual) != expected:
        raise ValueError('video_lineage_decoded_picture_set_mismatch')
    if set(outputs) != wanted:
        raise ValueError('video_lineage_requested_picture_not_decoded')
    return {'format': 'neo2-control/video-frame-lineage/v1', 'source': config['source'],
        'preparation': config['preparation'], 'normalization_nal_map': mapping,
        'decoder': {'pyav': av.__version__, 'libraries': av.library_versions,
            'copy_opaque': True, 'thread_count': 2, 'thread_type': 'SLICE',
            'error_detection': 'explode', 'hardware_decoding': False},
        'summary': {'decoded_frames': len(records), 'selected_images': len(outputs),
            'raw_picture_candidates': len(config['pictures']), 'packets': seen_packets,
            'decoded_packet_identity_verified': True, 'all_vcl_bodies_byte_identical': True,
            'pictureless_packets': pictureless}, 'frames': records,
        'clock_unit_verified': False, 'exposure_semantics_verified': False,
        'pc_clock_alignment_verified': False, 'live_preview_identity_verified': False,
        'flight_authorized': False}


if __name__ == '__main__':
    if sys.argv[1:] != ['--worker']:
        raise SystemExit('internal worker; use neo2ctl video-lineage')
    try:
        path = Path('work/input.json')
        if path.stat().st_size > MAX_JSON_BYTES:
            raise ValueError('video_lineage_json_limit')
        _json_new('result.json', _decode(json.loads(path.read_bytes())))
        print(json.dumps({'decoded': True}))
    except Exception as error:
        print(f'video_lineage_worker: {type(error).__name__}: {error}', file=sys.stderr)
        raise SystemExit(1)
