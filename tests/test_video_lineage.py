"""Actual codec output, reordered B pictures and source-byte lineage."""
import hashlib
import re
import shutil
import subprocess

import pytest

from neo2_control.hevc import HevcAnnexBFilter
from neo2_control import video_lineage
from neo2_control.video_lineage import (_TrackedFilter, _normalization_map,
    build_video_lineage, MAX_SELECTED)


def nal(kind, body=b'\x80', prefix=b'\0\0\0\1'):
    return prefix + bytes([kind << 1, 1]) + body


def escape(rbsp):
    result = bytearray()
    zeros = 0
    for value in rbsp:
        if zeros >= 2 and value <= 3:
            result.append(3)
            zeros = 0
        result.append(value)
        zeros = zeros + 1 if value == 0 else 0
    return bytes(result)


def metadata(index, checksum):
    a, b = bytearray(92), bytearray(25)
    tick = 65_520 + index * 33
    a[:2], a[4], a[7:11] = b'\x81\xca', 255, b'\x61\x04\xbe\x97'
    a[2:4], a[5:7] = (tick & 65535).to_bytes(2, 'little'), (tick >> 16).to_bytes(2, 'little')
    tick += 10
    b[15:17], b[18:20] = (tick & 65535).to_bytes(2, 'little'), (tick >> 16).to_bytes(2, 'little')
    b[17], b[20:23] = 240, b'\xc0\x00\x8f'
    b[9:11], b[12:14] = (checksum & 65535).to_bytes(2, 'little'), (checksum >> 16).to_bytes(2, 'little')
    b[11] = 255
    return nal(40, escape(b'\xff\x01\x5c' + a + b'\x80')) + nal(40, escape(b'\xf0\x19' + b + b'\x80'))


def decorate(raw):
    markers = list(re.finditer(b'\0\0\0?\1', raw))
    out = bytearray()
    index = checksum = 0
    for i, match in enumerate(markers):
        end = markers[i + 1].start() if i + 1 < len(markers) else len(raw)
        unit = raw[match.start():end]
        body = raw[match.end():end]
        kind = (body[0] >> 1) & 63
        out.extend(unit)
        checksum = 0 if kind == 35 else (checksum + sum(unit)) & 0xffffffff
        if kind <= 31 and body[2] & 128:
            suffix = metadata(index, checksum)
            out.extend(suffix)
            checksum += sum(suffix)
            index += 1
    assert index == 9
    return bytes(out)


@pytest.fixture(scope='module')
def encoded(tmp_path_factory):
    av = pytest.importorskip('av')
    if not all(shutil.which(name) for name in ('ffmpeg', 'prlimit')):
        pytest.skip('FFmpeg and prlimit required')
    folder = tmp_path_factory.mktemp('lineage-fixture')
    baseline = folder/'baseline.hevc'
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'lavfi',
        '-i', 'testsrc2=size=128x96:rate=6', '-frames:v', '9', '-c:v', 'libx265',
        '-preset', 'ultrafast', '-threads', '1', '-x265-params',
        'pools=none:frame-threads=1:wpp=0:bframes=2:b-adapt=0:keyint=30:repeat-headers=1:aud=1:log-level=error',
        '-f', 'hevc', str(baseline)], check=True, capture_output=True, timeout=20)
    source = folder/'with-vendor-sei.hevc'
    source.write_bytes(decorate(baseline.read_bytes()))
    # Independent decoding of the encoder output, without the injected SEI or
    # the lineage/normalization path. Keep display order and packed RGB hashes.
    hashes = []
    with av.open(str(baseline), format='hevc') as container:
        container.streams.video[0].codec_context.thread_count = 1
        for frame in container.decode(video=0):
            rgb = frame.to_rgb()
            plane = memoryview(rgb.planes[0])
            packed = b''.join(plane[y*rgb.planes[0].line_size:y*rgb.planes[0].line_size+3*rgb.width]
                for y in range(rgb.height))
            hashes.append(hashlib.sha256(packed).hexdigest())
    assert len(hashes) == 9 and len(set(hashes)) == 9
    return source, hashes


def test_decoder_opaque_identity_survives_b_frame_reordering_and_png_roundtrip(encoded, tmp_path):
    source, reference_hashes = encoded
    original = source.read_bytes()
    result = build_video_lineage(source, tmp_path/'result', pictures=[0, 4, 8], timeout=30)
    assert source.read_bytes() == original
    assert result['source']['sha256'] == hashlib.sha256(original).hexdigest()
    assert result['summary']['decoded_frames'] == 9
    assert result['summary']['decoded_packet_identity_verified']
    assert result['summary']['all_vcl_bodies_byte_identical']
    assert [f['pixels']['sha256'] for f in result['frames']] == reference_hashes
    assert 'B' in [f['picture_type'] for f in result['frames']]
    assert any(f['decoded_index'] != f['raw_picture_ordinal'] for f in result['frames'])
    assert {f['raw_picture_ordinal'] for f in result['frames']} == set(range(9))
    for frame in result['frames']:
        index = frame['raw_picture_ordinal']
        values = {c['payload_type']: c['split_tick_candidate'] for c in frame['candidate_fields']}
        assert values == {256: 65_520 + index*33, 240: 65_530 + index*33}
        assert frame['encoded_interval_checksum']['matched']
        if 'image' in frame:
            image = frame['image']
            saved = tmp_path/'result'/image['file']
            assert hashlib.sha256(saved.read_bytes()).hexdigest() == image['sha256']
            assert image['decoded_rgb_sha256'] == frame['pixels']['sha256']
            assert saved.stat().st_mode & 0o777 == 0o600
    assert (tmp_path/'result').stat().st_mode & 0o777 == 0o700
    assert not (tmp_path/'result/work').exists()
    assert (tmp_path/'result/manifest.json').is_file()
    for key in ('clock_unit_verified', 'exposure_semantics_verified', 'pc_clock_alignment_verified',
            'live_preview_identity_verified', 'flight_authorized'):
        assert result[key] is False


def test_existing_output_is_never_reused_or_overwritten(encoded, tmp_path):
    source, _ = encoded
    output = tmp_path/'existing'
    output.mkdir()
    (output/'keep').write_text('preserve')
    with pytest.raises(FileExistsError):
        build_video_lineage(source, output)
    assert (output/'keep').read_text() == 'preserve'


def test_undecodable_requested_prebootstrap_picture_leaves_no_success_manifest(encoded, tmp_path):
    source, _ = encoded
    raw = tmp_path/'prebootstrap.hevc'
    raw.write_bytes(nal(1) + source.read_bytes())
    output = tmp_path/'failed'
    with pytest.raises(RuntimeError, match='lineage_decode_failed'):
        build_video_lineage(raw, output, pictures=[0], timeout=30)
    assert not (output/'manifest.json').exists()
    assert not (output/'work').exists()
    assert (output/'failure.json').is_file()
    assert (output/'failure.json').stat().st_mode & 0o777 == 0o600


def test_original_changed_after_decoding_prevents_success_publication(encoded, tmp_path, monkeypatch):
    source, _ = encoded
    mutable = tmp_path/'mutable.hevc'
    mutable.write_bytes(source.read_bytes())
    run = video_lineage._run
    def mutate_after_decode(command, **options):
        result = run(command, **options)
        if options['stage'] == 'lineage_decode':
            with mutable.open('ab') as stream:stream.write(b'\0')
        return result
    monkeypatch.setattr(video_lineage, '_run', mutate_after_decode)
    output = tmp_path/'changed-source'
    with pytest.raises(ValueError, match='video_lineage_source_changed'):
        build_video_lineage(mutable, output, timeout=30)
    assert not (output/'manifest.json').exists()
    assert (output/'failure.json').is_file()
    assert not (output/'work').exists()


@pytest.mark.parametrize('chunk', [1, 2, 3, 7, 65536])
def test_tracked_filter_emits_exact_existing_filter_bytes_and_source_ranges(chunk):
    raw = (b'\0'*7 + nal(1) + nal(32, b'old') + nal(35) + nal(33) + nal(32)
        + nal(34) + nal(20) + metadata(0, 12345) + nal(35) + nal(1, prefix=b'\0\0\1'))
    tracked = _TrackedFilter()
    reference = HevcAnnexBFilter(wait_for_bootstrap=True, strip_sei=True)
    actual, expected = [], []
    for start in range(0, len(raw), chunk):
        data = raw[start:start + chunk]
        actual.append(tracked.feed(data))
        expected.append(reference.feed(data))
    actual.append(tracked.finish());expected.append(reference.finish())
    prepared = b''.join(actual)
    assert prepared == b''.join(expected)
    assert tracked.consumed == len(raw)
    assert tracked.prepared_bytes == len(prepared)
    assert b''.join(raw[r['source_start']:r['source_end']] for r in tracked.records) == prepared
    for r in tracked.records:
        assert raw[r['source_start']:r['source_end']] == prepared[r['prepared_start']:r['prepared_end']]
    assert [r['nal_type'] for r in tracked.records] == [32, 33, 34, 20, 35, 1]


@pytest.mark.parametrize('mutate', ['vcl', 'pps', 'order', 'added', 'removed'])
def test_normalization_map_rejects_changed_pixels_or_ambiguous_nal_lineage(tmp_path, mutate):
    pieces = [nal(32), nal(33), nal(34), nal(20)]
    tracked = _TrackedFilter()
    tracked.feed(b''.join(pieces));tracked.finish()
    if mutate == 'vcl':pieces[-1] = nal(20, b'\x81')
    elif mutate == 'pps':pieces[2] = nal(34, b'\x81')
    elif mutate == 'order':pieces[0], pieces[1] = pieces[1], pieces[0]
    elif mutate == 'added':pieces.append(nal(35))
    elif mutate == 'removed':pieces.pop()
    path = tmp_path/'normalized.hevc';path.write_bytes(b''.join(pieces))
    with pytest.raises(ValueError, match='video_lineage_normalizer_'):
        _normalization_map(path, tracked.records)


def test_timing_parameter_changes_keep_explicit_hashes_and_original_vcl_binding(tmp_path):
    pieces = [nal(32), nal(33), nal(34), nal(20)]
    tracked = _TrackedFilter();tracked.feed(b''.join(pieces));tracked.finish()
    pieces[:2] = [nal(32, b'new-timing'), nal(33, b'other-timing', prefix=b'\0\0\1')]
    path = tmp_path/'normalized.hevc';path.write_bytes(b''.join(pieces))
    mapping, report = _normalization_map(path, tracked.records)
    assert report['rewritten_parameter_nals'] == [0, 1]
    assert mapping[-1]['body_sha256'] == mapping[-1]['normalized_body_sha256']
    assert mapping[-1]['normalized_start'] != mapping[-1]['source_start']


@pytest.mark.parametrize('pictures', [[-1], [True], [0, 0], list(range(MAX_SELECTED+1)), '1'])
def test_invalid_selections_fail_before_creating_output(tmp_path, pictures):
    with pytest.raises(ValueError, match='invalid_picture_selection'):
        build_video_lineage(tmp_path/'missing.hevc', tmp_path/'absent', pictures=pictures)
    assert not (tmp_path/'absent').exists()


@pytest.mark.parametrize('options', [{'timeout': 0}, {'timeout': float('inf')}, {'nominal_fps': float('nan')},
    {'maximum_selected':0},{'maximum_selected':True},{'maximum_selected':97}])
def test_invalid_bounds_fail_before_creating_output(tmp_path, options):
    with pytest.raises(ValueError, match='video_lineage_invalid_'):
        build_video_lineage(tmp_path/'missing.hevc', tmp_path/'absent', **options)
    assert not (tmp_path/'absent').exists()
