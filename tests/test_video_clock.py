"""Source-byte clock evidence: framing, split tags and deliberately unknown time."""
import hashlib
import io
import json
import os

import pytest

from neo2_control import video_clock
from neo2_control.cli import main
from neo2_control.video_clock import inventory_hevc_clocks


def nal(kind, payload=b'\x80', prefix=b'\0\0\0\1'):
    return prefix + bytes([kind << 1, 1]) + payload


def escaped(raw):
    output = bytearray()
    zeros = 0
    for value in raw:
        if zeros >= 2 and value <= 3:
            output.append(3)
            zeros = 0
        output.append(value)
        zeros = zeros + 1 if value == 0 else 0
    return bytes(output)


def extended(value):
    return b'\xff' * (value // 255) + bytes([value % 255])


def sei(messages, kind=40):
    raw = b''.join(extended(kind) + extended(len(payload)) + payload
        for kind, payload in messages) + b'\x80'
    return nal(kind, escaped(raw))


def payload256(tick):
    p = bytearray(92)
    p[:2] = b'\x81\xca'
    p[2:4] = (tick & 65535).to_bytes(2, 'little')
    p[4] = 255
    p[5:7] = (tick >> 16).to_bytes(2, 'little')
    p[7:11] = b'\x61\x04\xbe\x97'
    return bytes(p)


def payload240(tick, tag=240):
    p = bytearray(25)
    p[15:17] = (tick & 65535).to_bytes(2, 'little')
    p[17] = tag
    p[18:20] = (tick >> 16).to_bytes(2, 'little')
    p[20:23] = b'\xc0\x00\x8f'
    return bytes(p)


def picture(tick, *, tag=240):
    return (nal(1) + sei([(256, payload256(tick))])
        + sei([(240, payload240(tick + 10, tag))]) + nal(35))


def report(tmp_path, raw):
    source = tmp_path / 'original.hevc'
    source.write_bytes(raw)
    result = inventory_hevc_clocks(source)
    assert source.read_bytes() == raw
    assert result['source']['sha256'] == hashlib.sha256(raw).hexdigest()
    assert result['source']['bytes'] == len(raw)
    for key in ('clock_unit_verified', 'exposure_semantics_verified',
            'pc_clock_alignment_verified', 'decoded_frame_association_verified', 'flight_authorized'):
        assert result[key] is False
    assert result['tick_unit'] is None
    return result


def test_all_chunk_sizes_preserve_original_offsets_and_unescape_only_payload(monkeypatch):
    raws = [nal(1, b'\x80\0\0\3\x01', b'\0\0\1'), nal(35), nal(40, b'\x80') + b'\0\0']
    raw = b'\0'*7 + b''.join(raws)
    # A four-byte start code consumes the final leading zero before a three-byte prefix.
    starts = [6, 7 + len(raws[0]), 7 + len(raws[0]) + len(raws[1])]
    ends = starts[1:] + [len(raw)]
    for chunk_size in range(1, len(raw) + 1):
        monkeypatch.setattr(video_clock, 'CHUNK_BYTES', chunk_size)
        digest = hashlib.sha256()
        units = list(video_clock._units(io.BytesIO(raw), digest))
        assert [(start, end) for start, end, _ in units] == list(zip(starts, ends))
        assert [body for _, _, body in units] == [b'\x02\x01\x80\0\0\3\x01', b'\x46\x01\x80', b'\x50\x01\x80']
        assert digest.hexdigest() == hashlib.sha256(raw).hexdigest()


def test_split_u16_wrap_is_continuous_and_both_240_tags_work(tmp_path):
    raw = picture(65520) + picture(65553, tag=241) + picture(65587)
    result = report(tmp_path, raw)
    summary = result['summary']
    assert summary['candidate_pairs'] == 3
    assert summary['pair_difference_histogram'] == [{'value': 10, 'count': 3}]
    for field in summary['candidate_fields']:
        assert field['last_minus_first_ticks'] == 67
        assert field['increment_histogram'] == [{'value': 33, 'count': 1}, {'value': 34, 'count': 1}]
        assert not field['nonpositive_increments']
    assert summary['candidate_fields'][1]['tag_histogram'] == [{'value': 240, 'count': 2}, {'value': 241, 'count': 1}]
    assert not result['pictures'][0]['after_parameter_bootstrap']
    for p in result['pictures']:
        for f in p['candidate_fields']:
            original_nal = raw[f['nal_offset'] + 4:f['nal_end_offset']]
            assert video_clock._messages(original_nal) == [(f['payload_type'], bytes.fromhex(f['payload_hex']))]


def test_bootstrap_is_a_structural_label_and_does_not_rename_raw_ordinals(tmp_path):
    before = picture(100)
    headers = nal(32) + nal(33) + nal(34)
    opening = nal(20) + sei([(256, payload256(133)), (240, payload240(143))])
    result = report(tmp_path, before + headers + opening + nal(35) + picture(166))
    assert result['summary']['raw_picture_candidates'] == 3
    assert result['summary']['pictures_after_parameter_bootstrap'] == 2
    assert [p['after_parameter_bootstrap'] for p in result['pictures']] == [False, True, True]
    assert [p['raw_picture_ordinal'] for p in result['pictures']] == [0, 1, 2]
    assert result['pictures'][1]['first_slice_byte_offset'] == len(before + headers)


def test_source_ticks_reset_or_stall_are_reported_without_wrap_correction(tmp_path):
    result = report(tmp_path, picture(4_294_967_280) + picture(20) + picture(20) + picture(53))
    summary = result['summary']['candidate_fields'][0]
    assert summary['nonpositive_increments'] == 2
    assert summary['increment_histogram'][0]['value'] == -4_294_967_260
    assert summary['wrap_or_reset_correction_applied'] is False


def test_broken_nal_does_not_publish_earlier_well_bounded_message(tmp_path):
    # A valid candidate followed by a malformed message in the same SEI.
    raw = extended(256) + extended(92) + payload256(123) + b'\xf5\x07' + b'A'*6 + b'\x80'
    broken = nal(40, escaped(raw))
    result = report(tmp_path, nal(1) + broken + sei([(240, payload240(133))]))
    assert result['summary']['candidate_fields'][0]['count'] == 0
    assert result['summary']['candidate_pairs'] == 0
    assert result['malformed_sei'] == [{'nal_offset': len(nal(1)),
        'nal_end_offset': len(nal(1) + broken), 'nal_type': 40, 'reason': 'payload_overruns_rbsp_stop'}]


def test_duplicate_pairs_and_aud_or_prefix_orphans_remain_explicit(tmp_path):
    raw = (nal(1) + sei([(256, payload256(100)), (256, payload256(100)), (240, payload240(110))])
        + nal(35) + sei([(256, payload256(133))])
        + nal(1) + sei([(256, payload256(166))], kind=39) + sei([(240, payload240(176))]))
    result = report(tmp_path, raw)
    assert result['summary']['candidate_pairs'] == 0
    assert result['summary']['unassociated_candidate_fields'] == 3
    assert len(result['pictures'][0]['candidate_fields']) == 3
    assert result['pictures'][1]['candidate_fields'] == []


@pytest.mark.parametrize('kind,index,value', [(256, 0, 0), (256, 4, 0), (256, 8, 0),
    (240, 17, 0xff), (240, 20, 0), (240, 24, 1)])
def test_changed_layout_is_counted_without_guessing(tmp_path, kind, index, value):
    payload = bytearray(payload256(100) if kind == 256 else payload240(100))
    payload[index] = value
    result = report(tmp_path, nal(1) + sei([(kind, bytes(payload))]))
    assert result['summary']['unrecognized_vendor_layouts'] == {kind: 1}
    assert result['pictures'][0]['candidate_fields'] == []


@pytest.mark.parametrize('raw', [b'', b'not annex b', b'\0'*10,
    b'\0\0\1\0\0\1\x46\x01\x80', b'\0\0\1\x50\0\x80',
    b'\0\0\1\xd0\x01\x80', b'\0\0\1\x02\x09\x80', b'\0\0\1\x02\x01'])
def test_invalid_or_multilayer_stream_fails_explicitly(tmp_path, raw):
    with pytest.raises(ValueError):
        report(tmp_path, raw)


def test_memory_and_record_limits_fail_explicitly(tmp_path, monkeypatch):
    monkeypatch.setattr(video_clock, 'MAX_PICTURES', 1)
    with pytest.raises(ValueError, match='picture_limit'):
        report(tmp_path, picture(100) + picture(133))
    monkeypatch.setattr(video_clock.HevcAnnexBFilter, 'MAX_NAL_BYTES', 8)
    with pytest.raises(ValueError, match='nal_limit'):
        report(tmp_path, nal(1, b'Z'*20))
    monkeypatch.setattr(video_clock, 'MAX_SOURCE_BYTES', 5)
    with pytest.raises(ValueError, match='bounded_regular'):
        report(tmp_path, picture(100))


def test_source_changed_during_read_is_rejected(tmp_path, monkeypatch):
    source = tmp_path / 'original.hevc'
    source.write_bytes(picture(100))
    original = video_clock._units
    def changing(stream, digest):
        yield from original(stream, digest)
        source.write_bytes(picture(133))
    monkeypatch.setattr(video_clock, '_units', changing)
    with pytest.raises(ValueError, match='source_changed'):
        inventory_hevc_clocks(source)


def test_symlinks_and_nonregular_sources_are_rejected_without_blocking(tmp_path):
    source = tmp_path / 'real.hevc'
    source.write_bytes(picture(100))
    link = tmp_path / 'link.hevc'
    link.symlink_to(source)
    with pytest.raises(OSError):
        inventory_hevc_clocks(link)
    fifo = tmp_path / 'fifo'
    os.mkfifo(fifo)
    with pytest.raises(ValueError, match='bounded_regular'):
        inventory_hevc_clocks(fifo)


def test_cli_writes_new_private_json_and_refuses_overwrite(tmp_path, capsys):
    source = tmp_path / 'original.hevc'
    source.write_bytes(picture(65553, tag=241))
    output = tmp_path / 'report.json'
    args = ['video-clock-report', str(source), '-o', str(output)]
    assert main(args) == 0
    published = json.loads(output.read_text())
    brief = json.loads(capsys.readouterr().out)
    assert published['summary'] == brief['summary']
    assert 'pictures' not in brief and len(published['pictures']) == 1
    assert output.stat().st_mode & 0o777 == 0o600
    original = output.read_bytes()
    assert main(args) == 1
    assert output.read_bytes() == original


def checksummed_interval(raw, tick):
    payload = bytearray(payload240(tick + 10))
    checksum = sum(raw) & 0xffffffff
    payload[9:11] = (checksum & 65535).to_bytes(2, 'little')
    payload[11] = 255
    payload[12:14] = (checksum >> 16).to_bytes(2, 'little')
    return raw + sei([(256, payload256(tick))]) + sei([(240, bytes(payload))])


@pytest.mark.parametrize('size', [1, 300, 70000])
def test_checksum_binds_original_escaped_interval_with_parameters_and_other_seis(tmp_path, size):
    # This range includes parameters, the original EBSP03, a malformed SEI and
    # vendor filler. Removing any of them would no longer match the metadata.
    headers = nal(32) + nal(33) + nal(34)
    vcl = nal(20, b'\x80\0\0\x03\x01' + b'\xff'*size, prefix=b'\0\0\1')
    malformed = bytes.fromhex('000000015001f5070300020000030080')
    interval = headers + vcl + malformed + sei([(508, b'\xff'*3)]) + b'\0\0'
    result = report(tmp_path, nal(35) + checksummed_interval(interval, 65553))
    check = result['pictures'][0]['encoded_interval_checksum']
    assert check['available'] and check['matched'] is True
    assert not check['native_zero_bypass']
    assert check['byte_offset_start'] == len(nal(35))
    assert check['byte_offset_end'] == len(nal(35) + interval)
    assert check['computed_from_original_bytes'] == sum(interval)
    assert result['summary']['encoded_interval_checksums'] == {
        'available': 1, 'matched': 1, 'mismatched': 0, 'native_zero_bypass': 0}
    assert result['summary']['malformed_sei_nals'] == 1


def test_checksum_detects_changed_compressed_bytes_and_resets_at_each_aud(tmp_path):
    first = checksummed_interval(nal(1, b'\x80pictureA'), 100)
    second = checksummed_interval(nal(1, b'\x80pictureB'), 133)
    raw = first + nal(35) + second
    valid = report(tmp_path, raw)
    assert valid['summary']['encoded_interval_checksums']['matched'] == 2
    assert valid['pictures'][1]['encoded_interval_checksum']['byte_offset_start'] == len(first + nal(35))
    modified = bytearray(raw)
    modified[8] ^= 1
    corrupted = report(tmp_path, bytes(modified))
    assert corrupted['summary']['encoded_interval_checksums']['mismatched'] == 1
    assert corrupted['summary']['encoded_interval_checksums']['matched'] == 1
    assert corrupted['pictures'][0]['candidate_fields'] == valid['pictures'][0]['candidate_fields']


def test_zero_checksum_is_native_bypass_and_cannot_report_a_verified_match(tmp_path):
    result = report(tmp_path, picture(100))
    check = result['pictures'][0]['encoded_interval_checksum']
    assert check['available'] and check['native_zero_bypass']
    assert check['matched'] is None
    assert result['summary']['encoded_interval_checksums']['matched'] == 0


def test_reversed_or_same_nal_metadata_does_not_establish_checksum_boundary(tmp_path):
    for metadata in (sei([(240, payload240(110))]) + sei([(256, payload256(100))]),
            sei([(256, payload256(100)), (240, payload240(110))])):
        result = report(tmp_path, nal(1) + metadata)
        assert result['summary']['candidate_pairs'] == 1
        assert result['summary']['encoded_interval_checksums']['available'] == 0
        assert result['pictures'][0]['encoded_interval_checksum'] == {
            'available': False, 'reason': 'encoded_interval_boundary_unestablished'}
