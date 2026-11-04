"""Byte preservation and bounded incremental framing of opaque HEVC NALs."""
import pytest

from neo2_control.hevc import HevcAnnexBFilter


P3 = b'\0\0\1'
P4 = b'\0\0\0\1'


def nal(kind, body=b'opaque', prefix=P4):
    return prefix + bytes([kind << 1, 1]) + body


def extended(value):
    return b'\xff' * (value // 255) + bytes([value % 255])


def escaped(rbsp):
    result = bytearray()
    zeros = 0
    for value in rbsp:
        if zeros >= 2 and value <= 3:
            result.append(3)
            zeros = 0
        result.append(value)
        zeros = zeros + 1 if value == 0 else 0
    return bytes(result)


def sei(messages, *, kind=40, prefix=P4):
    body = b''.join(extended(kind) + extended(len(payload)) + payload for kind, payload in messages) + b'\x80'
    return nal(kind, escaped(body), prefix)


def incompatible_sei():
    # Synthetic25-byte payload: an unescaped literal03 follows two zeros.
    # Generic payload bounds remain valid, but FFmpeg removes the03 as an
    # emulation-prevention byte and then rejects the advertised payload size.
    return nal(40, b'\xf0\x19' + b'A'*8 + b'\0\0\3\x7f' + b'B'*13 + b'\x80')


def filter_parts(parts, **options):
    f = HevcAnnexBFilter(**options)
    output = b''.join(f.feed(part) for part in parts) + f.finish()
    return f, output


def test_every_split_preserves_mixed_prefix_nals_and_only_drops_real_invalid245():
    good = nal(32, b'vps') + nal(1, b'picture', P3)
    # Exact real malformed NAL, synthetic test has no image/device identity.
    bad = bytes.fromhex('000000015001f5070300020000030080')
    tail = sei([(240, b'opaque-metadata')], prefix=P3) + nal(35, b'\x30')
    source, expected = good + bad + tail, good + tail
    for split in range(len(source) + 1):
        f, output = filter_parts([source[:split], source[split:]])
        assert output == expected
        assert f.input_bytes == len(source) and f.output_bytes == len(expected)
        assert f.dropped_sei_nals == 1 and f.dropped_sei_bytes == 16
        assert f.buffered_bytes == 0
    f, output = filter_parts([bytes([value]) for value in source])
    assert output == expected and f.dropped_sei_nals == 1


def test_extended_type_size_multiple_messages_and_emulation_prevention_are_opaque():
    metadata = sei([(511, bytes(300)), (128, b'\0\0\1\0\0\2\0\0\3\0\0\0')], kind=39)
    source = metadata + sei([(240, b'\0\0\x02')]) + nal(1)
    for split in range(len(metadata) + 1):
        f, output = filter_parts([source[:split], source[split:]])
        assert output == source and f.dropped_sei_nals == 0


def test_noncanonical_real_metadata_escape_is_preserved_not_rewritten():
    raw = bytes.fromhex('000000015001f0190a004800bf4c000002d058ff00008f9eb0f00100c0008fd60080')
    f, output = filter_parts([raw])
    assert output == raw and not f.dropped_sei_nals


def test_preview_strip_is_explicit_and_preserves_every_non_sei_byte_at_all_splits():
    prefix = sei([(5, b'caption'), (511, b'\0\0\1')], kind=39, prefix=P3)
    suffix = incompatible_sei() + b'\0\0'
    pictures = [nal(32), nal(33, prefix=P3), nal(34), nal(20), nal(1)]
    raw = b'\0'*7 + pictures[0] + prefix + b''.join(pictures[1:4]) + suffix + pictures[4]
    default, output = filter_parts([raw])
    assert output == raw and default.dropped_sei_nals == 0
    for split in range(len(raw)+1):
        f, output = filter_parts([raw[:split], raw[split:]], strip_sei=True)
        assert output == b'\0'*7 + b''.join(pictures)
        assert (f.dropped_sei_nals, f.dropped_sei_bytes) == (2, len(prefix)+len(suffix))
        assert f.input_bytes == f.output_bytes + f.dropped_sei_bytes
        assert f.buffered_bytes == 0
    _, output = filter_parts([bytes([x]) for x in raw], strip_sei=True)
    assert output == b'\0'*7 + b''.join(pictures)


def test_preview_strip_keeps_current_bootstrap_and_still_enforces_nal_bound():
    headers = nal(32)+nal(33)+nal(34)
    picture = nal(20)
    metadata = sei([(240, b'preview-only')], kind=39)+incompatible_sei()
    f, output = filter_parts([headers+metadata+picture], strip_sei=True, wait_for_bootstrap=True)
    assert output == headers+picture and f.bootstrap_ready
    assert f.bootstrap_discarded_bytes == 0 and f.dropped_sei_nals == 2
    with pytest.raises(ValueError, match='byte bound'):
        filter_parts([metadata], strip_sei=True, max_nal_bytes=8)


@pytest.mark.parametrize('option', [1, None, 'yes'])
def test_preview_strip_configuration_requires_boolean(option):
    with pytest.raises(ValueError, match='SEI removal option'):
        HevcAnnexBFilter(strip_sei=option)


@pytest.mark.parametrize('body', [
    b'', b'\x80',                  # Missing message.
    b'\xff\x80',                 # Truncated extended payloadType.
    b'\x01\xff\x80',            # Truncated extended payloadSize.
    b'\x01\x02A\x80',           # Claimed payload consumes the RBSP stop.
    b'\x01\x01A',                 # Missing rbsp_trailing_bits.
    b'\x01\x01A\x40',           # Non-byte-aligned/invalid trailing bits.
    b'\x01\x00\x80\x40',       # Bytes after valid trailing bits.
])
@pytest.mark.parametrize('kind', [39, 40])
def test_structurally_invalid_sei_is_removed_without_damaging_neighbor_nals(body, kind):
    left, bad, right = nal(33), nal(kind, body, P3), nal(1)
    f, output = filter_parts([left + bad + right])
    assert output == left + right
    assert (f.dropped_sei_nals, f.dropped_sei_bytes) == (1, len(bad))


def test_sei_header_errors_are_removed_but_non_sei_payload_syntax_is_not_interpreted():
    source = P4 + b'\x50\x00\x01\x00\x80' + nal(1, b'not-a-valid-picture')
    f, output = filter_parts([source])
    assert output == nal(1, b'not-a-valid-picture')
    assert f.dropped_sei_nals == 1


def test_leading_trailing_annex_b_zeros_and_empty_message_payload_preserve_bytes():
    source = b'\0' * 13 + sei([(255, b'')], prefix=P3) + b'\0' * 5 + nal(1, b'x', P3) + b'\0\0'
    f, output = filter_parts([source[:8], source[8:9], source[9:]])
    assert output == source and not f.dropped_sei_nals


def test_four_megabyte_nal_is_accepted_with_bounded_fragmented_lookbehind():
    f = HevcAnnexBFilter()
    source = nal(1, b'X' * (f.MAX_NAL_BYTES - 2))
    output = []
    for i in range(0, len(source), f.MAX_CHUNK_BYTES):
        output.append(f.feed(source[i:i + f.MAX_CHUNK_BYTES]))
        assert f.buffered_bytes <= f.MAX_NAL_BYTES + 7
    for byte in P4:
        output.append(f.feed(bytes([byte])))
    output.append(f.feed(b'\x46\x01\x30'))
    output.append(f.finish())
    assert b''.join(output) == source + nal(35, b'\x30')


def test_oversize_nal_and_chunk_fail_explicitly_and_release_buffer():
    f = HevcAnnexBFilter(max_nal_bytes=8)
    with pytest.raises(ValueError, match='byte bound'):
        f.feed(nal(1, b'X' * 7))
    assert f.buffered_bytes == 0
    with pytest.raises(ValueError, match='finished or failed'):
        f.finish()
    with pytest.raises(ValueError, match='65536'):
        HevcAnnexBFilter().feed(bytes(65537))


@pytest.mark.parametrize('source', [b'garbage', b'\0\1', P4 + b'\x50', P3 + P4 + b'\x46\x01\x30'])
def test_framing_errors_are_not_silently_discarded(source):
    f = HevcAnnexBFilter()
    with pytest.raises(ValueError):
        f.feed(source)
        f.finish()
    assert f.buffered_bytes == 0


def test_empty_legacy_input_finishes_once_and_accepts_no_further_input():
    f = HevcAnnexBFilter()
    assert f.feed(b'') == f.finish() == b''
    assert (f.input_bytes, f.output_bytes, f.dropped_sei_nals) == (0, 0, 0)
    for action in (lambda: f.finish(), lambda: f.feed(b'')):
        with pytest.raises(ValueError, match='finished or failed'):
            action()


@pytest.mark.parametrize('limit', [0, 1, 4194305, True, 3.0])
def test_bad_configuration_is_rejected(limit):
    with pytest.raises(ValueError):
        HevcAnnexBFilter(max_nal_bytes=limit)


def test_bootstrap_every_fragmentation_waits_for_current_parameter_sets_and_new_irap():
    orphan = nal(1,b'orphan') + sei([(240,b'old-metadata')]) + nal(20,b'no-parameter-sets')
    previous = nal(32,b'old-vps')
    vps,sps,pps = nal(32,b'vps'),nal(33,b'sps',P3),nal(34,b'pps')
    irap,tail = nal(20,b'new-reference',P3),sei([(240,b'current-metadata')])+nal(1,b'next-picture')
    bad = bytes.fromhex('000000015001f5070300020000030080')
    source = b'\0'*7 + orphan + previous + vps+sps+pps + bad + irap+tail
    expected = vps+sps+pps+irap+tail
    for split in range(len(source)+1):
        f,output=filter_parts([source[:split],source[split:]],wait_for_bootstrap=True)
        assert output==expected
        assert f.bootstrap_ready and f.bootstrap_opening_nal_type==20
        assert f.bootstrap_seen_parameter_sets=={32,33,34}
        assert f.bootstrap_discarded_bytes==len(source)-len(expected)-len(bad)
        assert f.bootstrap_discarded_nals==4  # Three orphan units and replaced VPS.
        assert f.dropped_sei_nals==1 and f.dropped_sei_bytes==len(bad)
        assert f.bootstrap_buffered_bytes==f.buffered_bytes==0
    _,output=filter_parts([bytes([x]) for x in source],wait_for_bootstrap=True)
    assert output==expected


def test_parameter_sets_alone_never_open_bootstrap_or_leak_between_instances():
    source=nal(32)+nal(33)+nal(34)+nal(1)+nal(35)
    f=HevcAnnexBFilter(wait_for_bootstrap=True)
    assert f.feed(source)==b''
    assert not f.bootstrap_ready and f.bootstrap_seen_parameter_sets=={32,33,34}
    assert f.finish()==b'' and f.bootstrap_discarded_bytes==len(source)
    assert f.bootstrap_buffered_bytes==0
    other,output=filter_parts([nal(20)+nal(35)],wait_for_bootstrap=True)
    assert output==b'' and not other.bootstrap_ready
    assert not other.bootstrap_seen_parameter_sets


def test_many_orphan_nals_do_not_accumulate_while_waiting_for_bootstrap():
    f=HevcAnnexBFilter(wait_for_bootstrap=True)
    chunk=nal(1,b'X'*120)+nal(35,b'aud')
    for _ in range(2000):
        assert f.feed(chunk)==b''
        assert f.bootstrap_buffered_bytes<=len(chunk)
    assert f.finish()==b''
    assert f.bootstrap_discarded_bytes==f.input_bytes==len(chunk)*2000
    assert f.bootstrap_discarded_nals==4000 and f.output_bytes==0


def test_bootstrap_bound_includes_parameter_cache_and_incomplete_nal(monkeypatch):
    monkeypatch.setattr(HevcAnnexBFilter,'MAX_BOOTSTRAP_BYTES',64)
    f=HevcAnnexBFilter(wait_for_bootstrap=True)
    assert f.feed(nal(32,b'V'*30)+nal(33,b'S'*10))==b''
    assert f.bootstrap_buffered_bytes<64
    with pytest.raises(ValueError,match='bootstrap.*byte bound'):
        f.feed(b'S'*20)
    assert f.bootstrap_buffered_bytes==f.buffered_bytes==0
    with pytest.raises(ValueError,match='finished or failed'):f.finish()


@pytest.mark.parametrize('option',[1,None,'yes'])
def test_bootstrap_configuration_requires_a_boolean(option):
    with pytest.raises(ValueError,match='bootstrap option'):
        HevcAnnexBFilter(wait_for_bootstrap=option)
