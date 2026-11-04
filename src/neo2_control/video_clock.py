"""Offline inventory of clock-like vendor SEI fields in original HEVC bytes.

The layouts are observations from retained Neo 2 recordings, not a vendor
specification. Values remain ticks: their unit, exposure semantics and alignment
with the PC clock are unverified. An observed encoded-interval byte checksum
corroborates suffix association, without proving decoded image identity. Nothing
here decodes or controls a device.
"""
from __future__ import annotations

from collections import Counter
import hashlib
import os
from pathlib import Path
import stat

from .hevc import HevcAnnexBFilter

MAX_SOURCE_BYTES = 512 * 1024 * 1024
MAX_NALS = 250_000
MAX_PICTURES = 60_000
MAX_ERRORS = 4096
CHUNK_BYTES = 65536


def _units(stream, digest):
    """Yield original [start,end) ranges and NAL bodies with bounded lookbehind."""
    buffer = bytearray()
    offset = scan = total = 0
    prefix = None
    while True:
        chunk = stream.read(CHUNK_BYTES)
        if chunk:
            total += len(chunk)
            if total > MAX_SOURCE_BYTES:
                raise ValueError('video_clock_source_limit')
            digest.update(chunk)
            buffer.extend(chunk)
        while True:
            match = buffer.find(b'\0\0\1', scan)
            if match < 0:
                break
            start = match - 1 if match and buffer[match - 1] == 0 else match
            next_prefix = 4 if start != match else 3
            if prefix is None:
                if any(buffer[:start]):
                    raise ValueError('video_clock_bytes_before_start_code')
            else:
                if start - prefix > HevcAnnexBFilter.MAX_NAL_BYTES:
                    raise ValueError('video_clock_nal_limit')
                yield offset, offset + start, bytes(buffer[prefix:start]).rstrip(b'\0')
            del buffer[:start]
            offset += start
            prefix = next_prefix
            scan = prefix
        if prefix is None:
            if any(buffer):
                raise ValueError('video_clock_missing_start_code')
            # Retain only a possible prefix at the next chunk boundary.
            discard = max(0, len(buffer) - 3)
            del buffer[:discard]
            offset += discard
            scan = 0
        else:
            zeros = min(3, len(buffer) - len(buffer.rstrip(b'\0')))
            if len(buffer) - prefix - zeros > HevcAnnexBFilter.MAX_NAL_BYTES:
                raise ValueError('video_clock_nal_limit')
            scan = max(prefix, len(buffer) - 3)
        if not chunk:
            if prefix is None:
                raise ValueError('video_clock_missing_start_code')
            if len(buffer) - prefix > HevcAnnexBFilter.MAX_NAL_BYTES:
                raise ValueError('video_clock_nal_limit')
            yield offset, total, bytes(buffer[prefix:]).rstrip(b'\0')
            return


def _messages(body):
    """All-or-nothing SEI bounds check; never infer a tick from a broken NAL."""
    rbsp = HevcAnnexBFilter._rbsp(body[2:])
    if not rbsp or rbsp[-1] != 0x80:
        raise ValueError('rbsp_stop_missing')
    cursor = 0
    stop = len(rbsp) - 1
    messages = []
    while cursor < stop:
        values = []
        for field in ('type', 'size'):
            value = 0
            while cursor < stop and rbsp[cursor] == 255:
                value += 255
                cursor += 1
            if cursor == stop:
                raise ValueError(f'{field}_overruns_rbsp_stop')
            value += rbsp[cursor]
            cursor += 1
            values.append(value)
        kind, size = values
        if size > stop - cursor:
            raise ValueError('payload_overruns_rbsp_stop')
        if len(messages) == 64:
            raise ValueError('sei_message_count_limit')
        messages.append((kind, bytes(rbsp[cursor:cursor + size])))
        cursor += size
    if not messages:
        raise ValueError('empty_sei')
    return messages


def _candidate(kind, payload):
    """Recognize only the observed layouts, including their intervening tags."""
    if kind == 256:
        if (len(payload) != 92 or payload[:2] != b'\x81\xca'
                or payload[4] != 0xff or payload[7:11] != b'\x61\x04\xbe\x97'):
            return None
        low, high, tag = 2, 5, 4
    elif kind == 240:
        if (len(payload) != 25 or payload[17] not in (0xf0, 0xf1)
                or payload[20:23] != b'\xc0\x00\x8f' or payload[24] != 0):
            return None
        low, high, tag = 15, 18, 17
    else:
        return None
    value = int.from_bytes(payload[low:low + 2], 'little')
    value += int.from_bytes(payload[high:high + 2], 'little') << 16
    return {'payload_type': kind, 'payload_size': len(payload),
        'payload_hex': payload.hex(), 'split_tick_candidate': value,
        'tag_byte': payload[tag], 'low_u16_offset': low, 'high_u16_offset': high}


def _histogram(values):
    return [{'value': value, 'count': count} for value, count in sorted(values.items())]


def _summary(pictures, kind):
    fields = [field for picture in pictures for field in picture['candidate_fields']
        if field['payload_type'] == kind]
    ticks = [field['split_tick_candidate'] for field in fields]
    deltas = [b - a for a, b in zip(ticks, ticks[1:])]
    return {'payload_type': kind, 'count': len(ticks),
        'first_tick': ticks[0] if ticks else None,
        'last_tick': ticks[-1] if ticks else None,
        'last_minus_first_ticks': ticks[-1] - ticks[0] if ticks else None,
        'nonpositive_increments': sum(value <= 0 for value in deltas),
        'increment_histogram': _histogram(Counter(deltas)),
        'tag_histogram': _histogram(Counter(field['tag_byte'] for field in fields)),
        'wrap_or_reset_correction_applied': False}


def _stamp(info):
    return info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns


def inventory_hevc_clocks(source):
    """Inspect a frozen local elementary stream; return source-bound evidence.

    A picture ordinal means a first-slice NAL in input order. It is deliberately
    not a decoded/display frame number, even when their counts happen to match.
    Suffix fields attach only to the preceding first-slice within the same AUD
    interval. Prefix fields, ambiguous pairs and malformed SEIs stay explicit.
    """
    path = Path(source).absolute()
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK)
    pictures, errors = [], []
    nal_counts, message_counts, rejected = Counter(), Counter(), Counter()
    parameter_sets = set()
    bootstrap = False
    current = None
    orphan_candidates = 0
    interval_start = interval_sum = 0
    checksum_prefixes = {}
    count = 0
    digest = hashlib.sha256()
    with os.fdopen(fd, 'rb') as stream:
        before = os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode) or not 0 < before.st_size <= MAX_SOURCE_BYTES:
            raise ValueError('video_clock_source_not_bounded_regular_file')
        for start, end, body in _units(stream, digest):
            count += 1
            if count > MAX_NALS:
                raise ValueError('video_clock_nal_count_limit')
            if len(body) < 2 or body[0] & 0x80 or not body[1] & 7:
                raise ValueError('video_clock_invalid_nal_header')
            if ((body[0] & 1) << 5) | (body[1] >> 3):
                raise ValueError('video_clock_multilayer_not_supported')
            kind = (body[0] >> 1) & 63
            nal_counts[kind] += 1
            prefix_sum = interval_sum
            if kind == 35:
                interval_start, interval_sum = end, 0
            else:
                # Both Annex-B prefix sizes sum to1; stripped trailing zeros
                # sum to0. This is the ORIGINAL escaped body, not its RBSP.
                interval_sum = (interval_sum + sum(body) + 1) & 0xffffffff
            if kind in (32, 33, 34):
                parameter_sets.add(kind)
                current = None
            elif kind in (35, 36, 37, 39):
                current = None
            if kind <= 31:
                if len(body) < 3:
                    raise ValueError('video_clock_truncated_slice')
                if body[2] & 128:
                    if len(pictures) == MAX_PICTURES:
                        raise ValueError('video_clock_picture_limit')
                    if 16 <= kind <= 23 and len(parameter_sets) == 3:
                        bootstrap = True
                    current = {'raw_picture_ordinal': len(pictures),
                        'first_slice_byte_offset': start, 'first_slice_nal_end_offset': end,
                        'nal_type': kind, 'after_parameter_bootstrap': bootstrap,
                        'candidate_fields': []}
                    pictures.append(current)
            if kind not in (39, 40):
                continue
            try:
                messages = _messages(body)
            except ValueError as error:
                if len(errors) == MAX_ERRORS:
                    raise ValueError('video_clock_error_limit') from None
                errors.append({'nal_offset': start, 'nal_end_offset': end,
                    'nal_type': kind, 'reason': str(error)})
                continue
            for message_type, payload in messages:
                message_counts[(message_type, len(payload))] += 1
                field = _candidate(message_type, payload)
                if field is None:
                    if message_type in (240, 256):
                        rejected[message_type] += 1
                    continue
                if current is None or kind != 40:
                    orphan_candidates += 1
                    continue
                if len(current['candidate_fields']) == 64:
                    raise ValueError('video_clock_picture_field_limit')
                current['candidate_fields'].append({**field,
                    'nal_offset': start, 'nal_end_offset': end})
                if message_type == 256:
                    checksum_prefixes[start] = (interval_start, prefix_sum)
        if _stamp(before) != _stamp(os.fstat(stream.fileno())):
            raise ValueError('video_clock_source_changed')
    pairs = []
    checks = []
    for picture in pictures:
        fields = picture['candidate_fields']
        complete = Counter(field['payload_type'] for field in fields) == {240: 1, 256: 1}
        picture['unambiguous_candidate_pair'] = complete
        check = {'available': False, 'reason': 'candidate_pair_missing_or_ambiguous'}
        if complete:
            values = {field['payload_type']: field['split_tick_candidate'] for field in fields}
            picture['type240_minus_type256_ticks'] = values[240] - values[256]
            pairs.append(values[240] - values[256])
            by_type = {field['payload_type']: field for field in fields}
            a, b = by_type[256], by_type[240]
            lower, computed = checksum_prefixes[a['nal_offset']]
            if (lower <= picture['first_slice_byte_offset']
                    and picture['first_slice_nal_end_offset'] <= a['nal_offset'] < b['nal_offset']):
                raw = bytes.fromhex(b['payload_hex'])
                expected = int.from_bytes(raw[9:11], 'little')
                expected += int.from_bytes(raw[12:14], 'little') << 16
                check = {'available': True, 'algorithm': 'sum_u8_mod_2_32',
                    'byte_offset_start': lower, 'byte_offset_end': a['nal_offset'],
                    'expected_from_sei240': expected, 'computed_from_original_bytes': computed,
                    'native_zero_bypass': expected == 0,
                    'matched': computed == expected if expected else None,
                    'scope': 'Observed AUD interval before SEI256; not a decoded image or exposure proof'}
                checks.append(check)
            else:
                check = {'available': False, 'reason': 'encoded_interval_boundary_unestablished'}
        picture['encoded_interval_checksum'] = check
    return {'format': 'neo2-control/video-clock-inventory/v1',
        'source': {'path': str(path), 'bytes': before.st_size, 'sha256': digest.hexdigest()},
        'scope': 'Original Annex-B bytes; first-slice input order and observed suffix-SEI layouts only',
        'tick_unit': None, 'clock_unit_verified': False, 'exposure_semantics_verified': False,
        'pc_clock_alignment_verified': False, 'decoded_frame_association_verified': False,
        'flight_authorized': False,
        'summary': {'nals': count, 'nal_types': dict(sorted(nal_counts.items())),
            'raw_picture_candidates': len(pictures),
            'pictures_after_parameter_bootstrap': sum(p['after_parameter_bootstrap'] for p in pictures),
            'candidate_pairs': len(pairs), 'pair_difference_histogram': _histogram(Counter(pairs)),
            'encoded_interval_checksums': {'available': len(checks),
                'matched': sum(c['matched'] is True for c in checks),
                'mismatched': sum(c['matched'] is False for c in checks),
                'native_zero_bypass': sum(c['native_zero_bypass'] for c in checks)},
            'candidate_fields': [_summary(pictures, kind) for kind in (256, 240)],
            'valid_sei_messages': [{'payload_type': kind, 'payload_size': size, 'count': total}
                for (kind, size), total in sorted(message_counts.items())],
            'malformed_sei_nals': len(errors),
            'unrecognized_vendor_layouts': dict(sorted(rejected.items())),
            'unassociated_candidate_fields': orphan_candidates},
        'pictures': pictures, 'malformed_sei': errors}
