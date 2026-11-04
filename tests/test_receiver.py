import json
from pathlib import Path
import struct

import pytest

from neo2_control.receiver import ReliableReceiveState
from neo2_control.transport import ReceiveWindowStatus, _build_datagram, describe_transport


def stream(sequence, payload=b'abcd', *, anchor=0x4000, end=None, channel=2, word=0x101, flags=(0, 0)):
    body = struct.pack('<HHHHI', anchor, sequence if end is None else end, *flags, word) + payload
    return _build_datagram(0x1234, channel, body, send_sequence=sequence)


def receiver(*, anchor=0x4000, fec=False, channel=2):
    return ReliableReceiveState(session=0x1234, anchor=anchor, channel=channel, capacity=100, fec=fec)


def test_reorders_and_acknowledges_only_received_data_including_duplicates():
    state = receiver()
    assert state.receive(stream(0x4018, b'c', end=0x4018)) == ()
    assert state.status == ReceiveWindowStatus(0x4000, 0x4018, (0x4008, 0x4010))
    assert state.receive(stream(0x4018, b'c', end=0x4018)) == ()
    assert state.duplicates == 1
    assert state.receive(stream(0x4008, b'a', end=0x4018)) == (b'a',)
    assert state.status == ReceiveWindowStatus(0x4008, 0x4018, (0x4010,))
    assert state.receive(stream(0x4010, b'b', end=0x4018)) == (b'b', b'c')
    assert state.status == ReceiveWindowStatus(0x4018, 0x4018)
    assert state.payloads_delivered == 3 and state.bytes_delivered == 3
    with pytest.raises(ValueError, match='stale'):
        state.receive(stream(0x4008, b'a', end=0x4018))
    assert state.payloads_delivered == 3


def test_loss_checkpoints_update_missing_states_and_bitmap_slots():
    state = receiver()
    state.receive(stream(0x4018, b'c'))
    assert state.status.encode().hex() == '0040184002000100f0'
    state.observe_window(0x4000, 0x4018, 0x4011, 0)
    assert state.status.missing == (0x400A, 0x4012)
    state.observe_window(0x4000, 0x4018, 0x4011, 0x4011)
    assert state.status.missing == (0x400C, 0x4014)
    assert state.status.encode().hex() == '0040184002000100fa'
    assert state.status.encode(compressed=False).hex() == '0040184002000c401440'
    # State 2 is stable under repeated checkpoints; it must not wrap to 0.
    state.observe_window(0x4000, 0x4018, 0x4011, 0x4011)
    assert state.status.missing == (0x400C, 0x4014)


def test_status_announces_missing_packets_before_any_stream_arrives():
    state = receiver()
    state.observe_window(0x4000, 0x4010)
    assert state.status.missing == (0x4008, 0x4010)
    assert state.receive(stream(0x4010, b'b')) == ()
    assert state.receive(stream(0x4008, b'a', end=0x4010)) == (b'a', b'b')


def test_sequence_wrap_and_sparse_bitmap_use_modulo_slot_distances():
    state = receiver(anchor=0xFFF0)
    state.receive(stream(8, b'c', anchor=0xFFF0))
    assert state.status.missing == (0xFFF8, 0)
    assert state.status.encode().hex() == 'f0ff080002000100f0'
    assert state.receive(stream(0xFFF8, b'a', anchor=0xFFF0, end=8)) == (b'a',)
    assert state.receive(stream(0, b'b', anchor=0xFFF0, end=8)) == (b'b', b'c')
    assert state.start == state.end == 8
    sparse = ReceiveWindowStatus(0xFFF0, 0x18, (0xFFF8, 0x14))
    assert sparse.encode().hex() == 'f0ff180004000100bc'


@pytest.mark.parametrize('kwargs', [
    {'anchor': 0x4008}, {'end': 0x4000}, {'end': 0x5000},
    {'flags': (0x5001, 0)}, {'channel': 3},
    {'word': (1 << 14)}, {'word': (8 << 21) | (1 << 14) | 0x100},
])
def test_bad_or_foreign_fragments_cannot_advance_receive_window(kwargs):
    state = receiver()
    with pytest.raises(ValueError):
        state.receive(stream(0x4008, **kwargs))
    assert state.status == ReceiveWindowStatus(0x4000, 0x4000)
    assert state.fragments_received == 0


def test_unrecoverable_gap_is_bounded_and_never_skipped():
    state = receiver()
    for index in range(2, 101):
        assert state.receive(stream(0x4000 + index * 8, b'x')) == ()
    before = state.status
    with pytest.raises(ValueError):
        state.receive(stream(0x4000 + 101 * 8, b'x'))
    assert state.status == before
    assert state.start == 0x4000 and state.bytes_delivered == 0
    assert state.receive(stream(0x4008, b'x', end=0x4320)) == (b'x',) * 100


def test_fec_disabled_still_applies_tail_and_filters_parity_without_faking_it():
    state = receiver()
    common = (3 << 21) | (1 << 14) | (2 << 8) | 7
    assert state.receive(stream(0x4008, b'abcd', word=common, end=0x4020)) == (b'abcd',)
    assert state.receive(stream(0x4010, b'wxyz', word=common | (1 << 15), end=0x4020)) == (b'wxy',)
    assert state.status.missing == (0x4018, 0x4020)
    for index in (2, 3):
        assert state.receive(stream(0x4008 + index * 8, b'----', word=common | (index << 15), end=0x4020)) == ()
    assert state.start == state.end == 0x4020
    assert state.fragments_recovered == 0


@pytest.mark.parametrize('erased', [(1, 3), (0, 4), (5, 6)])
def test_fec_recovers_and_reorders_native_data_and_synthesizes_only_parity(erased):
    state = receiver(fec=True)
    vector = json.loads((Path(__file__).parent / 'fixtures/swudp-fec-native.json').read_text())['vectors'][2]
    payloads = [bytes.fromhex(v) for v in vector['data_hex'] + vector['parity_hex']]
    common = (13 << 21) | (1 << 14) | (5 << 8) | 31
    delivered = []
    for index in reversed(range(7)):
        if index not in erased:
            delivered.extend(state.receive(stream(0x4008 + index * 8, payloads[index], word=common | (index << 15), end=0x4038)))
    assert delivered == payloads[:4] + [payloads[4][:13]]
    assert state.start == state.end == 0x4038
    assert not state.status.missing
    assert state.fragments_recovered == sum(index < 5 for index in erased)


def test_fec_waits_when_three_data_shards_are_missing():
    state = receiver(fec=True)
    word = (4 << 21) | (1 << 14) | (5 << 8) | 1
    for index in (6, 5, 4, 3):
        assert state.receive(stream(0x4008 + index * 8, b'abcd', word=word | (index << 15), end=0x4038)) == ()
    assert state.status.missing == (0x4008, 0x4010, 0x4018)
    assert state.fragments_recovered == 0


def test_fec_conflicting_group_metadata_is_rejected_before_ack():
    state = receiver(fec=True)
    word = (4 << 21) | (1 << 14) | (5 << 8) | 1
    state.receive(stream(0x4010, b'abcd', word=word | (1 << 15), end=0x4038))
    before = state.status
    with pytest.raises(ValueError, match='inconsistent'):
        state.receive(stream(0x4018, b'abcd', word=(word ^ (1 << 8)) | (2 << 15), end=0x4038))
    assert state.status == before


def test_synthetic_native_receive_and_ack_vectors():
    fixture = json.loads((Path(__file__).parent / 'fixtures/swudp-receiver-native.json').read_text())
    for case in fixture['cases']:
        state = receiver(anchor=case['anchor'], fec=case['fec'])
        delivered = []
        for record in case['records']:
            if record['kind'] == 'packet':
                delivered.extend(state.receive(bytes.fromhex(record['input_hex'])))
            else:
                state.observe_window(*record['window'])
            assert state.status.encode(compressed=True).hex() == record['compressed_ack_hex']
            assert state.status.encode(compressed=False).hex() == record['explicit_ack_hex']
            assert [v.hex() for v in delivered] == record['delivered_hex']
