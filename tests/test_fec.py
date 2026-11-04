"""Synthetic vectors encoded by DJI's ARM64 Jerasure, not by this decoder."""

import itertools
import json
from pathlib import Path

import pytest

from neo2_control.fec import InsufficientShards, recover_data_shards


ORACLE = json.loads((Path(__file__).parent / 'fixtures/swudp-fec-native.json').read_text())


@pytest.mark.parametrize('vector', ORACLE['vectors'], ids=lambda v: f"k{v['data_shard_count']}-{v['shard_bytes']}bytes")
def test_recovers_every_one_and_two_erasure_combination_from_native_parity(vector):
    k = vector['data_shard_count']
    data = tuple(bytes.fromhex(v) for v in vector['data_hex'])
    parity = [bytes.fromhex(v) for v in vector['parity_hex']]
    shards = dict(enumerate(list(data) + parity))
    for count in range(len(parity) + 1):
        for erased in itertools.combinations(shards, count):
            available = {i: value for i, value in shards.items() if i not in erased}
            assert recover_data_shards(k, available) == data
    assert shards == dict(enumerate(list(data) + parity))  # input remains reusable


def test_cannot_deliver_data_until_enough_independent_shards_have_arrived():
    vector = ORACLE['vectors'][2]
    k = vector['data_shard_count']
    shards = {i: bytes.fromhex(v) for i, v in enumerate(vector['data_hex'] + vector['parity_hex'])}
    for removed in itertools.combinations(shards, 3):
        if any(i < k for i in removed):
            available = {i: value for i, value in shards.items() if i not in removed}
            with pytest.raises(InsufficientShards):
                recover_data_shards(k, available)


@pytest.mark.parametrize('k,shards', [
    (0, {0: b'abcd'}), (64, {0: b'abcd'}), (True, {0: b'abcd'}),
    (2, {-1: b'abcd'}), (2, {4: b'abcd'}), (1, {2: b'abcd'}),
    (2, {0: b'abc'}), (2, {0: b''}), (2, {0: b'a' * 1456}),
    (2, {0: b'abcd', 1: b'abcdefgh'}), (2, {0: bytearray(b'abcd')}),
])
def test_rejects_invalid_shard_metadata_and_lengths(k, shards):
    with pytest.raises(ValueError):
        recover_data_shards(k, shards)


def test_empty_group_waits_for_input():
    with pytest.raises(InsufficientShards):
        recover_data_shards(2, {})
