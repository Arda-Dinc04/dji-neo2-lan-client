"""Erasure recovery used by the measured SWUDP receive path.

DJI Fly's native receiver calls Jerasure R6 with w=32, polynomial
x**32 + x**22 + x**2 + x + 1. Shards contain little-endian 32-bit words;
parity rows are [1, ... , 1] and [1, 2, 4, ...] in that field.
This corrects known missing shards, not undetected corruption.
"""

from __future__ import annotations

from collections.abc import Mapping
import struct


POLYNOMIAL = 0x100400007
MAX_SHARD_BYTES = 1452


class InsufficientShards(ValueError):
    """The available data and parity cannot yet fill the known erasures."""


def _multiply(a: int, b: int) -> int:
    result = 0
    while b:
        if b & 1:
            result ^= a
        b >>= 1
        a <<= 1
        if a & 0x100000000:
            a ^= POLYNOMIAL
    return result


def _inverse(value: int) -> int:
    if not value:
        raise ZeroDivisionError("zero has no GF(2**32) inverse")
    a, b = value, POLYNOMIAL
    x, y = 1, 0
    while a != 1:
        shift = a.bit_length() - b.bit_length()
        if shift < 0:
            a, b, x, y = b, a, y, x
            shift = -shift
        a ^= b << shift
        x ^= y << shift
    while x.bit_length() > 32:
        x ^= POLYNOMIAL << (x.bit_length() - 33)
    return x


def _scale(shard: bytes, coefficient: int) -> bytes:
    if coefficient == 1:
        return shard
    result = bytearray(len(shard))
    for index, (word,) in enumerate(struct.iter_unpack("<I", shard)):
        struct.pack_into("<I", result, index * 4, _multiply(word, coefficient))
    return bytes(result)


def _xor(shards: list[bytes], size: int) -> bytes:
    result = 0
    for shard in shards:
        result ^= int.from_bytes(shard, "little")
    return result.to_bytes(size, "little")


def recover_data_shards(
    data_shard_count: int,
    shards: Mapping[int, bytes],
) -> tuple[bytes, ...]:
    """Return data shards in order, or raise while recovery is impossible.

    Data indices are 0..k-1, XOR parity is k, weighted parity is k+1.
    The native k=1 profile has one duplicate parity shard at index 1.
    All inputs are complete equally padded shards; the caller applies the
    encoded tail length only after recovery. No missing bytes are fabricated.
    """

    if type(data_shard_count) is not int or not 1 <= data_shard_count <= 63:
        raise ValueError("data shard count must be between 1 and 63")
    k = data_shard_count
    total = 2 if k == 1 else k + 2
    if any(type(index) is not int or not 0 <= index < total for index in shards):
        raise ValueError("shard index is outside this group")
    if any(not isinstance(shard, bytes) for shard in shards.values()):
        raise ValueError("shards must be immutable bytes")
    sizes = {len(shard) for shard in shards.values()}
    if not sizes:
        raise InsufficientShards("no shards available")
    if len(sizes) != 1:
        raise ValueError("shards must have the same padded length")
    size = sizes.pop()
    if not 0 < size <= MAX_SHARD_BYTES or size % 4:
        raise ValueError("shard length must be a positive multiple of 4 up to 1452")
    missing = [index for index in range(k) if index not in shards]
    if not missing:
        return tuple(shards[index] for index in range(k))
    if len(missing) > 2 or len(shards) < k:
        raise InsufficientShards("not enough independent shards for recovery")
    if k == 1:
        return (shards[1],)

    coefficients = [1]
    for _ in range(1, k):
        coefficients.append(_multiply(coefficients[-1], 2))
    known = [index for index in range(k) if index in shards]
    xor_residual = (
        _xor([shards[k]] + [shards[index] for index in known], size)
        if k in shards else None
    )
    if len(missing) == 1 and xor_residual is not None:
        recovered = {missing[0]: xor_residual}
    else:
        if k + 1 not in shards or (len(missing) == 2 and xor_residual is None):
            raise InsufficientShards("required parity shard is missing")
        weighted = _xor(
            [shards[k + 1]]
            + [_scale(shards[index], coefficients[index]) for index in known],
            size,
        )
        first = missing[0]
        if len(missing) == 1:
            recovered = {first: _scale(weighted, _inverse(coefficients[first]))}
        else:
            second = missing[1]
            assert xor_residual is not None
            # P=x+y; Q=a*x+b*y, hence y=(Q+a*P)/(a+b).
            y = _scale(
                _xor([weighted, _scale(xor_residual, coefficients[first])], size),
                _inverse(coefficients[first] ^ coefficients[second]),
            )
            recovered = {first: _xor([xor_residual, y], size), second: y}
    return tuple(shards[index] if index in shards else recovered[index] for index in range(k))
