"""Optimized CRCs preserve the original bit recurrence and framing checks."""
import random

import pytest

from neo2_control.duml import DumlPacket,crc8,crc16
from neo2_control.duml_stream import DumlStreamDecoder


def original(data,initial,polynomial,mask):
    crc=initial
    for byte in data:
        crc^=byte
        for _ in range(8):
            crc=(crc>>1)^polynomial if crc&1 else crc>>1
    return crc&mask


def test_all_crc8_byte_state_combinations_match_original_bit_recurrence():
    for seed in range(256):
        for byte in range(256):
            data=bytes((byte,))
            assert crc8(data,seed)==original(data,seed,0x8c,0xff)


def test_every_crc16_initial_state_and_every_input_byte_match_original():
    for seed in range(65536):
        data=b'\xa5'
        assert crc16(data,seed)==original(data,seed,0x8408,0xffff)
    for seed in (0,0x3692,0xffff):
        for byte in range(256):
            data=bytes((byte,))
            assert crc16(data,seed)==original(data,seed,0x8408,0xffff)


@pytest.mark.parametrize('seed',[0,0x77,0x3692,0xffff,-1,0x123456789])
def test_crc_long_frames_and_incremental_updates_keep_initial_state_semantics(seed):
    rng=random.Random(0x4e454f32)
    for length in (0,1,3,13,98,512,1023,4096):
        data=rng.randbytes(length)
        assert crc8(data,seed)==original(data,seed,0x8c,0xff)
        assert crc16(data,seed)==original(data,seed,0x8408,0xffff)
        if 0<=seed<=0xff:
            assert crc8(data[3:],crc8(data[:3],seed))==crc8(data,seed)
        if 0<=seed<=0xffff:
            assert crc16(data[3:],crc16(data[:3],seed))==crc16(data,seed)


def test_existing_flight_vector_and_corruption_boundaries_are_unchanged():
    raw=bytes.fromhex('550e04660203018140032a01a641')
    packet=DumlPacket.decode(raw)
    assert packet.encode()==raw
    assert packet.payload==b'\1'
    for offset in range(len(raw)):
        damaged=bytearray(raw);damaged[offset]^=1
        with pytest.raises(ValueError):DumlPacket.decode(bytes(damaged))
    # Valid inner DUML must still not escape a rejected outer carrier.
    outer=bytearray(DumlPacket(3,2,22,0,0x51,1,raw).encode());outer[-1]^=1
    decoder=DumlStreamDecoder()
    assert decoder.feed(outer)==()
    assert decoder.discarded_bytes==len(outer)
