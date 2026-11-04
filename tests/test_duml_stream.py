import pytest

from neo2_control.duml import DumlPacket
from neo2_control.duml_stream import DumlStreamDecoder


PACKET = DumlPacket(7, 2, 0x8123, 0xC0, 7, 7, bytes(range(40)))
RAW = PACKET.encode()


@pytest.mark.parametrize('split', range(1, len(RAW)))
def test_all_notification_split_boundaries(split):
    decoder = DumlStreamDecoder()
    assert decoder.feed(RAW[:split]) == ()
    assert decoder.feed(RAW[split:]) == ((PACKET, RAW),)
    assert not decoder.buffer and decoder.discarded_bytes == 0


def test_stream_resynchronizes_after_noise_bad_crc_and_other_version():
    corrupt = bytearray(RAW)
    corrupt[-1] ^= 1
    other_version = DumlPacket(7, 2, 0x8123, 0xC0, 7, 7, b'', version=2).encode()
    decoder = DumlStreamDecoder()
    prefix = b'noise' + bytes(corrupt) + other_version
    assert decoder.feed(prefix + RAW + RAW) == ((PACKET, RAW), (PACKET, RAW))
    assert decoder.discarded_bytes == len(prefix) and not decoder.buffer


def test_maximum_frame_reassembles_and_buffer_limit_is_enforced():
    raw = DumlPacket(7, 2, 1, 0xC0, 7, 7, bytes(1010)).encode()
    decoder = DumlStreamDecoder(max_buffer=1023)
    for byte in raw[:-1]:
        assert decoder.feed(bytes([byte])) == ()
    assert decoder.feed(raw[-1:])[0][1] == raw
    with pytest.raises(ValueError, match='buffer limit'):
        decoder.feed(bytes(1024))
    assert not decoder.buffer


@pytest.mark.parametrize('unsupported', [False, True])
@pytest.mark.parametrize('split', [0, 11])
def test_rejected_carrier_payload_cannot_escape_as_a_top_level_frame(unsupported, split):
    carrier = bytearray(DumlPacket(7, 2, 1, 0, 0x51, 1, RAW,
                                  version=2 if unsupported else 1).encode())
    if not unsupported:
        carrier[-1] ^= 1
    decoder = DumlStreamDecoder()
    assert decoder.feed(bytes(carrier[:split])) == ()
    assert decoder.feed(bytes(carrier[split:]) + RAW) == ((PACKET, RAW),)
    assert not decoder.buffer and decoder.discarded_bytes == len(carrier)
