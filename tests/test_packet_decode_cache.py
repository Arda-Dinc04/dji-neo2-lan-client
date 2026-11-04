"""Bounded immutable framing reuse never caches connection authority."""
from dataclasses import replace

import pytest

from neo2_control.duml import DumlPacket,_decode_immutable_frame
from neo2_control.host_control import HostControlState
from neo2_control.telemetry import TelemetryReceiver
from neo2_control.wlm_switch import WIFI_SERVICE_MODES


def test_only_bounded_immutable_base_frames_are_reused():
    _decode_immutable_frame.cache_clear()
    raw=DumlPacket(3,2,99,0,3,0x43,bytes(85)).encode()
    first=DumlPacket.decode(raw)
    assert DumlPacket.decode(raw) is first
    assert _decode_immutable_frame.cache_info().hits==1
    mutable=bytearray(raw)
    assert DumlPacket.decode(mutable)==first
    mutable[-1]^=1
    with pytest.raises(ValueError,match='CRC-16'):DumlPacket.decode(mutable)
    count=_decode_immutable_frame.cache_info().currsize
    for _ in range(2):
        with pytest.raises(ValueError):DumlPacket.decode(b'bad')
    assert _decode_immutable_frame.cache_info().currsize==count
    # The public decoder historically accepts trailing input. Long buffers
    # keep that behavior without entering the bounded cache.
    assert DumlPacket.decode(raw+bytes(2048))==first
    assert _decode_immutable_frame.cache_info().currsize==count
    for sequence in range(1000):
        DumlPacket.decode(DumlPacket(3,2,sequence,0,3,0x43,bytes(85)).encode())
    assert _decode_immutable_frame.cache_info().currsize==256


def test_warm_frame_parse_never_supplies_or_refreshes_generation_provenance():
    raw=DumlPacket(3,0x0e,33,0,3,0x43,bytes(85)).encode()
    DumlPacket.decode(raw)  # Framing already cached, without any route.
    old=HostControlState(('a'*64,0x1234,0x5678,1),0,0,0,
        WIFI_SERVICE_MODES,WIFI_SERVICE_MODES,True,True)
    new=replace(old,generation=('a'*64,0x1234,0x5678,2),
        evaluated_at=.2,beacon_received_at=.2,committed_at=.2)
    receiver=TelemetryReceiver()
    receiver.feed(raw,0)
    assert receiver.accepted==0
    receiver.feed(raw,.1,direct_context=old)
    assert receiver.accepted==1 and receiver.last[5]==old.generation
    other=TelemetryReceiver()
    other.feed(raw[:10],.1,direct_context=old)
    other.feed(raw[10:],.2,direct_context=new)
    assert other.accepted==0  # Fragments cannot acquire the new provenance.
    other.feed(raw,.3,direct_context=new)
    assert other.accepted==1 and other.last[5]==new.generation
    other.feed(raw,1,direct_context=new)  # Owner proof has expired.
    assert other.accepted==1 and other.last[2]==.3
