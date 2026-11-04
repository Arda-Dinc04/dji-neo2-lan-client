from dataclasses import FrozenInstanceError
import json
from pathlib import Path

import pytest

from test_receiver import receiver,stream


def test_reordered_duplicate_does_not_replace_original_packet_receipt():
    state=receiver()
    assert state.receive(stream(0x4018,b'c'),received_at=1.) == ()
    assert state.last_deliveries == ()
    assert state.receive(stream(0x4018,b'c'),received_at=1.8) == ()
    assert state.receive(stream(0x4008,b'a',end=0x4018),received_at=2.) == (b'a',)
    assert state.last_deliveries[0].received_at == 2.
    assert state.receive(stream(0x4010,b'b',end=0x4018),received_at=2.4) == (b'b',b'c')
    assert [v.received_at for v in state.last_deliveries] == [2.4,1.]
    assert [v.sequence for v in state.last_deliveries] == [0x4010,0x4018]
    assert not any(v.recovered for v in state.last_deliveries)
    with pytest.raises(FrozenInstanceError):state.last_deliveries[1].received_at=2.4


def test_coded_tail_and_parity_keep_only_actual_payload_receipts():
    state=receiver(anchor=0xfff0)
    word=(3<<21)|(1<<14)|(2<<8)|7
    assert state.receive(stream(0xfff8,b'abcd',anchor=0xfff0,end=0x10,word=word),received_at=1.) == (b'abcd',)
    assert state.last_deliveries[0].received_at == 1.
    assert state.receive(stream(0,b'xyz_',anchor=0xfff0,end=0x10,word=word|(1<<15)),received_at=2.) == (b'xyz',)
    assert state.last_deliveries[0].sequence == 0
    assert state.last_deliveries[0].received_at == 2.
    for index,sequence in ((2,8),(3,16)):
        assert state.receive(stream(sequence,b'----',anchor=0xfff0,end=0x10,word=word|(index<<15)),received_at=3.) == ()
        assert state.last_deliveries == ()


def test_fec_reconstructed_payload_has_no_invented_original_receipt():
    state=receiver(fec=True)
    vector=json.loads((Path(__file__).parent/'fixtures/swudp-fec-native.json').read_text())['vectors'][2]
    payloads=[bytes.fromhex(v) for v in vector['data_hex']+vector['parity_hex']]
    word=(13<<21)|(1<<14)|(5<<8)|31
    deliveries=[]
    for index in reversed(range(7)):
        if index in (0,1):continue
        state.receive(stream(0x4008+index*8,payloads[index],word=word|(index<<15),end=0x4038),received_at=10.+index)
        deliveries.extend(state.last_deliveries)
    assert len(deliveries)==5
    assert [v.received_at for v in deliveries[:2]] == [None,None]
    assert all(v.recovered for v in deliveries[:2])
    assert all(not v.recovered and v.received_at is not None for v in deliveries[2:])


@pytest.mark.parametrize('stamp',[float('nan'),float('inf'),-1,True])
def test_invalid_receipt_does_not_advance_receive_state(stamp):
    state=receiver()
    with pytest.raises(ValueError,match='receive_packet_clock_invalid'):
        state.receive(stream(0x4008),received_at=stamp)
    assert state.start==0x4000 and state.fragments_received==0 and state.last_deliveries==()
