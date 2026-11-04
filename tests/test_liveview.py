"""Native keyframe bytes and PC-only actual-send policy, no device I/O."""
import json
from pathlib import Path

import pytest

from neo2_control.duml import DumlSequence
from neo2_control.liveview import WifiKeyframeRequests,wifi_keyframe_request


NATIVE=json.loads((Path(__file__).parent/'fixtures/iframe-request-native.json').read_text())


@pytest.mark.parametrize('row',[r for r in NATIVE['vectors'] if r['requested']])
def test_direct_request_matches_complete_native_wifi_output_with_fresh_request_bit(row):
    packet=wifi_keyframe_request(row['ssfn'],row['sequence'])
    assert packet.encode().hex()==row['wifi_transport_calls'][0]['raw_hex']
    assert (packet.sender,packet.receiver,packet.cmd_type,packet.cmd_set,packet.cmd_id)==(2,0xe9,0,0x18,0x47)
    assert packet.payload[6:]==b'\x01\0\0\0'


@pytest.mark.parametrize('arguments',[(True,1),(-1,1),(2**32,1),(1.5,1),
                                      (1,False),(1,-1),(1,65536),(1,'2')])
def test_builder_rejects_invalid_fields(arguments):
    with pytest.raises(ValueError):wifi_keyframe_request(*arguments)


def test_policy_uses_actual_send_time_and_never_requeues_a_pending_datagram():
    policy=WifiKeyframeRequests();sequence=DumlSequence(current=0xfffd)
    def next_packet(now,needed=True):
        return policy.next_packet(now,needed=needed,ssfn=0x22334455,next_sequence=sequence.next)
    first=next_packet(0)
    assert first.seq==0xfffe and policy.snapshot(0)['sent_count']==0
    assert next_packet(.5) is None
    policy.sent(first.seq,.8)
    assert next_packet(1.79) is None
    second=next_packet(1.8)
    assert second.seq==0x8000
    policy.sent(second.seq,2)
    assert next_packet(2.99) is None
    third=next_packet(3);policy.sent(third.seq,3)
    for when in (4,10,100):assert next_packet(when) is None
    snapshot=policy.snapshot(100)
    assert snapshot['sent_count']==3 and len(snapshot['requests'])==3
    assert snapshot['state']=='limit_reached' and not snapshot['refresh_confirmed']
    assert not snapshot['acknowledgement_expected']


def test_no_need_or_closure_cancels_scheduling_without_reporting_a_send():
    policy=WifiKeyframeRequests();sequence=DumlSequence()
    request=policy.next_packet(0,needed=True,ssfn=0,next_sequence=sequence.next)
    assert request is not None
    assert policy.next_packet(.1,needed=False,ssfn=0,next_sequence=sequence.next) is None
    state=policy.snapshot(.1)
    assert state['pending'] is None and state['sent_count']==0 and state['cancelled_pending']==1
    with pytest.raises(RuntimeError):policy.sent(request.seq,.2)
    policy.close(.3,'requested')
    assert policy.next_packet(.4,needed=True,ssfn=0,next_sequence=sequence.next) is None
    assert policy.snapshot(.4)['reason']=='requested'
