"""Pinned03/53 bytes and adversarial envelopes, without any transport owner."""
from dataclasses import replace

import pytest

from neo2_control.duml import DumlPacket
from neo2_control.host_control import HostControlState
from neo2_control.landing_confirmation import LandingConfirmationReceiver
from neo2_control.wlm_switch import WIFI_SERVICE_MODES
from test_observation_envelope import carrier


GENERATION = ('a'*64, 0x12345678, 0x23456789, 123)
TRUE_FRAME = bytes.fromhex('551104920302f2d500035333000000d756')
FALSE_FRAME = bytes.fromhex('551104920302dcd400035313000000d774')


def owner(now=1., **changes):
    return replace(HostControlState(GENERATION, now, now, .5,
        WIFI_SERVICE_MODES, WIFI_SERVICE_MODES, True, True), **changes)


def wrap(raw, **changes):
    return carrier(raw, destination=0xffffffff, **changes).encode()


def test_pinned_true_and_false_are_independent_receipts_not_latched_state():
    receiver = LandingConfirmationReceiver()
    receiver.feed(TRUE_FRAME,1.,direct_context=owner())
    first = receiver.snapshot(1.)['observations'][0]
    assert first['raw_payload_hex'] == '33000000'
    assert first['fields']['landing_confirmation_needed'] == {'value':True,'valid':True,'reason':None}
    assert first['source']['direct_connection']['generation'] == list(GENERATION)
    receiver.feed(FALSE_FRAME,1.1,direct_context=owner(1.1))
    latest = receiver.snapshot(1.1)['observations'][0]
    assert latest['received_at']==1.1 and latest['fields']['landing_confirmation_needed']['value'] is False
    stale = receiver.snapshot(1.600001)['observations'][0]
    assert stale['fields']['landing_confirmation_needed']['valid'] is False
    assert stale['fields']['landing_confirmation_needed']['reason']=='stale'


@pytest.mark.parametrize('body,value',[('20000000',True),('00200000',False),('13080000',False),('33000000',True)])
@pytest.mark.parametrize('receiver_id',[2,0x0e])
def test_exact_little_endian_bit_and_measured_recipients(body,value,receiver_id):
    receiver = LandingConfirmationReceiver()
    packet = replace(DumlPacket.decode(TRUE_FRAME),payload=bytes.fromhex(body),receiver=receiver_id)
    receiver.feed(wrap(packet.encode()),1.)
    item = receiver.snapshot(1.)['observations'][0]
    assert item['fields']['landing_confirmation_needed']['value'] is value
    assert item['source']['forwarding']=={'source_hash':0x12345678,
        'destination_hash':0xffffffff,'link_category':2}
    assert 'direct_connection' not in item['source']


@pytest.mark.parametrize('changes',[
    {'sender':2},{'sender':0x23},{'receiver':0xaa},{'receiver':3},
    {'cmd_set':2},{'cmd_id':0x43},{'version':2},
    *({'cmd_type':value} for value in [1,0x20,0x40,0x80,0xc0,0x81]),
])
def test_spoofed_or_unobserved_inner_profiles_do_not_enter_store(changes):
    receiver = LandingConfirmationReceiver()
    packet = replace(DumlPacket.decode(TRUE_FRAME),**changes)
    receiver.feed(packet.encode(),1.,direct_context=owner())
    assert not receiver.snapshot(1.)['available']


@pytest.mark.parametrize('length',[0,1,2,3,5,6,85])
def test_two_byte_native_read_does_not_relax_observed_four_byte_profile(length):
    receiver=LandingConfirmationReceiver()
    receiver.feed(replace(DumlPacket.decode(TRUE_FRAME),payload=b'\x33'*length).encode(),1.)
    assert receiver.accepted==0 and receiver.malformed==1


@pytest.mark.parametrize('change',['sender','receiver','type20','type80','category','destination','bare','nested','short-trailer','inner-crc'])
def test_only_exact_native_one_level_broadcast_carrier_is_observed(change):
    receiver=LandingConfirmationReceiver()
    packet=carrier(TRUE_FRAME,destination=0xffffffff)
    if change=='sender':packet=replace(packet,sender=0x3b)
    elif change=='receiver':packet=replace(packet,receiver=2)
    elif change.startswith('type'):packet=replace(packet,cmd_type=int(change[4:],16))
    elif change=='category':packet=carrier(TRUE_FRAME,destination=0xffffffff,category=3)
    elif change=='destination':packet=carrier(TRUE_FRAME,destination=GENERATION[2])
    elif change=='bare':packet=replace(packet,payload=TRUE_FRAME)
    elif change=='nested':packet=replace(packet,payload=packet.encode()+packet.payload[-22:])
    elif change=='short-trailer':packet=replace(packet,payload=packet.payload[:-1])
    else:
        changed=bytearray(packet.payload);changed[len(TRUE_FRAME)-1]^=1
        packet=replace(packet,payload=bytes(changed))
    receiver.feed(packet.encode(),1.,direct_context=owner())
    assert not receiver.snapshot(1.)['available']


def test_unbound_or_uncommitted_direct_data_remains_diagnostic_without_owner_provenance():
    receiver=LandingConfirmationReceiver()
    receiver.feed(TRUE_FRAME,1.)
    assert 'direct_connection' not in receiver.snapshot(1.)['observations'][0]['source']
    receiver.feed(TRUE_FRAME,1.1,direct_context=owner(1.1,committed_at=None))
    assert 'direct_connection' not in receiver.snapshot(1.1)['observations'][0]['source']
    with pytest.raises(ValueError,match='typed'):
        receiver.feed(TRUE_FRAME,1.2,direct_context={'generation':GENERATION})


@pytest.mark.parametrize('transition',['unbound-to-bound','new-generation'])
def test_partial_frame_cannot_inherit_new_owner_provenance(transition):
    receiver=LandingConfirmationReceiver()
    before=None if transition=='unbound-to-bound' else owner()
    after=owner(1.1,generation=GENERATION[:-1]+(124,)) if before is not None else owner(1.1)
    receiver.feed(TRUE_FRAME[:8],1.,direct_context=before)
    receiver.feed(TRUE_FRAME[8:],1.1,direct_context=after)
    assert receiver.accepted==0 and receiver.decoder.discarded_bytes>=8
    receiver.feed(TRUE_FRAME,1.2,direct_context=replace(after,evaluated_at=1.2))
    assert receiver.snapshot(1.2)['observations'][0]['source']['direct_connection']['generation']==list(after.generation)


def test_same_owner_split_frame_completes_but_new_receiver_never_adopts_old_prefix():
    receiver=LandingConfirmationReceiver()
    receiver.feed(TRUE_FRAME[:8],1.,direct_context=owner())
    receiver.feed(TRUE_FRAME[8:],1.1,direct_context=owner(1.1))
    assert receiver.accepted==1
    replacement=LandingConfirmationReceiver()
    replacement.feed(TRUE_FRAME[8:],1.1,direct_context=owner(1.1))
    assert replacement.accepted==0


def test_crc_failure_does_not_refresh_and_foreign_source_store_is_bounded():
    receiver=LandingConfirmationReceiver()
    damaged=TRUE_FRAME[:-1]+bytes([TRUE_FRAME[-1]^1])
    receiver.feed(damaged,1.)
    assert receiver.accepted==0
    for source in range(12):receiver.feed(wrap(TRUE_FRAME,source=source),1.)
    status=receiver.snapshot(1.)
    assert len(status['observations'])==8 and status['evicted']==4


@pytest.mark.parametrize('bad',[True,None,float('nan'),float('inf')])
def test_invalid_clocks_cannot_refresh_or_relabel_evidence(bad):
    receiver=LandingConfirmationReceiver()
    with pytest.raises(ValueError,match='time'):
        receiver.feed(TRUE_FRAME,bad)
