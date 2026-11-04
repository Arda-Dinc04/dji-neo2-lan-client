"""Diagnostic pushes use one exact WLM frame and retain forwarding provenance."""
from dataclasses import replace
import struct

import pytest

from neo2_control.battery import BatteryReceiver
from neo2_control.duml import DumlPacket
from neo2_control.wifi_status import WifiControlReceiver
from neo2_control.wlm_discovery import unpack_observation_payload
from test_battery import frame as battery_frame
from test_wifi_status import frame as wifi_frame


@pytest.fixture(params=[(BatteryReceiver,battery_frame),(WifiControlReceiver,wifi_frame)],
                ids=['battery','wifi-control'])
def profile(request):
    return request.param


def carrier(inner,*,source=0x12345678,destination=0x23456789,counter=1,category=2,kind=0):
    attributes=struct.pack('<IBIIBB7x',source,category,counter,destination,1,0x82)
    return DumlPacket(0x27,0xee,counter&0xffff,kind,0x51,1,inner+attributes)


def test_valid_forwarding_attributes_are_not_counted_as_malformed(profile):
    cls,frame=profile
    receiver=cls()
    receiver.feed(carrier(frame()).encode(),1)
    result=receiver.snapshot(1)
    assert result['accepted']==1 and result['malformed']==0
    assert result['observations'][0]['source']['forwarding']=={
        'source_hash':0x12345678,'destination_hash':0x23456789,'link_category':2}
    assert not result['flight_ready'] and not result['source_route_verified']


def test_peer_and_destination_hashes_have_independent_ages_and_counters_do_not_create_sources(profile):
    cls,frame=profile
    receiver=cls()
    receiver.feed(carrier(frame(),source=1,destination=2).encode(),1)
    receiver.feed(carrier(frame(),source=3,destination=2).encode(),1.5)
    receiver.feed(carrier(frame(),source=1,destination=0xffffffff).encode(),1.7)
    receiver.feed(carrier(frame(),source=3,destination=2,counter=2).encode(),2)
    result=receiver.snapshot(2.1)
    assert len(result['observations'])==3
    by_hash={(s['source']['forwarding']['source_hash'],s['source']['forwarding']['destination_hash']):s
             for s in result['observations']}
    assert not by_hash[1,2]['fresh'] and by_hash[3,2]['fresh']
    assert by_hash[3,2]['received_at']==2


@pytest.mark.parametrize('alteration',['short-trailer','extra-trailer','junk-prefix','extra-frame','crc'])
def test_malformed_native_envelope_never_partially_accepts_or_scans_for_a_push(profile,alteration):
    cls,frame=profile
    receiver=cls();packet=carrier(frame())
    payload=packet.payload
    if alteration=='short-trailer':payload=payload[:-1]
    elif alteration=='extra-trailer':payload+=b'\0'
    elif alteration=='junk-prefix':payload=b'junk'+payload
    elif alteration=='extra-frame':payload=frame()+payload
    else:
        changed=bytearray(payload);changed[len(frame())-1]^=1;payload=bytes(changed)
    receiver.feed(replace(packet,payload=payload).encode(),1)
    assert receiver.accepted==0 and receiver.malformed==1
    assert not receiver.snapshot(1)['available']


@pytest.mark.parametrize('kind',[1,0x21,0x41,0x60,0x80,0xa0,0xc0,0xe0])
def test_response_or_encrypted_outer_cannot_renew_inner_push(profile,kind):
    cls,frame=profile
    receiver=cls();receiver.feed(carrier(frame()).encode(),1)
    receiver.feed(carrier(frame(),kind=kind).encode(),4)
    result=receiver.snapshot(4)
    assert result['accepted']==1 and not result['observations'][0]['fresh']


@pytest.mark.parametrize('category',[0,1,3,15])
def test_native_other_link_category_does_not_refresh_wifi_observations(profile,category):
    cls,frame=profile
    receiver=cls();receiver.feed(carrier(frame(),category=category).encode(),1)
    assert receiver.accepted==0 and receiver.ignored==1 and receiver.malformed==0


def test_legacy_bare_and_native_carriers_are_exact_and_remain_separate(profile):
    cls,frame=profile
    receiver=cls();native=carrier(frame())
    receiver.feed(frame(),1)
    receiver.feed(replace(native,payload=frame()).encode(),1.1)
    receiver.feed(native.encode(),1.2)
    result=receiver.snapshot(1.2)
    assert len(result['observations'])==3 and result['malformed']==0
    assert sum('forwarding' in s['source'] for s in result['observations'])==1
    receiver.feed(replace(native,payload=frame()+b'junk').encode(),1.3)
    assert receiver.accepted==3 and receiver.malformed==1


def test_nested_tunnel_is_not_scanned_a_second_time(profile):
    cls,frame=profile
    receiver=cls();receiver.feed(carrier(carrier(frame()).encode()).encode(),1)
    assert receiver.accepted==0 and receiver.ignored==1


def test_fragmented_and_batched_native_carriers_preserve_frame_boundaries(profile):
    cls,frame=profile
    receiver=cls();raw=carrier(frame()).encode()
    for byte in raw:receiver.feed(bytes([byte]),1)
    receiver.feed(b''.join(carrier(frame(),counter=i).encode() for i in range(2,82)),2)
    assert receiver.accepted==81 and receiver.malformed==0
    assert len(receiver.observations)==1 and not receiver.decoder.buffer


def test_forwarded_peer_slots_are_bounded(profile):
    cls,frame=profile
    receiver=cls()
    for source in range(12):receiver.feed(carrier(frame(),source=source).encode(),source)
    assert len(receiver.observations)==8 and receiver.evicted==4


@pytest.mark.parametrize('payload',[b'',b'\0'*12,b'\0'*1011])
def test_observation_helper_has_bounded_input(payload):
    with pytest.raises(ValueError):unpack_observation_payload(payload)
