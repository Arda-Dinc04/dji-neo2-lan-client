from dataclasses import replace
import json
from pathlib import Path
import struct

import pytest

from neo2_control.duml import DumlPacket
from neo2_control.identity import WlmIdentity
from neo2_control.wlm_discovery import WlmBeacon, unpack_forwarded
from neo2_control.wlm_query import WlmCapabilityRead, capability_query


FIXTURES=Path(__file__).parent/'fixtures'
FORWARD=json.loads((FIXTURES/'wlm-forwarding-native.json').read_text())
DISCOVERY=json.loads((FIXTURES/'wlm-discovery-native.json').read_text())
STATE=json.loads((FIXTURES/'wlm-state-native.json').read_text())


def beacon():
    raw=bytearray.fromhex(DISCOVERY['beacons'][0]['payload_hex'])
    # Hash of this synthetic SN is independently produced by the state guest;
    # the three device-header bytes do not affect that native hash.
    raw[4:24]=b'pc-state-synthetic'.ljust(20,b'\0')
    return WlmBeacon(bytes(raw))


def read():
    return WlmCapabilityRead(WlmIdentity('pc-query-synthetic'),beacon(),
        sequence=0x8123,forwarding_sequence=1)


def response():
    row=STATE['read_only_query']['native_query_consumer_result']['emitted_frames'][0]
    native=DumlPacket.decode(bytes.fromhex(row['frame_hex']))
    # Reverse native APP/UAV direction to simulate the unobserved drone reply.
    inner=replace(native,sender=0xe9,receiver=0xee)
    outer=DumlPacket.decode(bytes.fromhex(DISCOVERY['beacons'][0]['carrier_hex']))
    attr=bytes.fromhex(DISCOVERY['beacons'][0]['attributes_hex'])
    return replace(outer,payload=inner.encode()+attr)


def alter_inner(outer,**fields):
    forwarded=unpack_forwarded(outer.payload)
    return replace(outer,payload=replace(forwarded.packet,**fields).encode()+forwarded.attributes)


def test_query_matches_entire_native_wifi_agent_output():
    row=FORWARD['wifi_agent_frames'][0]
    assert row['injected_peer_native_host']==0x0907
    expected=bytes.fromhex(row['carrier_hex'])
    assert capability_query(0x12345678,0x8123,0x1234).encode()==expected
    inner=unpack_forwarded(DumlPacket.decode(expected).payload).packet
    assert inner.payload[9:]==b'\x06\0\x0a\0'


@pytest.mark.parametrize('arguments', [(-1,1,1),(1,65536,1),(1,1,2**32),
    (True,1,1),(1,False,1),(1,1,1.5)])
def test_query_rejects_invalid_numeric_fields(arguments):
    with pytest.raises(ValueError):capability_query(*arguments)


def test_forwarding_counter_full32bits_and_outer16bits_match_native_layout():
    outer=capability_query(1,3,0x12345678)
    forwarded=unpack_forwarded(outer.payload)
    assert outer.seq==0x5678
    assert struct.unpack_from('<I',forwarded.attributes,5)[0]==0x12345678
    assert forwarded.attributes[13:]==b'\x01\x82'+bytes(7)


def test_native_reply_is_correlated_and_fragmented_without_readiness_claim():
    query=read()
    assert query.snapshot(0)['state']=='prepared'
    query.sent(1)
    for byte in response().encode():query.feed(bytes([byte]),1.1)
    result=query.snapshot(1.2)
    assert result['state']=='complete' and result['response_status']==0
    assert result['capabilities']=={'6':{'word_count':2,'mask':0,'support':0x1001f},
                                   '10':{'word_count':2,'mask':0,'support':0x47d}}
    assert not result['negotiated'] and not result['application_ready']
    assert 'pc-state-synthetic' not in json.dumps(result)
    assert query.peer_hash==int.from_bytes(bytes.fromhex(
        STATE['read_only_query']['native_query_consumer_result']['emitted_frames'][0]['payload_hex'])[5:9],'little')
    with pytest.raises(RuntimeError):query.sent(1.3)


@pytest.mark.parametrize('change', ['sequence','source','destination','type','outer_type',
    'outer_source','category','hash','operation','subtype','reserved'])
def test_unrelated_responses_do_not_complete_or_extend_deadline(change):
    query=read();query.sent(0)
    outer=response()
    inner=unpack_forwarded(outer.payload).packet
    if change=='sequence':outer=alter_inner(outer,seq=0x8124)
    elif change=='source':outer=alter_inner(outer,sender=2)
    elif change=='destination':outer=alter_inner(outer,receiver=2)
    elif change=='type':outer=alter_inner(outer,cmd_type=0x40)
    elif change=='outer_type':outer=replace(outer,cmd_type=0xc0)
    elif change=='outer_source':outer=replace(outer,sender=0x28)
    elif change=='category':
        raw=bytearray(outer.payload);raw[-18]=1;outer=replace(outer,payload=bytes(raw))
    else:
        raw=bytearray(inner.payload)
        raw[{'hash':5,'operation':1,'subtype':3,'reserved':2}[change]]^=1
        outer=alter_inner(outer,payload=bytes(raw))
    query.feed(outer.encode(),1.4)
    assert query.snapshot(1.4)['state']=='awaiting_response'
    assert query.snapshot(1.5)['state']=='timed_out'
    query.feed(response().encode(),1.6)
    assert query.snapshot(1.6)['capabilities'] is None


@pytest.mark.parametrize('change', ['duplicate_tag','short_vector','bad_header','missing_tag','bad_outer_crc'])
def test_malformed_or_unsupported_response_cannot_install_capabilities(change):
    query=read();query.sent(0)
    outer=response();inner=unpack_forwarded(outer.payload).packet
    raw=bytearray(inner.payload)
    if change=='duplicate_tag':raw[30]=6
    elif change=='short_vector':raw.pop();raw[9:11]=struct.pack('<H',len(raw)-11)
    elif change=='bad_header':raw[13]=0x21
    elif change=='missing_tag':
        raw=raw[:30];raw[9:11]=struct.pack('<H',19)
    outer=alter_inner(outer,payload=bytes(raw)).encode()
    if change=='bad_outer_crc':outer=outer[:-1]+bytes([outer[-1]^1])
    query.feed(outer,.1)
    assert query.snapshot(.1)['state']=='awaiting_response'
    assert query.snapshot(.1)['capabilities'] is None


def test_valid_error_status_is_distinct_from_success_and_unsent_or_duplicate_reply():
    query=read();query.feed(response().encode(),0)
    assert query.snapshot(0)['state']=='prepared'
    query.sent(1)
    outer=response();inner=unpack_forwarded(outer.payload).packet
    query.feed(alter_inner(outer,payload=b'\x01'+inner.payload[1:]).encode(),1.1)
    assert query.snapshot(1.1)['state']=='response_error'
    assert query.snapshot(1.1)['response_status']==1
    query.feed(response().encode(),1.2)
    assert query.snapshot(1.2)['state']=='response_error'
    assert query.snapshot(1.2)['capabilities'] is None


def test_read_has_bounded_input_and_monotonic_time():
    query=read();query.sent(1)
    with pytest.raises(ValueError):query.feed(bytes(1024*1024+1),1)
    with pytest.raises(ValueError):query.snapshot(.9)
    with pytest.raises(ValueError):query.snapshot(float('nan'))
