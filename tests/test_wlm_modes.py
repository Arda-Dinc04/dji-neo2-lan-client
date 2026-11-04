from dataclasses import replace
import json
from pathlib import Path

import pytest

from neo2_control.identity import WlmIdentity
from neo2_control.wlm_modes import ModeFollowRequest, WlmModes, mode_follow_reply

FIXTURE = json.loads((Path(__file__).parent/'fixtures/wlm-mode-follow-native.json').read_text())
CASES = {case['name']: case for case in FIXTURE['cases']}
IDENTITY = WlmIdentity('pc-state-synthetic')


@pytest.mark.parametrize('name', ['known_uav_v1', 'known_uav_v1_poison', 'scene_echo',
    'version0_derived_service', 'hybrid_peer_override', 'hybrid_version0_derived_service'])
def test_requested_state_matches_native_service_updater_including_version0_hybrid(name):
    case = CASES[name]
    request = ModeFollowRequest.decode(bytes.fromhex(case['input_payload_hex']))
    peer = WlmModes(2, 3, 2, 2 if name == 'hybrid_version0_derived_service' else 1,
                    1 if name == 'hybrid_version0_derived_service' else 0)
    state = request.requested_state(peer_state=peer if case['peer_hybrid'] else None)
    link_bytes = bytes.fromhex(case['after']['link_modes_hex'])
    service_bytes = bytes.fromhex(case['after']['service_modes_hex'])
    assert state.encode() == bytes([link_bytes[1], link_bytes[3], link_bytes[5],
                                     service_bytes[2], service_bytes[3]])
    assert not case['after']['ability_timer_exists']


@pytest.mark.parametrize('name', [name for name in CASES if name != 'wrong_length'])
def test_reply_defined_prefix_matches_actual_native_with_explicit_result_and_actual_state(name):
    case = CASES[name]
    request = ModeFollowRequest.decode(bytes.fromhex(case['input_payload_hex']))
    native = bytes.fromhex(case['frames'][0]['payload_hex'])
    actual = WlmModes(*native[12:17])
    result = int.from_bytes(native[8:12], 'little', signed=True)
    reply = mode_follow_reply(IDENTITY, request, actual, result=result, sequence=0x8123)
    assert reply.payload[:19] == native[:19]
    assert len(reply.payload) == 33 and reply.payload[19:] == bytes(14)
    assert reply.seq == 0x8123
    assert (reply.sender, reply.receiver, reply.cmd_type, reply.cmd_set, reply.cmd_id) == (0xee,0xe9,0xc0,0x51,0x1c)


def test_native_stack_poison_is_never_reproduced_by_pc_reply():
    zero = CASES['known_uav_v1']['frames'][0]
    poison = CASES['known_uav_v1_poison']['frames'][0]
    assert zero['sequence'] == 0 and poison['sequence'] == 0xa5a5
    assert bytes.fromhex(zero['payload_hex'])[:19] == bytes.fromhex(poison['payload_hex'])[:19]
    assert bytes.fromhex(zero['payload_hex'])[19:] != bytes.fromhex(poison['payload_hex'])[19:]
    request = ModeFollowRequest.decode(bytes.fromhex(CASES['known_uav_v1']['input_payload_hex']))
    reply = mode_follow_reply(IDENTITY, request, WlmModes(1,1,1,0,0), result=0, sequence=7)
    assert reply.seq == 7 and reply.payload[19:] == bytes(14)


@pytest.mark.parametrize('length', [0,18,20,1010])
def test_bad_request_length_cannot_be_followed(length):
    with pytest.raises(ValueError):ModeFollowRequest.decode(bytes(length))


@pytest.mark.parametrize('changes', [{'device_type':4}, {'device_hash':0}, {'version':2},
    {'link_modes':(8,1,1)}, {'service_modes':(3,0)}])
def test_pc_state_rejects_unverified_modes_versions_and_non_uav_peers(changes):
    request = ModeFollowRequest.decode(bytes.fromhex(CASES['known_uav_v1']['input_payload_hex']))
    with pytest.raises(ValueError):replace(request, **changes).requested_state()


def test_sdr_follow_and_wifi_transport_are_distinct_from_wifi_liveview_service():
    request = ModeFollowRequest.decode(bytes.fromhex(CASES['known_uav_v1']['input_payload_hex']))
    assert request.requested_state().metadata() == {'command':'SDR_ONLY','liveview':'SDR_ONLY',
        'common_data':'SDR_ONLY','liveview_service':'SDR','download_service':'COMMON'}
    assert WlmModes(5,4,4,2,0).metadata()['liveview_service'] == 'WIFI'
