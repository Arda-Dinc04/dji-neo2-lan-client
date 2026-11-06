"""Wire-boundary tests; no sockets, BLE clients, or hardware are created."""
from dataclasses import replace

import pytest

from neo2_control.ble_bootstrap import setup_request
from neo2_control.camera import camera_photo, camera_recording, camera_work_mode, gimbal_pitch
from neo2_control.duml import DumlPacket, neo2_heartbeat
from neo2_control.identity import BleClientIdentity, WlmIdentity
from neo2_control.liveview import wifi_keyframe_request
from neo2_control.preview_policy import validate_preview_ble, validate_preview_udp
from neo2_control.wlm_discovery import unpack_forwarded
from neo2_control.wlm_modes import ModeFollowRequest, WlmModes, mode_follow_reply
from neo2_control.wlm_presence import own_presence
from neo2_control.wlm_query import capability_query
from neo2_control.wlm_switch import switch_ack, switch_completion, wifi_service_request
from neo2_control.wlm_wire import wifi_forward


IDENTITY = WlmIdentity('synthetic-preview-pc')
OTHER = WlmIdentity('other-synthetic-pc')
BLE = BleClientIdentity('0' * 32)
PEER = b'\0\0\x24' + b'synthetic-aircraft'.ljust(20, b'\0')
OTHER_PEER = b'\0\0\x24' + b'different-aircraft'.ljust(20, b'\0')
SEQUENCE = 0x8123
FORWARD_SEQUENCE = 0x12348123


def peer_hash(peer=PEER):
    value = 0
    for byte in peer[3:].split(b'\0', 1)[0] + b'_0':
        value = (value * 256 + byte) % 0xfffffffb
    return value


def wrap(inner, *, peer=True, **kwargs):
    return wifi_forward(inner, IDENTITY.wlm_device_hash, FORWARD_SEQUENCE,
        destination_hash=peer_hash() if peer else 0xffffffff, **kwargs)


def follow():
    request = ModeFollowRequest(1, 0, 0, peer_hash(), 3, (1, 1, 1), (0, 0), bytes(6))
    return mode_follow_reply(IDENTITY, request, WlmModes(1, 1, 1, 0, 0), result=0, sequence=SEQUENCE)


def permitted_udp():
    return [IDENTITY.link_report(SEQUENCE, physical_port=9003),
        IDENTITY.wifi_report(SEQUENCE, physical_port=9003), neo2_heartbeat(SEQUENCE),
        wifi_keyframe_request(0x12345678, SEQUENCE),
        own_presence(IDENTITY, peer_format=5, nonce=123, counter=7,
            forwarding_sequence=FORWARD_SEQUENCE),
        own_presence(IDENTITY, peer_format=5, nonce=123, counter=7,
            forwarding_sequence=FORWARD_SEQUENCE, modes=WlmModes(5, 4, 4, 2, 1)),
        capability_query(IDENTITY.wlm_device_hash, SEQUENCE, FORWARD_SEQUENCE),
        wrap(follow(), peer=False),
        wrap(wifi_service_request(WlmModes(1, 1, 1, 0, 0), SEQUENCE)),
        wrap(switch_ack(PEER, 8, SEQUENCE)), wrap(switch_ack(PEER, 6, SEQUENCE)),
        wrap(switch_ack(PEER, 6, SEQUENCE, status=7)),
        wrap(switch_completion(IDENTITY, PEER, SEQUENCE)),
        wrap(switch_completion(IDENTITY, PEER, SEQUENCE, status=1))]


@pytest.mark.parametrize('packet', permitted_udp())
def test_accepts_audited_preview_producers(packet):
    assert validate_preview_udp(packet.encode(), IDENTITY, peer_id=PEER) is None


@pytest.mark.parametrize('command', (0x45, 0x0c, 7, 0x0e))
def test_ble_exact_setup_requests(command):
    packet = setup_request(command, SEQUENCE, BLE)
    assert validate_preview_ble(packet, BLE, IDENTITY) is None
    for changed in (replace(packet, sender=3), replace(packet, receiver=8),
            replace(packet, cmd_type=0), replace(packet, version=2),
            replace(packet, payload=packet.payload + b'\0')):
        with pytest.raises(ValueError, match='preview_outbound_not_allowed'):
            validate_preview_ble(changed, BLE, IDENTITY)


def test_ble_permission_ack_and_owned_registration_only():
    ack = DumlPacket(2, 7, 0, 0xc0, 7, 0x46, b'\0')
    validate_preview_ble(ack, BLE, None)
    report = IDENTITY.link_report(SEQUENCE, physical_port=9003)
    validate_preview_ble(report, BLE, IDENTITY)
    for packet, identity in ((report, None), (report, OTHER),
            (IDENTITY.link_report(SEQUENCE, physical_port=9004), IDENTITY),
            (replace(ack, payload=b'\1'), IDENTITY),
            (setup_request(0x45, 3, BleClientIdentity('1' * 32)), IDENTITY),
            (replace(ack, sender=258), IDENTITY)):
        with pytest.raises(ValueError):
            validate_preview_ble(packet, BLE, identity)


@pytest.mark.parametrize('packet', permitted_udp())
def test_rejects_mutated_application_headers_and_payload(packet):
    for changed in (replace(packet, sender=packet.sender ^ 1),
            replace(packet, receiver=packet.receiver ^ 1),
            replace(packet, cmd_type=packet.cmd_type ^ 1), replace(packet, version=2),
            replace(packet, payload=packet.payload + b'\0')):
        with pytest.raises(ValueError):
            validate_preview_udp(changed.encode(), IDENTITY, peer_id=PEER)


def test_exact_single_frame_crc_and_no_resynchronization():
    packet = neo2_heartbeat(SEQUENCE).encode()
    corrupt = bytearray(packet)
    corrupt[-1] ^= 1
    for raw in (b'', packet[:-1], bytes(corrupt), b'junk' + packet,
            packet + b'\0', packet + packet, bytearray(packet), 'not bytes'):
        with pytest.raises(ValueError):
            validate_preview_udp(raw, IDENTITY)


def test_private_errors_do_not_include_malformed_or_owned_payloads():
    with pytest.raises(ValueError) as error:
        validate_preview_udp(OTHER.link_report(SEQUENCE, physical_port=9003).encode(), IDENTITY)
    assert str(error.value) == 'preview_outbound_not_allowed'


def test_reports_require_owned_identity_and_actual_physical_port():
    for builder in (IDENTITY.link_report, IDENTITY.wifi_report):
        packet = builder(SEQUENCE, physical_port=9004).encode()
        with pytest.raises(ValueError):
            validate_preview_udp(packet, IDENTITY)
        validate_preview_udp(packet, IDENTITY, physical_port=9004)
        with pytest.raises(ValueError):
            validate_preview_udp(packet, OTHER, physical_port=9004)
    for port in (0, 65536, True, '9003'):
        with pytest.raises(ValueError):
            validate_preview_udp(neo2_heartbeat(1).encode(), IDENTITY, physical_port=port)


@pytest.mark.parametrize('offset', range(22))
def test_all_forwarding_attribute_bytes_checked(offset):
    packet = capability_query(IDENTITY.wlm_device_hash, SEQUENCE, FORWARD_SEQUENCE)
    payload = bytearray(packet.payload)
    payload[-22 + offset] ^= 1
    changed = replace(packet, payload=bytes(payload))
    # Forwarding counter is caller-owned; low16 must agree with outer sequence.
    if offset in (7, 8):
        validate_preview_udp(changed.encode(), IDENTITY)
    else:
        with pytest.raises(ValueError):
            validate_preview_udp(changed.encode(), IDENTITY)


@pytest.mark.parametrize('packet', [camera_photo(3), camera_recording(True, 3),
    camera_work_mode(1, 3), gimbal_pitch(-10, 3),
    DumlPacket(2, 3, 3, 0x40, 3, 0x2a, b'\1'),
    DumlPacket(2, 3, 3, 0x40, 0, 0x40, b'\0'),
    DumlPacket(2, 7, 3, 0x40, 7, 0x40, b'\0')])
def test_denies_control_firmware_and_camera_actions_direct_and_tunneled(packet):
    for candidate in (packet, wrap(packet)):
        with pytest.raises(ValueError):
            validate_preview_udp(candidate.encode(), IDENTITY, peer_id=PEER)


def test_switches_require_selected_peer_and_matching_bodies():
    for packet in (wrap(wifi_service_request(WlmModes(1, 1, 1, 0, 0), SEQUENCE)),
            wrap(switch_ack(PEER, 8, SEQUENCE)),
            wrap(switch_completion(IDENTITY, PEER, SEQUENCE))):
        for peer in (None, OTHER_PEER, b'bad', b'\0' * 23):
            with pytest.raises(ValueError):
                validate_preview_udp(packet.encode(), IDENTITY, peer_id=peer)
    for inner in (switch_ack(OTHER_PEER, 8, SEQUENCE),
            switch_ack(PEER, 8, SEQUENCE, status=1),
            switch_completion(OTHER, PEER, SEQUENCE),
            switch_completion(IDENTITY, OTHER_PEER, SEQUENCE),
            switch_completion(IDENTITY, PEER, SEQUENCE, detail=1),
            switch_completion(IDENTITY, PEER, SEQUENCE, status=2)):
        with pytest.raises(ValueError):
            validate_preview_udp(wrap(inner).encode(), IDENTITY, peer_id=PEER)


def test_checks_inner_crc_and_rejects_nested_envelopes():
    packet = capability_query(IDENTITY.wlm_device_hash, SEQUENCE, FORWARD_SEQUENCE)
    damaged = bytearray(packet.payload)
    damaged[12] ^= 1
    for candidate in (replace(packet, payload=bytes(damaged)), wrap(packet)):
        with pytest.raises(ValueError):
            validate_preview_udp(candidate.encode(), IDENTITY, peer_id=PEER)


@pytest.mark.parametrize('packet', [
    own_presence(IDENTITY, peer_format=5, nonce=7, counter=0),
    capability_query(IDENTITY.wlm_device_hash, SEQUENCE, 1),
    wrap(follow(), peer=False)])
def test_every_inner_body_byte_is_constrained_or_explicit_dynamic_field(packet):
    forwarded = unpack_forwarded(packet.payload)
    inner = forwarded.packet
    if inner.cmd_id == 0x13:
        # Modes, counter and nonce are state-owned dynamic fields.
        offsets = [0, 1, 23, 27, 32, 33, 38, 43, 48, 51, 54]
    elif inner.cmd_id == 0x41:
        offsets = range(len(inner.payload))
    else:
        offsets = [0, 1, 2, 3, 6, 8, 11, 17, 18, 19, 32]
    for offset in offsets:
        body = bytearray(inner.payload)
        body[offset] ^= 1
        candidate = wifi_forward(replace(inner, payload=bytes(body)),
            IDENTITY.wlm_device_hash, int.from_bytes(forwarded.attributes[5:9], 'little'))
        with pytest.raises(ValueError):
            validate_preview_udp(candidate.encode(), IDENTITY, peer_id=PEER)
