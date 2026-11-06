"""Fail-closed wire policy for the audited Neo 2 camera preview session.

This permits active registration and WLM service switching, not read-only
device communication. It does not grant flight authority. Session owners must
still enforce peer freshness, transaction phases, and bounded keyframe retries.
"""
from .ble_bootstrap import setup_request
from .duml import DumlPacket, neo2_heartbeat
from .identity import BleClientIdentity, WlmIdentity
from .liveview import wifi_keyframe_request
from .wlm_discovery import unpack_forwarded
from .wlm_modes import ModeFollowRequest, WlmModes, mode_follow_reply
from .wlm_presence import own_presence
from .wlm_query import capability_query
from .wlm_switch import switch_ack, switch_completion, wifi_service_request
from .wlm_wire import wifi_forward


def _deny():
    # Never include private identities, credentials, or peer payloads in errors.
    raise ValueError('preview_outbound_not_allowed')


def _checked_packet(packet):
    if not isinstance(packet, DumlPacket) or not isinstance(packet.payload, bytes):
        _deny()
    for value, maximum in ((packet.version, 1), (packet.sender, 255),
            (packet.receiver, 255), (packet.seq, 65535), (packet.cmd_type, 255),
            (packet.cmd_set, 255), (packet.cmd_id, 255)):
        if type(value) is not int or not 0 <= value <= maximum:
            _deny()
    if packet.version != 1 or len(packet.payload) > 1010:
        _deny()


def _peer_hash(peer_id):
    if (not isinstance(peer_id, bytes) or len(peer_id) != 23
            or peer_id[:3] != bytes((0, 0, 36))):
        _deny()
    serial = peer_id[3:].split(b'\0', 1)[0]
    if not serial or serial.ljust(20, b'\0') != peer_id[3:]:
        _deny()
    value = 0
    for byte in serial + b'_0':
        value = (value * 256 + byte) % 0xfffffffb
    return value


def validate_preview_ble(packet: DumlPacket, ble_identity: BleClientIdentity,
                         wlm_identity: WlmIdentity | None) -> None:
    """Allow only owned BLE setup/credential reads and their permission ACK."""
    _checked_packet(packet)
    if not isinstance(ble_identity, BleClientIdentity):
        _deny()
    if packet.cmd_set == 7 and packet.cmd_id in (0x45, 0x0c, 7, 0x0e):
        expected = setup_request(packet.cmd_id, packet.seq, ble_identity)
    elif (packet.cmd_set, packet.cmd_id) == (7, 0x46):
        expected = DumlPacket(2, 7, packet.seq, 0xc0, 7, 0x46, b'\0')
    elif (packet.cmd_set, packet.cmd_id) == (7, 0x93) and isinstance(wlm_identity, WlmIdentity):
        expected = wlm_identity.link_report(packet.seq, physical_port=9003)
    else:
        _deny()
    if packet != expected:
        _deny()


def validate_preview_udp(payload: bytes, identity: WlmIdentity, *,
                         peer_id: bytes | None = None, physical_port=9003) -> None:
    """Validate exactly one application DUML frame before SWUDP wrapping.

    No stream resynchronization or unknown-command fallback is permitted.
    Caller-owned reports must name the actual physical port. All WLM switching
    messages require the selected canonical aircraft ID and its routing hash.
    """
    if (not isinstance(payload, bytes) or not 13 <= len(payload) <= 1023
            or not isinstance(identity, WlmIdentity)
            or type(physical_port) is not int or not 1 <= physical_port <= 65535):
        _deny()
    try:
        packet = DumlPacket.decode(payload)
        _checked_packet(packet)
        if packet.encode() != payload:
            _deny()
        route = packet.sender, packet.receiver, packet.cmd_set, packet.cmd_id
        if route == (2, 7, 7, 0x93):
            expected = identity.link_report(packet.seq, physical_port=physical_port)
        elif route == (2, 0x1b, 7, 0x93):
            expected = identity.wifi_report(packet.seq, physical_port=physical_port)
        elif route == (2, 0x28, 0, 0):
            expected = neo2_heartbeat(packet.seq)
        elif route == (2, 0xe9, 0x18, 0x47) and len(packet.payload) == 10:
            expected = wifi_keyframe_request(int.from_bytes(packet.payload[2:6], 'little'), packet.seq)
        elif route == (0x3b, 0xe9, 0x51, 1):
            _validate_forwarded(packet, identity, peer_id)
            return
        else:
            _deny()
        if packet != expected:
            _deny()
    except (ValueError, TypeError, IndexError, OverflowError):
        _deny()


def _validate_forwarded(outer, identity, peer_id):
    forwarded = unpack_forwarded(outer.payload)
    inner, attrs = forwarded.packet, forwarded.attributes
    _checked_packet(inner)
    if (inner.sender, inner.receiver, inner.cmd_set) != (0xee, 0xe9, 0x51):
        _deny()
    forwarding_sequence = int.from_bytes(attrs[5:9], 'little')
    destination = 0xffffffff
    body = inner.payload
    if inner.cmd_id == 0x13 and len(body) == 55:
        modes = WlmModes(*body[24:27], *body[52:54])
        expected = own_presence(identity, peer_format=5,
            nonce=int.from_bytes(body[49:51], 'little'),
            counter=int.from_bytes(body[39:43], 'little'),
            forwarding_sequence=forwarding_sequence, modes=modes)
        if outer != expected:
            _deny()
        return
    if inner.cmd_id == 0x41:
        expected = capability_query(identity.wlm_device_hash, inner.seq, forwarding_sequence)
        if outer != expected:
            _deny()
        return
    if inner.cmd_id == 0x1c and len(body) == 33:
        modes = WlmModes(*body[12:17])
        request = ModeFollowRequest(1, 0, 0, 0, body[7], (0, 0, 0), (0, 0), bytes(6))
        expected_inner = mode_follow_reply(identity, request, modes, result=0, sequence=inner.seq)
    elif inner.cmd_id in (2, 6, 8):
        destination = _peer_hash(peer_id)
        if inner.cmd_id == 2 and len(body) == 11:
            modes = WlmModes(4, 4, 4, body[9], body[10])
            expected_inner = wifi_service_request(modes, inner.seq)
        elif inner.cmd_id in (6, 8) and inner.cmd_type == 0xc0 and len(body) == 24:
            # 06 echoes the peer's result, including a failure. 08 is success.
            if inner.cmd_id == 8 and body[23] != 0:
                _deny()
            expected_inner = switch_ack(peer_id, inner.cmd_id, inner.seq, status=body[23])
        elif inner.cmd_id == 6 and inner.cmd_type == 0x40 and len(body) == 51:
            if body[46] not in (0, 1):
                _deny()
            expected_inner = switch_completion(identity, peer_id, inner.seq, status=body[46])
        else:
            _deny()
    else:
        _deny()
    if inner != expected_inner or outer != wifi_forward(expected_inner,
            identity.wlm_device_hash, forwarding_sequence, destination_hash=destination):
        _deny()
