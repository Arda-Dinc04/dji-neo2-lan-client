"""Native WLM per-link Wi-Fi agent envelope, with deterministic PC padding."""
import struct

from .duml import DumlPacket


def wifi_forward(packet, own_hash, forwarding_sequence, *, selected_links=4, destination_hash=0xffffffff):
    """Wrap complete inner DUML on the measured3b→e9 Wi-Fi route.

    selected_links is the native SDR/LTE/WiFi mask. A Wi-Fi-only PC uses4;
    the native forced-all-links policy uses7 even for its Wi-Fi copy. The
    seven unwritten native tail bytes are explicitly zeroed by this client.
    """
    if not isinstance(packet, DumlPacket) or packet.version != 1:
        raise ValueError('WLM forwarding requires a version1 DUML packet')
    for value in (own_hash, forwarding_sequence, destination_hash):
        if type(value) is not int or not 0 <= value < 1 << 32:
            raise ValueError('WLM forwarding fields must fit uint32')
    if type(selected_links) is not int or not 1 <= selected_links <= 7 or not selected_links & 4:
        raise ValueError('WLM Wi-Fi forwarding requires a link mask containing Wi-Fi')
    inner = packet.encode()
    if len(inner) > 988:
        raise ValueError('WLM packet requires unsupported fragmentation')
    attributes = struct.pack('<IBIIBB7x', own_hash, 2, forwarding_sequence,
                             destination_hash, 1, 2 | (selected_links << 5))
    return DumlPacket(sender=0x3b, receiver=0xe9, seq=forwarding_sequence & 0xffff,
                      cmd_type=0, cmd_set=0x51, cmd_id=1, payload=inner + attributes)
