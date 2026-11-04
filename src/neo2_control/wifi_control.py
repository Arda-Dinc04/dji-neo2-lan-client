"""Native Wi-Fi control message codecs; no transport or radio side effects.

51/14 and51/15 address the WLM host. Decoding them does not establish that
the host is remote, that a device is current, or that control is authorized.
The caller must establish ownership, route, generation and receipt freshness.
"""
from dataclasses import dataclass

from .duml import DumlPacket
from .wlm_discovery import WlmBeacon


@dataclass(frozen=True, repr=False)
class WifiDeviceRecord:
    payload: bytes

    def __post_init__(self):
        if not isinstance(self.payload, bytes) or len(self.payload) != 61:
            raise ValueError('Wi-Fi device record requires exactly61bytes')

    @property
    def device_id(self):
        return self.payload[:23]

    @property
    def canonical_id(self):
        identifier = self.device_id[3:].split(b'\0', 1)[0]
        return self.device_id[:3] + identifier.ljust(20, b'\0')

    @property
    def selectable(self):
        # Only the native branch condition is known; no physical flag meaning.
        return self.payload[25] != 0

    @property
    def selection_timestamp(self):
        # Signed comparison is proven; units and clock domain remain unknown.
        return int.from_bytes(self.payload[45:53], 'little', signed=True)


@dataclass(frozen=True, repr=False)
class WifiDeviceList:
    records: tuple[WifiDeviceRecord, ...]
    trailing_bytes: bytes = b''

    def __post_init__(self):
        if (not isinstance(self.records, tuple)
                or any(not isinstance(record, WifiDeviceRecord) for record in self.records)
                or not isinstance(self.trailing_bytes, bytes)
                or 2 + 61*len(self.records) + len(self.trailing_bytes) > 1010):
            raise ValueError('Wi-Fi device list must contain bounded immutable records')

    @classmethod
    def decode(cls, body):
        if not isinstance(body, bytes) or not 2 <= len(body) <= 1010:
            raise ValueError('Wi-Fi device list requires a bounded countLE16 body')
        count = int.from_bytes(body[:2], 'little')
        end = 2 + 61 * count
        if end > len(body):
            raise ValueError('Wi-Fi device list has incomplete records')
        return cls(tuple(WifiDeviceRecord(body[i:i+61]) for i in range(2, end, 61)), body[end:])

    @property
    def selected(self):
        selected, newest = None, 0
        for record in self.records:
            if record.selectable and record.selection_timestamp > newest:
                selected, newest = record, record.selection_timestamp
        return selected


def _request(command, body, sequence):
    if type(sequence) is not int or not 0 <= sequence <= 0xffff:
        raise ValueError('Wi-Fi control sequence must be uint16')
    return DumlPacket(2, 0xee, sequence, 0x40, 0x51, command, body)


def wifi_device_selection(devices, beacon, sequence):
    """Assemble51/15 only for the same canonical Neo2 identity.

    Schema equality is checked here. It is not a current-source/boot check.
    """
    if not isinstance(devices, WifiDeviceList) or not isinstance(beacon, WlmBeacon):
        raise ValueError('Wi-Fi selection requires decoded devices and a beacon')
    selected = devices.selected
    if (selected is None or selected.device_id != selected.canonical_id
            or selected.device_id[:3] != bytes((0, 0, 36))
            or not selected.device_id[3:].split(b'\0', 1)[0]
            or selected.device_id != beacon.device_id):
        raise ValueError('selected device does not match the canonical Neo2 beacon')
    return _request(0x15, selected.device_id, sequence)


def wifi_control_mode(sequence):
    """Assemble native51/1A service0/mode2; caller owns prerequisite checks."""
    return _request(0x1a, bytes((0, 0, 2)) + bytes(29), sequence)


def _valid_packet(packet):
    return (isinstance(packet, DumlPacket)
        and all(type(value) is int and 0 <= value <= maximum for value, maximum in
            ((packet.sender,255), (packet.receiver,255), (packet.seq,65535),
             (packet.cmd_type,255), (packet.cmd_set,255), (packet.cmd_id,255)))
        and type(packet.version) is int and packet.version == 1
        and isinstance(packet.payload, bytes) and len(packet.payload) <= 1010)


def wifi_control_response(response, request):
    """Return the SDK result of a strict reversed-route, correlated ACK.

    Result0 acknowledges the host request. It does not confirm a mode change.
    Native51/15 matching omits some checks imposed by this PC codec.
    """
    if (not _valid_packet(request)
            or (request.sender, request.receiver, request.cmd_type, request.cmd_set) != (2, 0xee, 0x40, 0x51)
            or request.cmd_id not in (0x15, 0x1a)):
        raise ValueError('unsupported Wi-Fi control request')
    if request.cmd_id == 0x1a:
        if request.payload != bytes((0, 0, 2)) + bytes(29):
            raise ValueError('unsupported Wi-Fi control mode')
    elif (len(request.payload) != 23 or request.payload[:3] != bytes((0, 0, 36))
            or not request.payload[3:].split(b'\0', 1)[0]
            or request.payload[3:] != request.payload[3:].split(b'\0', 1)[0].ljust(20, b'\0')):
        raise ValueError('unsupported Wi-Fi device identity')
    if (not _valid_packet(response)
            or (response.sender, response.receiver, response.seq, response.cmd_set, response.cmd_id)
                != (request.receiver, request.sender, request.seq, request.cmd_set, request.cmd_id)
            or response.cmd_type not in (0x80, 0xc0)
            or len(response.payload) < (1 if request.cmd_id == 0x15 else 2)):
        raise ValueError('Wi-Fi control response does not match the pending request')
    if request.cmd_id == 0x15:
        return 0 if response.payload[0] == 0 else -7
    status = response.payload[1]
    return 0 if status == 0 else (-0x2200-status if status <= 11 else -0x22ff)
