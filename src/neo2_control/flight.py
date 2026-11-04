"""Offline flight command codecs, derived from saved native SDK producers.

There is no socket/transmitter here. An accepted action response confirms a
request acknowledgement; execution requires independent telemetry evidence.
"""

from dataclasses import dataclass
from enum import IntEnum
import math
import struct

from .duml import DumlPacket, encode_host_id


VIRTUAL_JOYSTICK_INTERVAL_SECONDS = 0.05
DEFAULT_YAW_CHANNEL_LIMIT = 200
MAX_YAW_CHANNEL_LIMIT = 600
ACQUISITION_ALTITUDE_LIMIT = 1.8
ACQUISITION_VERTICAL_CHANNEL_LIMIT = 200


def trial_channels_allowed(channels, *, acquisition_trial=False):
    """Both trial profiles forbid horizontal motion; scan permits bounded lift."""
    return (not any(channels[:2]) and
        (abs(channels[2])<=ACQUISITION_VERTICAL_CHANNEL_LIMIT if acquisition_trial else channels[2]==0))


def validate_yaw_channel_limit(value, *, yaw_trial):
    if type(value) is not int or not 1 <= value <= MAX_YAW_CHANNEL_LIMIT:
        raise ValueError('yaw_channel_limit_must_be_integer_in_1_600')
    if not yaw_trial and value != DEFAULT_YAW_CHANNEL_LIMIT:
        raise ValueError('yaw_channel_limit_requires_yaw_trial')
    return value


class FlightAction(IntEnum):
    TAKEOFF = 0x01
    LAND = 0x02
    RETURN_HOME = 0x06
    CANCEL_RETURN_HOME = 0x0C
    CANCEL_TAKEOFF = 0x0D
    CANCEL_LANDING = 0x0E
    CONFIRM_LANDING = 0x1E
    FORCE_LANDING = 0x1F
    PRECISION_TAKEOFF = 0x22


def _sequence(sequence: int):
    if type(sequence) is not int or not 0 <= sequence <= 0xFFFF:
        raise ValueError('DUML sequence must be an integer in 0..65535')


def flight_action(action: FlightAction, sequence: int, *, receiver_index: int = 0) -> DumlPacket:
    if not isinstance(action, FlightAction):
        raise ValueError('an explicitly named FlightAction is required')
    _sequence(sequence)
    if type(receiver_index) is not int:
        raise ValueError('receiver index must be an integer')
    return DumlPacket(2, encode_host_id(3, receiver_index), sequence, 0x40,
                      3, 0x2A, bytes([action]))


@dataclass(frozen=True)
class FlightActionReply:
    status_code: int

    def __post_init__(self):
        if type(self.status_code) is not int or not 0 <= self.status_code <= 255:
            raise ValueError('flight action status must be a byte')

    @property
    def accepted(self):
        return self.status_code == 0

    def as_dict(self):
        return {'status_code': self.status_code, 'accepted': self.accepted,
                'execution_confirmed': False}


def parse_action_response(packet: DumlPacket, request: DumlPacket) -> FlightActionReply:
    """Require the corresponding response before interpreting its status byte."""
    _sequence(request.seq)
    if ((request.sender, request.cmd_type, request.cmd_set, request.cmd_id, request.version)
        != (2, 0x40, 3, 0x2A, 1) or len(request.payload) != 1
        or type(request.receiver) is not int or not 0 <= request.receiver <= 255
        or request.receiver & 0x1F != 3
        or request.payload[0] not in FlightAction._value2member_map_):
        raise ValueError('request is not a supported flight action')
    # Neo2's observed action replies use0x80; the saved SDK/fixtures also
    # use0xc0. Both are plaintext responses, with all request fields still
    # correlated. A request or encrypted/reserved response is not an ACK.
    if ((packet.sender, packet.receiver, packet.seq,
         packet.cmd_set, packet.cmd_id, packet.version)
        != (request.receiver, request.sender, request.seq, 3, 0x2A, 1)
        or packet.cmd_type not in (0x80, 0xC0) or not packet.payload):
        raise ValueError('response does not match the pending flight action')
    return FlightActionReply(packet.payload[0])


def virtual_joystick(channels: tuple[int, int, int, int], sequence: int, *,
                     relative_time_ms: int | None = None) -> DumlPacket:
    """Encode four native channels in the UI's normalized ±1000 range.

    The inspected UI labels channels right, forward, up, turn-right; these
    are dimensionless intended movements, not measured aircraft responses.
    The fixed mode and optional clock syntax match the saved producer; the caller must have
    established clock synchronization before supplying relative_time_ms.
    Native code masks any int32, but this API rejects values outside the range
    confirmed in the app's stick UI. It neither clamps nor schedules messages.
    """
    _sequence(sequence)
    if not isinstance(channels, (tuple, list)) or len(channels) != 4:
        raise ValueError('exactly four normalized channels are required')
    word = 1 << 62
    for value, shift in zip(channels, (8, 19, 30, 41)):
        if type(value) is not int or not -1000 <= value <= 1000:
            raise ValueError('normalized channels must be integers in -1000..1000')
        scaled = math.trunc(value / 1000.0 * 660.0 + 1024.0)
        word |= (scaled & 0x7FF) << shift
    payload = b'\x01\x0d' + struct.pack('<QI', word, 0x200) + b'\x06\x55\x01\x04'
    if relative_time_ms is not None:
        if type(relative_time_ms) is not int or not 0 <= relative_time_ms < 1 << 64:
            raise ValueError('relative time must be uint64 milliseconds')
        payload += b'\x56\x08' + struct.pack('<Q', relative_time_ms)
    return DumlPacket(2, encode_host_id(9, 5), sequence, 0, 1, 0x0A, payload)


def virtual_joystick_directions(sequence: int, *, right: int = 0, forward: int = 0,
                                up: int = 0, turn_right: int = 0,
                                relative_time_ms: int | None = None) -> DumlPacket:
    """Encode normalized UI directions; no transport or flight authorization.

    Positive names follow the saved handheld UI across USA/JP/CH layouts.
    Values are integer stick positions in -1000..1000, not angles or speeds.
    Do not reapply the touch view's square-root response curve to these values.
    Actual Neo 2 response and fixed native mode bits remain unverified.
    """
    return virtual_joystick((right, forward, up, turn_right), sequence,
                            relative_time_ms=relative_time_ms)


def wifi_control_service(enabled: bool, sequence: int) -> DumlPacket:
    """Encode the saved UAV77 Wi-Fi enter/exit service request, offline only.

    Enter selects service 0, mode 2; exit selects service 0, mode 0. Its SDK
    guard, product selection, call order and aircraft acceptance are external
    preconditions, not supplied or bypassed by this serializer.
    """
    _sequence(sequence)
    if type(enabled) is not bool:
        raise ValueError('Wi-Fi control selection must be an explicit boolean')
    payload = bytes(2) + bytes([2 if enabled else 0]) + bytes(29)
    return DumlPacket(2, encode_host_id(14, 7), sequence, 0x40, 0x51, 0x1A, payload)


def joystick_control_authority(requested: bool, sequence: int) -> DumlPacket:
    """Encode the distinct SDK request/release authority operation, offline.

    This is not proven necessary or supported for Neo 2 handheld Wi-Fi.
    A request packet or ACK does not establish an actual flight authority.
    """
    _sequence(sequence)
    if type(requested) is not bool:
        raise ValueError('joystick authority selection must be an explicit boolean')
    return DumlPacket(2, encode_host_id(18, 5), sequence, 0x40, 0x49, 0x80,
                      bytes([int(requested)]))
