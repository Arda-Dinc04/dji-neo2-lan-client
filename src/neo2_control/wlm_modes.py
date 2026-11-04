"""WLM link-mode-follow payloads, distinct from service-switch requests.

These codecs implement the native defined fields. They perform no network or
device action. A response requires the caller's actual state and explicit
result; it does not silently claim that a requested service was applied.
"""
from dataclasses import dataclass
import struct

from .duml import DumlPacket
from .identity import WlmIdentity


LINK_MODE_NAMES = ('UNKNOWN', 'SDR_ONLY', 'LTE_ONLY', 'HYBRID_LTE_SDR',
                   'WIFI_ONLY', 'HYBRID_SDR_WIFI', 'HYBRID_LTE_WIFI', 'HYBRID_ALL')
LIVEVIEW_MODE_NAMES = ('SDR', 'HYBRID', 'WIFI')
DOWNLOAD_MODE_NAMES = ('COMMON', 'HIGHSPEED')


@dataclass(frozen=True)
class WlmModes:
    command: int
    liveview: int
    common_data: int
    liveview_service: int
    download_service: int

    def __post_init__(self):
        for value, maximum in ((self.command, 7), (self.liveview, 7),
                (self.common_data, 7), (self.liveview_service, 2), (self.download_service, 1)):
            if type(value) is not int or not 0 <= value <= maximum:
                raise ValueError('invalid WLM link or service mode')

    def encode(self):
        return bytes((self.command, self.liveview, self.common_data,
                      self.liveview_service, self.download_service))

    def metadata(self):
        return {'command': LINK_MODE_NAMES[self.command],
                'liveview': LINK_MODE_NAMES[self.liveview],
                'common_data': LINK_MODE_NAMES[self.common_data],
                'liveview_service': LIVEVIEW_MODE_NAMES[self.liveview_service],
                'download_service': DOWNLOAD_MODE_NAMES[self.download_service]}


@dataclass(frozen=True)
class ModeFollowRequest:
    version: int
    device_index: int
    device_type: int
    device_hash: int
    scene: int
    link_modes: tuple[int, int, int]
    service_modes: tuple[int, int]
    reserved: bytes

    @classmethod
    def decode(cls, payload):
        if not isinstance(payload, bytes) or len(payload) != 19:
            raise ValueError('WLM mode-follow request requires exactly19bytes')
        return cls(payload[0], payload[1], payload[2],
                   int.from_bytes(payload[3:7], 'little'), payload[7],
                   tuple(payload[8:11]), tuple(payload[11:13]), payload[13:])

    def requested_state(self, *, peer_state=None):
        """The APP handler's ordinary, version0 and hybrid-override rules.

        Returning this value does not apply it. Transport/service ownership
        must be resolved by the caller before reporting a successful follow.
        """
        if self.device_type != 0 or self.device_hash == 0 or self.version not in (0, 1):
            raise ValueError('unsupported WLM mode-follow peer or version')
        if peer_state is not None:
            if not isinstance(peer_state, WlmModes):
                raise ValueError('WLM peer state must be validated')
        hybrid_override = peer_state is not None and peer_state.liveview == 3 and self.link_modes[0] == 1
        command, liveview, common_data = ((peer_state.command, peer_state.liveview, peer_state.common_data)
                                        if hybrid_override else self.link_modes)
        if self.version:
            service_liveview, service_download = ((peer_state.liveview_service, peer_state.download_service)
                                                  if hybrid_override else self.service_modes)
        else:
            service_liveview, service_download = int(liveview == 3), int(common_data == 4)
        return WlmModes(command, liveview, common_data, service_liveview, service_download)


def mode_follow_reply(identity, request, actual_state, *, result, sequence):
    """Encode the native19byte prefix in a deterministic33byte response.

    The SDK leaves its sequence and response tail uninitialized. The PC
    deliberately uses an explicit caller-selected sequence and14zero bytes.
    This codec alone does not establish the UAV's response acceptance policy.
    """
    if not isinstance(identity, WlmIdentity) or not isinstance(request, ModeFollowRequest):
        raise ValueError('WLM response requires owned identity and decoded request')
    if not isinstance(actual_state, WlmModes):
        raise ValueError('WLM response requires actual validated state')
    if type(result) is not int or not -(1 << 31) <= result < 1 << 31:
        raise ValueError('WLM response result must fit signed32bits')
    if type(sequence) is not int or not 0 <= sequence < 1 << 16:
        raise ValueError('WLM response sequence must fit uint16')
    payload = (b'\x01\x02\x04' + struct.pack('<I', identity.wlm_device_hash)
               + bytes([request.scene]) + struct.pack('<i', result)
               + actual_state.encode() + bytes(2) + bytes(14))
    return DumlPacket(sender=0xee, receiver=0xe9, seq=sequence, cmd_type=0xc0,
                      cmd_set=0x51, cmd_id=0x1c, payload=payload)
