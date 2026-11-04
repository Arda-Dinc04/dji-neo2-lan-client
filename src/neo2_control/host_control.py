"""Typed state of the PC's owning WLM connection, without a transmitter.

The SDK classifies its selected peer's advertised services locally. Our
connectivity predicate is explicit PC policy: a source-bound fresh beacon,
the actual Wi-Fi backend and the owner's bilateral service commit. It is
not a reconstructed native heap or an aircraft permission report.
"""
from dataclasses import asdict, dataclass
import math

from .wlm_modes import WlmModes
from .wlm_switch import WIFI_SERVICE_MODES


def valid_generation(value):
    return (isinstance(value, tuple) and len(value) == 4
        and isinstance(value[0], str) and len(value[0]) == 64
        and all(c in '0123456789abcdef' for c in value[0])
        and all(type(v) is int and 0 <= v < 2**32 for v in value[1:3])
        and type(value[3]) is int and 0 < value[3] < 65536)


def canonical_neo2_id(value):
    return (isinstance(value, bytes) and len(value) == 23
        and value[:3] == bytes((0, 0, 36))
        and bool(value[3:].split(b'\0', 1)[0])
        and value[3:] == value[3:].split(b'\0', 1)[0].ljust(20, b'\0'))


@dataclass(frozen=True)
class HostControlState:
    generation: tuple[str, int, int, int]
    evaluated_at: float
    beacon_received_at: float
    committed_at: float | None
    local_modes: WlmModes
    peer_modes: WlmModes
    wifi_backend_ready: bool
    beacon_route_verified: bool

    EVALUATION_MAXIMUM_AGE = .5
    BEACON_MAXIMUM_AGE = 2.
    SOURCE = 'pc_owned_wlm_state'

    def __post_init__(self):
        times = (self.evaluated_at, self.beacon_received_at)
        if not valid_generation(self.generation):
            raise ValueError('host control state requires a bound peer generation')
        if (any(type(t) not in (int, float) or not math.isfinite(t) for t in times)
                or self.beacon_received_at > self.evaluated_at):
            raise ValueError('host control evaluation must follow its beacon receipt')
        if self.committed_at is not None and (
                type(self.committed_at) not in (int, float)
                or not math.isfinite(self.committed_at) or self.committed_at > self.evaluated_at):
            raise ValueError('host service commit must precede evaluation')
        if not all(isinstance(m, WlmModes) for m in (self.local_modes, self.peer_modes)):
            raise ValueError('host control state requires typed local and peer modes')
        if any(type(v) is not bool for v in (self.wifi_backend_ready, self.beacon_route_verified)):
            raise ValueError('host connectivity evidence must be explicit')

    def current(self, now):
        return (type(now) in (int, float) and math.isfinite(now)
            and 0 <= now-self.evaluated_at <= self.EVALUATION_MAXIMUM_AGE
            and 0 <= now-self.beacon_received_at < self.BEACON_MAXIMUM_AGE)

    def mode(self, now):
        # Native classification, with the explicitly named PC connection
        # predicate in place of the SDK's local connection-prefix lifecycle.
        if not self.current(now) or not self.wifi_backend_ready or not self.beacon_route_verified:
            return 65535
        if self.peer_modes.liveview_service == 2:
            return 2
        return int(self.peer_modes.download_service == 1)

    def problems(self, now):
        if type(now) not in (int, float) or not math.isfinite(now):
            return ['invalid_evaluation_time']
        reasons = []
        if not 0 <= now-self.evaluated_at <= self.EVALUATION_MAXIMUM_AGE:
            reasons.append('owner_evaluation_stale')
        if not 0 <= now-self.beacon_received_at < self.BEACON_MAXIMUM_AGE:
            reasons.append('peer_beacon_stale')
        if not self.beacon_route_verified:
            reasons.append('beacon_route_unverified')
        if not self.wifi_backend_ready:
            reasons.append('wifi_backend_unavailable')
        if self.committed_at is None:
            reasons.append('local_service_uncommitted')
        if self.local_modes != WIFI_SERVICE_MODES:
            reasons.append('local_modes_not_committed_target')
        if self.local_modes.command not in (4, 5, 6, 7) or self.peer_modes.command not in (4, 5, 6, 7):
            reasons.append('command_route_not_wifi')
        if self.mode(now) != 2:
            reasons.append('peer_operation_mode_unavailable')
        return reasons

    def permits_control(self, now):
        return not self.problems(now)

    def snapshot(self, now):
        if type(now) not in (int, float) or not math.isfinite(now):
            raise ValueError('host control snapshot time must be finite')
        return {**asdict(self), 'source': self.SOURCE,
            'connectivity_policy': 'source_bound_beacon_and_actual_owned_wifi_backend',
            'native_connection_prefix_lifecycle_verified': False,
            'mode_code': self.mode(now), 'control_ready': self.permits_control(now),
            'evaluation_age_seconds': now-self.evaluated_at,
            'beacon_age_seconds': now-self.beacon_received_at,
            'problems': self.problems(now), 'aircraft_execution_verified': False}
