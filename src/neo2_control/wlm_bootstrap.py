"""PC Wi-Fi report scheduling from the bounded native WLM bootstrap proof.

This is the Wi-Fi branch with an initially empty device cache. The BLE raw
report and its still unknown physical port are outside this module. Native
cache state, transport ACKs and report responses do not prove video readiness.
"""

from collections import deque
from dataclasses import dataclass
import hashlib

from .duml import DumlPacket
from .duml_stream import DumlStreamDecoder
from .identity import WlmIdentity


@dataclass(frozen=True, repr=False)
class ConnectedDevicePrefix:
    count: int
    device: bytes

    @property
    def device_type(self):
        return self.device[0]

    def metadata(self):
        return {'count': self.count, 'device_type': self.device_type,
                'device_sha256': hashlib.sha256(self.device).hexdigest()}


def connected_device_prefix(body: bytes) -> ConnectedDevicePrefix | None:
    """Read only the first entry fields actually used by the native observer.

    A zero low-five-bit count does not require an entry. For a nonzero count,
    only the 31-byte prefix is established; no list-length formula is inferred.
    """
    if not isinstance(body, bytes) or not 1 <= len(body) <= 1010:
        raise ValueError('invalid connected-device body size')
    count = body[0] & 0x1F
    if not count:
        return None
    if len(body) < 31:
        raise ValueError('truncated connected-device prefix')
    return ConnectedDevicePrefix(count, body[3:31])


class WlmWifiBootstrap:
    """At most three reports for one new PC Wi-Fi session, without retries.

    The first pass emits link24. A first type2 device observation enables the
    second pass, link24 then wifi31. The caller supplies the shared DUML
    sequence only when dispatching, and owns transport ACK scheduling.
    """

    def __init__(self, identity: WlmIdentity, physical_port: int):
        if not isinstance(identity, WlmIdentity):
            raise ValueError('WLM bootstrap requires a WLM identity')
        if type(physical_port) is not int or not 1 <= physical_port <= 65535:
            raise ValueError('physical Wi-Fi port must be between 1 and 65535')
        self.identity, self.physical_port = identity, physical_port
        self.decoder = DumlStreamDecoder()
        self.started = False
        self.device = None
        self._queued = deque()
        self.reports = []
        self.pushes = self.ignored = self.malformed = 0

    def open(self):
        if self.started:
            raise RuntimeError('WLM bootstrap can only be opened once')
        self.started = True
        self._queued.append(('link24', 'wifi_connected'))

    @property
    def has_pending(self):
        return bool(self._queued)

    def next_report(self, sequence: int):
        if not self._queued:
            raise RuntimeError('no WLM report is pending')
        kind, trigger = self._queued.popleft()
        builder = self.identity.link_report if kind == 'link24' else self.identity.wifi_report
        packet = builder(sequence, physical_port=self.physical_port)
        self.reports.append({'kind': kind, 'trigger': trigger, 'sequence': packet.seq,
                             'receiver': packet.receiver, 'transport_acknowledged': False,
                             'datagram_sent': False, 'delivery_mode': None,
                             'response_status': None})
        return packet, kind, trigger

    def sent(self, sequence, delivery_mode):
        for report in self.reports:
            if report['sequence'] == sequence:
                report['datagram_sent'] = True
                report['delivery_mode'] = delivery_mode
                return

    def acknowledge(self, sequence):
        for report in self.reports:
            if report['sequence'] == sequence:
                report['transport_acknowledged'] = True
                return

    def _accept(self, packet, event):
        if (packet.version, packet.receiver, packet.cmd_set) != (1, 2, 7):
            self.ignored += 1
            return
        if packet.cmd_id == 0x93 and packet.cmd_type == 0xC0:
            for report in self.reports:
                if (packet.sender, packet.seq) == (report['receiver'], report['sequence']):
                    if not packet.payload:
                        self.malformed += 1
                        event('wlm_response_rejected', reason='missing_status')
                    elif report['response_status'] is None:
                        report['response_status'] = packet.payload[0]
                        event('wlm_report_response', kind=report['kind'], sequence=packet.seq,
                              status=packet.payload[0], application_ready=False)
                    return
        # Native dispatch does not locally filter command type or sender.
        # The PC profile restricts the route to the measured Neo2 host7->APP2.
        if (packet.sender, packet.cmd_id) != (7, 0x94):
            self.ignored += 1
            return
        try:
            device = connected_device_prefix(packet.payload)
        except ValueError:
            self.malformed += 1
            event('wlm_device_rejected', reason='malformed_prefix')
            return
        self.pushes += 1
        if device is None or device.device_type != 2 or self.device is not None:
            return
        self.device = device
        self._queued.extend((('link24', 'device_observed'), ('wifi31', 'device_observed')))
        event('wlm_device_observed', **device.metadata(), native_cache_state=3,
              application_ready=False)

    def feed(self, data: bytes, event):
        if not self.started:
            raise RuntimeError('WLM bootstrap is not open')
        if len(data) > 1024 * 1024:
            raise ValueError('WLM protocol block exceeds bounded size')
        for offset in range(0, len(data), 1024):
            for packet, _ in self.decoder.feed(data[offset:offset + 1024]):
                self._accept(packet, event)

    def snapshot(self):
        return {'enabled': True, **self.identity.metadata(),
                'physical_wifi_port': self.physical_port,
                'state': ('device_observed' if self.device is not None else
                          'awaiting_device' if self.started else 'not_started'),
                'native_cache_state': 3 if self.device is not None else 0,
                'device': self.device.metadata() if self.device else None,
                'reports_pending': len(self._queued), 'reports_scheduled': len(self.reports),
                'reports_sent': sum(r['datagram_sent'] for r in self.reports),
                'reports_transport_acknowledged': sum(r['transport_acknowledged'] for r in self.reports),
                'reports': [dict(r) for r in self.reports], 'device_pushes': self.pushes,
                'ignored_frames': self.ignored, 'malformed_frames': self.malformed,
                'discarded_bytes': self.decoder.discarded_bytes, 'application_ready': False}
