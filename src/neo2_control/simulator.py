"""Deterministic controller fixture, not a model of DJI flight dynamics.

All telemetry is explicitly synthetic. Commands pass through the production
DUML codecs and acknowledgement matcher; this backend has no I/O imports.
"""

from collections import deque
from .controller import FlightObservation
from .duml import DumlPacket
from .flight import FlightAction


class SimulationBackend:
    synthetic = True

    def __init__(self):
        self.connected = False
        self.phase = 'grounded'
        self.last_sample = None
        self.sample_interval = 0.05
        self.ack_delay = 0.05
        self.motion_delay = 0.3
        self.telemetry_enabled = self.ack_enabled = self.motion_enabled = True
        self.action_status = 0
        self.battery_percent = 100.0
        self.position_valid = True
        self.altitude_override = self.distance_override = None
        self.sent = deque(maxlen=256)
        self.scheduled = []
        self.complete_at = None
        self.channels = (0, 0, 0, 0)

    def open(self, now):
        self.connected = True
        self.last_sample = None
        self._sample(now)

    def close(self, now):
        self.connected = False
        self.scheduled.clear()

    def observation(self):
        return self.last_sample

    def send(self, packet, now):
        if not self.connected:
            raise RuntimeError('synthetic link unavailable')
        # Include encoder/CRC/decoder in the path used by the supervisor.
        packet = DumlPacket.decode(packet.encode())
        self.sent.append((now, packet))
        if (packet.sender, packet.receiver, packet.cmd_type, packet.cmd_set, packet.cmd_id) == (2, 3, 0x40, 3, 0x2A):
            if packet.payload not in (bytes([FlightAction.TAKEOFF]), bytes([FlightAction.LAND])):
                raise ValueError('unsupported simulated action')
            self.scheduled.append((now + self.ack_delay, packet, self.action_status))
        elif (packet.sender, packet.receiver, packet.cmd_type, packet.cmd_set, packet.cmd_id) == (2, 0xA9, 0, 1, 0x0A):
            if len(packet.payload) != 18 or packet.payload[:2] != b'\x01\x0d':
                raise ValueError('invalid simulated joystick body')
            word = int.from_bytes(packet.payload[2:10], 'little')
            self.channels = tuple((word >> shift) & 0x7FF for shift in (8, 19, 30, 41))
        else:
            raise ValueError('unsupported simulated packet')

    def _sample(self, now):
        if self.telemetry_enabled and (self.last_sample is None or now - self.last_sample.received_at >= self.sample_interval - 1e-9):
            airborne = self.phase == 'airborne'
            grounded = self.phase == 'grounded'
            self.last_sample = FlightObservation(
                now, grounded, airborne, not grounded, self.battery_percent,
                self.position_valid, self.altitude_override if self.altitude_override is not None else (1.0 if airborne else 0.0),
                self.distance_override if self.distance_override is not None else 0.0,
                'controller_simulator', True)

    def advance(self, now):
        replies = []
        if not self.connected:
            return replies
        remaining = []
        for due, request, result in self.scheduled:
            if now < due:
                remaining.append((due, request, result))
                continue
            if self.ack_enabled:
                replies.append(DumlPacket(request.receiver, request.sender, request.seq,
                                          0xC0, 3, 0x2A, bytes([result])))
            if result == 0 and self.motion_enabled:
                self.phase = 'taking_off' if request.payload[0] == FlightAction.TAKEOFF else 'landing'
                self.complete_at = due + self.motion_delay
        self.scheduled = remaining
        if self.complete_at is not None and now >= self.complete_at:
            self.phase = 'airborne' if self.phase == 'taking_off' else 'grounded'
            self.complete_at = None
        self._sample(now)
        return replies
