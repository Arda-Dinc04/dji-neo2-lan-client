"""Incremental, bounded DUML v1 framing across BLE notification boundaries."""

from .duml import DumlPacket, crc8


class DumlStreamDecoder:
    def __init__(self, max_buffer: int = 4096):
        if type(max_buffer) is not int or max_buffer < 1023:
            raise ValueError('DUML stream buffer must hold at least one maximum frame')
        self.max_buffer = max_buffer
        self.buffer = bytearray()
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> tuple[tuple[DumlPacket, bytes], ...]:
        if len(self.buffer) + len(data) > self.max_buffer:
            raise ValueError('DUML stream buffer limit exceeded')
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            marker = self.buffer.find(0x55)
            if marker < 0:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break
            self.discarded_bytes += marker
            del self.buffer[:marker]
            if len(self.buffer) < 4:
                break
            word = int.from_bytes(self.buffer[1:3], 'little')
            length, version = word & 0x3FF, word >> 10
            if length < 13 or crc8(self.buffer[:3]) != self.buffer[3]:
                del self.buffer[0]
                self.discarded_bytes += 1
                continue
            if len(self.buffer) < length:
                break
            if version != 1:
                # A checked header defines the candidate's boundary. Do not
                # promote bytes inside an unsupported envelope to top-level
                # messages while looking for the next V1 frame.
                del self.buffer[:length]
                self.discarded_bytes += length
                continue
            raw = bytes(self.buffer[:length])
            try:
                packet = DumlPacket.decode(raw)
            except ValueError:
                # Likewise, discard the complete CRC-invalid candidate; an
                # embedded frame must not escape its rejected carrier.
                del self.buffer[:length]
                self.discarded_bytes += length
                continue
            del self.buffer[:length]
            frames.append((packet, raw))
        return tuple(frames)
