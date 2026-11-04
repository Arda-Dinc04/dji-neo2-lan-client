"""Bounded Annex-B filtering and optional current-stream decoder bootstrap.

This is a byte-preserving framing/payload-bounds filter. It does not interpret
camera metadata, decode pictures, normalize parameter sets or infer timestamps.
"""


class HevcAnnexBFilter:
    """Keep framed NALs, with optional removal of all preview SEI metadata.

    ``max_nal_bytes`` bounds the bytes after a NAL's3/4-byte Annex-B prefix.
    Up to three extra zeros may await the next prefix. Feed chunks are limited
    to65536bytes; the temporary append is therefore bounded by that amount too.
    Annex-B leading/trailing zeros are preserved. Dropped byte counts include
    the invalid SEI's prefix and its adjacent trailing zeros.

    Optional decoder bootstrap caches current-stream VPS/SPS/PPS, discards
    units before the first IRAP with all three present, then emits those exact
    parameter units and the IRAP. It does not validate their codec semantics.
    The default remains byte preserving outside structurally invalid SEI.
    ``strip_sei`` explicitly removes both prefix/suffix SEI, including valid
    messages. This is useful for a local preview decoder whose metadata parser
    rejects camera-specific escaping. It must not be used for raw capture.
    """

    MAX_NAL_BYTES = 4 * 1024 * 1024
    MAX_CHUNK_BYTES = 65536
    MAX_BOOTSTRAP_BYTES = 4 * 1024 * 1024

    def __init__(self, *, max_nal_bytes=MAX_NAL_BYTES, wait_for_bootstrap=False,
                 strip_sei=False):
        if type(max_nal_bytes) is not int or not 2 <= max_nal_bytes <= self.MAX_NAL_BYTES:
            raise ValueError('HEVC NAL bound must be2..4194304bytes')
        if type(wait_for_bootstrap) is not bool:
            raise ValueError('HEVC decoder bootstrap option must be boolean')
        if type(strip_sei) is not bool:
            raise ValueError('HEVC SEI removal option must be boolean')
        self.max_nal_bytes = max_nal_bytes
        self.wait_for_bootstrap = wait_for_bootstrap
        self.strip_sei = strip_sei
        self.bootstrap_ready = not wait_for_bootstrap
        self.bootstrap_discarded_nals = self.bootstrap_discarded_bytes = 0
        self.bootstrap_opening_nal_type = None
        self.bootstrap_seen_parameter_sets = frozenset()
        self._parameter_sets = {}
        self._parameter_bytes = 0
        self.input_bytes = self.output_bytes = 0
        self.dropped_sei_nals = self.dropped_sei_bytes = 0
        self._buffer = bytearray()
        self._prefix_length = None
        self._scan_from = 0
        self._seen_prefix = False
        self._finished = self._failed = False

    @property
    def buffered_bytes(self):
        return len(self._buffer)

    @property
    def bootstrap_buffered_bytes(self):
        return 0 if self.bootstrap_ready else self._parameter_bytes + len(self._buffer)

    def _error(self, message):
        self._failed = True
        self._buffer.clear()
        self._parameter_sets.clear()
        self._parameter_bytes = 0
        raise ValueError(message)

    def _check_open(self):
        if self._finished or self._failed:
            raise ValueError('HEVC Annex-B filter is already finished or failed')

    @staticmethod
    def _rbsp(ebsp):
        """Remove legal escape bytes, without rewriting noncanonical input.

        In particular000002 occurs in real DJI metadata and is tolerated by
        FFmpeg. Canonical escaping is outside this bounds filter's purpose.
        """
        result = bytearray()
        index = 0
        while index < len(ebsp):
            if (index + 3 < len(ebsp) and ebsp[index:index + 3] == b'\0\0\3'
                    and ebsp[index + 3] <= 3):
                result.extend(b'\0\0')
                index += 3
            else:
                result.append(ebsp[index])
                index += 1
        return result

    @classmethod
    def _valid_sei(cls, body):
        if body[0] & 0x80 or not body[1] & 7:
            return False
        # Annex-B trailing_zero_8bits follow, rather than belong to, the RBSP.
        rbsp = cls._rbsp(body[2:].rstrip(b'\0'))
        if not rbsp or rbsp[-1] != 0x80:
            return False
        cursor = messages = 0
        stop = len(rbsp) - 1
        while cursor < stop:
            # payloadType and payloadSize are byte-aligned ff-extended values.
            while cursor < stop and rbsp[cursor] == 0xFF:
                cursor += 1
            if cursor == stop:
                return False
            cursor += 1  # Final payloadType byte; semantics are opaque here.
            size = 0
            while cursor < stop and rbsp[cursor] == 0xFF:
                size += 255
                cursor += 1
            if cursor == stop:
                return False
            size += rbsp[cursor]
            cursor += 1
            if size > stop - cursor:
                return False
            cursor += size
            messages += 1
        return cursor == stop and messages > 0

    def _unit(self, raw, prefix_length):
        size = len(raw) - prefix_length
        if size < 2:
            self._error('HEVC Annex-B NAL has a truncated two-byte header')
        if size > self.max_nal_bytes:
            self._error('HEVC Annex-B NAL exceeds the configured byte bound')
        body = raw[prefix_length:]
        if (body[0] >> 1) & 63 in (39, 40) and (self.strip_sei or not self._valid_sei(body)):
            self.dropped_sei_nals += 1
            self.dropped_sei_bytes += len(raw)
            return b''
        if not self.bootstrap_ready:
            kind = (body[0] >> 1) & 63
            if kind in (32, 33, 34):
                previous = self._parameter_sets.get(kind, b'')
                if self._parameter_bytes - len(previous) + len(raw) > self.MAX_BOOTSTRAP_BYTES:
                    self._error('HEVC decoder bootstrap exceeds the configured byte bound')
                if previous:
                    self.bootstrap_discarded_nals += 1
                    self.bootstrap_discarded_bytes += len(previous)
                self._parameter_sets[kind] = raw
                self._parameter_bytes += len(raw) - len(previous)
                self.bootstrap_seen_parameter_sets |= {kind}
                return b''
            if 16 <= kind <= 23 and len(self._parameter_sets) == 3:
                result = b''.join(self._parameter_sets[k] for k in (32, 33, 34)) + raw
                self._parameter_sets.clear()
                self._parameter_bytes = 0
                self.bootstrap_opening_nal_type = kind
                self.bootstrap_ready = True
                return result
            self.bootstrap_discarded_nals += 1
            self.bootstrap_discarded_bytes += len(raw)
            return b''
        return raw

    def _preamble(self, raw):
        if not self.bootstrap_ready:
            self.bootstrap_discarded_bytes += len(raw)
            return b''
        return raw

    def feed(self, data):
        self._check_open()
        if not isinstance(data, bytes) or len(data) > self.MAX_CHUNK_BYTES:
            self._error('HEVC feed requires bytes of at most65536bytes')
        self.input_bytes += len(data)
        self._buffer.extend(data)
        output = []
        while True:
            match = self._buffer.find(b'\0\0\1', self._scan_from)
            if match < 0:
                if self._prefix_length is None:
                    if any(self._buffer):
                        self._error('HEVC Annex-B stream must begin with a start code')
                    if len(self._buffer) > 3:
                        output.append(self._preamble(bytes(self._buffer[:-3])))
                        del self._buffer[:-3]
                    self._scan_from = 0
                else:
                    # A final run of at most three zeros may begin the next
                    # prefix; all other bytes already belong to this NAL.
                    body_size = len(self._buffer) - self._prefix_length
                    possible_prefix = min(3, len(self._buffer) - len(self._buffer.rstrip(b'\0')))
                    if body_size - possible_prefix > self.max_nal_bytes:
                        self._error('HEVC Annex-B NAL exceeds the configured byte bound')
                    self._scan_from = max(self._prefix_length, len(self._buffer) - 3)
                break
            start = match - 1 if match and self._buffer[match - 1] == 0 else match
            prefix_length = 4 if start != match else 3
            if self._prefix_length is None:
                if any(self._buffer[:start]):
                    self._error('HEVC Annex-B stream has bytes before its start code')
                output.append(self._preamble(bytes(self._buffer[:start])))
                del self._buffer[:start]
                self._prefix_length = prefix_length
                self._seen_prefix = True
                self._scan_from = prefix_length
            else:
                output.append(self._unit(bytes(self._buffer[:start]), self._prefix_length))
                del self._buffer[:start]
                self._prefix_length = prefix_length
                self._scan_from = prefix_length
        if self.bootstrap_buffered_bytes > self.MAX_BOOTSTRAP_BYTES:
            self._error('HEVC decoder bootstrap exceeds the configured byte bound')
        result = b''.join(output)
        self.output_bytes += len(result)
        return result

    def finish(self):
        self._check_open()
        self._finished = True
        if not self._seen_prefix:
            if self.input_bytes:
                self._error('HEVC Annex-B stream ended before its first start code')
            return b''
        result = self._unit(bytes(self._buffer), self._prefix_length)
        self._buffer.clear()
        if not self.bootstrap_ready:
            self.bootstrap_discarded_nals += len(self._parameter_sets)
            self.bootstrap_discarded_bytes += self._parameter_bytes
            self._parameter_sets.clear()
            self._parameter_bytes = 0
        self.output_bytes += len(result)
        return result
