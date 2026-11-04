"""Bounded UDP receipt and owner-cost diagnostics; never flight evidence.

Linux timestamps use CLOCK_REALTIME. Their comparison with a paired userspace
clock is diagnostic only: neither timestamp nor a derived delay renews a lease,
changes protocol receipt times, or authorizes a command.
"""
from contextlib import contextmanager
from copy import deepcopy
import errno
import platform
import socket
import struct
import sys
import time


# Linux asm-generic/socket.h. NEW uses two signed 64-bit fields on both
# 32- and 64-bit hosts. Do not guess these option numbers on other ABIs.
_GENERIC_LINUX = sys.platform == 'linux' and platform.machine().lower() in (
    'x86_64', 'amd64', 'i386', 'i686', 'aarch64', 'arm64', 'armv6l', 'armv7l')
_TIMESTAMPNS_NEW = getattr(socket, 'SO_TIMESTAMPNS_NEW', 64 if _GENERIC_LINUX else None)
_RXQ_OVFL = getattr(socket, 'SO_RXQ_OVFL', 40 if _GENERIC_LINUX else None)
_PAIR_LIMIT_NS = 10_000_000


class OwnerCosts:
    """Fixed stage counters, without per-operation logs or growing samples."""
    STAGES = frozenset(('tick', 'snapshot', 'pre_send_guard', 'socket_send',
        'send_receipt', 'protocol_delivery', 'protocol_journal',
        'raw_video_write', 'video_enqueue', 'capture_append'))

    def __init__(self, *, clock=time.monotonic_ns):
        self.clock = clock
        self.values = {}

    @contextmanager
    def measure(self, name):
        if name not in self.STAGES:
            raise ValueError('unknown_owner_cost_stage')
        start = self.clock()
        try:
            yield
        finally:
            elapsed = max(0, self.clock()-start)
            item = self.values.setdefault(name, {'count':0, 'total_ns':0,
                                                'maximum_ns':0, 'last_ns':0})
            item['count'] += 1
            item['total_ns'] += elapsed
            item['maximum_ns'] = max(item['maximum_ns'], elapsed)
            item['last_ns'] = elapsed

    def snapshot(self):
        return deepcopy(self.values)


class UdpReceiptObserver:
    """Enable optional socket metadata on an already owned UDP socket.

    Unsupported options fall back to the existing userspace receipt clocks.
    Errors from recv/recvmsg still propagate to the transport owner.
    """
    def __init__(self, sock, *, monotonic_ns=time.monotonic_ns,
                 realtime_ns=time.time_ns):
        self.sock, self.monotonic_ns, self.realtime_ns = sock, monotonic_ns, realtime_ns
        self.timestamp_enabled = self.overflow_enabled = False
        self.option_errors = {}
        self.previous_pair = None
        self.previous_drops = 0
        self.receipts = self.timestamped = self.unusable_timestamps = self.reported_drops = 0
        self.maximum_delay_ns = self.last_delay_ns = None
        self.last = None
        if not callable(getattr(sock, 'recvmsg', None)):
            self.option_errors['recvmsg'] = 'unavailable'
            return
        for name, option in (('timestamp', _TIMESTAMPNS_NEW), ('overflow', _RXQ_OVFL)):
            if option is None:
                self.option_errors[name] = 'unsupported_abi'
                continue
            try:
                sock.setsockopt(socket.SOL_SOCKET, option, 1)
            except OSError as error:
                self.option_errors[name] = errno.errorcode.get(error.errno, 'OSERROR')
            else:
                setattr(self, name+'_enabled', True)

    def receive(self, maximum=65535):
        if self.timestamp_enabled or self.overflow_enabled:
            raw, ancillary, flags, _ = self.sock.recvmsg(
                maximum, socket.CMSG_SPACE(16)+socket.CMSG_SPACE(4))
        else:
            raw, ancillary, flags = self.sock.recv(maximum), (), 0
        # Preserve the operational socket-read clock before parsing metadata.
        monotonic = self.monotonic_ns()
        realtime = self.realtime_ns()
        paired_at = self.monotonic_ns()
        details = self.observe(ancillary, flags, monotonic, realtime, paired_at)
        return raw, monotonic, realtime, details

    def observe(self, ancillary, flags, monotonic, realtime, paired_at):
        kernel = drops = None
        truncated = bool(flags & socket.MSG_CTRUNC)
        malformed = False
        if not truncated:
            for level, kind, data in ancillary:
                if level != socket.SOL_SOCKET:
                    continue
                if self.timestamp_enabled and kind == _TIMESTAMPNS_NEW:
                    if len(data) != 16 or kernel is not None:
                        malformed = True
                        continue
                    seconds, nanos = struct.unpack('=qq', data)
                    if seconds < 0 or not 0 <= nanos < 1_000_000_000:
                        malformed = True
                    else:
                        kernel = seconds*1_000_000_000+nanos
                elif self.overflow_enabled and kind == _RXQ_OVFL:
                    if len(data) != 4 or drops is not None:
                        malformed = True
                    else:
                        drops = struct.unpack('=I', data)[0]
        pair_span = paired_at-monotonic
        reason = ('ancillary_truncated' if truncated else 'ancillary_malformed' if malformed
            else 'kernel_timestamp_unavailable' if kernel is None else None)
        if not 0 <= pair_span <= _PAIR_LIMIT_NS:
            reason = 'clock_pair_delayed'
        if self.previous_pair is not None:
            old_monotonic, old_realtime, old_span = self.previous_pair
            drift = (realtime-old_realtime)-(monotonic-old_monotonic)
            if abs(drift) > _PAIR_LIMIT_NS+max(pair_span,0)+max(old_span,0):
                reason = 'realtime_discontinuity'
        self.previous_pair = monotonic, realtime, pair_span
        if kernel is not None and realtime < kernel:
            reason = 'kernel_timestamp_in_future'
        delay = realtime-kernel if reason is None else None
        self.receipts += 1
        if kernel is not None:self.timestamped += 1
        if delay is None:self.unusable_timestamps += 1
        else:
            self.maximum_delay_ns = max(self.maximum_delay_ns or 0,delay)
        self.last_delay_ns = delay
        if drops is not None and not malformed:
            self.reported_drops += (drops-self.previous_drops) & 0xffffffff
            self.previous_drops = drops
        self.last = {'kernel_realtime_ns':kernel, 'clock_pair_span_ns':pair_span,
            'kernel_to_userspace_ns':delay, 'delay_unavailable_reason':reason,
            'socket_drop_counter':drops, 'ancillary_truncated':truncated,
            'payload_truncated':bool(flags & socket.MSG_TRUNC)}
        return dict(self.last)

    def snapshot(self):
        return {'timestamp_enabled':self.timestamp_enabled,
            'overflow_enabled':self.overflow_enabled, 'option_errors':dict(self.option_errors),
            'clock':'realtime', 'operational_clock':'userspace_monotonic',
            'diagnostic_only':True, 'receipts':self.receipts,
            'timestamped':self.timestamped, 'unusable_timestamps':self.unusable_timestamps,
            'kernel_to_userspace_maximum_ns':self.maximum_delay_ns,
            'kernel_to_userspace_last_ns':self.last_delay_ns,
            'reported_socket_drops':self.reported_drops,
            'last':deepcopy(self.last)}
