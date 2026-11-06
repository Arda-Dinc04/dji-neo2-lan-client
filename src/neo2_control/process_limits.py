"""Explicit per-platform bounds for owned video subprocesses.

Linux retains its prlimit address-space hard limit. Darwin uses a standalone
Python wrapper to set descriptor/file-size hard limits before exec, plus a
parent-side resident-memory watchdog. This is deliberately not presented as
an equivalent Darwin address-space hard cap. No preexec_fn is used in the
threaded caller. The watchdog may overshoot between samples.
"""
from __future__ import annotations

import ctypes
import errno
import os
from pathlib import Path
import sys

ADDRESS_SPACE_BYTES = 1610612736
OPEN_FILES = 64
FILE_BYTES = 4194304
MEMORY_CHECK_SECONDS = 0.25


class LimitPolicyError(RuntimeError):
    pass


class _DarwinTaskInfo(ctypes.Structure):
    # Public Darwin sys/proc_info.h, PROC_PIDTASKINFO (flavor 4).
    _fields_ = [
        ('virtual_size', ctypes.c_uint64), ('resident_size', ctypes.c_uint64),
        ('total_user', ctypes.c_uint64), ('total_system', ctypes.c_uint64),
        ('threads_user', ctypes.c_uint64), ('threads_system', ctypes.c_uint64),
        *[(name, ctypes.c_int32) for name in (
            'policy', 'faults', 'pageins', 'cow_faults', 'messages_sent',
            'messages_received', 'syscalls_mach', 'syscalls_unix', 'context_switches',
            'thread_count', 'running_threads', 'priority')],
    ]


class _DarwinResidentMemory:
    def __init__(self):
        try:
            library = ctypes.CDLL('/usr/lib/libproc.dylib', use_errno=True)
            self._query = library.proc_pidinfo
            self._query.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_uint64,
                                   ctypes.c_void_p, ctypes.c_int]
            self._query.restype = ctypes.c_int
            self.read(os.getpid())  # Fail before creating output if unavailable.
        except (OSError, AttributeError) as error:
            raise LimitPolicyError('video_memory_monitor_unavailable') from error

    def read(self, pid):
        data = _DarwinTaskInfo()
        ctypes.set_errno(0)
        count = self._query(pid, 4, 0, ctypes.byref(data), ctypes.sizeof(data))
        if count != ctypes.sizeof(data):
            code = ctypes.get_errno()
            if code == errno.ESRCH:
                raise ProcessLookupError(code, 'owned video process exited')
            raise OSError(code or errno.EIO, 'resident memory sample unavailable')
        return int(data.resident_size)


class VideoProcessLimits:
    def __init__(self, *, platform=None, linux_limiter=None,
                 memory_bytes=ADDRESS_SPACE_BYTES):
        self.platform = sys.platform if platform is None else platform
        if type(memory_bytes) is not int or memory_bytes < 1:
            raise ValueError('video memory limit must be a positive byte count')
        self.memory_bytes = memory_bytes
        self.linux_limiter = linux_limiter
        self._peak_resident_bytes = 0
        self._memory = None
        if self.platform.startswith('linux'):
            if not linux_limiter:
                raise LimitPolicyError('video_requires_ffmpeg_and_prlimit')
        elif self.platform == 'darwin':
            self._memory = _DarwinResidentMemory()
        else:
            raise LimitPolicyError('video_process_limits_unsupported_platform')

    def wrap(self, command):
        command = list(command)
        if not command or not all(isinstance(value, str) and value for value in command):
            raise ValueError('video subprocess command must contain strings')
        if self.platform.startswith('linux'):
            return [self.linux_limiter, f'--as={self.memory_bytes}',
                    f'--nofile={OPEN_FILES}', f'--fsize={FILE_BYTES}', '--', *command]
        # Absolute script path works with the intentionally minimal child env,
        # changed cwd, an editable checkout, and an installed wheel alike.
        return [sys.executable, '-I', str(Path(__file__).resolve()), '--', *command]

    def check_memory(self, processes):
        if self._memory is None:
            return
        for process in processes:
            if process is None or process.poll() is not None:
                continue
            try:
                resident = self._memory.read(process.pid)
            except ProcessLookupError:
                if process.poll() is not None:
                    continue
                raise LimitPolicyError('video_memory_monitor_unavailable') from None
            except OSError as error:
                raise LimitPolicyError('video_memory_monitor_unavailable') from error
            self._peak_resident_bytes = max(self._peak_resident_bytes, resident)
            if resident > self.memory_bytes:
                raise LimitPolicyError('video_process_memory_limit_exceeded')

    def snapshot(self):
        darwin = self.platform == 'darwin'
        return {
            'platform': self.platform,
            'launcher': 'python_setrlimit_exec' if darwin else 'linux_prlimit',
            'open_file_hard_limit': OPEN_FILES,
            'individual_file_hard_limit_bytes': FILE_BYTES,
            'address_space_hard_limit_bytes': None if darwin else self.memory_bytes,
            'resident_memory_watchdog_bytes': self.memory_bytes if darwin else None,
            'memory_sample_interval_seconds': MEMORY_CHECK_SECONDS if darwin else None,
            'peak_sampled_process_resident_bytes': self._peak_resident_bytes if darwin else None,
            'memory_scope': 'each_owned_process',
            'memory_hard_cap': not darwin,
            'memory_limit_note': ('Darwin RSS watchdog; may overshoot between samples. '
                                  'No address-space hard cap claimed.' if darwin else
                                  'Linux per-process address-space hard limit.'),
        }


def _set_hard_limit(resource_module, kind, requested):
    _, inherited_hard = resource_module.getrlimit(kind)
    limit = requested if inherited_hard == resource_module.RLIM_INFINITY else min(requested, inherited_hard)
    resource_module.setrlimit(kind, (limit, limit))
    actual = resource_module.getrlimit(kind)
    if actual != (limit, limit):
        raise RuntimeError('video child resource limit was not applied')


def _exec_limited(argv):
    """Set limits in a fresh, single-threaded interpreter; exec preserves PID."""
    if len(argv) < 2 or argv[0] != '--' or not os.path.isabs(argv[1]):
        raise ValueError('bounded video launcher requires an absolute executable')
    import resource
    _set_hard_limit(resource, resource.RLIMIT_NOFILE, OPEN_FILES)
    _set_hard_limit(resource, resource.RLIMIT_FSIZE, FILE_BYTES)
    # Darwin VM layouts and rlimit behavior differ from Linux. The parent
    # applies an explicit RSS watchdog rather than pretending RLIMIT_AS here
    # is an equivalent portable cap for dynamically loaded FFmpeg.
    os.execv(argv[1], argv[1:])


if __name__ == '__main__':
    try:
        _exec_limited(sys.argv[1:])
    except (OSError, ValueError, RuntimeError) as error:
        print('video_child_resource_setup_failed: ' + str(error), file=sys.stderr)
        raise SystemExit(126)
