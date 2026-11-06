"""Real child limit application and explicit platform memory-policy behavior."""
import json
import os
import resource
import subprocess
import sys

import pytest

from neo2_control import process_limits as limits
from neo2_control.video import HlsVideo


def test_linux_command_preserves_existing_prlimit_profile():
    policy = limits.VideoProcessLimits(platform='linux', linux_limiter='/usr/bin/prlimit')
    assert policy.wrap(['/usr/bin/ffmpeg', '-version']) == [
        '/usr/bin/prlimit', '--as=1610612736', '--nofile=64', '--fsize=4194304',
        '--', '/usr/bin/ffmpeg', '-version']
    assert policy.snapshot()['address_space_hard_limit_bytes'] == 1610612736
    assert policy.snapshot()['memory_hard_cap']


def test_missing_linux_limiter_and_unknown_platform_fail_closed():
    with pytest.raises(limits.LimitPolicyError, match='requires_ffmpeg_and_prlimit'):
        limits.VideoProcessLimits(platform='linux', linux_limiter=None)
    with pytest.raises(limits.LimitPolicyError, match='unsupported_platform'):
        limits.VideoProcessLimits(platform='unsupported')


@pytest.mark.skipif(sys.platform != 'darwin', reason='Darwin native memory sampler')
def test_darwin_exec_wrapper_applies_hard_file_limits_and_preserves_pid():
    policy = limits.VideoProcessLimits()
    command = policy.wrap([sys.executable, '-c',
        'import json,os,resource; print(json.dumps({"pid":os.getpid(),'
        '"nofile":resource.getrlimit(resource.RLIMIT_NOFILE),'
        '"fsize":resource.getrlimit(resource.RLIMIT_FSIZE)}))'])
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               env={'PATH': '/usr/bin:/bin'})
    stdout, stderr = process.communicate(timeout=5)
    assert process.returncode == 0, stderr
    result = json.loads(stdout)
    assert result['pid'] == process.pid
    assert result['nofile'] == [64, 64]
    assert result['fsize'] == [4194304, 4194304]
    snapshot = policy.snapshot()
    assert snapshot['address_space_hard_limit_bytes'] is None
    assert snapshot['resident_memory_watchdog_bytes'] == 1610612736
    assert not snapshot['memory_hard_cap']
    assert '-I' in command and 'preexec_fn' not in command


@pytest.mark.skipif(sys.platform != 'darwin', reason='Darwin native memory sampler')
def test_native_sampler_reads_owned_process_without_shell():
    resident = limits._DarwinResidentMemory().read(os.getpid())
    assert resident > 0


class OwnedProcess:
    pid = 12345

    def poll(self):
        return None


def test_memory_watchdog_fails_closed_on_excess_or_monitor_loss(monkeypatch):
    class Memory:
        def read(self, pid):
            assert pid == 12345
            return 101
    monkeypatch.setattr(limits, '_DarwinResidentMemory', Memory)
    policy = limits.VideoProcessLimits(platform='darwin', memory_bytes=100)
    with pytest.raises(limits.LimitPolicyError, match='memory_limit_exceeded'):
        policy.check_memory([OwnedProcess()])
    assert policy.snapshot()['peak_sampled_process_resident_bytes'] == 101

    def unavailable(pid):
        raise OSError('synthetic monitor failure')
    policy._memory.read = unavailable
    with pytest.raises(limits.LimitPolicyError, match='memory_monitor_unavailable'):
        policy.check_memory([OwnedProcess()])


@pytest.mark.skipif(sys.platform != 'darwin', reason='Darwin video memory policy')
def test_video_memory_watchdog_reaps_decoder_on_limit_excess(tmp_path, monkeypatch):
    # Exercise the real video worker/cleanup boundary, not just the sampler.
    video = HlsVideo(tmp_path / 'hls')
    video.process_limits.memory_bytes = 1
    with video:
        from test_observation import until
        until(lambda: video.closed)
    result = video.snapshot()
    assert result['error'] == 'video_process_memory_limit_exceeded'
    assert result['cleanup_verified'] and video.process.poll() is not None
    assert not video.feed(b'ignored')
