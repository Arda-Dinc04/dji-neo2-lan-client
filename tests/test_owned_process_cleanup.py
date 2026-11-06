"""Owned synthetic child cleanup when Darwin group signaling races with exit."""
import os
import signal
import subprocess
import sys

import pytest

from neo2_control import process_limits
from neo2_control import video as module
from neo2_control.video import HlsVideo
from test_hevc import nal
from test_observation import until


@pytest.mark.parametrize('pipeline', ['no_bootstrap', 'stalled'])
def test_pipeline_reaps_exact_children_when_group_signal_is_denied(tmp_path, monkeypatch, pipeline):
    denied, fallback = [], []
    original = subprocess.Popen.send_signal
    def deny_group(pid, sig):
        denied.append((pid, sig))
        raise PermissionError(1, 'synthetic group permission race')
    def send_owned(process, sig):
        fallback.append((process.pid, sig))
        return original(process, sig)
    monkeypatch.setattr(module.os, 'killpg', deny_group)
    monkeypatch.setattr(subprocess.Popen, 'send_signal', send_owned)
    if pipeline == 'stalled':
        monkeypatch.setattr(HlsVideo, '_normalizer_command',
                            lambda self: [sys.executable, '-u', '-c', 'import time; time.sleep(30)'])
        monkeypatch.setattr(module, 'MAX_QUEUED_CHUNKS', 8)
    video = HlsVideo(tmp_path / 'hls', input_codec='hevc', preview_jpeg=True)
    try:
        with video:
            if pipeline == 'no_bootstrap':
                assert video.feed(nal(1) + nal(35))
                until(lambda: video.snapshot()['queued_bytes'] == 0)
            else:
                assert video.feed(nal(32) + nal(33) + nal(34) + nal(20) + nal(35))
                until(lambda: video.snapshot()['hevc_bootstrap']['ready'])
                chunk = b'\0\0\0\1\x02\x01' + b'X' * 4090
                for _ in range(10000):
                    if not video.feed(chunk):
                        break
                else:
                    pytest.fail('synthetic stalled queue remained unbounded')
        assert video.snapshot()['cleanup_verified']
        assert video.snapshot()['error'] == ('video_no_codec_bootstrap' if pipeline == 'no_bootstrap'
                                             else 'video_input_queue_full')
        assert denied and fallback and denied == fallback
        owned = {video.process.pid, video.normalizer.pid}
        assert {pid for pid, _ in fallback} <= owned
        for process in (video.normalizer, video.process):
            assert process.poll() is not None
            with pytest.raises(ProcessLookupError):
                os.kill(process.pid, 0)
    finally:
        # The intentionally failing pre-fix run must not leave synthetic children.
        for process in (video.normalizer, video.process):
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=2)


class Child:
    pid = 123456789

    def __init__(self, code=None, fail=False):
        self.code, self.fail = code, fail
        self.signals = []

    def poll(self): return self.code

    def send_signal(self, sig):
        self.signals.append(sig)
        if self.fail:
            raise PermissionError(1, 'synthetic owned PID permission denial')


def test_ended_child_receives_no_signal(monkeypatch):
    monkeypatch.setattr(process_limits.os, 'killpg', lambda *args: pytest.fail('ended child signaled'))
    child = Child(code=0)
    process_limits.signal_owned_child(child, signal.SIGTERM)
    assert not child.signals


def test_group_permission_failure_falls_back_only_to_owned_popen(monkeypatch):
    child = Child()
    calls = []
    def denied(pid, sig):
        calls.append((pid, sig))
        raise PermissionError(1, 'synthetic group permission denial')
    monkeypatch.setattr(process_limits.os, 'killpg', denied)
    process_limits.signal_owned_child(child, signal.SIGKILL)
    assert calls == [(child.pid, signal.SIGKILL)] and child.signals == [signal.SIGKILL]


def test_both_group_and_owned_pid_denial_remain_failure(monkeypatch):
    child = Child(fail=True)
    def denied(*args): raise PermissionError(1, 'synthetic group permission denial')
    monkeypatch.setattr(process_limits.os, 'killpg', denied)
    with pytest.raises(PermissionError):
        process_limits.signal_owned_child(child, signal.SIGTERM)
    assert child.signals == [signal.SIGTERM]


def test_already_disappeared_group_is_harmless(monkeypatch):
    def gone(*args): raise ProcessLookupError(3, 'synthetic vanished process group')
    monkeypatch.setattr(process_limits.os, 'killpg', gone)
    child = Child()
    process_limits.signal_owned_child(child, signal.SIGTERM)
    assert child.signals == []
