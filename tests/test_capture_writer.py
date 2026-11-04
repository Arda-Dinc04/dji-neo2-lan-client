"""Persistent capture emits each line synchronously and closes on failure."""
import json
import os

import pytest

from neo2_control.capture import CaptureRecorder,load_capture


def record(writer):
    return writer.record(interface='udp',direction='rx',peer='127.0.0.1:1',
        raw=b'\x00\xff',timestamp_ns=1000,monotonic_ns=100,
        observation={'kernel_realtime_ns':900})


def test_each_record_is_visible_before_context_close_and_descriptor_is_closed(tmp_path):
    path=tmp_path/'capture.jsonl'
    writer=CaptureRecorder(path,decode_annotations=False,persistent=True)
    with pytest.raises(RuntimeError,match='not_open'):record(writer)
    with writer:
        fd=writer._fd
        for sequence in range(1,4):
            record(writer)
            assert len(load_capture(path))==sequence
    with pytest.raises(OSError):os.fstat(fd)
    with pytest.raises(RuntimeError,match='not_open'):record(writer)
    with pytest.raises(RuntimeError,match='cannot_be_reused'):writer.__enter__()


def test_short_writes_preserve_one_complete_line_and_error_still_closes(tmp_path,monkeypatch):
    write=os.write
    def short(fd,data):return write(fd,data[:7])
    monkeypatch.setattr(os,'write',short)
    path=tmp_path/'capture.jsonl'
    with CaptureRecorder(path,persistent=True) as writer:record(writer)
    assert load_capture(path)[0]['raw_hex']=='00ff'
    with pytest.raises(OSError,match='capture_write_failed'):
        with CaptureRecorder(tmp_path/'failed.jsonl',persistent=True) as writer:
            fd=writer._fd
            monkeypatch.setattr(os,'write',lambda *_:0)
            record(writer)
    with pytest.raises(OSError):os.fstat(fd)


def test_persistent_writer_refuses_symlink_target(tmp_path):
    original=tmp_path/'original'
    original.write_text('unchanged')
    link=tmp_path/'capture.jsonl';link.symlink_to(original)
    with pytest.raises(OSError):
        with CaptureRecorder(link,persistent=True):pass
    assert original.read_text()=='unchanged'


def test_default_and_persistent_writers_produce_identical_json_values(tmp_path):
    normal=tmp_path/'normal.jsonl';persistent=tmp_path/'persistent.jsonl'
    record(CaptureRecorder(normal,decode_annotations=False))
    with CaptureRecorder(persistent,decode_annotations=False,persistent=True) as writer:record(writer)
    assert json.loads(normal.read_text())==json.loads(persistent.read_text())
