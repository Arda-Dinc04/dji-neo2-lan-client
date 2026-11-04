"""Lossless JSONL capture with nanosecond timestamps and optional annotations."""

from __future__ import annotations

from dataclasses import dataclass
from copy import deepcopy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import time
from typing import Any

from .protocol import decode_datagram


@dataclass(frozen=True)
class CaptureEvent:
    sequence: int
    timestamp: str
    timestamp_ns: int
    monotonic_ns: int
    interface: str
    direction: str
    peer: str
    raw_hex: str
    decoded: dict[str, Any]
    note: str = ""
    observation: dict[str, Any] | None = None


class CaptureRecorder:
    """Keep every packet; live owners may defer diagnostic decoding to reports."""
    def __init__(self, path: str | Path, *, decode_annotations: bool = True, persistent=False):
        if type(decode_annotations) is not bool:
            raise ValueError('capture annotation option must be boolean')
        if type(persistent) is not bool:raise ValueError('capture persistent option must be boolean')
        self.path = Path(path)
        self.decode_annotations = decode_annotations
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._sequence = 0
        self.persistent = persistent
        self._fd = None
        self._entered = False

    def __enter__(self):
        with self._lock:
            if self._entered:raise RuntimeError('capture_writer_cannot_be_reused')
            self._entered=True
            if self.persistent:
                self._fd=os.open(self.path,os.O_WRONLY|os.O_CREAT|os.O_APPEND|
                    os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
        return self

    def __exit__(self,*args):
        with self._lock:
            if self._fd is not None:
                fd,self._fd=self._fd,None
                os.close(fd)

    def record(
        self,
        *,
        interface: str,
        direction: str,
        peer: str,
        raw: bytes,
        note: str = "",
        timestamp_ns: int | None = None,
        monotonic_ns: int | None = None,
        observation: dict[str, Any] | None = None,
    ) -> CaptureEvent:
        now_ns = time.time_ns() if timestamp_ns is None else timestamp_ns
        received_ns = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        if any(type(value) is not int or value < 0 for value in (now_ns, received_ns)):
            raise ValueError('capture timestamps must be nonnegative integer nanoseconds')
        if observation is not None and not isinstance(observation, dict):
            raise ValueError('capture observation must be a dictionary')
        with self._lock:
            if self.persistent and self._fd is None:
                raise RuntimeError('capture_persistent_writer_not_open')
            self._sequence += 1
            decoded = {"length": len(raw)}
            if interface in ("udp", "ble-duml"):
                decoded = (decode_datagram(raw) if self.decode_annotations else
                           {"length": len(raw), "annotations_deferred": True})
            event = CaptureEvent(
                sequence=self._sequence,
                timestamp=datetime.fromtimestamp(now_ns / 1_000_000_000, timezone.utc).isoformat(),
                timestamp_ns=now_ns,
                monotonic_ns=received_ns,
                interface=interface,
                direction=direction,
                peer=peer,
                raw_hex=raw.hex(),
                decoded=decoded,
                note=note,
                observation=deepcopy(observation),
            )
            # All fields already contain JSON values. Recursive dataclass
            # copying needlessly rewalks every annotation/receipt per packet.
            record=vars(event).copy()
            if observation is None:record.pop('observation')
            line=json.dumps(record, ensure_ascii=False, separators=(",", ":"))+'\n'
            if self.persistent:
                # Each line reaches the kernel before returning. No background
                # queue or Python write buffer can lose its tail on SIGKILL.
                pending=memoryview(line.encode('utf-8'))
                while pending:
                    written=os.write(self._fd,pending)
                    if written<=0:raise OSError('capture_write_failed')
                    pending=pending[written:]
            else:
                with self.path.open('a',encoding='utf-8',newline='\n') as stream:
                    stream.write(line)
        return event


def load_capture(path: str | Path) -> list[dict[str, Any]]:
    events = []
    with Path(path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid JSONL at line {line_number}: {error}") from error
    return events
