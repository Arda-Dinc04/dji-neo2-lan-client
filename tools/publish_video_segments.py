#!/usr/bin/env python3
"""Publish bounded HLS MPEG-TS segments through the existing Pi agent channel."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import time
from typing import Any
from urllib.error import HTTPError
from urllib.parse import quote, urlencode
from urllib.request import Request, urlopen


MAX_SEGMENT_BYTES = 2_500_000
MAX_RESPONSE_BYTES = 1_048_576
SEGMENT_RE = re.compile(r"(\d+)\.ts$")


class VideoPublisher:
    def __init__(
        self,
        server: str,
        agent_id: str,
        token: str,
        *,
        timeout: float = 15,
    ) -> None:
        if not server.startswith("https://"):
            raise ValueError("server must be an HTTPS URL")
        if not agent_id or len(agent_id) > 100:
            raise ValueError("agent_id is required")
        if not token:
            raise ValueError("agent token is required")
        self.server = server.rstrip("/")
        self.agent_id = agent_id
        self.token = token
        self.timeout = timeout
        self.session_id: str | None = None

    def start(
        self,
        *,
        width: int | None = None,
        height: int | None = None,
        nominal_fps: float | None = None,
    ) -> dict[str, Any]:
        metadata = {
            "codec": "h264",
            "width": width,
            "height": height,
            "nominal_fps": nominal_fps,
        }
        response = self._request_json(
            "POST",
            f"/agents/{quote(self.agent_id, safe='')}/drone/sessions",
            json.dumps(metadata, separators=(",", ":")).encode(),
            "application/json",
        )
        session_id = response.get("sessionId")
        if not isinstance(session_id, str) or not session_id:
            raise RuntimeError("video session response has no sessionId")
        self.session_id = session_id
        return response

    def upload(self, sequence: int, duration: float, segment: bytes) -> dict[str, Any]:
        if self.session_id is None:
            raise RuntimeError("video session has not been started")
        if sequence < 0:
            raise ValueError("sequence must be positive")
        if not 0 < duration <= 5:
            raise ValueError("duration must be between 0 and 5 seconds")
        if not segment or len(segment) > MAX_SEGMENT_BYTES:
            raise ValueError(
                f"segment must contain between 1 and {MAX_SEGMENT_BYTES} bytes"
            )
        query = urlencode({"duration": f"{duration:.3f}"})
        return self._request_json(
            "PUT",
            (
                f"/agents/{quote(self.agent_id, safe='')}/drone/sessions/"
                f"{quote(self.session_id, safe='')}/segments/{sequence}?{query}"
            ),
            segment,
            "video/mp2t",
        )

    def stop(self) -> dict[str, Any] | None:
        if self.session_id is None:
            return None
        session_id = self.session_id
        self.session_id = None
        return self._request_json(
            "POST",
            (
                f"/agents/{quote(self.agent_id, safe='')}/drone/sessions/"
                f"{quote(session_id, safe='')}/stop"
            ),
            b"",
            "application/octet-stream",
        )

    def _request_json(
        self,
        method: str,
        path: str,
        body: bytes,
        content_type: str,
    ) -> dict[str, Any]:
        request = Request(
            self.server + path,
            data=body,
            method=method,
            headers={
                "Authorization": f"Bearer {self.token}",
                "Content-Type": content_type,
                "User-Agent": "neo2-video-publisher/0.1",
            },
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                data = response.read(MAX_RESPONSE_BYTES + 1)
        except HTTPError as error:
            detail = error.read(4096).decode("utf-8", errors="replace")
            raise RuntimeError(f"video server returned HTTP {error.code}: {detail}") from error
        if len(data) > MAX_RESPONSE_BYTES:
            raise RuntimeError("video server response is too large")
        parsed = json.loads(data or b"{}")
        if not isinstance(parsed, dict):
            raise RuntimeError("video server returned a non-object response")
        return parsed


def segment_sequence(path: Path) -> int | None:
    match = SEGMENT_RE.search(path.name)
    return int(match.group(1)) if match else None


def ready_segments(
    directory: Path,
    observed_sizes: dict[Path, int],
    uploaded: set[Path],
) -> list[Path]:
    ready: list[tuple[int, Path]] = []
    for path in directory.glob("*.ts"):
        if path in uploaded or not path.is_file():
            continue
        size = path.stat().st_size
        previous = observed_sizes.get(path)
        observed_sizes[path] = size
        sequence = segment_sequence(path)
        if sequence is not None and size > 0 and size == previous:
            ready.append((sequence, path))
    return [path for _, path in sorted(ready)]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("segment_dir", type=Path)
    parser.add_argument(
        "--server",
        default="https://example.com/home-server/api",
    )
    parser.add_argument("--agent-id", default="example-agent")
    parser.add_argument("--token-env", default="HOME_AGENT_REMOTE_TOKEN")
    parser.add_argument("--segment-duration", type=float, default=1.0)
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--fps", type=float)
    parser.add_argument("--poll-interval", type=float, default=0.15)
    parser.add_argument("--delete-after-upload", action="store_true")
    parser.add_argument(
        "--include-existing",
        action="store_true",
        help="publish segments already present when the process starts",
    )
    args = parser.parse_args()

    token = os.environ.get(args.token_env, "")
    if not token:
        raise SystemExit(f"missing agent token in environment variable {args.token_env}")
    if not args.segment_dir.is_dir():
        raise SystemExit("segment directory does not exist")
    if not 0.05 <= args.poll_interval <= 5:
        raise SystemExit("poll interval must be between 0.05 and 5 seconds")

    publisher = VideoPublisher(args.server, args.agent_id, token)
    observed_sizes: dict[Path, int] = {}
    uploaded: set[Path] = set()
    if not args.include_existing:
        uploaded.update(args.segment_dir.glob("*.ts"))

    try:
        while True:
            paths = ready_segments(args.segment_dir, observed_sizes, uploaded)
            for path in paths:
                sequence = segment_sequence(path)
                if sequence is None:
                    continue
                segment = path.read_bytes()
                if publisher.session_id is None:
                    publisher.start(
                        width=args.width,
                        height=args.height,
                        nominal_fps=args.fps,
                    )
                    print("video session started", flush=True)
                publisher.upload(sequence, args.segment_duration, segment)
                uploaded.add(path)
                observed_sizes.pop(path, None)
                print(
                    f"uploaded segment {sequence} ({len(segment)} bytes)",
                    flush=True,
                )
                if args.delete_after_upload:
                    path.unlink()
            time.sleep(args.poll_interval)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            publisher.stop()
        except (OSError, RuntimeError, ValueError) as error:
            print(f"unable to stop video session: {error}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
