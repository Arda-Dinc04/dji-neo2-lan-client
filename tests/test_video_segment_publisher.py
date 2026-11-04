from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import json
import tempfile
import threading

import pytest

from tools.publish_video_segments import (
    MAX_SEGMENT_BYTES,
    VideoPublisher,
    ready_segments,
)


class RecordingHandler(BaseHTTPRequestHandler):
    calls: list[dict[str, object]] = []

    def _handle(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length)
        self.calls.append(
            {
                "method": self.command,
                "path": self.path,
                "authorization": self.headers.get("Authorization"),
                "content_type": self.headers.get("Content-Type"),
                "body": body,
            }
        )
        response: dict[str, object] = {"live": True}
        if self.path.endswith("/drone/sessions"):
            response["sessionId"] = "session-1"
        encoded = json.dumps(response).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    do_POST = _handle
    do_PUT = _handle

    def log_message(self, format: str, *args: object) -> None:
        return


def test_publishes_session_segment_and_stop_without_token_in_url() -> None:
    RecordingHandler.calls = []
    server = ThreadingHTTPServer(("127.0.0.1", 0), RecordingHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        publisher = VideoPublisher(
            f"https://127.0.0.1:{server.server_port}",
            "example-agent",
            "secret-token",
        )
        # The test server is HTTP-only; replace the transport boundary while
        # preserving all request construction and validation.
        publisher.server = f"http://127.0.0.1:{server.server_port}"
        publisher.start(width=1920, height=1080, nominal_fps=30)
        publisher.upload(7, 1.0, b"mpeg-ts")
        publisher.stop()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)

    assert [call["method"] for call in RecordingHandler.calls] == [
        "POST",
        "PUT",
        "POST",
    ]
    assert RecordingHandler.calls[1]["path"].endswith(
        "/segments/7?duration=1.000"
    )
    assert RecordingHandler.calls[1]["content_type"] == "video/mp2t"
    assert all(
        call["authorization"] == "Bearer secret-token"
        for call in RecordingHandler.calls
    )
    assert all("secret-token" not in str(call["path"]) for call in RecordingHandler.calls)


def test_detects_only_stable_numbered_segments_and_enforces_bound() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        stable = root / "segment-0003.ts"
        stable.write_bytes(b"abc")
        (root / "notes.ts").write_bytes(b"ignored")
        observed: dict[Path, int] = {}
        uploaded: set[Path] = set()

        assert ready_segments(root, observed, uploaded) == []
        assert ready_segments(root, observed, uploaded) == [stable]

    publisher = VideoPublisher("https://example.test", "pi", "token")
    publisher.session_id = "session"
    with pytest.raises(ValueError, match="segment must contain"):
        publisher.upload(0, 1.0, b"x" * (MAX_SEGMENT_BYTES + 1))
