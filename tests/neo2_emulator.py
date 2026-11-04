"""Bounded stand-in for the Neo 2 SWUDP endpoint, shared by the probe tests.

The emulator accepts one session, then acknowledges reliable uplink packets in
the order they arrive. It never initiates a stream, never invents a response
body it has not been given, and never carries a flight command.
"""

from __future__ import annotations

import socket
import threading

from neo2_control.duml import DumlPacket
from neo2_control.transport import STREAM_HEADER_LENGTH, xor_checksum

# Exact 15-byte acceptance observed on the Neo 2, session bytes excluded.
SESSION_ACCEPT_TEMPLATE = "0f800000000000000101000104401f"
TYPE_1_HEADER_LENGTH = 34


class Neo2Emulator:
    def __init__(self, *, timeout: float = 2.0) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("127.0.0.1", 0))
        self._socket.settimeout(timeout)
        self.port = self._socket.getsockname()[1]
        self.received: list[bytes] = []
        # Offset applied to the drone video stream window. Every real session
        # captured so far leaves it at the link anchor: nothing is queued.
        self.video_window_advance = 0
        self._session = b"\x00\x00"
        self._anchor = 0

    def accept_session(self) -> bytes:
        """Receive the SYN and reply with the exact acceptance datagram."""

        syn, peer = self._socket.recvfrom(65535)
        self.received.append(syn)
        self._session = syn[2:4]
        self._anchor = int.from_bytes(syn[8:10], "little")
        accept = bytearray.fromhex(SESSION_ACCEPT_TEMPLATE)
        accept[2:4] = self._session
        accept[7] = xor_checksum(accept[:7])
        self._socket.sendto(bytes(accept), peer)
        return syn

    def acknowledge_next(self, *, response_payload: bytes | None = None) -> bytes:
        """Acknowledge one type 5, optionally echoing a DUML response.

        ``response_payload`` is opaque on purpose: no response body of the
        reconstructed requests has been observed on the wire yet.
        """

        request, peer = self._socket.recvfrom(65535)
        self.received.append(request)
        duml = b""
        if response_payload is not None:
            duml = self._echo_response(request, response_payload)
        self._socket.sendto(self._type_1_ack(request, duml), peer)
        return request

    def run_in_background(
        self,
        *response_payloads: bytes | None,
    ) -> threading.Thread:
        """Accept one session then acknowledge one packet per argument."""

        def sequence() -> None:
            self.accept_session()
            for payload in response_payloads:
                self.acknowledge_next(response_payload=payload)

        thread = threading.Thread(target=sequence)
        thread.start()
        return thread

    def close(self) -> None:
        self._socket.close()

    def _type_1_ack(self, request: bytes, duml: bytes) -> bytes:
        """Build the measured 34-byte state: three window blocks then DUML."""

        video_end = (self._anchor + self.video_window_advance) & 0xFFFF
        ack = bytearray(TYPE_1_HEADER_LENGTH + len(duml))
        ack[:2] = (0x8000 | len(ack)).to_bytes(2, "little")
        ack[2:4] = self._session
        ack[6] = 1
        ack[8:10] = self._anchor.to_bytes(2, "little")
        ack[10:12] = video_end.to_bytes(2, "little")
        ack[16:18] = self._anchor.to_bytes(2, "little")
        ack[18:20] = self._anchor.to_bytes(2, "little")
        # Block 3 collapses on the sequence being acknowledged.
        ack[24:26] = request[4:6]
        ack[26:28] = request[4:6]
        ack[32:34] = len(duml).to_bytes(2, "little")
        ack[7] = xor_checksum(ack[:7])
        ack[TYPE_1_HEADER_LENGTH:] = duml
        return bytes(ack)

    @staticmethod
    def _echo_response(request: bytes, payload: bytes) -> bytes:
        duml = DumlPacket.decode(request[STREAM_HEADER_LENGTH:])
        return DumlPacket(
            sender=duml.receiver,
            receiver=duml.sender,
            seq=duml.seq,
            cmd_type=0x80,
            cmd_set=duml.cmd_set,
            cmd_id=duml.cmd_id,
            payload=payload,
        ).encode()
