#!/usr/bin/env python3
"""Decode the DUML tunnelled inside SWUDP ``0x51/0x01`` frames.

A clear-text SWUDP capture (from ``deploy/capture_swudp_mitm.py`` or a monitor
adapter) shows that the phone's uplink is dominated by ``0x51/0x01`` frames.
These are not a command: ``0x51/0x01`` is the Wi-Fi link's DUML tunnel, sent
from the Wi-Fi host to the aircraft, and each frame wraps one real application
DUML command in its payload (heartbeat ``0x00/0x00``, XRCE subscribe
``0x00/0x99``, monitor ``0x03/0xDA`` and dozens more, all ``2.0 -> 8.x``).

A byte-scanning finder does not see the tunnelled commands: it walks over the
outer ``0x51/0x01`` frame and never recurses into its payload. This tool walks
frames strictly by their declared length and CRC, then recurses one level into
every ``0x51/0x01`` payload, so the real command inventory becomes visible.

It reads a pcap through ``tshark`` and prints, for every UDP/9003 datagram from
the given source, the tunnelled command inventory. It sends nothing.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from neo2_control.duml import DumlPacket, crc8, crc16, decode_host_id, SOF  # noqa: E402

TUNNEL = (0x51, 0x01)


def walk(blob: bytes):
    """Yield DumlPacket by strict sequential, CRC-validated framing."""
    offset = 0
    while offset + 13 <= len(blob):
        if blob[offset] != SOF or crc8(blob[offset:offset + 3]) != blob[offset + 3]:
            offset += 1
            continue
        length = int.from_bytes(blob[offset + 1:offset + 3], "little") & 0x3FF
        if length < 13 or offset + length > len(blob):
            offset += 1
            continue
        frame = blob[offset:offset + length]
        if crc16(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
            offset += 1
            continue
        try:
            yield DumlPacket.decode(frame)
        except ValueError:
            offset += 1
            continue
        offset += length


def _tshark_payloads(pcap: str, display_filter: str) -> list[bytes]:
    result = subprocess.run(
        ["tshark", "-r", pcap, "-Y", display_filter,
         "-T", "fields", "-e", "frame.time_relative", "-e", "data.data"],
        capture_output=True, text=True, check=True,
    )
    payloads = []
    seen = set()
    for line in result.stdout.strip().splitlines():
        parts = line.split("\t")
        if len(parts) < 2 or not parts[1]:
            continue
        payload = bytes.fromhex(parts[1].replace(":", ""))
        key = (parts[0], payload)
        if key in seen:  # a MITM capture sees each forwarded packet twice
            continue
        seen.add(key)
        payloads.append(payload)
    return payloads


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcap")
    parser.add_argument("--source-ip", default="192.168.2.107",
                        help="uplink source (phone) to inventory")
    parser.add_argument("--json", type=Path, help="write the inventory as JSON")
    parser.add_argument("--catalogue", action="append", default=[], metavar="LIB",
                        help="SDK .so used to name triplets (repeatable)")
    args = parser.parse_args()

    names: dict[tuple[int, int], str] = {}
    if args.catalogue:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from extract_command_catalogue import extract_catalogue  # noqa: E402
        for lib in args.catalogue:
            for row in extract_catalogue(Path(lib).read_bytes()):
                names.setdefault((row["command_set"], row["command_id"]), row["request"])

    payloads = _tshark_payloads(
        args.pcap, f"udp.port==9003 && ip.src=={args.source_ip}")
    outer = Counter()
    inner = Counter()
    inner_hosts = Counter()
    tunnels = 0
    for payload in payloads:
        for packet in walk(payload):
            outer[(packet.cmd_set, packet.cmd_id)] += 1
            if (packet.cmd_set, packet.cmd_id) == TUNNEL:
                tunnels += 1
                for nested in walk(packet.payload):
                    inner[(nested.cmd_set, nested.cmd_id)] += 1
                    st = decode_host_id(nested.sender)
                    rt = decode_host_id(nested.receiver)
                    inner_hosts[(f"{st[0]}.{st[1]}", f"{rt[0]}.{rt[1]}")] += 1

    print(f"datagrams: {len(payloads)}   0x51/0x01 tunnels: {tunnels}")
    print(f"outer frames: {[(f'0x{c:02x}/0x{i:02x}', n) for (c, i), n in outer.most_common(6)]}")
    named = sum(1 for key in inner if key in names)
    suffix = f" ({named}/{len(inner)} named)" if names else ""
    print(f"\ntunnelled (real) DUML commands, {sum(inner.values())} total, "
          f"{len(inner)} triplets{suffix}:")
    for (cmd_set, cmd_id), count in inner.most_common(len(inner)):
        name = names.get((cmd_set, cmd_id), "")
        print(f"  0x{cmd_set:02x}/0x{cmd_id:02x}  x{count:<6} {name}")
    print("\ntunnelled host pairs:")
    for (src, dst), count in inner_hosts.most_common(10):
        print(f"  {src:>6} -> {dst:>6}  x{count}")

    if args.json:
        args.json.write_text(json.dumps({
            "datagrams": len(payloads),
            "tunnels": tunnels,
            "tunnelled_commands": {
                f"0x{c:02x}/0x{i:02x}": {"count": n, "name": names.get((c, i), "")}
                for (c, i), n in inner.most_common()
            },
            "tunnelled_hosts": {f"{s}->{d}": n for (s, d), n in inner_hosts.most_common()},
        }, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
