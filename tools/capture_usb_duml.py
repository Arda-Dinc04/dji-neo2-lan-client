#!/usr/bin/env python3
"""Read the Neo 2 vendor-specific USB bulk channels and census their DUML.

The drone enumerates as a composite device whose interfaces 3 to 7 expose five
vendor-specific bulk pairs. Interface 4 was already observed pushing clear-text
DUML on its own, without any host request. This tool listens on every vendor
IN endpoint at once and never writes a single byte to the drone: the OUT
endpoints are opened by no code path here.

Its purpose is discriminating rather than descriptive. Six downstream streams
are already known from an idle drone; the census marks every other triplet as
new, and dumps in full any ``0x00/0x99`` — the DUML envelope of the XRCE-DDS
publish/subscribe traffic, whose payload is the missing piece of M2.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass, field
import json
from pathlib import Path
import sys
import threading
import time
from typing import Iterable

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from neo2_control.duml import decode_host_id, find_packets  # noqa: E402

DJI_VENDOR_ID = 0x2CA3
VENDOR_INTERFACE_CLASS = 0xFF
DEFAULT_INTERFACES = (3, 4, 5, 6, 7)
READ_CHUNK_BYTES = 16384
READ_TIMEOUT_MS = 500

# Triplets already observed on a drone that no application was talking to.
# Anything outside this set appeared because of what the session was doing.
IDLE_BASELINE = frozenset(
    {
        (0x04, 0x05),
        (0x03, 0xCE),
        (0x03, 0x43),
        (0x00, 0x82),
        (0x00, 0xF1),
    }
)

SUBSCRIPTION_AGENT = (0x00, 0x99)


@dataclass
class Transfer:
    monotonic_ns: int
    endpoint: int
    data: bytes


@dataclass
class EndpointResult:
    endpoint: int
    interface: int
    transfers: list[Transfer] = field(default_factory=list)
    error: str | None = None

    @property
    def byte_count(self) -> int:
        return sum(len(transfer.data) for transfer in self.transfers)


def _vendor_in_endpoints(
    device,
    wanted: Iterable[int],
) -> list[tuple[int, int]]:
    """Return ``(interface_number, endpoint_address)`` for each vendor IN pair."""

    import usb.util

    wanted = set(wanted)
    found: list[tuple[int, int]] = []
    configuration = device.get_active_configuration()
    for interface in configuration:
        if interface.bInterfaceNumber not in wanted:
            continue
        if interface.bInterfaceClass != VENDOR_INTERFACE_CLASS:
            continue
        for endpoint in interface:
            is_in = usb.util.endpoint_direction(endpoint.bEndpointAddress) == usb.util.ENDPOINT_IN
            is_bulk = usb.util.endpoint_type(endpoint.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK
            if is_in and is_bulk:
                found.append((interface.bInterfaceNumber, endpoint.bEndpointAddress))
    return sorted(set(found))


def _drain_endpoint(
    device,
    interface: int,
    endpoint: int,
    deadline_ns: int,
    result: EndpointResult,
) -> None:
    import usb.core

    while time.monotonic_ns() < deadline_ns:
        try:
            data = device.read(endpoint, READ_CHUNK_BYTES, READ_TIMEOUT_MS)
        except usb.core.USBTimeoutError:
            continue
        except usb.core.USBError as error:
            result.error = str(error)
            return
        if len(data):
            result.transfers.append(
                Transfer(time.monotonic_ns(), endpoint, bytes(data))
            )


def capture(
    *,
    vendor_id: int,
    product_id: int | None,
    interfaces: Iterable[int],
    duration: float,
) -> tuple[dict, list[EndpointResult]]:
    import usb.core
    import usb.util

    # pyusb matches every keyword literally, so an unset product must be
    # omitted rather than passed as None.
    criteria = {"idVendor": vendor_id}
    if product_id is not None:
        criteria["idProduct"] = product_id
    device = usb.core.find(**criteria)
    if device is None:
        product = f"{product_id:04x}" if product_id is not None else "*"
        raise SystemExit(f"no USB device {vendor_id:04x}:{product} present")

    identity = {
        "vendor_id": f"0x{device.idVendor:04x}",
        "product_id": f"0x{device.idProduct:04x}",
        "manufacturer": usb.util.get_string(device, device.iManufacturer) or "",
        "product": usb.util.get_string(device, device.iProduct) or "",
        "serial_number": usb.util.get_string(device, device.iSerialNumber) or "",
    }

    pairs = _vendor_in_endpoints(device, interfaces)
    if not pairs:
        raise SystemExit("no vendor-specific bulk IN endpoint matched the request")

    results = [EndpointResult(endpoint=endpoint, interface=interface) for interface, endpoint in pairs]
    claimed: list[int] = []
    for interface, _ in pairs:
        usb.util.claim_interface(device, interface)
        claimed.append(interface)

    deadline_ns = time.monotonic_ns() + int(duration * 1e9)
    threads = [
        threading.Thread(
            target=_drain_endpoint,
            args=(device, result.interface, result.endpoint, deadline_ns, result),
            daemon=True,
        )
        for result in results
    ]
    started_at = time.time()
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=duration + 10)

    for interface in claimed:
        try:
            usb.util.release_interface(device, interface)
        except Exception:  # noqa: BLE001 - releasing must never mask the capture
            pass
    usb.util.dispose_resources(device)

    identity["started_at"] = started_at
    identity["duration_s"] = duration
    return identity, results


def census(results: Iterable[EndpointResult], duration: float) -> dict:
    """Group every decodable DUML packet by endpoint and command triplet."""

    streams: dict[tuple, dict] = defaultdict(
        lambda: {"count": 0, "payload_lengths": set(), "first_offset": None}
    )
    agent_packets: list[dict] = []
    undecoded_bytes = 0
    total_packets = 0

    for result in results:
        blob = b"".join(transfer.data for transfer in result.transfers)
        consumed = 0
        for offset, packet, raw in find_packets(blob):
            total_packets += 1
            consumed += len(raw)
            sender_type, sender_index = decode_host_id(packet.sender)
            receiver_type, receiver_index = decode_host_id(packet.receiver)
            key = (
                result.endpoint,
                f"{sender_type}.{sender_index}",
                f"{receiver_type}.{receiver_index}",
                packet.cmd_type,
                packet.cmd_set,
                packet.cmd_id,
            )
            entry = streams[key]
            entry["count"] += 1
            entry["payload_lengths"].add(len(packet.payload))
            if entry["first_offset"] is None:
                entry["first_offset"] = offset
            if (packet.cmd_set, packet.cmd_id) == SUBSCRIPTION_AGENT:
                agent_packets.append(
                    {
                        "endpoint": f"0x{result.endpoint:02x}",
                        "offset": offset,
                        "sender": f"{sender_type}.{sender_index}",
                        "receiver": f"{receiver_type}.{receiver_index}",
                        "command_type": f"0x{packet.cmd_type:02x}",
                        "sequence": packet.seq,
                        "payload_length": len(packet.payload),
                        "payload_hex": packet.payload.hex(),
                        "raw_hex": raw.hex(),
                    }
                )
        undecoded_bytes += len(blob) - consumed

    rows = []
    for key, entry in sorted(streams.items(), key=lambda item: -item[1]["count"]):
        endpoint, sender, receiver, cmd_type, cmd_set, cmd_id = key
        rows.append(
            {
                "endpoint": f"0x{endpoint:02x}",
                "sender": sender,
                "receiver": receiver,
                "command_type": f"0x{cmd_type:02x}",
                "command_set": f"0x{cmd_set:02x}",
                "command_id": f"0x{cmd_id:02x}",
                "count": entry["count"],
                "hz": round(entry["count"] / duration, 2) if duration else None,
                "payload_lengths": sorted(entry["payload_lengths"]),
                "new_versus_idle": (cmd_set, cmd_id) not in IDLE_BASELINE,
            }
        )

    return {
        "duml_packets": total_packets,
        "undecoded_bytes": undecoded_bytes,
        "streams": rows,
        "subscription_agent_packets": agent_packets,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--vendor-id", type=lambda v: int(v, 0), default=DJI_VENDOR_ID)
    parser.add_argument("--product-id", type=lambda v: int(v, 0), default=None)
    parser.add_argument(
        "--interface",
        type=int,
        action="append",
        dest="interfaces",
        help="vendor interface to listen on; repeatable, defaults to 3 4 5 6 7",
    )
    parser.add_argument("--label", default="", help="free-text context for the run")
    parser.add_argument("-o", "--output", type=Path, required=True, help="JSONL transfer log")
    parser.add_argument("--summary", type=Path, help="JSON census next to the transfer log")
    args = parser.parse_args()

    interfaces = tuple(args.interfaces) if args.interfaces else DEFAULT_INTERFACES

    identity, results = capture(
        vendor_id=args.vendor_id,
        product_id=args.product_id,
        interfaces=interfaces,
        duration=args.duration,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    transfers = sorted(
        (transfer for result in results for transfer in result.transfers),
        key=lambda transfer: transfer.monotonic_ns,
    )
    base_ns = transfers[0].monotonic_ns if transfers else 0
    with args.output.open("w", encoding="utf-8") as stream:
        stream.write(
            json.dumps({"type": "header", "label": args.label, **identity}) + "\n"
        )
        for transfer in transfers:
            stream.write(
                json.dumps(
                    {
                        "type": "transfer",
                        "t_ns": transfer.monotonic_ns - base_ns,
                        "endpoint": f"0x{transfer.endpoint:02x}",
                        "hex": transfer.data.hex(),
                    }
                )
                + "\n"
            )

    report = census(results, args.duration)
    report["label"] = args.label
    report["device"] = identity
    report["endpoints"] = [
        {
            "endpoint": f"0x{result.endpoint:02x}",
            "interface": result.interface,
            "transfers": len(result.transfers),
            "bytes": result.byte_count,
            "error": result.error,
        }
        for result in results
    ]
    if args.summary:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"device      {identity['product']} {identity['serial_number']}")
    print(f"capture     {args.output}")
    for endpoint in report["endpoints"]:
        marker = f"  error={endpoint['error']}" if endpoint["error"] else ""
        print(
            f"  ep {endpoint['endpoint']} if{endpoint['interface']}"
            f"  transfers={endpoint['transfers']:6d}  bytes={endpoint['bytes']:8d}{marker}"
        )
    print(f"DUML valid  {report['duml_packets']}  undecoded bytes {report['undecoded_bytes']}")
    print()
    print(f"{'ep':>5} {'from':>6} {'to':>6} {'type':>5} {'set/id':>10} {'count':>7} {'Hz':>7}  new")
    for row in report["streams"]:
        print(
            f"{row['endpoint']:>5} {row['sender']:>6} {row['receiver']:>6} "
            f"{row['command_type']:>5} {row['command_set']}/{row['command_id']:>4} "
            f"{row['count']:>7} {row['hz']:>7}  {'NEW' if row['new_versus_idle'] else ''}"
        )
    print()
    agent = report["subscription_agent_packets"]
    print(f"0x00/0x99 subscription-agent packets: {len(agent)}")
    for packet in agent[:5]:
        print(f"  {packet['sender']} -> {packet['receiver']} "
              f"type {packet['command_type']} len {packet['payload_length']}")
        print(f"    {packet['payload_hex']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
