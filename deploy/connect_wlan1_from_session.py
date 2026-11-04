#!/usr/bin/env python3
"""Connect the dedicated Pi adapter using credentials from a private session."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess


def _response(session: dict, command_id: int) -> bytes:
    matches = [
        bytes.fromhex(packet["payload_hex"])
        for packet in session["duml"]
        if packet["direction"] == "rx"
        and packet["command_set"] == 0x07
        and packet["command_id"] == command_id
    ]
    if len(matches) != 1:
        raise RuntimeError(f"expected exactly one response for command 0x{command_id:02x}")
    payload = matches[0]
    if not payload or payload[0] != 0:
        raise RuntimeError(f"invalid response payload for command 0x{command_id:02x}")
    return payload[1:]


def _text_response(session: dict, command_id: int) -> bytes:
    body = _response(session, command_id)
    if not body or len(body) != body[0] + 1:
        raise RuntimeError(f"invalid text response for command 0x{command_id:02x}")
    return body[1:]


def _run(*arguments: str) -> None:
    subprocess.run(arguments, check=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    parser.add_argument("--interface", default="wlan1", choices=("wlan1",))
    parser.add_argument("--connection", default="neo2-lab")
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()

    session = json.loads(args.session.read_text(encoding="utf-8-sig"))
    ssid = _text_response(session, 0x07).decode("ascii")
    password = _text_response(session, 0x0E).decode("ascii")
    bssid_bytes = _response(session, 0x0C)
    if len(bssid_bytes) != 6:
        raise RuntimeError("invalid BSSID response length")
    bssid = bssid_bytes.hex(":")
    print(json.dumps({"ssid": ssid, "bssid": bssid, "password_length": len(password)}))
    if not args.apply:
        return 0
    if os.geteuid() != 0:
        raise PermissionError("--apply must run as root")

    existing = subprocess.run(
        ["nmcli", "-g", "NAME", "connection", "show"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    if args.connection not in existing:
        _run(
            "nmcli", "connection", "add", "type", "wifi",
            "ifname", args.interface, "con-name", args.connection, "ssid", ssid,
        )
    _run(
        "nmcli", "connection", "modify", args.connection,
        "connection.interface-name", args.interface,
        "802-11-wireless.ssid", ssid,
        "802-11-wireless.bssid", bssid,
        "802-11-wireless-security.key-mgmt", "wpa-psk",
        "802-11-wireless-security.psk", password,
        "ipv4.method", "auto",
        "ipv4.never-default", "yes",
        "ipv6.method", "disabled",
        "ipv6.never-default", "yes",
    )
    _run("nmcli", "connection", "up", args.connection, "ifname", args.interface)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
