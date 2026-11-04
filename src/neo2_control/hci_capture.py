"""Raw Bluetooth HCI capture using BlueZ btmon (Wireshark-readable btsnoop)."""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
import time


def capture_hci(output: str | Path, duration: float = 20.0) -> Path:
    if duration <= 0 or duration > 600:
        raise ValueError("HCI capture duration must be between 0 and 600 seconds")
    executable = shutil.which("btmon")
    if not executable:
        raise RuntimeError("btmon was not found; install the BlueZ tools")
    target = Path(output).resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(
        [executable, "-w", str(target)],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        process.wait(timeout=duration)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
    if process.returncode not in (0, -15):
        error = (process.stderr.read() if process.stderr else "").strip()
        raise RuntimeError(f"btmon failed with status {process.returncode}: {error}")
    if not target.is_file():
        raise RuntimeError("btmon did not create the capture file")
    return target
