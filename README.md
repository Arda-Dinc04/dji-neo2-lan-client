# DJI Neo 2 LAN client — Mac camera research

Mac camera-preview research fork of
[smichea/dji-neo2-lan-client](https://github.com/smichea/dji-neo2-lan-client/tree/014e3ad0d3ee98fecba94dfa48fe512a217a155f),
part of [Neo 2 open research](https://github.com/Arda-Dinc04/neo2-open-research).
The Mac entry point starts disconnected with active setup disabled by default.
Opt-in preview setup performs BLE registration, credential reads and Wi-Fi
live-view service selection. It blocks flight, firmware, gimbal, photo, onboard
recording and camera work-mode commands. This is camera interoperability work
around stock firmware, not a receive-only session or a validated Mac flight SDK.

Actual aircraft preview has been demonstrated with the aircraft's USB host
connection disconnected and DJI Fly closed: Mac-only startup, an earlier
operator-ended 7:59 preview, and a separate 1280 × 720 WebRTC trial at about
30 fps. The English viewer uses a loopback-only MediaMTX bridge and JPEG
fallback, with bounded numeric receiver diagnostics. Total camera-to-screen
delay, actual radio reconnect recovery and an O4-to-Mac ground bridge remain
unverified. Raw captures, credentials and user media are not published.

Start with [Mac setup and observed results](docs/MACOS-CAMERA.md),
[WebRTC installation and playback](docs/MEDIAMTX.md), and the
[read-only integration boundary](https://github.com/Arda-Dinc04/neo2-open-research/blob/main/docs/integration.md).
Upstream history, licenses and the original Linux implementation are retained.
The upstream control trials described below are separate from this Mac preview
work and are not evidence that Mac flight control has been validated.

## Original Linux client

Experimental MIT-licensed Python client for communicating with a **DJI Neo 2**
from Linux over its local BLE and Wi-Fi interfaces, using the stock firmware.
The Python package and CLI retain their names: `neo2_control` and `neo2ctl`.

The implementation includes BLE setup, SWUDP/DUML session transport, video
reception, selected telemetry, a local operator interface, simulation and
bounded flight programs. Hardware trials documented during development
include a short takeoff/landing cycle and complete yaw rotations. Full flight
control, link-loss behavior and physical calibration remain incomplete.

The independent reconstruction engine is published separately as
[Spatial Capture 3D](https://github.com/smichea/spatial-capture-3d).
It is optional for radio communication, telemetry and the operator interface.

## Install

Python 3.11 or newer is required for the core package. Use Python 3.13 for the
full optional development environment tested for this release.

```sh
git clone https://github.com/smichea/dji-neo2-lan-client.git
cd dji-neo2-lan-client
python3 -m venv .venv
.venv/bin/pip install -e '.[flight,test]'
.venv/bin/neo2ctl --help
```

Linux camera/flight setup also uses BlueZ, NetworkManager and FFmpeg. Use a
dedicated Wi-Fi interface when the PC must remain connected to another network.
The optional `vision`, `depth`, `sfm` and `video-analysis`
extras install the corresponding offline analysis dependencies.

## Start with observation and simulation

```sh
.venv/bin/neo2ctl ble-scan --duration 10 --json
.venv/bin/neo2ctl simulate --help
.venv/bin/neo2ctl camera-web --help
.venv/bin/neo2ctl flight-web --help
```

Radio examples and test addresses use synthetic values. Supply your own
observed device address with `--ble-address` when opening a camera or flight
session. The example `02:00:00:00:00:01` is not a discovered device.
The operator page opens disconnected; preparation and flight actions require
explicit operator confirmations. The implemented software guards do not
establish general hardware failsafe behavior.

## Documentation

- [Protocol scope and implementation](docs/PROTOCOL.md)
- [Documented results and limitations](docs/VALIDATION.md)
- [Publication checks](docs/RELEASE-VALIDATION.md)
- [Public snapshot provenance](docs/PUBLIC-SNAPSHOT.md)
- [Candidate OpenDeviceDB record](docs/opendevicedb/dji-neo-2.yaml)
- [Attribution](ATTRIBUTION.md)

This is an interoperability research client. It does not replace the stock
firmware. Initial vendor activation/account requirements on a new device
have not been established.

## Tests

For the broader offline suite, install the optional dependencies:

```sh
.venv/bin/pip install -e '.[flight,vision,sfm,video-analysis,test]'
.venv/bin/python -m pytest -q
```

Tests use fixtures, temporary directories, simulations and loopback peers.
Optional browser, GPU and native-tool tests depend on locally available tools.
Vendor APKs, shared libraries, SDK-log decryption parameters, instruction
excerpts and private device identities are not distributed. The private binary
analysis toolkit is omitted. Saved protocol vectors remain available for
offline comparison; they do not establish public hardware replay evidence.
