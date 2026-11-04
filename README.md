# DJI Neo 2 LAN client

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
