# Local WebRTC camera preview

The browser path is H.264 video from a low-latency FFmpeg tee, published over
RTSP/TCP on loopback to MediaMTX. The local camera server on port 8793 proxies
WHEP signaling under its own origin; WebRTC carries the browser video. The tee
also preserves the existing local preview output. This is a Mac preview path;
it sends no aircraft flight, firmware, gimbal, photo, recording or work-mode
commands.

The bridge binds its RTSP, WHEP and ICE listeners to `127.0.0.1`, advertises
only the loopback ICE candidate, and uses no external STUN/TURN servers. It
disables RTMP, HLS, SRT, **MoQ**, playback, API, metrics and profiling listeners.
MoQ is enabled in this release's upstream default configuration, so the unmodified
sample config is unsuitable. RTSP transports are TCP only. The bridge owns
the generated configuration and child process; the installer starts neither.
Its RTSP/TCP, WHEP/HTTP and ICE/UDP ports are assigned locally at startup. Only
the same-origin camera endpoint on port 8793 is exposed to the browser; private
SDP and internal session addresses do not belong in logs or public status.
See the [official pinned configuration](https://github.com/bluenviron/mediamtx/blob/v1.21.3/mediamtx.yml).

## Install the pinned local dependency

Run from the Mac camera worktree:

```sh
.venv/bin/python tools/install_mediamtx.py
```

The default prints a plan. It performs no network access, creates no directory
and launches no process. To download and install the reviewed release:

```sh
.venv/bin/python tools/install_mediamtx.py --install
```

This installs only the binary and license at `build/mediamtx/mediamtx` and
`build/mediamtx/LICENSE`. The existing `build/` Git ignore covers both files.
It requires Apple Silicon macOS, uses no Homebrew or system install, and never
runs the downloaded binary. An existing destination is refused rather than
overwritten. Inspect any existing installation before removing it manually.
The pinned v1.21.3 dependency was installed and checksum-verified during local
validation on 6 November 2026; the install command need not be repeated there.

The installer pins [official release v1.21.3](https://github.com/bluenviron/mediamtx/releases/tag/v1.21.3):

| Item | SHA256 |
| --- | --- |
| `mediamtx_v1.21.3_darwin_arm64.tar.gz` | `bc1a0762fd18ffb413bcd81dcb9a072ac159fbd1f27e79ac6f14598aaac5ed3f` |
| `checksums.sha256` | `7df899490a0941533d55eab3c31048d365f55c9ed519a962394118d609183431` |

It downloads the [official checksum list](https://github.com/bluenviron/mediamtx/releases/download/v1.21.3/checksums.sha256)
and archive over HTTPS, verifies the checksum file's pinned digest, requires its
exact archive entry, then verifies the archive digest before reading members.
Paths, duplicates, links, special files and expansion limits are checked;
only regular `mediamtx` and `LICENSE` files are copied into a temporary staging
directory and moved into place. A failed install does not leave a partial
destination. Updating the version requires reviewing and replacing the pinned
URLs and both digests together.

## Run the aircraft preview

Keep the USB host end unplugged and release the phone's DJI Fly connection
before the approved Mac camera session. Wi-Fi association remains a separate
operator-authorized action. Installing MediaMTX does not associate a network or
start BLE/WLM setup.

```sh
.venv/bin/python tools/macos_camera.py --allow-camera-session-setup --startup-keyframe-refresh --webrtc --duration 660 --port 8793
```

Open `http://127.0.0.1:8793`. Use **Auto (WebRTC)** for browser video with
automatic JPEG fallback, or select **JPEG** directly. The existing physical
power-button confirmation and Wi-Fi switching steps are unchanged; see
[Mac camera setup](MACOS-CAMERA.md). Keep the USB host end unplugged and DJI Fly
closed on the phone. Starting the server alone does not connect the aircraft.

**Live aircraft WebRTC verified on 6 November 2026.** The Mac preview was
observed for 100.563 seconds and encoded 2,999 frames. The browser displayed
1280 x 720 video at about 30 fps: 31 live samples span 95.24294 seconds, with
2,892 frames and 16 dropped frames reported at the last live sample. Page
labels and controls are fully English.

The requested window was 120 seconds. The operator confirmed stopping the
session or changing Wi-Fi before it finished; the diagnostic
`camera_session_failed` / `network_error` labels do not establish a spontaneous
dropout. The target Wi-Fi identity was verified, the original SSID was restored,
and camera cleanup passed. The independent synthetic bridge-shutdown test below
verified JPEG fallback; this live run does not independently certify fallback
or reconnection.

This establishes actual aircraft video in the WebRTC browser path. It does not
measure glass-to-glass latency or certify a long soak or reconnect behavior.
Network receipt time is not camera exposure time, and no synchronized
orientation overlay is established. Mac-only startup without Fly was separately
demonstrated with the JPEG path.

## Reproduce the synthetic browser test

```sh
.venv/bin/python tools/webrtc_fixture.py --duration 120 --kill-bridge-after 80 --port 8794
```

Open `http://127.0.0.1:8794`. This generates **synthetic test images**, with no
aircraft, BLE connection or Wi-Fi association. The bridge is deliberately
stopped after 80 seconds so Auto mode can demonstrate fresh JPEG fallback.
The fixture stops after 120 seconds.

The actual local browser test observed 90 synthetic WebRTC frames over
3.0019 seconds with zero reported dropped frames. Forced bridge shutdown
produced fresh JPEG fallback. The 120-second fixture encoded 3,600 frames and
verified cleanup. These results validate the local browser/bridge path and its
fallback; they are separate from aircraft reception and do not measure camera
exposure-to-display latency.

## Local signaling boundary

The same-origin WHEP proxy checks `Host`, `Origin` and `X-Neo2-Token`. It accepts
SDP offers of at most 64 KiB and returns HTTP 201 with an opaque local
`Location`. Session deletion accepts only the fixed local route with a
48-character hexadecimal identifier; it does not proxy arbitrary URLs. The
internal MediaMTX WHEP URLs are replaced by the local opaque route. Private SDP
remains out of logs and public status.

## Installer checks

The installer is covered by offline archive/download fixtures:

```sh
.venv/bin/python -m pytest -q tests/test_install_mediamtx.py
```

Those tests perform no actual download, MediaMTX startup, radio session or
aircraft operation.
