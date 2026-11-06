# Mac camera preview port

This local branch starts from upstream `014e3ad0d3ee98fecba94dfa48fe512a217a155f`.
It does not replace firmware or expose flight commands. The original upstream
checkout and USB orientation viewer are separate worktrees/processes.

## What has been verified

- CoreBluetooth advertisement discovery found a uniquely named Neo 2 candidate.
- A metadata-only GATT connection verified service FFF0, notifications on FFF4
  and write-without-response support on FFF5. No DJI application write was sent.
- CoreBluetooth UUID identifiers now pass configuration, BLE setup and device
  ownership locks; Linux MAC identifiers remain supported.
- IPv4 interface binding uses Darwin IP_BOUND_IF, sourced from the installed
  Apple SDK header, while retaining Linux SO_BINDTODEVICE behavior.
- The local FFmpeg HEVC/HLS/JPEG fixture pipeline runs on the Mac. Fixture
  pictures are synthetic evidence, not camera frames.
- A local signed helper makes current Wi-Fi identity available after normal OS
  Location authorization. It requests no location fixes and changes no network.
- The disconnected preview page has connection disabled by default and hides
  gimbal/photo/onboard-recording controls. The backend rejects those actions.

**No actual aircraft image has been received yet.** The first live frame,
ten-minute reliability, reconnect behavior and production recording remain
separate validation milestones.

An explicitly approved camera setup attempt subsequently exchanged BLE
identification `07/45`. The aircraft requested verification, then reported
permission result 0 about 13 seconds later. The client acknowledged the
permission notification and closed cleanly as `verification_denied` after
14.44 seconds. Exactly one setup request and two GATT write chunks were sent;
no credentials were saved and no Wi-Fi/WLM session began. Physical first-link
confirmation must be coordinated before the next attempt. This is a real BLE
observation, not a live-video result.

The combined focused suite passed **280 tests** on the Apple Silicon Mac,
covering BLE/configuration portability, exclusive ownership, strict preview
wire policy, manual network identity, local HTTP, session transport and generated
HEVC/HLS/JPEG fixtures. A browser inspection found no console errors and verified
hidden action controls. A direct HTTP connect request in default mode returned
409 before BLE setup. The independent USB viewer remained streaming with zero
DJI application-command bytes sent. No full optional vision/SfM suite is claimed.

## Prepare

Use an isolated Python environment and existing FFmpeg/Apple command-line tools:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-macos.txt
.venv/bin/python tools/build_macos_wifi.py
.venv/bin/python tools/macos_discover.py
.venv/bin/python tools/macos_camera.py --preflight
```

The preflight performs no DJI application writes. The Wi-Fi helper can request
macOS Location permission because SSID/BSSID access is privacy-controlled.
Its only output is current network identity, consumed privately by the client.
No GPS/location sample is requested. Denied or unavailable identity stops setup;
the client does not infer aircraft ownership from a common private IP address.

## Review the disconnected preview

```sh
.venv/bin/python tools/macos_camera.py --port 8793
```

Open http://127.0.0.1:8793. This mode disables connection at the backend and
browser. Ctrl-C closes only this camera server; it does not stop the USB viewer.

## Active camera connection — explicit separate step

Camera reception requires application writes. The source-backed preview policy
permits BLE caller registration, credential reads/permission ACKs, owned WLM
registration, presence/capability reads, live-view route/service switching and
bounded encoder keyframe recovery. WLM traffic is checked through its inner
DUML frame, forwarding attributes, owned identity and selected peer. Flight,
firmware, gimbal, photography, onboard recording and work-mode writes are rejected.

Service selection includes a hybrid command route and Wi-Fi live-view/common-data
routes. That is not an FC authority grant, but it changes session routing; the
reviewed source does not establish persistence across reboot. This active phase
therefore requires approval when the prior scope is strictly read-only.

After that approval, stop the disconnected camera server and run:

```sh
.venv/bin/python tools/macos_camera.py --allow-camera-session-setup --duration 60 --port 8793
```

Open the page and connect. When BLE credentials are received, the process writes
For first-link verification, click **Connect camera** with a finger ready at
the aircraft's power button. The local page shows **CONFIRM NOW** as soon as
the aircraft requests approval. Confirm on the aircraft immediately; a chat
round trip can exceed its observed short permission window. This indication is
bound to the current connection generation and clears on a permission result
or BLE cleanup. No automatic request retry or alternate identity is generated.

When BLE credentials are received, the process writes
a mode-0600 `join-network.json` inside the new private camera-session directory
and prints its **path**, without printing the password. Open that local file
to obtain the network details, then manually join the aircraft Wi-Fi. The client
waits up to 120 seconds while retaining BLE. No Wi-Fi profile or association is
changed automatically. Do not post the credential file or raw session captures.

The Mac verifier requires exact SSID and BSSID, then monitors identity. The UDP
owner checks the cached identity before sends, with a 1.5-second freshness bound;
identity mismatch, monitoring failure or stale checks stop the session. This is
sampled host evidence, not an atomic guarantee against an instantaneous switch.

With Wi-Fi as the Mac's only internet path, joining the aircraft interrupts the
online chat. The local page and processes keep running. Let the bounded capture
finish before returning to the normal network; its result/media remain local
for inspection afterward. A complete SYN or protocol ACK is not a live frame.

## Decoder resource policy

Linux retains upstream prlimit controls. The Mac uses a separate Python exec
wrapper for hard open-file and per-file size limits and a parent RSS watchdog
for each owned decoder process. The snapshot discloses a sampled 1.5-GiB RSS
limit, checks every 250 ms and possible overshoot; it claims no Mac address-space
hard cap. Queue, stderr, storage and process cleanup bounds remain enforced.

Read-only frame integration with the main viewer comes after actual Mac preview
validation. Keep independent freshness and host timestamps; exposure-time pose
synchronization and camera/body alignment remain unverified.
