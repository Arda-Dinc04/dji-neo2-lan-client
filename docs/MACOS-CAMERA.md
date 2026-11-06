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

A later local power-button confirmation succeeded and the BLE credential reads
completed. With explicit authorization to join the aircraft Wi-Fi, the bounded
Mac coordinator verified the exact network, passed the IPv4 readiness gate,
then restored the original Wi-Fi successfully. The actual UDP session ran for
57.73 seconds: 1,211 datagrams transmitted, 2,306 received, zero rejected, and
1,136,612 protocol bytes delivered. Capability reading and the live-view service
switch completed, but **zero video bytes and zero decoded frames** arrived.
All three camera cleanup checks passed. This verifies transport and service
setup; it does not verify camera imagery or long-running preview reliability.
The next investigation uses that private capture to locate the missing video
session condition before any further physical attempt.

Offline replay of that capture accounts for all aggregate bytes as 13,806
CRC-valid DUML frames, with no discarded or pending bytes. An independent
bounded codec probe found no HEVC stream. This rules out the apparent Annex-B
prefixes in protocol fields as evidence of hidden camera video.

The pinned `iframe-request-native.json` includes a synthetic SSFN-zero vector
for the direct `18/47` encoder-refresh producer. Its documented boundaries do
not establish firmware acceptance or camera readiness. The existing first-byte
prerequisite is explicitly a PC policy; removing it for startup is an experiment,
not an established missing activation command.

The separate `--startup-keyframe-refresh` option enables that experiment only in
preview mode. It requires committed live-view service, matching peer generation
and routing hashes, a fresh post-commit beacon, a live backend and the current
beacon's SSFN. Queued requests bind to the switch instance, generation,
completion time and SSFN; changes cancel the send. The existing maximum of
three requests, spaced at least one second apart, remains. All other command
restrictions remain active; no special-control fallback is sent. The default
keeps the original first-video-byte prerequisite.

The first authorized attempt with this option stopped at `verification_denied`
when the aircraft's confirmation window expired. No Wi-Fi join or startup
refresh occurred in that attempt. A synchronized physical confirmation is still
needed to evaluate the hypothesis on this aircraft.

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

For the source-backed startup experiment, additionally pass
`--startup-keyframe-refresh` to that server command. Launching the server alone
does not connect or send an encoder request.

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
shows the network details on the authenticated local page and waits up to ten
minutes while retaining BLE. The password is masked until the operator chooses
to reveal it. Details are never sent through chat or stored in browser local
storage; the credential endpoint requires the local page token and disables
caching. No Wi-Fi profile or association is
changed automatically. Do not post the credential file or raw session captures.

The Mac verifier requires exact SSID and BSSID plus a local IPv4 lease whose
subnet contains the source-defined aircraft address and whose gateway is that
address (`192.168.2.1`). It reads local DHCP state without probing the network.
This prevents starting UDP with a stale lease from the previous network.
It then monitors identity and DHCP readiness. The UDP
owner checks the cached identity before sends, with a 1.5-second freshness bound;
identity mismatch, monitoring failure or stale checks stop the session. This is
sampled host evidence, not an atomic guarantee against an instantaneous switch.

With Wi-Fi as the Mac's only internet path, joining the aircraft interrupts the
online chat. The local page and processes keep running. Let the bounded capture
finish before returning to the normal network; its result/media remain local
for inspection afterward. A complete SYN or protocol ACK is not a live frame.

With separate operator authorization to change the Mac's network, the bounded
coordinator can join using credentials from the current authenticated session:

```sh
.venv/bin/python tools/macos_capture_once.py --execute
```

Without `--execute` it prints a plan and performs no connection or network change.
It passes the aircraft password to `networksetup` through stdin, checks exact
SSID/BSSID, captures for at most 60 seconds, disconnects the camera session and
attempts to restore the original SSID using macOS saved settings. It verifies
restoration and reports failure explicitly; saved-network restoration is not
guaranteed. It reads no unrelated keychain passwords. If restoration fails,
choose the usual network from macOS Wi-Fi settings. Results stay under
`private/macos/auto-capture/`; no credentials are printed in those results.

## Decoder resource policy

Linux retains upstream prlimit controls. The Mac uses a separate Python exec
wrapper for hard open-file and per-file size limits and a parent RSS watchdog
for each owned decoder process. The snapshot discloses a sampled 1.5-GiB RSS
limit, checks every 250 ms and possible overshoot; it claims no Mac address-space
hard cap. Queue, stderr, storage and process cleanup bounds remain enforced.

Read-only frame integration with the main viewer comes after actual Mac preview
validation. Keep independent freshness and host timestamps; exposure-time pose
synchronization and camera/body alignment remain unverified.
