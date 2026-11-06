# Mac camera preview port

This local branch starts from upstream `014e3ad0d3ee98fecba94dfa48fe512a217a155f`.
It does not replace firmware or expose flight commands. The original upstream
checkout and USB orientation viewer are separate worktrees/processes.

## What has been verified

**Latest result: startup without DJI Fly and a 7-minute-59-second Mac-only
follow-up verified.** The operator power-cycled the aircraft, closed Fly and
kept the USB host disconnected. The first Mac-only attempt decoded 407 frames
in 27.229 seconds; a subsequent Mac-only session decoded 13,855 frames and
received 359,206,127 video bytes in 478.925 seconds. The operator intentionally
stopped the later run and relaxed the exact ten-minute test target. No
spontaneous 47-second dropout has been established.

After the first ten seconds of the longer run, the largest video-delivery gap
was 257 ms, with none above 500 ms. Decoder and normalizer exited successfully;
all cleanup checks passed. All 1,915 transmitted application messages passed
the preview policy. This is host reception/decoding evidence, not camera
exposure latency or a browser-rendered frame counter. Media remains private.

The earlier automated attempt exposed a real coordinator bug: discovery
beacons expired while video remained fresh, and the status layer reported
`camera_link_interrupted`; the coordinator then stopped the preview. A committed
preview with fresh video now reports this particular discovery expiry as a
warning. The WLM send gates stay closed; other errors and stale video remain
fatal. The exact regression, non-preview behavior and stale-video behavior
are covered by tests. The fixed server has been loaded; no further radio trial
of the fix is claimed. The selected preview/transport/decoder regression suite
passed **266 tests**.

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

**Actual Mac camera preview verified on 6 November 2026.** With the external
USB cable disconnected at the Mac end and the transceiver still installed,
the approved live session delivered 22,764,198 video bytes and encoded 1,329
preview frames. The retained actual JPEG is 960 × 540; the operator's screenshot
shows the local page displaying a live aircraft image. Raw media, identifiers,
packet receipt timestamps and the retained preview stay in the private session.

The operator stopped the stream after 47.672 seconds. The app recorded
`network_error`, so its termination label does not reflect the operator-reported
stop; this is not evidence of a spontaneous 47-second dropout. There were 376
rejected datagrams and 144 receive-window rejections labeled stale
sequence/window overrun; the decoder reported no error. BLE, Wi-Fi observer and
video cleanup all passed, and the Mac was verified off the aircraft SSID
afterward. This establishes the first-frame milestone, not ten-minute
reliability, reconnect behavior or production recording.

The working bench condition is **no USB host connection**. The cable can remain
in the transceiver with its far end unplugged. The operator also confirmed phone
preview in this condition. Earlier USB-connected trials produced no video even
after three startup refreshes. The precise device-side suppression mechanism and
concurrent USB orientation plus Wi-Fi video remain unverified. The old battery
was replaced before the phone comparison. A reported warning code `180059` is
retained as an operator observation; its exact meaning was not verified from
an official Neo 2 code reference.

Video was delivered at 0.0262 seconds, before service commit at 1.9214 seconds
and the single encoder-refresh send at 2.4260 seconds. Thus refresh is not
established as the stream-activation step. Because DJI Fly preview preceded
this first attempt, an existing encoder/session state from the phone was not
excluded. The later Mac-only startup result above supplies the separate cold
test. All 194 transmitted
application messages passed the strict preview validator; the audit found no
flight, firmware, gimbal, photo, recording or work-mode command.

The operator attributed the slow connection to DJI Fly remaining connected on
the phone. For repeat tests, release the phone's aircraft connection, keep the
Mac USB end unplugged, and use the existing local power-button confirmation
flow. Keep independent internet available if continuous chat is needed; on a
Wi-Fi-only Mac the local preview continues while the chat loses internet.

## Earlier no-video trials and startup experiment

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
That capture prompted the offline startup investigation below.

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
when the aircraft's confirmation window expired. A subsequent physically
approved session completed a real 60.05-second transport test. The service
switch completed; three SSFN-zero refresh requests were sent at 1.910, 2.933
and 3.940 seconds. The session received 2,397 datagrams and 1,186,494 protocol
bytes, but **zero video bytes and zero decoded frames**. All cleanup checks
passed. This demonstrates that removing the first-byte prerequisite and sending
the bounded encoder refresh is insufficient to activate this aircraft's stream.
It does not establish a command rejection, since this producer expects no ACK.
The Mac was subsequently verified off the aircraft SSID. A later phone comparison
and USB-host-disconnected Mac trial produced the first-frame result above.

The original combined focused suite passed **280 tests** on the Apple Silicon Mac,
covering BLE/configuration portability, exclusive ownership, strict preview
wire policy, manual network identity, local HTTP, session transport and generated
HEVC/HLS/JPEG fixtures. A browser inspection found no console errors and verified
hidden action controls. A direct HTTP connect request in default mode returned
409 before BLE setup. The independent USB viewer remained streaming with zero
DJI application-command bytes sent. It was stopped for the cable comparison.
The startup/preview/network/HTTP regression selection subsequently passed 203
tests; approved-session continuation passed 29 focused orchestration tests.
These are offline/loopback checks, separate from the actual camera trial above.
No full optional vision/SfM suite is claimed.

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
If the operator has already clicked Connect and completed physical confirmation,
`--execute --attach-current` can continue that exact approved generation without
another BLE request. It requires current credentials, no pending verification
and an active preview-only session; mismatched or replaced sessions are rejected.
It passes the aircraft password to `networksetup` through stdin, checks exact
SSID/BSSID, observes 60 seconds of preview by default (up to 600 with explicit
`--duration`), disconnects the camera session and
attempts to restore the original SSID using macOS saved settings. It verifies
restoration and reports failure explicitly; saved-network restoration is not
guaranteed. It reads no unrelated keychain passwords. If restoration fails,
choose the usual network from macOS Wi-Fi settings. Results stay under
`private/macos/auto-capture/`; no credentials are printed in those results.

For a ten-minute observation, use a server session of 660 seconds to allow
startup, then `tools/macos_capture_once.py --execute --attach-current --duration 600`.
The observation timer starts at the first live preview. No first frame within
60 seconds ends the attempt. Early stop, frozen frame counts and observation
gaps do not pass as continuous preview. Results include requested/observed
duration, first-preview delay, sampled live counts and longest observed stalls.
This is sampled host evidence, not exposure-time or browser-render latency.

## Internet and browser transport

The Mac's camera Wi-Fi interface must associate with the aircraft AP. The local
page does not remove that requirement. Prefer Ethernet for internet or an iPhone
USB Personal Hotspot while Wi-Fi serves the camera; the aircraft's USB remains
disconnected. Verify the independent internet route before the aircraft join.
See Apple's [Ethernet guide](https://support.apple.com/en-ie/101589) and
[USB hotspot guide](https://support.apple.com/en-by/guide/iphone/iph45447ca6/27/ios/27).

The existing JPEG preview producer runs at five frames per second and the page
polls every 250 ms. A browser-native WebRTC path, for example through a local
[MediaMTX bridge](https://mediamtx.org/docs/features/publish-and-read), is a future
option for smoother live video. It changes the host-to-browser transport, not
the aircraft Wi-Fi requirement. No WebRTC implementation or performance result
is claimed by the current JPEG preview tests.

## Decoder resource policy

Linux retains upstream prlimit controls. The Mac uses a separate Python exec
wrapper for hard open-file and per-file size limits and a parent RSS watchdog
for each owned decoder process. The snapshot discloses a sampled 1.5-GiB RSS
limit, checks every 250 ms and possible overshoot; it claims no Mac address-space
hard cap. Queue, stderr, storage and process cleanup bounds remain enforced.

Read-only frame integration with the main viewer comes after actual Mac preview
validation. Keep independent freshness and host timestamps; exposure-time pose
synchronization and camera/body alignment remain unverified.
