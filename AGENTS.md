# Mac camera worktree

This branch ports the upstream camera preview to macOS. Preserve upstream
attribution and keep original Linux behavior where compatible.

- Default `tools/macos_camera.py` serves a disconnected preview with connection
  disabled. Active BLE/WLM setup requires `--allow-camera-session-setup`.
- Preview setup is active registration/service selection, not read-only device
  communication. Do not call it receive-only or claim routing choices cannot persist.
- Always use `preview_only=True` on both BLE bootstrap and UDP session. Inspect
  inner DUML in forwarded traffic; outer `51/01` is not a sufficient allowlist.
- No flight, firmware, gimbal, photo, onboard recording or camera work-mode
  commands. Do not enable the general flight/operator CLI as a camera shortcut.
- The Mac Wi-Fi backend verifies manual association. It must not change the
  default route or join networks automatically by default. The separate
  `tools/macos_capture_once.py --execute` coordinator requires explicit operator
  authorization to join the aircraft and restore the saved original network.
  Never obtain unrelated keychain secrets, bypass OS privacy permissions or
  guess network identity from IP alone.
- Private credentials, Bluetooth identifiers, media and raw captures stay under
  `private/`, never Git. The helper app and Python environment stay in ignored
  build/.venv paths. It requests Wi-Fi identity authorization without location fixes.
- Preserve per-process memory/queue/file limits and accurate platform disclosures.
  Test local fixtures as synthetic evidence, separately from actual aircraft frames.
- The existing orientation viewer owns USB interface 4. Camera code must not
  claim it. Camera/USB receipt clocks are not verified exposure-time alignment.
- Run tests in the isolated `.venv`; camera/BLE/network tests use fake devices or
  local peers. Physical/radio sessions are a separate explicit execution step.
