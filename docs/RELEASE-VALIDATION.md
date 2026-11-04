# Public snapshot validation — 2026-11-04

Publication checks use the exported source and a fresh Python 3.13.16 virtual
environment, with the optional 3D dependency installed over HTTPS from public
commit `e14ec1702e934d473c3327bcaaeeafba061bc127` of `spatial-capture-3d`.

- Focused Python suite: **256 passed** in 15.40 seconds. Coverage includes
  anonymized BLE addresses, btsnoop fixtures, camera/flight connection,
  incident/recovery handling, operator pages, CLI, video publishing and the
  public plaintext log analyzer.
- Home-server integration tests: **5 passed**.
- Complete current suite collection: **3,272 tests**, no collection errors.
  The complete client suite was not run to completion for this release.
- The Python wheel builds and installs successfully in a new environment,
  including its optional public 3D dependency. Both CLIs and packaged HTML
  resources are available from the installed packages outside the checkout.
- Installed-wheel simulation scenarios: normal, control loss, ACK loss,
  telemetry loss and link loss. These are virtual-time checks, not flights.
- The candidate OpenDeviceDB record validates against upstream schema blob
  `868e42201e27f0aee791a79f61eddf45851d9da6`; it is not submitted by this release.
- Snapshot secret-scan findings were reviewed as hashes, packet vectors,
  synthetic identifiers and deliberately invalid test credentials. No live
  credential was identified in that review; future changes need their own review.

## Earlier checks and correction

An initial sequential exploratory run was interrupted after 404 passes.
A subsequent exploratory whole-suite run was interrupted after 1,864 passes
and one collection error: the log analyzer still imported the omitted SDK
log decoder. The public analyzer now reads plaintext only and rejects
encrypted SDK logs before writing reports. Its five focused tests pass,
including that rejection. Final collection and the focused suite above pass.
These interrupted runs do not establish full-suite success.

The independent 3D snapshot has its own complete Python result:
**334 passed, 4 skipped**, plus **5 JavaScript tests passed**. Optional native
GPU runtimes and physical calibration are not qualified by publication checks.
No device connection or aircraft operation was performed for this release.
