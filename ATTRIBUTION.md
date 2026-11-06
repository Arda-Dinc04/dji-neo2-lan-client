# Attribution

The DUML framing, reliable UDP transport structure, and initial BLE UUID leads
were informed by Leonardo Silva's MIT-licensed `lslv1243/neo-control` research:

https://github.com/lslv1243/neo-control

This project does not assume DJI Neo 1 commands are valid for DJI Neo 2. It uses
those public protocol structures only as hypotheses to test against observations
made on a DJI Neo 2. Selected flight commands are implemented experimentally;
their documented validation is specific to the Neo 2 trials described in
`docs/VALIDATION.md`.

The optional local WebRTC preview uses [MediaMTX](https://github.com/bluenviron/mediamtx),
pinned to v1.21.3 by the workspace installer. MediaMTX is MIT-licensed; its
release license is retained alongside the separately downloaded binary under
ignored `build/mediamtx/`. The binary is not included in this repository.
