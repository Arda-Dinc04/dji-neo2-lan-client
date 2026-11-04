# Results and limits

The following results summarize development reports from October 2026.
Original captures and photographs remain private. These summaries are not
publicly replayable hardware evidence and do not confer an OpenDeviceDB
hardware verification badge.

| Trial | Documented result | Scope |
| --- | --- | --- |
| Vertical cycle, 18 October | One TAKEOFF and one LAND, ACK delays of 57.795 ms and 45.838 ms; maximum relative FC altitude about 0.6 m; landed and stopped propellers confirmed by the operator | Short vertical cycle; full control and link-loss protections not validated |
| Yaw, 18 October | Final successful attempt measured 360.8° followed by LAND and normal descent confirmation | FC telemetry angle, without independent optical angle calibration; earlier attempts failed |
| Two-height acquisition, 19 October | Two turns measured 360.4° and 360.8° around FC height levels of 1 m and 1.50 m; landed and stopped propellers confirmed | Experimental vertical/yaw profile; no general horizontal flight validation |
| Proximity analysis | Recorded `24/08` messages contain 360 horizontal values and two up/down values | Physical units and sentinel meanings remain unresolved |

A flight on 17 October ended through the aircraft's low-battery protection
after an acknowledged LAND did not produce the expected landing. Later
corrections and successful trials do not establish general failsafe behavior.

Firmware `01.00.0500` was recorded in the development status report. Its
association with each retained trial must be established before claiming a
version-specific public hardware replay. The original client was tested with
DJI Fly 1.19.0 as an analysis reference; vendor software is not included.

Video decoding, image lineage and reconstruction experiments also include
offline replays and synthetic checks. They must not be treated as new flights,
physical calibration or continuous real-time mapping. See the independent
[3D engine documentation](https://github.com/smichea/spatial-capture-3d).

The proposed OpenDeviceDB status is `partial-protocol`. Complete flight
control, independent physical calibration, full room coverage and dependable
continuous mapping remain unqualified.

## Public snapshot checks

Release validation is recorded in `RELEASE-VALIDATION.md` alongside this file.
It distinguishes installation and offline tests from the historical hardware
results above. No aircraft is operated during publication.
