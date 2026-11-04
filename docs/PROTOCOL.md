# DJI Neo 2 local protocol: scope

This document describes the implemented interfaces. It is a partial protocol
description, not a specification of every behavior of the aircraft.

## Connection path

BLE uses GATT service `FFF0` and DUML messages to establish the local setup
context and read the aircraft Wi-Fi parameters. Device-specific identities,
Wi-Fi credentials and captured responses are generated or obtained locally
and must remain private. See [BLE setup](../src/neo2_control/ble_bootstrap.py),
[identity handling](../src/neo2_control/identity.py) and
[Wi-Fi association](../src/neo2_control/wifi_link.py).

Observed sessions use the aircraft's Wi-Fi access point and SWUDP at
`192.168.2.1:9003`. This is the local network destination observed during
development, not an Internet service. Addressing and radio availability must
be checked on the actual device.

## Framing and transport

[DUML](../src/neo2_control/duml.py) implements packet framing, declared length,
header and packet checksums, sender/receiver, sequence and command fields.
[The stream reader](../src/neo2_control/duml_stream.py) preserves complete
validated packets across input boundaries.

[SWUDP transport](../src/neo2_control/transport.py),
[receiver](../src/neo2_control/receiver.py) and
[FEC](../src/neo2_control/fec.py) provide the observed session envelopes,
reliable channels, ordering and acknowledgements. The
[session owner](../src/neo2_control/session.py) coordinates the connection.
Unknown fields remain opaque; a fixture match alone does not establish a
hardware effect.

## Video and observations

Video is received and reassembled locally. The
[HEVC reader](../src/neo2_control/hevc.py) and
[incremental decoder](../src/neo2_control/video_stream.py) retain original
stream metadata and frame lineage. The managed decoder bounds queues and
observation age. Network arrival time is not camera exposure time.

The [telemetry decoder](../src/neo2_control/telemetry.py),
[battery decoder](../src/neo2_control/battery.py) and
[Wi-Fi status decoder](../src/neo2_control/wifi_status.py) interpret selected
observations. The [proximity reader](../src/neo2_control/obstacles.py) decodes
`24/08` bodies; physical units and the `60000` sentinel remain unresolved.

## Experimental control

The [flight codecs](../src/neo2_control/flight.py),
[channel](../src/neo2_control/flight_channel.py) and
[supervisor](../src/neo2_control/controller.py) implement selected actions and
virtual stick commands with explicit ownership, expiry and observation checks.
An ACK indicates a protocol response; it does not establish completion of the
physical action. See [validation](VALIDATION.md) for the measured scope.

The independent [3D engine](https://github.com/smichea/spatial-capture-3d)
consumes images and camera metadata. Its legacy serialization identifiers may
contain `neo2-control`; its implementation does not require the DJI transport.
