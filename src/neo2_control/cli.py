"""Command-line entry point for the Neo 2 protocol laboratory."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import sys
import threading
import time

from . import __version__
from .ble import (
    observe_gatt_notifications_with_intervals,
    probe_gatt_sync,
    probe_gatt_with_intervals,
    read_gatt_values_with_intervals,
    scan_sync,
)
from .btsnoop import decode_neo2_session
from .ble_bootstrap import run_ble_wifi_setup_sync
from .hci_capture import capture_hci
from .identity import load_or_create_identity
from .controller_runtime import SCENARIOS, run_simulation
from .report import generate_report
from .service import serve
from .session import run_session
from .udp_probe import run_probe
from .wifi_link import run_wifi_session
from .observation import ManagedObservation, ObservationConfig
from .video_publish import publication_succeeded


def _capture_name(prefix: str) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path("captures") / f"{prefix}-{stamp}.jsonl"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="neo2ctl", description="DJI Neo 2 protocol laboratory")
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    commands = parser.add_subparsers(dest="command", required=True)

    ble = commands.add_parser("ble-scan", help="read-only BLE advertisement scan")
    ble.add_argument("--duration", type=float, default=10)
    ble.add_argument("--json", action="store_true")

    hci = commands.add_parser("ble-hci-capture", help="raw BlueZ HCI capture for Wireshark")
    hci.add_argument("--duration", type=float, default=20)
    hci.add_argument("--output", "-o", type=Path)

    gatt = commands.add_parser("ble-gatt-probe", help="enumerate GATT metadata without custom DJI value access")
    gatt.add_argument("address")
    gatt.add_argument("--timeout", type=float, default=20)
    gatt.add_argument("--interval-min-units", type=int)
    gatt.add_argument("--interval-max-units", type=int)

    read = commands.add_parser("ble-read", help="explicitly read selected GATT characteristic values")
    read.add_argument("address")
    read.add_argument("--uuid", action="append", required=True)
    read.add_argument("--timeout", type=float, default=20)
    read.add_argument("--interval-min-units", type=int, default=12)
    read.add_argument("--interval-max-units", type=int, default=20)

    notify = commands.add_parser(
        "ble-notify",
        help="observe selected GATT notifications without sending an application payload",
    )
    notify.add_argument("address")
    notify.add_argument("--uuid", action="append", required=True)
    notify.add_argument("--duration", type=float, default=10)
    notify.add_argument("--timeout", type=float, default=30)
    notify.add_argument("--interval-min-units", type=int, default=12)
    notify.add_argument("--interval-max-units", type=int, default=20)

    setup = commands.add_parser(
        "ble-wifi-setup", help="run the four measured BLE Wi-Fi requests and save private credentials",
    )
    setup.add_argument("address")
    setup.add_argument("--identity-file", type=Path, required=True)
    setup.add_argument('--wlm-identity-file', type=Path, help='send the native raw WLM identity report after BLE permission')
    setup.add_argument("--credentials-file", type=Path, required=True)
    setup.add_argument("--capture", type=Path)
    setup.add_argument("--timeout", type=float, default=30)
    setup.add_argument("--request-timeout", type=float, default=3)
    setup.add_argument("--interval-min-units", type=int)
    setup.add_argument("--interval-max-units", type=int)

    snoop = commands.add_parser("btsnoop-duml", help="decode BLE ATT and DUML from a btsnoop capture")
    snoop.add_argument("capture", type=Path)
    snoop.add_argument("--address", required=True)

    udp = commands.add_parser("udp-probe", help="capture UDP with bounded transport-only probes")
    udp.add_argument("--host", default="192.168.2.1")
    udp.add_argument("--port", type=int, default=9003)
    udp.add_argument("--duration", type=float, default=8)
    udp.add_argument("--capture", type=Path)
    udp.add_argument("--identity-file", type=Path, help="persistent PC WLM identity for a Wi-Fi information probe")
    udp.add_argument("--interface", default=None, help="Linux interface dedicated to the drone, e.g. wlan1")
    udp.add_argument("--send-syn", action="store_true", help="send one non-flight session SYN")
    udp.add_argument(
        "--send-heartbeat",
        action="store_true",
        help="after acceptance, send exactly one reconstructed non-flight V1 heartbeat",
    )
    udp.add_argument(
        "--send-wifi-device-info",
        action="store_true",
        help="after acceptance, send one Wi-Fi software-device information request",
    )
    udp.add_argument(
        "--send-wifi-then-heartbeat",
        action="store_true",
        help=(
            "send Wi-Fi information, wait for its transport ACK, then send "
            "one heartbeat in the same reliable session"
        ),
    )
    udp.add_argument(
        "--send-static-capability",
        action="store_true",
        help=(
            "extend the stateful sequence with the product static capability "
            "read 0x00/0xB7 type 4, gated by the heartbeat transport ACK"
        ),
    )
    udp.add_argument(
        "--send-idle-window",
        action="store_true",
        help="after acceptance, send bounded type-6 windows with an empty application queue",
    )
    udp.add_argument(
        "--idle-window-count",
        type=int,
        default=1,
        help="bounded number of empty type-6 windows (maximum 100)",
    )
    udp.add_argument(
        "--idle-window-interval",
        type=float,
        default=0.1,
        help="seconds between empty type-6 windows (minimum 0.05)",
    )
    udp.add_argument(
        "--idle-window-mode",
        choices=("periodic", "immediate", "alternate"),
        default="periodic",
        help="send type 6, type 4, or alternate both empty formats",
    )

    identity = commands.add_parser("identity-init", help="create or verify the PC's private WLM identity")
    identity.add_argument("--file", type=Path, required=True)

    session = commands.add_parser(
        "session", help="maintain the measured SWUDP transport profile and journal its state",
    )
    session.add_argument("--host", default="192.168.2.1")
    session.add_argument("--port", type=int, default=9003)
    session.add_argument("--interface", help="Linux interface dedicated to the drone")
    session.add_argument("--duration", type=float, default=30)
    session.add_argument("--capture", type=Path)
    session.add_argument("--video-output", type=Path, help="save ordered type-2 callback bytes to a new file")
    session.add_argument('--wlm-identity-file', type=Path, help='enable native WLM Wi-Fi report scheduling with a persistent PC identity')
    session.add_argument('--wlm-mode-follow', action='store_true',
                         help='experimental persistent WLM presence, UAV mode synchronization and capability read; requires owned identity')
    session.add_argument('--wlm-wifi-liveview', action='store_true',
                         help='experimental bilateral Wi-Fi video service switch; includes mode follow and requires owned identity')
    session.add_argument('--hls-dir', type=Path, help='decode raw Annex B video into a new private HLS directory')
    session.add_argument('--video-fps', type=float, default=30, help='local nominal timing for elementary video')
    session.add_argument('--video-codec', choices=('h264', 'hevc'), default='h264')
    session.add_argument('--publish-config', type=Path, help='private local HTTPS video publication config; requires HLS')
    session.add_argument(
        "--heartbeat", action="store_true",
        help="send the reconstructed V1 heartbeat every 500 ms, gated by transport ACKs",
    )

    wifi = commands.add_parser(
        "wifi-session", help="plan or run a SWUDP session on a temporary dedicated Wi-Fi link",
    )
    wifi.add_argument("--credentials-file", type=Path, required=True)
    wifi.add_argument("--interface", required=True)
    wifi.add_argument("--capture", type=Path)
    wifi.add_argument("--duration", type=float, default=30)
    wifi.add_argument("--video-output", type=Path)
    wifi.add_argument("--heartbeat", action="store_true")
    wifi.add_argument('--wlm-identity-file', type=Path, help='enable native WLM Wi-Fi report scheduling with a persistent PC identity')
    wifi.add_argument('--hls-dir', type=Path)
    wifi.add_argument('--video-fps', type=float, default=30)
    wifi.add_argument('--video-codec', choices=('h264', 'hevc'), default='h264')
    wifi.add_argument('--publish-config', type=Path)
    wifi.add_argument("--apply", action="store_true", help="activate a new runtime profile and remove it after the session")

    connect = commands.add_parser('connect-session', help='own BLE, temporary Wi-Fi and a bounded receive session until cleanup')
    connect.add_argument('address')
    connect.add_argument('--interface', required=True)
    connect.add_argument('--ble-identity-file', type=Path, required=True)
    connect.add_argument('--wlm-identity-file', type=Path, required=True)
    connect.add_argument('--capture-dir', type=Path, default=Path('private/captures'))
    connect.add_argument('--duration', type=float, default=30)
    connect.add_argument('--ble-setup-timeout', type=float, default=30)
    connect.add_argument('--heartbeat', action='store_true')
    connect.add_argument('--video', action='store_true')
    connect.add_argument('--hls', action='store_true', help='decode Annex B and encode bounded private HLS')
    connect.add_argument('--video-fps', type=float, default=30)
    connect.add_argument('--video-codec', choices=('h264', 'hevc'), default='h264')
    connect.add_argument('--publish-config', type=Path)

    report = commands.add_parser("report", help="generate interactive bit-level HTML report")
    report.add_argument("capture", type=Path)
    report.add_argument("--output", "-o", type=Path)

    simulate = commands.add_parser('simulate', help='exercise the local supervisor with synthetic telemetry and virtual time')
    simulate.add_argument('--scenario', choices=SCENARIOS, default='normal')
    simulate.add_argument('--journal', type=Path, required=True)

    metric_model = commands.add_parser('simulate-metric-route',
        help='offline metric out/back or scan/advance/land feedback with synthetic occupancy and delayed kinematics')
    metric_model.add_argument('--scenario', choices=('normal', 'stalled', 'pose-loss', 'owner-change', 'obstacle'), default='normal')
    metric_model.add_argument('--response-gain', type=float, default=1.)
    metric_model.add_argument('--mission', choices=('out-and-back', 'scan-advance-land'), default='out-and-back')
    metric_model.add_argument('--output', '-o', type=Path, required=True, help='new private model trajectory JSON')

    camera = commands.add_parser('camera-web', help='local HTML camera controls and live preview')
    camera.add_argument('--interface', required=True, help='dedicated drone Wi-Fi interface')
    camera.add_argument('--port', type=int, default=8793)
    camera.add_argument('--duration', type=float, default=300)
    camera.add_argument('--wlm-identity-file', type=Path, default=Path('private/client-wlm-identity.json'))
    camera.add_argument('--ble-identity-file', type=Path, default=Path('private/client-ble-identity.json'))
    camera.add_argument('--ble-address', default='02:00:00:00:00:01')
    camera.add_argument('--capture-directory', type=Path, default=Path('private/camera-sessions'))
    camera.add_argument('--last-frame', type=Path, help='optional last captured image, labelled offline')

    flight = commands.add_parser('flight-web', help='local camera and explicitly confirmed flight controls')
    flight.add_argument('--interface', required=True, help='dedicated drone Wi-Fi interface')
    flight.add_argument('--port', type=int, default=8794)
    flight.add_argument('--duration', type=float, default=300)
    profile=flight.add_mutually_exclusive_group()
    profile.add_argument('--vertical-trial', action='store_true',
        help='experimental vertical-only test: no GPS/distance requirement, no directional control, 10 s/1.5 m software guards')
    profile.add_argument('--yaw-trial',action='store_true',
        help='explicit yaw-only trial: no translation/throttle or GPS requirement, 30 s/1.5 m software guards')
    flight.add_argument('--yaw-channel-limit',type=int,default=200,
        help='local yaw-trial stick ceiling in 1..600 (default 200); requires --yaw-trial when changed')
    flight.add_argument('--acquisition-trial',action='store_true',
        help='explicit yaw profile extension: vertical channel <=200, no horizontal motion, 60 s/1.8 m guards; requires --yaw-trial')
    flight.add_argument('--wlm-identity-file', type=Path, default=Path('private/client-wlm-identity.json'))
    flight.add_argument('--ble-identity-file', type=Path, default=Path('private/client-ble-identity.json'))
    flight.add_argument('--ble-address', default='02:00:00:00:00:01')
    flight.add_argument('--capture-directory', type=Path, default=Path('private/flight-sessions'))
    flight.add_argument('--incident-directory', type=Path,
        default=Path.home()/'.local/state/neo2-control/flight-incidents',
        help='persistent private flight uncertainty, independent of session captures')
    flight.add_argument('--journal', type=Path, help='new private decision journal; generated when omitted')

    mission=commands.add_parser('yaw360',help='one explicitly confirmed rotation using an already connected yaw-trial operator page')
    mission.add_argument('--url',default='http://127.0.0.1:8794')
    mission.add_argument('--operator-ready',action='store_true',required=True,
        help='operator confirms grounded, stopped, cable-free aircraft and remains at distance')
    mission.add_argument('--journal',type=Path,required=True,help='new private mission journal')

    program=commands.add_parser('run-program',help='takeoff, execute a bounded motion JSON program on the PC owner, then land')
    program.add_argument('program',type=Path)
    endpoint=program.add_mutually_exclusive_group()
    endpoint.add_argument('--url',default='http://127.0.0.1:8794')
    endpoint.add_argument('--cloud-api',help='HTTPS control API, for example https://example.com/home-server/api/drone/control')
    program.add_argument('--token-file',type=Path,help='private file containing the current console bearer token')
    program.add_argument('--operator-ready',action='store_true',required=True,
        help='operator confirms grounded, stopped, cable-free aircraft and remains at distance')
    program.add_argument('--journal',type=Path,required=True,help='new private mission journal')

    obstacles=commands.add_parser('obstacles-report',help='offline raw birdview proximity report from an existing capture')
    obstacles.add_argument('capture',type=Path)
    obstacles.add_argument('--output','-o',type=Path,required=True,help='new standalone HTML report; no drone access')

    gimbal=commands.add_parser('gimbal-report',help='offline UDP gimbal fields; no camera-frame or clock calibration')
    gimbal.add_argument('capture',type=Path)
    gimbal.add_argument('--output','-o',type=Path,required=True,help='new private JSON report; no drone access')

    reconstruction=commands.add_parser('reconstruct-pair',help='offline sparse 3D points; no metric scale or flight authority')
    reconstruction.add_argument('first',type=Path)
    reconstruction.add_argument('second',type=Path)
    reconstruction.add_argument('--camera-model',type=Path,required=True)
    reconstruction.add_argument('--noise-trials',type=int,default=16,
        help='pixel perturbation trials:8..32;0 retains an explicitly unchecked candidate')
    reconstruction.add_argument('--noise-report',type=Path,
        help='new diagnostic JSON, also retained when the numerical stability check rejects the pair')
    reconstruction.add_argument('--output','-o',type=Path,required=True,help='new client scan JSON')

    scan=commands.add_parser('scan-report',help='offline interactive 3D view of a saved client scan')
    scan.add_argument('scan',type=Path)
    scan.add_argument('--output','-o',type=Path,required=True)

    localization=commands.add_parser('scan-localize',help='offline camera pose in the saved scan and its retained units')
    localization.add_argument('scan',type=Path)
    localization.add_argument('image',type=Path)
    localization.add_argument('--output','-o',type=Path,required=True)

    tracking=commands.add_parser('scan-track',help='offline ordered image localization in a frozen client map')
    tracking.add_argument('scan',type=Path)
    tracking.add_argument('images',type=Path,nargs='+')
    tracking.add_argument('--independent',action='store_true',help='use descriptors separately for every image')
    tracking.add_argument('--output','-o',type=Path,required=True)

    live_tracking=commands.add_parser('track-preview',help='read-only local camera pose observations with fresh stopped-aircraft telemetry')
    live_tracking.add_argument('scan',type=Path)
    live_tracking.add_argument('--url',default='http://127.0.0.1:8794')
    live_tracking.add_argument('--output','-o',type=Path,required=True,help='new private observation directory')
    live_tracking.add_argument('--duration',type=float,default=60.)
    live_tracking.add_argument('--observations',type=int,default=240)
    live_tracking.add_argument('--interval',type=float,default=.25)
    live_tracking.add_argument('--full-features',action='store_true',help='extract features at full image resolution (default pixel budget: 230400)')

    capture=commands.add_parser('capture-calibration',help='read-only local preview capture with fresh grounded/stopped telemetry')
    capture.add_argument('--url',default='http://127.0.0.1:8794',help='local PC controller URL')
    capture.add_argument('--board',type=Path,required=True,help='ChArUco reference-board JSON')
    capture.add_argument('--output','-o',type=Path,required=True,help='new private image directory')
    capture.add_argument('--views',type=int,default=16)
    capture.add_argument('--duration',type=float,default=120.)
    capture.add_argument('--interval',type=float,default=.5)

    calibration=commands.add_parser('calibrate-camera',help='offline checkerboard or ChArUco lens calibration from 10–40 images')
    calibration.add_argument('images',nargs='+',type=Path)
    calibration.add_argument('--columns',type=int,default=9,help='inner corners across')
    calibration.add_argument('--rows',type=int,default=6,help='inner corners down')
    calibration.add_argument('--charuco-board',type=Path,help='reference-board JSON, instead of the plain checkerboard')
    calibration.add_argument('--output','-o',type=Path,required=True,help='new camera-model JSON')
    calibration.add_argument('--evidence',type=Path,required=True,help='new fit/provenance JSON')

    reference=commands.add_parser('reference-board',help='create a printable ChArUco reference; measure its physical size before use')
    reference.add_argument('--reference-id',required=True,help='fixed physical reference name; do not move or duplicate this board')
    reference.add_argument('--first-marker-id',type=int,default=0)
    reference.add_argument('--output','-o',type=Path,required=True,help='new printable A4 SVG')
    reference.add_argument('--spec',type=Path,required=True,help='new board specification JSON')

    reference_pose=commands.add_parser('reference-pose',help='offline camera pose relative to a physically measured ChArUco board')
    reference_pose.add_argument('image',type=Path)
    reference_pose.add_argument('--camera-model',type=Path,required=True)
    reference_pose.add_argument('--board',type=Path,required=True)
    reference_pose.add_argument('--measured-square-mm',type=float,required=True)
    reference_pose.add_argument('--measurement-note',required=True,help='how the actual physical square was measured')
    reference_pose.add_argument('--output','-o',type=Path,required=True)

    anchor=commands.add_parser('scan-anchor',help='anchor a saved sparse scan to the same two images of a fixed measured reference')
    anchor.add_argument('scan',type=Path)
    anchor.add_argument('--first-pose',type=Path,required=True)
    anchor.add_argument('--second-pose',type=Path,required=True)
    anchor.add_argument('--output','-o',type=Path,required=True)

    dense=commands.add_parser('scan-densify',help='offline temporal-stereo depth from the exact images of a metric scan')
    dense.add_argument('scan',type=Path)
    dense.add_argument('first',type=Path)
    dense.add_argument('second',type=Path)
    dense.add_argument('--stride',type=int,default=4)
    dense.add_argument('--extension',type=int,help='atlas extension index; defaults to the latest extension')
    dense.add_argument('--minimum-depth',type=float,default=.25)
    dense.add_argument('--maximum-depth',type=float,default=4.)
    dense.add_argument('--output','-o',type=Path,required=True)

    memory=commands.add_parser('memory-build',help='offline voxel observations from metric depth scans; unknown remains unknown')
    memory.add_argument('scans',nargs='+',type=Path)
    memory.add_argument('--lower',nargs=3,type=float,required=True)
    memory.add_argument('--upper',nargs=3,type=float,required=True)
    memory.add_argument('--resolution',type=float,default=.1)
    memory.add_argument('--surface-margin',type=float,default=.05,help='declared model margin, not a calibrated error bound')
    memory.add_argument('--output','-o',type=Path,required=True)

    extension=commands.add_parser('memory-extend',help='retain previous observations and fuse new same-frame depth scans')
    extension.add_argument('memory',type=Path)
    extension.add_argument('scans',nargs='+',type=Path)
    extension.add_argument('--output','-o',type=Path,required=True)

    plan=commands.add_parser('memory-plan',help='offline candidate for measured out-and-back flight; no actuator commands')
    plan.add_argument('memory',type=Path)
    plan.add_argument('--position',nargs=3,type=float,required=True,help='planned hover position in the saved frame')
    plan.add_argument('--forward',nargs=3,type=float,required=True)
    plan.add_argument('--up',nargs=3,type=float,required=True,help='physical up vector in the saved frame; never inferred from board axes')
    plan.add_argument('--radius',type=float,required=True,help='declared radius enclosing the complete aircraft including propellers')
    plan.add_argument('--distance',type=float,default=1.)
    plan.add_argument('--tracking-margin',type=float,default=.1)
    plan.add_argument('--angle-step',type=int,default=15)
    plan.add_argument('--output','-o',type=Path,required=True)

    atlas=commands.add_parser('atlas-create',help='retain a metric seed for incremental scene landmarks')
    atlas.add_argument('scan',type=Path)
    atlas.add_argument('--output','-o',type=Path,required=True)

    growth=commands.add_parser('atlas-extend',help='add landmarks from two translated views tracked against the existing atlas')
    growth.add_argument('atlas',type=Path)
    growth.add_argument('first',type=Path)
    growth.add_argument('second',type=Path)
    growth.add_argument('--noise-report',type=Path,help='new pixel-perturbation diagnostic, including a stability rejection')
    growth.add_argument('--output','-o',type=Path,required=True)

    export=commands.add_parser('export-video',help='export a recorded local elementary stream as a new MP4')
    export.add_argument('source',type=Path)
    export.add_argument('--output','-o',type=Path,required=True)
    export.add_argument('--codec',choices=('h264','hevc'),default='hevc')
    export.add_argument('--fps',type=float,default=30.,help='local nominal timeline; not camera timestamps')

    clocks=commands.add_parser('video-clock-report',help='offline candidate-clock inventory of original HEVC; no camera-time claim')
    clocks.add_argument('source',type=Path)
    clocks.add_argument('--output','-o',type=Path,required=True,help='new private JSON report')

    lineage=commands.add_parser('video-lineage',help='offline decoded-frame identity, original HEVC clocks and selected lossless PNGs')
    lineage.add_argument('source',type=Path)
    lineage.add_argument('--output','-o',type=Path,required=True,help='new private output directory')
    lineage.add_argument('--picture',type=int,action='append',default=[],help='raw first-slice ordinal to save as PNG; repeat up to32 times')
    lineage.add_argument('--fps',type=float,default=30.,help='local decoder normalization, never an exposure clock')
    lineage.add_argument('--timeout',type=float,default=180.)

    api = commands.add_parser("serve", help="loopback diagnostic API for the Scaleway tunnel")
    api.add_argument("--listen", default="127.0.0.1")
    api.add_argument("--port", type=int, default=8792)
    api.add_argument("--token-file", type=Path, required=True)
    api.add_argument("--capture-dir", type=Path, default=Path("captures"))
    api.add_argument("--report-dir", type=Path, default=Path("reports"))
    api.add_argument("--allow-active-syn", action="store_true")
    api.add_argument('--drone-mode', choices=('observe', 'simulate'), default='observe')
    api.add_argument('--controller-journal', type=Path)
    api.add_argument('--observe-interface', help='locally selected dedicated interface for managed receive sessions')
    api.add_argument('--observe-max-duration', type=float, default=600)
    api.add_argument('--observe-heartbeat', action='store_true')
    api.add_argument('--observe-video', action='store_true', help='save ordered video callback bytes during managed sessions')
    api.add_argument('--observe-credentials-file', type=Path, help='locally selected private Wi-Fi parameters; activate and clean up one temporary profile')
    api.add_argument('--observe-wlm-identity-file', type=Path, help='locally select a persistent WLM identity and enable Wi-Fi report scheduling')
    api.add_argument('--observe-ble-address', help='locally select BLE credentials acquisition before Wi-Fi activation')
    api.add_argument('--observe-ble-identity-file', type=Path)
    api.add_argument('--observe-ble-setup-timeout', type=float, default=30)
    api.add_argument('--observe-hls', action='store_true', help='enable local Annex B decoding and private HLS')
    api.add_argument('--observe-video-fps', type=float, default=30)
    api.add_argument('--observe-video-codec', choices=('h264', 'hevc'), default='h264')
    api.add_argument('--observe-publish-config', type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "ble-scan":
            devices = [device.as_dict() for device in scan_sync(args.duration)]
            if args.json:
                print(json.dumps(devices, ensure_ascii=False, indent=2))
            else:
                candidates = [device for device in devices if device["candidate"]]
                print(f"{len(devices)} appareil(s), {len(candidates)} candidat(s) DJI/Neo")
                for device in candidates:
                    print(f"- {device['name'] or '?'} {device['address']} RSSI={device['rssi']} services={','.join(device['service_uuids']) or '-'}")
            return 0
        if args.command == "ble-hci-capture":
            output = args.output or Path("captures") / f"ble-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.btsnoop"
            print(capture_hci(output, args.duration))
            return 0
        if args.command == "ble-gatt-probe":
            if (args.interval_min_units is None) != (args.interval_max_units is None):
                raise ValueError("both interval bounds must be supplied together")
            if args.interval_min_units is None:
                result = probe_gatt_sync(args.address, args.timeout)
            else:
                result = probe_gatt_with_intervals(
                    args.address,
                    minimum_units=args.interval_min_units,
                    maximum_units=args.interval_max_units,
                    timeout=args.timeout,
                )
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0
        if args.command == "ble-read":
            result = read_gatt_values_with_intervals(
                args.address,
                args.uuid,
                minimum_units=args.interval_min_units,
                maximum_units=args.interval_max_units,
                timeout=args.timeout,
            )
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0
        if args.command == "ble-notify":
            result = observe_gatt_notifications_with_intervals(
                args.address,
                args.uuid,
                duration=args.duration,
                minimum_units=args.interval_min_units,
                maximum_units=args.interval_max_units,
                timeout=args.timeout,
            )
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0
        if args.command == "ble-wifi-setup":
            result = run_ble_wifi_setup_sync(
                address=args.address,
                identity_file=args.identity_file,
                credentials_path=args.credentials_file,
                capture_path=args.capture or Path("private") / _capture_name("ble-setup").name,
                timeout=args.timeout,
                request_timeout=args.request_timeout,
                interval_min_units=args.interval_min_units,
                interval_max_units=args.interval_max_units,
                wlm_identity_file=args.wlm_identity_file,
            )
            print(json.dumps(result.as_dict(), ensure_ascii=False, indent=2))
            return 0 if result.state == "complete" else 2
        if args.command == "btsnoop-duml":
            print(
                json.dumps(
                    decode_neo2_session(args.capture, args.address),
                    ensure_ascii=False,
                    indent=2,
                )
            )
            return 0
        if args.command == "udp-probe":
            capture = args.capture or _capture_name("udp")
            result = run_probe(
                capture_path=capture,
                host=args.host,
                port=args.port,
                duration=args.duration,
                send_syn=args.send_syn,
                send_heartbeat=args.send_heartbeat,
                send_wifi_device_info=args.send_wifi_device_info,
                identity_file=args.identity_file,
                send_wifi_then_heartbeat=args.send_wifi_then_heartbeat,
                send_static_capability=args.send_static_capability,
                send_idle_window=args.send_idle_window,
                idle_window_count=args.idle_window_count,
                idle_window_interval=args.idle_window_interval,
                idle_window_mode=args.idle_window_mode,
                interface=args.interface,
            )
            print(json.dumps(result.as_dict(), ensure_ascii=False, indent=2))
            return 0
        if args.command == "identity-init":
            identity = load_or_create_identity(args.file)
            print(json.dumps({"store": str(args.file), **identity.metadata()}, indent=2))
            return 0
        if args.command == "session":
            result = run_session(
                capture_path=args.capture or _capture_name("session"),
                host=args.host,
                port=args.port,
                interface=args.interface,
                duration=args.duration,
                heartbeat=args.heartbeat,
                video_path=args.video_output,
                wlm_identity_file=args.wlm_identity_file,
                **({'wlm_mode_follow':True} if args.wlm_mode_follow else {}),
                **({'wlm_wifi_liveview':True} if args.wlm_wifi_liveview else {}),
                hls_directory=args.hls_dir, video_nominal_fps=args.video_fps, video_codec=args.video_codec,
                publish_config_file=args.publish_config,
            )
            print(json.dumps(result.as_dict(), ensure_ascii=False, indent=2))
            success = (result.state == 'closed' and result.transport_accepted
                and (args.hls_dir is None or (not result.video.get('error') and result.video.get('frames_encoded', 0) > 0)))
            if args.publish_config is not None:
                success = success and publication_succeeded(result.video_publish)
            return 0 if success else 2
        if args.command == "wifi-session":
            result = run_wifi_session(
                credentials_file=args.credentials_file, interface=args.interface,
                capture_path=args.capture or Path("private") / _capture_name("wifi-session").name,
                duration=args.duration, heartbeat=args.heartbeat, video_path=args.video_output,
                apply=args.apply,
                wlm_identity_file=args.wlm_identity_file,
                hls_directory=args.hls_dir, video_nominal_fps=args.video_fps, video_codec=args.video_codec,
                publish_config_file=args.publish_config,
            )
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0 if result["state"] in ("planned", "closed") else 2
        if args.command == 'connect-session':
            config = ObservationConfig(args.interface, maximum_duration=args.duration,
                heartbeat=args.heartbeat, record_video=args.video,
                ble_address=args.address, ble_identity_file=args.ble_identity_file,
                wlm_identity_file=args.wlm_identity_file, ble_setup_timeout=args.ble_setup_timeout,
                record_hls=args.hls, video_nominal_fps=args.video_fps, video_codec=args.video_codec,
                publish_config_file=args.publish_config)
            manager = ManagedObservation(config, args.capture_dir)
            main_thread = threading.current_thread() is threading.main_thread()
            previous_sigterm = signal.getsignal(signal.SIGTERM) if main_thread else None
            term_requested = False
            def terminate(signum, frame):
                nonlocal term_requested
                if not term_requested:
                    term_requested = True
                    raise KeyboardInterrupt
            if main_thread:
                signal.signal(signal.SIGTERM, terminate)
            try:
                manager.open({'duration': args.duration})
                while manager.busy:
                    time.sleep(.05)
            finally:
                try:
                    manager.shutdown()
                finally:
                    if main_thread:
                        signal.signal(signal.SIGTERM, previous_sigterm)
            result = manager.snapshot()
            print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
            video = result['session'].get('video', {})
            success = (result['state'] == 'closed' and result['session'].get('transport_accepted')
                and (not args.hls or (not video.get('error') and video.get('frames_encoded', 0) > 0)))
            if args.publish_config is not None:
                success = success and publication_succeeded(result['session'].get('video_publish', {}))
            return 0 if success else 2
        if args.command == "report":
            output = args.output or Path("reports") / f"{args.capture.stem}.html"
            print(generate_report(args.capture, output))
            return 0
        if args.command == 'simulate':
            result = run_simulation(args.journal, args.scenario)
            print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
            return 0 if result['journal_healthy'] and result['scenario_passed'] else 2
        if args.command == 'simulate-metric-route':
            from .metric_model import simulate_metric_route
            from .reconstruction import _json_file, _vision
            cv, _ = _vision(); cv.setNumThreads(2)
            result = simulate_metric_route(scenario=args.scenario, response_gain=args.response_gain, mission=args.mission)
            _json_file(args.output, result)
            summary = {k: result[k] for k in ('mission', 'scenario', 'scenario_passed', 'duration_s',
                'outbound_endpoint_error_m', 'return_endpoint_error_m', 'final_heading_error_deg',
                'flight_authorized')}
            summary.update(stage=result['feedback']['stage'], reason=result['feedback']['reason'],
                output=str(args.output.absolute()))
            print(json.dumps(summary, ensure_ascii=False, allow_nan=False))
            return 0 if result['scenario_passed'] else 2
        if args.command == 'camera-web':
            from .camera_console import CameraConfig, CameraConsole
            from .camera_web import serve_camera_web
            config=CameraConfig(interface=args.interface,wlm_identity_file=args.wlm_identity_file,
                ble_identity_file=args.ble_identity_file,ble_address=args.ble_address,
                duration=args.duration,capture_directory=args.capture_directory,last_frame=args.last_frame)
            serve_camera_web(CameraConsole(config),port=args.port)
            return 0
        if args.command == 'flight-web':
            from .flight_connection import FlightConfig
            from .flight_web import FlightWeb,serve_flight_web
            if not 0<=args.port<=65535:raise ValueError('invalid_flight_port')
            config=FlightConfig(interface=args.interface,wlm_identity_file=args.wlm_identity_file,
                ble_identity_file=args.ble_identity_file,ble_address=args.ble_address,
                duration=args.duration,capture_directory=args.capture_directory,camera_preview=True,
                vertical_trial=args.vertical_trial,yaw_trial=args.yaw_trial,
                acquisition_trial=args.acquisition_trial,
                yaw_channel_limit=args.yaw_channel_limit,incident_directory=args.incident_directory)
            if args.journal is None:
                import secrets
                args.journal=Path('private/flight-journals')/(secrets.token_hex(12)+'.jsonl')
            serve_flight_web(FlightWeb(config,args.journal),port=args.port)
            return 0
        if args.command=='run-program':
            from .buffered_operator_client import BufferedFlightOperatorClient
            from .cloud_operator_client import CloudFlightOperatorClient,read_bearer_token
            from .controller import DecisionJournal
            from .program_mission import read_program,run_program_mission
            if args.cloud_api and args.token_file is None:raise ValueError('cloud_token_file_required')
            if args.token_file is not None and not args.cloud_api:raise ValueError('cloud_token_file_requires_cloud_api')
            program=read_program(args.program)
            token=None if args.token_file is None else read_bearer_token(args.token_file)
            journal=DecisionJournal(args.journal)
            try:
                client=(BufferedFlightOperatorClient(args.url) if args.cloud_api is None else
                    CloudFlightOperatorClient(args.cloud_api,token))
                result=run_program_mission(client,program,
                    operator_confirmed_grounded=args.operator_ready,event_sink=journal.record)
                journal.record({'event':'mission_result',**result})
                print(json.dumps({k:v for k,v in result.items() if k!='last_status'},ensure_ascii=False))
                if result['failure'] is not None or not result['motion_complete']:return 1
                return 2 if result['last_status'].get('possible_airborne') is not False else 0
            finally:journal.close()
        if args.command=='yaw360':
            from .controller import DecisionJournal
            from .operator_client import FlightOperatorClient
            from .yaw_mission import run_yaw_mission
            journal=DecisionJournal(args.journal)
            try:
                result=run_yaw_mission(FlightOperatorClient(args.url),
                    operator_confirmed_grounded=args.operator_ready,event_sink=journal.record)
                journal.record({'event':'mission_result',**result})
                print(json.dumps({k:v for k,v in result.items() if k!='last_status'},ensure_ascii=False))
                return 0 if result['failure'] is None and result['rotation']['complete'] else 1
            finally:journal.close()
        if args.command=='obstacles-report':
            from .obstacles import write_birdview_report
            print(json.dumps(write_birdview_report(args.capture,args.output),ensure_ascii=False))
            return 0
        if args.command=='gimbal-report':
            from .gimbal_telemetry import write_gimbal_report
            print(json.dumps(write_gimbal_report(args.capture,args.output),ensure_ascii=False))
            return 0
        if args.command in ('reconstruct-pair','scan-report','scan-localize','scan-track','track-preview','calibrate-camera','capture-calibration',
                'reference-board','reference-pose','scan-anchor','scan-densify',
                'memory-build','memory-extend','memory-plan','atlas-create','atlas-extend'):
            from .reconstruction import _vision
            cv,_=_vision();cv.setNumThreads(2)
        if args.command=='track-preview':
            from .metric_reference import load_client_scan
            from .operator_client import FlightOperatorClient
            from .preview_tracking import track_preview
            result=track_preview(FlightOperatorClient(args.url),load_client_scan(args.scan),args.output,
                duration=args.duration,observations=args.observations,interval=args.interval,
                maximum_feature_pixels=None if args.full_features else 640*360,
                event_sink=lambda event:print(json.dumps(event,ensure_ascii=False),file=sys.stderr,flush=True))
            print(json.dumps({k:v for k,v in result.items() if k!='samples'},ensure_ascii=False))
            if result['reason']=='preview_tracking_interrupted':return 130
            if result['reason'] not in ('observation_limit','capture_deadline') or not result['localized_count']:return 1
            return 2 if result['lost_count'] else 0
        if args.command=='capture-calibration':
            from .calibration_capture import capture_calibration
            from .metric_reference import read_reference_board
            from .operator_client import FlightOperatorClient
            board=read_reference_board(args.board)
            result=capture_calibration(FlightOperatorClient(args.url),board,args.output,
                views=args.views,duration=args.duration,interval=args.interval,
                event_sink=lambda event:print(json.dumps(event,ensure_ascii=False),file=sys.stderr,flush=True))
            print(json.dumps({k:v for k,v in result.items() if k!='sources'},ensure_ascii=False))
            return 0 if result['target_reached'] else 1
        if args.command=='reconstruct-pair':
            from .reconstruction import read_camera_model,reconstruct_pair,save_scan,_json_file
            from .pair_stability import PairStabilityError
            outputs=[args.output]+([args.noise_report] if args.noise_report is not None else [])
            if (len({p.absolute() for p in outputs})!=len(outputs)
                    or any(p.exists() or p.is_symlink() for p in outputs)):
                raise ValueError('reconstruction_outputs_must_be_new_and_distinct')
            model=read_camera_model(args.camera_model)
            try:scan=reconstruct_pair(args.first,args.second,model,noise_trials=args.noise_trials)
            except PairStabilityError as error:
                if args.noise_report is not None:_json_file(args.noise_report,error.report)
                raise
            if args.noise_report is not None:_json_file(args.noise_report,scan['pair_noise_check'])
            print(json.dumps(save_scan(args.output,scan),ensure_ascii=False))
            return 0
        if args.command=='scan-report':
            from .metric_reference import load_client_scan
            from .scan_report import write_scan_report
            print(json.dumps(write_scan_report(load_client_scan(args.scan),args.output),ensure_ascii=False))
            return 0
        if args.command=='scan-localize':
            from .reconstruction import localize_scan,_json_file
            from .metric_reference import load_client_scan,localize_metric_scan
            scan=load_client_scan(args.scan)
            if scan['format'] in ('neo2-control/depth-scan/v1','neo2-control/depth-scan/v2','neo2-control/spatial-memory/v1'):
                raise ValueError('scan_has_no_tracking_descriptors_use_metric_scan')
            if scan['format']=='neo2-control/metric-atlas/v1':
                from .metric_atlas import localize_atlas
                operation=localize_atlas
            else:
                operation=localize_metric_scan if scan['format']=='neo2-control/metric-scan/v1' else localize_scan
            result=operation(scan,args.image)
            _json_file(args.output,result)
            print(json.dumps(result,ensure_ascii=False))
            return 0
        if args.command=='scan-track':
            from .metric_reference import load_client_scan
            from .reconstruction import _json_file
            from .visual_tracking import track_images
            if args.output.exists() or args.output.is_symlink():raise ValueError('tracking_output_must_be_new')
            result=track_images(load_client_scan(args.scan),args.images,temporal=not args.independent,
                event_sink=lambda event:print(json.dumps(event,ensure_ascii=False),file=sys.stderr,flush=True))
            _json_file(args.output,result)
            print(json.dumps({key:result[key] for key in ('coordinate_frame','distance_unit',
                'localized_count','lost_count','continuous_segments','flight_authorized')},ensure_ascii=False))
            return 0 if not result['lost_count'] else 2 if result['localized_count'] else 1
        if args.command=='reference-board':
            from .metric_reference import ReferenceBoard,write_reference_board
            from .reconstruction import _json_file
            if args.output.absolute()==args.spec.absolute():raise ValueError('reference_outputs_must_differ')
            if any(path.exists() or path.is_symlink() for path in (args.output,args.spec)):
                raise ValueError('reference_outputs_must_be_new')
            board=ReferenceBoard(args.reference_id,args.first_marker_id)
            result=write_reference_board(board,args.output)
            _json_file(args.spec,board.as_dict())
            print(json.dumps(result|{'spec':str(args.spec.absolute())},ensure_ascii=False))
            return 0
        if args.command=='reference-pose':
            from .metric_reference import read_reference_board,estimate_reference_pose
            from .reconstruction import read_camera_model,_json_file
            result=estimate_reference_pose(args.image,read_camera_model(args.camera_model),
                read_reference_board(args.board),measured_square_mm=args.measured_square_mm,
                measurement_note=args.measurement_note)
            _json_file(args.output,result)
            print(json.dumps(result,ensure_ascii=False))
            return 0
        if args.command=='scan-anchor':
            from .metric_reference import anchor_scan,read_reference_pose,save_metric_scan
            from .reconstruction import load_scan
            result=anchor_scan(load_scan(args.scan),read_reference_pose(args.first_pose),
                read_reference_pose(args.second_pose))
            print(json.dumps(save_metric_scan(args.output,result),ensure_ascii=False))
            return 0
        if args.command=='scan-densify':
            from .metric_reference import load_client_scan
            from .depth_scan import densify_scan,save_depth_scan
            scan=load_client_scan(args.scan)
            options={'stride':args.stride,'minimum_m':args.minimum_depth,'maximum_m':args.maximum_depth}
            if scan['format']=='neo2-control/metric-atlas/v1':
                from .metric_atlas import densify_atlas
                index=args.extension if args.extension is not None else len(scan['extensions'])-1
                result=densify_atlas(scan,index,args.first,args.second,**options)
            elif scan['format']=='neo2-control/metric-scan/v1' and args.extension is None:
                result=densify_scan(scan,args.first,args.second,**options)
            else:raise ValueError('depth_source_requires_metric_scan_or_atlas')
            print(json.dumps(save_depth_scan(args.output,result),ensure_ascii=False))
            return 0
        if args.command in ('memory-build','memory-extend'):
            from .spatial_memory import build_memory,load_memory,save_memory
            from .depth_scan import load_depth_scan
            scans=[load_depth_scan(path) for path in args.scans]
            if args.command=='memory-extend':
                previous=load_memory(args.memory)
                scans=previous['sources']+scans
                options={'lower':previous['lower'],'upper':previous['upper'],
                    'resolution':previous['resolution_m'],'surface_margin':previous['surface_margin_m']}
            else:
                options={'lower':args.lower,'upper':args.upper,'resolution':args.resolution,
                    'surface_margin':args.surface_margin}
            print(json.dumps(save_memory(args.output,build_memory(scans,**options)),ensure_ascii=False))
            return 0
        if args.command=='memory-plan':
            from .spatial_memory import choose_heading,load_memory
            from .reconstruction import _json_file
            result=choose_heading(load_memory(args.memory),position=args.position,
                forward=args.forward,up=args.up,radius_m=args.radius,distance_m=args.distance,
                tracking_margin_m=args.tracking_margin,angle_step_degrees=args.angle_step)
            _json_file(args.output,result)
            print(json.dumps({'output':str(args.output.absolute()),'candidate':result['candidate'],
                'flight_authorized':False},ensure_ascii=False))
            return 0
        if args.command=='atlas-create':
            from .metric_reference import load_metric_scan
            from .metric_atlas import create_atlas,save_atlas
            print(json.dumps(save_atlas(args.output,create_atlas(load_metric_scan(args.scan))),ensure_ascii=False))
            return 0
        if args.command=='atlas-extend':
            from .metric_atlas import load_atlas,extend_atlas,save_atlas
            from .pair_stability import PairStabilityError
            from .reconstruction import _json_file
            outputs=[args.output]+([args.noise_report] if args.noise_report is not None else [])
            if (len({p.absolute() for p in outputs})!=len(outputs)
                    or any(p.exists() or p.is_symlink() for p in outputs)):
                raise ValueError('atlas_outputs_must_be_new_and_distinct')
            try:atlas=extend_atlas(load_atlas(args.atlas),args.first,args.second)
            except PairStabilityError as error:
                if args.noise_report is not None:_json_file(args.noise_report,error.report)
                raise
            if args.noise_report is not None:_json_file(args.noise_report,atlas['extensions'][-1]['pair_noise_check'])
            print(json.dumps(save_atlas(args.output,atlas),ensure_ascii=False))
            return 0
        if args.command=='calibrate-camera':
            from .calibration import calibrate_checkerboard
            from .reconstruction import _json_file
            if args.output.absolute()==args.evidence.absolute():raise ValueError('calibration_outputs_must_differ')
            if any(path.exists() or path.is_symlink() for path in (args.output,args.evidence)):
                raise ValueError('calibration_outputs_must_be_new')
            if args.charuco_board is not None:
                from .calibration import calibrate_charuco
                from .metric_reference import read_reference_board
                model,result=calibrate_charuco(args.images,read_reference_board(args.charuco_board))
            else:
                model,result=calibrate_checkerboard(args.images,columns=args.columns,rows=args.rows)
            _json_file(args.output,model.as_dict());_json_file(args.evidence,result)
            print(json.dumps({'output':str(args.output.absolute()),'evidence':str(args.evidence.absolute()),
                'accepted_views':result['accepted_views'],'rms_pixels':result['rms_pixels'],
                'metric_scale_verified':False},ensure_ascii=False))
            return 0
        if args.command=='video-lineage':
            from .video_lineage import build_video_lineage
            result=build_video_lineage(args.source,args.output,pictures=args.picture,
                nominal_fps=args.fps,timeout=args.timeout)
            print(json.dumps({'output':str(args.output.absolute()),'source':result['source'],
                'summary':result['summary'],'flight_authorized':False},ensure_ascii=False))
            return 0
        if args.command=='video-clock-report':
            from .video_clock import inventory_hevc_clocks
            from .reconstruction import _json_file
            if args.output.exists() or args.output.is_symlink():raise ValueError('video_clock_output_must_be_new')
            result=inventory_hevc_clocks(args.source)
            _json_file(args.output,result)
            print(json.dumps({k:v for k,v in result.items() if k not in ('pictures','malformed_sei')},ensure_ascii=False))
            return 0
        if args.command=='export-video':
            from .video_export import export_video
            print(json.dumps(export_video(args.source,args.output,input_codec=args.codec,nominal_fps=args.fps),ensure_ascii=False))
            return 0
        if args.command == "serve":
            token = args.token_file.read_text(encoding="utf-8").strip()
            serve(
                listen=args.listen,
                port=args.port,
                token=token,
                capture_dir=args.capture_dir,
                report_dir=args.report_dir,
                allow_active_syn=args.allow_active_syn,
                drone_mode=args.drone_mode,
                controller_journal=args.controller_journal,
                observe_interface=args.observe_interface,
                observe_max_duration=args.observe_max_duration,
                observe_heartbeat=args.observe_heartbeat,
                observe_video=args.observe_video,
                observe_credentials_file=args.observe_credentials_file,
                observe_wlm_identity_file=args.observe_wlm_identity_file,
                observe_ble_address=args.observe_ble_address,
                observe_ble_identity_file=args.observe_ble_identity_file,
                observe_ble_setup_timeout=args.observe_ble_setup_timeout,
                observe_hls=args.observe_hls,
                observe_video_fps=args.observe_video_fps,
                observe_video_codec=args.observe_video_codec,
                observe_publish_config=args.observe_publish_config,
            )
            return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
