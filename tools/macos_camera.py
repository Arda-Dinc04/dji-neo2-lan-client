#!/usr/bin/env python3
"""Mac camera preview, disconnected by default; active setup requires an explicit flag."""
import argparse
import json
from pathlib import Path
import platform
import re
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from neo2_control.camera_console import CameraConfig, CameraConsole
from neo2_control.camera_web import serve_camera_web
from neo2_control.macos_wifi import wifi_identity


class MacPreviewConsole(CameraConsole):
    def __init__(self, config, *, setup_authorized=False):
        super().__init__(config)
        self.setup_authorized = setup_authorized

    def connect(self):
        if not self.setup_authorized:raise RuntimeError('camera_setup_requires_operator_enable')
        return super().connect()

    def status(self):
        result = super().status()
        result['session_setup_authorized'] = self.setup_authorized
        return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preflight',action='store_true',help='Read local readiness only; no DJI application writes')
    parser.add_argument('--allow-camera-session-setup',action='store_true',
                        help='Authorize BLE registration, credential reads and WLM live-view service selection')
    parser.add_argument('--startup-keyframe-refresh',action='store_true',
                        help='After a committed live-view switch, allow up to three native encoder refresh requests before first video')
    parser.add_argument('--interface',default='en0')
    parser.add_argument('--port',type=int,default=8793)
    parser.add_argument('--duration',type=float,default=60)
    parser.add_argument('--ble-id',help='CoreBluetooth peripheral UUID discovered on this Mac')
    args=parser.parse_args()
    if platform.system()!='Darwin':parser.error('This entry point uses macOS CoreBluetooth/CoreWLAN')
    reasons=[]
    address=args.ble_id
    if address is None:
        path=ROOT/'private/macos/ble-scan.json'
        devices=json.loads(path.read_text()) if path.is_file() else []
        devices=[d for d in devices if re.search(r'NEO[ _-]?2',d['name'].upper())
                 and '0000fff0-0000-1000-8000-00805f9b34fb' in d['service_uuids']]
        if len(devices)==1:address=devices[0]['address']
        else:reasons.append('unique_local_neo2_advertisement_required')
    if shutil.which('ffmpeg') is None:reasons.append('ffmpeg_missing')
    # The disconnected web page can be reviewed without requesting OS permission.
    if args.preflight or args.allow_camera_session_setup:
        try:wifi_identity(args.interface)
        except RuntimeError as error:reasons.append(str(error))
    if args.preflight:
        print(json.dumps({'ready':not reasons,'reasons':reasons,'active_setup_required':True,
                          'automatic_mac_wifi_changes':False,'dji_application_writes':0},indent=2))
        return 0 if not reasons else 2
    if reasons:parser.error('; '.join(reasons))
    private=ROOT/'private/macos'
    private.mkdir(mode=0o700,parents=True,exist_ok=True)
    config=CameraConfig(args.interface,private/'wlm-identity.json',private/'ble-identity.json',
                        ble_address=address,duration=args.duration,capture_directory=private/'camera-sessions',
                        preview_only=True,manual_macos_wifi=True,
                        startup_keyframe_refresh=args.startup_keyframe_refresh)
    console=MacPreviewConsole(config,setup_authorized=args.allow_camera_session_setup)
    print(json.dumps({'preview_only':True,'camera_setup_enabled':args.allow_camera_session_setup,
                      'startup_keyframe_refresh':args.startup_keyframe_refresh,
                      'automatic_mac_wifi_changes':False,'camera_actions_available':False}),flush=True)
    serve_camera_web(console,port=args.port)
    return 0


if __name__=='__main__':
    try:raise SystemExit(main())
    except (OSError,ValueError,RuntimeError) as error:
        print(str(error),file=sys.stderr);raise SystemExit(2)
