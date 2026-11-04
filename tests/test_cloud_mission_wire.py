"""HTTPS mission through owned UDP loopback; fake BLE/NM and video readiness.

The fixture supplies its own synthetic angle model. This proves orchestration
and wire behavior, not camera operation or DJI aircraft dynamics.
"""
import struct
import time

from neo2_control.flight_channel import FlightChannel
from neo2_control.program_mission import run_program_mission
from test_cloud_operator_client import certificate,gateway,make
from test_control_transport_session import ControllerPeer
from test_flight_owner import Owner


class ProgramPeer(ControllerPeer):
    """Loopback fixture with synthetic 120 degree/s yaw; no dynamics claim."""
    def __init__(self,channel):
        self.yaw=0.;self.rate=0.;self.last_yaw=time.monotonic()
        super().__init__(channel)
    def camera_frame(self,request,**options):
        if (request.cmd_set,request.cmd_id)==(3,0x43) and 'payload' in options:
            now=time.monotonic();self.yaw=(self.yaw+self.rate*(now-self.last_yaw)+180)%360-180
            self.last_yaw=now
            body=bytearray(options['payload']);struct.pack_into('<h',body,28,round(self.yaw*10))
            options['payload']=bytes(body)
        return super().camera_frame(request,**options)
    def additional_packet(self,inner,attributes):
        if (inner.cmd_set,inner.cmd_id)!=(1,0x0a):return super().additional_packet(inner,attributes)
        word=int.from_bytes(inner.payload[2:10],'little')
        channels=tuple((word>>shift)&2047 for shift in (8,19,30,41))
        self.controls.append(channels);self.flight_frames.append(inner);self.record_allocation(inner)
        self.rate=120. if channels[3]>1024 else -120. if channels[3]<1024 else 0.
        return b''


def test_full_https_mission_with_unrelated_clocks_drives_rotation_translation_and_land(tmp_path,monkeypatch,certificate):

    timestamps={'now','received_at','motion_received_at','control_received_at','battery_received_at',
        'land_requested_at','lease_expires_at','server_time','expires_at','issued_at','started_at',
        'deadline','step_started_at','time','ready_at','route_seen_at'}
    def shifted(value):
        if isinstance(value,dict):
            return {key:(field+1_000_000. if key in timestamps and type(field) in (int,float)
                else shifted(field)) for key,field in value.items()}
        if isinstance(value,list):return [shifted(field) for field in value]
        return value
    def response(value):
        value=shifted(value)
        status=value.get('status') if 'status' in value else value if 'controller_owner' in value else None
        if isinstance(status,dict):
            video=status.setdefault('connection',{}).setdefault('video',{})
            video.update(live=True,fixture_readiness=True)
        return value

    with ProgramPeer(FlightChannel()) as peer:
        owned=Owner(tmp_path,monkeypatch,peer,duration=40)
        try:
            owned.runtime.handle('open')
            owned.drive(lambda s:s.get('telemetry') is not None and s['telemetry']['control_ready'],renew=False)
            with gateway(tmp_path/'https',certificate,runtime_factory=lambda *a,**kw:owned.runtime) as (url,trust,bridge,app):
                bridge.transform=response;bridge.delay=.08
                client=make(url,trust)
                result=run_program_mission(client,{'version':1,'finish':'land','steps':[
                    {'kind':'translate','seconds':.3,'right':50,'forward':0,'up':0},
                    {'kind':'rotate','degrees':90,'channel':150,'timeout_seconds':3},
                    {'kind':'translate','seconds':.3,'right':0,'forward':50,'up':0}]},
                    operator_confirmed_grounded=True)
                assert result['failure'] is None and result['motion_complete']
                assert result['land_requested'] and result['needs_operator_ground_confirmation']
                assert result['program']['completed_steps']==3
                assert result['program']['last_rotation']['progress_degrees']>=90
                actions=[r['body']['action'] for r in bridge.requests if r['method']=='POST']
                assert actions.count('takeoff')==1 and actions.count('program_start')==1
                assert actions.count('confirm_ground')==1 and 'confirm_landing' not in actions
                assert any(c[0]>1024 for c in peer.controls)
                assert any(c[1]>1024 for c in peer.controls)
                assert any(c[3]>1024 for c in peer.controls)
                # Explicit fixture postflight attestation is outside the tested mission.
                owned.ground();owned.drive(lambda s:not s['possible_airborne'],renew=False)
        finally:owned.cleanup()
