"""Buffered HTTP targets through the real owner, flight queue and loopback UDP."""
from dataclasses import replace
import json
import struct
import time

from neo2_control.control_owner import ProcessControllerRuntime
from neo2_control.flight_channel import FlightChannel
from test_control_transport_session import ControllerPeer
from test_flight_owner import Owner
from test_flight_web import server, raw_request, window_ingress


def test_remote_targets_keep_native_yaw_cadence_across_slow_http_updates(tmp_path,monkeypatch):
    class Peer(ControllerPeer):
        def camera_frame(self,request,**options):
            if (request.cmd_set,request.cmd_id)==(3,0x43) and 'payload' in options:
                body=bytearray(options['payload'])
                struct.pack_into('<I',body,32,struct.unpack_from('<I',body,32)[0]&~0x388000)
                struct.pack_into('<3h',body,24,0,0,420)
                options['payload']=bytes(body)
            return super().camera_frame(request,**options)

        def additional_packet(self,inner,attributes):
            if (inner.cmd_set,inner.cmd_id)!=(1,0x0a):return super().additional_packet(inner,attributes)
            word=int.from_bytes(inner.payload[2:10],'little')
            channels=tuple((word>>shift)&2047 for shift in (8,19,30,41))
            assert channels[:3]==(1024,)*3 and channels[3] in (1024,1156)
            self.controls.append(channels);self.flight_frames.append(inner);self.record_allocation(inner)
            return b''

    initialize=ProcessControllerRuntime.__init__
    def yaw_owner(self,*args,**kwargs):
        kwargs['flight_config']=replace(kwargs['flight_config'],yaw_trial=True)
        initialize(self,*args,**kwargs)
    monkeypatch.setattr(ProcessControllerRuntime,'__init__',yaw_owner)
    with Peer(FlightChannel()) as peer:
        owned=Owner(tmp_path,monkeypatch,peer)
        try:
            owned.takeoff()
            with server(tmp_path/'http',runtime_factory=lambda *a,**kw:owned.runtime) as (base,app):
                for sequence in range(3):
                    window=json.loads(raw_request(base,'/api/remote-flight/window')[2])
                    code,_,body=raw_request(base,'/api/remote-flight/intent',payload={
                        'action':'control','parameters':{'lease':owned.lease,'sequence':sequence,
                            'channels':[0,0,0,200]},'ingress':window_ingress(window),'round_trip_seconds':.6})
                    result=json.loads(body)
                    assert code==202,result.get('error')
                    assert result['status']['state']=='flying'
                    assert result['status']['telemetry']['position_valid'] is False
                    time.sleep(.6)
                assert len(peer.controls)>=12
                status=owned.runtime.status()
                assert status['state']=='flying' and status['remote_control']['active']
                window=json.loads(raw_request(base,'/api/remote-flight/window')[2])
                assert raw_request(base,'/api/remote-flight/intent',payload={'action':'land',
                    'parameters':{},'ingress':window_ingress(window),'round_trip_seconds':.6})[0]==202
                assert not owned.runtime.status()['remote_control']['active']
                owned.drive(lambda s:s['telemetry']['flying'] is False and s['telemetry']['motors_on'] is False,renew=False)
                owned.ground()
        finally:owned.cleanup()
