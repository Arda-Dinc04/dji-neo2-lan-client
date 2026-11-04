"""Actual OS isolation and real UDP; aircraft, BLE and NM are fixtures."""
import json
from pathlib import Path
import signal
import subprocess
import time

import pytest

from neo2_control.control_owner import ProcessControllerRuntime
from neo2_control.controller import IntentRejected
from neo2_control.flight_channel import FlightChannel
from neo2_control.flight_connection import FlightConfig
from neo2_control.duml_stream import DumlStreamDecoder
from neo2_control.wlm_discovery import unpack_forwarded
from test_control_transport_session import ControllerPeer
from test_wlm_liveview import IDENTITY


def log(path):
    try:return [json.loads(line) for line in path.read_text().splitlines()]
    except (FileNotFoundError,json.JSONDecodeError):return []


class Owner:
    def __init__(self,tmp_path,monkeypatch,peer,*,duration=20,camera_preview=False):
        original=subprocess.Popen
        fixture=Path(__file__).with_name('flight_owner_fixture.py')
        def spawn(command,**kwargs):
            command=list(command)
            index=command.index('-m')
            assert command[index+1]=='neo2_control.control_owner'
            command[index:index+2]=[str(fixture)]
            command.append(str(peer.sock.getsockname()[1]))
            return original(command,**kwargs)
        monkeypatch.setattr('neo2_control.control_owner.subprocess.Popen',spawn)
        identity=tmp_path/'wlm.json';identity.write_text(json.dumps({
            'format':'neo2-control/wlm-identity/v1','identification_id':IDENTITY.identification_id}))
        self.journal=tmp_path/'owner.jsonl'
        self.config=FlightConfig('fixture0',identity,tmp_path/'ble.json',duration=duration,
                                capture_directory=tmp_path/'sessions',camera_preview=camera_preview,
                                incident_directory=tmp_path/'incidents')
        self.runtime=ProcessControllerRuntime(self.journal,mode='flight',flight_config=self.config)
        self.lease=None

    def drive(self,predicate,timeout=6,renew=True):
        end=time.monotonic()+timeout;last=0.
        while time.monotonic()<end:
            status=self.runtime.status()
            if self.lease and renew and status['state'] in ('armed','taking_off','flying') and time.monotonic()-last>=.1:
                try:
                    self.runtime.handle('keepalive',{'lease':self.lease})
                except IntentRejected as error:
                    # The independent watchdog can revoke authority after
                    # the status read. That exact rejection is expected in
                    # loss scenarios; other errors still fail this driver.
                    status=self.runtime.status()
                    if str(error)!='operator_lease_invalid' or status['state'] not in ('landing','fault'):
                        raise
                last=time.monotonic()
            if predicate(status):return status
            time.sleep(.01)
        pytest.fail(f'loopback owner did not reach state: {self.runtime.status()}')

    def ground(self):
        token=self.runtime.handle('prepare',{'operation':'ground'})['confirmation_token']
        self.runtime.handle('confirm_ground',{'confirmation_token':token})

    def takeoff(self):
        self.runtime.handle('open')
        self.drive(lambda s:s.get('telemetry') is not None and s['telemetry']['control_ready'])
        self.ground()
        token=self.runtime.handle('prepare',{'operation':'arm'})['confirmation_token']
        self.lease=self.runtime.handle('arm',{'confirmation_token':token})['lease']
        token=self.runtime.handle('prepare',{'operation':'takeoff','lease':self.lease})['confirmation_token']
        self.runtime.handle('takeoff',{'confirmation_token':token,'lease':self.lease})
        self.drive(lambda s:s['state']=='flying')

    def cleanup(self):
        try:self.runtime.close()
        finally:
            # This helper owns a deliberately localhost-only fixture child.
            # Resume it if a test failed mid-suspension; production never kills
            # an uncertain owner to manufacture cleanup.
            if self.runtime.process.poll() is None:
                self.runtime.process.send_signal(signal.SIGCONT)
                self.runtime.process.kill();self.runtime.process.wait(timeout=2)


class SparseDiscoveryPeer(ControllerPeer):
    def __init__(self,channel):
        super().__init__(channel);self.pause_discovery=False

    def beacon_bytes(self):
        payload=super().beacon_bytes()
        if not self.pause_discovery:return payload
        packets=[]
        for outer,_ in DumlStreamDecoder().feed(payload):
            inner=unpack_forwarded(outer.payload).packet
            if (inner.cmd_set,inner.cmd_id)!=(0x51,0x13):packets.append(outer.encode())
        return b''.join(packets)


def test_owner_lands_on_sparse_discovery_then_requires_ground_before_rearming(tmp_path,monkeypatch):
    with SparseDiscoveryPeer(FlightChannel()) as peer:
        owner=Owner(tmp_path,monkeypatch,peer)
        try:
            owner.takeoff();peer.pause_discovery=True
            owner.drive(lambda s:s['state']=='landing',timeout=4)
            owner.drive(lambda s:any(e['event']=='action_response' and e.get('request_action')=='land'
                                    for e in s['events']),renew=False)
            assert owner.runtime.status()['possible_airborne']
            # Measured aircraft pauses last6-9s while FC data continue.
            time.sleep(4.1)
            peer.pause_discovery=False
            status=owner.drive(lambda s:s.get('telemetry') is not None and s['telemetry']['control_ready'],renew=False)
            assert status['telemetry']['grounded'] is None
            assert status['possible_airborne']
            assert [p.payload for p in peer.flight_frames if (p.cmd_set,p.cmd_id)==(3,0x2a)]==[b'\1',b'\2']
            owner.ground();owner.drive(lambda s:not s['possible_airborne'],renew=False)
            assert owner.runtime.status()['state']=='fault'
            owner.runtime.handle('reset');assert owner.runtime.status()['state']=='ready'
            owner.runtime.handle('close');owner.runtime.close()
            assert owner.runtime.cleanup_verified
        finally:owner.cleanup()


@pytest.mark.parametrize('observed_action_replies',[False,True])
def test_independent_flight_owner_drives_native_wire_and_cleans_ble_wifi_socket(tmp_path,monkeypatch,observed_action_replies):
    with ControllerPeer(FlightChannel(),observed_action_replies=observed_action_replies,
                        direct_motion=observed_action_replies) as peer:
        owner=Owner(tmp_path,monkeypatch,peer)
        try:
            owner.takeoff()
            status=owner.runtime.status()
            assert status['controller_owner']['scope']=='flight' and status['controller_owner']['independent_watchdog']
            assert not status['synthetic'] and status['telemetry']['synthetic'] is False
            owner.runtime.handle('control',{'lease':owner.lease,'sequence':1,'channels':[50,0,0,0],
                'expires_at':time.monotonic()+.2})
            owner.drive(lambda s:(1057,1024,1024,1024) in peer.controls,timeout=1)
            owner.runtime.handle('land')
            owner.drive(lambda s:s['telemetry']['motors_on'] is False and any(
                e['event']=='action_response' and e.get('request_action')=='land' for e in s['events']))
            owner.ground()
            owner.drive(lambda s:s['state']=='ready')
            owner.runtime.handle('close')
            owner.runtime.close()
            assert owner.runtime.cleanup_verified and owner.runtime.final_received
            assert owner.runtime.last_status['connection']['cleanup_verified']
            assert owner.runtime.last_status['connection']['cleanup']=={'ble':True,'wifi':True,'socket':True}
            assert owner.runtime.last_status['landing_confirmed'] and not owner.runtime.last_status['possible_airborne']
            actions=[p for p in peer.flight_frames if (p.cmd_set,p.cmd_id)==(3,0x2a)]
            assert [p.payload for p in actions]==[b'\1',b'\2']
            assert all((p.cmd_set,p.cmd_id) not in ((0x51,0x15),(0x51,0x1a)) for p in peer.requests)
        finally:owner.cleanup()


def test_parent_timeout_preserves_uncertain_flight_and_resource_cleanup_evidence(tmp_path,monkeypatch):
    # The deliberately short fixture connection ends by its own duration.
    # No operator ground statement is fabricated after parent suspension.
    with ControllerPeer(FlightChannel()) as peer:
        owner=Owner(tmp_path,monkeypatch,peer,duration=12)
        try:
            owner.takeoff()
            owner.runtime.stopped.set();owner.runtime.thread.join(timeout=1)
            end=time.monotonic()+11
            while time.monotonic()<end and owner.runtime.process.poll() is None:time.sleep(.025)
            assert owner.runtime.process.poll()==0
            events=log(owner.journal)
            assert any(e['event']=='flight_owner_retaining_connection' for e in events)
            final=events[-1]
            assert final['event']=='controller_owner_stopped' and final['cleanup_verified']
            assert final['possible_airborne'] and not final['landing_confirmed']
            assert final['fault_reason']=='controller_parent_timeout'
            assert [p.payload for p in peer.flight_frames if (p.cmd_set,p.cmd_id)==(3,0x2a)]==[b'\1',b'\2']
            owner.runtime.status()
            assert owner.runtime.final_received
        finally:owner.cleanup()
