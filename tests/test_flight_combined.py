"""One real localhost socket, child owner and HEVC decoder; no radio."""
from pathlib import Path
import struct
import subprocess
import time

from neo2_control.flight_channel import FlightChannel
from neo2_control.transport import _build_datagram,describe_transport
from test_control_transport_session import ControllerPeer
from test_flight_owner import Owner


class CombinedPeer(ControllerPeer):
    def __init__(self,elementary):
        super().__init__(FlightChannel())
        self.blocks=[elementary[i:i+1452] for i in range(0,len(elementary),1452)]
        self.next_block=0
        self.last_video=0.
        self.frontier=None
        self.last_raw=None

    def receive_idle(self):
        # Aircraft telemetry is periodic, independent of operator/ACK traffic.
        # Do not model a quiet controller socket as a quiet aircraft.
        if self.last_raw is None:return
        outgoing=self.beacon_bytes()
        if outgoing:
            for packet in self.response_datagrams(self.last_raw,outgoing):
                self.sock.sendto(packet,self.peer_address)

    def response_datagrams(self,raw,outgoing):
        self.last_raw=raw
        packets=[]
        now=time.monotonic()
        reported=describe_transport(raw)[0].get('video_window_start')
        if self.frontier is None:self.frontier=self.anchor
        if reported is not None and ((reported-self.frontier)&65535)<32768:self.frontier=reported
        if self.completed and self.next_block<len(self.blocks) and now-self.last_video>=.12:
            self.last_video=now
            end=min(self.next_block+2,len(self.blocks))
            for i in range(self.next_block,end):
                payload=struct.pack('<HHHHI',(self.anchor+self.next_block*8)&65535,
                    (self.anchor+end*8)&65535,0,0,0x101)+self.blocks[i]
                packets.append(_build_datagram(self.session,2,payload,
                    send_sequence=(self.anchor+(i+1)*8)&65535))
            self.next_block=end
        packet=bytearray(super().response_datagrams(raw,outgoing)[0])
        for offset in (8,10):packet[offset:offset+2]=self.frontier.to_bytes(2,'little')
        return packets+[bytes(packet)]


def test_combined_camera_video_and_flight_share_owner_and_reap_actual_codecs(tmp_path,monkeypatch):
    elementary=subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-f','lavfi',
        '-i','testsrc2=size=128x96:rate=8','-frames:v','120','-c:v','libx265','-preset','ultrafast',
        '-x265-params','pools=none:frame-threads=1:wpp=0:keyint=8:log-level=error',
        '-f','hevc','pipe:1'],check=True,capture_output=True,timeout=15).stdout
    assert len(elementary)>32768
    with CombinedPeer(elementary) as peer:
        owner=Owner(tmp_path,monkeypatch,peer,camera_preview=True)
        try:
            owner.runtime.handle('open')
            status=owner.drive(lambda s:s.get('connection',{}).get('video',{}).get('live'),timeout=10)
            root=Path(owner.config.capture_directory)/status['connection']['generation']
            assert (root/'hls'/'preview.jpg').read_bytes().startswith(b'\xff\xd8')
            assert status['connection']['camera']['ready']
            owner.runtime.camera_command('recenter')
            owner.drive(lambda s:bool(peer.camera))
            assert (peer.camera[-1].cmd_set,peer.camera[-1].cmd_id)==(4,0x14)
            assert owner.runtime.status()['connection']['generation']==status['connection']['generation']
            owner.ground()
            token=owner.runtime.handle('prepare',{'operation':'arm'})['confirmation_token']
            owner.lease=owner.runtime.handle('arm',{'confirmation_token':token})['lease']
            token=owner.runtime.handle('prepare',{'operation':'takeoff','lease':owner.lease})['confirmation_token']
            owner.runtime.handle('takeoff',{'confirmation_token':token,'lease':owner.lease})
            owner.drive(lambda s:s['state']=='flying')
            owner.runtime.handle('land')
            owner.drive(lambda s:s['telemetry']['motors_on'] is False and any(
                e['event']=='action_response' and e.get('request_action')=='land' for e in s['events']))
            owner.ground();owner.drive(lambda s:s['state']=='ready')
            owner.runtime.handle('close');owner.runtime.close()
            assert owner.runtime.cleanup_verified
            result=owner.runtime.status()['connection']
            assert result['cleanup']=={'ble':True,'wifi':True,'socket':True,'video':True}
            assert len(peer.allocated)==len(set(peer.allocated))
            assert not any(command in (0x15,0x1a) for command,kind in peer.handshake)
        finally:owner.cleanup()
