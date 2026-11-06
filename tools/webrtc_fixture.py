#!/usr/bin/env python3
"""Serve generated video through the real WebRTC pipeline; never connect an aircraft."""
import argparse
import json
import os
from pathlib import Path
import secrets
import selectors
import shutil
import signal
import subprocess
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from neo2_control.camera_web import serve_camera_web
from neo2_control.process_limits import VideoProcessLimits,signal_owned_child
from neo2_control.video import HlsVideo
from neo2_control.webrtc_bridge import WebRtcBridge


class FixtureConsole:
    def __init__(self,directory,*,duration=60,kill_bridge_after=None):
        self.directory=directory;self.stop=threading.Event();self.closed=False
        self.bridge=self.video=self.producer=self.worker=self.timer=None
        try:self._start(directory,duration=duration,kill_bridge_after=kill_bridge_after)
        except BaseException:
            self.close()
            raise

    def _start(self,directory,*,duration,kill_bridge_after):
        self.directory=directory;directory.mkdir(mode=0o700,parents=True)
        self.generation=secrets.token_hex(12);self.stop=threading.Event()
        self.bridge=WebRtcBridge(directory/'bridge');self.bridge.start()
        self.video=HlsVideo(directory/'hls',input_codec='hevc',preview_jpeg=True,
                            rtsp_output=self.bridge.rtsp_url)
        self.video.__enter__()
        self.limits=VideoProcessLimits(linux_limiter=shutil.which('prlimit'))
        command=[shutil.which('ffmpeg'),'-hide_banner','-loglevel','error','-nostdin',
            '-re','-f','lavfi','-i','testsrc2=size=640x360:rate=30','-t',str(duration),
            '-c:v','libx265','-preset','ultrafast','-threads','1',
            '-x265-params','pools=none:frame-threads=1:wpp=0:log-level=error:bframes=0:keyint=30:scenecut=0',
            '-f','hevc','pipe:1']
        self.producer=subprocess.Popen(self.limits.wrap(command),stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,stdin=subprocess.DEVNULL,start_new_session=True)
        self.worker=threading.Thread(target=self._feed,daemon=True);self.worker.start()
        self.timer=None
        if kill_bridge_after is not None:
            self.timer=threading.Timer(kill_bridge_after,self.bridge.close)
            self.timer.daemon=True;self.timer.start()

    def _feed(self):
        selector=selectors.DefaultSelector()
        os.set_blocking(self.producer.stdout.fileno(),False)
        selector.register(self.producer.stdout,selectors.EVENT_READ)
        try:
            while not self.stop.is_set():
                self.limits.check_memory([self.producer])
                for _,_ in selector.select(.1):
                    chunk=os.read(self.producer.stdout.fileno(),65536)
                    if not chunk:return
                    if not self.video.feed(chunk):return
        finally:selector.close()

    def status(self):
        video=self.video.snapshot();path=self.frame_path()
        live=bool(video.get('last_encoded_age_seconds') is not None
            and video['last_encoded_age_seconds']<=2 and video.get('preview_age_seconds') is not None
            and video['preview_age_seconds']<=3 and not self.stop.is_set())
        bridge=self.bridge.snapshot();bridge['ready']=bool(bridge['ready'] and live)
        return {'state':'connected' if not self.stop.is_set() else 'disconnected',
            'state_label':'Synthetic video test','source_mode':'synthetic','session_generation':self.generation,
            'connected':not self.stop.is_set(),'preview_only':True,'session_setup_authorized':False,
            'verification_pending':False,'cleanup_verified':False,'camera_ready':False,
            'frame':{'available':path is not None,'version':str(path.stat().st_mtime_ns) if path else None,'kind':'session'},
            'video':{'live':live,'frames_encoded':video['frames_encoded'],'state':video['state']},
            'webrtc':bridge,'battery_percent':None,'capabilities':{},'pitch_limits':None,
            'commanded_pitch':None,'recording':False,'last_command':None,
            'error':video['error'],'warnings':[],'flight_commands_available':False}

    def frame_path(self):
        path=self.directory/'hls/preview.jpg'
        return path if path.is_file() else None
    def network_instructions(self):return None
    def webrtc_offer(self,sdp):return self.bridge.offer(sdp)
    def webrtc_delete(self,session):self.bridge.delete(session)
    def connect(self):raise RuntimeError('synthetic_fixture_has_no_aircraft_connection')
    def command(self,*args):raise RuntimeError('synthetic_fixture_has_no_aircraft_commands')
    def disconnect(self):self.stop.set();return {'accepted':True}

    def close(self):
        if self.closed:return
        self.closed=True
        self.stop.set()
        if self.timer:self.timer.cancel()
        if self.producer is not None and self.producer.poll() is None:
            signal_owned_child(self.producer,signal.SIGTERM)
            try:self.producer.wait(timeout=2)
            except subprocess.TimeoutExpired:
                signal_owned_child(self.producer,signal.SIGKILL);self.producer.wait(timeout=2)
        if self.worker is not None and self.worker.ident is not None:self.worker.join(2)
        if self.producer is not None:self.producer.stdout.close()
        try:
            if self.video is not None:self.video.close()
        finally:
            bridge=self.bridge.close() if self.bridge is not None else {'cleanup_verified':True}
        result={'source_mode':'synthetic','video':self.video.snapshot() if self.video is not None else
                    {'cleanup_verified':True,'frames_encoded':0},
                'bridge':bridge,'producer_cleanup_verified':self.producer is None or self.producer.poll() is not None}
        if self.directory.is_dir():
            fd=os.open(self.directory/'fixture-result.json',os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
            with os.fdopen(fd,'w') as output:json.dump(result,output,indent=2);output.write('\n')
        print(json.dumps({'fixture_finished':True,'encoded_frames':result['video']['frames_encoded'],
            'video_cleanup_verified':result['video']['cleanup_verified'],
            'bridge_cleanup_verified':result['bridge']['cleanup_verified']}),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=int,default=60)
    parser.add_argument('--port',type=int,default=8794)
    parser.add_argument('--kill-bridge-after',type=int)
    args=parser.parse_args()
    if not 5<=args.duration<=180:parser.error('duration must be5..180seconds')
    directory=ROOT/'private/macos/webrtc-fixtures'/secrets.token_hex(8)
    console=FixtureConsole(directory,duration=args.duration,kill_bridge_after=args.kill_bridge_after)
    timer=threading.Timer(args.duration,lambda:os.kill(os.getpid(),signal.SIGTERM));timer.daemon=True
    timer.start()
    try:serve_camera_web(console,port=args.port)
    finally:timer.cancel()


if __name__=='__main__':main()
