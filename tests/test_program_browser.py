"""Real Chromium, HTTP, IPC and UDP loopback. BLE/NM, video and yaw are fixtures."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import pytest

from neo2_control.flight_channel import FlightChannel
from test_cloud_mission_wire import ProgramPeer
from test_flight_owner import Owner,log
from test_flight_web import server


@pytest.mark.skipif(not all(shutil.which(name) for name in ('node','chromium','ffmpeg')),
    reason='optional real browser fixture needs Node.js, Chromium and FFmpeg')
def test_browser_executes_three_step_program_and_retains_ground_confirmation(tmp_path,monkeypatch):
    if subprocess.check_output(['node','-p','typeof WebSocket'],text=True).strip()!='function':
        pytest.skip('Node.js with built-in WebSocket required')
    image=tmp_path/'fixture.jpg'
    subprocess.run(['ffmpeg','-v','error','-f','lavfi','-i','color=c=0x18212f:s=640x360',
        '-frames:v','1','-threads','1',str(image)],check=True,timeout=10)
    jpeg=image.read_bytes()
    profile=tmp_path/'browser';profile.mkdir()
    original_popen=subprocess.Popen
    browser=original_popen(['chromium','--headless=new','--no-sandbox','--disable-gpu',
        '--disable-background-networking','--disable-component-update','--disable-sync',
        '--no-first-run','--no-default-browser-check','--remote-debugging-address=127.0.0.1',
        '--remote-debugging-port=0','--user-data-dir='+str(profile),'about:blank'],
        stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
    try:
        deadline=time.monotonic()+10
        while not (profile/'DevToolsActivePort').exists():
            if browser.poll() is not None:pytest.fail('fixture browser stopped')
            if time.monotonic()>deadline:pytest.fail('fixture browser debugger not ready')
            time.sleep(.05)
        port=(profile/'DevToolsActivePort').read_text().splitlines()[0]
        with ProgramPeer(FlightChannel()) as peer:
            owned=Owner(tmp_path,monkeypatch,peer,duration=40)
            try:
                owned.runtime.handle('open')
                owned.drive(lambda s:s.get('telemetry') is not None and s['telemetry']['control_ready'],renew=False)
                actual_status=owned.runtime.status
                def fixture_status():
                    status=actual_status();status['synthetic']=True
                    status['connection']['video']={'live':True,'preview_available':True,
                        'last_encoded_age_seconds':0.,'preview_age_seconds':0.,'fixture_readiness':True}
                    return status
                monkeypatch.setattr(owned.runtime,'status',fixture_status)
                with server(tmp_path/'http',runtime_factory=lambda *a,**kw:owned.runtime) as (url,app):
                    frame_sequence=0
                    def fixture_frame():
                        nonlocal frame_sequence
                        frame_sequence+=1
                        return jpeg,{'X-Neo2-Connection-Generation':actual_status()['connection']['generation'],
                            'X-Neo2-Frame-Age-Seconds':'0','X-Neo2-Frame-Id':'fixture-'+str(frame_sequence)}
                    monkeypatch.setattr(app,'read_frame',fixture_frame)
                    driver=original_popen(['node',str(Path(__file__).with_name('browser_program_probe.cjs')),
                        port,url,str(tmp_path/'program-browser.png')],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
                    try:stdout,stderr=driver.communicate(timeout=30)
                    except subprocess.TimeoutExpired:
                        driver.kill();stdout,stderr=driver.communicate();pytest.fail(stderr)
                    assert driver.returncode==0,stderr
                    result=json.loads(stdout)
                    status=owned.drive(lambda s:s['telemetry']['motors_on'] is False,renew=False)
                    assert status['possible_airborne'] is True
                    report=status['remote_control']['program']
                    assert report['phase']=='motion_complete' and report['completed_steps']==3
                    assert report['last_rotation']['progress_degrees']>=90
                    events=log(owned.journal)
                    actions=[e['action'] for e in events if e['event']=='intent_received']
                    assert actions.count('takeoff')==1
                    assert sum(e['event']=='motion_program_started' for e in events)==1
                    assert actions.count('confirm_ground')==1 and 'confirm_landing' not in actions
                    assert any(c[0]>1024 for c in peer.controls) and any(c[1]>1024 for c in peer.controls)
                    assert any(c[3]>1024 for c in peer.controls)
                    (tmp_path/'program-browser-result.json').write_text(json.dumps(result,indent=2))
                    owned.ground();owned.drive(lambda s:not s['possible_airborne'],renew=False)
            finally:owned.cleanup()
    finally:
        if browser.poll() is None:
            os.killpg(browser.pid,signal.SIGTERM)
            try:browser.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(browser.pid,signal.SIGKILL);browser.wait(timeout=2)
