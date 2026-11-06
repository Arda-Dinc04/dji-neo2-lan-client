import {WhepPreview} from './webrtc.js';
const $ = id => document.getElementById(id);
let token = document.querySelector('meta[name="neo2-token"]').content;
let networkGeneration=null;
let current = null, frameVersion = null, frameObjectUrl = null, working = false, refreshing = false;
let lastStatusAt = null, jpegFrameAt = null, displayedGeneration = null;
const playback = new WhepPreview({video: $('webrtc-preview'), onChange: () => updatePresentation()});
const messages = {camera_video_unavailable:'Video could not start. Disconnect and reconnect the aircraft.',camera_link_interrupted:'The camera link was interrupted.',camera_setup_incomplete:'Camera setup did not complete.',camera_session_changed:'The session changed. Send the command again.',camera_busy:'Stop recording before taking a photo.',camera_connection_failed:'Connection failed.',camera_cleanup_unconfirmed:'Connection cleanup needs verification.',drone_not_connected:'Connect the aircraft first.',camera_not_ready:'The camera is preparing.',command_pending:'A command is already in progress.',command_expired:'The command expired.',setup_timeout:'The aircraft is not responding. Check that it is powered on and available.',peer_beacon_expired:'The aircraft connection was lost.'};
function message(text, error=false){$('message').textContent=text||'';$('message').classList.toggle('error',error);}
async function api(path, body){const response=await fetch(path,{method:body===undefined?'GET':'POST',headers:{'X-Neo2-Token':token,...(body===undefined?{}:{'Content-Type':'application/json'})},...(body===undefined?{}:{body:JSON.stringify(body)}),cache:'no-store',signal:AbortSignal.timeout(5000)});if(response.status===401){location.reload();throw new Error('The page is reloading.');}const data=await response.json();if(!response.ok)throw new Error(messages[data.error]||data.message||data.error||'The request failed.');return data;}
async function send(action,parameters={}){if(working)return;working=true;try{const result=await api('/api/camera/command',{action,parameters});message(result.message||'Command sent.');}catch(error){message(error.message,true);}finally{working=false;await refresh();}}
function drawPitch(value){$('pitch-label').textContent=`${Math.round(value)}°`;$('camera-direction').style.transform=`rotate(${-value}deg)`;}
function render(s){current=s;const connected=s.connected===true;const busy=['connecting','stopping'].includes(s.state);$('link-state').textContent=connected?'Connected':s.state==='connecting'?'Connecting…':s.state==='stopping'?'Closing…':'Disconnected';const dot=document.createElement('i');$('link-state').prepend(dot);$('link-state').classList.toggle('connected',connected);$('connect').hidden=connected||s.state==='stopping';$('connect').disabled=busy;$('disconnect').hidden=!connected&&s.state!=='connecting';$('disconnect').disabled=s.state==='stopping';$('session-state').textContent=s.state_label||s.state;$('image-detail').textContent=s.video?.resolution||'Neo 2 camera';$('battery').textContent=Number.isFinite(s.battery_percent)?`Battery ${s.battery_percent} %`:'Battery —';const caps=s.capabilities||{};const allowed=connected&&s.camera_ready===true;for(const id of ['pitch','tilt-up','tilt-down'])$(id).disabled=!(allowed&&caps.pitch);$('recenter').disabled=!(allowed&&caps.recenter);$('photo').disabled=!(allowed&&caps.photo)||s.recording===true;$('record').disabled=!(allowed&&caps.record);$('stop-record').disabled=!(allowed&&caps.stop_record);$('record').hidden=s.recording===true;$('stop-record').hidden=!connected||!caps.stop_record;$('record-state').hidden=s.recording!==true;if(s.pitch_limits){$('pitch').min=s.pitch_limits.minimum;$('pitch').max=s.pitch_limits.maximum;$('pitch-min').textContent=`${s.pitch_limits.minimum}°`;$('pitch-max').textContent=`${s.pitch_limits.maximum}°`;}if(Number.isFinite(s.commanded_pitch)&&document.activeElement!==$('pitch')){$('pitch').value=s.commanded_pitch;drawPitch(s.commanded_pitch);}$('pitch-note').textContent=allowed?'Requested angle · use the video preview to check framing.':'Control available after connection.';$('last-command').textContent=s.last_command?.label||s.last_command?.state||'—';if(s.error)message(messages[s.error]||s.error,true);}
function syncPlayback(s){
  const freshStatus = lastStatusAt != null && performance.now()-lastStatusAt <= 3000;
  playback.update({enabled:s?.webrtc?.enabled===true,ready:s?.webrtc?.ready===true,
    generation:s?.webrtc?.generation,sessionGeneration:s?.session_generation,
    connected:s?.connected===true&&freshStatus,live:s?.video?.live===true&&freshStatus,
    error:s?.webrtc?.error,token});
}
function updatePresentation(){
  const s=current;
  const state=playback.snapshot;
  const backendLive=s?.connected===true&&s?.video?.live===true&&lastStatusAt!=null&&performance.now()-lastStatusAt<=3000;
  const rtcLive=backendLive&&state.fresh;
  const jpegLive=backendLive&&jpegFrameAt!=null&&performance.now()-jpegFrameAt<=3000&&!!frameObjectUrl;
  const live=rtcLive||jpegLive;
  const synthetic=s?.source_mode==='synthetic';
  $('webrtc-preview').hidden=!rtcLive;
  $('preview').hidden=rtcLive||!frameObjectUrl;
  $('empty-preview').hidden=rtcLive||!!frameObjectUrl;
  $('image-state').textContent=live?(synthetic?'● LIVE · SYNTHETIC':'● LIVE'):frameObjectUrl?'LAST IMAGE':'WAITING';
  $('image-state').classList.toggle('live',live);
  $('source-state').hidden=!synthetic;
  const media=rtcLive?$('webrtc-preview'):$('preview');
  const width=media.videoWidth||media.naturalWidth,height=media.videoHeight||media.naturalHeight;
  $('image-detail').textContent=(synthetic?'Synthetic test':'Neo 2 camera')+(width&&height?` · ${width} × ${height}`:'');
  document.querySelector('.viewer-caption .dot').classList.toggle('live',live);
  $('video-state').textContent=live?'Live':frameObjectUrl?'Last image':['waiting_for_codec_config','waiting_for_keyframe'].includes(s?.video?.state)?'Waiting for complete frame':'Waiting';
  const source=synthetic?'Synthetic video test':'Aircraft camera';
  $('caption').textContent=live?`${source} · ${rtcLive?'WebRTC':'JPEG'}`:frameObjectUrl?'Last received image · live playback inactive':'Waiting for camera video';
  $('transport-state').textContent=rtcLive?'WebRTC':jpegLive?'JPEG':'—';
  const reasons={unsupported:'WebRTC unavailable in this browser',stalled:'WebRTC stalled',no_frame:'No WebRTC frame received',ice_failed:'WebRTC network connection failed',connection_failed:'WebRTC connection interrupted',track_ended:'WebRTC video ended',playback_blocked:'Browser blocked video playback',ice_gather_timeout:'WebRTC connection timed out',offer_rejected:'WebRTC session unavailable',negotiation_failed:'WebRTC negotiation failed',offer_invalid:'WebRTC offer unavailable',answer_invalid:'WebRTC answer unavailable',answer_too_large:'WebRTC answer rejected'};
  let detail;
  if(rtcLive)detail='WebRTC · smooth playback';
  else if(playback.mode==='jpeg')detail=jpegLive?'JPEG · live frames':'JPEG · waiting for a fresh frame';
  else if(state.state==='connecting')detail=jpegLive?'JPEG live · connecting WebRTC…':'Connecting WebRTC…';
  else if(state.state==='fallback')detail=`${reasons[state.reason]||'WebRTC unavailable'} · JPEG${state.attempts>=3?' · retry limit reached':''}`;
  else if(jpegLive)detail=s?.webrtc?.enabled?'JPEG live · waiting for WebRTC':'JPEG · live frames';
  else detail=frameObjectUrl?'JPEG · last image':'Waiting for video';
  if($('playback-status').textContent!==detail)$('playback-status').textContent=detail;
}
async function frame(s){
  if(playback.snapshot.fresh||!s.frame?.available||s.frame.version===frameVersion)return;
  const generation=s.session_generation;
  const response=await fetch('/api/camera/frame',{headers:{'X-Neo2-Token':token},cache:'no-store',signal:AbortSignal.timeout(5000)});
  if(!response.ok)return;
  const next=URL.createObjectURL(await response.blob());
  try{
    const candidate=new Image();
    if(candidate.decode){candidate.src=next;await candidate.decode();}
    else await new Promise((resolve,reject)=>{candidate.onload=resolve;candidate.onerror=reject;candidate.src=next;});
    if(current?.session_generation!==generation){URL.revokeObjectURL(next);return;}
    $('preview').src=next;
    if(frameObjectUrl)URL.revokeObjectURL(frameObjectUrl);
    frameObjectUrl=next;frameVersion=s.frame.version;jpegFrameAt=performance.now();
    updatePresentation();
  }catch(error){URL.revokeObjectURL(next);throw error;}
}
async function refresh(){
  if(refreshing)return;
  refreshing=true;
  try{
    const s=await api('/api/camera/status');lastStatusAt=performance.now();
    if(displayedGeneration!==s.session_generation){
      displayedGeneration=s.session_generation;frameVersion=null;jpegFrameAt=null;
      if(frameObjectUrl){URL.revokeObjectURL(frameObjectUrl);frameObjectUrl=null;$('preview').removeAttribute('src');}
    }
    render(s);applyPreviewOnly(s);syncPlayback(s);updatePresentation();
    await updateNetworkInstructions(s);await frame(s);
  }catch(error){lastStatusAt=null;syncPlayback(current);updatePresentation();message(error.message,true);}
  finally{refreshing=false;$('clock').textContent=new Date().toLocaleTimeString('en-US');}
}
$('playback-mode').addEventListener('change',event=>{playback.setMode(event.target.value);updatePresentation();void refresh();});
$('connect').addEventListener('click',async()=>{message('Connecting to aircraft…');try{await api('/api/camera/connect',{});}catch(error){message(error.message,true);}await refresh();});
$('disconnect').addEventListener('click',async()=>{try{await api('/api/camera/disconnect',{});message('Closing the connection…');}catch(error){message(error.message,true);}await refresh();});
$('pitch').addEventListener('input',()=>drawPitch(Number($('pitch').value)));
$('pitch').addEventListener('change',()=>send('pitch',{degrees:Number($('pitch').value)}));
for(const [id,delta] of [['tilt-up',10],['tilt-down',-10]])$(id).addEventListener('click',()=>{const value=Math.max(Number($('pitch').min),Math.min(Number($('pitch').max),Number($('pitch').value)+delta));$('pitch').value=value;drawPitch(value);send('pitch',{degrees:value});});
$('recenter').addEventListener('click',()=>send('recenter'));$('photo').addEventListener('click',()=>send('photo'));$('record').addEventListener('click',()=>send('record'));$('stop-record').addEventListener('click',()=>send('stop_record'));
await refresh();
const refreshTimer=setInterval(refresh,250);
const presentationTimer=setInterval(()=>{
  const latestToken=document.querySelector('meta[name="neo2-token"]').content;
  if(latestToken!==token)token=latestToken;
  syncPlayback(current);updatePresentation();
},250);
window.addEventListener('pagehide',()=>{clearInterval(refreshTimer);clearInterval(presentationTimer);playback.stop();if(frameObjectUrl)URL.revokeObjectURL(frameObjectUrl);},{once:true});

function applyPreviewOnly(s){
  if(s.preview_only!==true)return;
  document.title='Neo 2 camera preview';
  document.querySelector('.capture-bar').style.display='none';
  document.querySelector('.control-card').style.display='none';
  document.querySelector('h1').textContent='Camera preview';
  document.querySelector('#empty-preview h2').textContent='Neo 2 camera';
  document.querySelector('#empty-preview p').textContent='No camera frame received yet.';
  $('connect').textContent='Connect camera';$('disconnect').textContent='Disconnect';
  $('connect').disabled=s.session_setup_authorized===false||!['disconnected','failed'].includes(s.state)||s.cleanup_verified!==true;
  const labels={disconnected:'Disconnected',connecting:'Connecting',connected:'Connected',stopping:'Closing',failed:'Connection failed'};
  const dot=document.createElement('i');$('link-state').replaceChildren(dot,document.createTextNode(labels[s.state]||'Waiting'));
  $('session-state').textContent=labels[s.state]||'Waiting';
  document.querySelector('.session-card .eyebrow').textContent='LOCAL SESSION';
  const rows=document.querySelectorAll('.session-card dl>div');
  rows[0].querySelector('dt').textContent='Connection';rows[1].querySelector('dt').textContent='Video';rows[2].style.display='none';
  if(s.session_setup_authorized===false)message('Camera session setup awaits your approval.');
  else if(s.verification_pending===true)message('CONFIRM NOW: press and hold the aircraft power button, then release when it acknowledges.');
  else if(s.warnings?.includes('peer_beacon_expired_while_video_live'))message('Camera stream available. Device status updates are delayed.');
  else if(s.video?.live===true&&!s.error)message(s.source_mode==='synthetic'?'Synthetic video test · local fixture.':'Camera stream available.');
  else if($('message').textContent.startsWith('CONFIRM NOW:'))message(s.error?'The aircraft did not approve this request.':'Connection confirmation received; preparing camera network.');
}

async function updateNetworkInstructions(s){
  if(s.preview_only!==true||!s.session_generation)return;
  let card=document.getElementById('wifi-join');
  if(!card){
    card=document.createElement('section');card.id='wifi-join';card.className='session-card';card.style.display='none';
    const title=document.createElement('h2');title.textContent='Join aircraft Wi-Fi';
    const note=document.createElement('p');note.textContent='Keep this page open during the preview. The local coordinator can join this network and restore your usual Wi-Fi afterward. An independent internet connection lets the chat stay online.';
    const label=document.createElement('p');label.textContent='Network';
    const name=document.createElement('strong');name.id='join-ssid';
    const keyLabel=document.createElement('p');keyLabel.textContent='Wi-Fi password';
    const key=document.createElement('input');key.id='join-key';key.type='password';key.readOnly=true;key.autocomplete='off';key.spellcheck=false;key.style.cssText='width:100%;box-sizing:border-box;padding:10px;margin-bottom:10px';
    const reveal=document.createElement('button');reveal.id='join-reveal';reveal.textContent='Show password';reveal.onclick=()=>{key.type=key.type==='password'?'text':'password';reveal.textContent=key.type==='password'?'Show password':'Hide password';};
    card.append(title,note,label,name,keyLabel,key,reveal);document.querySelector('.controls').append(card);
  }
  if(networkGeneration!==s.session_generation){card.style.display='none';$('join-key').value='';$('join-ssid').textContent='';$('join-key').type='password';$('join-reveal').textContent='Show password';networkGeneration=null;}
  if(s.connected){card.style.display='none';$('join-key').type='password';$('join-reveal').textContent='Show password';return;}
  if(networkGeneration===s.session_generation){card.style.display='block';return;}
  const response=await fetch('/api/camera/network',{headers:{'X-Neo2-Token':token},cache:'no-store',signal:AbortSignal.timeout(3000)});
  if(!response.ok)return;
  const details=await response.json();
  if(details.session_generation!==s.session_generation||typeof details.ssid!=='string'||typeof details.password!=='string')return;
  $('join-ssid').textContent=details.ssid;$('join-key').value=details.password;networkGeneration=s.session_generation;card.style.display='block';
  message(s.error==='manually_join_aircraft_wifi_before_session'?'Join the network shown, then click Connect camera again.':'Connection approved. Join the aircraft Wi-Fi shown on this page.');
}
