const $ = id => document.getElementById(id);
const token = document.querySelector('meta[name="neo2-token"]').content;
let networkGeneration=null;
let current = null, frameVersion = null, frameObjectUrl = null, working = false, refreshing = false;
const messages = {camera_video_unavailable:'La vidéo n’a pas pu démarrer. Déconnecte puis reconnecte le drone.',camera_link_interrupted:'La liaison caméra a été interrompue.',camera_setup_incomplete:'La préparation de la caméra n’a pas abouti.',camera_session_changed:'La session a changé. Renvoie la commande.',camera_busy:'Arrête l’enregistrement avant de prendre une photo.',camera_connection_failed:'La connexion a échoué.',camera_cleanup_unconfirmed:'La fermeture de la connexion doit être vérifiée.',drone_not_connected:'Connecte d’abord le drone.',camera_not_ready:'La caméra se prépare.',command_pending:'Une commande est déjà en cours.',command_expired:'La commande a expiré.',setup_timeout:'Le drone ne répond pas. Vérifie qu’il est allumé et libre.',peer_beacon_expired:'La connexion au drone a été perdue.'};
function message(text, error=false){$('message').textContent=text||'';$('message').classList.toggle('error',error);}
async function api(path, body){const response=await fetch(path,{method:body===undefined?'GET':'POST',headers:{'X-Neo2-Token':token,...(body===undefined?{}:{'Content-Type':'application/json'})},...(body===undefined?{}:{body:JSON.stringify(body)}),cache:'no-store',signal:AbortSignal.timeout(5000)});if(response.status===401){location.reload();throw new Error('La page se recharge.');}const data=await response.json();if(!response.ok)throw new Error(messages[data.error]||data.message||data.error||'La demande a échoué.');return data;}
async function send(action,parameters={}){if(working)return;working=true;try{const result=await api('/api/camera/command',{action,parameters});message(result.message||'Commande envoyée.');}catch(error){message(error.message,true);}finally{working=false;await refresh();}}
function drawPitch(value){$('pitch-label').textContent=`${Math.round(value)}°`;$('camera-direction').style.transform=`rotate(${-value}deg)`;}
function render(s){current=s;const connected=s.connected===true;const busy=['connecting','stopping'].includes(s.state);$('link-state').textContent=connected?'Connecté':s.state==='connecting'?'Connexion…':s.state==='stopping'?'Fermeture…':'Déconnecté';const dot=document.createElement('i');$('link-state').prepend(dot);$('link-state').classList.toggle('connected',connected);$('connect').hidden=connected||s.state==='stopping';$('connect').disabled=busy;$('disconnect').hidden=!connected&&s.state!=='connecting';$('disconnect').disabled=s.state==='stopping';$('session-state').textContent=s.state_label||s.state;$('video-state').textContent=s.video?.live?'En direct':['waiting_for_codec_config','waiting_for_keyframe'].includes(s.video?.state)?'Attente d’une image complète':s.frame?.available?'Dernière image':'En attente';$('image-state').textContent=s.video?.live?'● EN DIRECT':s.frame?.available?'DERNIÈRE CAPTURE':'EN ATTENTE';$('image-state').classList.toggle('live',s.video?.live===true);$('caption').textContent=s.video?.live?'Image reçue du Neo 2':s.frame?.available?'Image conservée · le retour direct est inactif':'Aucune image reçue';$('image-detail').textContent=s.video?.resolution||'Caméra Neo 2';$('battery').textContent=Number.isFinite(s.battery_percent)?`Batterie ${s.battery_percent} %`:'Batterie —';const caps=s.capabilities||{};const allowed=connected&&s.camera_ready===true;for(const id of ['pitch','tilt-up','tilt-down'])$(id).disabled=!(allowed&&caps.pitch);$('recenter').disabled=!(allowed&&caps.recenter);$('photo').disabled=!(allowed&&caps.photo)||s.recording===true;$('record').disabled=!(allowed&&caps.record);$('stop-record').disabled=!(allowed&&caps.stop_record);$('record').hidden=s.recording===true;$('stop-record').hidden=!connected||!caps.stop_record;$('record-state').hidden=s.recording!==true;if(s.pitch_limits){$('pitch').min=s.pitch_limits.minimum;$('pitch').max=s.pitch_limits.maximum;$('pitch-min').textContent=`${s.pitch_limits.minimum}°`;$('pitch-max').textContent=`${s.pitch_limits.maximum}°`;}if(Number.isFinite(s.commanded_pitch)&&document.activeElement!==$('pitch')){$('pitch').value=s.commanded_pitch;drawPitch(s.commanded_pitch);}$('pitch-note').textContent=allowed?'Angle demandé · la réception vidéo permet de vérifier le cadrage.':'Commande disponible après connexion.';$('last-command').textContent=s.last_command?.label||s.last_command?.state||'—';if(s.error)message(messages[s.error]||s.error,true);}
async function frame(s){if(!s.frame?.available||s.frame.version===frameVersion)return;const response=await fetch('/api/camera/frame',{headers:{'X-Neo2-Token':token},cache:'no-store',signal:AbortSignal.timeout(5000)});if(!response.ok)return;const next=URL.createObjectURL(await response.blob());$('preview').src=next;$('preview').hidden=false;$('empty-preview').hidden=true;if(frameObjectUrl)URL.revokeObjectURL(frameObjectUrl);frameObjectUrl=next;frameVersion=s.frame.version;}
async function refresh(){if(refreshing)return;refreshing=true;try{const s=await api('/api/camera/status');render(s);applyPreviewOnly(s);await updateNetworkInstructions(s);await frame(s);}catch(error){message(error.message,true);}finally{refreshing=false;$('clock').textContent=new Date().toLocaleTimeString('fr-FR');}}
$('connect').addEventListener('click',async()=>{message('Connexion au drone…');try{await api('/api/camera/connect',{});}catch(error){message(error.message,true);}await refresh();});
$('disconnect').addEventListener('click',async()=>{try{await api('/api/camera/disconnect',{});message('Fermeture de la connexion…');}catch(error){message(error.message,true);}await refresh();});
$('pitch').addEventListener('input',()=>drawPitch(Number($('pitch').value)));
$('pitch').addEventListener('change',()=>send('pitch',{degrees:Number($('pitch').value)}));
for(const [id,delta] of [['tilt-up',10],['tilt-down',-10]])$(id).addEventListener('click',()=>{const value=Math.max(Number($('pitch').min),Math.min(Number($('pitch').max),Number($('pitch').value)+delta));$('pitch').value=value;drawPitch(value);send('pitch',{degrees:value});});
$('recenter').addEventListener('click',()=>send('recenter'));$('photo').addEventListener('click',()=>send('photo'));$('record').addEventListener('click',()=>send('record'));$('stop-record').addEventListener('click',()=>send('stop_record'));
await refresh();setInterval(refresh,250);

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
  $('caption').textContent=s.video?.live?'Live aircraft image':s.frame?.available?'Last received image; stream inactive':'Waiting for aircraft camera';
  document.querySelector('.session-card .eyebrow').textContent='LOCAL SESSION';
  const rows=document.querySelectorAll('.session-card dl>div');
  rows[0].querySelector('dt').textContent='Connection';rows[1].querySelector('dt').textContent='Video';rows[2].style.display='none';
  if(s.session_setup_authorized===false)message('Camera session setup awaits your approval.');
  else if(s.verification_pending===true)message('CONFIRM NOW: press and hold the aircraft power button, then release when it acknowledges.');
  else if($('message').textContent.startsWith('CONFIRM NOW:'))message(s.error?'The aircraft did not approve this request.':'Connection confirmation received; preparing camera network.');
}

async function updateNetworkInstructions(s){
  if(s.preview_only!==true||!s.session_generation)return;
  let card=document.getElementById('wifi-join');
  if(!card){
    card=document.createElement('section');card.id='wifi-join';card.className='session-card';card.style.display='none';
    const title=document.createElement('h2');title.textContent='Join aircraft Wi-Fi';
    const note=document.createElement('p');note.textContent='Join this network in macOS Wi-Fi settings. Keep this page open; the 60-second capture starts after the network is verified. Return to your normal Wi-Fi afterward.';
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
