const $ = id => document.getElementById(id);
const token = document.querySelector('meta[name="neo2-token"]').content;
const configuredPrefix = document.querySelector('meta[name="neo2-api-prefix"]')?.content;
const apiPrefix = typeof configuredPrefix==='string'&&/^\/(?:[a-zA-Z0-9_-]+\/)*[a-zA-Z0-9_-]+$/.test(configuredPrefix)?configuredPrefix:'/api/flight';
const authorization = document.querySelector('meta[name="neo2-auth"]')?.content==='bearer'
  ? {Authorization:'Bearer '+token} : {'X-Neo2-Token':token};
const remoteTargets = document.querySelector('meta[name="neo2-command-mode"]')?.content==='remote-target';
let nextWindow=null;
const roundTrips=[];
const remoteRoundTrip=()=>roundTrips.length?Math.max(...roundTrips):null;
const evidenceLimit=()=>remoteTargets?Math.min(2,.5+(remoteRoundTrip()||0)):.5;
let current = {}, lease = null, sequence = 0, held = false, renewing = false;
let refreshing = false, frameBusy = false, frameUrl = null, challenge = null, confirmationTimer;
let frameAt = -Infinity, frameGeneration = null;
let frameAgeSeconds = Infinity, frameId = null, statusAt = -Infinity;
let landingConfirmationBusy = false;
let recoveringOwner = false, operatorContext = 0;
let mutation = null;
let loadedProgram=null,programError='',programStarting=false,activeProgramId=null,programFileSerial=0;
const keys = new Set();
const airborne = () => ['taking_off', 'flying'].includes(current.state);
const recoveryPeerMatches = () => current.recovery?.required!==true || current.recovery.peer_match===true;
const canRequestLand = () => !recoveringOwner && current.controller_owner?.available!==false && recoveryPeerMatches() && (airborne() || (!!current.possible_airborne && current.connected===true && ['landing','fault'].includes(current.state)));
const confirmationFresh = () => ['flag','motion','control','ready'].every(name=>statusAge(current.landing_confirmation?.component_age_seconds?.[name])<=evidenceLimit())&&statusAge(current.landing_confirmation?.component_age_seconds?.route)<2;
const canConfirmLanding = () => !recoveringOwner && current.controller_owner?.available!==false && recoveryPeerMatches() && confirmationFresh() && current.landing_confirmation?.available===true && current.landing_confirmation?.needed===true && current.connected===true && current.possible_airborne===true && ['landing','fault'].includes(current.state);
const focused = () => document.visibilityState === 'visible' && document.hasFocus();
const liveImage = () => !!current.connection?.video?.live && statusAge(current.connection?.video?.last_encoded_age_seconds)<=2 && statusAge(current.connection?.video?.preview_age_seconds)<=3 && frameGeneration===current.connection?.generation && frameAgeSeconds+(performance.now()-frameAt)/1000<3;
const states = {disconnected:'Au repos', not_ready:'Préparation', ready:'Prêt', armed:'Essai autorisé', taking_off:'Décollage', flying:'En vol', landing:'Atterrissage', fault:'Vérification nécessaire'};
const reasons = {ground_state_unconfirmed:'Confirme visuellement que le drone est posé, moteurs arrêtés.', ground_confirmation_unavailable:'Attends les données du drone avant de confirmer le sol.', camera_not_ready:'La caméra se prépare.', command_pending:'Une commande caméra est déjà en cours.', invalid_state:'L’état du drone a changé. Vérifie la page.', operator_lease_invalid:'L’autorisation a expiré. Prépare un nouvel essai.', confirmation_invalid_or_expired:'La confirmation a expiré.', drone_or_wifi_interface_already_owned:'Une autre page utilise le drone. Déconnecte-la avant de continuer.', controller_owner_unavailable:'Le contrôleur ne répond plus.', camera_busy:'Arrête l’enregistrement avant une nouvelle capture.'};
function message(text, error=false) { $('message').textContent=text; $('message').classList.toggle('error',error); }
function obsoleteMutation(){const error=new Error('La connexion a changé. Vérifie l’état du drone avant une nouvelle demande.');error.name='ObsoleteMutation';return error;}
function statusAge(value){const elapsed=(performance.now()-statusAt)/1000;return Number.isFinite(value)&&value>=0&&Number.isFinite(elapsed)&&elapsed>=0?value+elapsed:Infinity;}
function conservativeStatus(data,roundTrip){
  const age=value=>Number.isFinite(value)&&value>=0?value+roundTrip:null;
  const t=data.telemetry,c=data.connection,proof=data.landing_confirmation;
  const video=c?.video?{...c.video,last_encoded_age_seconds:age(c.video.last_encoded_age_seconds),preview_age_seconds:age(c.video.preview_age_seconds)}:undefined;
  const confirmationAges={};
  for(const [name,field] of [['flag','received_at'],['motion','motion_received_at'],['control','control_received_at'],['route','route_seen_at'],['ready','ready_at']]){
    const stamp=proof?.[field];confirmationAges[name]=Number.isFinite(data.now)&&Number.isFinite(stamp)?age(data.now-stamp):null;
  }
  return {...data,_client_received_at:performance.now(),
    ...(t?{telemetry:{...t,age_seconds:age(t.age_seconds),component_age_seconds:Object.fromEntries(Object.entries(t.component_age_seconds||{}).map(([key,value])=>[key,age(value)]))}}:{}),
    ...(c?{connection:{...c,...(video?{video}:{})}}:{}),
    ...(proof?{landing_confirmation:{...proof,component_age_seconds:confirmationAges}}:{})};
}
async function jsonRequest(path,body,context,signal){
  const start=performance.now();
  const r=await fetch(apiPrefix+'/'+path,{method:body===undefined?'GET':'POST',headers:{...authorization,...(body===undefined?{}:{'Content-Type':'application/json'})},...(body===undefined?{}:{body:JSON.stringify(body)}),cache:'no-store',signal:signal||AbortSignal.timeout(1500)});
  if(r.status===401)throw new Error('Recharge la page pour reprendre la session.');
  const data=await r.json();
  if(context!==operatorContext)throw obsoleteMutation();
  if(!r.ok)throw new Error(reasons[data.error]||'La demande n’a pas abouti. Vérifie l’état du drone et renouvelle la préparation si nécessaire.');
  const elapsed=(performance.now()-start)/1000;
  if(!Number.isFinite(elapsed)||elapsed<0)throw new Error('La durée de la demande est invalide.');
  if(remoteTargets){roundTrips.push(elapsed);if(roundTrips.length>8)roundTrips.shift();}
  const result=path==='status'?conservativeStatus(data,elapsed):data.status?{...data,status:conservativeStatus(data.status,elapsed)}:data;
  return remoteTargets&&body!==undefined?{...result,_request_started_at:start}:result;
}
async function api(path,body,{background=false}={}){
  const context=operatorContext;
  if(body===undefined)return jsonRequest(path,body,context);
  const expectedOwnerEpoch=current.controller_owner?.epoch,expectedConnectionGeneration=current.connection?.generation;
  if(typeof expectedOwnerEpoch!=='string'||!/^[0-9a-f]{48}$/.test(expectedOwnerEpoch)||(expectedConnectionGeneration!==null&&(typeof expectedConnectionGeneration!=='string'||!/^[0-9a-f]{24}$/.test(expectedConnectionGeneration))))throw new Error('Attends un état valide du contrôleur avant une nouvelle demande.');
  if(mutation){
    const preparation=path==='intent'&&['prepare','takeoff','disarm','program_start'].includes(body.action);
    const preemptBackground=!background&&preparation&&mutation.background&&['control','keepalive'].includes(mutation.action);
    if(preemptBackground||(path==='intent'&&body.action==='land'&&(mutation.path==='camera'||['control','keepalive','program_start','program_keepalive'].includes(mutation.action)))){mutation.controller.abort();mutation=null;}
    else throw new Error('Une demande est déjà en cours. Vérifie son résultat avant une nouvelle action.');
  }
  const item={path,action:body.action,context,background,controller:new AbortController()};mutation=item;
  const signal=AbortSignal.any([item.controller.signal,AbortSignal.timeout(1500)]);
  try{
    let issuedAfter=performance.now(),window;
    if(remoteTargets&&nextWindow&&
        (issuedAfter-nextWindow.issuedAfter)/1000+(remoteRoundTrip()||0)+.2<nextWindow.value.expires_in_seconds){
      ({value:window,issuedAfter}=nextWindow);nextWindow=null;
    }else{
      nextWindow=null;window=await jsonRequest('window',undefined,context,signal);
    }
    if(mutation!==item||context!==operatorContext)throw obsoleteMutation();
    if(typeof window.ticket!=='string'||!window.ticket||!Number.isSafeInteger(window.mutation_sequence)||window.mutation_sequence<1||!Number.isFinite(window.expires_in_seconds)||window.expires_in_seconds<=0||window.expires_in_seconds>(remoteTargets?2:.25))throw new Error('La demande ne peut pas être autorisée. Vérifie la connexion.');
    if(window.owner_epoch!==expectedOwnerEpoch||window.connection_generation!==expectedConnectionGeneration){invalidateOperatorContext();throw obsoleteMutation();}
    if(performance.now()-issuedAfter>=window.expires_in_seconds*1000)throw new Error('La demande a expiré avant son envoi. Vérifie la connexion.');
    const result=await jsonRequest(path,{...body,...(remoteTargets?{round_trip_seconds:remoteRoundTrip()}:{}),ingress:{ticket:window.ticket,mutation_sequence:window.mutation_sequence,expected_owner_epoch:expectedOwnerEpoch,expected_connection_generation:expectedConnectionGeneration}},context,signal);
    if(mutation!==item||context!==operatorContext)throw obsoleteMutation();
    if(remoteTargets&&result.next_window)nextWindow={value:result.next_window,issuedAfter:result._request_started_at};
    return result;
  }catch(error){if(mutation!==item||context!==operatorContext)throw obsoleteMutation();throw error;}
  finally{if(mutation===item)mutation=null;}
}
const intent=async(action,parameters={},options)=>{const result=await api('intent',{action,parameters},options);if(remoteTargets&&result.status)render(result.status);return result;};
async function perform(fn) {try {await fn();}catch(e){if(e.name!=='ObsoleteMutation')message(e.message,true);}finally{await refresh();}}
function cancelConfirmation(){challenge=null;clearTimeout(confirmationTimer);$('takeoff-dialog').close();}
function invalidateOperatorContext(){
  nextWindow=null;
  operatorContext++;lease=null;sequence=0;held=false;activeProgramId=null;programStarting=false;keys.clear();cancelConfirmation();
  if(mutation){mutation.controller.abort();mutation=null;}
  $('deadman').textContent='Espace relâchée';$('deadman').classList.remove('active');$('confirm-takeoff').disabled=true;
  if(frameUrl)URL.revokeObjectURL(frameUrl);
  frameUrl=null;frameAt=-Infinity;frameAgeSeconds=Infinity;frameId=null;frameGeneration=null;$('preview').src='';$('preview').hidden=true;$('empty-preview').hidden=false;
}
async function requestOwnerRecovery(){
  if(recoveringOwner||current.controller_owner?.restart_available!==true)return;
  recoveringOwner=true;invalidateOperatorContext();$('recover-owner').disabled=true;
  try{
    const result=await intent('recover_owner');
    if(result.status)render(result.status);
    message('La reprise est prête. Connecte le drone pour vérifier son état et demander l’atterrissage si nécessaire.');
  }catch(e){message(e.message,true);}finally{recoveringOwner=false;await refresh();}
}
async function requestConnectionRetry(){
  if(recoveringOwner||current.controller_owner?.available===false||current.recovery?.connection_retry_available!==true)return;
  recoveringOwner=true;invalidateOperatorContext();$('retry-connection').disabled=true;
  try{
    await intent('reconnect');
    message('Reconnexion demandée pour vérifier le drone. Aucun mouvement ne sera repris.');
  }catch(e){message(e.message,true);}finally{recoveringOwner=false;await refresh();}
}
async function requestLand(){if(!canRequestLand())return;lease=null;activeProgramId=null;cancelConfirmation();try{const result=await intent('land');message(result.landing_already_pending?'La demande d’atterrissage est déjà en cours. Vérifie le drone.':'Atterrissage demandé. Vérifie le drone.');}catch(e){message(e.message,true);}await refresh();}
async function requestLandingConfirmation(){
  if(landingConfirmationBusy||!canConfirmLanding())return;
  landingConfirmationBusy=true;lease=null;held=false;keys.clear();cancelConfirmation();
  $('deadman').textContent='Espace relâchée';$('deadman').classList.remove('active');
  $('confirm-landing').disabled=true;
  const generation=current.connection?.generation;
  try{
    const prepared=await intent('prepare',{operation:'landing'});
    if(generation!==current.connection?.generation||!canConfirmLanding())throw new Error('L’état du drone a changé. Vérifie la demande d’atterrissage.');
    if(typeof prepared.confirmation_token!=='string'||!prepared.confirmation_token)throw new Error('La confirmation a expiré. Vérifie la demande du drone.');
    await intent('confirm_landing',{confirmation_token:prepared.confirmation_token});
    message('Confirmation d’atterrissage demandée. Vérifie le drone.');
  }catch(e){message(e.message,true);}finally{landingConfirmationBusy=false;await refresh();}
}
function release(){held=false;keys.clear();$('deadman').textContent='Espace relâchée';$('deadman').classList.remove('active');$('confirm-takeoff').disabled=true;if(airborne())void requestLand();}
function validateProgram(program){
  const fields=(value,names)=>value&&typeof value==='object'&&!Array.isArray(value)&&Object.keys(value).sort().join(',')===names.split(',').sort().join(',');
  const finite=(value,min,max)=>typeof value==='number'&&Number.isFinite(value)&&value>=min&&value<=max;
  if(!fields(program,'version,finish,steps')||program.version!==1||program.finish!=='land'||!Array.isArray(program.steps)||program.steps.length<1||program.steps.length>32)throw new Error('Programme attendu : version 1, finish « land », de 1 à 32 étapes.');
  let budget=0;const channels=[];
  for(const step of program.steps){
    const minimumChannel=step?.minimum_channel===undefined?40:step.minimum_channel;
    if(fields(step,'kind,seconds')&&step.kind==='hold'&&finite(step.seconds,.2,5)){
      budget+=step.seconds+.1;channels.push([0,0,0,0]);
    }else if(fields(step,'kind,seconds,right,forward,up')&&step.kind==='translate'&&finite(step.seconds,.2,5)){
      const axes=[step.right,step.forward,step.up];
      if(!axes.every(value=>Number.isInteger(value)&&Math.abs(value)<=250)||!axes.some(Boolean))throw new Error('Translation : au moins un axe non nul, entier entre −250 et 250.');
      budget+=step.seconds+.1;channels.push([...axes,0]);
    }else if((fields(step,'kind,degrees,channel,timeout_seconds')||fields(step,'kind,degrees,channel,timeout_seconds,minimum_channel'))&&step.kind==='rotate'&&finite(step.degrees,5,360)&&Number.isInteger(step.channel)&&finite(Math.abs(step.channel),40,600)&&finite(step.timeout_seconds,.2,25)&&Number.isInteger(minimumChannel)&&finite(minimumChannel,40,Math.abs(step.channel))){
      budget+=step.timeout_seconds+.1;channels.push([0,0,0,step.channel]);
    }else throw new Error('Étape invalide : hold, translate ou rotate avec les champs et limites prévus.');
  }
  if(budget>30)throw new Error('Le budget total du programme dépasse 30 secondes.');
  return {program,budget,channels};
}
function programCompatible(){
  if(!loadedProgram||current.vertical_trial===true)return false;
  const maximum=current.limits?.maximum_channel;
  return Number.isInteger(maximum)&&maximum>0&&loadedProgram.channels.every(axes=>axes.every(value=>Math.abs(value)<=maximum)&&(current.yaw_trial!==true||axes.slice(0,3).every(value=>value===0)));
}
function canStartProgram(){
  const available=!mutation||(mutation.background&&['control','keepalive'].includes(mutation.action));
  return remoteTargets&&programCompatible()&&!programStarting&&!activeProgramId&&available&&current.remote_control?.program?.phase!=='running'&&!!lease&&held&&focused()&&liveImage()&&current.connected===true&&current.state==='flying'&&current.controller_owner?.available!==false&&current.recovery?.required!==true&&statusAge(current.telemetry?.component_age_seconds?.motion)<=evidenceLimit()&&statusAge(current.telemetry?.component_age_seconds?.control)<=evidenceLimit()&&statusAge(current.telemetry?.component_age_seconds?.battery)<=3;
}
function updateProgramInput(){
  loadedProgram=null;programError='';
  const text=$('program-json').value;
  if(typeof text==='string'&&text.trim()){
    try{
      if(new Blob([text]).size>8192)throw new Error('Le programme dépasse 8 192 octets.');
      loadedProgram=validateProgram(JSON.parse(text));
    }catch(error){programError=error instanceof SyntaxError?'JSON invalide. Vérifie le programme.':error.message;}
  }
  renderProgram();
}
async function loadProgramFile(){
  const serial=++programFileSerial,file=$('program-file').files?.[0];
  if(!file)return;
  try{
    if(file.size>8192)throw new Error('Le programme dépasse 8 192 octets.');
    const text=await file.text();
    if(serial!==programFileSerial)return;
    if(airborne()||programStarting||activeProgramId)throw new Error('Charge le fichier avant le vol.');
    $('program-json').value=text;updateProgramInput();
  }catch(error){if(serial===programFileSerial){loadedProgram=null;programError=error.message;renderProgram();}}
}
function renderProgram(){
  $('program-card').hidden=!remoteTargets;
  const report=current.remote_control?.program;
  if(activeProgramId&&(report?.program_id!==activeProgramId||report?.phase!=='running')){
    activeProgramId=null;
    if(airborne()){lease=null;release();}
  }
  const busy=programStarting||report?.phase==='running';
  $('program-json').readOnly=busy||airborne();$('program-file').disabled=busy||airborne();
  $('program-start').disabled=!canStartProgram();$('program-stop').disabled=!canRequestLand();
  $('program-summary').textContent=programError||(loadedProgram?`${loadedProgram.program.steps.length} étape(s) · budget maximal ${loadedProgram.budget.toFixed(1)} s · atterrissage demandé à la fin. Translations en consignes et durées, distances non étalonnées.`:'Translations en consignes et durées ; distances non étalonnées. Le parcours se termine par une demande d’atterrissage.');
  $('program-summary').classList.toggle('error',!!programError);
  $('program-progress').hidden=!report;
  if(report){
    $('program-progress').max=report.step_count||1;$('program-progress').value=report.completed_steps||0;
    const rotation=report.rotation||report.last_rotation;
    const angle=Number.isFinite(rotation?.progress_degrees)?` · rotation mesurée ${rotation.progress_degrees.toFixed(1)}° / ${rotation.target_degrees}°`:'';
    $('program-state').textContent=report.phase==='running'?`Parcours en cours · ${report.completed_steps} / ${report.step_count} étapes terminées${angle}`:report.phase==='motion_complete'?`Mouvements terminés${angle}. Atterrissage à vérifier.`:'Parcours interrompu. Vérifie le drone avant un nouvel essai.';
  }else $('program-state').textContent=programStarting?'Démarrage demandé…':loadedProgram?!programCompatible()?'Le programme dépasse les limites ou les axes autorisés pour cet essai.':canStartProgram()?'Prêt à démarrer le parcours.':'Programme chargé. Décollage séparé ; maintiens Espace en vol pour le démarrer.':programError?'Corrige le programme avant le vol.':'Aucun programme chargé.';
}
async function startProgram(){
  if(!canStartProgram())return;
  programStarting=true;renderProgram();
  try{
    const result=await intent('program_start',{lease,program:loadedProgram.program});
    if(typeof result.program_id!=='string'||!/^[0-9a-f]{24}$/.test(result.program_id))throw new Error('Le démarrage du parcours reste incertain. Atterrissage demandé.');
    activeProgramId=result.program_id;
    message('Parcours demandé. Maintiens Espace ; les touches de déplacement reprennent la main.');
  }catch(error){if(error.name!=='ObsoleteMutation'){lease=null;release();message(error.message,true);}}
  finally{programStarting=false;renderProgram();}
}
function render(input){
  const s=input._client_received_at===undefined?conservativeStatus(input,0):input;
  if(s.controller_owner?.epoch===current.controller_owner?.epoch&&s.connection?.generation===current.connection?.generation&&Number.isFinite(s.now)&&Number.isFinite(current.now)&&s.now<current.now)return;
  const ownerLost=s.controller_owner?.available===false||s.controller_owner?.restart_available===true;
  const previousOwnerLost=current.controller_owner?.available===false||current.controller_owner?.restart_available===true;
  if((ownerLost&&!previousOwnerLost)||(s.recovery?.required===true&&current.recovery?.required!==true)||s.controller_owner?.epoch!==current.controller_owner?.epoch||s.connection?.generation!==current.connection?.generation)invalidateOperatorContext();
  current=s;statusAt=s._client_received_at;const c=s.connection||{}, cam=c.camera||{}, video=c.video||{}, t=s.telemetry||{};
  const recovery=s.recovery?.required===true, peerMatches=recoveryPeerMatches();
  const yawOnly=s.yaw_trial===true,verticalOnly=s.vertical_trial===true;
  const flightLimit=Number.isFinite(s.limits?.maximum_flight_seconds)?s.limits.maximum_flight_seconds:verticalOnly?10:30;
  const altitudeLimit=Number.isFinite(s.limits?.maximum_altitude_m)?s.limits.maximum_altitude_m:yawOnly||verticalOnly?1.5:3;
  const distanceLimit=Number.isFinite(s.limits?.maximum_distance_m)?s.limits.maximum_distance_m:5;
  $('movement-guide').hidden=yawOnly||verticalOnly;$('vertical-guide').hidden=yawOnly||verticalOnly;$('yaw-guide').hidden=verticalOnly;
  $('pilot-help').textContent=yawOnly?'Rotation seule : maintiens Espace, puis A ou E pour tourner. Déplacements et montée/descente désactivés. Relâcher Espace demande un atterrissage.':verticalOnly?'Essai vertical : maintiens Espace. Les commandes de déplacement et de rotation sont désactivées. Relâcher Espace demande un atterrissage.':'Maintiens Espace pendant le vol. Relâcher, quitter l’onglet ou perdre la liaison demande un atterrissage.';
  $('trial-limits').textContent=`Limites logicielles de l’essai : ${flightLimit} s · altitude relative ${altitudeLimit} m`+(s.acquisition_trial===true?' · acquisition verticale et rotation':yawOnly?' · rotation seule':verticalOnly?' · vertical seul':` · distance ${distanceLimit} m`)+'. L’atterrissage reste à vérifier.';
  const active=!!c.worker_alive,fresh=statusAge(t.component_age_seconds?.motion)<=evidenceLimit(),controlFresh=statusAge(t.component_age_seconds?.control)<=evidenceLimit(),batteryFresh=statusAge(t.component_age_seconds?.battery)<=3;
  $('link-state').textContent=s.connected?'Connecté':active?'Connexion…':'Déconnecté';$('link-state').classList.toggle('connected',s.connected===true);
  $('connect').hidden=s.state!=='disconnected'||ownerLost;$('connect').disabled=active||recoveringOwner;
  $('connect').textContent=recovery?'Reconnecter pour vérifier le drone':'Connecter le drone';
  $('recover-owner').hidden=s.controller_owner?.restart_available!==true;$('recover-owner').disabled=recoveringOwner||s.controller_owner?.restart_available!==true;
  $('retry-connection').hidden=ownerLost||s.recovery?.connection_retry_available!==true;$('retry-connection').disabled=recoveringOwner||ownerLost||s.recovery?.connection_retry_available!==true;
  $('recovery-state').hidden=!recovery&&!ownerLost;
  $('recovery-state').textContent=ownerLost?(s.controller_owner?.restart_available===true?'Le contrôleur s’est arrêté. Reprends la connexion, puis vérifie le drone. Aucun mouvement ne sera repris.':'Le contrôleur ne répond plus. La reprise attend la confirmation de son arrêt. Vérifie le drone.'):
    s.recovery?.reason==='incident_storage_unavailable'?'La reprise ne peut pas vérifier le vol précédent. Les commandes restent bloquées. Vérifie le drone.':
    s.recovery?.peer_match===false?'Le drone connecté ne correspond pas au vol à vérifier. Les commandes restent bloquées.':
    s.connected&&!peerMatches?'La connexion vérifie encore le drone du vol précédent. Les commandes restent bloquées.':
    s.connected?'Vol précédent à vérifier : seuls l’atterrissage et la confirmation visuelle au sol sont disponibles.':'L’état du vol précédent reste inconnu. Reconnecte pour observer le drone et demander son atterrissage si nécessaire.';
  $('disconnect').hidden=s.state==='disconnected';$('disconnect').disabled=!!s.possible_airborne||airborne()||s.state==='landing';
  $('flight-state').textContent=(recovery||ownerLost)&&s.possible_airborne?'Vol à vérifier':states[s.state]||'Indisponible';
  $('battery').textContent=batteryFresh&&Number.isFinite(t.battery_percent)?`Batterie ${t.battery_percent} %`:'Batterie —';
  $('altitude').textContent=fresh&&Number.isFinite(t.altitude_m)?`${t.altitude_m.toFixed(1)} m`:'—';
  $('distance').textContent=fresh&&Number.isFinite(t.distance_m)?`${t.distance_m.toFixed(1)} m`:'—';
  $('ground-state').textContent=fresh&&t.grounded===true?'Confirmé':s.possible_airborne?'À vérifier':'À confirmer';
  $('image-state').textContent=s.synthetic?'SIMULATION LOCALE':liveImage()?'● EN DIRECT':frameUrl?'DERNIÈRE IMAGE':'EN ATTENTE';$('image-state').classList.toggle('live',liveImage());
  $('caption').textContent=s.synthetic?'Données de test locales':liveImage()?'Image reçue du Neo 2':frameUrl?'Image conservée · retour direct inactif':'Aucune image reçue';
  for(const [id,action] of [['pitch','pitch'],['recenter','recenter'],['photo','photo'],['record','record'],['stop-record','stop_record']])$(id).disabled=recovery||ownerLost||!controlFresh||!(cam.ready&&cam.capabilities?.[action])||!!cam.pending;
  $('photo').disabled||=cam.recording===true;$('record-state').hidden=cam.recording!==true;
  if(Number.isFinite(cam.commanded_pitch)&&document.activeElement!==$('pitch')){$('pitch').value=cam.commanded_pitch;$('pitch-label').textContent=`${cam.commanded_pitch}°`;}
  $('camera-command').textContent=cam.last_command?({sent:'Commande transmise',sent_unconfirmed:'Mouvement transmis · à vérifier sur l’image',acknowledged:'Réception confirmée',timed_out:'Délai de réponse dépassé',queued:'Commande en attente',scheduled:'Prête à envoyer',waiting_mode:'Préparation de la capture',rejected:'Commande refusée',expired:'Commande expirée',cancelled:'Commande annulée'}[cam.last_command.state]||'Commande en cours'):cam.ready?'Caméra disponible.':s.connected?'Préparation des commandes caméra…':'Commande disponible après connexion.';
  $('ground').disabled=ownerLost||!peerMatches||!(fresh&&t.motors_on===false&&t.flying===false&&['not_ready','ready','landing','fault'].includes(s.state));
  $('arm').disabled=recovery||ownerLost||!fresh||!controlFresh||!batteryFresh||s.state!=='ready';$('arm').hidden=s.state==='armed';$('disarm').hidden=s.state!=='armed';
  $('takeoff').disabled=recovery||ownerLost||!fresh||!controlFresh||!batteryFresh||s.state!=='armed'||!lease||!liveImage();$('land').disabled=ownerLost||!canRequestLand();$('reset').hidden=recovery||s.state!=='fault'||!!s.possible_airborne||t.grounded!==true;
  $('land').textContent=s.possible_airborne&&['landing','fault'].includes(s.state)?'↓ Demander à nouveau l’atterrissage':'↓ Atterrir';
  const landingConfirmation=s.landing_confirmation||{};
  $('confirm-landing').hidden=landingConfirmation.needed!==true;
  $('confirm-landing').disabled=landingConfirmationBusy||!canConfirmLanding();
  $('landing-confirmation-state').hidden=landingConfirmation.needed!==true;
  $('landing-confirmation-state').textContent=canConfirmLanding()?'Le drone attend ta confirmation pour finir sa descente. Vérifie que la zone sous lui est libre.':'La confirmation attend des données récentes du drone.';
  $('readiness').textContent=s.possible_airborne&&s.state==='fault'?'Atterrissage non confirmé. Si le drone vole encore et la liaison est active, demande à nouveau l’atterrissage. Confirme le sol seulement après son arrêt.':s.state==='armed'?'Maintiens Espace et confirme le décollage. L’autorisation expire après 5 secondes.':s.state==='ready'?'Données disponibles et sol confirmé. Tu peux autoriser un essai.':'Confirme visuellement que le drone est posé, moteurs arrêtés. Le décollage attend aussi une position valide et une image directe.';
  if(s.vertical_trial===true&&!(s.possible_airborne&&s.state==='fault'))$('readiness').textContent=`Essai vertical expérimental : commandes de déplacement désactivées. Limites logicielles de ${flightLimit} s et ${altitudeLimit} m ; l’atterrissage reste à vérifier. Confirme le sol et vérifie l’image directe.`;
  if(yawOnly&&!verticalOnly&&!(s.possible_airborne&&s.state==='fault'))$('readiness').textContent=`Essai de rotation seule : A/E pour tourner avec Espace maintenue. Déplacements et montée/descente désactivés. Limites logicielles de ${flightLimit} s et ${altitudeLimit} m ; l’atterrissage reste à vérifier. Confirme le sol et vérifie l’image directe.`;
  if(active&&c.control_setup?.state==='failed'){$('camera-command').textContent='Commandes indisponibles pour cette session.';$('readiness').textContent=s.possible_airborne?'La liaison de commande est indisponible. Utilise les commandes DJI habituelles pour atterrir. Confirme le sol après l’arrêt.':'La préparation des commandes n’a pas abouti. Vérifie le drone au sol avant une nouvelle connexion.';}
  if(recovery||ownerLost){lease=null;held=false;keys.clear();if(challenge)cancelConfirmation();$('readiness').textContent=$('recovery-state').textContent;$('camera-command').textContent='Commandes caméra suspendues pendant la vérification du drone.';}
  if(!['armed','taking_off','flying'].includes(s.state)){lease=null;if(challenge)cancelConfirmation();}
  renderProgram();
  if(c.error)message(reasons[c.error]||(s.possible_airborne?'La liaison est interrompue et l’atterrissage reste à vérifier.':'La connexion n’a pas abouti. Vérifie le drone au sol avant un nouvel essai.'),true);
}
async function refresh(){if(refreshing)return;refreshing=true;try{render(await api('status'));}catch(e){lease=null;release();message(e.message,true);}finally{refreshing=false;$('clock').textContent=new Date().toLocaleTimeString('fr-FR');}}
async function frame(){
  if(frameBusy||recoveringOwner||!current.connection?.video?.preview_available)return;
  frameBusy=true;const generation=current.connection.generation,context=operatorContext,start=performance.now();
  try{
    const r=await fetch(apiPrefix+'/frame',{headers:authorization,cache:'no-store',signal:AbortSignal.timeout(1500)});
    if(!r.ok||context!==operatorContext||generation!==current.connection?.generation)return;
    const servedGeneration=r.headers.get('X-Neo2-Connection-Generation'),ageHeader=r.headers.get('X-Neo2-Frame-Age-Seconds'),id=r.headers.get('X-Neo2-Frame-Id');
    const age=ageHeader===null||ageHeader.trim()===''?NaN:Number(ageHeader);
    if(servedGeneration!==generation||!Number.isFinite(age)||age<0||typeof id!=='string'||!id||id===frameId){await r.body?.cancel();return;}
    const next=URL.createObjectURL(await r.blob());
    await new Promise((resolve,reject)=>{const probe=new Image();probe.onload=resolve;probe.onerror=reject;probe.src=next;}).catch(e=>{URL.revokeObjectURL(next);throw e;});
    const received=performance.now(),elapsed=(received-start)/1000;
    if(context!==operatorContext||generation!==current.connection?.generation||!Number.isFinite(elapsed)||elapsed<0){URL.revokeObjectURL(next);return;}
    $('preview').src=next;$('preview').hidden=false;$('empty-preview').hidden=true;if(frameUrl)URL.revokeObjectURL(frameUrl);
    frameUrl=next;frameAt=received;frameAgeSeconds=age+elapsed;frameId=id;frameGeneration=generation;
  }catch{}finally{frameBusy=false;}
}
async function renew(){
  if(renewing||mutation||recoveringOwner||current.recovery?.required===true||!lease||!focused())return;
  if(statusAge(current.telemetry?.component_age_seconds?.motion)>evidenceLimit()||statusAge(current.telemetry?.component_age_seconds?.control)>evidenceLimit())return;
  if(!['armed','taking_off','flying'].includes(current.state))return;
  if(airborne()&&!held){void requestLand();return;}
  renewing=true;try{
    const manualKeys=['ArrowRight','ArrowLeft','ArrowUp','ArrowDown','KeyR','KeyF','KeyE','KeyA'].some(key=>keys.has(key));
    if(activeProgramId&&!manualKeys)await intent('program_keepalive',{lease,program_id:activeProgramId},{background:true});
    else if(current.state==='flying'&&held&&current.vertical_trial!==true){
      const axis=(a,b,amplitude=250)=>(Number(keys.has(a))-Number(keys.has(b)))*amplitude;
      const yawAmplitude=Number.isInteger(current.limits?.maximum_channel)&&current.limits.maximum_channel>0?Math.min(current.limits.maximum_channel,200):0;
      const channels=current.yaw_trial===true?[0,0,0,axis('KeyE','KeyA',yawAmplitude)]:[axis('ArrowRight','ArrowLeft'),axis('ArrowUp','ArrowDown'),axis('KeyR','KeyF'),axis('KeyE','KeyA')];
      activeProgramId=null;
      await intent('control',{lease,sequence:sequence++,channels},{background:true});
    }
    else await intent('keepalive',{lease},{background:true});
  }catch(e){if(e.name!=='ObsoleteMutation'){lease=null;release();message(e.message,true);}}finally{renewing=false;}
}
$('connect').onclick=()=>perform(async()=>{await intent('open');message('Connexion au drone…');});
$('recover-owner').onclick=requestOwnerRecovery;
$('retry-connection').onclick=requestConnectionRetry;
$('disconnect').onclick=()=>perform(async()=>{await intent('close');message('Fermeture de la connexion…');});
$('ground').onclick=()=>perform(async()=>{const p=await intent('prepare',{operation:'ground'});await intent('confirm_ground',{confirmation_token:p.confirmation_token});message('Observation au sol confirmée.');});
$('arm').onclick=()=>perform(async()=>{const p=await intent('prepare',{operation:'arm'});lease=(await intent('arm',{confirmation_token:p.confirmation_token})).lease;sequence=0;message('Essai autorisé pour 5 secondes.');});
$('disarm').onclick=()=>perform(()=>intent('disarm',{lease}));$('reset').onclick=()=>perform(()=>intent('reset'));
$('takeoff').onclick=()=>perform(async()=>{challenge=await intent('prepare',{operation:'takeoff',lease});$('takeoff-dialog').showModal();$('confirm-takeoff').disabled=!held;confirmationTimer=setTimeout(cancelConfirmation,5000);});
$('cancel-takeoff').onclick=cancelConfirmation;$('takeoff-dialog').addEventListener('cancel',cancelConfirmation);
$('confirm-takeoff').onclick=()=>perform(async()=>{if(!held||!focused()||!challenge||!liveImage())throw new Error('Maintiens Espace et vérifie le retour direct pour confirmer.');const p=challenge;cancelConfirmation();await intent('takeoff',{confirmation_token:p.confirmation_token,lease});message('Décollage demandé. Maintiens Espace.');});
$('land').onclick=requestLand;
$('confirm-landing').onclick=requestLandingConfirmation;
$('program-json').oninput=()=>{programFileSerial++;updateProgramInput();};$('program-file').onchange=loadProgramFile;
$('program-start').onclick=startProgram;$('program-stop').onclick=requestLand;
for(const [id,action] of [['recenter','recenter'],['photo','photo'],['record','record'],['stop-record','stop_record']])$(id).onclick=()=>perform(async()=>{await api('camera',{action,parameters:{}});message('Commande caméra demandée.');});
$('pitch').oninput=()=>{$('pitch-label').textContent=`${$('pitch').value}°`;};$('pitch').onchange=()=>perform(()=>api('camera',{action:'pitch',parameters:{degrees:Number($('pitch').value)}}));
window.addEventListener('keydown',e=>{if(e.code==='Escape'){release();cancelConfirmation();return;}if(e.target.matches('input,textarea,select'))return;if((current.yaw_trial===true||current.vertical_trial===true)&&['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','KeyR','KeyF',...(current.vertical_trial===true?['KeyA','KeyE']:[])].includes(e.code)){e.preventDefault();return;}if(['Space','ArrowUp','ArrowDown','ArrowLeft','ArrowRight','KeyR','KeyF','KeyA','KeyE'].includes(e.code)){e.preventDefault();keys.add(e.code);if(e.code==='Space'){held=true;$('deadman').textContent='Espace maintenue';$('deadman').classList.add('active');$('confirm-takeoff').disabled=!challenge;}}});
window.addEventListener('keyup',e=>{keys.delete(e.code);if(e.code==='Space'){e.preventDefault();release();}});
window.addEventListener('blur',release);document.addEventListener('visibilitychange',()=>{if(document.hidden)release();});
await refresh();setInterval(refresh,200);setInterval(frame,200);setInterval(renew,100);setInterval(()=>{if(current._client_received_at!==undefined)render(current)},100);
