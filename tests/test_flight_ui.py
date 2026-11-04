"""Flight-page decisions executed in JavaScript against a minimal DOM, no RF."""
import json
from html.parser import HTMLParser
from pathlib import Path
import shutil
import subprocess

import pytest


NODE=shutil.which('node')
pytestmark=pytest.mark.skipif(NODE is None,reason='optional JavaScript UI checks require Node.js')
SCRIPT=r'''
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const scenario=JSON.parse(fs.readFileSync(0,'utf8'));
const elements=new Map(),calls=[];
const confirmationScenario=scenario.startsWith('confirm_');
let finishPreparation,windowSequence=0;
function element(id){
  if(!elements.has(id))elements.set(id,{disabled:false,hidden:false,textContent:'',value:0,
    classList:{toggle(){},add(){},remove(){}},close(){},showModal(){},addEventListener(){}});
  return elements.get(id);
}
const status={state:'fault',connected:true,possible_airborne:true,synthetic:false,now:100,
  controller_owner:{epoch:'c'.repeat(48),available:true,restart_available:false},
  telemetry:{grounded:null,motors_on:true,flying:true,battery_percent:null,
    component_age_seconds:{motion:.1,battery:null,control:.1}},
  connection:{worker_alive:true,generation:'a'.repeat(24),control_setup:{state:'ready'},
    camera:{ready:false},video:{live:false,preview_available:false}}};
if(scenario==='fault_no_link'){status.connected=false;status.connection.worker_alive=false;}
if(scenario==='fault_grounded'){status.possible_airborne=false;status.telemetry.grounded=true;}
if(scenario==='landing_bound')status.state='landing';
if(['vertical_renew','ordinary_renew'].includes(scenario)){
  status.state='flying';status.vertical_trial=scenario==='vertical_renew';
}
if(scenario==='missing_motion'){
  status.telemetry.component_age_seconds.motion=null;
  status.telemetry.motors_on=false;status.telemetry.flying=false;
}
if(scenario==='setup_failed'){
  status.connected=false;status.connection.control_setup.state='failed';
  status.connection.error='command_setup_failed';
}
if(confirmationScenario){
  status.state=scenario==='confirm_fault'?'fault':'landing';
  status.landing_confirmation={available:true,needed:true,reason:null,received_at:99.9,
    motion_received_at:99.9,control_received_at:99.9,route_seen_at:99.9,ready_at:99.9};
  if(scenario==='confirm_absent')delete status.landing_confirmation;
  if(scenario==='confirm_false')status.landing_confirmation={available:false,needed:false,reason:null};
  if(scenario==='confirm_stale')status.landing_confirmation={available:false,needed:true,reason:'landing_status_stale'};
  if(scenario==='confirm_no_link')status.connected=false;
  if(scenario==='confirm_grounded'){
    status.possible_airborne=false;status.telemetry.grounded=true;
    status.telemetry.motors_on=status.telemetry.flying=false;
  }
  if(scenario==='confirm_wrong_state')status.state='flying';
}
const context={document:{getElementById:element,querySelector:()=>({content:'fixture-token'}),
  visibilityState:'visible',hasFocus:()=>true,addEventListener(){},activeElement:null},
  window:{addEventListener(){}},performance:{now:()=>0},AbortSignal,AbortController,
  setInterval(){},clearTimeout(){},setTimeout,URL,Image:class{},
  fetch:async(path,options={})=>{
    if(path.endsWith('/window'))return {status:200,ok:true,json:async()=>({ticket:'window-'+(++windowSequence),mutation_sequence:windowSequence,expires_in_seconds:.25,owner_epoch:status.controller_owner.epoch,connection_generation:status.connection.generation})};
    const body=options.body?JSON.parse(options.body):null;
    if(body){assert.equal(typeof body.ingress.ticket,'string');assert(Number.isSafeInteger(body.ingress.mutation_sequence));assert.equal(body.ingress.expected_owner_epoch,status.controller_owner.epoch);assert.equal(body.ingress.expected_connection_generation,status.connection.generation);calls.push(body);}
    if(confirmationScenario&&body?.action==='prepare'){
      const reply={status:200,ok:true,json:async()=>({confirmation_token:'fixture-landing-token'})};
      if(scenario==='confirm_prepare_error')return {status:409,ok:false,json:async()=>({error:'invalid_state'})};
      if(scenario==='confirm_missing_token')return {status:200,ok:true,json:async()=>({})};
      if(scenario==='confirm_double')return await new Promise(resolve=>{finishPreparation=()=>resolve(reply)});
      if(scenario==='confirm_generation_changed'){
        status.connection.generation='b'.repeat(24);context.ui.render(status);
      }
      if(scenario==='confirm_flag_changed'){
        status.landing_confirmation.available=false;context.ui.render(status);
      }
      return reply;
    }
    if(confirmationScenario&&body?.action==='confirm_landing'){
      status.landing_confirmation.available=false;
      if(scenario==='confirm_send_error')return {status:409,ok:false,json:async()=>({error:'invalid_state'})};
    }
    return {status:200,ok:true,json:async()=>path.endsWith('/status')?status:
      scenario==='landing_bound'?{landing_already_pending:true}:{}};
  }};
(async()=>{
  const source=fs.readFileSync(process.argv[1],'utf8');
  await vm.runInNewContext('(async()=>{'+source+
    ';globalThis.ui={render,requestLand,requestLandingConfirmation,release,renew,setLease:value=>{lease=value},setHeld:value=>{held=value}};})()',context);
  if(['vertical_renew','ordinary_renew'].includes(scenario)){
    context.ui.setLease('fixture-lease');context.ui.setHeld(true);await context.ui.renew();
    assert.deepEqual(calls.map(call=>call.action),[status.vertical_trial?'keepalive':'control']);
    if(status.vertical_trial)assert(/vertical/.test(element('readiness').textContent));
    console.log(JSON.stringify({scenario,flight_intents:calls.map(call=>call.action)}));return;
  }
  if(confirmationScenario){
    const disabled=['confirm_absent','confirm_false','confirm_stale','confirm_no_link','confirm_grounded','confirm_wrong_state'].includes(scenario);
    assert.equal(element('confirm-landing').disabled,disabled);
    assert.equal(element('confirm-landing').hidden,['confirm_absent','confirm_false'].includes(scenario));
    assert.equal(element('arm').disabled,true);assert.equal(element('takeoff').disabled,true);
    assert.equal(calls.length,0); // Loading/rendering never emits confirmation.
    if(scenario==='confirm_no_implicit'){
      for(let i=0;i<3;i++)context.ui.render(status);
      context.ui.release();context.ui.setLease('old-lease');await context.ui.renew();
      await new Promise(resolve=>setImmediate(resolve));
      assert.equal(calls.length,0);
    }else if(scenario==='confirm_double'){
      const first=context.ui.requestLandingConfirmation();
      await new Promise(resolve=>setImmediate(resolve));
      assert.equal(element('confirm-landing').disabled,true);
      await context.ui.requestLandingConfirmation();
      assert.deepEqual(calls.map(call=>call.action),['prepare']);
      finishPreparation();await first;
    }else await context.ui.requestLandingConfirmation();
    const preparationOnly=['confirm_prepare_error','confirm_missing_token','confirm_flag_changed','confirm_generation_changed'].includes(scenario);
    const expected=disabled||scenario==='confirm_no_implicit'?[]:
      preparationOnly?['prepare']:['prepare','confirm_landing'];
    assert.deepEqual(calls.map(call=>call.action),expected);
    if(calls.length)assert.deepEqual(calls[0].parameters,{operation:'landing'});
    if(calls.length===2){
      assert.deepEqual(calls[1].parameters,{confirmation_token:'fixture-landing-token'});
      assert.equal(element('confirm-landing').disabled,true);
      assert(!/posé|arrêté/i.test(element('message').textContent));
      assert.equal(status.possible_airborne,true);assert.equal(status.telemetry.flying,true);
    }
    if(!['confirm_no_link','confirm_grounded'].includes(scenario))assert.equal(element('land').disabled,false);
    assert(calls.every(call=>!('lease' in call.parameters)));
    console.log(JSON.stringify({scenario,flight_intents:calls.map(call=>call.action)}));return;
  }
  const enabled=!['fault_no_link','fault_grounded','setup_failed'].includes(scenario);
  assert.equal(element('land').disabled,!enabled);
  assert.equal(element('arm').disabled,true);
  assert.equal(element('takeoff').disabled,true);
  if(scenario==='missing_motion')assert.equal(element('ground').disabled,true);
  if(scenario==='setup_failed'){
    assert(!/redémarre/i.test(element('readiness').textContent));
    assert(/atterrir/.test(element('readiness').textContent));
  }
  if(scenario==='release_fault'){
    context.ui.release();await new Promise(resolve=>setImmediate(resolve));
    assert.equal(calls.length,0);
  }else if(scenario==='renew_fault'){
    context.ui.setLease('expired-lease');await context.ui.renew();
    assert.equal(calls.length,0);
  }else{
    await context.ui.requestLand();
    assert.deepEqual(calls.map(call=>call.action),enabled?['land']:[]);
    if(scenario==='landing_bound')assert(/déjà en cours/.test(element('message').textContent));
  }
  console.log(JSON.stringify({scenario,flight_intents:calls.map(call=>call.action)}));
})().catch(error=>{console.error(error);process.exitCode=1});
'''


@pytest.mark.parametrize('scenario',[
    'vertical_renew','ordinary_renew','fault_bound','landing_bound','fault_no_link','fault_grounded',
    'release_fault','renew_fault','missing_motion','setup_failed',
    'confirm_requested','confirm_fault','confirm_absent','confirm_false','confirm_stale',
    'confirm_no_link','confirm_grounded','confirm_wrong_state','confirm_no_implicit',
    'confirm_double','confirm_prepare_error','confirm_missing_token',
    'confirm_flag_changed','confirm_generation_changed','confirm_send_error',
])
def test_explicit_landing_recovery_does_not_restore_movement_or_retry_implicitly(scenario):
    source=Path(__file__).resolve().parents[1]/'src/neo2_control/web/flight.js'
    result=subprocess.run([NODE,'-e',SCRIPT,str(source)],input=json.dumps(scenario),
                          text=True,capture_output=True,timeout=10)
    assert result.returncode==0,result.stderr
    assert json.loads(result.stdout)['scenario']==scenario


def test_landing_confirmation_has_a_separate_initially_disabled_control_and_explanation():
    class Page(HTMLParser):
        def __init__(self):
            super().__init__();self.elements={};self.labels={};self.button=None

        def handle_starttag(self,tag,attributes):
            attrs=dict(attributes)
            if 'id' in attrs:self.elements[attrs['id']]=(tag,attrs)
            if tag=='button':self.button=attrs.get('id')

        def handle_data(self,text):
            if self.button:self.labels[self.button]=self.labels.get(self.button,'')+text

        def handle_endtag(self,tag):
            if tag=='button':self.button=None

    page=Page()
    page.feed((Path(__file__).resolve().parents[1]/'src/neo2_control/web/flight.html').read_text())
    tag,attrs=page.elements['confirm-landing']
    assert tag=='button' and 'disabled' in attrs and 'hidden' in attrs
    assert page.labels['confirm-landing']=='Confirmer l’atterrissage'
    assert attrs['aria-describedby']=='landing-confirmation-state'
    assert 'landing-confirmation-state' in page.elements
    assert page.labels['land']=='↓ Atterrir'
    assert page.labels['ground']=='Je confirme : posé, moteurs arrêtés'


INGRESS_SCRIPT=r'''
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const scenario=JSON.parse(fs.readFileSync(0,'utf8'));
let now=0,windows=0,blobReads=0,finishWindow,finishPost,frameIdentifier='frame-one';
const swappedWindow=scenario.match(/^window_swap_(owner|connection)_(open|ground|recover|land)$/);
const posts=[],signals=[],elements=new Map(),listeners=new Map();
function element(id){
  if(!elements.has(id))elements.set(id,{disabled:false,hidden:false,textContent:'',value:0,src:'',
    classList:{toggle(){},add(){},remove(){}},close(){},showModal(){},addEventListener(){}});
  return elements.get(id);
}
const status={now:100,state:'flying',connected:true,possible_airborne:true,
  controller_owner:{epoch:'c'.repeat(48),available:true,restart_available:false},recovery:{required:false},
  telemetry:{grounded:false,motors_on:true,flying:true,battery_percent:90,altitude_m:1,distance_m:0,
    component_age_seconds:{motion:.1,battery:.1,control:.1}},
  connection:{worker_alive:true,generation:'a'.repeat(24),
    camera:{ready:true,capabilities:{photo:true,record:true,pitch:true}},
    video:{preview_available:true,live:true,last_encoded_age_seconds:.01,preview_age_seconds:.01}}};
if(scenario==='unknown_owner')delete status.controller_owner.epoch;
if(scenario==='invalid_owner')status.controller_owner.epoch='not-an-owner';
if(scenario==='invalid_generation')status.connection.generation='not-a-generation';
if(scenario==='missing_generation')delete status.connection.generation;
if(scenario==='null_generation')status.connection.generation=null;
if(swappedWindow?.[2]==='open'){status.state='disconnected';status.connected=false;status.possible_airborne=false;}
if(swappedWindow?.[2]==='ground'){status.state='fault';status.telemetry.motors_on=status.telemetry.flying=false;}
if(swappedWindow?.[2]==='recover')Object.assign(status.controller_owner,{available:false,restart_available:true});
if(['status_delay','status_elapsed'].includes(scenario)){
  status.state='ready';status.possible_airborne=false;
  status.telemetry.grounded=true;status.telemetry.motors_on=status.telemetry.flying=false;
}
if(scenario==='confirmation_delay'){
  status.state='landing';status.landing_confirmation={available:true,needed:true,received_at:99.51,
    motion_received_at:99.99,control_received_at:99.99,ready_at:99.99,route_seen_at:99.99};
}
if(scenario.startsWith('yaw_')){
  status.yaw_trial=!['yaw_normal_profile','yaw_vertical_unchanged','yaw_profile_transition'].includes(scenario);
  status.vertical_trial=scenario==='yaw_vertical_unchanged';
  status.limits={maximum_channel:200,maximum_flight_seconds:30,maximum_altitude_m:1.5,maximum_distance_m:5};
  if(scenario==='yaw_custom_limits')Object.assign(status.limits,{maximum_channel:75,maximum_flight_seconds:12,maximum_altitude_m:.8});
  if(scenario==='yaw_missing_limit')delete status.limits.maximum_channel;
  if(scenario==='yaw_overlarge_limit')status.limits.maximum_channel=250;
}
const clone=value=>JSON.parse(JSON.stringify(value));
const reply=data=>({ok:true,status:200,json:async()=>data});
const context={document:{getElementById:element,querySelector:()=>({content:'fixture-token'}),
  visibilityState:'visible',hasFocus:()=>true,addEventListener(){},activeElement:null},
  window:{addEventListener:(type,handler)=>listeners.set(type,handler)},performance:{now:()=>now},AbortSignal,AbortController,
  setInterval(){},clearTimeout(){},setTimeout,
  URL:{createObjectURL:()=> 'blob:image',revokeObjectURL(){}},
  Image:class{set src(value){if(scenario==='frame_delayed')now+=100;queueMicrotask(()=>this.onload())}},
  fetch:async(path,options={})=>{
    if(path.endsWith('/window')){
      const seq=++windows;
      const ticket={ticket:'ticket-'+seq,mutation_sequence:seq,expires_in_seconds:.25,
        owner_epoch:status.controller_owner.epoch,connection_generation:status.connection.generation};
      if(scenario==='invalid_ticket')ticket.ticket='';
      if(scenario==='invalid_sequence')ticket.mutation_sequence=Number.MAX_SAFE_INTEGER+1;
      if(scenario==='invalid_window_duration')ticket.expires_in_seconds=.3;
      if(scenario==='expired_window')now+=300;
      if(swappedWindow)return await new Promise(resolve=>{finishWindow=()=>{
        if(swappedWindow[1]==='owner')status.controller_owner.epoch=ticket.owner_epoch='d'.repeat(48);
        else status.connection.generation=ticket.connection_generation='b'.repeat(24);
        resolve(reply(ticket));
      }});
      if(['changed_window','land_before_old_window'].includes(scenario)&&seq===1)
        return await new Promise(resolve=>{finishWindow=()=>resolve(reply(ticket))});
      if(scenario==='land_context_changed'&&seq===2){status.connection.generation=ticket.connection_generation='b'.repeat(24);}
      return reply(ticket);
    }
    if(options.method==='POST'){
      const body=JSON.parse(options.body);posts.push({path,...body});signals.push(options.signal);
      assert.deepEqual(Object.keys(body).sort(),['action','ingress','parameters']);
      assert.deepEqual(Object.keys(body.ingress).sort(),['expected_connection_generation','expected_owner_epoch','mutation_sequence','ticket']);
      assert.equal(body.ingress.ticket,'ticket-'+body.ingress.mutation_sequence);
      assert.equal(body.ingress.expected_owner_epoch,status.controller_owner.epoch);
      assert.equal(body.ingress.expected_connection_generation,status.connection.generation);
      if(scenario==='uncertain_post')throw new Error('fixture network uncertainty');
      if(['busy_camera','land_over_camera','land_over_control','land_over_keepalive','land_over_renew','land_context_changed','changed_post'].includes(scenario)&&posts.length===1)
        return await new Promise(resolve=>{finishPost=()=>resolve(reply({accepted:true}))});
      if(body.action==='land')status.state='landing';
      return reply({accepted:true});
    }
    if(path.endsWith('/frame')){
      if(scenario==='frame_delayed')now+=200;
      const headers=new Map([
        ['X-Neo2-Connection-Generation',scenario==='frame_wrong_generation'?'b'.repeat(24):status.connection.generation],
        ['X-Neo2-Frame-Age-Seconds',scenario==='frame_delayed'?'2.5':scenario==='frame_old'?'3.1':'.1'],
        ['X-Neo2-Frame-Id',frameIdentifier],
      ]);
      if(scenario==='frame_missing_age')headers.delete('X-Neo2-Frame-Age-Seconds');
      if(scenario==='frame_missing_id')headers.delete('X-Neo2-Frame-Id');
      return {ok:true,headers:{get:key=>headers.get(key)??null},blob:async()=>{
        blobReads++;if(scenario==='frame_delayed')now+=400;return {};
      }};
    }
    if(path.endsWith('/status')){
      if(scenario==='status_delay')now+=400;
      if(scenario==='confirmation_delay')now+=100;
      return {ok:true,status:200,json:async()=>{if(scenario==='status_delay')now+=200;return clone(status)}};
    }
    throw new Error('unhandled fixture path '+path);
  }};
const settle=()=>new Promise(resolve=>setImmediate(resolve));
(async()=>{
  const source=fs.readFileSync(process.argv[1],'utf8');
  await vm.runInNewContext('(async()=>{'+source+`;globalThis.ui={api,renew,requestLand,frame,liveImage,render,invalidateOperatorContext,
    seed(){lease='fixture';held=true;},age(){render(current)},
    evidence(){return {lease,held,frameAt,frameAgeSeconds,frameId,frameUrl,ages:current.telemetry.component_age_seconds}}
  };})()`,context);
  if(swappedWindow){
    context.ui.seed();
    const button={open:'connect',ground:'ground',recover:'recover-owner',land:'land'}[swappedWindow[2]];
    const request=element(button).onclick();await settle();
    assert.equal(windows,1);assert.equal(posts.length,0);
    finishWindow();await request;
    assert.equal(posts.length,0);assert.equal(windows,1);
    assert.equal(context.ui.evidence().lease,null);assert.equal(context.ui.evidence().held,false);
    await context.ui.renew();assert.equal(posts.length,0);
  }else if(['unknown_owner','invalid_owner','invalid_generation','missing_generation'].includes(scenario)){
    await assert.rejects(context.ui.api('intent',{action:'open',parameters:{}}));
    assert.equal(windows,0);assert.equal(posts.length,0);
  }else if(scenario==='null_generation'){
    await context.ui.api('intent',{action:'open',parameters:{}});
    assert.equal(windows,1);assert.equal(posts.length,1);
    assert.equal(posts[0].ingress.expected_connection_generation,null);
  }else if(scenario==='owner_changed_same_generation'){
    context.ui.seed();await context.ui.frame();
    status.controller_owner.epoch='d'.repeat(48);context.ui.render(clone(status));
    assert.equal(context.ui.evidence().lease,null);assert.equal(context.ui.evidence().held,false);
    assert.equal(context.ui.evidence().frameUrl,null);
    await context.ui.renew();assert.equal(posts.length,0);
  }else if(scenario.startsWith('yaw_')){
    context.ui.seed();
    const event=code=>({code,target:{matches:()=>false},preventDefault(){}});
    for(const key of ['Space','ArrowUp','ArrowRight','KeyR'])listeners.get('keydown')(event(key));
    if(scenario!=='yaw_no_auto')listeners.get('keydown')(event(scenario==='yaw_negative'?'KeyA':'KeyE'));
    if(scenario==='yaw_both')listeners.get('keydown')(event('KeyA'));
    if(scenario==='yaw_profile_transition'){status.yaw_trial=true;context.ui.render(clone(status));}
    if(scenario==='yaw_no_auto'){
      for(let i=0;i<3;i++)context.ui.render(clone(status));assert.equal(posts.length,0);
    }else{
      await context.ui.renew();
      assert.equal(posts.length,1);assert.equal(windows,1);
      if(scenario==='yaw_vertical_unchanged'){
        assert.equal(posts[0].action,'keepalive');assert(element('yaw-guide').hidden);
      }else{
        const expected=scenario==='yaw_normal_profile'?[250,250,250,250]:[0,0,0,
          scenario==='yaw_negative'?-200:scenario==='yaw_both'||scenario==='yaw_missing_limit'?0:scenario==='yaw_custom_limits'?75:200];
        assert.deepEqual(posts[0].parameters.channels,expected);
        assert.equal(element('movement-guide').hidden,scenario!=='yaw_normal_profile');
        assert.equal(element('vertical-guide').hidden,scenario!=='yaw_normal_profile');
        if(scenario!=='yaw_normal_profile')assert(/rotation seule/i.test(element('trial-limits').textContent));
      }
      if(scenario==='yaw_custom_limits'){
        assert(element('trial-limits').textContent.includes('12 s'));assert(element('trial-limits').textContent.includes('0.8 m'));
      }
      if(scenario==='yaw_release'){
        listeners.get('keyup')(event('Space'));await settle();
        assert.equal(posts.filter(p=>p.action==='land').length,1);
        await context.ui.renew();assert.equal(posts.filter(p=>p.action==='control').length,1);
      }
    }
  }else if(scenario==='one_window_per_post'){
    await context.ui.api('intent',{action:'open',parameters:{}});
    await context.ui.api('camera',{action:'photo',parameters:{}});
    assert.equal(windows,2);assert.deepEqual(posts.map(p=>p.ingress.mutation_sequence),[1,2]);
  }else if(['invalid_ticket','invalid_sequence','invalid_window_duration','expired_window'].includes(scenario)){
    await assert.rejects(context.ui.api('intent',{action:'open',parameters:{}}));
    assert.equal(windows,1);assert.equal(posts.length,0);
  }else if(scenario==='uncertain_post'){
    await assert.rejects(context.ui.api('camera',{action:'photo',parameters:{}}));
    assert.equal(windows,1);assert.equal(posts.length,1);
  }else if(['changed_window','changed_post'].includes(scenario)){
    const previous=context.ui.api('intent',{action:'keepalive',parameters:{lease:'old'}});
    const rejected=assert.rejects(previous,error=>error.name==='ObsoleteMutation');
    await settle();context.ui.invalidateOperatorContext();(finishWindow||finishPost)();await rejected;
    assert.equal(posts.length,scenario==='changed_window'?0:1);assert.equal(windows,1);
  }else if(scenario==='busy_camera'){
    const previous=context.ui.api('camera',{action:'photo',parameters:{}});await settle();
    context.ui.seed();await context.ui.renew();
    await assert.rejects(context.ui.api('camera',{action:'photo',parameters:{}}));
    assert.equal(windows,1);assert.equal(posts.length,1);finishPost();await previous;
    await settle();assert.equal(windows,1);assert.equal(posts.length,1);
  }else if(scenario==='land_context_changed'){
    context.ui.seed();
    const previous=context.ui.api('intent',{action:'control',parameters:{lease:'old'}});
    const rejected=assert.rejects(previous,error=>error.name==='ObsoleteMutation');
    await settle();await context.ui.requestLand();
    assert.equal(windows,2);assert.equal(posts.length,1);assert.equal(posts[0].action,'control');
    assert(signals[0].aborted);finishPost();await rejected;
    assert.equal(context.ui.evidence().lease,null);assert.equal(context.ui.evidence().held,false);
    await context.ui.renew();assert.equal(posts.length,1);
  }else if(scenario.startsWith('land_')){
    context.ui.seed();
    const action=scenario==='land_over_keepalive'?'keepalive':scenario==='land_over_camera'?'photo':'control';
    const path=scenario==='land_over_camera'?'camera':'intent';
    const previous=scenario==='land_over_renew'?context.ui.renew():context.ui.api(path,{action,parameters:{}});
    const rejected=scenario==='land_over_renew'?previous:assert.rejects(previous,error=>error.name==='ObsoleteMutation');
    await settle();await context.ui.requestLand();
    assert.equal(windows,2);assert.equal(posts.at(-1).action,'land');
    assert.equal(posts.at(-1).ingress.mutation_sequence,2);
    if(scenario==='land_before_old_window'){finishWindow();assert.equal(posts.length,1);}
    else{assert(signals[0].aborted);finishPost();}
    await rejected;await settle();assert.equal(windows,2);
    assert.equal(posts.filter(p=>p.action==='land').length,1);
  }else if(scenario==='status_delay'){
    assert(context.ui.evidence().ages.motion>.5);assert(element('ground').disabled&&element('arm').disabled);
    assert.equal(element('altitude').textContent,'—');
  }else if(scenario==='status_elapsed'){
    assert(!element('arm').disabled);now+=600;context.ui.age();
    assert(element('arm').disabled&&element('ground').disabled);
    context.ui.seed();await context.ui.renew();assert.equal(posts.length,0);
  }else if(scenario==='confirmation_delay'){
    assert(element('confirm-landing').disabled);assert.equal(posts.length,0);
  }else if(scenario.startsWith('frame_')){
    await context.ui.frame();
    if(['frame_wrong_generation','frame_missing_age','frame_missing_id'].includes(scenario)){
      assert.equal(context.ui.evidence().frameUrl,null);assert.equal(blobReads,0);
    }else if(['frame_delayed','frame_old'].includes(scenario)){
      assert(context.ui.evidence().frameAgeSeconds>3);assert(!context.ui.liveImage());
    }else{
      assert(context.ui.liveImage());const first=context.ui.evidence().frameAt;
      now=2900;context.ui.render(clone(status));await context.ui.frame();
      assert.equal(context.ui.evidence().frameAt,first);assert.equal(blobReads,1);
      now=3200;context.ui.render(clone(status));assert(!context.ui.liveImage());
      if(scenario==='frame_new_id'){
        frameIdentifier='frame-two';await context.ui.frame();assert(context.ui.liveImage());assert.equal(blobReads,2);
      }
    }
  }else throw new Error('unknown scenario');
  console.log(JSON.stringify({scenario,windows,posts:posts.map(p=>p.action)}));
})().catch(error=>{console.error(error);process.exitCode=1});
'''


@pytest.mark.parametrize('scenario',[
    *[f'window_swap_{context}_{action}' for context in ('owner','connection')
      for action in ('open','ground','recover','land')],
    'unknown_owner','invalid_owner','invalid_generation','missing_generation','null_generation',
    'owner_changed_same_generation','land_context_changed',
    'one_window_per_post','invalid_ticket','invalid_sequence','invalid_window_duration','expired_window',
    'uncertain_post','changed_window','changed_post','busy_camera',
    'land_over_camera','land_over_control','land_over_keepalive','land_over_renew','land_before_old_window',
    'status_delay','status_elapsed','confirmation_delay',
    'frame_wrong_generation','frame_missing_age','frame_missing_id','frame_delayed','frame_old',
    'frame_duplicate','frame_new_id',
    'yaw_positive','yaw_negative','yaw_both','yaw_custom_limits','yaw_missing_limit','yaw_overlarge_limit',
    'yaw_release','yaw_normal_profile','yaw_vertical_unchanged','yaw_profile_transition','yaw_no_auto',
])
def test_pc_ingress_priority_and_conservative_browser_freshness(scenario):
    source=Path(__file__).resolve().parents[1]/'src/neo2_control/web/flight.js'
    result=subprocess.run([NODE,'-e',INGRESS_SCRIPT,str(source)],input=json.dumps(scenario),
        text=True,capture_output=True,timeout=10)
    assert result.returncode==0,result.stderr
    assert json.loads(result.stdout)['scenario']==scenario


@pytest.mark.parametrize('prefix,expected',[
    ('/home-server/api/drone/control','/home-server/api/drone/control'),
    ('https://other.invalid/control','/api/flight'),
    ('//other.invalid/control','/api/flight'),
    ('/home-server/../control','/api/flight'),
])
@pytest.mark.parametrize('scenario',['one_window_per_post','frame_new_id'])
def test_authenticated_remote_prefix_covers_windows_commands_and_frames(prefix,expected,scenario):
    metadata={'neo2-token':'fixture-jwt','neo2-api-prefix':prefix,'neo2-auth':'bearer'}
    selection="querySelector:selector=>({content:"+json.dumps(metadata)+"[selector.match(/name=\"([^\"]+)\"/)[1]]})"
    script=INGRESS_SCRIPT.replace("querySelector:()=>({content:'fixture-token'})",selection)
    script=script.replace('fetch:async(path,options={})=>{',
        'fetch:async(path,options={})=>{'+
        'assert(path.startsWith('+json.dumps(expected+'/')+'));'+
        'assert.equal(options.headers.Authorization,"Bearer fixture-jwt");'+
        'assert.equal(options.headers["X-Neo2-Token"],undefined);')
    source=Path(__file__).resolve().parents[1]/'src/neo2_control/web/flight.js'
    result=subprocess.run([NODE,'-e',script,str(source)],input=json.dumps(scenario),
        text=True,capture_output=True,timeout=10)
    assert result.returncode==0,result.stderr


@pytest.mark.parametrize('scenario',['remote_piggyback','remote_cached_expiry','remote_invalid_next',
    'remote_stale_status','land_over_control','land_over_camera','land_context_changed'])
def test_remote_target_ui_measures_latency_and_bounds_cached_windows(scenario):
    metadata={'neo2-token':'fixture-jwt','neo2-api-prefix':'/home-server/api/drone/control',
        'neo2-auth':'bearer','neo2-command-mode':'remote-target'}
    selection="querySelector:selector=>({content:"+json.dumps(metadata)+"[selector.match(/name=\"([^\"]+)\"/)[1]]})"
    script=INGRESS_SCRIPT.replace("querySelector:()=>({content:'fixture-token'})",selection)
    script=script.replace('let now=0,windows=0,','let now=0,windows=0,issued=0,')
    script=script.replace('const seq=++windows;','windows++;const seq=++issued;')
    script=script.replace('expires_in_seconds:.25,','expires_in_seconds:2,')
    script=script.replace("assert.deepEqual(Object.keys(body).sort(),['action','ingress','parameters']);",
        "assert.deepEqual(Object.keys(body).sort(),['action','ingress','parameters','round_trip_seconds']);")
    script=script.replace('return reply(ticket);','now+=550;return reply(ticket);')
    script=script.replace('return reply({accepted:true});',r'''
      now+=550;
      const next=++issued;
      return reply({accepted:true,next_window:{ticket:'ticket-'+next,mutation_sequence:next,
        expires_in_seconds:scenario==='remote_invalid_next'?3:1.5,
        owner_epoch:status.controller_owner.epoch,connection_generation:status.connection.generation}});
    ''')
    # Branch into the same executed production JS/DOM fixture used for native
    # priority tests, with a 550 ms clock advance on each HTTP exchange.
    script=script.replace('if(swappedWindow){\n    context.ui.seed();',r'''
  if(scenario.startsWith('remote_')){
    await context.ui.api('camera',{action:'photo',parameters:{}});
    assert.equal(windows,1);assert.equal(posts.length,1);
    assert.equal(posts[0].round_trip_seconds,.55);
    if(scenario==='remote_invalid_next'){
      await assert.rejects(context.ui.api('camera',{action:'photo',parameters:{}}));
      assert.equal(posts.length,1);
    }else if(scenario==='remote_stale_status'){
      const recent=clone(status);recent.now=200;context.ui.render(recent);
      const old=clone(status);old.now=199;old.state='ready';context.ui.render(old);
      assert.equal(element('flight-state').textContent,'En vol');
    }else{
      if(scenario==='remote_cached_expiry')now+=2000;
      await context.ui.api('camera',{action:'photo',parameters:{}});
      assert.equal(windows,scenario==='remote_cached_expiry'?2:1);
      assert.equal(posts.length,2);
      assert.equal(posts[1].ingress.mutation_sequence,scenario==='remote_cached_expiry'?3:2);
    }
  }else if(swappedWindow){
    context.ui.seed();''')
    source=Path(__file__).resolve().parents[1]/'src/neo2_control/web/flight.js'
    result=subprocess.run([NODE,'-e',script,str(source)],input=json.dumps(scenario),
        text=True,capture_output=True,timeout=10)
    assert result.returncode==0,result.stderr
