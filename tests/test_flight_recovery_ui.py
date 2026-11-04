"""Exercise recovery decisions and stale asynchronous browser work without RF."""
from html.parser import HTMLParser
import json
from pathlib import Path
import shutil
import subprocess

import pytest

NODE=shutil.which('node')
pytestmark=pytest.mark.skipif(NODE is None,reason='optional JavaScript UI checks require Node.js')
SCRIPT=r'''
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const scenario=JSON.parse(fs.readFileSync(0,'utf8'));
const elements=new Map(),calls=[],revoked=[];
function element(id){
  if(!elements.has(id))elements.set(id,{disabled:false,hidden:false,textContent:'',value:0,src:'',
    classList:{toggle(){},add(){},remove(){}},close(){this.closed=true},showModal(){},addEventListener(){}});
  return elements.get(id);
}
let status={state:'fault',connected:false,possible_airborne:true,
  telemetry:{grounded:null,motors_on:true,flying:true,component_age_seconds:{motion:.1}},
  controller_owner:{epoch:'c'.repeat(48),available:false,restart_available:true},
  recovery:{required:true,incident_id:'must-not-be-shown',peer_match:null,reason:'unresolved_previous_flight'},
  connection:{generation:'a'.repeat(24),worker_alive:false,camera:{ready:false},video:{preview_available:true,live:true}}};
if(scenario==='alive_retained')status.controller_owner.restart_available=false;
const retryScenario=scenario.startsWith('retry_')||scenario==='connection_retry';
if(retryScenario){
  status.controller_owner={...status.controller_owner,available:scenario!=='retry_owner_dead',restart_available:scenario==='retry_owner_dead'};
  status.recovery.connection_retry_available=scenario!=='retry_unavailable';
}
if(scenario.startsWith('peer_')||scenario.endsWith('_ground')){
  status.controller_owner={...status.controller_owner,available:true,restart_available:false};status.connected=true;
  status.connection.worker_alive=true;
  status.recovery.peer_match=scenario==='peer_mismatch'?false:scenario==='peer_unknown'?null:true;
  status.connection.camera={ready:true,capabilities:{photo:true,record:true,stop_record:true,pitch:true,recenter:true}};
  if(scenario.endsWith('_ground')){
    status.telemetry.motors_on=status.telemetry.flying=false;
    if(scenario==='stale_ground')status.telemetry.component_age_seconds.motion=.6;
  }
}
let finishRecovery,finishPreparation,finishFrame,windowSequence=0;
const context={document:{getElementById:element,querySelector:()=>({content:'fixture-token'}),
  visibilityState:'visible',hasFocus:()=>true,addEventListener(){},activeElement:null},
  window:{addEventListener(){}},performance:{now:()=>0},AbortSignal,AbortController,
  setInterval(){},clearTimeout(){},setTimeout,
  URL:{createObjectURL:()=> 'blob:late-frame',revokeObjectURL:value=>revoked.push(value)},
  Image:class{set src(value){queueMicrotask(()=>this.onload())}},
  fetch:async(path,options={})=>{
    if(path.endsWith('/window'))return {status:200,ok:true,json:async()=>({ticket:'window-'+(++windowSequence),mutation_sequence:windowSequence,expires_in_seconds:.25,owner_epoch:status.controller_owner.epoch,connection_generation:status.connection.generation})};
    const body=options.body?JSON.parse(options.body):null;
    if(body){assert.equal(typeof body.ingress.ticket,'string');assert(Number.isSafeInteger(body.ingress.mutation_sequence));assert.equal(body.ingress.expected_owner_epoch,status.controller_owner.epoch);assert.equal(body.ingress.expected_connection_generation,status.connection.generation);calls.push(body);}
    if(path.endsWith('/frame'))return await new Promise(resolve=>{finishFrame=()=>resolve({ok:true,blob:async()=>({})})});
    if(body?.action==='prepare'&&scenario==='old_prepare_reply')return await new Promise(resolve=>{
      finishPreparation=()=>resolve({ok:true,status:200,json:async()=>({confirmation_token:'old-token'})});
    });
    if(body?.action==='recover_owner'){
      if(scenario==='recovery_failed')return {ok:false,status:409,json:async()=>({error:'controller_owner_still_running'})};
      const result=()=>{
        status={...status,state:'disconnected',controller_owner:{epoch:'d'.repeat(48),available:true,restart_available:false},
          connection:{worker_alive:false,generation:null,video:{},camera:{}}};
        return {ok:true,status:202,json:async()=>({accepted:true,status})};
      };
      if(scenario==='double_click')return await new Promise(resolve=>{finishRecovery=()=>resolve(result())});
      return result();
    }
    if(body?.action==='reconnect'){
      if(scenario==='retry_failed')return {ok:false,status:409,json:async()=>({error:'previous_flight_cleanup_unconfirmed'})};
      const result=()=>{
        status={...status,recovery:{...status.recovery,connection_retry_available:false},
          connection:{...status.connection,generation:'b'.repeat(24),worker_alive:true}};
        return {ok:true,status:202,json:async()=>({accepted:true})};
      };
      if(scenario==='retry_double_click')return await new Promise(resolve=>{finishRecovery=()=>resolve(result())});
      return result();
    }
    return {ok:true,status:200,json:async()=>path.endsWith('/status')?status:{}};
  }};
(async()=>{
  const source=fs.readFileSync(process.argv[1],'utf8');
  await vm.runInNewContext('(async()=>{'+source+`;globalThis.ui={render,requestOwnerRecovery,requestConnectionRetry,requestLand,renew,frame,
    seed(){lease='old-lease';held=true;keys.add('ArrowUp');challenge={confirmation_token:'old-token'};
      frameUrl='blob:old-frame';frameGeneration='a'.repeat(24);frameAt=0;},
    evidence(){return {lease,held,keys:keys.size,challenge,frameUrl,frameGeneration}},
  };})()`,context);
  assert.equal(calls.length,0);
  assert(!element('recovery-state').textContent.includes('must-not-be-shown'));
  assert.equal(element('flight-state').textContent,'Vol à vérifier');
  if(retryScenario){
    const available=!['retry_owner_dead','retry_unavailable'].includes(scenario);
    assert.equal(element('retry-connection').hidden,!available);
    assert.equal(element('retry-connection').disabled,!available);
    for(let i=0;i<3;i++)context.ui.render(status);
    assert.equal(calls.length,0);context.ui.seed();
    if(scenario==='retry_polling'){await context.ui.renew();assert.equal(calls.length,0);}
    else{
      const retry=context.ui.requestConnectionRetry();
      if(scenario==='retry_double_click'){
        await new Promise(resolve=>setImmediate(resolve));
        assert(element('retry-connection').disabled);await context.ui.requestConnectionRetry();
        assert.deepEqual(calls.map(c=>c.action),['reconnect']);finishRecovery();
      }
      await retry;
      assert.deepEqual(calls.map(c=>c.action),available?['reconnect']:[]);
      if(available){
        const proof=context.ui.evidence();
        assert.equal(proof.lease,null);assert.equal(proof.held,false);assert.equal(proof.keys,0);
        assert.equal(proof.challenge,null);assert.equal(proof.frameUrl,null);
        assert(status.possible_airborne&&status.recovery.required);
        assert(element('arm').disabled&&element('takeoff').disabled);
        assert.equal(element('retry-connection').hidden,scenario!=='retry_failed');
      }
    }
  }else if(scenario==='alive_retained'){
    assert(element('recover-owner').hidden&&element('recover-owner').disabled);
    await context.ui.requestOwnerRecovery();assert.equal(calls.length,0);
  }else if(scenario.startsWith('peer_')||scenario.endsWith('_ground')){
    assert(element('recover-owner').hidden&&element('arm').disabled&&element('takeoff').disabled);
    assert(element('photo').disabled&&element('record').disabled&&element('pitch').disabled);
    const matches=status.recovery.peer_match===true;
    assert.equal(element('land').disabled,!matches);
    assert.equal(element('ground').disabled,scenario!=='fresh_ground');
    context.ui.seed();await context.ui.renew();assert.equal(calls.length,0);
    await context.ui.requestLand();assert.deepEqual(calls.map(c=>c.action),matches?['land']:[]);
  }else if(scenario==='dead_polling'){
    assert(!element('recover-owner').hidden&&!element('recover-owner').disabled);
    for(let i=0;i<3;i++)context.ui.render(status);
    context.ui.seed();await context.ui.renew();assert.equal(calls.length,0);
  }else{
    context.ui.seed();let previous;
    if(scenario==='old_prepare_reply'){
      previous=element('ground').onclick();await new Promise(resolve=>setImmediate(resolve));
      assert.deepEqual(calls.map(c=>c.action),['prepare']);
    }
    if(scenario==='old_frame_reply'){
      previous=context.ui.frame();await new Promise(resolve=>setImmediate(resolve));
    }
    const recovery=context.ui.requestOwnerRecovery();
    if(scenario==='double_click'){
      await new Promise(resolve=>setImmediate(resolve));
      assert(element('recover-owner').disabled);
      await context.ui.requestOwnerRecovery();
      assert.deepEqual(calls.map(c=>c.action),['recover_owner']);finishRecovery();
    }
    await recovery;
    if(finishPreparation){finishPreparation();await previous;}
    if(finishFrame){finishFrame();await previous;}
    const proof=context.ui.evidence();
    assert.equal(proof.lease,null);assert.equal(proof.held,false);assert.equal(proof.keys,0);
    assert.equal(proof.challenge,null);assert.equal(proof.frameUrl,null);assert.equal(proof.frameGeneration,null);
    assert(revoked.includes('blob:old-frame'));assert(element('preview').hidden);
    assert(!calls.some(c=>['open','control','keepalive','confirm_ground','takeoff','land'].includes(c.action)));
    assert.equal(calls.filter(c=>c.action==='recover_owner').length,1);
    if(scenario==='recovery_failed')assert(!element('recover-owner').hidden&&!element('recover-owner').disabled);
    else{
      assert(element('recover-owner').hidden);assert(!element('connect').hidden&&!element('connect').disabled);
      assert(element('arm').disabled&&element('takeoff').disabled&&element('ground').disabled);
      if(scenario==='explicit_reconnect'){
        await element('connect').onclick();assert.deepEqual(calls.map(c=>c.action),['recover_owner','open']);
      }
    }
  }
  console.log(JSON.stringify({scenario,actions:calls.map(c=>c.action)}));
})().catch(error=>{console.error(error);process.exitCode=1});
'''


@pytest.mark.parametrize('scenario',[
    'alive_retained','dead_polling','explicit_reconnect','double_click','recovery_failed',
    'peer_unknown','peer_mismatch','peer_match','fresh_ground','stale_ground',
    'old_prepare_reply','old_frame_reply',
    'connection_retry','retry_double_click','retry_failed','retry_unavailable','retry_owner_dead','retry_polling',
])
def test_owner_recovery_requires_explicit_action_and_never_restores_old_authority(scenario):
    source=Path(__file__).resolve().parents[1]/'src/neo2_control/web/flight.js'
    result=subprocess.run([NODE,'-e',SCRIPT,str(source)],input=json.dumps(scenario),
        text=True,capture_output=True,timeout=10)
    assert result.returncode==0,result.stderr
    assert json.loads(result.stdout)['scenario']==scenario


def test_recovery_button_starts_hidden_and_does_not_claim_to_resume_flight():
    class Page(HTMLParser):
        def __init__(self):super().__init__();self.elements={}
        def handle_starttag(self,tag,attributes):
            attrs=dict(attributes)
            if 'id' in attrs:self.elements[attrs['id']]=(tag,attrs)
    page=Page();page.feed((Path(__file__).resolve().parents[1]/'src/neo2_control/web/flight.html').read_text())
    tag,attrs=page.elements['recover-owner']
    assert tag=='button' and 'hidden' in attrs and 'disabled' in attrs
    assert attrs['aria-describedby']=='recovery-state'
    assert page.elements['recovery-state'][1]['role']=='status'
    retry_tag,retry_attrs=page.elements['retry-connection']
    assert retry_tag=='button' and 'hidden' in retry_attrs and 'disabled' in retry_attrs
    assert retry_attrs['aria-describedby']=='recovery-state'
