"""Production page program actions in Node's VM; fake DOM/HTTP, no devices."""
import json
from pathlib import Path
import shutil
import subprocess

import pytest

from neo2_control.motion_program import MotionProgram

NODE=shutil.which('node')
pytestmark=pytest.mark.skipif(NODE is None,reason='JavaScript UI checks need Node.js')
SOURCE=Path(__file__).resolve().parents[1]/'src/neo2_control/web/flight.js'
PROGRAM={'version':1,'finish':'land','steps':[
    {'kind':'translate','seconds':.3,'right':50,'forward':0,'up':0},
    {'kind':'rotate','degrees':90,'channel':150,'timeout_seconds':3}]}
SCRIPT=r'''
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const options=JSON.parse(fs.readFileSync(0,'utf8')),scenario=options.scenario;
const elements=new Map(),listeners=new Map(),posts=[];
function element(id){
  if(!elements.has(id))elements.set(id,{disabled:false,hidden:false,value:'',textContent:'',files:[],
    classList:{toggle(){},add(){},remove(){}},close(){},showModal(){},addEventListener(){}});
  return elements.get(id);
}
let now=0,sequence=0,finishPost,blockedAction=null;
const clone=value=>JSON.parse(JSON.stringify(value));
const id='f'.repeat(24);
const status={now:100,state:'ready',connected:true,possible_airborne:false,
  limits:{maximum_channel:250,maximum_flight_seconds:30},
  controller_owner:{epoch:'a'.repeat(48),available:true},
  telemetry:{motors_on:false,flying:false,altitude_m:0,battery_percent:90,
    component_age_seconds:{motion:0,control:0,battery:0}},
  connection:{generation:'b'.repeat(24),worker_alive:true,
    video:{live:true,last_encoded_age_seconds:0,preview_age_seconds:0},camera:{}},
  remote_control:{program:null}};
const ticket=()=>({ticket:'ticket-'+(++sequence),mutation_sequence:sequence,expires_in_seconds:.8,
  owner_epoch:status.controller_owner.epoch,connection_generation:status.connection.generation});
const reply=data=>({ok:true,status:200,json:async()=>clone(data)});
const meta={'neo2-token':'fixture','neo2-api-prefix':'/api/remote-flight',
  'neo2-command-mode':'remote-target','neo2-auth':'local'};
const context={document:{getElementById:element,querySelector:selector=>({content:meta[selector.match(/name="([^"]+)/)?.[1]]}),
  hasFocus:()=>true,visibilityState:'visible',addEventListener:(event,handler)=>listeners.set(event,handler)},
  window:{addEventListener:(event,handler)=>listeners.set(event,handler)},
  performance:{now:()=>now},Blob,AbortSignal,AbortController,URL,Image:class{},
  setInterval(){},setTimeout,clearTimeout,
  fetch:async(path,request={})=>{
    if(path.endsWith('/window'))return reply(ticket());
    if(path.endsWith('/status'))return reply(status);
    assert.equal(request.method,'POST');
    const body=JSON.parse(request.body);posts.push(body);
    assert.equal(body.ingress.expected_owner_epoch,status.controller_owner.epoch);
    assert.equal(body.ingress.expected_connection_generation,status.connection.generation);
    if(body.action==='program_start'){
      status.remote_control.program={program_id:id,phase:'running',step_count:2,completed_steps:0};
      if(scenario==='uncertain_start')throw new Error('fixture dropped reply');
    }
    if(body.action==='control'&&status.remote_control.program)status.remote_control.program.phase='cancelled';
    if(body.action==='land'){
      status.state='landing';if(status.remote_control.program)status.remote_control.program.phase='cancelled';
    }
    const result={accepted:true,status:clone(status),next_window:ticket(),
      ...(body.action==='program_start'?{program_id:scenario==='missing_identifier'?null:id}:{})};
    if(body.action===blockedAction)return await new Promise(resolve=>{finishPost=()=>resolve(reply(result))});
    return reply(result);
  }};
const settle=()=>new Promise(resolve=>setImmediate(resolve));
const event=code=>({code,target:{matches:()=>false},preventDefault(){}});
(async()=>{
  const source=fs.readFileSync(process.argv[1],'utf8');
  await vm.runInNewContext('(async()=>{'+source+`;globalThis.ui={api,validateProgram,updateProgramInput,loadProgramFile,startProgram,renew,render,release,
    seed(){lease='fixture';held=true;frameAt=performance.now();frameAgeSeconds=0;frameGeneration=current.connection.generation},
    state(){return {activeProgramId,lease,held,programStarting}},contextChange:invalidateOperatorContext};})()`,context);
  if(scenario==='validate'){
    let valid=true;try{context.ui.validateProgram(options.program)}catch{valid=false}
    assert.equal(valid,options.valid);return;
  }
  const load=()=>{element('program-json').value=JSON.stringify(options.program);element('program-json').oninput()};
  load();assert.equal(posts.length,0);assert(element('program-start').disabled);
  if(scenario==='oversize'){
    element('program-json').value=' '.repeat(8193)+'{}';element('program-json').oninput();
    assert(element('program-summary').textContent.includes('8 192'));return;
  }
  if(scenario==='file_race'){
    let finishFile;element('program-file').files=[{size:100,text:()=>new Promise(resolve=>{finishFile=resolve})}];
    const loading=context.ui.loadProgramFile();load();finishFile('{}');await loading;
    assert(!element('program-summary').textContent.includes('invalide'));assert.equal(posts.length,0);return;
  }
  if(scenario==='grounded'){
    context.ui.seed();await context.ui.startProgram();assert.equal(posts.length,0);return;
  }
  Object.assign(status,{state:'flying',possible_airborne:true});
  Object.assign(status.telemetry,{motors_on:true,flying:true,altitude_m:1});
  context.ui.render(clone(status));context.ui.seed();context.ui.render(clone(status));
  if(['yaw_profile','vertical_profile','channel_limit','stale_status','no_image','no_focus','no_hold'].includes(scenario)){
    if(scenario==='yaw_profile')status.yaw_trial=true;
    if(scenario==='vertical_profile')status.vertical_trial=true;
    if(scenario==='channel_limit')status.limits.maximum_channel=20;
    if(scenario==='stale_status')status.telemetry.component_age_seconds.motion=2;
    if(scenario==='no_image')status.connection.video.live=false;
    if(scenario==='no_focus')context.document.hasFocus=()=>false;
    if(scenario==='no_hold')context.ui.release();
    await settle();context.ui.render(clone(status));await context.ui.startProgram();
    assert(element('program-start').disabled);assert(!posts.some(p=>p.action==='program_start'));return;
  }
  assert(!element('program-start').disabled);
  if(scenario==='program_over_renew'||scenario==='takeoff_over_renew'){
    blockedAction=scenario==='program_over_renew'?'control':'keepalive';
    if(scenario==='takeoff_over_renew'){status.state='armed';context.ui.render(clone(status));}
    const renewal=context.ui.renew();await settle();assert(finishPost);
    if(scenario==='program_over_renew')await context.ui.startProgram();
    else await context.ui.api('intent',{action:'prepare',parameters:{operation:'takeoff',lease:'fixture'}});
    finishPost();await renewal;
    assert.deepEqual(posts.map(p=>p.action),scenario==='program_over_renew'?['control','program_start']:['keepalive','prepare']);
    assert.equal(context.ui.state().lease,'fixture');
    if(scenario==='program_over_renew')assert.equal(context.ui.state().activeProgramId,id);
    return;
  }
  if(scenario==='land_over_start'){
    blockedAction='program_start';const started=context.ui.startProgram();await settle();
    assert(finishPost);await element('program-stop').onclick();finishPost();await started;
    assert.equal(context.ui.state().activeProgramId,null);assert.equal(context.ui.state().lease,null);
    assert.deepEqual(posts.map(p=>p.action),['program_start','land']);await context.ui.renew();return;
  }
  await context.ui.startProgram();
  if(['uncertain_start','missing_identifier'].includes(scenario)){
    await settle();assert.deepEqual(posts.map(p=>p.action),['program_start','land']);
    assert.equal(context.ui.state().lease,null);await context.ui.renew();assert.equal(posts.length,2);return;
  }
  assert.equal(context.ui.state().activeProgramId,id);
  if(scenario==='keepalive'){
    await context.ui.renew();await context.ui.renew();
    assert.deepEqual(posts.map(p=>p.action),['program_start','program_keepalive','program_keepalive']);
    assert.equal(posts[1].parameters.program_id,id);assert(element('program-json').readOnly);
    status.remote_control.program={...status.remote_control.program,phase:'motion_complete',completed_steps:2,
      last_rotation:{progress_degrees:90,target_degrees:90}};status.state='landing';context.ui.render(clone(status));
    assert(element('program-state').textContent.includes('Atterrissage à vérifier'));
    assert.equal(element('program-progress').value,2);await context.ui.renew();assert.equal(posts.length,3);
  }else if(scenario==='manual'){
    listeners.get('keydown')(event('ArrowRight'));await context.ui.renew();
    assert.equal(posts.at(-1).action,'control');assert.equal(posts.at(-1).parameters.channels[0],250);
    assert.equal(context.ui.state().activeProgramId,null);assert.equal(context.ui.state().lease,'fixture');
    assert(!posts.some(p=>p.action==='land'));
  }else if(scenario==='release'||scenario==='blur'){
    if(scenario==='release')listeners.get('keyup')(event('Space'));else listeners.get('blur')();
    await settle();await context.ui.renew();assert.deepEqual(posts.map(p=>p.action),['program_start','land']);
  }else if(scenario==='land_over_renew'){
    blockedAction='program_keepalive';const renewal=context.ui.renew();await settle();
    assert(finishPost);await element('program-stop').onclick();finishPost();await renewal;
    assert.equal(context.ui.state().activeProgramId,null);assert.equal(context.ui.state().lease,null);
    assert.deepEqual(posts.map(p=>p.action),['program_start','program_keepalive','land']);
  }else if(scenario==='generation_changed'){
    status.connection.generation='c'.repeat(24);context.ui.render(clone(status));await context.ui.renew();
    assert.equal(context.ui.state().activeProgramId,null);assert.equal(posts.length,1);
  }else if(scenario==='unexpected_program'){
    status.remote_control.program.program_id='d'.repeat(24);context.ui.render(clone(status));await settle();
    assert.deepEqual(posts.map(p=>p.action),['program_start','land']);
  }else throw new Error('Unknown scenario '+scenario);
})().catch(error=>{console.error(error);process.exitCode=1});
'''


def run(scenario,**options):
    result=subprocess.run([NODE,'-e',SCRIPT,str(SOURCE)],
        input=json.dumps({'scenario':scenario,'program':PROGRAM,**options}),
        capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr


@pytest.mark.parametrize('scenario',[
    'grounded','keepalive','manual','release','blur','generation_changed','unexpected_program',
    'yaw_profile','vertical_profile','channel_limit','stale_status','no_image','no_focus','no_hold',
    'land_over_start','land_over_renew','uncertain_start','missing_identifier','oversize','file_race',
    'program_over_renew','takeoff_over_renew',
])
def test_page_program_authority_and_land_priority(scenario):run(scenario)


@pytest.mark.parametrize('program',[
    PROGRAM,{'version':1,'finish':'land','steps':[{'kind':'hold','seconds':.2}]},
    {'version':1,'finish':'land','steps':[{'kind':'rotate','degrees':360,'channel':-200,'timeout_seconds':5}]},
    {'version':1,'finish':'land','steps':[{'kind':'rotate','degrees':360,'channel':400,'timeout_seconds':5}]},
    {'version':1,'finish':'land','steps':[{'kind':'rotate','degrees':360,'channel':600,'minimum_channel':600,'timeout_seconds':5}]},
    {},[],{'version':True,'finish':'land','steps':[{'kind':'hold','seconds':1}]},
    {'version':1,'finish':'hover','steps':[{'kind':'hold','seconds':1}]},
    {'version':1,'finish':'land','steps':[]},
    *({'version':1,'finish':'land','steps':[step]} for step in [
        {'kind':'hold','seconds':0}, {'kind':'hold','seconds':1,'extra':True},
        {'kind':'translate','seconds':1,'right':0,'forward':0,'up':0},
        {'kind':'translate','seconds':1,'right':251,'forward':0,'up':0},
        {'kind':'translate','seconds':1,'right':True,'forward':0,'up':0},
        {'kind':'rotate','degrees':90,'channel':20,'timeout_seconds':1},
        {'kind':'rotate','degrees':90,'channel':601,'timeout_seconds':1},
        *({'kind':'rotate','degrees':90,'channel':400,'timeout_seconds':1,'minimum_channel':value}
          for value in [None,True,39,401]),
        {'kind':'rotate','degrees':400,'channel':100,'timeout_seconds':1},
        {'kind':'rotate','degrees':90,'channel':100,'timeout_seconds':26},
    ]),
    {'version':1,'finish':'land','steps':[{'kind':'hold','seconds':5}]*6},
])
def test_browser_validator_matches_authoritative_program_schema(program):
    try:MotionProgram.from_dict(program);valid=True
    except ValueError:valid=False
    run('validate',program=program,valid=valid)
