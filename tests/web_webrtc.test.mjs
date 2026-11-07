import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const code = await readFile(new URL('../src/neo2_control/web/webrtc.js', import.meta.url), 'utf8');
const {WhepPreview, ReceiverDiagnostics} = await import(`data:text/javascript;base64,${Buffer.from(code).toString('base64')}`);
const settle = async () => { for (let i=0;i<12;i++) await Promise.resolve(); };
const base = {connected:true,live:true,enabled:true,ready:true,generation:'bridge-a',sessionGeneration:'session-a',token:'fixture-token'};
class Peer extends EventTarget {
  static peers=[];
  constructor(config){super();this.config=config;this.iceGatheringState='complete';this.connectionState='new';this.iceConnectionState='new';Peer.peers.push(this);}
  addTransceiver(kind,options){this.transceiver={kind,...options};}
  async createOffer(){return {type:'offer',sdp:'v=0\r\nfixture-offer\r\n'};}
  async setLocalDescription(value){this.localDescription=value;}
  async setRemoteDescription(value){this.remoteDescription=value;}
  close(){this.closed=true;}
}
function setup({fetchImpl,PeerConnection=Peer,callbacks=true}={}){
  let time=0;
  const calls=[];
  const video={readyState:2,paused:false,ended:false,currentTime:0,srcObject:null,
    pause(){this.paused=true;},play(){this.paused=false;return Promise.resolve();}};
  if(callbacks){video.requestVideoFrameCallback=cb=>{video.callback=cb;return 1;};video.cancelVideoFrameCallback=()=>{video.callback=null;};}
  const fetcher=fetchImpl|| (async (path,options)=>{calls.push({path,options});return options.method==='DELETE'?new Response(null,{status:204}):new Response('v=0\r\nfixture-answer\r\n',{status:201,headers:{Location:'/api/camera/webrtc/abcdef12'}});});
  const client=new WhepPreview({video,fetchImpl:fetcher,PeerConnection,clock:()=>time});
  clearInterval(client.timer);
  return {client,video,calls,advance:ms=>{time+=ms;client._tick();},present:frames=>{video.callback(0,{presentedFrames:frames,mediaTime:frames/30});},track:()=>client.session.pc.ontrack({track:{kind:'video'},streams:[{}]})};
}

test('connects only to a fresh ready backend with local recvonly SDP and no ICE services',async()=>{
  const f=setup();
  try{
    f.client.update({...base,ready:false});await settle();assert.equal(f.calls.length,0);
    f.client.update(base);await settle();
    assert.equal(f.calls.length,1);assert.equal(f.client.snapshot.fresh,false);
    assert.equal(f.client.snapshot.state,'connecting');
    assert.deepEqual(f.client.session.pc.config.iceServers,[]);
    assert.deepEqual(f.client.session.pc.transceiver,{kind:'video',direction:'recvonly'});
    assert.equal(f.calls[0].path,'/api/camera/webrtc');
    assert.equal(f.calls[0].options.headers['Content-Type'],'application/sdp');
    assert.equal(f.calls[0].options.headers['X-Neo2-Token'],base.token);
    assert.equal(f.calls[0].options.redirect,'error');
    assert.equal(f.client.session.pc.remoteDescription.type,'answer');
  }finally{f.client.stop();}
});

test('labels live only after a presented frame, then discards frozen video even if its clock advances',async()=>{
  const f=setup();
  try{
    f.client.update(base);await settle();f.track();await settle();
    assert.equal(f.client.snapshot.fresh,false);
    f.present(1);assert.equal(f.client.snapshot.fresh,true);
    f.video.currentTime=100;f.advance(3001);
    assert.equal(f.client.snapshot.fresh,false);
    assert.equal(f.client.snapshot.reason,'stalled');
    assert.equal(f.video.srcObject,null);
    assert.equal(f.calls.filter(c=>c.options.method==='DELETE').length,1);
  }finally{f.client.stop();}
});

test('uses advancing currentTime when frame callbacks are unsupported',async()=>{
  const f=setup({callbacks:false});
  try{
    f.client.update(base);await settle();f.track();f.advance(250);
    assert.equal(f.client.snapshot.fresh,false);
    f.video.currentTime=.25;f.advance(250);assert.equal(f.client.snapshot.fresh,true);
    f.advance(3001);assert.equal(f.client.snapshot.reason,'stalled');
  }finally{f.client.stop();}
});

test('generation or token change closes old playback and deletes using its original token',async()=>{
  const f=setup();
  try{
    f.client.update(base);await settle();const old=f.client.session.pc;
    f.client.update({...base,generation:'bridge-b',token:'replacement-token'});await settle();
    assert.equal(old.closed,true);assert.equal(f.client.snapshot.attempts,1);
    const cleanup=f.calls.find(c=>c.options.method==='DELETE');
    assert.equal(cleanup.options.headers['X-Neo2-Token'],'fixture-token');
    assert.equal(f.calls.filter(c=>c.options.method==='POST').length,2);
    f.client.update({...base,generation:'bridge-b',token:'replacement-token',connected:false});
    assert.equal(f.client.session,null);assert.equal(f.client.snapshot.fresh,false);
  }finally{f.client.stop();}
});

test('JPEG selection tears down and stops WHEP requests',async()=>{
  const f=setup();
  try{
    f.client.update(base);await settle();f.client.setMode('jpeg');f.advance(30000);
    assert.equal(f.client.session,null);assert.equal(f.client.snapshot.reason,'jpeg_selected');
    assert.equal(f.calls.filter(c=>c.options.method==='POST').length,1);
    assert.equal(f.calls.filter(c=>c.options.method==='DELETE').length,1);
  }finally{f.client.stop();}
});

test('failed negotiation has a cooldown and at most three attempts per generation',async()=>{
  let posts=0;
  const f=setup({fetchImpl:async()=>{posts++;return new Response('unavailable',{status:503});}});
  try{
    f.client.update(base);await settle();assert.equal(posts,1);
    f.advance(2499);await settle();assert.equal(posts,1);
    f.advance(1);await settle();assert.equal(posts,2);
    f.advance(2500);await settle();assert.equal(posts,3);
    f.advance(60000);await settle();assert.equal(posts,3);
    f.client.update({...base,generation:'bridge-b'});await settle();assert.equal(posts,4);
  }finally{f.client.stop();}
});

test('obsolete offer replies are cleaned up without applying their SDP',async()=>{
  let resolve;
  const calls=[];
  const f=setup({fetchImpl:async(path,options)=>{calls.push({path,options});if(options.method==='DELETE')return new Response(null,{status:204});return await new Promise(r=>{resolve=r;});}});
  try{
    f.client.update(base);await settle();const old=f.client.session.pc;
    f.client.setMode('jpeg');
    resolve(new Response('old-answer',{status:201,headers:{Location:'/api/camera/webrtc/abcd'}}));await settle();
    assert.equal(old.remoteDescription,undefined);
    assert.equal(calls.filter(c=>c.options.method==='DELETE').length,1);
  }finally{f.client.stop();}
});

test('rejects foreign cleanup URLs and oversized SDP answers',async()=>{
  for(const response of [new Response('answer',{status:201,headers:{Location:'https://foreign.invalid/session'}}),new Response('x'.repeat(65537),{status:201,headers:{Location:'/api/camera/webrtc/abcd'}})]){
    const calls=[];const f=setup({fetchImpl:async(path,options)=>{calls.push({path,options});return options.method==='DELETE'?new Response(null,{status:204}):response;}});
    try{
      f.client.update(base);await new Promise(r=>setTimeout(r,10));
      assert.equal(f.client.snapshot.fresh,false);assert.equal(f.client.snapshot.state,'fallback');
      assert.ok(calls.every(c=>c.path.startsWith('/api/camera/webrtc')));
    }finally{f.client.stop();}
  }
});

test('unsupported browsers leave JPEG fallback without requests',()=>{
  const f=setup({PeerConnection:null});
  try{f.client.update(base);assert.equal(f.client.snapshot.reason,'unsupported');assert.equal(f.calls.length,0);}finally{f.client.stop();}
});


test('a connected peer without any presented frame times out and falls back',async()=>{
  const f=setup();
  try{
    f.client.update(base);await settle();f.track();await settle();
    f.advance(12001);assert.equal(f.client.snapshot.reason,'no_frame');
    assert.equal(f.client.snapshot.fresh,false);assert.equal(f.video.srcObject,null);
  }finally{f.client.stop();}
});

const stats = (overrides={}) => new Map([['video', {type:'inbound-rtp',kind:'video',id:'private-track',
  framesDecoded:100,jitterBufferDelay:5,jitterBufferEmittedCount:100,totalDecodeTime:1,
  frameWidth:1280,frameHeight:720,...overrides}]]);

test('receiver diagnostics measure interval averages rather than cumulative lifetime averages',()=>{
  const d=new ReceiverDiagnostics();d.reset(1000);d.begin(1000);
  d.sample(stats(),{totalVideoFrames:100,droppedVideoFrames:2},1000);
  assert.equal(d.snapshot(1000).decoded_fps,null);
  d.sample(stats({framesDecoded:130,jitterBufferDelay:6.5,jitterBufferEmittedCount:130,totalDecodeTime:1.06}),
    {totalVideoFrames:130,droppedVideoFrames:3},2000);
  const sample=d.snapshot(2000);
  assert.equal(sample.decoded_fps,30);assert.equal(sample.browser_buffer_ms,50);
  assert.ok(Math.abs(sample.browser_decode_ms-2)<1e-10);
  assert.equal(sample.browser_dropped_frames,3);
  assert.equal(d.export(2000,'live_aircraft').camera_to_screen_delay_ms,null);
});

test('counter resets, stream changes and long sample gaps discard invalid intervals',()=>{
  const d=new ReceiverDiagnostics();d.begin(0);
  d.sample(stats(),null,0);
  d.sample(stats({framesDecoded:1,jitterBufferDelay:.01,jitterBufferEmittedCount:1,totalDecodeTime:.01}),null,1000);
  assert.equal(d.snapshot(1000).decoded_fps,null);assert.equal(d.snapshot(1000).browser_buffer_ms,null);
  d.sample(stats({id:'replacement-track'}),null,2000);
  assert.equal(d.snapshot(2000).decoded_fps,null);
  d.sample(stats({id:'replacement-track',framesDecoded:160}),null,20000);
  assert.equal(d.snapshot(20000).decoded_fps,null);
  d.begin(20000);d.sample(stats({framesDecoded:200}),null,21000);
  assert.equal(d.snapshot(21000).decoded_fps,null);
});

test('missing or invalid measurements remain unavailable and stale values do not look current',()=>{
  const d=new ReceiverDiagnostics();d.begin(0);
  d.sample(stats({framesDecoded:undefined,jitterBufferDelay:'5',totalDecodeTime:NaN}),{},0);
  const first=d.snapshot(0);assert.equal(first.frames_decoded,null);assert.equal(first.browser_dropped_frames,null);
  assert.equal(first.browser_decode_ms,null);
  assert.equal(d.snapshot(3001).available,false);
  d.end();assert.equal(d.snapshot(0).available,false);
  const ambiguous=stats();ambiguous.set('second',{...ambiguous.get('video'),id:'other'});
  d.sample(ambiguous,{},4000);assert.equal(d.history.length,1);
});

test('diagnostic export is bounded, detached and contains no raw browser identifiers or addresses',()=>{
  const d=new ReceiverDiagnostics();d.begin(0);
  const report=stats({id:'secret-track-id',address:'192.0.2.1',ssrc:12345,codecId:'private-codec'});
  report.set('candidate',{type:'local-candidate',address:'198.51.100.1',usernameFragment:'private-ufrag'});
  for(let i=0;i<130;i++)d.sample(report,{totalVideoFrames:i,droppedVideoFrames:0},i*1000);
  const value=d.export(129000,'private-ssid');
  assert.equal(value.source_mode,'unavailable');assert.equal(value.samples.length,120);
  const text=JSON.stringify(value);
  for(const secret of ['secret-track-id','192.0.2.1','12345','private-codec','198.51.100.1','private-ufrag','private-ssid'])assert.ok(!text.includes(secret));
  value.samples[0].attempt=100;assert.equal(d.history[0].attempt,1);
  d.reset(130000);assert.equal(d.export(130000,'synthetic').samples.length,0);
});

test('receiver statistics never block playback and obsolete pending reports are ignored',async()=>{
  const f=setup();let resolve,calls=0;
  try{
    f.client.update(base);await settle();f.track();f.present(1);
    f.client.session.pc.getStats=()=>{calls++;return new Promise(r=>{resolve=r;});};
    f.advance(1000);f.advance(1000);assert.equal(calls,1);
    f.client.update({...base,generation:'new-bridge'});await settle();
    resolve(stats());await settle();assert.equal(f.client.diagnostics.sample_count,0);
    f.track();f.present(1);
    f.client.session.pc.getStats=async()=>{throw new Error('private browser failure');};
    f.advance(1000);await settle();assert.equal(f.client.snapshot.fresh,true);
    assert.equal(f.client.diagnostics.sample_count,0);
  }finally{f.client.stop();}
});

test('connection loss clears the old peer, respects cooldown and recovers on a fresh frame',async()=>{
  const f=setup();
  try{
    f.client.update(base);await settle();f.track();f.present(1);
    const old=f.client.session.pc;old.connectionState='disconnected';old.onconnectionstatechange();
    assert.equal(f.client.snapshot.fresh,false);assert.equal(old.closed,true);
    f.advance(2499);await settle();assert.equal(f.client.session,null);
    f.advance(1);await settle();assert.notEqual(f.client.session.pc,old);
    assert.equal(f.client.snapshot.fresh,false);
    f.track();f.present(1);assert.equal(f.client.snapshot.fresh,true);
    assert.equal(f.calls.filter(c=>c.options.method==='POST').length,2);
    assert.equal(f.calls.filter(c=>c.options.method==='DELETE').length,1);
  }finally{f.client.stop();}
});
