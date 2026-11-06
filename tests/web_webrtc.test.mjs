import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const code = await readFile(new URL('../src/neo2_control/web/webrtc.js', import.meta.url), 'utf8');
const {WhepPreview} = await import(`data:text/javascript;base64,${Buffer.from(code).toString('base64')}`);
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
