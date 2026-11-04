"""Offline birdview records: raw proximity values, never flight authority.

The saved SDK's perception_birdview_distance_push callback establishes the
u16 layout and horizontal/up/down model fields. Units, angular calibration
and special-value meanings are deliberately separate from that wire layout.
"""
from dataclasses import dataclass
import hashlib
import html
import json
import os
from pathlib import Path
import stat
import struct

from .duml import find_packets
from .transport import describe_transport
from .wlm_discovery import unpack_forwarded


@dataclass(frozen=True)
class BirdviewDistances:
    header_hex: str
    up_raw: int
    down_raw: int
    horizontal_raw: tuple[int, ...]


def decode_birdview(payload: bytes) -> BirdviewDistances:
    if not isinstance(payload,bytes) or not 10<=len(payload)<=728:
        raise ValueError('birdview_payload_length_invalid')
    up,down,count=struct.unpack_from('<3H',payload,2)
    if not 1<=count<=360 or len(payload)!=8+2*count:
        raise ValueError('birdview_count_or_length_invalid')
    return BirdviewDistances(payload[:2].hex(),up,down,
        struct.unpack_from('<'+'H'*count,payload,8))


def read_birdview_capture(path,*,maximum_frames=5000):
    """Read bounded, CRC-checked complete frames; no reassembly or live I/O."""
    if type(maximum_frames) is not int or not 1<=maximum_frames<=5000:
        raise ValueError('birdview_frame_limit_invalid')
    frames=[];digest=hashlib.sha256();rejected=observed=read_bytes=0;last_stamp=-1
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC|os.O_NONBLOCK)
    with os.fdopen(fd,'rb') as source:
        before=os.fstat(source.fileno())
        if not stat.S_ISREG(before.st_mode) or before.st_size>512*1024*1024:
            raise ValueError('birdview_capture_size_invalid')
        while line:=source.readline(256*1024+1):
            if len(line)>256*1024:raise ValueError('birdview_capture_line_too_large')
            read_bytes+=len(line)
            if read_bytes>512*1024*1024:raise ValueError('birdview_capture_size_invalid')
            digest.update(line)
            try:
                row=json.loads(line)
                if not isinstance(row,dict):raise ValueError
                if row.get('direction')!='rx':continue
                stamp=row['monotonic_ns']
                if type(stamp) is not int or not 0<=stamp<2**63:raise ValueError
                raw_hex=row['raw_hex']
                if not isinstance(raw_hex,str) or len(raw_hex)>131070:raise ValueError
                raw=bytes.fromhex(raw_hex)
                if describe_transport(raw)[0]['type']==2:continue
            except (ValueError,KeyError,TypeError):
                rejected+=1;continue
            for _,packet,_ in find_packets(raw):
                if (packet.cmd_set,packet.cmd_id)==(0x51,1):
                    try:packet=unpack_forwarded(packet.payload).packet
                    except ValueError:continue
                if (packet.cmd_set,packet.cmd_id)!=(0x24,8):continue
                try:decoded=decode_birdview(packet.payload)
                except ValueError:rejected+=1;continue
                if stamp<last_stamp:rejected+=1;continue
                last_stamp=stamp
                observed+=1
                if len(frames)>=maximum_frames:continue
                frames.append({'at':stamp/1e9,'sender':packet.sender,'receiver':packet.receiver,
                    'header_hex':decoded.header_hex,'up_raw':decoded.up_raw,'down_raw':decoded.down_raw,
                    'horizontal_raw':list(decoded.horizontal_raw)})
        after=os.fstat(source.fileno())
    if (before.st_size,before.st_mtime_ns,before.st_ctime_ns)!=(after.st_size,after.st_mtime_ns,after.st_ctime_ns):
        raise ValueError('birdview_capture_changed')
    if not frames:raise ValueError('birdview_capture_has_no_supported_frames')
    start=frames[0]['at']
    for frame in frames:frame['seconds']=frame['at']-start
    return {'format':'neo2-control/birdview-raw/v1','source':str(Path(path).absolute()),
        'sha256':digest.hexdigest(),'units':'raw SDK integers; metric scale not asserted',
        'angles':'DJI app display: 360*i/(n-1)-90 degrees; physical orientation and sampling not calibrated',
        'special_values':'60000 can be hidden for inspection; not proof of free space',
        'scope':'complete CRC-checked frames in non-video datagrams, without stream reassembly or source-ownership attribution',
        'observed_frames':observed,'retained_frames':len(frames),'truncated':observed>len(frames),
        'rejected_records':rejected,'frames':frames}


def write_birdview_report(capture,output):
    data=read_birdview_capture(capture)
    encoded=json.dumps(data,ensure_ascii=False,separators=(',',':'),allow_nan=False)
    encoded=encoded.replace('&','\\u0026').replace('<','\\u003c').replace('>','\\u003e')
    title=html.escape(Path(capture).name)
    prefix,_,suffix=_PAGE.partition('__DATA__')
    page=prefix.replace('__SOURCE__',title)+encoded+suffix
    target=Path(output)
    fd=os.open(target,os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
    with os.fdopen(fd,'w',encoding='utf-8') as stream:stream.write(page)
    return {key:value for key,value in data.items() if key!='frames'}|{'output':str(target.absolute())}


_PAGE='''<!doctype html><html lang="fr"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Neo 2 — obstacles enregistrés</title>
<style>body{font:16px system-ui;background:#101924;color:#e2edf5;margin:0;padding:24px;max-width:1040px;margin:auto}h1{font-size:25px}p{line-height:1.5;color:#b7cbd9}main{display:flex;flex-wrap:wrap;gap:24px}canvas{max-width:100%;width:560px;background:#142333;border-radius:16px}aside{flex:1;min-width:260px}label{display:block;margin:20px 0}input[type=range]{width:100%}button{background:#255a76;border:0;color:white;padding:10px 18px;border-radius:7px;cursor:pointer}output{display:block;white-space:pre-line;line-height:1.8}small{color:#a7bccb}code{word-break:break-all}</style>
<h1>Obstacles — données brutes enregistrées</h1>
<p>Un anneau horizontal, plus une valeur au-dessus et une en dessous.
Ce relevé ne constitue pas un scan 3D ni une carte de navigation.</p>
<main><canvas id="radar" width="640" height="640" aria-label="Relevé horizontal enregistré, en unités brutes"></canvas><aside>
<button id="play">Lire</button><label>Message <span id="index"></span><input id="timeline" type="range" min="0" value="0"></label>
<label>Rayon affiché, en unités brutes<input id="radius" type="range" min="500" max="10000" step="100" value="4000"></label>
<label><input id="mask" type="checkbox" checked> Masquer le code 60000</label>
<output id="details"></output>
<p>La disposition angulaire reprend celle de l’application : l’index 0 est en haut.
Les unités métriques et les codes spéciaux restent à confirmer.</p>
<p>Une zone sans point ne signifie pas que l’espace est libre.</p>
<small>Source : __SOURCE__<br>SHA-256 : <code id="digest"></code><br><span id="scope"></span></small>
</aside></main><script id="dataset" type="application/json">__DATA__</script>
<script>
'use strict';
const data=JSON.parse(document.getElementById('dataset').textContent),frames=data.frames;
const el=id=>document.getElementById(id),slider=el('timeline'),canvas=el('radar'),ctx=canvas.getContext('2d');
slider.max=frames.length-1;
slider.value=Math.max(0,frames.findIndex(frame=>frame.horizontal_raw.some(value=>value!==60000&&value<=4000)));
el('digest').textContent=data.sha256;
el('scope').textContent=`${data.retained_frames} messages affichés / ${data.observed_frames} observés. Destinataires dupliqués possibles.`;
let playing=false,playStarted=0,playFrom=0;
function draw(){
 const frame=frames[Number(slider.value)],range=Number(el('radius').value),cx=320,cy=320,r=270;
 ctx.clearRect(0,0,640,640);ctx.strokeStyle='#355064';ctx.lineWidth=1;ctx.font='14px system-ui';ctx.fillStyle='#a8c1d1';
 for(let i=1;i<=4;i++){ctx.beginPath();ctx.arc(cx,cy,r*i/4,0,Math.PI*2);ctx.stroke();ctx.fillText(String(Math.round(range*i/4)),cx+6,cy-r*i/4+16)}
 ctx.beginPath();ctx.moveTo(cx-r,cy);ctx.lineTo(cx+r,cy);ctx.moveTo(cx,cy-r);ctx.lineTo(cx,cy+r);ctx.stroke();
 ctx.fillText('index 0 · direction non étalonnée',195,25);
 let visible=0,filtered=0,outside=0;ctx.fillStyle='#68d8ba';
 frame.horizontal_raw.forEach((value,index)=>{
  if(el('mask').checked&&value===60000){filtered++;return}
  if(value>range){outside++;return}
  const a=index*2*Math.PI/Math.max(1,frame.horizontal_raw.length-1)-Math.PI/2,d=value/range*r;
  ctx.beginPath();ctx.arc(cx+Math.cos(a)*d,cy+Math.sin(a)*d,2.3,0,Math.PI*2);ctx.fill();visible++;
 });
 ctx.fillStyle='#ffcb75';ctx.beginPath();ctx.arc(cx,cy,5,0,Math.PI*2);ctx.fill();
 el('index').textContent=`${Number(slider.value)+1} / ${frames.length}`;
 el('details').textContent=`Temps relatif : ${frame.seconds.toFixed(2)} / ${frames.at(-1).seconds.toFixed(2)} s\nHaut : ${frame.up_raw} brut\nBas : ${frame.down_raw} brut\nÉchantillons horizontaux : ${frame.horizontal_raw.length}\nPoints visibles : ${visible} · hors rayon : ${outside}\nCode 60000 masqué : ${filtered}\nEn-tête : ${frame.header_hex} · destinataire : ${frame.receiver}\nRayon : ${range} unités brutes`;
}
function resetClock(){playStarted=performance.now();playFrom=frames[Number(slider.value)].seconds}
for(const id of ['timeline','radius','mask'])el(id).addEventListener('input',()=>{resetClock();draw()});
el('play').onclick=()=>{
 if(!playing&&Number(slider.value)===frames.length-1)slider.value=0;
 playing=!playing;resetClock();el('play').textContent=playing?'Pause':'Lire';draw();
};
setInterval(()=>{
 if(!playing)return;
 const target=playFrom+(performance.now()-playStarted)/1000;
 while(Number(slider.value)<frames.length-1&&frames[Number(slider.value)+1].seconds<=target)slider.value=Number(slider.value)+1;
 if(Number(slider.value)===frames.length-1){playing=false;el('play').textContent='Lire'}
 draw();
},40);
draw();
</script></html>'''
