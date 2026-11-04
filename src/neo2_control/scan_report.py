"""Standalone orbit view of a saved sparse scan, with its original units."""
import json
import os
from pathlib import Path

from .reconstruction import _validate_scan


def write_scan_report(scan,output):
    metric=isinstance(scan,dict) and scan.get('format')=='neo2-control/metric-scan/v1'
    dense=isinstance(scan,dict) and scan.get('format') in ('neo2-control/depth-scan/v1','neo2-control/depth-scan/v2')
    memory=isinstance(scan,dict) and scan.get('format')=='neo2-control/spatial-memory/v1'
    atlas=isinstance(scan,dict) and scan.get('format')=='neo2-control/metric-atlas/v1'
    if atlas:
        from .metric_atlas import atlas_geometry
        scan=atlas_geometry(scan)
    elif memory:
        from .spatial_memory import memory_geometry
        scan=memory_geometry(scan)
    elif dense:
        from .depth_scan import depth_geometry
        scan=depth_geometry(scan)
    elif metric:
        from .metric_reference import metric_geometry
        scan=metric_geometry(scan)
    else:
        _validate_scan(scan)
    fields=('points','rgb','second_world_to_camera_R','second_world_to_camera_t',
        'second_camera_center','camera_model','coordinate_frame','distance_unit',
        'image_sha256','metric_scale_verified','free_space_observed',
        'reprojection_error_px','parallax_degrees')
    dataset={key:scan[key] for key in fields if key in scan}
    for key in ('first_world_to_camera_R','first_world_to_camera_t','reference_id','scale_source',
            'kind','depth_m','minimum_depth_m','maximum_depth_m','grid_width','grid_height',
            'full_resolution_valid_fraction','grid_states','grid_shape','grid_lower',
            'resolution_m','state_counts','source_count','extension_count','new_point_count',
            'keyframe_poses'):
        if key in scan:dataset[key]=scan[key]
    encoded=json.dumps(dataset,ensure_ascii=False,allow_nan=False,separators=(',',':'))
    encoded=encoded.replace('&','\\u0026').replace('<','\\u003c').replace('>','\\u003e')
    prefix,_,suffix=_PAGE.partition('__DATA__');page=prefix+encoded+suffix
    fd=os.open(output,os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
    with os.fdopen(fd,'w',encoding='utf-8') as stream:stream.write(page)
    return {'output':str(Path(output).absolute()),'point_count':len(scan['points']),
        'distance_unit':scan['distance_unit'],'metric_scale_verified':False,'free_space_observed':False}


_PAGE='''<!doctype html><html lang="fr"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Neo 2 — reconstruction 3D enregistrée</title>
<style>body{font:16px system-ui;background:#101924;color:#e2edf5;padding:24px;max-width:1050px;margin:auto}h1{font-size:25px}p{line-height:1.5;color:#b7cbd9}.note{padding:16px;background:#342e24;border-radius:10px;color:#ffd38b}canvas{width:100%;background:#142333;border-radius:16px;touch-action:none;cursor:grab}label{display:block;margin:20px 0}input{width:min(500px,100%)}button{background:#255a76;border:0;color:white;padding:10px 18px;border-radius:7px;cursor:pointer}output{white-space:pre-line;display:block;line-height:1.8}code{word-break:break-all;color:#b7cbd9}details{margin-top:25px}small{color:#a7bccb}</style>
<style>#memory-slice{height:420px;object-fit:contain;cursor:default}</style>
<h1>Reconstruction 3D enregistrée</h1>
<p id="summary"></p>
<p class="note" id="scale-note">Échelle non étalonnée : une unité correspond à la distance entre les deux prises de vue, pas à un mètre. Les zones sans points restent inconnues ; ce relevé ne valide pas encore « 1 m libre ».</p>
<section id="depth-section" hidden><p>Profondeur : rouge près, bleu loin, gris sans mesure. Il s’agit de surfaces observées, pas d’un passage déclaré libre.</p>
<canvas id="depth-map" aria-label="Profondeurs et zones sans mesure"></canvas><output id="depth-value"></output></section>
<section id="memory-section" hidden><p>Gris : inconnu. Bleu : volume observé avant une surface, sous les hypothèses du modèle. Orange : surface ou marge.</p>
<label>Coupe selon l’axe <select id="slice-axis"><option value="0">x</option><option value="1">y</option><option value="2" selected>z</option></select></label>
<input id="slice-index" type="range" min="0" value="0" step="1"><output id="slice-label"></output>
<canvas id="memory-slice" aria-label="Coupe des cellules observées et inconnues"></canvas></section>
<canvas id="scene" width="1000" height="650" aria-label="Points 3D enregistrés et prises de vue"></canvas>
<p>Glisser pour orienter la vue ; molette ou curseur pour zoomer. Les cadres représentent les prises de vue.</p>
<label>Zoom <input id="zoom" type="range" min="0.5" max="4" value="1" step="0.05"></label>
<button id="reset">Recentrer</button><output id="quality"></output>
<details><summary>Provenance et calibration</summary><p id="calibration"></p><code id="identity"></code></details>
<script id="dataset" type="application/json">__DATA__</script><script>
'use strict';
const data=JSON.parse(document.getElementById('dataset').textContent),el=id=>document.getElementById(id);
const canvas=el('scene'),ctx=canvas.getContext('2d'),points=data.points;
const center=points.length?[0,1,2].map(axis=>points.reduce((sum,p)=>sum+p[axis],0)/points.length):[0,0,0];
const extent=Math.max(1,...points.map(p=>Math.hypot(...p.map((v,i)=>v-center[i]))),Math.hypot(...center));
let yaw=.35,pitch=-.2,drag=null;
const median=values=>[...values].sort((a,b)=>a-b)[Math.floor(values.length/2)];
el('summary').textContent=`${points.length} points triangulés depuis deux images et conservés sur le PC. Repère de la première caméra : x vers la droite, y vers le bas, z vers l’avant.`;
if(data.kind==='atlas'){
 el('summary').textContent=`${points.length} points de repère mémorisés, dont ${data.new_point_count} ajoutés depuis ${data.extension_count} paire(s) de nouvelles vues. Coordonnées en mètres dans le repère initial.`;
 el('scale-note').textContent='Les nouvelles poses sont estimées sur les points connus ; le repère et l’échelle de départ sont conservés. Dérive et précision physique à vérifier. Les zones sans points restent inconnues.';
}else if(data.kind==='memory'){
 el('summary').textContent=`Mémoire PC de ${data.source_count} relevé(s) : ${data.state_counts.surface_or_margin} cellules de surface ou marge, ${data.state_counts.model_free} cellules observées sous le modèle et ${data.state_counts.unknown} cellules inconnues. La vue 3D affiche ${points.length} cellules de surface.`;
 el('scale-note').textContent='Mémoire historique : scène actuelle, précision des distances et marges physiques à vérifier. Les cellules bleues ne sont pas une autorisation de vol. Le repère enregistré n’est pas automatiquement celui de la gravité.';
}else if(data.kind==='depth'){
 el('summary').textContent=`${points.length} mesures de profondeur conservées sur le PC, en mètres dans le repère enregistré. Les deux vues proviennent de la même caméra déplacée.`;
 el('scale-note').textContent='Échelle issue du damier mesuré et poses estimées. Précision physique et scène immobile à vérifier. Les zones sans mesure restent inconnues ; ce relevé ne valide pas encore « 1 m libre ».';
}else if(data.distance_unit==='m'){
 el('summary').textContent=`${points.length} points conservés sur le PC, en mètres dans le repère du damier « ${data.reference_id} ». Ce damier doit rester fixe ; le repère n’est pas celui de la gravité.`;
 el('scale-note').textContent='Échelle issue de la taille physique déclarée du damier. Précision réelle à vérifier. Les zones sans points restent inconnues ; ce relevé ne valide pas encore « 1 m libre ».';
}
el('quality').textContent=data.kind==='memory'?`Taille des cellules : ${(100*data.resolution_m).toFixed(1)} cm\nLes coupes montrent toutes les cellules ; la vue 3D peut échantillonner les surfaces. Un trajet doit tenir compte de tout le volume du drone.`:data.kind==='depth'?`Mesures admises à pleine définition : ${(100*data.full_resolution_valid_fraction).toFixed(1)} %\nLes autres pixels restent sans profondeur admise. La cohérence visuelle ne valide pas la précision physique.`:`Parallaxe médiane : ${median(data.parallax_degrees).toFixed(2)}°\nErreur de reprojection maximale : ${Math.max(...data.reprojection_error_px).toFixed(2)} pixel\nCes contrôles de géométrie ne valident pas la précision physique.`;
if(data.kind==='atlas')el('quality').textContent='Chaque ajout conserve ses images sources, ses poses appuyées sur les points précédents et ses observations de triangulation. La précision du suivi et la fermeture d’un tour complet restent à valider.';
el('calibration').textContent=`Calibration : ${data.camera_model.status==='candidate'?'estimée, à confirmer':'déclarée calibrée'} · ${data.camera_model.provenance}`;
el('identity').textContent=`Repère conservé : ${data.coordinate_frame}\nImages : ${data.image_sha256.join(' / ')}`;
if(data.kind==='depth'){
 el('depth-section').hidden=false;const map=el('depth-map');map.width=data.grid_width;map.height=data.grid_height;
 const paint=map.getContext('2d'),raster=paint.createImageData(map.width,map.height);
 for(let y=0;y<map.height;y++)for(let x=0;x<map.width;x++){
  const z=data.depth_m[y][x],i=4*(y*map.width+x);let color=[30,44,58];
  if(z!==null){const f=Math.max(0,Math.min(1,Math.log(z/data.minimum_depth_m)/Math.log(data.maximum_depth_m/data.minimum_depth_m)));color=[255*(1-f),100+80*(1-Math.abs(2*f-1)),255*f]}
  raster.data.set([...color,255],i);
 }
 paint.putImageData(raster,0,0);map.style.cursor='crosshair';
 map.addEventListener('pointermove',event=>{const r=map.getBoundingClientRect(),x=Math.floor((event.clientX-r.left)*map.width/r.width),y=Math.floor((event.clientY-r.top)*map.height/r.height);const z=data.depth_m[y]?.[x];el('depth-value').textContent=z==null?'Sans mesure admise':`Profondeur : ${z.toFixed(3)} m`});
}
if(data.kind==='memory'){
 el('memory-section').hidden=false;const map=el('memory-slice'),paint=map.getContext('2d'),colors=[[30,44,58],[69,126,165],[245,168,90]];
 const drawSlice=()=>{
  const axis=Number(el('slice-axis').value),others=[0,1,2].filter(i=>i!==axis),index=Number(el('slice-index').value);
  map.width=data.grid_shape[others[0]];map.height=data.grid_shape[others[1]];
  const raster=paint.createImageData(map.width,map.height);
  for(let y=0;y<map.height;y++)for(let x=0;x<map.width;x++){
   const p=[0,0,0];p[axis]=index;p[others[0]]=x;p[others[1]]=y;
   const state=data.grid_states[(p[0]*data.grid_shape[1]+p[1])*data.grid_shape[2]+p[2]];
   raster.data.set([...colors[state],255],4*(y*map.width+x));
  }
  paint.putImageData(raster,0,0);map.style.imageRendering='pixelated';
  el('slice-label').textContent=`${'xyz'[axis]} = ${(data.grid_lower[axis]+(index+.5)*data.resolution_m).toFixed(2)} m`;
 };
 const axisChanged=()=>{el('slice-index').max=data.grid_shape[Number(el('slice-axis').value)]-1;el('slice-index').value=Math.floor(Number(el('slice-index').max)/2);drawSlice()};
 el('slice-axis').addEventListener('change',axisChanged);el('slice-index').addEventListener('input',drawSlice);axisChanged();
}
function project(point){
 let [x,y,z]=point.map((value,index)=>value-center[index]);
 const x1=Math.cos(yaw)*x+Math.sin(yaw)*z,z1=-Math.sin(yaw)*x+Math.cos(yaw)*z;
 const y1=Math.cos(pitch)*y-Math.sin(pitch)*z1,z2=Math.sin(pitch)*y+Math.cos(pitch)*z1;
 const depth=z2+extent*2.7/Number(el('zoom').value);
 if(depth<=.05)return null;
 return [500+x1/depth*850,325+y1/depth*850,depth];
}
function segment(a,b,color,label){
 const p=project(a),q=project(b);if(!p||!q)return;
 ctx.strokeStyle=color;ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1]);ctx.stroke();
 if(label){ctx.fillStyle=color;ctx.fillText(label,q[0]+5,q[1]+5)}
}
function camera(R,t,color,label){
 const world=p=>[0,1,2].map(i=>R[0][i]*(p[0]-t[0])+R[1][i]*(p[1]-t[1])+R[2][i]*(p[2]-t[2]));
 const c=world([0,0,0]),corners=[[-.2,-.12,.3],[.2,-.12,.3],[.2,.12,.3],[-.2,.12,.3]].map(world);
 for(let i=0;i<4;i++){segment(c,corners[i],color);segment(corners[i],corners[(i+1)%4],color)}
 const screen=project(c);if(screen){ctx.fillStyle=color;ctx.fillText(label,screen[0]+6,screen[1]-8)}
}
function draw(){
 ctx.clearRect(0,0,1000,650);ctx.font='15px system-ui';ctx.lineWidth=1.5;
 segment([0,0,0],[1,0,0],'#f18983','x');segment([0,0,0],[0,1,0],'#78dca4','y');segment([0,0,0],[0,0,1],'#78aff0','z');
 const projected=points.map((p,index)=>({screen:project(p),index})).filter(p=>p.screen).sort((a,b)=>b.screen[2]-a.screen[2]);
 for(const p of projected){ctx.fillStyle=`rgb(${data.rgb[p.index].join(',')})`;ctx.beginPath();ctx.arc(p.screen[0],p.screen[1],3.5,0,Math.PI*2);ctx.fill()}
 camera(data.first_world_to_camera_R||[[1,0,0],[0,1,0],[0,0,1]],data.first_world_to_camera_t||[0,0,0],'#ffcb75','Caméra 1');
 camera(data.second_world_to_camera_R,data.second_world_to_camera_t,'#83c7ff','Caméra 2');
 if(data.keyframe_poses)data.keyframe_poses.forEach((pose,index)=>camera(pose.world_to_camera_R,pose.world_to_camera_t,'#b2c999',`Vue ${index+3}`));
 const firstCenter=data.first_world_to_camera_R?[0,1,2].map(i=>-data.first_world_to_camera_R.reduce((sum,row,j)=>sum+row[i]*data.first_world_to_camera_t[j],0)):[0,0,0];
 segment(firstCenter,data.second_camera_center,'#a5a7b4');
}
canvas.addEventListener('pointerdown',event=>{drag=[event.clientX,event.clientY];canvas.setPointerCapture(event.pointerId)});
canvas.addEventListener('pointermove',event=>{if(!drag)return;yaw+=(event.clientX-drag[0])*.008;pitch=Math.max(-1.4,Math.min(1.4,pitch+(event.clientY-drag[1])*.008));drag=[event.clientX,event.clientY];draw()});
for(const name of ['pointerup','pointercancel','lostpointercapture'])canvas.addEventListener(name,()=>drag=null);
canvas.addEventListener('wheel',event=>{event.preventDefault();el('zoom').value=Math.max(.5,Math.min(4,Number(el('zoom').value)*Math.exp(-event.deltaY*.001)));draw()},{passive:false});
el('zoom').addEventListener('input',draw);el('reset').onclick=()=>{yaw=.35;pitch=-.2;el('zoom').value=1;draw()};
draw();
</script></html>'''
