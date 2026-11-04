"""Write a local, non-metric review of a COLMAP candidate and its source images.

Requires PyCOLMAP 4.2.1 and the project's vision extra. No controller, radio,
camera command or navigation-map import is available. Geometric display
filters do not establish physical accuracy or global reconstruction stability.
"""
import argparse
import base64
from collections import Counter
import hashlib
import html
import json
import os
from pathlib import Path

import numpy as np

from neo2_control.scan_report import _PAGE
from neo2_control.reconstruction import _read


def digest(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def write_new(path,value):
    fd=os.open(path,os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
    with os.fdopen(fd,'w') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def write_report(model_directory,manifest_path,images_directory,output):
    import pycolmap as colmap
    if colmap.__version__!='4.2.1':raise ValueError('multiview_report_requires_pycolmap_4_2_1')
    model_directory,manifest_path,images_directory,output=map(Path,
        (model_directory,manifest_path,images_directory,output))
    manifest=json.loads(_read(manifest_path,64*1024*1024))
    records=manifest.get('frames')
    if not isinstance(records,list) or not 3<=len(records)<=96:
        raise ValueError('multiview_report_source_manifest_invalid')
    if any(not isinstance(row,dict) or not isinstance(row.get('file'),str)
            or not isinstance(row.get('png_sha256'),str) for row in records):
        raise ValueError('multiview_report_source_manifest_invalid')
    by_name={row['file']:row for row in records}
    if len(by_name)!=len(records):raise ValueError('multiview_report_duplicate_image')
    for name,row in by_name.items():
        if Path(name).name!=name or name in ('.','..') or Path(name).suffix.lower()!='.png':
            raise ValueError('multiview_report_image_name_invalid')
        path=images_directory/name
        if not 0<path.stat().st_size<=32*1024*1024 or digest(path)!=row['png_sha256']:
            raise ValueError('multiview_report_image_identity_changed')
    model_files=sorted(model_directory.glob('*.bin'))
    if not model_files or sum(p.stat().st_size for p in model_files)>512*1024*1024:
        raise ValueError('multiview_report_model_size_invalid')
    identities={p.name:digest(p) for p in model_files}
    rec=colmap.Reconstruction(model_directory)
    if not rec.is_valid() or not 3<=rec.num_reg_images()<=96 or not 20<=rec.num_points3D()<=60000:
        raise ValueError('multiview_report_model_invalid')
    views={}
    for image_id in rec.reg_image_ids():
        image=rec.images[image_id]
        if image.name not in by_name:raise ValueError('multiview_report_untracked_source_image')
        pose=image.cam_from_world();R=pose.rotation.matrix();t=pose.translation;C=-R.T@t
        if not np.isfinite(R).all() or not np.isfinite(t).all():raise ValueError('multiview_report_nonfinite_pose')
        views[image_id]={'R':R,'t':t,'C':C,'name':image.name}
    ids=sorted(views)
    a,b=max(((a,b) for j,a in enumerate(ids) for b in ids[j+1:]),
        key=lambda pair:np.linalg.norm(views[pair[0]]['C']-views[pair[1]]['C']))
    R0,C0=views[a]['R'],views[a]['C'];baseline=float(np.linalg.norm(views[b]['C']-C0))
    if not np.isfinite(baseline) or baseline<1e-9:raise ValueError('multiview_report_no_baseline')
    transformed={i:{'world_to_camera_R':v['R']@R0.T,
        'world_to_camera_t':(v['R']@C0+v['t'])/baseline,
        'camera_center':R0@(v['C']-C0)/baseline,'source_file':v['name']} for i,v in views.items()}
    selected=[];excluded=Counter();maximum_projection_difference=0.
    for point_id,point in rec.points3D.items():
        track=list(point.track.elements);track_ids=[int(e.image_id) for e in track]
        if len(track_ids)!=len(set(track_ids)):
            excluded['multiple_observations_in_one_image']+=1;continue
        if len(track)<3:excluded['fewer_than_three_views']+=1;continue
        xyz=np.asarray(point.xyz);X=R0@(xyz-C0)/baseline;errors=[];rays=[];positive=True
        if not np.isfinite(X).all():excluded['nonfinite']+=1;continue
        centers=np.array([views[i]['C'] for i in track_ids])
        span=np.max(np.linalg.norm(centers[:,None]-centers[None,:],axis=2))
        # A point and all observing cameras can collapse onto one numerical
        # location during a rotation-only fit. Low pixel residuals and angles
        # computed from near-zero rays do not establish its depth.
        if span<=baseline*1e-6:
            excluded['vanishing_track_baseline']+=1;continue
        if np.min(np.linalg.norm(centers-xyz,axis=1))<=baseline*1e-6:
            excluded['vanishing_point_range']+=1;continue
        for e in track:
            v=views[e.image_id];camera=rec.cameras[rec.images[e.image_id].camera_id]
            original=v['R']@xyz+v['t'];q=transformed[e.image_id]
            changed=q['world_to_camera_R']@X+q['world_to_camera_t']
            positive=positive and original[2]>0
            pixel=camera.img_from_cam(original);check=camera.img_from_cam(changed)
            if pixel is None or check is None:positive=False;continue
            maximum_projection_difference=max(maximum_projection_difference,float(np.linalg.norm(pixel-check)))
            errors.append(float(np.linalg.norm(pixel-rec.images[e.image_id].points2D[e.point2D_idx].xy)))
            ray=xyz-v['C'];ray/=np.linalg.norm(ray);rays.append(ray)
        angles=[float(np.degrees(np.arccos(np.clip(x@y,-1,1))))
            for j,x in enumerate(rays) for y in rays[j+1:]]
        if not positive:excluded['behind_camera']+=1;continue
        if not errors or not angles or max(errors)>1 or max(angles)<1 or np.linalg.norm(X)>50:
            excluded['display_geometry_filter']+=1;continue
        selected.append({'point3D_id':int(point_id),'xyz':X.tolist(),'rgb':point.color.tolist(),
            'maximum_reprojection_error_px':max(errors),'maximum_parallax_degrees':max(angles),
            'source_observations':[{'image_id':int(e.image_id),'point2D_idx':int(e.point2D_idx)} for e in track]})
    if maximum_projection_difference>1e-8:raise ValueError('multiview_report_coordinate_conversion_failed')
    if len(selected)<20:raise ValueError('multiview_report_insufficient_display_points')
    for name,expected in identities.items():
        if digest(model_directory/name)!=expected:raise ValueError('multiview_report_model_changed')
    output.mkdir(mode=0o700)
    pose=lambda i:{key:value.tolist() if isinstance(value,np.ndarray) else value for key,value in transformed[i].items()}
    identity=hashlib.sha256(json.dumps([identities,a,b,baseline],sort_keys=True).encode()).hexdigest()
    cameras={int(i):{'model':camera.model_name,'width':camera.width,'height':camera.height,
        'params':camera.params.tolist()} for i,camera in rec.cameras.items()}
    dataset={'format':'neo2-control/multiview-candidate/v1','kind':'multiview_candidate',
        'points':[p['xyz'] for p in selected],'rgb':[p['rgb'] for p in selected],
        'coordinate_frame':identity,'distance_unit':'baseline','metric_scale_verified':False,
        'physical_accuracy_verified':False,'global_stability_verified':False,'free_space_observed':False,
        'navigation_eligible':False,'hardware_used':False,
        'image_sha256':[by_name[views[i]['name']]['png_sha256'] for i in ids],
        'camera_model':{'status':'candidate','provenance':'Paramètres du modèle importé, calibration physique non vérifiée.',
            'native_cameras':cameras},
        'first_world_to_camera_R':pose(a)['world_to_camera_R'],'first_world_to_camera_t':pose(a)['world_to_camera_t'],
        'second_world_to_camera_R':pose(b)['world_to_camera_R'],'second_world_to_camera_t':pose(b)['world_to_camera_t'],
        'second_camera_center':pose(b)['camera_center'],
        'keyframe_poses':[pose(i) for i in ids if i not in (a,b)],
        'reprojection_error_px':[p['maximum_reprojection_error_px'] for p in selected],
        'parallax_degrees':[p['maximum_parallax_degrees'] for p in selected]}
    encoded=json.dumps(dataset,ensure_ascii=False,allow_nan=False).replace('&','\\u0026').replace('<','\\u003c').replace('>','\\u003e')
    page=_PAGE.replace('__DATA__',encoded)
    # A few distant points must not shrink the useful scene to a tiny patch.
    # Keep every point in the dataset; this changes only the initial framing.
    page=page.replace(
        'const center=points.length?[0,1,2].map(axis=>points.reduce((sum,p)=>sum+p[axis],0)/points.length):[0,0,0];\n'
        'const extent=Math.max(1,...points.map(p=>Math.hypot(...p.map((v,i)=>v-center[i]))),Math.hypot(...center));',
        'const quantile=(values,q)=>[...values].sort((a,b)=>a-b)[Math.floor((values.length-1)*q)];\n'
        'const center=[0,1,2].map(axis=>quantile(points.map(p=>p[axis]),.5));\n'
        'const extent=Math.max(1,quantile(points.map(p=>Math.hypot(...p.map((v,i)=>v-center[i]))),.98),Math.hypot(...center));')
    page=page.replace('`Vue ${index+3}`', "''")
    page=page.replace('Les cadres représentent les prises de vue.',
        'Les cadres représentent les prises de vue. Dézoomer permet de voir les points éloignés du cadrage initial.')
    page=page.replace('Reconstruction 3D enregistrée','Nuage 3D expérimental — non métrique')
    page=page.replace('${points.length} points triangulés depuis deux images et conservés sur le PC.',
        f'${{points.length}} points affichés depuis {len(views)} vues, conservés sur le PC.')
    page=page.replace('Repère de la première caméra','Repère de la première vue de référence')
    page=page.replace('Parallaxe médiane :','Parallaxe maximale par point (médiane) :')
    page=page.replace('Échelle non étalonnée : une unité correspond à la distance entre les deux prises de vue, pas à un mètre.',
        'Candidat partiel : calibration physique et stabilité globale non vérifiées. Une unité est la distance entre les deux vues de référence, pas un mètre.')
    for i in (a,b):
        raw=(images_directory/views[i]['name']).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=by_name[views[i]['name']]['png_sha256']:
            raise ValueError('multiview_report_reference_image_changed')
        photograph=f'<details><summary>Image source {html.escape(views[i]["name"])}</summary><img style="max-width:100%" src="data:image/png;base64,{base64.b64encode(raw).decode()}"></details>'
        page=page.replace('</html>',photograph+'</html>')
    fd=os.open(output/'review.html',os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
    with os.fdopen(fd,'w') as stream:stream.write(page)
    report={'format':'neo2-control/multiview-candidate-review/v1','model_directory':str(model_directory),
        'model_files_sha256':identities,'source_manifest_sha256':digest(manifest_path),'source':manifest.get('source'),
        'input_points':rec.num_points3D(),'registered_views':len(views),'displayed_points':len(selected),
        'excluded_points':dict(excluded),'coordinate_frame':identity,'axes':'reference_camera_right_down_forward',
        'native_cameras':cameras,
        'scale_basis':{'images':[views[a]['name'],views[b]['name']],'native_distance':baseline,'unit':'baseline'},
        'maximum_projection_difference_after_frame_conversion_px':maximum_projection_difference,
        'points':selected,'poses':{int(i):pose(i) for i in ids},'review_sha256':digest(output/'review.html'),
        'physical_accuracy_verified':False,'metric_scale_verified':False,'global_stability_verified':False,
        'navigation_eligible':False,'free_space_observed':False,'hardware_used':False}
    write_new(output/'candidate.json',dataset);write_new(output/'provenance.json',report)
    return {key:report[key] for key in ('input_points','registered_views','displayed_points','excluded_points',
        'maximum_projection_difference_after_frame_conversion_px','navigation_eligible')}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('model_directory','source_manifest','images_directory','output'):parser.add_argument(name,type=Path)
    args=parser.parse_args()
    print(json.dumps(write_report(args.model_directory,args.source_manifest,args.images_directory,args.output),indent=2))
