"""CPU-only offline multiview candidate; no controller/radio or flight API."""
from pathlib import Path
import argparse,hashlib,json,os,shutil,sqlite3,time
import numpy as np
import cv2 as cv
import pycolmap as p
cv.setNumThreads(2)
ROOT=Path('private/multiview-room-20261019')
assert p.__version__=='4.2.1'
def sha(path):
 with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(path,value):
 fd=os.open(path,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
 with os.fdopen(fd,'w') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
def prepare():
 reference=json.loads(Path('private/video-lineage-20261018/flight/manifest.json').read_text())
 canonical={row['raw_picture_ordinal']:row for row in reference['frames']}
 images=ROOT/'images';images.mkdir(mode=0o700)
 found={}
 for directory in (Path('private/multiview-room-20261019/frames'),Path('private/gimbal-geometry-20261018/rotation-frames'),Path('private/geometry-consistency-20261018/takeoff-frames')):
  manifest=json.loads((directory/'manifest.json').read_text())
  for row in manifest['frames']:
   if 'image' not in row:continue
   n=row['raw_picture_ordinal'];old=canonical[n]
   assert row['pixels']==old['pixels'] and row['packet']['source_vcl_body_sha256']==old['packet']['source_vcl_body_sha256']
   source=directory/row['image']['file'];assert sha(source)==row['image']['sha256']
   image=cv.imread(str(source));assert image is not None and image.shape==(720,1280,3)
   assert hashlib.sha256(cv.cvtColor(image,cv.COLOR_BGR2RGB).tobytes()).hexdigest()==old['pixels']['sha256']
   if n in found:
    assert found[n]['rgb_sha256']==old['pixels']['sha256'];continue
   dest=images/f'raw-{n:05d}.png';shutil.copyfile(source,dest);dest.chmod(0o600)
   found[n]={'raw_picture_ordinal':n,'file':dest.name,'source_file':str(source),'png_sha256':sha(dest),
    'rgb_sha256':old['pixels']['sha256'],'source_vcl_body_sha256':old['packet']['source_vcl_body_sha256'],
    'source_first_slice_byte_offset':old['source_first_slice_byte_offset']}
 result={'source':reference['source'],'frames':sorted(found.values(),key=lambda x:x['raw_picture_ordinal']),
  'scope':'Lossless source-bound images, raw ordinals are not timestamps; no exposure alignment or metric scale.'}
 save(ROOT/'images.json',result);return result

def database_stats(path):
 with sqlite3.connect(f'file:{path}?mode=ro',uri=True) as db:
  images=[dict(zip(('image_id','name','keypoints'),row)) for row in db.execute('SELECT images.image_id,images.name,keypoints.rows FROM images LEFT JOIN keypoints ON images.image_id=keypoints.image_id ORDER BY images.image_id')]
  configs=[{'config':a,'pairs':b,'inliers':c} for a,b,c in db.execute('SELECT config,count(*),sum(rows) FROM two_view_geometries GROUP BY config')]
  matches=db.execute('SELECT count(*),sum(rows) FROM matches').fetchone()
 return {'images':images,'raw_matched_pairs':matches[0],'raw_matches':matches[1],'two_view_configs':configs}

def run():
 started=time.monotonic();source=prepare();images=ROOT/'images';db=ROOT/'database.db'
 reader=p.ImageReaderOptions(camera_model='PINHOLE',camera_params='680,680,640,360')
 extraction=p.FeatureExtractionOptions(num_threads=2,max_image_size=1280,use_gpu=False)
 extraction.sift.max_num_features=6000;extraction.sift.max_num_orientations=1
 matching=p.FeatureMatchingOptions(num_threads=2,use_gpu=False)
 save(ROOT/'options.json',{'pycolmap':p.__version__,'native_binary':[{'path':str(x),'sha256':sha(x)} for x in Path(p.__file__).parent.glob('*.so')],
  'numpy':np.__version__,'camera':'PINHOLE','candidate_intrinsics':[680,680,640,360],
  'intrinsics_fixed':True,'sift_orientations_per_keypoint':1,'CPU_threads':2,'CUDA_used':False,
  'scope':'Experimental external SfM candidate; existing map/flight qualification gates are not bypassed.'})
 print('extract_features',len(source['frames']),flush=True)
 p.extract_features(db,images,camera_mode=p.CameraMode.SINGLE,reader_options=reader,extraction_options=extraction,device=p.Device.cpu)
 print('match_exhaustive',flush=True)
 p.match_exhaustive(db,matching_options=matching,device=p.Device.cpu)
 stats=database_stats(db);save(ROOT/'matching.json',stats)
 options=p.IncrementalPipelineOptions(num_threads=2,random_seed=0,max_runtime_seconds=180,
  max_num_models=3,ba_refine_focal_length=False,ba_refine_principal_point=False,ba_refine_extra_params=False,
  ba_use_gpu=False,ba_global_max_num_iterations=50,ba_local_max_num_iterations=30)
 options.mapper.num_threads=2;options.mapper.random_seed=0
 options.mapper.abs_pose_refine_focal_length=False;options.mapper.abs_pose_refine_extra_params=False
 save(ROOT/'mapping-options.json',json.loads(json.dumps(options.todict(),default=str)))
 output=ROOT/'sparse';output.mkdir(mode=0o700)
 print('incremental_mapping',flush=True)
 maps=p.incremental_mapping(db,images,output,options=options)
 report={'format':'neo2-control/multiview-candidate-audit/v1','source':source['source'],
  'input_images':len(source['frames']),'camera_model_status':'candidate_unverified',
  'physical_accuracy_verified':False,'metric_scale_verified':False,'free_space_observed':False,
  'hardware_used':False,'models':[]}
 for index,recon in maps.items():
  print('model',index,recon.summary(),flush=True)
  dest=ROOT/f'candidate-{index}.ply';recon.export_PLY(dest);dest.chmod(0o600)
  views={}
  for image_id in recon.reg_image_ids():
   image=recon.images[image_id];pose=image.cam_from_world()
   R=pose.rotation.matrix();t=pose.translation
   views[int(image_id)]={'name':image.name,'R':R.tolist(),'t':t.tolist(),'center':(-R.T@t).tolist()}
  points=[]
  for point_id,point in recon.points3D.items():
   xyz=np.asarray(point.xyz);track=[];rays=[]
   for element in point.track.elements:
    view=views[int(element.image_id)];image=recon.images[element.image_id]
    pixel=image.points2D[element.point2D_idx].xy
    track.append({'image_id':int(element.image_id),'point2D_idx':int(element.point2D_idx),'xy':pixel.tolist()})
    ray=xyz-np.array(view['center']);ray/=np.linalg.norm(ray);rays.append(ray)
   angles=[np.degrees(np.arccos(np.clip(a@b,-1,1))) for i,a in enumerate(rays) for b in rays[i+1:]]
   points.append({'point3D_id':int(point_id),'xyz':xyz.tolist(),'color':point.color.tolist(),
    'error_px':float(point.error),'track':track,'max_triangulation_angle_degrees':float(max(angles)) if angles else 0.})
  item={'index':int(index),'summary':recon.summary(),'registered_images':len(views),'points3D':len(points),
   'views':views,'points':points,'ply':str(dest),'ply_sha256':sha(dest)}
  report['models'].append(item)
 report['elapsed_seconds']=time.monotonic()-started
 save(ROOT/'candidate-audit.json',report)
 print(json.dumps({'input_images':report['input_images'],'models':[{k:v for k,v in m.items() if k not in ('points','views','summary')} for m in report['models']],
  'seconds':report['elapsed_seconds'],'metric_scale_verified':False}),flush=True)
 for path in ROOT.rglob('*'):
  if path.is_file() and 'packages' not in path.parts:path.chmod(0o600)

if __name__=='__main__':
 parser=argparse.ArgumentParser(description='Replay the source-bound recorded-room SfM experiment, CPU only. Run from the repository root. Requires retained private image manifests and the sfm extra.')
 parser.add_argument('--output',required=True,type=Path)
 args=parser.parse_args();ROOT=args.output
 ROOT.mkdir(mode=0o700)
 run()
