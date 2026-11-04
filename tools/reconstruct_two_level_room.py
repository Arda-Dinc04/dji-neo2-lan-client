"""Offline SfM from verified two-level PNGs; never a navigation map."""
from pathlib import Path
import argparse,hashlib,json,os,time
import cv2 as cv
import numpy as np
import pycolmap as p
from reconstruct_recorded_room import database_stats
from write_multiview_report import write_report
cv.setNumThreads(2)
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input',required=True,type=Path,help='prepared directory containing images.json and images/')
parser.add_argument('--output',required=True,type=Path,help='new private SfM output directory')
args=parser.parse_args()
ROOT,OUT=args.input,args.output
OUT.mkdir(mode=0o700)
def sha(path):
 with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(name,value):
 fd=os.open(OUT/name,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
 with os.fdopen(fd,'w') as f:json.dump(value,f,indent=2,allow_nan=False)
assert p.__version__=='4.2.1'
manifest=json.loads((ROOT/'images.json').read_text());images=ROOT/'images'
assert len(manifest['frames'])==32
for row in manifest['frames']:
 image=images/row['file'];assert sha(image)==row['png_sha256']
 pixels=cv.imread(str(image));assert pixels.shape==(720,1280,3)
 assert hashlib.sha256(cv.cvtColor(pixels,cv.COLOR_BGR2RGB).tobytes()).hexdigest()==row['rgb_sha256']
started=time.monotonic();database=OUT/'database.db'
reader=p.ImageReaderOptions(camera_model='PINHOLE',camera_params='680,680,640,360')
extraction=p.FeatureExtractionOptions(num_threads=2,max_image_size=1280,use_gpu=False)
extraction.sift.max_num_features=6000;extraction.sift.max_num_orientations=1
matching=p.FeatureMatchingOptions(num_threads=2,use_gpu=False)
options=p.IncrementalPipelineOptions(num_threads=2,random_seed=0,max_runtime_seconds=120,
 max_num_models=3,structure_less_registration_fallback=False,init_num_trials=100,
 ba_refine_focal_length=False,ba_refine_principal_point=False,ba_refine_extra_params=False,
 ba_use_gpu=False,ba_global_max_num_iterations=50,ba_local_max_num_iterations=30)
options.mapper.num_threads=2;options.mapper.random_seed=0
options.mapper.abs_pose_refine_focal_length=False;options.mapper.abs_pose_refine_extra_params=False
save('options.json',{'pycolmap':p.__version__,'source_manifest_sha256':sha(ROOT/'images.json'),
 'candidate_intrinsics':[680,680,640,360],'intrinsics_fixed':True,
 'physical_optics_verified':False,'metric_scale_verified':False,'CPU_threads':2,
 'options':json.loads(json.dumps(options.todict(),default=str))})
print('extract_features',flush=True)
p.extract_features(database,images,camera_mode=p.CameraMode.SINGLE,reader_options=reader,
 extraction_options=extraction,device=p.Device.cpu)
print('match_exhaustive',flush=True)
p.match_exhaustive(database,matching_options=matching,device=p.Device.cpu)
save('matching.json',database_stats(database))
sparse=OUT/'sparse';sparse.mkdir(mode=0o700)
print('incremental_mapping',flush=True)
models=p.incremental_mapping(database,images,sparse,options=options)
report={'source':manifest['source'],'image_count':32,'models':[],
 'physical_accuracy_verified':False,'metric_scale_verified':False,
 'global_stability_verified':False,'free_space_observed':False,'navigation_eligible':False}
for index,rec in models.items():
 row={'index':int(index),'registered_images':rec.num_reg_images(),'raw_points':rec.num_points3D()}
 bad=[]
 for point_id,point in rec.points3D.items():
  ids=[int(e.image_id) for e in point.track.elements]
  if len(ids)!=len(set(ids)):bad.append(point_id)
 for point_id in bad:rec.delete_point3D(point_id)
 row['ambiguous_tracks_removed']=len(bad)
 if rec.num_reg_images()>=3 and rec.num_points3D()>=20:
  ba=p.BundleAdjustmentOptions(refine_focal_length=False,refine_principal_point=False,
   refine_extra_params=False,refine_sensor_from_rig=False,print_summary=False)
  ba.ceres.use_gpu=False;ba.ceres.auto_select_solver_type=False
  solver=ba.ceres.solver_options;solver.num_threads=2;solver.max_num_iterations=60
  solver.max_solver_time_in_seconds=15.;solver.function_tolerance=1e-6
  solver.linear_solver_type=type(solver.linear_solver_type).__members__['SPARSE_SCHUR']
  config=p.BundleAdjustmentConfig()
  for image_id in rec.reg_image_ids():config.add_image(image_id)
  for camera_id in rec.cameras:config.set_constant_cam_intrinsics(camera_id)
  config.fix_gauge(p.BundleAdjustmentGauge.TWO_CAMS_FROM_WORLD)
  summary=p.create_default_bundle_adjuster(ba,config,rec).solve()
  row['bundle_adjustment']={'usable':summary.is_solution_usable(),'summary':summary.brief_report()}
  refined=OUT/f'refined-{index}';refined.mkdir(mode=0o700);rec.write(refined)
  try:row['review']=write_report(refined,ROOT/'images.json',images,OUT/f'review-{index}')
  except ValueError as error:row['review_error']=str(error)
 row['retained_points']=rec.num_points3D();report['models'].append(row)
 print(json.dumps(row),flush=True)
report['seconds']=time.monotonic()-started;save('result.json',report)
for path in OUT.rglob('*'):
 if path.is_file():path.chmod(0o600)
print(json.dumps(report),flush=True)
