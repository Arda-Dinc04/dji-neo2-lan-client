"""Offline decoder copy with explicit DJI suffix-metadata quarantine.

Original bytes remain unchanged. Selected image receipt labels are not exposure
times or calibrated camera heights. This script has no controller/radio API.
"""
from pathlib import Path
from collections import Counter
import argparse, bisect, hashlib, json, os, statistics
from neo2_control.video_clock import _units, inventory_hevc_clocks
from neo2_control.video_lineage import build_video_lineage
from neo2_control.video_export import _stamp

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--session',required=True,type=Path,help='retained, closed flight-session directory')
parser.add_argument('--mission',required=True,type=Path,help='retained mission directory containing events.jsonl')
parser.add_argument('--output',required=True,type=Path,help='new private output directory')
args=parser.parse_args()
SOURCE,FLIGHT,ROOT=args.session,args.mission,args.output
closed=json.loads((FLIGHT/'closed-connection.json').read_text())
connection=json.loads((SOURCE/'connection-result.json').read_text())
mission=json.loads((FLIGHT/'mission-result.json').read_text())
if (closed.get('connection',{}).get('generation')!=SOURCE.name
        or closed.get('possible_airborne') is not False
        or closed.get('connection',{}).get('cleanup_verified') is not True
        or connection.get('phase')!='closed'
        or not connection.get('cleanup') or not all(v is True for v in connection['cleanup'].values())
        or mission.get('motion_complete') is not True):
 raise ValueError('two_level_requires_matching_closed_completed_mission')
ROOT.mkdir(mode=0o700)
def sha(path):
 with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(path,value):
 fd=os.open(path,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
 with os.fdopen(fd,'w') as f:json.dump(value,f,indent=2,allow_nan=False)

raw=SOURCE/'camera.hevc';initial=raw.stat();source_hash=sha(raw)
deliveries=[];offset=0;origins=[]
with (SOURCE/'swudp.jsonl.session.jsonl').open() as f:
 for line in f:
  r=json.loads(line)
  if r.get('event')=='stream_payload_delivered' and r.get('channel')==2:
   offset+=r['bytes'];deliveries.append((offset,r['elapsed_seconds']))
  if r.get('event')=='telemetry_updated':
   for o in r['telemetry'].get('observations',[]):
    if type(o.get('received_at')) in (int,float) and type(o.get('age_seconds')) in (int,float):
     origins.append(o['received_at']+o['age_seconds']-r['elapsed_seconds'])
assert offset==initial.st_size and max(origins)-min(origins)<1e-8
origin=statistics.median(origins);ends=[row[0] for row in deliveries]
prepared=ROOT/'decoder-input.hevc';retained=[];dropped=[];pictures=[];written=0
quarantine=False;checksum=hashlib.sha256()
with raw.open('rb') as stream,raw.open('rb') as original,prepared.open('xb') as dest:
 os.fchmod(dest.fileno(),0o600)
 for start,end,body in _units(stream,checksum):
  kind=(body[0]>>1)&63
  # This recording uses exact pic_type=1 AUD delimiters. Quarantine begins
  # with an observed SEI and includes all false Annex-B prefixes in its
  # noncanonical vendor payload, until the next exact AUD delimiter.
  if kind==35 and body==b'\x46\x01\x30':quarantine=False
  if not quarantine and kind in (39,40):quarantine=True
  if quarantine:
   if dropped and dropped[-1]['end']==start:dropped[-1]['end']=end
   else:dropped.append({'start':start,'end':end})
   continue
  assert len(body)>=2 and not body[0]&128 and body[1]&7 and not (((body[0]&1)<<5)|(body[1]>>3))
  assert kind<=38
  original.seek(start);chunk=original.read(end-start);assert len(chunk)==end-start
  record={'source_start':start,'source_end':end,'prepared_start':written,'prepared_end':written+len(chunk),
   'nal_type':kind,'body_sha256':hashlib.sha256(body).hexdigest(),'raw_sha256':hashlib.sha256(chunk).hexdigest()}
  retained.append(record);dest.write(chunk);written+=len(chunk)
  if kind<=31 and len(body)>=3 and body[2]&128:
   delivery=deliveries[bisect.bisect_left(ends,end)]
   pictures.append({**record,'prepared_picture_ordinal':len(pictures),
    'last_byte_delivered_monotonic':origin+delivery[1],
    'exposure_alignment_verified':False})
assert checksum.hexdigest()==source_hash and _stamp(raw.stat())==_stamp(initial)
inventory=inventory_hevc_clocks(prepared)
assert inventory['summary']['raw_picture_candidates']==len(pictures)
source={'path':str(raw.absolute()),'bytes':initial.st_size,'sha256':source_hash}
save(ROOT/'decoder-copy-audit.json',{'source':source,'prepared_source':inventory['source'],
 'method':'Quarantine each suffix/prefix SEI region through the next exact AUD 460130. Retain all other byte ranges exactly.',
 'quarantined_ranges':dropped,'retained_units':retained,'pictures':pictures,
 'scope':'Decoder preparation only; quarantined metadata is not parsed or assigned a clock. Raw stream is preserved.',
 'source_events_sha256':sha(SOURCE/'swudp.jsonl.session.jsonl'),'pc_session_origin_monotonic':origin,
 'origin_consistent_observations':len(origins),'exposure_clock_verified':False,'metric_scale_verified':False})

events=[json.loads(line) for line in (FLIGHT/'events.jsonl').read_text().splitlines()]
targets=[]
for level in (0,1):
 rows=[r for r in events if r.get('event')=='scan_progress' and r.get('level')==level and r.get('phase')=='rotate']
 for degrees in range(0,360,24):
  row=min(rows,key=lambda r:abs(r['rotation']['progress_degrees']-degrees))
  targets.append({'level':level,'desired_yaw_progress_degrees':degrees,'event':row})
# Additional translated views from the second ascent connect the two circles.
for altitude in (1.1,1.3):
 rows=[r for r in events if r.get('event')=='scan_progress' and r.get('level')==1 and r.get('phase')=='climb']
 row=min(rows,key=lambda r:abs(r['altitude_m']-altitude))
 targets.append({'level':1,'intermediate_climb':True,'event':row})
selected={}
for target in targets:
 row=target['event'];picture=min(pictures,key=lambda p:abs(p['last_byte_delivered_monotonic']-row['monotonic']))
 n=picture['prepared_picture_ordinal'];assert n not in selected
 selected[n]={'picture':picture,'selection_target':target,
  'receipt_to_operator_observation_delta_seconds':picture['last_byte_delivered_monotonic']-row['monotonic'],
  'scope':'Closest software byte-delivery time, not camera acquisition time or a calibrated pose.'}
save(ROOT/'selection.json',selected)
print(json.dumps({'prepared_bytes':written,'pictures':len(pictures),'selected':len(selected),'source_sha256':source_hash}),flush=True)
lineage=build_video_lineage(prepared,ROOT/'lineage',pictures=sorted(selected),timeout=300)
by_ordinal={r['prepared_picture_ordinal']:r for r in pictures}
frames=[]
images=ROOT/'images';images.mkdir(mode=0o700)
for frame in lineage['frames']:
 if 'image' not in frame:continue
 n=frame['raw_picture_ordinal'];picture=by_ordinal[n]
 assert frame['source_first_slice_byte_offset']==picture['prepared_start']
 assert frame['packet']['source_vcl_body_sha256']==picture['body_sha256']
 original=ROOT/'lineage'/frame['image']['file'];name=f'picture-{n:05d}.png'
 blob=original.read_bytes();assert hashlib.sha256(blob).hexdigest()==frame['image']['sha256']
 fd=os.open(images/name,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
 with os.fdopen(fd,'wb') as f:f.write(blob)
 frames.append({'file':name,'png_sha256':hashlib.sha256(blob).hexdigest(),
  'rgb_sha256':frame['pixels']['sha256'],'prepared_picture_ordinal':n,
  'original_first_slice_byte_offset':picture['source_start'],
  'source_vcl_body_sha256':picture['body_sha256'],'selection':selected[n]})
assert len(frames)==32 and sha(raw)==source_hash and _stamp(raw.stat())==_stamp(initial)
save(ROOT/'images.json',{'source':source,'decoder_copy':inventory['source'],
 'frames':frames,'physical_accuracy_verified':False,'metric_scale_verified':False,
 'scope':'Original VCL bodies are byte-identical in decoder copy; PNG pixel hashes and normalization lineage retained. Receipt labels are not exposure times.'})
print(json.dumps({'extracted_pngs':len(frames),'decoded_frames':len(lineage['frames'])}),flush=True)
