"""Prepare a full-flight, delivery-paced image input from an audited decoder copy.

Includes ascent and descent. Neither camera poses nor telemetry are supplied to
the reconstruction. The original HEVC and source byte identities are retained.
"""
import argparse,hashlib,json,math,os
from pathlib import Path
from neo2_control.room_mvs import digest,write_new
from neo2_control.video_lineage import build_video_lineage


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('prepared','mission','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--period',type=float,default=1.)
    args=parser.parse_args()
    if not math.isfinite(args.period) or not 1<=args.period<=5:raise ValueError('progressive_preparation_period_invalid')
    audit=json.loads((args.prepared/'decoder-copy-audit.json').read_text())
    raw=Path(audit['source']['path']);decoder=Path(audit['prepared_source']['path'])
    closed=json.loads((args.mission/'closed-connection.json').read_text())
    mission=json.loads((args.mission/'mission-result.json').read_text())
    if (closed.get('possible_airborne') is not False or closed.get('connection',{}).get('cleanup_verified') is not True
            or closed.get('connection',{}).get('generation')!=raw.parent.name or mission.get('motion_complete') is not True):
        raise ValueError('progressive_preparation_requires_matching_closed_flight')
    if digest(raw)!=audit['source']['sha256'] or digest(decoder)!=audit['prepared_source']['sha256']:
        raise ValueError('progressive_preparation_source_changed')
    events=[json.loads(line) for line in (args.mission/'events.jsonl').read_text().splitlines()]
    start=next(e['monotonic'] for e in events if e['event']=='takeoff_requested')
    stop=json.loads((args.mission/'stopped-telemetry.json').read_text())['telemetry']['motion_received_at']
    if not start<stop<=start+120:raise ValueError('progressive_preparation_flight_times_invalid')
    pictures=audit['pictures'];selected={}
    targets=[start+i*args.period for i in range(math.floor((stop-start)/args.period)+1)]+[stop]
    for target in targets:
        picture=min(pictures,key=lambda p:abs(p['last_byte_delivered_monotonic']-target))
        ordinal=picture['prepared_picture_ordinal']
        selected[ordinal]={'picture':picture,'selection_target':{'kind':'periodic_full_flight',
            'seconds_after_takeoff':target-start},'receipt_target_delta_seconds':picture['last_byte_delivered_monotonic']-target,
            'scope':'PC byte delivery, not camera exposure; no pose supplied to mapper.'}
    if not 3<=len(selected)<=96:raise ValueError('progressive_preparation_keyframe_count_invalid')
    root=args.output;root.mkdir(mode=0o700);(root/'images').mkdir(mode=0o700)
    write_new(root/'selection.json',selected)
    lineage=build_video_lineage(decoder,root/'lineage',pictures=sorted(selected),timeout=600,maximum_selected=96)
    frames=[]
    for frame in lineage['frames']:
        if 'image' not in frame:continue
        ordinal=frame['raw_picture_ordinal'];picture=selected[ordinal]['picture']
        if (frame['source_first_slice_byte_offset']!=picture['prepared_start']
                or frame['packet']['source_vcl_body_sha256']!=picture['body_sha256']):
            raise ValueError('progressive_preparation_vcl_identity_mismatch')
        blob=(root/'lineage'/frame['image']['file']).read_bytes();sha=hashlib.sha256(blob).hexdigest()
        if sha!=frame['image']['sha256']:raise ValueError('progressive_preparation_pixels_changed')
        name=f'picture-{ordinal:05d}.png'
        fd=os.open(root/'images'/name,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
        with os.fdopen(fd,'wb') as stream:stream.write(blob)
        frames.append({'file':name,'png_sha256':sha,'rgb_sha256':frame['pixels']['sha256'],
            'prepared_picture_ordinal':ordinal,'original_first_slice_byte_offset':picture['source_start'],
            'source_vcl_body_sha256':picture['body_sha256'],'selection':selected[ordinal]})
    if len(frames)!=len(selected) or digest(raw)!=audit['source']['sha256'] or digest(decoder)!=audit['prepared_source']['sha256']:
        raise ValueError('progressive_preparation_incomplete_or_changed')
    write_new(root/'images.json',{'source':audit['source'],'decoder_copy':audit['prepared_source'],
        'preparation_audit_sha256':digest(args.prepared/'decoder-copy-audit.json'),
        'mission_events_sha256':digest(args.mission/'events.jsonl'),'frames':frames,
        'physical_accuracy_verified':False,'metric_scale_verified':False,
        'scope':'Periodic full flight including ascent, turns and descent; retained pixels and VCL identities; no mapper pose input.'})
    print(json.dumps({'frames':len(frames),'duration_seconds':stop-start,'output':str(root.absolute())}),flush=True)


if __name__=='__main__':main()
