"""Estimate a recorded visual reconstruction's scale from measured FC heights.

Does not connect to hardware, read flight setpoints or modify the model/replay.
"""
import argparse
import json
from pathlib import Path
from neo2_control.room_altitude import RecordedAltitude
from neo2_control.room_mvs import digest,write_new
from spatial_capture.altitude_scale import estimate_altitude_scale


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--poses',type=Path,required=True,help='independent room-poses/v1 visual poses')
    parser.add_argument('--session-events',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    data=json.loads(args.poses.read_text())
    if data.get('format')!='neo2-control/room-poses/v1':raise ValueError('room_scale_requires_visual_poses')
    altitude=RecordedAltitude(args.session_events);records={}
    for pose in data['poses']:
        time=pose['selection']['picture']['last_byte_delivered_monotonic']
        observation=altitude.at(time)
        records[pose['file']]={'received_at':time,'source_metadata':
            {'altitude_observation':observation} if observation is not None else {}}
    result=estimate_altitude_scale(data['poses'],records)
    result['provenance']={'poses_sha256':digest(args.poses),
        'session_events_sha256':digest(args.session_events),'source':data.get('source'),
        'model_files_sha256':data.get('model_files_sha256'),
        'pose_source':'Independent visual reconstruction; replay positions are not used'}
    write_new(args.output,{'scale':result,'observations':records})
    print(json.dumps(result,allow_nan=False))


if __name__=='__main__':main()
