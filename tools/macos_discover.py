#!/usr/bin/env python3
"""Discover Neo 2 advertisements; save only matching candidates privately."""
import argparse
import json
import os
from pathlib import Path
import re
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from neo2_control.ble import scan_sync, DJI_CANDIDATE_SERVICE


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=8)
    args=parser.parse_args()
    candidates=[d.as_dict() for d in scan_sync(args.duration)
                if re.search(r'NEO[ _-]?2',d.name.upper()) and DJI_CANDIDATE_SERVICE in d.service_uuids]
    directory=ROOT/'private/macos'
    directory.mkdir(mode=0o700,parents=True,exist_ok=True)
    if len(candidates)==1:
        target=directory/'ble-scan.json'
        temporary=directory/'ble-scan.json.tmp'
        fd=os.open(temporary,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
        with os.fdopen(fd,'w') as stream:json.dump(candidates,stream,indent=2)
        temporary.replace(target)
    print(json.dumps({'neo2_candidates':len(candidates),'application_writes':0,
                      'saved_unique_candidate':len(candidates)==1}))
    return 0 if len(candidates)==1 else 2


if __name__=='__main__':raise SystemExit(main())
