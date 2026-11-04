#!/usr/bin/env python3
"""Apply the control integration to its verified Home Server source baseline.

No network access, process launch, secrets or deployment. The HLS page is
preserved at /drone/video. The source PC UI is copied at application build time.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile

REPO=Path(__file__).resolve().parents[1]
PACKAGE=REPO/'integrations/home-server'
STATE='.neo2-control-integration.json'
FORMAT='neo2-control/home-server-integration/v1'


def digest(data):return hashlib.sha256(data).hexdigest()


def safe(root,name):
    relative=Path(name)
    if relative.is_absolute() or '..' in relative.parts:raise ValueError('invalid_integration_path')
    path=root/relative
    for part in (path,*path.parents):
        if part==root:break
        if part.is_symlink():raise ValueError('integration_symlink_refused')
    return path


def apply(target,*,check=False):
    root=Path(target).resolve(strict=True)
    baseline=json.loads((PACKAGE/'baseline.json').read_text())
    state_file=safe(root,STATE)
    state=json.loads(state_file.read_text()) if state_file.exists() else {'format':FORMAT,'files':{}}
    if state.get('format')!=FORMAT or not isinstance(state.get('files'),dict):raise ValueError('invalid_integration_state')
    files={}
    for source in (PACKAGE/'files').rglob('*'):
        if source.is_file():files[str(source.relative_to(PACKAGE/'files'))]=source.read_bytes()
    for name in ('favicon.svg','camera.css','flight.css','flight.js'):
        files['apps/web/static/neo2/'+name]=(REPO/'src/neo2_control/web'/name).read_bytes()
    template=(REPO/'src/neo2_control/web/flight.html').read_bytes()
    if (template.count(b'__NEO2_TOKEN__')!=1 or b'name="neo2-auth" content="local"' not in template
            or b'name="neo2-api-prefix" content="/api/flight"' not in template):
        raise ValueError('incompatible_pc_flight_template')
    files['apps/web/src/lib/server/neo2-flight.html']=template
    # Preserve the original page once; never copy our replacement into it.
    for original,preserved in baseline['preserve'].items():
        destination=safe(root,preserved)
        data=destination.read_bytes() if destination.exists() else safe(root,original).read_bytes()
        if digest(data)!=baseline['files'][original]:raise ValueError('legacy_video_page_changed')
        files[preserved]=data
    for name,data in files.items():
        path=safe(root,name)
        before=digest(path.read_bytes()) if path.exists() else None
        expected=baseline['files'].get(name)
        previous=state['files'].get(name)
        if before not in (expected,previous,digest(data)):
            raise ValueError('integration_target_changed:'+name)
    # Check every baseline input before any writes, including compose wiring.
    for name,expected in baseline['files'].items():
        path=safe(root,name)
        if not path.is_file():raise ValueError('missing_integration_baseline:'+name)
        actual=digest(path.read_bytes())
        if actual not in (expected,state['files'].get(name),digest(files.get(name,b''))):
            raise ValueError('integration_baseline_changed:'+name)
    if check:return {'checked':len(files),'modified':False}
    def write(path,data):
        path.parent.mkdir(parents=True,exist_ok=True)
        fd,temporary=tempfile.mkstemp(prefix='.neo2-control-',dir=path.parent)
        try:
            with os.fdopen(fd,'wb') as stream:stream.write(data);stream.flush();os.fsync(stream.fileno())
            os.chmod(temporary,0o644);os.replace(temporary,path)
        finally:
            if os.path.exists(temporary):os.unlink(temporary)
    for name,data in files.items():write(safe(root,name),data)
    result={'format':FORMAT,'files':{name:digest(data) for name,data in files.items()}}
    write(state_file,(json.dumps(result,indent=2)+'\n').encode())
    return {'installed':len(files),'state':str(state_file)}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target',type=Path)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    print(json.dumps(apply(args.target,check=args.check)))
