#!/usr/bin/env python3
"""Render guarded user services; never start a service or connect a drone."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile

REPO=Path(__file__).resolve().parents[1]
FORMAT='neo2-control/user-services/v1'


def digest(data):return hashlib.sha256(data).hexdigest()


def quote(value):
    value=str(value)
    if any(c in value for c in ('\n','\r','\0')):raise ValueError('invalid_unit_argument')
    return '"'+value.replace('\\','\\\\').replace('"','\\"').replace('%','%%')+'"'


def install(directory,interface,target,profile,*,check=False,yaw_channel_limit=200):
    if re.fullmatch(r'[A-Za-z0-9_.-]{1,15}',interface) is None:raise ValueError('invalid_flight_interface')
    if re.fullmatch(r'root@[A-Za-z0-9][A-Za-z0-9.-]{0,252}',target) is None:raise ValueError('invalid_cloud_ssh_target')
    if profile not in ('ordinary','yaw-trial','vertical-trial'):raise ValueError('invalid_flight_profile')
    if type(yaw_channel_limit) is not int or not 1<=yaw_channel_limit<=600:
        raise ValueError('invalid_yaw_channel_limit')
    if profile!='yaw-trial' and yaw_channel_limit!=200:raise ValueError('yaw_channel_limit_requires_yaw_trial')
    root=Path(directory).absolute()
    if any(p.is_symlink() for p in (root,*root.parents)):raise ValueError('unit_directory_symlink_refused')
    state_path=root/'neo2-control-install.json'
    if state_path.is_symlink():raise ValueError('unit_state_symlink_refused')
    state=json.loads(state_path.read_text()) if state_path.exists() else {'format':FORMAT,'files':{}}
    if state.get('format')!=FORMAT or not isinstance(state.get('files'),dict):raise ValueError('invalid_unit_state')
    # WorkingDirectory is a scalar setting, unlike ExecStart's argument list;
    # quotation marks there would become part of the actual path.
    values={'REPO':str(REPO).replace('%','%%'),'PYTHON':quote(REPO/'.venv/bin/python'),
        'TUNNEL':quote(REPO/'tools/cloud_tunnel.py'),'INTERFACE':quote(interface),
        'SSH_TARGET':quote(target),'PROFILE':('' if profile=='ordinary' else '--'+profile)
            +(f' --yaw-channel-limit {yaw_channel_limit}' if yaw_channel_limit!=200 else '')}
    files={}
    for name in ('neo2-flight-web.service','neo2-cloud-tunnel.service'):
        text=(REPO/'deploy/pc'/(name+'.in')).read_text()
        for key,value in values.items():text=text.replace('@'+key+'@',value)
        files[name]=text.encode()
        path=root/name
        if path.is_symlink():raise ValueError('unit_symlink_refused')
        if path.exists() and digest(path.read_bytes()) not in (digest(files[name]),state['files'].get(name)):
            raise ValueError('unit_changed:'+name)
    if check:return {'checked':list(files),'modified':False}
    root.mkdir(parents=True,exist_ok=True)
    def write(path,data):
        fd,name=tempfile.mkstemp(prefix='.neo2-',dir=root)
        try:
            with os.fdopen(fd,'wb') as stream:stream.write(data);stream.flush();os.fsync(stream.fileno())
            os.chmod(name,0o644);os.replace(name,path)
        finally:
            if os.path.exists(name):os.unlink(name)
    for name,data in files.items():write(root/name,data)
    write(state_path,(json.dumps({'format':FORMAT,'files':{n:digest(d) for n,d in files.items()}},indent=2)+'\n').encode())
    return {'installed':list(files),'started':False}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,default=Path.home()/'.config/systemd/user')
    parser.add_argument('--interface',required=True)
    parser.add_argument('--ssh-target',required=True)
    parser.add_argument('--profile',choices=('ordinary','yaw-trial','vertical-trial'),default='ordinary')
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--yaw-channel-limit',type=int,default=200)
    args=parser.parse_args()
    print(json.dumps(install(args.directory,args.interface,args.ssh_target,args.profile,
        check=args.check,yaw_channel_limit=args.yaw_channel_limit)))
