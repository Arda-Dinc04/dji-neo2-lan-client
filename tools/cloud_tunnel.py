#!/usr/bin/env python3
"""Expose only the loopback controller through a private reverse SSH socket."""
import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess

REPO=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ssh-target',required=True)
    parser.add_argument('--check',action='store_true',help='prepare/check the private cloud socket, without forwarding')
    args=parser.parse_args()
    if re.fullmatch(r'root@[A-Za-z0-9][A-Za-z0-9.-]{0,252}',args.ssh_target) is None:
        raise ValueError('invalid_cloud_ssh_target')
    common=['ssh','-T','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',
        '-o','ConnectTimeout=8','-o','ControlMaster=no','-o','ControlPath=none']
    source=(REPO/'src/neo2_control/tunnel_socket.py').read_text()
    source+='\nprint(prepare_socket(),flush=True)\n'
    subprocess.run([*common,args.ssh_target,'python3','-c',shlex.quote(source)],
        check=True,stdin=subprocess.DEVNULL,timeout=15)
    if args.check:return
    command=[*common,'-N','-o','ExitOnForwardFailure=yes','-o','StreamLocalBindUnlink=no',
        '-o','StreamLocalBindMask=0177','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=2',
        '-R','/run/neo2-control/flight.sock:127.0.0.1:8794',args.ssh_target]
    os.execvp(command[0],command)


if __name__=='__main__':main()
