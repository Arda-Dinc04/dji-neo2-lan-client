"""Recorded gimbal fields, without camera-frame or sensor-clock attribution.

The SDK serializer establishes field order, not quaternion conventions or
optical alignment. This module never unwraps counter12 or produces a pose.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import struct

from .duml import find_packets
from .reconstruction import _json_file
from .transport import describe_transport
from .wlm_discovery import unpack_forwarded


@dataclass(frozen=True)
class GimbalFields:
    pitch_degrees: float
    roll_degrees: float
    yaw_degrees: float
    counter12_u16: int
    orientation: float
    rotation_x: float
    rotation_y: float
    rotation_z: float

    def as_dict(self):
        return {'pitch_roll_yaw_degrees':[self.pitch_degrees,self.roll_degrees,self.yaw_degrees],
            'counter12_u16':self.counter12_u16,
            'quaternion_sdk_fields':{'orientation':self.orientation,'rotationX':self.rotation_x,
                'rotationy':self.rotation_y,'rotationZ':self.rotation_z},
            'quaternion_norm':math.hypot(self.orientation,self.rotation_x,self.rotation_y,self.rotation_z)}


def decode_gimbal_fields(payload):
    if not isinstance(payload,bytes) or len(payload)!=50:
        raise ValueError('gimbal_payload_length_invalid')
    angles=struct.unpack_from('<3h',payload)
    quaternion=struct.unpack_from('<4f',payload,24)
    if not all(math.isfinite(v) for v in quaternion):
        raise ValueError('gimbal_quaternion_nonfinite')
    return GimbalFields(*(v*.1 for v in angles),struct.unpack_from('<H',payload,12)[0],*quaternion)


def read_gimbal_capture(path,*,maximum_frames=5000):
    """Retain complete CRC-checked UDP RX packets and receipt timestamps.

    Duplicate deliveries stay visible. There is no stream reassembly,
    owning-session validation, clock interpolation or reference-frame fit.
    """
    if type(maximum_frames)is not int or not 1<=maximum_frames<=5000:
        raise ValueError('gimbal_frame_limit_invalid')
    frames=[];digest=hashlib.sha256();observed=rejected=read_bytes=line_number=0;last_stamp=-1
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK|os.O_CLOEXEC)
    with os.fdopen(fd,'rb') as source:
        before=os.fstat(source.fileno())
        if not stat.S_ISREG(before.st_mode) or before.st_size>512*1024*1024:
            raise ValueError('gimbal_capture_size_invalid')
        while line:=source.readline(256*1024+1):
            line_number+=1
            if len(line)>256*1024:raise ValueError('gimbal_capture_line_too_large')
            read_bytes+=len(line)
            if read_bytes>512*1024*1024:raise ValueError('gimbal_capture_size_invalid')
            digest.update(line)
            try:
                row=json.loads(line)
                if not isinstance(row,dict):raise ValueError
                if row.get('direction')!='rx' or row.get('interface')!='udp':continue
                stamp=row['monotonic_ns'];raw_hex=row['raw_hex']
                if type(stamp)is not int or not 0<=stamp<2**63:raise ValueError
                if not isinstance(raw_hex,str) or len(raw_hex)>131070:raise ValueError
                raw=bytes.fromhex(raw_hex)
                if describe_transport(raw)[0]['type']==2:continue
            except (ValueError,KeyError,TypeError):
                rejected+=1;continue
            for offset,packet,_ in find_packets(raw):
                tunnel=None
                if (packet.cmd_set,packet.cmd_id)==(0x51,1):
                    tunnel={'sender':packet.sender,'receiver':packet.receiver,'sequence':packet.seq,
                        'version':packet.version,'command_type':packet.cmd_type}
                    try:packet=unpack_forwarded(packet.payload).packet
                    except ValueError:continue
                if (packet.cmd_set,packet.cmd_id)!=(4,5):continue
                if packet.version!=1 or packet.cmd_type!=0:
                    rejected+=1;continue
                try:fields=decode_gimbal_fields(packet.payload)
                except ValueError:rejected+=1;continue
                if stamp<last_stamp:rejected+=1;continue
                last_stamp=stamp;observed+=1
                if len(frames)>=maximum_frames:continue
                frames.append({'pc_received_at_ns':stamp,'capture_line':line_number,
                    'datagram_carrier_offset':offset,'datagram_frame_offset':offset+(11 if tunnel else 0),
                    'tunnel':tunnel,'sender':packet.sender,
                    'receiver':packet.receiver,'sequence':packet.seq,'version':packet.version,
                    'command_type':packet.cmd_type,'payload_hex':packet.payload.hex(),
                    'payload_sha256':hashlib.sha256(packet.payload).hexdigest(),**fields.as_dict()})
        after=os.fstat(source.fileno())
    if (before.st_size,before.st_mtime_ns,before.st_ctime_ns)!=(after.st_size,after.st_mtime_ns,after.st_ctime_ns):
        raise ValueError('gimbal_capture_changed')
    if not frames:raise ValueError('gimbal_capture_has_no_supported_frames')
    return {'format':'neo2-control/gimbal-raw/v1','source':str(Path(path).absolute()),
        'sha256':digest.hexdigest(),'observed_frames':observed,'retained_frames':len(frames),
        'truncated':observed>len(frames),'rejected_records':rejected,'frames':frames,
        'angle_unit':'degree; SDK pitch/roll/yaw order, unresolved reference frame',
        'counter12_unit':None,'quaternion_convention':None,
        'camera_exposure_alignment_verified':False,'camera_frame_verified':False,
        'physical_accuracy_verified':False,'hardware_used':False,'flight_authorized':False,
        'scope':'Complete CRC-checked DUML V1 type0x00 04/05 bodies in UDP RX datagrams, unwrap 51/01 once, no stream reassembly or owning-session attribution. PC receipt only; opaque counter12; no camera/body transform, pose, interpolation or flight authority.'}


def write_gimbal_report(capture,output):
    report=read_gimbal_capture(capture)
    _json_file(output,report)
    return {k:v for k,v in report.items() if k!='frames'}
