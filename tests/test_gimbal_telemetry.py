"""SDK field order and recorded provenance, without a camera-pose inference."""
import hashlib
import json
import socket
import struct

import pytest

from neo2_control.capture import CaptureRecorder
from neo2_control.cli import main
from neo2_control.duml import DumlPacket
from neo2_control.gimbal_telemetry import decode_gimbal_fields,read_gimbal_capture
from test_session import status


def body(counter=65530,quaternion=(.25,.5,.75,1.)):
    raw=bytearray(50)
    struct.pack_into('<3h',raw,0,1234,-2345,3456)
    struct.pack_into('<H',raw,12,counter)
    struct.pack_into('<4f',raw,24,*quaternion)
    return bytes(raw)


def record(path,*,counter=65530,stamp=100_000_000_000,tunnel=False,payload=None):
    packet=DumlPacket(4,2,123,0,4,5,body(counter) if payload is None else payload)
    if tunnel:packet=DumlPacket(39,238,321,0,0x51,1,packet.encode()+bytes(22))
    CaptureRecorder(path,decode_annotations=False).record(interface='udp',direction='rx',
        peer='fixture',raw=status(payload=packet.encode()),monotonic_ns=stamp,timestamp_ns=0)
    return path


def test_wire_quaternion_order_matches_native_serializer_without_renaming_orientation_w():
    fields=decode_gimbal_fields(body()).as_dict()
    assert fields['pitch_roll_yaw_degrees']==pytest.approx([123.4,-234.5,345.6])
    assert fields['quaternion_sdk_fields']=={
        'orientation':.25,'rotationX':.5,'rotationy':.75,'rotationZ':1.}
    assert fields['counter12_u16']==65530
    assert fields['quaternion_norm']==pytest.approx((.25**2+.5**2+.75**2+1.)**.5)


@pytest.mark.parametrize('payload',[b'',bytes(49),bytes(51),bytearray(50),
    body(quaternion=(float('nan'),0,0,1)),body(quaternion=(1,0,float('inf'),0))])
def test_unknown_lengths_and_nonfinite_values_never_create_sensor_fields(payload):
    with pytest.raises(ValueError):decode_gimbal_fields(payload)


def test_zero_quaternion_is_preserved_without_fabricating_a_unit_orientation():
    fields=decode_gimbal_fields(body(quaternion=(0,0,0,0))).as_dict()
    assert fields['quaternion_norm']==0
    assert fields['quaternion_sdk_fields']['orientation']==0


def test_forwarded_packets_keep_original_receipts_raw_payloads_and_unresolved_claims(tmp_path):
    source=record(tmp_path/'capture.jsonl',tunnel=True)
    report=read_gimbal_capture(source);row=report['frames'][0]
    assert report['observed_frames']==report['retained_frames']==1
    assert row['capture_line']==1 and row['pc_received_at_ns']==100_000_000_000
    assert row['datagram_carrier_offset']==34 and row['datagram_frame_offset']==45
    assert (row['sender'],row['receiver'],row['sequence'])==(4,2,123)
    assert row['tunnel']=={'sender':39,'receiver':238,'sequence':321,'version':1,'command_type':0}
    assert (row['version'],row['command_type'])==(1,0)
    assert row['payload_hex']==body().hex()
    assert row['payload_sha256']==hashlib.sha256(body()).hexdigest()
    assert report['counter12_unit'] is None and report['quaternion_convention'] is None
    assert not any(report[k] for k in ('camera_frame_verified','camera_exposure_alignment_verified',
        'physical_accuracy_verified','flight_authorized','hardware_used'))


def test_counter_wrap_and_duplicate_deliveries_are_retained_without_clock_interpolation(tmp_path):
    path=tmp_path/'capture.jsonl'
    for i,counter in enumerate((65530,4,4)):
        record(path,counter=counter,stamp=100_000_000_000+i*100_000_000)
    report=read_gimbal_capture(path)
    assert [r['counter12_u16'] for r in report['frames']]==[65530,4,4]
    assert report['observed_frames']==3 and report['counter12_unit'] is None


def test_limit_counts_whole_source_and_drops_bad_or_regressing_receipts(tmp_path):
    path=record(tmp_path/'capture.jsonl')
    record(path,stamp=100_100_000_000);record(path,stamp=99_000_000_000)
    record(path,stamp=2**63);record(path,payload=bytes(49),stamp=100_200_000_000)
    report=read_gimbal_capture(path,maximum_frames=1)
    assert report['observed_frames']==2 and report['retained_frames']==1
    assert report['truncated'] and report['rejected_records']==3
    assert report['sha256']==hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.mark.parametrize('limit',[0,5001,True,1.5])
def test_retention_bound_is_explicit(tmp_path,limit):
    with pytest.raises(ValueError,match='frame_limit'):read_gimbal_capture(tmp_path/'missing',maximum_frames=limit)


def test_bad_crc_and_transmitted_commands_are_not_sensor_observations(tmp_path):
    path=record(tmp_path/'capture.jsonl');row=json.loads(path.read_text())
    bad=bytearray.fromhex(row['raw_hex']);bad[-1]^=1
    with path.open('a') as stream:
        for extra in (dict(row,raw_hex=bad.hex()),dict(row,direction='tx')):
            stream.write(json.dumps(extra)+'\n')
    assert read_gimbal_capture(path)['observed_frames']==1


def test_non_udp_frames_cannot_be_misclassified_by_a_duml_sequence_byte(tmp_path):
    path=record(tmp_path/'mixed.jsonl');base=json.loads(path.read_text())
    with path.open('a') as stream:
        for sequence in (2,3,123):
            raw=DumlPacket(4,2,sequence,0,4,5,body()).encode()
            stream.write(json.dumps(dict(base,interface='ble-duml',raw_hex=raw.hex()))+'\n')
    report=read_gimbal_capture(path)
    assert report['observed_frames']==1 and report['frames'][0]['sequence']==123


def test_unknown_duml_versions_or_command_types_do_not_inherit_the_observed_push_layout(tmp_path):
    path=record(tmp_path/'capture.jsonl');base=json.loads(path.read_text())
    with path.open('a') as stream:
        for version,kind in ((2,0),(1,1),(1,0x80)):
            raw=status(payload=DumlPacket(4,2,123,kind,4,5,body(),version=version).encode())
            stream.write(json.dumps(dict(base,raw_hex=raw.hex()))+'\n')
    report=read_gimbal_capture(path)
    assert report['observed_frames']==1 and report['rejected_records']==3


def test_cli_stays_offline_and_preserves_existing_reports(tmp_path,monkeypatch,capsys):
    path=record(tmp_path/'capture.jsonl');output=tmp_path/'report.json'
    def forbidden(*args,**kw):raise AssertionError('socket opened by offline report')
    monkeypatch.setattr(socket,'socket',forbidden)
    assert main(['gimbal-report',str(path),'-o',str(output)])==0
    assert json.loads(capsys.readouterr().out)['retained_frames']==1
    report=json.loads(output.read_text());assert report['frames'][0]['counter12_u16']==65530
    assert output.stat().st_mode&0o777==0o600
    saved=output.read_bytes()
    assert main(['gimbal-report',str(path),'-o',str(output)])==1
    assert output.read_bytes()==saved


def test_absent_sensor_frames_do_not_create_an_empty_report(tmp_path):
    source=tmp_path/'empty.jsonl';source.write_text('')
    output=tmp_path/'report.json'
    assert main(['gimbal-report',str(source),'-o',str(output)])==1
    assert not output.exists()
