"""Recorded proximity layout/report only; no hardware, depth or flight model."""
import json
import struct

import pytest

from neo2_control.capture import CaptureRecorder
from neo2_control.duml import DumlPacket
from neo2_control.obstacles import decode_birdview, read_birdview_capture, write_birdview_report
from test_session import status


def payload(values=(471,60000,3377),*,count=None):
    return b'\x01\xa3'+struct.pack('<3H',2280,51,len(values) if count is None else count)+struct.pack('<'+'H'*len(values),*values)


def capture(path):
    packet=DumlPacket(146,2,123,0,0x24,8,payload())
    recorder=CaptureRecorder(path,decode_annotations=False)
    recorder.record(interface='udp',direction='rx',peer='fixture',raw=status(payload=packet.encode()),
        monotonic_ns=100_000_000_000,timestamp_ns=1_000_000_000)
    return path


def test_native_layout_preserves_unknown_flags_values_and_special_code_without_units():
    value=decode_birdview(payload())
    assert value.header_hex=='01a3'
    assert value.up_raw==2280 and value.down_raw==51
    assert value.horizontal_raw==(471,60000,3377)
    assert decode_birdview(payload((60000,)*360)).horizontal_raw==(60000,)*360


@pytest.mark.parametrize('body',[b'',b'\0'*9,payload()[:-1],payload()+b'\0',
    payload(count=361),payload(count=0),payload(count=2),bytearray(payload())])
def test_truncated_oversized_and_inconsistent_counts_do_not_create_distances(body):
    with pytest.raises(ValueError):decode_birdview(body)


def test_capture_report_retains_digest_raw_values_and_ownership_limit(tmp_path):
    path=capture(tmp_path/'capture.jsonl')
    value=read_birdview_capture(path)
    assert value['observed_frames']==value['retained_frames']==1
    assert value['frames'][0]['horizontal_raw']==[471,60000,3377]
    assert value['frames'][0]['seconds']==0
    assert 'without stream reassembly' in value['scope']
    assert value['units'].startswith('raw')
    output=tmp_path/'report.html';summary=write_birdview_report(path,output)
    assert summary['sha256']==value['sha256']
    page=output.read_text()
    assert 'ne constitue pas un scan 3D' in page and '60000' in page
    assert '<canvas' in page
    with pytest.raises(FileExistsError):write_birdview_report(path,output)


def test_report_escapes_capture_path_and_refuses_symlink(tmp_path):
    path=capture(tmp_path/'<script>.jsonl')
    output=tmp_path/'view.html';write_birdview_report(path,output)
    text=output.read_text()
    assert '&lt;script&gt;.jsonl' in text and '\\u003cscript\\u003e.jsonl' in text
    link=tmp_path/'link';link.symlink_to(path)
    with pytest.raises(OSError):read_birdview_capture(link)


@pytest.mark.parametrize('name',['__DATA__.jsonl','__SOURCE__.jsonl'])
def test_capture_names_cannot_be_interpreted_as_report_template_fields(tmp_path,name):
    path=capture(tmp_path/name);output=tmp_path/'report.html'
    write_birdview_report(path,output)
    page=output.read_text()
    dataset=json.loads(page.split('<script id="dataset" type="application/json">',1)[1].split('</script>',1)[0])
    assert dataset['source']==str(path.absolute())
    assert 'Source : '+name in page


def test_absent_or_bad_frames_never_mean_clear_space(tmp_path):
    path=tmp_path/'empty.jsonl';path.write_text(json.dumps({'direction':'rx','monotonic_ns':True,'raw_hex':''})+'\n')
    with pytest.raises(ValueError,match='no_supported_frames'):read_birdview_capture(path)


def test_frame_limit_preserves_whole_capture_hash_and_counts_without_inventing_samples(tmp_path):
    import hashlib
    path=capture(tmp_path/'capture.jsonl')
    row=json.loads(path.read_text())
    second=dict(row,monotonic_ns=row['monotonic_ns']+100_000_000)
    regressing=dict(row,monotonic_ns=row['monotonic_ns']-1)
    impossible=dict(row,monotonic_ns=2**100)
    with path.open('a') as stream:
        for value in (second,regressing,impossible):stream.write(json.dumps(value)+'\n')
    result=read_birdview_capture(path,maximum_frames=1)
    assert result['observed_frames']==2 and result['retained_frames']==1
    assert result['truncated'] and result['rejected_records']==2
    assert result['sha256']==hashlib.sha256(path.read_bytes()).hexdigest()


def test_bad_crc_is_not_a_measurement_and_transmitted_packets_are_not_sensor_observations(tmp_path):
    path=capture(tmp_path/'capture.jsonl')
    row=json.loads(path.read_text())
    bad=bytearray.fromhex(row['raw_hex']);bad[-1]^=1
    with path.open('a') as stream:
        stream.write(json.dumps(dict(row,raw_hex=bad.hex()))+'\n')
        stream.write(json.dumps(dict(row,direction='tx'))+'\n')
    result=read_birdview_capture(path)
    assert result['observed_frames']==1


def test_cli_creates_report_and_reports_no_supported_data_without_clobbering(tmp_path,capsys):
    from neo2_control.cli import main
    path=capture(tmp_path/'capture.jsonl');output=tmp_path/'report.html'
    assert main(['obstacles-report',str(path),'-o',str(output)])==0
    assert json.loads(capsys.readouterr().out)['retained_frames']==1
    original=output.read_bytes()
    assert main(['obstacles-report',str(path),'-o',str(output)])==1
    assert output.read_bytes()==original
    capsys.readouterr()
    empty=tmp_path/'empty.jsonl';empty.write_text('')
    assert main(['obstacles-report',str(empty),'-o',str(tmp_path/'absent.html')])==1
    assert 'no_supported_frames' in capsys.readouterr().err
    assert not (tmp_path/'absent.html').exists()
