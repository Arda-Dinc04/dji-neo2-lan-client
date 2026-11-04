"""Retain the exact audited HEVC input for a continuous recorded-room stream.

The original recording stays untouched. This repeats the existing timing
normalizer and refuses any difference from the retained frame-lineage audit.
No geometry, camera pose, telemetry or drone connection enters the stream.
"""
import argparse,hashlib,json,os,shutil,time
from pathlib import Path

from neo2_control.reconstruction import _read
from neo2_control.room_mvs import digest,write_new
from neo2_control.video_lineage import _TrackedFilter,_normalization_map
from neo2_control.video_export import CHUNK_BYTES,MAX_SOURCE_BYTES,_normalizer,_run


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('prepared','lineage','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    audit_raw=_read(args.prepared/'decoder-copy-audit.json',64*1024**2)
    lineage_raw=_read(args.lineage,64*1024**2)
    audit=json.loads(audit_raw);lineage=json.loads(lineage_raw)
    decoder=Path(audit['prepared_source']['path']);original=Path(audit['source']['path'])
    if (lineage.get('format')!='neo2-control/video-frame-lineage/v1'
            or any(lineage['source'][k]!=audit['prepared_source'][k] for k in ('sha256','bytes'))
            or digest(original)!=audit['source']['sha256'] or digest(decoder)!=lineage['source']['sha256']):
        raise ValueError('room_stream_preparation_source_mismatch')
    ffmpeg,limiter=shutil.which('ffmpeg'),shutil.which('prlimit')
    if not ffmpeg or not limiter:raise ValueError('room_stream_preparation_requires_ffmpeg_prlimit')
    root=args.output.absolute();root.mkdir(mode=0o700)
    try:
        filtered=_TrackedFilter();sha=hashlib.sha256();total=0
        with decoder.open('rb') as incoming,(root/'bootstrap.hevc').open('xb') as output:
            os.fchmod(output.fileno(),0o600)
            while data:=incoming.read(CHUNK_BYTES):
                total+=len(data)
                if total>MAX_SOURCE_BYTES:raise ValueError('room_stream_preparation_source_limit')
                sha.update(data);output.write(filtered.feed(data))
            output.write(filtered.finish())
        if total!=lineage['source']['bytes'] or sha.hexdigest()!=lineage['source']['sha256']:
            raise ValueError('room_stream_preparation_decoder_changed')
        fd=os.open(root,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW)
        try:
            with (root/'bootstrap.hevc').open('rb') as incoming,(root/'canonical.hevc').open('xb') as output:
                os.fchmod(output.fileno(),0o600)
                _,log=_run(_normalizer(limiter,ffmpeg,lineage['preparation']['nominal_fps'],root),
                    directory_fd=fd,deadline=time.monotonic()+90,stage='room_stream_normalize',
                    stdin=incoming,stdout=output)
        finally:os.close(fd)
        _,normalization=_normalization_map(root/'canonical.hevc',filtered.records)
        expected=lineage['preparation']['normalization']
        if normalization!=expected:raise ValueError('room_stream_preparation_normalization_changed')
        if digest(original)!=audit['source']['sha256'] or digest(decoder)!=lineage['source']['sha256']:
            raise ValueError('room_stream_preparation_original_changed')
        for name,raw in (('lineage.json',lineage_raw),('audit.json',audit_raw)):
            fd=os.open(root/name,os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW,0o600)
            with os.fdopen(fd,'wb') as stream:stream.write(raw)
        result={'format':'neo2-control/room-stream-input/v1','source':audit['source'],
            'decoder_source':audit['prepared_source'],'canonical':normalization,
            'lineage_sha256':hashlib.sha256(lineage_raw).hexdigest(),
            'audit_sha256':hashlib.sha256(audit_raw).hexdigest(),
            'frames':lineage['summary']['decoded_frames'],'nominal_fps':lineage['preparation']['nominal_fps'],
            'normalizer_log':log,'seed_map_used':False,'camera_poses_supplied':False,'hardware_used':False}
        write_new(root/'input.json',result);(root/'bootstrap.hevc').unlink()
        print(json.dumps({'output':str(root),'frames':result['frames'],'canonical':normalization}),flush=True)
    except BaseException as error:
        write_new(root/'failure.json',{'error':type(error).__name__,'reason':str(error)})
        raise


if __name__=='__main__':main()
