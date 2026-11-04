"""Build a bounded JPEG replay source from audited recording pixels, without poses."""
import argparse,hashlib,json,os
from pathlib import Path

from neo2_control.reconstruction import _read
from neo2_control.room_mvs import write_new
from neo2_control.room_stream import audited_frames


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recording',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--first-index',type=int,default=0)
    p.add_argument('--end-index',type=int)
    p.add_argument('--stride',type=int,choices=(1,2,3,4),default=2)
    args=p.parse_args()
    import cv2 as cv
    cv.setNumThreads(1)
    source=json.loads(_read(args.recording/'input.json',65536))
    args.output.mkdir(mode=0o700);(args.output/'frames').mkdir(mode=0o700)
    rows=[];total=0
    try:
        for f in audited_frames(args.recording/'canonical.hevc',args.recording/'lineage.json',
                args.recording/'audit.json',first=args.first_index,end=args.end_index,stride=args.stride):
            ok,data=cv.imencode('.jpg',cv.cvtColor(f.rgb,cv.COLOR_RGB2BGR),[cv.IMWRITE_JPEG_QUALITY,90])
            if not ok:raise ValueError('room_preview_jpeg_encode_failed')
            raw=data.tobytes();total+=len(raw)
            if len(rows)>=4096 or len(raw)>4*1024**2 or total>128*1024**2:
                raise ValueError('room_preview_cache_budget')
            name=f'frame-{len(rows):05d}.jpg'
            with (args.output/'frames'/name).open('xb') as stream:
                os.fchmod(stream.fileno(),0o600);stream.write(raw)
            rows.append({'file':name,'sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw),
                'decoded_index':f.decoded_index,'recorded_available_at':f.recorded_available_at,
                'source_rgb_sha256':f.metadata['rgb_sha256'],
                'source_vcl_body_sha256':f.metadata['source_vcl_body_sha256']})
        manifest={'format':'neo2-control/recorded-room-preview/v1','source':source['source'],
            'frames':rows,'jpeg_bytes':total,'source_pixels_verified':True,
            'encoding':'lossy_jpeg_quality_90','poses_supplied':False,'hardware_used':False}
        write_new(args.output/'preview.json',manifest)
        print(json.dumps({'frames':len(rows),'jpeg_bytes':total}),flush=True)
    except BaseException as error:
        write_new(args.output/'failure.json',{'reason':str(error)})
        raise


if __name__=='__main__':main()
