"""Feed verified recording pixels or a connected read-only preview to Spatial Capture.

No connection opening, camera action, telemetry pose or flight intent is sent.
The drone is unnecessary for the recording mode.
"""
import argparse,hashlib,json,time
from contextlib import closing
from pathlib import Path

from neo2_control.reconstruction import read_camera_model,_read
from neo2_control.room_mvs import write_new
from spatial_capture.stream_acquisition import Acquisition


def main():
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--recording',type=Path)
    source.add_argument('--live-url')
    for name in ('camera','native-command-file','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--first-index',type=int,default=0);p.add_argument('--end-index',type=int)
    p.add_argument('--stride',type=int,choices=(1,2,3,4),default=2)
    p.add_argument('--working-width',type=int,default=640)
    p.add_argument('--min-size',type=int,default=200)
    p.add_argument('--fast-threshold',type=int,default=10)
    p.add_argument('--duration',type=float,default=120.)
    p.add_argument('--fps',type=float,default=15.)
    p.add_argument('--map-period',type=float,default=1.)
    p.add_argument('--maximum-age',type=float,default=.5)
    p.add_argument('--decoder-threads',type=int,choices=(1,2,3,4),default=2)
    p.add_argument('--decoder-thread-type',choices=('SLICE','FRAME'),default='FRAME')
    p.add_argument('--time-aware-motion',action='store_true')
    p.add_argument('--native-log-level',choices=('warn','info','debug'),default='warn')
    p.add_argument('--synchronous-publication',action='store_true')
    args=p.parse_args()
    if not 1<=args.duration<=600 or not 1<=args.fps<=30:raise ValueError('stream_capture_limits_invalid')
    camera=read_camera_model(args.camera)
    command=json.loads(_read(args.native_command_file,65536))['command']
    client=feed=None
    if args.recording:
        manifest=json.loads(_read(args.recording/'input.json',65536))
        context='recorded:'+manifest['source']['sha256']
        kind='recording_at_original_delivery_cadence'
        fps=manifest['nominal_fps']/args.stride
    else:
        from neo2_control.operator_client import FlightOperatorClient
        from neo2_control.preview_acquisition import ReadOnlyPreviewFeed
        client=FlightOperatorClient(args.live_url);status=client.status();client.pin_connection(status)
        context=':'.join(client._expected_context)
        feed=ReadOnlyPreviewFeed(client,client._expected_context)
        kind='read_only_preview';fps=args.fps
    acquisition=Acquisition(args.output,camera,context,command,source=kind,
        working_width=args.working_width,fps=fps,min_size=args.min_size,
        fast_threshold=args.fast_threshold,map_period=args.map_period,maximum_age=args.maximum_age,
        time_aware_motion=args.time_aware_motion,native_log_level=args.native_log_level,
        asynchronous_publication=not args.synchronous_publication)
    reason='source_finished';source_error=None;started=time.monotonic()
    try:
        if args.recording:
            from neo2_control.room_stream import audited_frames
            write_new(args.output/'decoder-configuration.json',{
                'thread_type':args.decoder_thread_type,'threads':args.decoder_threads,
                'availability_clock':'picture packet and prior coded inputs; independent of decoder buffering'})
            origin=recorded_origin=None
            drops=(args.output/'decoder-drops.jsonl').open('x',encoding='utf8')
            def accept(index,available):
                if origin is None:return True
                due=origin+available-recorded_origin
                now=time.monotonic()
                if now-due<=args.maximum_age:return True
                drops.write(json.dumps({'decoded_index':index,'recorded_available_at':available,
                    'replay_due_at':due,'dropped_at':now,'reason':'stale_before_rgb_conversion',
                    'encoded_packet_verified':True,'rgb_pixels_verified':False})+'\n');drops.flush()
                return False
            iterator=audited_frames(args.recording/'canonical.hevc',args.recording/'lineage.json',
                args.recording/'audit.json',first=args.first_index,end=args.end_index,stride=args.stride,
                accept=accept,decoder_threads=args.decoder_threads,
                decoder_thread_type=args.decoder_thread_type)
            with drops,closing(iterator):
                for frame in iterator:
                    if origin is None:
                        origin=time.monotonic();recorded_origin=frame.recorded_available_at
                        write_new(args.output/'replay-origin.json',{'pc_monotonic':origin,
                            'recorded_available_at':recorded_origin,'first_decoded_index':frame.decoded_index,
                            'scope':'Native ready and first input decoded before replay clock starts'})
                    due=origin+frame.recorded_available_at-recorded_origin
                    while time.monotonic()<due:time.sleep(min(.05,max(0,due-time.monotonic())))
                    if time.monotonic()-origin>=args.duration:reason='duration_limit';break
                    acquisition.submit_rgb(frame.rgb,timestamp=frame.nominal_timestamp,
                        available_at=due,context=context,metadata=frame.metadata|{'replay_due_at':due})
        else:
            import cv2 as cv
            import numpy as np
            while time.monotonic()-started<args.duration:
                poll_started=time.monotonic()
                frame=feed.read()
                if frame is not None:
                    bgr=cv.imdecode(np.frombuffer(frame['raw'],dtype=np.uint8),cv.IMREAD_COLOR)
                    if bgr is None:raise ValueError('stream_capture_preview_decode_failed')
                    metadata=frame['metadata']
                    # Conservative availability bound; exposure time is unknown.
                    available=metadata['read_started_at']-metadata['encoded_age_seconds']
                    acquisition.submit_rgb(cv.cvtColor(bgr,cv.COLOR_BGR2RGB),
                        timestamp=metadata['read_finished_at'],available_at=available,context=context,
                        metadata=metadata|{'encoded_sha256':hashlib.sha256(frame['raw']).hexdigest()})
                time.sleep(max(0,1/args.fps-(time.monotonic()-poll_started)))
    except KeyboardInterrupt:reason='operator_stopped_analysis'
    except BaseException as error:
        reason='source_failed';source_error=error
        write_new(args.output/'source-failure.json',{'reason':str(error)})
    finally:result=acquisition.finish(reason)
    print(json.dumps(result),flush=True)
    if source_error is not None:raise source_error


if __name__=='__main__':main()
