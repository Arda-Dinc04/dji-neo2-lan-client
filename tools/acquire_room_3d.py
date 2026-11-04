"""Acquire progressive 3D from a connected camera or a paced recording.

The camera mode uses read-only preview requests. No connection, camera action,
motor command, seed map or precomputed camera pose is supplied to the mapper.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import signal
import time

from neo2_control.room_mvs import write_new,digest
from neo2_control.reconstruction import read_camera_model


def select_pending(pending,batch_size):
    """Bound work while retaining the newest image and spaced bridge views."""
    if type(batch_size) is not int or batch_size not in (1,2,4):raise ValueError('acquisition_batch_size_invalid')
    if len(pending)<=batch_size:return pending
    if batch_size==1:return [pending[-1]]
    return [pending[i*(len(pending)-1)//(batch_size-1)] for i in range(batch_size)]


def partition_pending(pending,batch_size,bridge_backlog=0):
    """Keep a bounded chronological bridge reserve without changing clocks.

    A reserve delays the newest exposure by at most the retained work instead
    of permanently discarding every intermediate view. Its count bounds work,
    not elapsed time: actual end-to-end source age must still be measured.
    """
    chosen=select_pending(pending,batch_size)
    if type(bridge_backlog) is not int or bridge_backlog not in (0,4,8):
        raise ValueError('acquisition_bridge_backlog_invalid')
    if bridge_backlog==0:return chosen,[],[p for p in pending if p not in chosen]
    if batch_size==1:raise ValueError('acquisition_bridge_backlog_requires_batch')
    limit=batch_size+bridge_backlog
    kept=(pending if len(pending)<=limit else
        [pending[i*(len(pending)-1)//(limit-1)] for i in range(limit)])
    return kept[:batch_size],kept[batch_size:],[p for p in pending if p not in kept]


def worker(root,camera,context,stop,feature_backend,batch_size,separate_mapping=False,dense_stereo=False,async_stereo=False,
        estimated_model=None,estimated_device='CPU',estimated_grid_side=96,estimated_prefetch=False,analysis_nice=10,
        native_facets=True,estimated_warm_start=False,guided_tracking=False,native_expansion=False,estimated_locked_transfer=False,estimated_reprojected_transfer=False,
        native_matching_workers=2,feature_runtime=None,global_track_target=None,heading_retrieval=False,local_pose_recovery=False,
        bridge_backlog=0,estimated_stereo_calibration=False,historical_query_budget=None,native_direct_verification=False):
    import resource,traceback
    os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='2'
    os.nice(analysis_nice)
    resource.setrlimit(resource.RLIMIT_AS,(4*1024**3,(8 if estimated_model else 4)*1024**3))
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    log=os.open(root/'mapper.log',os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
    os.dup2(log,1);os.dup2(log,2);os.close(log)
    from neo2_control.progressive_room import IncrementalRoomMap,_atomic_json
    mapper=None;admission=None
    try:
        if separate_mapping:
            from spatial_capture.progressive_acquisition import AsyncRoomMap
            mapper=AsyncRoomMap(root,camera,context,feature_backend=feature_backend,
                dense_stereo=dense_stereo,async_stereo=async_stereo,
                estimated_model=estimated_model,estimated_device=estimated_device,estimated_grid_side=estimated_grid_side,
                estimated_prefetch=estimated_prefetch,native_facets=native_facets,
                estimated_warm_start=estimated_warm_start,guided_tracking=guided_tracking,native_expansion=native_expansion,
                estimated_locked_transfer=estimated_locked_transfer,estimated_reprojected_transfer=estimated_reprojected_transfer,
                native_matching_workers=native_matching_workers,feature_runtime=feature_runtime,
                global_track_target=global_track_target,heading_retrieval=heading_retrieval,local_pose_recovery=local_pose_recovery,
                estimated_stereo_calibration=estimated_stereo_calibration,historical_query_budget=historical_query_budget,
                native_direct_verification=native_direct_verification)
        else:mapper=IncrementalRoomMap(root/'map',camera,context,feature_backend=feature_backend,
            dense_stereo=dense_stereo,async_stereo=async_stereo,native_facets=native_facets,
            guided_tracking=guided_tracking,native_expansion=native_expansion,native_matching_workers=native_matching_workers,
            feature_runtime=feature_runtime,global_track_target=global_track_target,heading_retrieval=heading_retrieval,
            local_pose_recovery=local_pose_recovery,historical_query_budget=historical_query_budget,
            native_direct_verification=native_direct_verification)
        write_new(root/'analysis-ready.json',{'ready':True,'estimated_depth':estimated_model is not None,
            'process_nice':os.getpriority(os.PRIO_PROCESS,0)})
        seen=set();updates=[];skipped=[];peak_deferred=0;admission_sequence=0
        admission=(root/'admission-events.jsonl').open('x');os.fchmod(admission.fileno(),0o600)
        while True:
            if separate_mapping:mapper.check()
            pending=[p for p in sorted((root/'incoming').glob('*.json')) if p.name not in seen]
            selected,deferred,discarded=partition_pending(pending,batch_size,bridge_backlog)
            if pending:
                admission_sequence+=1;peak_deferred=max(peak_deferred,len(deferred))
                admission.write(json.dumps({'sequence':admission_sequence,'observed_at':time.monotonic(),
                    'pending':[p.name for p in pending],'selected':[p.name for p in selected],
                    'deferred':[p.name for p in deferred],'discarded':[p.name for p in discarded]})+'\n')
                admission.flush()
            for path in discarded:
                skipped.append(json.loads(path.read_text()));seen.add(path.name)
            for path in selected:
                packet=json.loads(path.read_text());raw=(root/'incoming'/packet['file']).read_bytes()
                if hashlib.sha256(raw).hexdigest()!=packet['sha256']:raise ValueError('acquisition_input_changed')
                mapper.add(raw,packet['received_at'],packet['context'],packet['metadata']);seen.add(path.name)
            if pending:
                snapshot=mapper.update()
                if not separate_mapping:
                    snapshot['skipped_for_backpressure']=len(skipped)
                    updates.append({k:v for k,v in snapshot.items() if k not in ('points','rgb','patches','poses')})
                    _atomic_json(root/f'update-{len(updates):04d}.json',updates[-1])
            elif stop.is_set():break
            else:time.sleep(.1)
        admission.close()
        asynchronous=mapper.finish() if separate_mapping else {'stereo_shutdown':mapper.close()}
        write_new(root/'worker-result.json',{**asynchronous,'state':'finished','inputs':len(seen)-len(skipped),
            'bridge_backlog_limit':bridge_backlog,'peak_deferred_frames':peak_deferred,
            'skipped_for_backpressure':skipped,'updates':asynchronous.get('updates',updates),
            'seed_map_used':False,'live_hardware_validated':False})
    except BaseException:
        if admission is not None:admission.close()
        if mapper is not None:mapper.abort()
        traceback.print_exc()
        write_new(root/'worker-failure.json',{'state':'failed'})
        raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--replay',type=Path,help='prepared directory with images.json and images/')
    source.add_argument('--live-url',help='already-connected loopback flight-web origin')
    parser.add_argument('--altitude-session-events',type=Path,
        help='recorded FC observations for automatic scale; requires --replay, uses past measurements only')
    parser.add_argument('--camera',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--period',type=float,default=1.)
    parser.add_argument('--duration',type=float,default=120.)
    parser.add_argument('--feature-backend',choices=('orb','sift','native_sift','native_sift_960','opencv_sift_960','vulkan_sift_960'),default='sift')
    parser.add_argument('--feature-runtime',type=Path,help='local hashed Vulkan SIFT runtime directory')
    parser.add_argument('--global-track-target',type=int,
        help='experimental initial global tracks per view, 40–512; final retriangulation uses all observations')
    parser.add_argument('--heading-retrieval',action='store_true',
        help='experimental 75-degree measured-heading hint for historical native descriptor retrieval')
    parser.add_argument('--historical-query-budget',type=int,choices=(16,),default=None,
        help='experimental limit on historical native retrieval; retain early, recent and spaced source views')
    parser.add_argument('--analysis-batch-size',type=int,choices=(1,2,4),default=1,
        help='maximum frames per update; all captured images remain on disk')
    parser.add_argument('--bridge-backlog',type=int,choices=(0,4,8),default=0,
        help='experimental bounded reserve of intermediate views; requires analysis batch size 2 or 4')
    parser.add_argument('--native-matching-workers',type=int,choices=(2,4,6),default=2,
        help='native SIFT matching/verification CPU workers; extraction, retrieval and mapping keep their own budgets')
    parser.add_argument('--native-direct-verification',action='store_true',
        help='experimental direct native two-view verification with the same thresholds and deterministic seed0')
    parser.add_argument('--separate-mapping',action='store_true',
        help='run geometry in a separate process, with one pending immutable database')
    parser.add_argument('--guided-tracking',action='store_true',
        help='try visual landmark registration guided by past measured height/yaw; periodically refresh native geometry')
    parser.add_argument('--native-expansion',action='store_true',
        help='after guided tracking fails, register the current image from native matches and triangulate without moving the existing world')
    parser.add_argument('--local-pose-recovery',action='store_true',
        help='experimental preloaded local joint pose recovery with a 2.5-second job budget; requires guided tracking')
    parser.add_argument('--no-native-facets',dest='native_facets',action='store_false',
        help='keep verified native points and poses without constructing their optional photographic facets')
    parser.add_argument('--dense-stereo',action='store_true',
        help='add bounded stereo surfaces independently checked in three views')
    parser.add_argument('--async-stereo',action='store_true',
        help='measure stereo in one bounded worker while SfM registers later views')
    parser.add_argument('--estimated-depth-model',type=Path,
        help='local hashed depth IR; separate estimated surface layer, requires --separate-mapping')
    parser.add_argument('--depth-device',choices=('CPU','GPU'),default='CPU')
    parser.add_argument('--depth-grid-side',type=int,choices=(64,96,128),default=96,
        help='longest estimated fusion grid side; smaller values give coarser geometry')
    parser.add_argument('--depth-prefetch',action='store_true',
        help='predict received pixels on the depth worker before their visual poses are available')
    parser.add_argument('--depth-warm-start',action='store_true',
        help='refit a previous scalar depth fit on current training landmarks, with current withheld validation')
    parser.add_argument('--depth-locked-transfer',action='store_true',
        help='validate frozen coefficients from a query-free past fit when current landmarks cannot support a new fit')
    parser.add_argument('--depth-reprojected-transfer',action='store_true',
        help='fit a weak view from one pinned past inferred field, hold out all current native points, and keep a primary-only grid')
    parser.add_argument('--depth-stereo-calibration',action='store_true',
        help='experimental three-photo stereo targets for weak views; native landmarks remain withheld, one group per update')
    parser.add_argument('--analysis-nice',type=int,choices=(0,10),default=10,
        help='0 keeps normal inherited process priority; 10 lowers analysis priority')
    args=parser.parse_args()
    if args.bridge_backlog and args.analysis_batch_size==1:
        parser.error('--bridge-backlog requires --analysis-batch-size 2 or 4')
    if args.global_track_target is not None and not 40<=args.global_track_target<=512:
        parser.error('--global-track-target must be between 40 and 512')
    if (args.feature_backend=='vulkan_sift_960')!=(args.feature_runtime is not None):
        parser.error('--feature-runtime is required only for vulkan_sift_960')
    if args.native_matching_workers!=2 and args.feature_backend not in ('native_sift','native_sift_960','opencv_sift_960','vulkan_sift_960'):
        parser.error('--native-matching-workers requires a native matching backend')
    if args.native_direct_verification and args.feature_backend not in ('native_sift','native_sift_960','opencv_sift_960','vulkan_sift_960'):
        parser.error('--native-direct-verification requires a native matching backend')
    if args.heading_retrieval and args.feature_backend not in ('native_sift','native_sift_960','opencv_sift_960','vulkan_sift_960'):
        parser.error('--heading-retrieval requires a native matching backend')
    if args.historical_query_budget is not None and args.feature_backend not in ('native_sift','native_sift_960','opencv_sift_960','vulkan_sift_960'):
        parser.error('--historical-query-budget requires a native matching backend')
    if args.guided_tracking and args.feature_backend not in ('sift','opencv_sift_960','vulkan_sift_960'):
        parser.error('--guided-tracking requires sift, opencv_sift_960 or vulkan_sift_960')
    if args.native_expansion and not args.guided_tracking:
        parser.error('--native-expansion requires --guided-tracking')
    if args.local_pose_recovery and not args.guided_tracking:
        parser.error('--local-pose-recovery requires --guided-tracking')
    if args.altitude_session_events and not args.replay:
        parser.error('--altitude-session-events requires --replay')
    if args.async_stereo and not args.dense_stereo:parser.error('--async-stereo requires --dense-stereo')
    if args.estimated_depth_model and not args.separate_mapping:
        parser.error('--estimated-depth-model requires --separate-mapping')
    if args.depth_prefetch and not args.estimated_depth_model:
        parser.error('--depth-prefetch requires --estimated-depth-model')
    if args.depth_warm_start and not args.estimated_depth_model:
        parser.error('--depth-warm-start requires --estimated-depth-model')
    if args.depth_locked_transfer and not args.estimated_depth_model:
        parser.error('--depth-locked-transfer requires --estimated-depth-model')
    if args.depth_reprojected_transfer and not args.estimated_depth_model:
        parser.error('--depth-reprojected-transfer requires --estimated-depth-model')
    if args.depth_stereo_calibration and not args.estimated_depth_model:
        parser.error('--depth-stereo-calibration requires --estimated-depth-model')
    if not .1<=args.period<=5 or not 1<=args.duration<=600:raise ValueError('acquisition_limits_invalid')
    root=args.output.absolute();root.mkdir(mode=0o700);(root/'incoming').mkdir(mode=0o700)
    camera=read_camera_model(args.camera);client=None;manifest=None;altitude=None
    if args.replay:
        manifest=json.loads((args.replay/'images.json').read_text());context='recorded:'+manifest['source']['sha256']
        if args.altitude_session_events:
            from neo2_control.room_altitude import RecordedAltitude
            altitude=RecordedAltitude(args.altitude_session_events)
    else:
        from neo2_control.operator_client import FlightOperatorClient
        client=FlightOperatorClient(args.live_url);status=client.status();client.pin_connection(status)
        if status.get('connected') is not True or status.get('connection',{}).get('video',{}).get('live') is not True:
            raise ValueError('acquisition_requires_connected_live_preview')
        context=':'.join(client._expected_context)
    write_new(root/'capture-configuration.json',{'source':'recording_at_original_delivery_cadence' if args.replay else 'read_only_preview',
        'context':context,'period_seconds':args.period,'duration_limit_seconds':args.duration,
        'camera':camera.as_dict(),'seed_map':None,'maximum_keyframes':96,'feature_backend':args.feature_backend,
        'analysis_batch_size':args.analysis_batch_size,'native_facets':args.native_facets,'guided_tracking':args.guided_tracking,
        'native_matching_workers':args.native_matching_workers,
        'native_direct_verification':args.native_direct_verification,
        'feature_runtime':str(args.feature_runtime.resolve()) if args.feature_runtime else None,
        'global_track_target':args.global_track_target,
        'heading_retrieval':args.heading_retrieval,
        'historical_query_budget':args.historical_query_budget,
        'native_expansion':args.native_expansion,
        'local_pose_recovery':args.local_pose_recovery,'bridge_backlog':args.bridge_backlog,
        'estimated_locked_transfer':args.depth_locked_transfer or args.depth_reprojected_transfer,
        'estimated_reprojected_transfer':args.depth_reprojected_transfer,
        'estimated_stereo_calibration':args.depth_stereo_calibration,
        'mapping_execution':'separate_process' if args.separate_mapping else 'inline',
        'maximum_pending_map_requests':1 if args.separate_mapping else 0,
        'dense_stereo':args.dense_stereo,
        'async_stereo':args.async_stereo,
        'estimated_depth_model':str(args.estimated_depth_model.resolve()) if args.estimated_depth_model else None,
        'estimated_depth_device':args.depth_device if args.estimated_depth_model else None,
        'estimated_depth_grid_side':args.depth_grid_side if args.estimated_depth_model else None,
        'estimated_depth_prefetch':args.depth_prefetch,
        'estimated_depth_warm_start':args.depth_warm_start,
        'analysis_nice':args.analysis_nice,
        'altitude_source':'recorded_measured_FC' if altitude is not None else (
            'existing_read_only_status' if client else None),
        'altitude_session_events_sha256':digest(args.altitude_session_events) if altitude is not None else None,
        'pc_boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()})
    import neo2_control.progressive_room as backend_module
    import neo2_control.preview_acquisition as preview_module
    import neo2_control.descriptor_matching as matching_module
    implementation_paths=[Path(__file__).resolve(),Path(backend_module.__file__),
        Path(preview_module.__file__),Path(matching_module.__file__)]
    from spatial_capture import altitude_scale,global_recovery
    from neo2_control import room_altitude
    implementation_paths.extend(Path(module.__file__) for module in (altitude_scale,room_altitude,global_recovery))
    if args.feature_runtime:
        from spatial_capture import vulkan_features
        implementation_paths.append(Path(vulkan_features.__file__))
        implementation_paths.extend(args.feature_runtime.resolve()/name for name in
            ('manifest.json','libvulkansift.so','spatial_vulkan_bridge.so'))
    if args.guided_tracking:
        from spatial_capture import native_tracking,landmark_matching
        implementation_paths.extend(Path(module.__file__) for module in (native_tracking,landmark_matching))
    if args.native_expansion:
        from spatial_capture import native_expansion
        implementation_paths.append(Path(native_expansion.__file__))
    if args.local_pose_recovery:
        from spatial_capture import local_pose_recovery,local_pose_worker,native_snapshot,native_expansion,process_rpc
        implementation_paths.extend(Path(module.__file__) for module in
            (local_pose_recovery,local_pose_worker,native_snapshot,native_expansion,process_rpc))
    if args.separate_mapping:
        from spatial_capture import progressive_acquisition,mapping_worker,latest_worker
        implementation_paths.extend(Path(module.__file__) for module in
            (progressive_acquisition,mapping_worker,latest_worker))
    if args.dense_stereo:
        from spatial_capture import dense_stereo,dense_room
        implementation_paths.extend(Path(module.__file__) for module in (dense_stereo,dense_room))
    if args.estimated_depth_model:
        from spatial_capture import estimated_worker,estimated_room,estimated_surface,relative_depth,depth_field
        implementation_paths.extend(Path(module.__file__) for module in
            (estimated_worker,estimated_room,estimated_surface,relative_depth,depth_field))
        if args.depth_prefetch:
            from spatial_capture import depth_prefetch
            implementation_paths.append(Path(depth_prefetch.__file__))
        if args.depth_locked_transfer or args.depth_reprojected_transfer:
            from spatial_capture import depth_transfer
            implementation_paths.append(Path(depth_transfer.__file__))
        if args.depth_reprojected_transfer:
            from spatial_capture import projected_depth
            implementation_paths.append(Path(projected_depth.__file__))
    write_new(root/'implementation.json',{'files_sha256':{str(path):digest(path) for path in implementation_paths}})
    mp=multiprocessing.get_context('spawn');stop=mp.Event()
    process=mp.Process(target=worker,args=(root,camera,context,stop,args.feature_backend,args.analysis_batch_size,
        args.separate_mapping,args.dense_stereo,args.async_stereo,
        args.estimated_depth_model,args.depth_device,args.depth_grid_side,args.depth_prefetch,args.analysis_nice,
        args.native_facets,args.depth_warm_start,args.guided_tracking,args.native_expansion,args.depth_locked_transfer,args.depth_reprojected_transfer,
        args.native_matching_workers,args.feature_runtime,args.global_track_target,args.heading_retrieval,args.local_pose_recovery,
        args.bridge_backlog,args.depth_stereo_calibration,args.historical_query_budget,args.native_direct_verification));process.start()
    if args.estimated_depth_model:
        startup_deadline=time.monotonic()+40
        while not (root/'analysis-ready.json').exists():
            if not process.is_alive():process.join();raise RuntimeError('acquisition_analysis_startup_failed')
            if time.monotonic()>startup_deadline:
                process.terminate();process.join(5)
                if process.is_alive():process.kill();process.join(5)
                raise TimeoutError('acquisition_analysis_startup_timeout')
            time.sleep(.1)
    count=0;began=time.monotonic();last_report=0;reason='source_finished'
    def packet(raw,metadata,received_at=None):
        nonlocal count
        if count>=96:raise ValueError('acquisition_keyframe_limit')
        if not process.is_alive():raise RuntimeError('acquisition_mapper_stopped')
        if len(raw)>4*1024**2:raise ValueError('acquisition_frame_size_limit')
        received=time.monotonic() if received_at is None else received_at
        suffix='.png' if raw.startswith(b'\x89PNG') else '.jpg'
        name=f'input-{count:05d}{suffix}';fd=os.open(root/'incoming'/name,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
        with os.fdopen(fd,'wb') as stream:stream.write(raw)
        record={'file':name,'sha256':hashlib.sha256(raw).hexdigest(),'received_at':received,
            'context':context,'metadata':metadata}
        temp=root/'incoming'/f'packet-{count:05d}.new';write_new(temp,record)
        os.replace(temp,temp.with_suffix('.json'));count+=1
    def report():
        nonlocal last_report
        paths=sorted(root.glob('update-*.json'))
        if paths and len(paths)>last_report:
            r=json.loads(paths[-1].read_text());last_report=len(paths)
            print(json.dumps({'event':'map_updated',**r}),flush=True)
    try:
        if args.replay:
            rows=sorted(manifest['frames'],key=lambda f:f['selection']['picture']['last_byte_delivered_monotonic'])
            origin=rows[0]['selection']['picture']['last_byte_delivered_monotonic']
            for row in rows:
                due=began+row['selection']['picture']['last_byte_delivered_monotonic']-origin
                while time.monotonic()<due:report();time.sleep(min(.1,max(0,due-time.monotonic())))
                raw=(args.replay/'images'/row['file']).read_bytes()
                if hashlib.sha256(raw).hexdigest()!=row['png_sha256']:raise ValueError('acquisition_replay_image_changed')
                metadata={'original_file':row['file'],'recorded_delivery_monotonic':row['selection']['picture']['last_byte_delivered_monotonic'],
                    'original_source':manifest['source'],'scope':'recorded input; no camera pose supplied'}
                if altitude is not None:
                    observation=altitude.at(metadata['recorded_delivery_monotonic'],replayed_at=due)
                    if observation is not None:metadata['altitude_observation']=observation
                packet(raw,metadata,received_at=due)
                report()
        else:
            from neo2_control.preview_acquisition import ReadOnlyPreviewFeed
            feed=ReadOnlyPreviewFeed(client,client._expected_context)
            while time.monotonic()-began<args.duration and count<96:
                frame=feed.read()
                if frame is not None:packet(frame['raw'],frame['metadata'],frame['received_at'])
                report();time.sleep(args.period)
            if count>=96:reason='keyframe_budget_reached'
    except KeyboardInterrupt:reason='operator_stopped_analysis'
    except BaseException as error:
        reason='capture_failed'
        write_new(root/'source-failure.json',{'error':type(error).__name__,'reason':str(error)[:400]})
        raise
    finally:
        stop.set();deadline=time.monotonic()+45
        while process.is_alive() and time.monotonic()<deadline:
            report();process.join(.2)
        if process.is_alive():process.terminate();process.join(5);reason='mapper_shutdown_timeout'
        if process.is_alive():process.kill();process.join(5)
        report()
        write_new(root/'capture-result.json',{'captured_keyframes':count,'wall_seconds':time.monotonic()-began,
            'mapper_exitcode':process.exitcode,'reason':reason,'worker_finished':(root/'worker-result.json').exists(),
            'failed':reason in ('capture_failed','mapper_shutdown_timeout') or process.exitcode!=0,
            'seed_map_used':False,'input_pose_source':None,'live_hardware_validated':False})
    if process.exitcode!=0:raise SystemExit(1)

if __name__=='__main__':main()
