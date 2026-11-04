"""Repeated localization in a frozen client map, without flight or radio I/O.

Distances describe camera centers in the saved map's units. Ordered images
provide no acquisition clock or aircraft extrinsics. A missing pose stays
missing; no previous position is reused or interpolated across a loss.
"""
import hashlib
from pathlib import Path

from .descriptor_matching import FrozenDescriptorMatcher
from .metric_atlas import _assemble, _fit_features
from .landmark_region import propose_region
from .metric_reference import _digest, _rotation_error, metric_geometry
from .reconstruction import CameraModel, _features, _finite, _image, _validate_scan, _vision, localize_correspondences


class MapLocalizer:
    """Validate and assemble one immutable map snapshot before processing frames."""

    def __init__(self,scan,*,maximum_feature_pixels=None,region_search=False):
        if type(region_search) is not bool:raise ValueError('tracking_region_option_invalid')
        _,np=_vision()
        kind=scan.get('format') if isinstance(scan,dict) else None
        if kind=='neo2-control/metric-atlas/v1':
            self.model,points,_,descriptors=_assemble(scan)
            self.seed_count=len(scan['seed']['scan']['points'])
        elif kind in ('neo2-control/sparse-scan/v1','neo2-control/metric-scan/v1'):
            geometry=metric_geometry(scan) if kind=='neo2-control/metric-scan/v1' else _validate_scan(scan)
            self.model=CameraModel.from_dict(geometry['camera_model'])
            points=geometry['points'];descriptors=geometry['reference_descriptors']
            self.seed_count=len(points)
        else:
            raise ValueError('tracking_requires_landmark_map')
        self._points=np.array(points,dtype=np.float64,copy=True)
        self._matcher=FrozenDescriptorMatcher(np.asarray(descriptors,dtype=np.float32))
        self._descriptors=self._matcher._descriptors
        self._points.setflags(write=False)
        self.coordinate_frame=scan['coordinate_frame']
        self.distance_unit=scan['distance_unit']
        self.map_sha256=_digest(scan)
        if maximum_feature_pixels is not None and (type(maximum_feature_pixels) is not int
                or not 640*360<=maximum_feature_pixels<=1280*720):
            raise ValueError('feature_pixel_budget_invalid')
        self.maximum_feature_pixels=maximum_feature_pixels
        self.region_search=region_search

    def _solve(self,frame):
        matching={}
        coarse=[]
        def match(features):
            pairs,details=self._matcher.match(features);matching.update(details)
            coarse[:]=pairs
            return pairs
        pixels,features=_features(frame,maximum_pixels=self.maximum_feature_pixels)
        region=None
        try:
            pose,evidence=_fit_features(pixels,features,self.model,self._points,self._descriptors,
                descriptor_matcher=match)
        except ValueError:
            if (not self.region_search or self.maximum_feature_pixels is None
                    or frame.shape[0]*frame.shape[1]<=self.maximum_feature_pixels):raise
            region=propose_region(self._points,pixels,coarse,self.model)
            x0,y0,x1,y1=region['bounds_xyxy']
            pixels,features=_features(frame[y0:y1,x0:x1])
            pixels+= [x0,y0]
            # Start the original >=20-landmark fit afresh, without using the
            # low-support pose as an initial estimate or adding its votes.
            pose,evidence=_fit_features(pixels,features,self.model,self._points,self._descriptors,
                descriptor_matcher=match)
        pose['descriptor_matching']=matching
        if region is not None:pose['region_search']=region
        return pose,evidence

    def _format_pose(self,pose,evidence,source_hash,method):
        added=sum(i>=self.seed_count for i in evidence['landmark_indices'])
        return pose|{'coordinate_frame':self.coordinate_frame,'distance_unit':self.distance_unit,
            'map_sha256':self.map_sha256,'image_sha256':source_hash,
            'tracking_method':method,
            'maximum_feature_pixels':self.maximum_feature_pixels,
            'map_point_count':len(self._points),'added_landmark_inliers':added,
            'seed_landmark_inliers':len(evidence['landmark_indices'])-added,
            'physical_accuracy_verified':False,'tracking_accuracy_verified':False,
            'free_space_observed':False}

    def _localize(self,frame,source_hash):
        return self._format_pose(*self._solve(frame),source_hash,'descriptors')

    def localize(self,image):
        return self._localize(*_image(image,self.model))

    def localize_bytes(self,raw):
        """Process an encoded image; callers retain separate receipt/timing evidence."""
        cv,np=_vision()
        if not isinstance(raw,bytes) or not 0<len(raw)<=32*1024*1024:
            raise ValueError('tracking_encoded_image_invalid')
        frame=cv.imdecode(np.frombuffer(raw,dtype=np.uint8),cv.IMREAD_COLOR)
        if frame is None or frame.shape!=(self.model.height,self.model.width,3):
            raise ValueError('reconstruction_image_or_size_invalid')
        return self._localize(frame,hashlib.sha256(raw).hexdigest())

    def localize_rgb(self,raw,width,height):
        """Localize immutable packed decoder pixels without JPEG re-encoding."""
        cv,np=_vision()
        if (not isinstance(raw,bytes) or type(width) is not int or type(height) is not int
                or (width,height)!=(self.model.width,self.model.height)
                or len(raw)!=width*height*3 or len(raw)>1920*1080*3):
            raise ValueError('tracking_rgb_image_invalid')
        frame=cv.cvtColor(np.frombuffer(raw,dtype=np.uint8).reshape(height,width,3),cv.COLOR_RGB2BGR)
        return self._localize(frame,hashlib.sha256(raw).hexdigest())|{'image_format':'rgb24'}


class TemporalMapTracker(MapLocalizer):
    """Track observed landmark pixels, then solve against their fixed 3D points.

    Bidirectional flow, patch correlation and PnP all have to agree. Failure
    falls back to descriptor localization. No dead reckoning fills a loss.
    """

    def __init__(self,scan,*,maximum_feature_pixels=None,maximum_flow_frames=None,
            maximum_flow_rotation_degrees=None,region_search=False):
        if maximum_flow_frames is not None and (type(maximum_flow_frames) is not int
                or not 0<=maximum_flow_frames<=60):
            raise ValueError('tracking_flow_frame_limit_invalid')
        if maximum_flow_rotation_degrees is not None and (not _finite(maximum_flow_rotation_degrees)
                or not .1<=maximum_flow_rotation_degrees<=45.):
            raise ValueError('tracking_flow_rotation_limit_invalid')
        super().__init__(scan,maximum_feature_pixels=maximum_feature_pixels,region_search=region_search)
        self.maximum_flow_frames=maximum_flow_frames
        self.maximum_flow_rotation_degrees=maximum_flow_rotation_degrees
        self.reset()

    def reset(self):self._previous=None;self._flow_frames=0;self._descriptor_R=None

    def _flow(self,gray):
        cv,np=_vision()
        old,pixels,ids=self._previous
        input_count=len(ids)
        dst,valid,error=cv.calcOpticalFlowPyrLK(old,gray,pixels,None,winSize=(21,21),maxLevel=4)
        if dst is None or valid is None or error is None:raise ValueError('tracking_flow_unavailable')
        keep=(valid.ravel()!=0)&np.isfinite(dst).all(axis=(1,2))&np.isfinite(error.ravel())&(error.ravel()<=25)
        keep&=(dst[:,0,0]>=7)&(dst[:,0,0]<self.model.width-7)
        keep&=(dst[:,0,1]>=7)&(dst[:,0,1]<self.model.height-7)
        if keep.sum()<20:raise ValueError('tracking_flow_insufficient_points')
        ids=ids[keep];pixels=pixels[keep];dst=dst[keep]
        back,valid_back,_=cv.calcOpticalFlowPyrLK(gray,old,dst,None,winSize=(21,21),maxLevel=4)
        if back is None or valid_back is None:raise ValueError('tracking_flow_unavailable')
        fb=np.linalg.norm(back-pixels,axis=2).ravel()
        keep=(valid_back.ravel()!=0)&np.isfinite(fb)&(fb<=.75)
        # Brightness residual alone can admit unrelated textured patches after
        # a large cut. Require the same local pattern, after removing its mean.
        for i in np.flatnonzero(keep):
            a=cv.getRectSubPix(old,(15,15),tuple(map(float,pixels[i,0]))).astype(np.float32)
            b=cv.getRectSubPix(gray,(15,15),tuple(map(float,dst[i,0]))).astype(np.float32)
            a-=a.mean();b-=b.mean()
            aa,bb=float(np.sum(a*a)),float(np.sum(b*b))
            keep[i]=min(aa,bb)>=64*225 and float(np.sum(a*b))/np.sqrt(aa*bb)>=.9
        if keep.sum()<20:raise ValueError('tracking_flow_insufficient_consistent_patches')
        pose=localize_correspondences(self._points[ids[keep]],dst[keep,0],self.model,include_inlier_indices=True)
        selected=np.array(pose.pop('inlier_indices'))
        # A small surviving subset can fit PnP after a large image cut even
        # when most previous landmark identities have been lost. Reacquire
        # descriptors instead of extending that weak temporal association.
        if len(selected)<.25*input_count:
            raise ValueError('tracking_flow_insufficient_retained_landmarks')
        evidence={'landmark_indices':ids[keep][selected].tolist(),'pixels':dst[keep,0][selected].tolist()}
        return pose,evidence,{'maximum_forward_backward_error_px':float(fb[keep][selected].max()),
            'input_landmarks':input_count,'retained_landmarks':len(selected),
            'minimum_retained_fraction':.25}

    def _localize(self,frame,source_hash):
        cv,np=_vision();gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
        details={};fallback=None
        refresh=(self._previous is not None and self.maximum_flow_frames is not None
            and self._flow_frames>=self.maximum_flow_frames)
        refresh_reason='flow_frame_limit' if refresh else None
        if self._previous is not None and not refresh:
            try:
                pose,evidence,details=self._flow(gray)
                method='optical_flow'
                if (self.maximum_flow_rotation_degrees is not None
                        and _rotation_error(np.array(pose['world_to_camera_R']),self._descriptor_R)
                            >self.maximum_flow_rotation_degrees):
                    # A successful PnP fit need not make LK's pixel identities
                    # reliable after rotation. Discard it and reacquire them.
                    refresh=True;refresh_reason='rotation_limit';details={}
            except (ValueError,cv.error) as error:
                fallback=str(error) if isinstance(error,ValueError) else 'tracking_flow_failed'
        if self._previous is None or fallback is not None or refresh:
            pose,evidence=self._solve(frame);method='descriptors'
            self._flow_frames=0
            self._descriptor_R=np.array(pose['world_to_camera_R'])
        else:self._flow_frames+=1
        self._previous=(gray,np.array(evidence['pixels'],dtype=np.float32).reshape(-1,1,2),
            np.array(evidence['landmark_indices']))
        result=self._format_pose(pose,evidence,source_hash,method)
        if details:result['flow_consistency']=details
        if fallback is not None:result['flow_fallback_reason']=fallback
        if self.maximum_flow_frames is not None or self.maximum_flow_rotation_degrees is not None:
            result.update(maximum_flow_frames=self.maximum_flow_frames,
                maximum_flow_rotation_degrees=self.maximum_flow_rotation_degrees,
                flow_frames_since_descriptors=self._flow_frames,descriptor_refresh=refresh,
                descriptor_refresh_reason=refresh_reason)
        return result

    def localize(self,image):
        try:return super().localize(image)
        except (ValueError,OSError):self.reset();raise

    def localize_bytes(self,raw):
        try:return super().localize_bytes(raw)
        except ValueError:self.reset();raise

    def localize_rgb(self,raw,width,height):
        try:return super().localize_rgb(raw,width,height)
        except ValueError:self.reset();raise


def track_images(scan,images,*,temporal=True,event_sink=None):
    """Localize each supplied image in order; temporal gaps are never bridged."""
    _,np=_vision()
    paths=list(images)
    if not 1<=len(paths)<=600:raise ValueError('tracking_image_count_invalid')
    if type(temporal) is not bool:raise ValueError('tracking_mode_invalid')
    localizer=(TemporalMapTracker if temporal else MapLocalizer)(scan)
    samples=[];first=None;previous=None;segment=0
    for index,path in enumerate(paths):
        sample={'index':index,'image':str(Path(path)), 'pose':None,'segment':None,
            'displacement_from_first':None,'distance_from_first':None,
            'translation_from_previous':None,'rotation_from_previous_degrees':None}
        try:
            pose=localizer.localize(path)
        except (ValueError,OSError) as error:
            sample.update(state='lost',reason=str(error))
            previous=None
        else:
            if first is None:first=pose
            if previous is None:segment+=1
            delta=np.array(pose['camera_center'])-np.array(first['camera_center'])
            sample.update(state='localized',reason=None,pose=pose,segment=segment,
                displacement_from_first=delta.tolist(),distance_from_first=float(np.linalg.norm(delta)))
            if previous is not None:
                sample['translation_from_previous']=float(np.linalg.norm(
                    np.array(pose['camera_center'])-np.array(previous['camera_center'])))
                sample['rotation_from_previous_degrees']=_rotation_error(
                    np.array(pose['world_to_camera_R']),np.array(previous['world_to_camera_R']))
            previous=pose
        samples.append(sample)
        if event_sink is not None:
            event_sink({key:sample[key] for key in ('index','image','state','reason','segment')})
    return {'format':'neo2-control/visual-track/v1','map_sha256':localizer.map_sha256,
        'coordinate_frame':localizer.coordinate_frame,'distance_unit':localizer.distance_unit,
        'temporal_tracking':temporal,
        'camera_model':localizer.model.as_dict(),'samples':samples,
        'localized_count':sum(s['state']=='localized' for s in samples),
        'lost_count':sum(s['state']=='lost' for s in samples),'continuous_segments':segment,
        'physical_accuracy_verified':False,'tracking_accuracy_verified':False,
        'acquisition_synchronization_verified':False,'current_pose_verified':False,
        'flight_authorized':False,
        'scope':'Offline camera-center estimates in one frozen map. Input order is not an acquisition clock. Missing poses remain null; adjacent motion is absent across losses. No aircraft extrinsics, velocity, free-volume evidence or flight authority.'}
