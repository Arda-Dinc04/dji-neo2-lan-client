"""Explicit rigid camera-to-body geometry, without physical calibration claims.

Body axes are right/down/forward. Camera axes follow OpenCV. The mount is an
input assumption; moving a gimbal invalidates a fixed mount unless separately
measured. This module does not infer acquisition time or flight authority.
"""
from dataclasses import dataclass
import math
import re

from .metric_reference import _digest
from .metric_route import MetricBodyPose, _unit
from .reconstruction import _finite, _vision


def _rotation(value):
    _, np = _vision()
    try:
        matrix = np.asarray(value, dtype=float)
        if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
                or not np.allclose(matrix.T@matrix, np.eye(3), atol=1e-6, rtol=0)
                or not math.isclose(float(np.linalg.det(matrix)), 1., abs_tol=1e-6)):
            raise ValueError
    except (TypeError, ValueError):
        raise ValueError('body_pose_rotation_invalid') from None
    return matrix


def _hex(value, length):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{%d}' % length, value) is not None


@dataclass(frozen=True)
class RigidCameraMount:
    body_to_camera_R: tuple[tuple[float, float, float], ...]
    camera_center_in_body_m: tuple[float, float, float]
    note: str

    def __post_init__(self):
        _rotation(self.body_to_camera_R)
        if (not isinstance(self.body_to_camera_R, tuple)
                or any(not isinstance(row, tuple) for row in self.body_to_camera_R)
                or not isinstance(self.camera_center_in_body_m, tuple)
                or len(self.camera_center_in_body_m) != 3
                or any(not _finite(v) for v in self.camera_center_in_body_m)
                or sum(v*v for v in self.camera_center_in_body_m) > .5**2
                or not isinstance(self.note, str) or not 1 <= len(self.note.strip()) <= 500):
            raise ValueError('camera_mount_invalid')

    def as_dict(self):
        return {'format': 'neo2-control/rigid-camera-mount/v1',
            'body_axes': 'right_down_forward',
            'body_to_camera_R': [list(row) for row in self.body_to_camera_R],
            'camera_center_in_body_m': list(self.camera_center_in_body_m),
            'note': self.note, 'physical_accuracy_verified': False}

    @classmethod
    def from_dict(cls, value):
        try:
            mount = cls(tuple(tuple(row) for row in value['body_to_camera_R']),
                tuple(value['camera_center_in_body_m']), value['note'])
            if value != mount.as_dict():
                raise ValueError
        except (KeyError, TypeError, ValueError):
            raise ValueError('camera_mount_invalid') from None
        return mount


@dataclass(frozen=True)
class BodyPoseProjection:
    body: MetricBodyPose
    evidence: dict


class CameraBodyProjector:
    def __init__(self, coordinate_frame, map_sha256, mount, *,
            zero_forward=(0., 0., 1.), up=(0., -1., 0.)):
        _, np = _vision()
        if not _hex(coordinate_frame, 64) or not _hex(map_sha256, 64) or not isinstance(mount, RigidCameraMount):
            raise ValueError('body_projector_context_invalid')
        self.coordinate_frame, self.map_sha256 = coordinate_frame, map_sha256
        self._up = np.array(_unit(up))
        forward = np.array(_unit(zero_forward))
        forward -= self._up*float(forward@self._up)
        self._forward = np.array(_unit(forward.tolist()))
        self._right = np.cross(self._forward, self._up)
        self._R_cb = _rotation(mount.body_to_camera_R).copy()
        self._camera_center_b = np.array(mount.camera_center_in_body_m)
        self.mount_sha256 = _digest(mount.as_dict())
        for value in (self._up, self._forward, self._right, self._R_cb, self._camera_center_b):
            value.flags.writeable = False

    def project(self, camera_pose, *, observed_at, flying, motors_on, owner_epoch, generation, synthetic):
        _, np = _vision()
        if (not isinstance(camera_pose, dict) or camera_pose.get('coordinate_frame') != self.coordinate_frame
                or camera_pose.get('map_sha256') != self.map_sha256
                or camera_pose.get('distance_unit') != 'm'
                or not _hex(camera_pose.get('image_sha256'), 64)):
            raise ValueError('camera_pose_metric_context_invalid')
        try:
            R_cw = _rotation(camera_pose['world_to_camera_R'])
            center = np.asarray(camera_pose['camera_center'], dtype=float)
            t_cw = np.asarray(camera_pose['world_to_camera_t'], dtype=float)
            inliers, residual = camera_pose['inliers'], camera_pose['maximum_reprojection_error_px']
            if (center.shape != (3,) or t_cw.shape != (3,) or not np.isfinite(center).all()
                    or not np.isfinite(t_cw).all() or (np.abs(center) > 30).any()
                    or not np.allclose(center, -R_cw.T@t_cw, atol=1e-6, rtol=0)
                    or type(inliers) is not int or inliers < 20
                    or not _finite(residual) or not 0 <= residual <= 2.5):
                raise ValueError
        except (KeyError, TypeError, ValueError):
            raise ValueError('camera_pose_geometry_invalid') from None
        # R_cb maps body vectors to the camera; R_wc maps camera to world.
        R_wb = R_cw.T@self._R_cb
        body_center = center-R_wb@self._camera_center_b
        body_up = -R_wb[:, 1]
        tilt = math.degrees(math.acos(float(np.clip(body_up@self._up, -1, 1))))
        if tilt > 20:
            raise ValueError('body_tilt_outside_model')
        forward = R_wb[:, 2]
        heading = math.degrees(math.atan2(float(forward@self._right), float(forward@self._forward)))
        heading = (heading+180) % 360-180
        body = MetricBodyPose(tuple(float(v) for v in body_center), heading, observed_at,
            flying, motors_on, self.coordinate_frame, owner_epoch, generation, synthetic)
        return BodyPoseProjection(body, {'coordinate_frame': self.coordinate_frame,
            'map_sha256': self.map_sha256, 'image_sha256': camera_pose['image_sha256'],
            'camera_mount_sha256': self.mount_sha256, 'body_to_world_R': R_wb.tolist(),
            'body_tilt_deg': tilt, 'inliers': inliers, 'maximum_reprojection_error_px': residual,
            'assumed_fixed_mount': True, 'physical_accuracy_verified': False,
            'acquisition_time_verified': False, 'flight_authorized': False,
            'scope': 'Body pose computed under an explicit rigid mount and declared gravity axes. Source timing and physical calibration remain external requirements.'})
