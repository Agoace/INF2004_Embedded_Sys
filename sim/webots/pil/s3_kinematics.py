"""Nominal no-slip S3 evidence geometry, NOT a plant or firmware correction."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2:
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_rad: float = 0.0


def _check(*values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite kinematic input')


def advance(pose, left_mm, right_mm, track_mm=111.0):
    _check(pose.x_mm, pose.y_mm, pose.yaw_rad,
           left_mm, right_mm, track_mm)
    if track_mm <= 0:
        raise ValueError('track must be positive')
    ds = (left_mm + right_mm) / 2
    angle = (right_mm - left_mm) / track_mm
    half = angle / 2
    sinc = math.sin(half)/half if abs(half) > 1e-9 else 1 - half*half/6
    return Pose2(pose.x_mm + ds*sinc*math.cos(pose.yaw_rad + half),
                 pose.y_mm + ds*sinc*math.sin(pose.yaw_rad + half),
                 pose.yaw_rad + angle)


def point_world(pose, forward_mm=80.0, left_mm=77.0):
    _check(pose.x_mm, pose.y_mm, pose.yaw_rad, forward_mm, left_mm)
    c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    return (pose.x_mm + c*forward_mm - s*left_mm,
            pose.y_mm + s*forward_mm + c*left_mm)


def point_velocity_body(left_mm_s, right_mm_s, forward_mm=80.0,
                        left_mm=77.0, track_mm=111.0):
    _check(left_mm_s, right_mm_s, forward_mm, left_mm, track_mm)
    if track_mm <= 0:
        raise ValueError('track must be positive')
    v = (left_mm_s + right_mm_s) / 2
    omega = (right_mm_s - left_mm_s) / track_mm
    return (v - omega*left_mm, omega*forward_mm)


def stripe_progress(point, origin, normal):
    """Signed projection onto fixed WORLD stripe normal, not travelled length."""
    _check(*point, *origin, *normal)
    length = math.hypot(*normal)
    if length == 0:
        raise ValueError('stripe normal must be nonzero')
    return sum((p-o)*n for p, o, n in zip(point, origin, normal))/length
