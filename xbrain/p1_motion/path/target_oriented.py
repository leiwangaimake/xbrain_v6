"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: target_oriented.py
Brief: MOT-PM-23 target_oriented (400) face-stop / follow / hold + boundary

Description:
target_oriented is P1 priority 400. It consumes BehaviorCommand
from P2 (mode_motion mapping) with three modes:
  face_target_stop   face target, no linear motion
  face_target_follow face target + close to keep_dist_m via v_max
  hold               zero-vel

keep_dist_m and max_speed_mps are REQUIRED fields on
BehaviorCommand.params -- writing a Python default (dataclass
default_factory or kwargs default) would let missing config produce
a robot that quietly moves without a keep-distance target, which is
the failure direction 16 S4.3.2 warns against.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


class SchemaError(RuntimeError):
    """keep_dist_m or max_speed_mps missing."""


@dataclass(frozen=True)
class TargetOrientedParams:
    """From BehaviorCommand.params. Both REQUIRED, no defaults."""
    keep_dist_m: float
    max_speed_mps: float
    stop_at_fence: bool


def compute_face_target(
    target_dx_m: float, target_dy_m: float,
    params: TargetOrientedParams,
    mode: str,
) -> tuple:
    """Compute (vx, vy, wz) command for face-target modes.

    mode in {face_target_stop, face_target_follow, hold}.

    face_target_stop: wz only, no linear motion.
    face_target_follow: wz + vx toward target until keep_dist_m.
    hold: zero.
    """
    import math
    if mode == "hold":
        return (0.0, 0.0, 0.0)

    dist = (target_dx_m ** 2 + target_dy_m ** 2) ** 0.5
    bearing = math.atan2(target_dy_m, target_dx_m)   # radians

    if mode == "face_target_stop":
        # wz alone; no linear velocity.
        return (0.0, 0.0, _wz_from_bearing(bearing))

    if mode == "face_target_follow":
        # Close distance until >= keep_dist_m. Sign of vx: positive
        # (move forward) when target is ahead and further than
        # keep_dist; negative (back off) when target is closer than
        # keep_dist.
        if dist > params.keep_dist_m:
            vx = min(params.max_speed_mps, 0.5 * (dist - params.keep_dist_m))
        elif dist < params.keep_dist_m * 0.8:
            vx = -min(params.max_speed_mps, 0.3)   # slow back-off
        else:
            vx = 0.0
        return (vx, 0.0, _wz_from_bearing(bearing))

    raise SchemaError("unknown target_oriented mode: %r" % mode)


def _wz_from_bearing(bearing_rad: float) -> float:
    """Bearing to angular velocity via P controller. Bounded."""
    kp = 1.0
    return max(-1.0, min(1.0, kp * bearing_rad))
