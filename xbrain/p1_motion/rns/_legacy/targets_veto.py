"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: targets_veto.py
Brief: MOT-RD-5 targets veto channel (D_veto folds back to raw geometry)

Description:
20 §7 targets veto: when perception targets are within D_veto
distance of the robot, RNS folds back to RAW OBSTACLE GEOMETRY
mode (does NOT try to steer around a moving target). Rationale: a
moving human is neither an obstacle to route around nor a target to
approach; the safest response is 'stop and wait' rather than 'plan
an evasive path'.

Reverse motion is BANNED here: even if the raw-geometry check
says 'backing up would give clearance', a moving target behind
the robot could be in the reverse path. Better to stop.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


class ReverseNotAllowed(RuntimeError):
    """Attempted reverse motion in targets-veto mode."""


def within_veto_distance(targets_min_dist_m: float,
                          d_veto_m: float) -> bool:
    return targets_min_dist_m < d_veto_m


def geometry_fallback_velocity(current_vx: float) -> Tuple[float, float]:
    """When in veto mode: forward motion capped, reverse REFUSED.
    Result is (vx, wz)."""
    if current_vx < 0:
        raise ReverseNotAllowed(
            "targets veto: reverse (vx=%f) refused; a target behind "
            "may enter the reverse path" % current_vx)
    # Forward motion clipped to zero (safest: stop).
    return (0.0, 0.0)
