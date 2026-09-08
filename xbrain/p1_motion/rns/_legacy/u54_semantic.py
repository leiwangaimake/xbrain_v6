"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: u54_semantic.py
Brief: MOT-RD-10 U54 one-way safety-distance semantics (RNS-U1..U4 + A-U-1)

Description:
99 U54 the safety-distance rule is ONE-WAY (RNS-U1..RNS-U4):
  U-1  robot's OWN motion may not bring it within safety_distance
       of a person / obstacle
  U-2  a person moving toward a STATIONARY robot -> robot may fall
       within safety_distance PASSIVELY (stopped is fine, retreating
       is NOT required nor encouraged -- retreat could steer into
       another hazard)
  U-3  when the person moves AWAY (widening the gap), robot may
       resume motion
  U-4  the check is per-frame; a single tick with a person entering
       the ring does NOT trigger retreat -- it triggers STOP

A-U-1: target-level (b-type) slowdown; V5's approach is ported --
smoothly decelerate to a stop as the target approaches, so the
final stop is comfortable rather than abrupt.
"""

from __future__ import annotations


def is_within_safety(dist_m: float, safety_m: float) -> bool:
    """Person is inside the safety ring."""
    return dist_m < safety_m


def should_stop_but_not_retreat(
    dist_m: float,
    safety_m: float,
    robot_vx_mps: float,
) -> bool:
    """U-1..U-4: if person is inside safety AND robot is moving,
    stop. But do NOT decide to retreat -- the caller sets vx=0,
    not vx negative."""
    if not is_within_safety(dist_m, safety_m):
        return False
    if robot_vx_mps <= 0:
        # already stopped or reversing (caller decision); no action here
        return False
    return True


def type_b_slowdown_factor(
    dist_m: float,
    d_start_slowdown_m: float,
    safety_m: float,
) -> float:
    """A-U-1: linear slowdown factor from d_start down to safety.
    Above d_start: factor 1 (no slowdown).
    Below safety: factor 0 (stop).
    In between: linear ramp."""
    if dist_m >= d_start_slowdown_m:
        return 1.0
    if dist_m <= safety_m:
        return 0.0
    denom = d_start_slowdown_m - safety_m
    if denom <= 0:
        return 0.0
    return (dist_m - safety_m) / denom
