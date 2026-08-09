"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: motion_behavior.py
Brief: BIZ-P2-26 -- mode_motion -> cmd/motion/behavior mapping (NAV-50, CFG-40)

Description:
When P2 enters D (alarm) or B (broadcast) mode, it publishes
cmd/motion/behavior to P1 telling it HOW to move relative to the
target. The mapping table is p2_core.yaml.mode_motion.d_alarm /
b_cast, whose behavior field is a closed 3-value set
{face_target_stop, face_target_follow, hold}.

The values keep_dist_m / max_speed_mps / stop_at_fence are advisory
inputs to face_target_follow only; the speed gate on P1 clips
max_speed_mps to whatever f() and g() decide (12 S6).

This module owns the mapping ONLY (mode + config -> BehaviorCommand).
Actual publish goes through P2Publisher.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class MotionBehaviorParams:
    """From p2_core.yaml.mode_motion.<mode>.params."""
    keep_dist_m: float
    max_speed_mps: float
    stop_at_fence: bool


@dataclass(frozen=True)
class BehaviorCommand:
    """cmd/motion/behavior message shape."""
    behavior: str                    # closed set: face_target_stop / follow / hold
    keep_dist_m: Optional[float]
    max_speed_mps: Optional[float]
    stop_at_fence: bool


def command_for_mode(
    mode: str,
    mode_motion_cfg: dict,
    on_target_lost: str = "hold",
) -> Optional[BehaviorCommand]:
    """Return the behavior command for `mode`, or None if the mode
    has no configured behavior (IDLE / DIALOG etc.).

    mode_motion_cfg is p2_core.yaml.mode_motion (dict with d_alarm /
    b_cast keys).
    """
    if mode not in ("d_alarm", "b_cast"):
        return None
    block = mode_motion_cfg.get(mode)
    if block is None:
        return None
    behavior = block["behavior"]
    params = block.get("params", {})
    return BehaviorCommand(
        behavior=behavior,
        keep_dist_m=params.get("keep_dist_m"),
        max_speed_mps=params.get("max_speed_mps"),
        stop_at_fence=params.get("stop_at_fence", False),
    )


def command_for_target_lost(on_target_lost: str) -> BehaviorCommand:
    """After a target is confirmed lost (target_track.tick_frame_absent
    threshold), publish behavior 'hold' or 'resume_task' per config."""
    if on_target_lost not in ("hold", "resume_task"):
        raise ValueError(
            "on_target_lost=%r not in {hold, resume_task}" % on_target_lost)
    return BehaviorCommand(
        behavior=on_target_lost,
        keep_dist_m=None,
        max_speed_mps=None,
        stop_at_fence=False,
    )
