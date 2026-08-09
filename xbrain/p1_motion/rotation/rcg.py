"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rcg.py
Brief: MOT-PM-10/11/12 rotation permit judge (RCG-1..RCG-4)

Description:
The speed gate only constrains linear motion. Angular motion (wz) needs its own check: with no forward progress you cannot 'see' obstacles at your side move away. RCG-1 requires clearance of r_eff around the robot before spin is permitted. r_eff has a fallback of 0.60 m so a placeholder r_robot=0.0 does not perma-reject rotation. Three rejection detail.kind values map to the three sub-rules for observability.
"""



from __future__ import annotations


# RCG-1: minimum clearance ring around robot for spin permission.
# r_eff = max(r_robot, r_eff_fallback). r_robot may be null placeholder
# (0.0) if V-03 not measured; fallback 0.60 m ensures the check STAYS ACTIVE.
R_EFF_FALLBACK_M = 0.60


def is_spin_like(vx_mps: float, wz_radps: float,
                 spin_vx_epsilon: float = 0.05) -> bool:
    """12 S6A: spin_like = wz != 0 AND |vx| < epsilon.
    If vx > epsilon it's a path_follow style turn, NOT a spin."""
    return abs(wz_radps) > 0.0 and abs(vx_mps) < spin_vx_epsilon


def rotation_permitted(clearance_m: float,
                        r_robot: float,
                        r_eff_fallback: float = R_EFF_FALLBACK_M) -> bool:
    """RCG-1: spin permitted iff clearance around robot >= r_eff.
    r_eff = max(r_robot, r_eff_fallback) -- fallback prevents perma-
    reject on placeholder r_robot=0.0."""
    r_eff = max(r_robot, r_eff_fallback)
    return clearance_m >= r_eff


class RotationErrorKind:
    """Three detail.kind values for rotation-permit rejection audit."""
    RC1_INSUFFICIENT_CLEARANCE = "rc1_insufficient_clearance"
    RC2_UNAVAILABLE_GRID = "rc2_unavailable_grid"
    RC3_FENCE_TOO_CLOSE = "rc3_fence_too_close"
