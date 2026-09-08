"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: candidate_gen.py
Brief: MOT-RD-7 candidate construction + wz geometric clamp (RNS-ROT-1..4)

Description:
20 §9 candidate construction: given the selected corridor and
robot state, propose (vx, wz). wz is GEOMETRICALLY CLAMPED to
prevent skidding: wz_max = vx / turning_radius, where
turning_radius is derived from r_eff (RNS-ROT-4).

r_eff is the SINGLE authoritative source for the robot's
effective turning radius; computing it in multiple places would
let one caller use r_robot (which may be a null placeholder 0.0)
and quietly skid.
"""

from __future__ import annotations


def r_eff(r_robot: float, r_eff_fallback: float = 0.60) -> float:
    """RNS-ROT-4: single authoritative r_eff computation.
    max(r_robot, r_eff_fallback). Callers must NEVER inline this
    formula; use this function so a change to the fallback
    propagates everywhere consistently."""
    return max(r_robot, r_eff_fallback)


def wz_clamp_geometric(vx_mps: float,
                        wz_requested_radps: float,
                        r_robot: float,
                        r_eff_fallback: float = 0.60) -> float:
    """RNS-ROT-1/2/3: wz_max = vx / r_eff. A pure-spin (vx=0) has
    no geometric wz limit here (rotation permit is checked separately
    via rotation/rcg.py). Sign is preserved."""
    if vx_mps == 0:
        return wz_requested_radps      # rotation permit gates this elsewhere
    r = r_eff(r_robot, r_eff_fallback)
    wz_max = abs(vx_mps) / r
    if wz_requested_radps > wz_max:
        return wz_max
    if wz_requested_radps < -wz_max:
        return -wz_max
    return wz_requested_radps
