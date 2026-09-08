"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: inflate.py
Brief: MOT-RD-2 dynamic inflation r_inflate + v_infl=max(|v_current|, v_req)

Description:
20 §4 dynamic inflation: obstacle-avoidance safety radius grows with
speed. r_inflate = r_robot + v_infl * t_lat, where v_infl is
max(|v_current|, v_req_by_source). The MAX is critical: if we used
only v_current, a source requesting acceleration would find the
world 'shrunk' toward the corridor edges as it accelerates.
"""

from __future__ import annotations


def compute_r_inflate(
    r_robot_m: float,
    v_current_mps: float,
    v_requested_mps: float,
    t_lat_s: float,
) -> float:
    """r_inflate = r_robot + max(|v_current|, v_requested) * t_lat.

    Using MAX prevents an acceleration request from finding a
    'shrunken' corridor -- the inflation grows to accommodate the
    intended new speed, not just the current speed."""
    v_infl = max(abs(v_current_mps), v_requested_mps)
    return r_robot_m + v_infl * t_lat_s
