"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: g_targets.py
Brief: MOT-PM-8 g(targets) semantic slowdown

Description:
When perception reports non-empty targets, the semantic channel clips speed to obstacle_avoid.max_mps (0.5) regardless of what f(d_free) would allow. Empty targets => 1.0 multiplier (no semantic clip). g() is a dimensionless factor; CON-07 forbids the caller from confusing it with a m/s value even though the numeric value in the non-empty branch happens to look like one.
"""



from __future__ import annotations


# 12 S6.4 verbatim: targets nonempty -> g = obstacle_avoid.max_mps (0.5)
# targets empty -> g = 1.0 (no semantic slowdown)
def g_targets(targets_present: bool,
              obstacle_avoid_max_mps: float = 0.5) -> float:
    """g() is a MULTIPLICATIVE FACTOR (dimensionless). The result is
    used as `g` in the gate rule; it is NOT a m/s value even though
    it's numerically the same in the 'targets_present' branch."""
    if targets_present:
        return obstacle_avoid_max_mps
    return 1.0
