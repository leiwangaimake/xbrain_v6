"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: audit.py
Brief: MOT-PM-9 gate audit output (limiter argmax + list)

Description:
Every 20 Hz tick, the gate produces `limiter` (the single constraint that clipped this tick) and `limiter_all` (the near-tied constraints). HL-4: heading and rtk stay separate entries in limiter_all; they never collapse into a single 'health' bucket. The closed 14-value limiter set is the tie-break for argmax.
"""



from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional


# 12 S6.7 14-value limiter enum (verbatim).
_LIMITER_VALUES = (
    "none", "f_speed", "g_targets", "h_heading", "i_rtk",
    "hard_upper", "fence_soft", "fence_hard", "rotation_permit",
    "profile", "estop", "hes", "cmd_timeout", "source_deactivated",
)


def limiter_argmax(candidates: Dict[str, float]) -> str:
    """5-step argmax: pick the smallest-value limiter (i.e. the one
    that actually clipped). Ties broken by closed-set order."""
    valid = {k: v for k, v in candidates.items() if k in _LIMITER_VALUES}
    if not valid:
        return "none"
    min_v = min(valid.values())
    for k in _LIMITER_VALUES:   # closed-set order tie-break
        if k in valid and valid[k] == min_v:
            return k
    return "none"


def limiter_all(candidates: Dict[str, float],
                threshold_delta: float = 0.05) -> List[str]:
    """limiter_all: list of limiters that clipped within threshold_delta
    of the min. HL-4: heading and rtk are ALWAYS separate entries
    (never merged into 'health')."""
    valid = {k: v for k, v in candidates.items() if k in _LIMITER_VALUES}
    if not valid:
        return []
    min_v = min(valid.values())
    return [k for k in _LIMITER_VALUES
            if k in valid and valid[k] <= min_v + threshold_delta]
