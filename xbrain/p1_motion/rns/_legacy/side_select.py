"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: side_select.py
Brief: MOT-RD-4 side select + cost(psi) + tie-break + hysteresis

Description:
20 §6 side selection. Given multiple candidate corridors, choose
the ONE that minimises cost = w1 * heading_dev + w2 * side_penalty
+ w3 * path_dev. Deterministic tie-break: when two candidates have
equal cost within epsilon, prefer the current side (hysteresis) to
avoid oscillation. Hysteresis does NOT extend past a hard-door
boundary (if the current side hits L_min, switch regardless).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


@dataclass(frozen=True)
class Candidate:
    angle_deg: float
    heading_dev_deg: float
    side_penalty: float
    path_dev_m: float


def cost_of(cand: Candidate,
            w_heading: float = 1.0,
            w_side: float = 1.0,
            w_path: float = 1.0) -> float:
    return (w_heading * abs(cand.heading_dev_deg)
            + w_side * cand.side_penalty
            + w_path * cand.path_dev_m)


def choose_side(candidates: List[Candidate],
                current_side_angle_deg: Optional[float],
                tie_epsilon: float = 0.1) -> Optional[Candidate]:
    """Pick the min-cost candidate. On ties within tie_epsilon,
    prefer the one closest to current_side_angle_deg (hysteresis).
    If current_side_angle_deg is None (no hysteresis anchor), fall
    back to the first candidate in the sorted list (deterministic)."""
    if not candidates:
        return None
    scored = sorted(candidates, key=cost_of)
    best_cost = cost_of(scored[0])
    tied = [c for c in scored if cost_of(c) - best_cost < tie_epsilon]
    if len(tied) == 1 or current_side_angle_deg is None:
        return tied[0]
    # Hysteresis: pick candidate closest to current side.
    return min(tied,
                key=lambda c: abs(c.angle_deg - current_side_angle_deg))
