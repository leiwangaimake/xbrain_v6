"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: corridor.py
Brief: MOT-RD-3 corridor search + L_min hard-door + unknown = blocked

Description:
20 §5 forward half-plane corridor search. Cast rays in a fan
around the heading direction; find the widest passable corridor.
Each ray reports the distance to the first blocked cell. Unknown
cells count as BLOCKED (safety direction: assume the worst until
proven clear).

L_min is a hard gate: if the passable-corridor length is below
L_min, RNS refuses to propose motion (returns 'blocked'). This
prevents 'squeeze through a gap that would trap the robot'.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


@dataclass(frozen=True)
class CorridorSample:
    """One ray's distance to the first blocked cell in that direction."""
    angle_deg: float
    distance_m: float
    unknown_cells_count: int


def is_blocked(sample: CorridorSample) -> bool:
    """Ray is 'blocked' when its distance to the first non-free cell.
    Unknown cells count as blocked, so a ray with unknown_cells_count > 0
    at close range is blocked."""
    if sample.unknown_cells_count > 0:
        return True
    return False


def find_widest_corridor(
    samples: List[CorridorSample],
    L_min_m: float,
) -> Optional[tuple]:
    """Return (start_angle, end_angle, min_distance) of the widest
    consecutive run of un-blocked samples whose min_distance >= L_min.
    None if no run meets L_min."""
    best: Optional[tuple] = None
    best_width = 0.0
    i = 0
    while i < len(samples):
        if is_blocked(samples[i]) or samples[i].distance_m < L_min_m:
            i += 1
            continue
        # Extend run.
        j = i
        min_d = samples[i].distance_m
        while (j < len(samples) - 1
               and not is_blocked(samples[j + 1])
               and samples[j + 1].distance_m >= L_min_m):
            j += 1
            if samples[j].distance_m < min_d:
                min_d = samples[j].distance_m
        width = samples[j].angle_deg - samples[i].angle_deg
        if width > best_width:
            best_width = width
            best = (samples[i].angle_deg, samples[j].angle_deg, min_d)
        i = j + 1
    return best
