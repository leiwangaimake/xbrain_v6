"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: teach.py
Brief: BIZ-P3-26 teach recording (1 Hz + 0.5m dedup, memory -> geo commit)

Description:
15 S8.9 teach: while a 'teach' task is active, p3 samples current
pose at 1 Hz. To avoid recording a dense cloud of near-duplicate
points at slow speed, a sample is only kept if it is at least
0.5m from the previous KEPT sample (dedup_min_dist_m from configs;
no default in code).

The buffer is 'memory' table (fast local kv). On task complete, the
buffer is atomically committed to the geo tables (waypoints + a
new route referencing them), then cleared.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List


@dataclass(frozen=True)
class TeachSample:
    x_m: float
    y_m: float
    heading_rad: float


def should_keep(new_sample: TeachSample,
                  last_kept: TeachSample | None,
                  dedup_min_dist_m: float) -> bool:
    """Keep the first sample; keep subsequent samples only if the
    XY distance from last kept exceeds the threshold."""
    if last_kept is None:
        return True
    d = math.hypot(new_sample.x_m - last_kept.x_m,
                    new_sample.y_m - last_kept.y_m)
    return d >= dedup_min_dist_m


def dedup_run(samples, dedup_min_dist_m: float) -> List[TeachSample]:
    """Reduce a dense stream to a sparse route. Preserves first
    and last sample regardless of density."""
    out: List[TeachSample] = []
    last = None
    for s in samples:
        if should_keep(s, last, dedup_min_dist_m):
            out.append(s)
            last = s
    if samples and (not out or out[-1] is not samples[-1]):
        out.append(samples[-1])
    return out
