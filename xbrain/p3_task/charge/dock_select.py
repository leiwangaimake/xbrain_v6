"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: dock_select.py
Brief: BIZ-P3-16 three-tier dock selection (energy reach -> route -> cost)

Description:
15 §8.4 three-tier funnel:

  Tier 1  ENERGY-REACHABILITY FILTER
          keep docks the robot can reach on current SoC with margin
          (per V-3 style computation but with a slightly larger
          reserve dedicated to dock approach)
  Tier 2  ROUTE FILTER
          if a route is being followed, prefer docks close to the
          current route (skip docks that would require doubling back)
  Tier 3  COST FUNCTION
          cost = w1 * d_to_handover + w2 * skip_len
          d_to_handover = straight-line distance to the dock's
                          handover pose
          skip_len      = arc length skipped from the current route
          Smaller cost wins.

An empty candidate list at any tier is a decision result, not a
failure: the caller escalates to §8.2A (stop-in-place) if reach
returns empty.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Dock:
    dock_id: str
    handover_x: float
    handover_y: float


@dataclass(frozen=True)
class DockCandidate:
    dock: Dock
    d_to_handover_m: float
    skip_len_m: float


def energy_reach_filter(docks,
                         robot_x: float, robot_y: float,
                         soc_pct: float,
                         energy_per_meter_pct: float,
                         dock_reserve_pct: float):
    """Tier 1: retain only docks whose reach cost is under budget.
    A dock at distance d costs d * energy_per_meter_pct + reserve;
    keep only those with strict inequality (<) so the reserve stays
    untouched (matches V-3)."""
    kept = []
    for d in docks:
        dist = math.hypot(d.handover_x - robot_x, d.handover_y - robot_y)
        need = dist * energy_per_meter_pct + dock_reserve_pct
        if soc_pct > need:
            kept.append(DockCandidate(dock=d, d_to_handover_m=dist,
                                        skip_len_m=0.0))
    return kept


def route_filter(candidates, current_route_wps):
    """Tier 2: compute skip_len (arc length the robot skips by
    diverting to this dock at the current position). If no route is
    active, all candidates keep skip_len=0."""
    if not current_route_wps:
        return list(candidates)
    total_route_len = _cumulative_len(current_route_wps)[-1]
    out = []
    for c in candidates:
        skip = _skip_from_dock(c.dock, current_route_wps, total_route_len)
        out.append(DockCandidate(dock=c.dock,
                                   d_to_handover_m=c.d_to_handover_m,
                                   skip_len_m=skip))
    return out


def _cumulative_len(wps):
    s = [0.0]
    for i in range(1, len(wps)):
        s.append(s[-1] + math.hypot(
            wps[i][0] - wps[i - 1][0], wps[i][1] - wps[i - 1][1]))
    return s


def _skip_from_dock(dock, wps, total_len) -> float:
    """Estimate arc length skipped: project handover onto the route,
    return remaining arc from that projection to the end."""
    best_s = 0.0
    best_d = float("inf")
    s = _cumulative_len(wps)
    for i in range(len(wps) - 1):
        d = math.hypot(dock.handover_x - wps[i][0],
                        dock.handover_y - wps[i][1])
        if d < best_d:
            best_d = d
            best_s = s[i]
    return max(0.0, total_len - best_s)


def cost_select(candidates, w1: float, w2: float):
    """Tier 3: cost = w1*d_to_handover + w2*skip_len; pick min.
    Returns None on empty."""
    if not candidates:
        return None
    return min(candidates, key=lambda c: w1 * c.d_to_handover_m
                                            + w2 * c.skip_len_m)
