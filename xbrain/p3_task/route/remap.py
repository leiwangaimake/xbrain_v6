"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: remap.py
Brief: BIZ-P3-13 breakpoint remap algorithm §7.3A (5-step arc-length normalise)

Description:
When the route body changes while a task is suspended, resume
cannot use the old waypoint index directly (waypoints have shifted
in and out). §7.3A defines a 5-step arc-length remap:

  1. Compute cumulative arc length s_old[i] over OLD route
  2. Compute cumulative arc length s_new[j] over NEW route
  3. Let s_resume = s_old[wp_ix] + within_segment * segment_length
  4. Binary-search s_new for the smallest j with s_new[j] >= s_resume
  5. Reject if abs(s_new[j] - s_resume) > remap_tol (§7.3A B-2);
     the two routes are too different to safely resume

The tolerance remap_tol comes from configs (no default in code, per
CLAUDE.md §3.1). Rejection surfaces to the operator; auto-restart
is NOT chosen for them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


class RemapTooFar(Exception):
    """Arc-length delta between old and new route exceeds remap_tol."""


@dataclass(frozen=True)
class RemapResult:
    new_wp_ix: int
    new_within_segment: float
    arc_length_delta_m: float


def _seg_len(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def cumulative_arc_lengths(waypoints):
    """s[i] = arc length from start to waypoint i (in meters).
    len(s) == len(waypoints); s[0] = 0."""
    s = [0.0]
    for i in range(1, len(waypoints)):
        s.append(s[-1] + _seg_len(waypoints[i - 1], waypoints[i]))
    return s


def remap(old_waypoints,
           new_waypoints,
           wp_ix: int,
           within_segment: float,
           remap_tol_m: float) -> RemapResult:
    """Compute (new_wp_ix, new_within_segment) so that the arc
    length from new_waypoints[0] matches the arc length the robot
    covered on the OLD route.

    Within [0, s_new[-1]] the interpolation is exact so delta is 0.
    If s_resume falls PAST the end of the new route, delta =
    s_resume - s_new[-1] and we reject when it exceeds remap_tol_m."""
    s_old = cumulative_arc_lengths(old_waypoints)
    s_new = cumulative_arc_lengths(new_waypoints)
    if wp_ix + 1 >= len(s_old):
        seg_len_old = 0.0
    else:
        seg_len_old = s_old[wp_ix + 1] - s_old[wp_ix]
    s_resume = s_old[wp_ix] + within_segment * seg_len_old
    total_new = s_new[-1]
    if s_resume > total_new:
        delta = s_resume - total_new
        if delta > remap_tol_m:
            raise RemapTooFar(
                f"remap delta={delta:.3f}m exceeds remap_tol="
                f"{remap_tol_m:.3f}m at s_resume={s_resume:.3f}m")
        # Land exactly on the last waypoint.
        return RemapResult(
            new_wp_ix=len(new_waypoints) - 1, new_within_segment=0.0,
            arc_length_delta_m=delta)
    # Find smallest j with s_new[j] >= s_resume.
    lo, hi = 0, len(s_new) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if s_new[mid] >= s_resume:
            hi = mid
        else:
            lo = mid + 1
    j = lo
    if j == 0:
        return RemapResult(new_wp_ix=0, new_within_segment=0.0,
                            arc_length_delta_m=0.0)
    seg_len_new = s_new[j] - s_new[j - 1]
    new_within = 0.0 if seg_len_new == 0 else (
        (s_resume - s_new[j - 1]) / seg_len_new)
    new_within = max(0.0, min(1.0, new_within))
    return RemapResult(
        new_wp_ix=j - 1, new_within_segment=new_within,
        arc_length_delta_m=0.0)
