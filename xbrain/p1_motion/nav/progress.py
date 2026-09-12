"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: progress.py
Brief: state/motion/path_progress (11 S3.5B) body builder from the live route mission

Description:
P1-12: p3_task's only progress input (11 S3.5B verbatim "P3 手里根本没有这两个
数"); 2 Hz plus a re-send on every terminal / state edge. This builder makes
the body from what the host knows -- the RouteSet in flight, the host's state
word and the polyline projection -- and never guesses a field:

  waypoint_index  last passed vertex, 0-based; -1 before the first (11 S3.5B).
                  For a multi-point route the projection's seg_index i means
                  "on P[i]..P[i+1]", so P[i] is passed -> index i. A single
                  point route (goto) has no passed vertex until arrival, so -1
                  while running and 0 on arrived.
  seg_done_m      t * |P[i]P[i+1]| from the projection; 0 for a goto.
  dist_done_m     the projection's arc-length progress s (RNS-N-4 monotone
                  window projection); on arrived it is the route length.
  loop_index / loop_total / dir_sign  single pass this phase: 0 / 1 / +1
                  (loop_mode is carried by the RouteSet but RNS runs the
                  polyline once; NEXT.md holds the loop item).
  state           closed set idle | route_loading | running | arrived |
                  aborted | failed; fail_reason REQUIRED iff failed (the 20
                  S9.0.2 reason verbatim, no re-mapping), None otherwise --
                  both directions are asserted here, not left to the caller.
  loading         True only for route_loading (RG-4).
  ts / mono       wall seconds (align/log) and monotonic seconds (B5: the
                  consumer times out on mono).

Why the projection is passed in and not computed here: route.Mission owns the
monotone tracker index (RNS-N-4); the caller projects through the mission's
public tracker at the publish cadence and hands the result over, so this file
depends on nothing of RNS beyond the Projection shape.

What it does NOT do: no publishing, no cadence (the wiring keeps the 2 Hz timer
and the edge trigger), no pct (that is TaskState.progress on the p3 side,
11 S3.5B "字段分工").
"""
from __future__ import annotations

import math
from typing import Any, Dict, Optional

from xbrain.p1_motion.nav.route_intake import RouteSet
from xbrain.p1_motion.rns.route import Projection

#: 11 S3.5B state closed set, verbatim order.
PROGRESS_STATES = ("idle", "route_loading", "running", "arrived", "aborted", "failed")


class ProgressError(ValueError):
    """A body that would violate 11 S3.5B (state off-set, fail_reason mismatch)."""


def _route_len_m(route: RouteSet) -> float:
    pts = route.points_xy
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def build_path_progress(*, route: Optional[RouteSet], state: str,
                        fail_reason: Optional[str], proj: Optional[Projection],
                        now_mono_s: float, ts_wall_s: float) -> Dict[str, Any]:
    """The 11 S3.5B body. Raises ProgressError rather than emit an off-contract
    frame (CLAUDE.md 3.5).
    mutant: allow state == "failed" with fail_reason None -> p3 cannot tell
    a navigation failure from an abort -> test_failed_requires_reason red."""
    if state not in PROGRESS_STATES:
        raise ProgressError("state %r not in %s" % (state, list(PROGRESS_STATES)))
    if state == "failed":
        if not isinstance(fail_reason, str) or not fail_reason:
            raise ProgressError("state failed requires fail_reason (20 S9.0.2 reason)")
    elif fail_reason is not None:
        raise ProgressError("fail_reason only allowed with state failed")
    waypoint_index = -1
    seg_done = 0.0
    dist_done = 0.0
    total = 0
    if route is not None:
        total = route.waypoint_total
        if state == "arrived":
            waypoint_index = total - 1
            dist_done = _route_len_m(route)
        elif proj is not None and total >= 2:
            i = min(proj.seg_index, total - 2)
            a, b = route.points_xy[i], route.points_xy[i + 1]
            seg_done = proj.t * math.hypot(b[0] - a[0], b[1] - a[1])
            dist_done = proj.s_arc_m
            waypoint_index = i
    return {
        "v": 1,
        "route_id": route.route_id if route is not None else None,
        "route_rev": route.route_rev if route is not None else None,
        "loading": state == "route_loading",
        "waypoint_index": waypoint_index,
        "waypoint_total": total,
        "seg_done_m": round(seg_done, 3),
        "dist_done_m": round(dist_done, 3),
        "fail_reason": fail_reason,
        "loop_index": 0,
        "loop_total": 1,
        "dir_sign": 1,
        "state": state,
        "ts": ts_wall_s,          # WALL-CLOCK-OK(align/log)
        "mono": now_mono_s,
    }
