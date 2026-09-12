"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: relmove_intake.py
Brief: cmd/motion/relative_move (11 S9.3.2) -> a goto goal in site metres, or a coded reject

Description:
12 S4.2c.3 second row: the voice / cloud displacement ("前进 3 米") keeps its
command shell (P1-5 / P1-6 status, timeout, preemption) but its EXECUTION is a
translation: P1 resolves the world endpoint G from the CURRENT pose and hands
RNS a goto mission with origin=relmove (12 S4.2c.1). #20-9 (12 v0.8) removed
the old odometry-closed-loop executor; this module is the translation step --
pure, pose in, goal out.

Pre-checks (12 S4.5.2, the rows this phase can evaluate) and their codes, all
imported from xbrain.common.errors (never literals, CLAUDE.md 3.5):
  * schema: cmd_id / dx_m / dy_m / dyaw_rad present and numeric -> E_SCHEMA
  * TY-1: target_yaw_rad != null AND dyaw_rad != 0 -> E_SCHEMA, detail.field
    "target_yaw_rad" (mutually exclusive, no implied precedence)
  * |dx_m| or |dy_m| > relative_move.max_distance_m, |dyaw_rad| >
    relative_move.max_yaw_rad -> E_SCHEMA with detail.field + detail.limit
  * dy_m != 0 and spec.holonomic == false -> E_CAPABILITY
  * allow_motion == false (the health factor) -> E_UNHEALTHY (12 S4.5.2 row 3)
  * no usable position (pose None) -> E_DEGRADED, detail.item "no_fix"
    (11 S3.2.1: no_fix / single / dgps forbid every autonomous motion)
  * heading_valid == false -> E_NO_HEADING. 12 S4.5.2 states it for pure
    rotation; a translation needs the heading for the same reason (G is the
    body-frame delta rotated into the site frame) and TY-3 forbids a "last
    known" heading, so no move is accepted without a valid heading.
  * pure rotation (|(dx, dy)| <= pure_rotation_eps_m with dyaw or target_yaw)
    -> E_CAPABILITY, detail.item "nav2_spin": 12 S4.5.3 delegates spin to Nav2
    via behavior_proxy, which is not wired this phase. Refused, never faked.
    A zero displacement WITHOUT rotation is accepted: the goto is the current
    pose and RNS arrives at once (an honest 'done', not a reject).
  * a translation that ALSO asks for a rotation (dyaw != 0 or target_yaw set)
    -> E_CAPABILITY, detail.item "heading_goal": the RNS Mission this phase
    takes points only (route.Mission has no goal-heading input); 18's voice
    table never produces a combined move, so nothing legitimate is lost.

Geometry: G = pose + R(yaw) * (dx, dy), with yaw the ENU heading (11 S3.3, east
0 CCW), so "左移 1 米" (dy = +1) lands on the robot's left in the site frame.

Timeout / abort_on_obstacle: taken from the message when present, else from
relative_move.default_timeout_s / relative_move.abort_on_obstacle of the
resolved config (12 S12) -- injected, not a literal here.

What it does NOT do: no status publishing, no execution, no accounting of the
achieved displacement (RM-2: report the ACTUAL amount; the wiring computes it
from pose deltas against start_xy / start_yaw_rad stored on the goal).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from xbrain.common.errors import (E_CAPABILITY, E_DEGRADED, E_NO_HEADING, E_SCHEMA,
                                  E_UNHEALTHY)


class RelMoveReject(Exception):
    """A refused relative_move: `code` is the 11 S13 code, `detail` the per-code
    required fields (field / limit for E_SCHEMA, item otherwise)."""

    def __init__(self, code: str, detail: Dict[str, Any]) -> None:
        super().__init__("%s %s" % (code, detail))
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class RelMoveLimits:
    """12 S12 relative_move.* values, injected (no defaults here)."""
    max_distance_m: float
    max_yaw_rad: float
    pure_rotation_eps_m: float
    default_timeout_s: float
    abort_on_obstacle: bool


@dataclass(frozen=True)
class RelMoveGoal:
    """An accepted displacement, resolved to a site-frame endpoint."""
    cmd_id: str
    dx_m: float
    dy_m: float
    dyaw_rad: float
    endpoint_xy: Tuple[float, float]
    start_xy: Tuple[float, float]
    start_yaw_rad: float
    timeout_s: float
    abort_on_obstacle: bool
    source: Optional[str]
    max_speed_mps: Optional[float]


def _num(body: Dict[str, Any], key: str) -> float:
    """Required finite number (bool excluded) or E_SCHEMA naming the field."""
    v = body.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise RelMoveReject(E_SCHEMA, {"field": key})
    return float(v)


def _opt_num(body: Dict[str, Any], key: str) -> Optional[float]:
    v = body.get(key)
    if v is None:
        return None
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise RelMoveReject(E_SCHEMA, {"field": key})
    return float(v)


def translate_relative_move(body: Any, *, pose_xy: Optional[Tuple[float, float]],
                            yaw_rad: Optional[float], heading_valid: bool,
                            holonomic: bool, limits: RelMoveLimits,
                            allow_motion: bool = True) -> RelMoveGoal:
    """Body -> RelMoveGoal, or RelMoveReject. Check order follows 12 S4.5.2's
    table top-down: shape, TY-1, range, capability, position, heading, then
    the two this-phase boundaries.
    mutant: build G from (dx, dy) without rotating by yaw -> "前进 1 米" at
    yaw = pi/2 lands east instead of north -> test_endpoint_rotates_by_yaw
    red."""
    if not isinstance(body, dict):
        raise RelMoveReject(E_SCHEMA, {"field": "body"})
    cmd_id = body.get("cmd_id")
    if not isinstance(cmd_id, str) or not cmd_id:
        raise RelMoveReject(E_SCHEMA, {"field": "cmd_id"})
    dx = _num(body, "dx_m")
    dy = _num(body, "dy_m")
    dyaw = _num(body, "dyaw_rad")
    target_yaw = _opt_num(body, "target_yaw_rad")
    if target_yaw is not None and dyaw != 0.0:
        raise RelMoveReject(E_SCHEMA, {"field": "target_yaw_rad"})     # TY-1
    for name, val, lim in (("dx_m", dx, limits.max_distance_m),
                           ("dy_m", dy, limits.max_distance_m),
                           ("dyaw_rad", dyaw, limits.max_yaw_rad)):
        if abs(val) > lim:
            raise RelMoveReject(E_SCHEMA, {"field": name, "limit": lim})
    if dy != 0.0 and not holonomic:
        raise RelMoveReject(E_CAPABILITY, {"item": "lateral"})
    if not allow_motion:
        raise RelMoveReject(E_UNHEALTHY, {"item": "allow_motion"})    # 12 S4.5.2 row 3
    if pose_xy is None:
        raise RelMoveReject(E_DEGRADED, {"item": "no_fix"})
    if not heading_valid or yaw_rad is None:
        raise RelMoveReject(E_NO_HEADING, {"item": "heading_invalid"})
    wants_rotation = dyaw != 0.0 or target_yaw is not None
    if wants_rotation:
        # below the displacement dead band it IS a pure rotation -> the Nav2
        # spin path (12 S4.5.3, not wired); above it, a combined move.
        item = ("nav2_spin" if math.hypot(dx, dy) <= limits.pure_rotation_eps_m
                else "heading_goal")
        raise RelMoveReject(E_CAPABILITY, {"item": item})
    # a zero displacement without rotation is a legal no-op: the goto lands on
    # the current pose and RNS reports arrival at once (12 S4.5.3 dead band).
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    gx = pose_xy[0] + dx * c - dy * s
    gy = pose_xy[1] + dx * s + dy * c
    timeout = _opt_num(body, "timeout_s")
    if timeout is None:
        timeout = float(limits.default_timeout_s)
    elif timeout <= 0.0:
        raise RelMoveReject(E_SCHEMA, {"field": "timeout_s"})
    aoo = body.get("abort_on_obstacle")
    if aoo is None:
        aoo = bool(limits.abort_on_obstacle)
    elif not isinstance(aoo, bool):
        raise RelMoveReject(E_SCHEMA, {"field": "abort_on_obstacle"})
    src = body.get("source")
    return RelMoveGoal(cmd_id=cmd_id, dx_m=dx, dy_m=dy, dyaw_rad=dyaw,
                       endpoint_xy=(gx, gy), start_xy=(float(pose_xy[0]), float(pose_xy[1])),
                       start_yaw_rad=float(yaw_rad), timeout_s=timeout,
                       abort_on_obstacle=aoo,
                       source=src if isinstance(src, str) else None,
                       max_speed_mps=_opt_num(body, "max_speed_mps"))


def achieved_body_delta(goal: RelMoveGoal, pose_xy: Tuple[float, float]) -> Tuple[float, float]:
    """RM-2: the displacement actually travelled, expressed in the START body
    frame (dx_done_m, dy_done_m), for relative_move/status.progress."""
    ex = pose_xy[0] - goal.start_xy[0]
    ey = pose_xy[1] - goal.start_xy[1]
    c, s = math.cos(goal.start_yaw_rad), math.sin(goal.start_yaw_rad)
    return (c * ex + s * ey, -s * ex + c * ey)
