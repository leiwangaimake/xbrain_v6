"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: basic_motion_test.py
Brief: TEMPORARY basic-motion test executor: relmove closed-loop on chassis odom, no RNS / gates

Description:
User ruling 2026-10-26: for the basic-motion test, commx commands go through
the standard processes (p4 -> p2 -> p1 -> quadruped) and NOTHING may block
the six basic motions; restore the standard flow after the test. Enabled only
by XBRAIN_BASIC_MOTION_TEST=1, which scripts/start_basic_motion.sh sets.

With it, nav_wiring hands each cmd/motion/relative_move to this executor
instead of the mission host / RNS: one axis at a time (dx_m, dy_m or
dyaw_rad), closed-loop on quadruped's integrated odom (rt/chassis/state
odom x / y / yaw_rad, indoor ruling: no RTK needed). Stops on: target reached,
soft estop, stale state / invalid odom, timeout, stall, wrong direction,
over-speed, off-axis drift (same guard as scripts/diag/chassis_bench.py).
Speeds come from XBRAIN_BASIC_MOTION_LIN / _LAT / _ANG (the start script).
"""
from __future__ import annotations

import logging
import math
import os
from typing import Any, Dict, List, Optional, Tuple

_logger = logging.getLogger("xbrain.p1.basic_motion_test")

TICK_S = 0.05
RAMP_S = 0.5
TAPER_S = 0.6
TAPER_MIN = 0.3
DONE_TOL_M = 0.02
DONE_TOL_RAD = math.radians(1.5)
STALL_AFTER_S = 2.0
STALL_WIN_S = 1.5
STALL_FRAC = 0.25
WRONG_DIR_M = 0.10
WRONG_DIR_RAD = math.radians(10.0)
OFFAXIS_M = 0.30
SPEED_WIN_S = 0.5
OVERSPEED_K = 1.5
OVERSPEED_LIN = 0.10
OVERSPEED_ANG = 0.10
OVERSPEED_TICKS = 5
STATE_STALE_S = 0.5
MAX_DIST_M = 3.0
MAX_ANGLE_RAD = math.pi


def enabled() -> bool:
    """The test switch (set only by scripts/start_basic_motion.sh)."""
    return os.environ.get("XBRAIN_BASIC_MOTION_TEST") == "1"


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class _Guard:
    """Per-tick decision for one motion (see chassis_bench.MotionGuard)."""

    def __init__(self, axis: str, sign: int, amount: float, speed: float) -> None:
        self.axis, self.sign, self.amount, self.speed = axis, sign, amount, speed
        rot = axis == "yaw"
        self.tol = DONE_TOL_RAD if rot else DONE_TOL_M
        self.wrong = WRONG_DIR_RAD if rot else WRONG_DIR_M
        self.margin = OVERSPEED_ANG if rot else OVERSPEED_LIN
        self.timeout_s = amount / speed * 2.0 + 3.0
        self.t0: Optional[float] = None
        self.hist: List[Tuple[float, float, float]] = []
        self.over_n = 0
        self.last_progress = 0.0

    def _window(self, t: float, span: float):
        old = [h for h in self.hist if h[0] <= t - span]
        if not old:
            return None
        t_a, p_a, _c = old[-1]
        cmd_int = sum(c for tt, _p, c in self.hist if tt > t_a) * TICK_S
        return t - t_a, self.last_progress - p_a, cmd_int

    def step(self, t: float, progress: float, off_axis: float) -> Tuple[float, Optional[str]]:
        if self.t0 is None:
            self.t0 = t
        el = t - self.t0
        self.last_progress = progress
        if progress < -self.wrong:
            return 0.0, "wrong_direction"
        if off_axis > OFFAXIS_M:
            return 0.0, "off_axis_drift"
        if el > self.timeout_s:
            return 0.0, "timeout"
        w = self._window(t, SPEED_WIN_S)
        if w is not None and w[0] > 0.0:
            recent = self.hist[-int(round(SPEED_WIN_S / TICK_S)):]
            cmd_max = max(c for _t, _p, c in recent)
            self.over_n = self.over_n + 1 if w[1] / w[0] > OVERSPEED_K * cmd_max + self.margin else 0
            if self.over_n >= OVERSPEED_TICKS:
                return 0.0, "over_speed"
        if el >= STALL_AFTER_S:
            s = self._window(t, STALL_WIN_S)
            if s is not None and s[2] > 0.0 and s[1] < STALL_FRAC * s[2]:
                return 0.0, "stall"
        rest = self.amount - progress
        if rest <= self.tol:
            return 0.0, "done"
        scalar = self.speed * min(1.0, el / RAMP_S)
        scalar = min(scalar, self.speed * max(TAPER_MIN, min(1.0, rest / (self.speed * TAPER_S))))
        self.hist.append((t, progress, scalar))
        return scalar, None


def _pose(state: Dict[str, Any]) -> Optional[Tuple[float, float, float]]:
    o = state.get("odom")
    if not isinstance(o, dict) or o.get("valid") is not True:
        return None
    try:
        return float(o["x"]), float(o["y"]), float(o["yaw_rad"])
    except (KeyError, TypeError, ValueError):
        return None


class BasicMotionTest:
    """Holds at most one motion; nav_wiring calls start() and step()."""

    def __init__(self) -> None:
        self.speeds = {"x": float(os.environ["XBRAIN_BASIC_MOTION_LIN"]),
                       "y": float(os.environ["XBRAIN_BASIC_MOTION_LAT"]),
                       "yaw": float(os.environ["XBRAIN_BASIC_MOTION_ANG"])}
        self._g: Optional[_Guard] = None
        self._start: Optional[Tuple[float, float, float]] = None
        self._yaw_raw = 0.0
        self._yaw_total = 0.0
        self.cmd_id: Optional[str] = None
        self.last_result: Optional[str] = None

    @property
    def active(self) -> bool:
        return self._g is not None

    def start(self, body: Any, state: Dict[str, Any], state_age_s: float) -> Optional[str]:
        """Begin a relmove; returns None or the reason it was refused."""
        if not isinstance(body, dict):
            return "bad_body"
        if body.get("target_yaw_rad") is not None:
            return "face_heading_unsupported_in_test"
        try:
            dx, dy, dyaw = (float(body.get(k) or 0.0) for k in ("dx_m", "dy_m", "dyaw_rad"))
        except (TypeError, ValueError):
            return "bad_body"
        pose = _pose(state)
        if pose is None or state_age_s > STATE_STALE_S:
            return "no_odom"
        if abs(dyaw) > 1e-6:
            axis, val, cap = "yaw", dyaw, MAX_ANGLE_RAD
        elif abs(dy) > 1e-6:
            axis, val, cap = "y", dy, MAX_DIST_M
        elif abs(dx) > 1e-6:
            axis, val, cap = "x", dx, MAX_DIST_M
        else:
            return "zero_move"
        amount = min(abs(val), cap)
        self._g = _Guard(axis, 1 if val > 0 else -1, amount, self.speeds[axis])
        self._start = pose
        self._yaw_raw = self._yaw_total = pose[2]
        self.cmd_id = body.get("cmd_id")
        _logger.warning("BASIC-MOTION-TEST start cmd_id=%s axis=%s amount=%.3f speed=%.2f",
                        self.cmd_id, axis, amount * (1 if val > 0 else -1), self.speeds[axis])
        return None

    def abort(self, why: str) -> None:
        if self._g is not None:
            _logger.warning("BASIC-MOTION-TEST end cmd_id=%s ended=%s progress=%.3f",
                            self.cmd_id, why, self._g.last_progress)
            self.last_result = why
        self._g = None

    def step(self, now_s: float, state: Dict[str, Any], state_age_s: float) -> Tuple[float, float, float]:
        """(vx, vy, wz) for this tick; zero and ended when the guard stops."""
        g = self._g
        if g is None:
            return 0.0, 0.0, 0.0
        pose = _pose(state)
        if pose is None or state_age_s > STATE_STALE_S:
            self.abort("state_stale_or_odom_invalid")
            return 0.0, 0.0, 0.0
        self._yaw_total += wrap_pi(pose[2] - self._yaw_raw)
        self._yaw_raw = pose[2]
        sx, sy, syaw = self._start
        dx, dy = pose[0] - sx, pose[1] - sy
        c, s = math.cos(syaw), math.sin(syaw)
        ax, ay = dx * c + dy * s, -dx * s + dy * c
        if g.axis == "x":
            prog, off = g.sign * ax, abs(ay)
        elif g.axis == "y":
            prog, off = g.sign * ay, abs(ax)
        else:
            prog, off = g.sign * (self._yaw_total - syaw), math.hypot(dx, dy)
        scalar, ended = g.step(now_s, prog, off)
        if ended is not None:
            self.abort(ended)
            return 0.0, 0.0, 0.0
        v = g.sign * scalar
        if g.axis == "x":
            return v, 0.0, 0.0
        if g.axis == "y":
            return 0.0, v, 0.0
        return 0.0, 0.0, v
