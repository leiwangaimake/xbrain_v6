"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: mission_host.py
Brief: route / relative_move mission lifecycle around rns_avoid and report routing by origin

Description:
The command shell around the single navigation source (12 S4.2c): the two
entries in, the terminal reports out, one channel per origin (12 S4.2c.6).
Pure -- no Zenoh, no clock, no thread. The wiring feeds it the parsed intake
results and every tick's (NavInputs, NavOutput); it returns Emit records the
wiring serialises and publishes. That split is what makes the shell's rules
mutant-testable against the real RnsSource.

Route (origin route, P1-11 in / P1-12 out):
  * op=set  -> rns.load_route; path_progress state running (a relmove in
              flight is aborted preempted, 12 S4.5.2 "抢占旧的")
  * op=clear-> rns.cancel; state idle; NO failure report (12 S4.2c.4)
  * arrival -> state arrived (edge re-send, 11 S3.5B "每到点补发")
  * failure -> state failed + fail_reason = 20 S9.0.2 reason verbatim + an
              event/{sev}/motion
  * estop / teleop (out.suspended) -> state aborted while suspended (11 S3.5B
              "aborted 保留原义(抢占/estop)"), back to running on release --
              the mission itself is KEPT (20 RNS-M-7)
  * 2 Hz periodic body always (p3 times the stream out at 3 s)

relative_move (origin relmove, P1-5 in / P1-6 out), the 12 S4.5.1 shell:
  * translate (relmove_intake) -> rejected {code, detail} or accepted, then
    running at 2 Hz with progress = the ACTUAL body-frame displacement (RM-2)
  * arrival -> succeeded (progress = actual, 12 S4.5.5)
  * RNS failure -> aborted + 12 S4.2c.6 abort_reason / detail.item + event
  * soft estop -> aborted soft_estop (12 S4.5.1 v0.4 branch; the mission is
    cancelled, unlike a route)
  * teleop active -> aborted preempted (RM-4)
  * timeout_s elapsed -> aborted timeout (RM-3, no auto-retry)
  * abort_on_obstacle and RNS enters an avoidance state (thread / detour /
    wall_follow / wait_dynamic) -> aborted obstacle, detail.item = that state
    (12 S4.5.6 true row: stop and tell, do not detour)
  * a new relative_move or a route while one runs -> the old one aborted
    preempted

Why the terminal latches are read here and nowhere else: take_arrival /
take_failure are one-shot (20 W2); a second reader would see nothing and the
report would be lost.

What it does NOT do: no parsing of wire bodies for route (RouteAssembler does
that off-thread), no gate, no cmd_vel, no Nav2 spin (pure rotation is refused
at intake), no loops (single pass; NEXT.md).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple, Union

from xbrain.common.errors import E_TIMEOUT
from xbrain.p1_motion.nav.nav_tick import NavInputs, NavOutput
from xbrain.p1_motion.nav.progress import build_path_progress
from xbrain.p1_motion.nav.relmove_intake import (RelMoveGoal, RelMoveLimits,
                                                 RelMoveReject,
                                                 achieved_body_delta,
                                                 translate_relative_move)
from xbrain.p1_motion.nav.report_map import (SEV_WARN, event_severity,
                                             map_failure)
from xbrain.p1_motion.nav.route_intake import RouteClear, RouteSet
from xbrain.p1_motion.rns.types import NavFailure, NavState, Origin
from xbrain.p1_motion.sources.rns_avoid import RnsAvoidSource

#: 11 S3.5B / S9.3.2: 2 Hz periodic, plus edge re-sends.
PROGRESS_PERIOD_MS = 500
#: RNS states that mean "avoidance took over" for the abort_on_obstacle rule.
AVOIDANCE_STATES = frozenset({NavState.THREAD, NavState.DETOUR,
                              NavState.WALL_FOLLOW, NavState.WAIT_DYNAMIC})
CH_PROGRESS = "path_progress"
CH_RELMOVE = "relmove_status"
CH_EVENT = "event"


@dataclass(frozen=True)
class Emit:
    """One thing to publish: channel + body (+ severity for events)."""
    channel: str
    body: Dict[str, Any]
    severity: Optional[str] = None


class MissionHost:
    """The shell. One per process, driven from the 20 Hz thread."""

    def __init__(self, src: RnsAvoidSource, *, relmove_limits: RelMoveLimits,
                 holonomic: bool, now_mono_ms: int) -> None:
        self._src = src
        self._lim = relmove_limits
        self._holo = bool(holonomic)
        self._route_state = "idle"
        self._route_fail: Optional[str] = None
        self._goal: Optional[RelMoveGoal] = None
        self._goal_accepted_ms = 0
        self._last_periodic_ms = now_mono_ms
        self._last_pose: Optional[Tuple[float, float]] = None
        self._ts_wall_s = 0.0

    # ---- observability -------------------------------------------------------
    @property
    def route_state(self) -> str:
        return self._route_state

    @property
    def goal(self) -> Optional[RelMoveGoal]:
        return self._goal

    # ---- bodies --------------------------------------------------------------
    def _progress(self, now: int, pose: Optional[Tuple[float, float]]) -> Emit:
        proj = (self._src.progress_projection(pose)
                if self._route_state == "running" else None)
        body = build_path_progress(
            route=self._src.route, state=self._route_state,
            fail_reason=self._route_fail, proj=proj,
            now_mono_s=now / 1000.0, ts_wall_s=self._ts_wall_s)
        return Emit(CH_PROGRESS, body)

    def _relmove_body(self, goal: RelMoveGoal, state: str, *,
                      pose: Optional[Tuple[float, float]] = None,
                      abort_reason: Optional[str] = None,
                      detail: Optional[Dict[str, Any]] = None,
                      code: Optional[str] = None) -> Emit:
        """11 S9.3.2 status body. progress is the ACTUAL displacement (RM-2)
        in the start body frame; dyaw_done is 0 (no rotation is ever accepted
        this phase, TY-4 keeps the relative form)."""
        if pose is not None:
            dx, dy = achieved_body_delta(goal, pose)
        else:
            dx, dy = 0.0, 0.0
        body: Dict[str, Any] = {
            "cmd_id": goal.cmd_id, "state": state,
            "progress": {"dx_done_m": round(dx, 3), "dy_done_m": round(dy, 3),
                         "dyaw_done_rad": 0.0},
            "abort_reason": abort_reason,
        }
        if detail is not None:
            body["detail"] = detail
        if code is not None:
            body["code"] = code
        return Emit(CH_RELMOVE, body)

    @staticmethod
    def _event(origin: Optional[Origin], failure: NavFailure) -> Emit:
        """event/{sev}/motion for a navigation failure (both origins)."""
        return Emit(CH_EVENT, {
            "title": "nav_failed",
            "dedup_key": "nav:failed:%s" % failure.reason.value,
            "detail": {"reason": failure.reason.value,
                       "origin": origin.value if origin else None,
                       "detail": dict(failure.detail or {})},
        }, event_severity(failure))

    # ---- relmove helpers -----------------------------------------------------
    def _end_relmove(self, state: str, pose, *, abort_reason=None,
                     detail=None, code=None) -> Emit:
        goal = self._goal
        assert goal is not None
        self._goal = None
        return self._relmove_body(goal, state, pose=pose,
                                  abort_reason=abort_reason, detail=detail, code=code)

    def _preempt_relmove(self, now: int, pose, why: str) -> List[Emit]:
        """A running relative_move loses to a newer command (12 S4.5.2)."""
        if self._goal is None:
            return []
        self._src.cancel(now)
        return [self._end_relmove("aborted", pose, abort_reason="preempted",
                                  detail={"item": why})]

    # ---- entries -------------------------------------------------------------
    def on_route(self, msg: Union[RouteSet, RouteClear], now: int,
                 pose: Optional[Tuple[float, float]]) -> List[Emit]:
        """cmd/motion/route (P1-11): set loads / supersedes, clear cancels.
        mutant: leave a running relative_move untouched on route set -> two
        missions "in flight" from the shell's view, the relmove never reports
        -> test_route_set_preempts_relmove red."""
        out = self._preempt_relmove(now, pose, "route")
        if isinstance(msg, RouteClear):
            self._src.cancel(now)
            self._route_state = "idle"
            self._route_fail = None
        else:
            self._src.load_route(msg, now)
            self._route_state = "running"
            self._route_fail = None
        out.append(self._progress(now, pose))
        self._last_periodic_ms = now
        return out

    def on_relmove(self, body: Any, *, now: int,
                   pose: Optional[Tuple[float, float]], yaw_rad: Optional[float],
                   heading_valid: bool, allow_motion: bool = True) -> List[Emit]:
        """cmd/motion/relative_move (P1-5): reject with a code, or accept and
        hand RNS the goto. A route in flight is superseded (state aborted:
        preempted by the displacement)."""
        try:
            goal = translate_relative_move(
                body, pose_xy=pose, yaw_rad=yaw_rad, heading_valid=heading_valid,
                holonomic=self._holo, limits=self._lim, allow_motion=allow_motion)
        except RelMoveReject as rej:
            cmd_id = body.get("cmd_id") if isinstance(body, dict) else None
            return [Emit(CH_RELMOVE, {
                "cmd_id": cmd_id if isinstance(cmd_id, str) else None,
                "state": "rejected",
                "progress": {"dx_done_m": 0.0, "dy_done_m": 0.0, "dyaw_done_rad": 0.0},
                "abort_reason": None, "code": rej.code, "detail": rej.detail})]
        out = self._preempt_relmove(now, pose, "relative_move")
        if self._route_state == "running":
            # the route mission is superseded by the goto (20 S9.0.3 audit);
            # 11 S3.5B has no "preempted" word: aborted is the preemption state.
            self._route_state = "aborted"
            out.append(self._progress(now, pose))
        self._src.load_goto(goal, now)
        self._goal = goal
        self._goal_accepted_ms = now
        out.append(self._relmove_body(goal, "accepted", pose=pose))
        self._last_periodic_ms = now
        return out

    # ---- per tick ------------------------------------------------------------
    def after_tick(self, inp: NavInputs, out: NavOutput) -> List[Emit]:
        """Run after NavTick.run for the same tick. Terminal latches first,
        then the relmove shell rules, then the route suspension edge, then
        the 2 Hz periodic bodies."""
        now = inp.now_mono_ms
        pose = inp.pose_xy
        self._ts_wall_s = inp.ts_wall_s
        emits: List[Emit] = []
        origin = self._src.origin
        if self._src.take_arrival():
            if origin is Origin.ROUTE:
                self._route_state = "arrived"
                self._route_fail = None
                emits.append(self._progress(now, pose))
            elif self._goal is not None:
                emits.append(self._end_relmove("succeeded", pose))
        failure = self._src.take_failure()
        if failure is not None:
            if origin is Origin.ROUTE:
                self._route_state = "failed"
                self._route_fail = failure.reason.value
                emits.append(self._progress(now, pose))
            elif self._goal is not None:
                reason, item, _sev = map_failure(failure)
                detail: Dict[str, Any] = dict(failure.detail or {})
                if item is not None:
                    detail["item"] = item
                emits.append(self._end_relmove("aborted", pose,
                                               abort_reason=reason, detail=detail))
            emits.append(self._event(origin, failure))
        # relmove shell rules (12 S4.5.1) -- only while its mission is in flight
        if self._goal is not None and self._src.mission_loaded():
            goal = self._goal
            state = self._src.nav_state()
            if inp.estop:
                self._src.cancel(now)
                emits.append(self._end_relmove("aborted", pose, abort_reason="soft_estop"))
            elif inp.teleop_active:
                self._src.cancel(now)
                emits.append(self._end_relmove("aborted", pose, abort_reason="preempted",
                                               detail={"item": "teleop"}))
            elif now - self._goal_accepted_ms >= int(goal.timeout_s * 1000.0):
                self._src.cancel(now)
                # RM-3: abort AND report E_TIMEOUT; no auto-retry.
                emits.append(self._end_relmove("aborted", pose, abort_reason="timeout",
                                               code=E_TIMEOUT))
            elif goal.abort_on_obstacle and state in AVOIDANCE_STATES:
                self._src.cancel(now)
                emits.append(self._end_relmove("aborted", pose, abort_reason="obstacle",
                                               detail={"item": state.value}))
        # route suspension edge (11 S3.5B aborted == estop / preempted; mission kept)
        if origin is Origin.ROUTE and self._src.route is not None:
            if out.suspended and self._route_state == "running":
                self._route_state = "aborted"
                emits.append(self._progress(now, pose))
            elif (not out.suspended and self._route_state == "aborted"
                  and self._src.mission_loaded()):
                self._route_state = "running"
                emits.append(self._progress(now, pose))
        # 2 Hz periodic
        if now - self._last_periodic_ms >= PROGRESS_PERIOD_MS:
            self._last_periodic_ms = now
            emits.append(self._progress(now, pose))
            if self._goal is not None:
                emits.append(self._relmove_body(self._goal, "running", pose=pose))
        self._last_pose = pose
        return emits
