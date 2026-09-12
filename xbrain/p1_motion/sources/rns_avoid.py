"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rns_avoid.py
Brief: behaviour source rns_avoid (900) -- the RnsSource adapter for the P1 arbiter and mission entries

Description:
12 S4.2c: RNS is the system's single navigation source, mounted in the P1
ladder as rns_avoid at 900 (arbiter_p1.BehaviorSource.RNS_AVOID). RnsSource
itself (rns/source.py) is the 20-册 module: is_active / compute / on_preempted
/ on_release with a Mission loaded through load_mission. This adapter is the
host-side face of it: it owns the mapping from the two P1 entries to Mission
(12 S4.2c.2 / S4.2c.3) and the cancel / supersede audit (20 S9.0.3), and it
exposes the one-shot arrival / failure latches unchanged.

Entries:
  load_route(RouteSet)   cmd/motion/route op=set (P1-11). One point -> GOTO
                         (RNS-N-1 degenerate polyline, start = current pose,
                         route.Mission B4 anchor); >= 2 points -> PATH. Origin
                         ROUTE, so terminal reports go to path_progress. Arrival
                         radius = the endpoint's arrive_radius_m from the wire
                         (11 S3.5A per-point; 20 A-RT-2 endpoint-only). A
                         mission already in flight is SUPERSEDED: audit entry,
                         no failure report (20 S9.0.3 / A-FAIL-3).
  load_goto(RelMoveGoal) cmd/motion/relative_move (P1-5): GOTO, origin RELMOVE,
                         so reports go to relative_move/status. Arrival radius =
                         rns.route.arrival_radius_m (configs/rns.yaml), as the
                         SIL goto does.
  cancel(now)            op=clear: audit "cancelled", clear_mission, IDLE; no
                         failure report (12 S4.2c.4).

Config: the RNS section (configs/rns.yaml "rns") is injected; search_window and
max_deviation_m come from it for every mission (as the SIL and the three
batteries do), so the follow geometry stays identical to what validated v2.0.

What it does NOT do: no ticking (nav/nav_tick.py), no gate, no report
serialisation (the wiring maps take_failure() by origin per 12 S4.2c.6). It
never reads RnsSource privates; the projection for path_progress is recomputed
by the wiring from the public mission tracker.

Trap: constructing a fresh RnsSource per mission. The memory grid, the
perception acceptance state (epoch / last-accepted) and the watchdog live in
the instance; ONE RnsSource for the process lifetime, exactly as hot_battery.py
proved (one source across eight legs).
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from xbrain.p1_motion.nav.relmove_intake import RelMoveGoal
from xbrain.p1_motion.nav.route_intake import RouteSet
from xbrain.p1_motion.rns.audit import AuditRecord, Outcome, on_route_superseded
from xbrain.p1_motion.rns.route import Mission
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rns.types import MissionKind, NavFailure, NavState, Origin
from xbrain.p1_motion.sources.arbiter_p1 import BehaviorSource, priority_of


class RnsAvoidSource:
    """The behaviour-source face (12 S4.1) over one RnsSource."""

    name = BehaviorSource.RNS_AVOID.value
    priority = priority_of(BehaviorSource.RNS_AVOID)

    def __init__(self, rns: RnsSource, rns_cfg: Dict[str, Any]) -> None:
        rc = rns_cfg["route"]
        self._rns = rns
        self._search_window = int(rc["search_window"])
        self._max_deviation_m = float(rc["max_deviation_m"])
        self._goto_arrival_radius_m = float(rc["arrival_radius_m"])
        self._route: Optional[RouteSet] = None
        self._goal: Optional[RelMoveGoal] = None
        self._origin: Optional[Origin] = None

    # ---- observability -------------------------------------------------------
    @property
    def rns(self) -> RnsSource:
        return self._rns

    @property
    def route(self) -> Optional[RouteSet]:
        """The route of the current / last ROUTE mission (kept after the
        terminal so path_progress can name route_id / route_rev)."""
        return self._route

    @property
    def goal(self) -> Optional[RelMoveGoal]:
        return self._goal

    @property
    def origin(self) -> Optional[Origin]:
        """Which entry the current / last mission came in on (12 S4.2c.1)."""
        return self._origin

    def nav_state(self) -> NavState:
        return self._rns.nav_state()

    def mission_loaded(self) -> bool:
        return self._rns.nav_state() != NavState.IDLE

    # ---- 12 S4.1 behaviour source interface ---------------------------------
    def is_active(self, ctx: Any) -> bool:
        return self._rns.is_active(ctx)

    def compute(self, ctx: Any):
        return self._rns.compute(ctx)

    def on_preempted(self, ctx: Any) -> None:
        """TR-RNS-1 / RNS-M-7: suspend, keep the mission."""
        self._rns.on_preempted(ctx)

    def on_release(self, ctx: Any) -> None:
        """Release -> RNS re-evaluates from FOLLOW (never replays)."""
        self._rns.on_release(ctx)

    def take_arrival(self) -> bool:
        return self._rns.take_arrival()

    def take_failure(self) -> Optional[NavFailure]:
        return self._rns.take_failure()

    # ---- mission entries -----------------------------------------------------
    def _supersede_if_running(self, now_mono_ms: int, new_rev: int) -> None:
        """20 S9.0.3 replacement: the running mission ends WITHOUT a failure
        report; audited as superseded (A-FAIL-3)."""
        if self.mission_loaded():
            old_rev = self._route.route_rev if self._route is not None else -1
            on_route_superseded(self._rns.audit, now_mono_ms, old_rev, new_rev)

    def load_route(self, route: RouteSet, now_mono_ms: int) -> None:
        """cmd/motion/route op=set -> PATH (>= 2 points) or GOTO (1 point).
        mutant: pass Origin.RELMOVE here -> a p3 route's failure would go to
        relative_move/status and p3 never sees it -> test_route_origin red."""
        self._supersede_if_running(now_mono_ms, route.route_rev)
        kind = MissionKind.PATH if route.waypoint_total >= 2 else MissionKind.GOTO
        self._rns.load_mission(Mission(
            kind, Origin.ROUTE, list(route.points_xy),
            search_window=self._search_window,
            arrival_radius_m=route.endpoint_arrive_radius_m,
            max_deviation_m=self._max_deviation_m))
        self._route = route
        self._goal = None
        self._origin = Origin.ROUTE

    def load_goto(self, goal: RelMoveGoal, now_mono_ms: int) -> None:
        """cmd/motion/relative_move -> GOTO with origin RELMOVE."""
        self._supersede_if_running(now_mono_ms, -1)
        self._rns.load_mission(Mission(
            MissionKind.GOTO, Origin.RELMOVE, [goal.endpoint_xy],
            search_window=self._search_window,
            arrival_radius_m=self._goto_arrival_radius_m,
            max_deviation_m=self._max_deviation_m))
        self._goal = goal
        self._route = None
        self._origin = Origin.RELMOVE

    def cancel(self, now_mono_ms: int) -> bool:
        """op=clear (12 S4.2c.4 / 20 S9.0.3): IDLE, audit cancelled, NO failure
        report. Returns whether a mission was actually in flight."""
        was = self.mission_loaded()
        if was:
            self._rns.audit.append(AuditRecord(
                now_mono_ms, Outcome.CANCELLED.value,
                {"origin": self._origin.value if self._origin else None}))
        self._rns.clear_mission()
        return was
