"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: source.py
Brief: RNS behavior-source shell (20 S1.1 RNS-M-3/M-7 / 12 S4.2c.1)

Description:
The ONLY outward face of the RNS module (RNS-M-3): the behavior-source interface
(is_active / compute / on_preempted / on_input_lost) that 12 S4.1 arbitrates.
Everything else in rns/ is reached only through here; no other module reads the
RNS grid or corridor (that would be a second d_free truth source, D-29).

P0.4 status: SHELL. is_active() returns False unconditionally and compute()
returns None, so wiring this into p1's arbiter does not change robot behavior
(RNS_TODO P0.4: "is_active recanted false, no smoke"). The tick body lands
across P1..P6; each phase flips on its slice.

Lifecycle contract this shell owns (kept here from day one so later phases fill
in, not restructure):
  - RNS-M-7 estop/preempt: on_preempted -> SUSPENDED, mission KEPT; release ->
    re-evaluate from FOLLOW, NEVER replay the frozen pre-suspend intent (the
    "release == recover last velocity" bug is what A-ES-2 kills).
  - 12 S4.2c.1 origin: mission carries origin (route|relmove); on failure the
    report channel is chosen by origin, not by mission category. source.py is
    where that choice is made (route -> path_progress.state; relmove -> status).
  - RNS-M-3: compute returns a VelocityCandidate OR None; None is the normal
    "no output this tick" value, distinct from a zero candidate.
"""

from __future__ import annotations

import math
from typing import Optional

from .route import Mission, lookahead_distance
from .speed import SpeedCaps, cap_deviation
from .types import MissionKind, NavState, Origin, VelocityCandidate
from xbrain.common.types.units import Mps


class RnsSource:
    """P1 behavior source `rns_avoid` (12 S4.2 priority 900). Process-local, no
    Zenoh session, no thread (RNS-M-1/M-4/M-5). Constructed once; ticked from the
    p1 ctrl loop with the tick snapshot.

    P7 assembly (this slice): compute() now runs the follow pipeline when a
    mission is loaded, using the modules built across P1..P6. is_active stays
    mission-gated: with NO mission (the state after wiring but before P3 pushes a
    route), is_active is False and compute returns None -- wiring this in still
    changes nothing until a mission is loaded (A-FAIL-2 / P0.4 no-smoke holds).

    The estop/preempt gate and the perception/grid/candidate/wall stages are NOT
    yet threaded here -- those need the real tick snapshot (ctx) shape, which
    lands with P7.1 perception wiring. This assembly is the follow+speed spine;
    the avoidance stages plug into it as ctx is defined."""

    def __init__(self, cfg: Optional[dict] = None) -> None:
        # cfg is the rns.yaml snapshot (12 S12.0A); None keeps the source inert
        # (no mission can be loaded without config, so is_active stays False).
        self._cfg = cfg
        self._state: NavState = NavState.IDLE
        self._mission: Optional[Mission] = None
        self._suspend = EstopSuspension()

    def load_mission(self, mission: Mission) -> None:
        """P3/route pushes a mission (goto or path). Transitions IDLE -> FOLLOW.
        The mission carries origin (route|relmove) for report routing."""
        self._mission = mission
        self._state = NavState.FOLLOW

    def clear_mission(self) -> None:
        """Terminal (arrive/fail/cancel) -> back to IDLE, no mission."""
        self._mission = None
        self._state = NavState.IDLE

    def is_active(self, ctx) -> bool:
        """12 S4.1: active iff a navigation mission is in flight and not
        SUSPENDED (20 S9.0.1). No mission -> False; suspended -> False. So an
        unwired / mission-less instance never holds the arbiter slot."""
        if self._mission is None:
            return False
        return not self._suspend.suspended()

    def compute(self, ctx) -> Optional[VelocityCandidate]:
        """RNS-M-3: candidate or None. No mission or suspended -> None (no
        output). With a mission, run the follow+speed spine: project, lookahead,
        deviation-limited speed. The avoidance stages (grid fusion, candidates,
        wall-follow) fold in as ctx gains their inputs (P7.1)."""
        if self._mission is None:
            return None
        cand = self._suspend.gate_output(self._run_follow(ctx))
        return cand

    def _run_follow(self, ctx) -> Optional[VelocityCandidate]:
        """The follow spine (P1 modules). ctx must supply pose (x, y, yaw) and
        the speed-cap config; when ctx lacks them (pre-P7.1), returns None rather
        than guessing (RNS-M-3: None is a valid 'no output')."""
        pose = getattr(ctx, "pose_xy", None)
        if pose is None or self._cfg is None:
            return None
        route_cfg = self._cfg["rns"]["route"]
        v_nom = getattr(ctx, "v_nom_mps", None)
        if v_nom is None:
            return None
        lka = lookahead_distance(v_nom, route_cfg["lookahead_k"],
                                 route_cfg["lookahead_min_m"],
                                 route_cfg["lookahead_max_m"])
        fs = self._mission.advance(pose, lka)
        if fs.arrived:
            return VelocityCandidate(vx=Mps(0.0), vy=Mps(0.0), wz=0.0)
        # deviation cap (the only cap wireable without perception ctx yet).
        speed_cfg = self._cfg["rns"]["speed"]
        cap = cap_deviation(fs.projection.deviation_m, v_nom,
                            speed_cfg["dev_e0_m"], route_cfg["max_deviation_m"],
                            speed_cfg["dev_g_min"])
        caps = SpeedCaps({"deviation": cap})
        v = caps.limit()
        # heading toward R (full align law folds in with P1.4 wiring here).
        r = fs.lookahead_point
        heading = math.atan2(r[1] - pose[1], r[0] - pose[0])
        return VelocityCandidate(vx=Mps(v), vy=Mps(0.0), wz=heading)

    def on_preempted(self, ctx) -> None:
        """RNS-M-7: teleop/estop took the slot. Suspend, KEEP mission. The
        EstopSuspension gate now forces compute() to zero (A-ES-1); SUSPENDED is
        non-terminal, so is_active goes False but the mission survives."""
        self._suspend.on_estop_or_preempt()
        self._state = NavState.SUSPENDED

    def on_release(self, ctx) -> None:
        """Preempt/estop released. Leave SUSPENDED -> FOLLOW and RE-EVALUATE: the
        next compute() runs a fresh follow tick; no pre-suspend intent is
        replayed (RNS-M-7 c / A-ES-2). Mission was kept, so FOLLOW resumes."""
        self._suspend.on_release()
        if self._mission is not None:
            self._state = NavState.FOLLOW

    def on_input_lost(self, ctx) -> None:
        """A required input (12 abort_reason input_lost family) went stale.
        SHELL: no-op; the real handling (which reason, which carrier) lands with
        the failure path once ctx carries the input-health flags (P7.1)."""
        return None


class EstopSuspension:
    """RNS-M-7 estop/preempt suspension logic (20 S1.1, A-ES-1/2). Separated from
    the RnsSource shell so it is testable now while is_active stays inert until
    P7 wiring.

    Three obligations (RNS-M-7):
      (a) while suspended, compute() emits ZERO output -- never a non-zero
          candidate (A-ES-1).
      (b) do not become active again before release.
      (c) on release, RE-EVALUATE from FOLLOW -- never replay the frozen pre-
          suspend intent (A-ES-2). The "release == recover last velocity" bug is
          "let go of estop and it lurches"; (c) kills it.

    The mission is KEPT across suspension (estop is not cancel); only the intent
    is discarded on release, forcing a fresh compute."""

    def __init__(self) -> None:
        self._suspended = False

    def on_estop_or_preempt(self) -> None:
        """Enter suspension. Mission kept; SUSPENDED is non-terminal (S9.0.1)."""
        self._suspended = True

    def on_release(self) -> None:
        """Leave suspension. The caller must RE-EVALUATE (fresh compute) -- this
        method deliberately carries no cached velocity to replay (A-ES-2)."""
        self._suspended = False

    def suspended(self) -> bool:
        return self._suspended

    def gate_output(self, candidate: "Optional[VelocityCandidate]"
                    ) -> "Optional[VelocityCandidate]":
        """(a): while suspended, force zero output. Returns None (no output) when
        suspended, regardless of what the tick computed -- the candidate the tick
        produced is discarded, not scaled. mutant: pass the candidate through
        while suspended -> non-zero output under estop -> A-ES-1 red."""
        if self._suspended:
            return None
        return candidate
