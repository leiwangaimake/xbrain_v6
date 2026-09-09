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

from .config import RnsConfigError, run_startup_assertions
from .route import Mission, align_omega, lookahead_distance
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

    def __init__(self, cfg: Optional[dict] = None,
                 r_eff_m: Optional[float] = None) -> None:
        # cfg is the rns.yaml snapshot (12 S12.0A); None keeps the source inert
        # (no mission can be loaded without config, so is_active stays False).
        # REVIEW-FIX W1 (2026-09-09): a non-None cfg MUST pass the startup
        # assertions here (D2 + person lock, config.py). They were written and
        # tested but never called from production -- the exact "built, tested,
        # not wired" failure CLAUDE.md 3.2 warns about. r_eff_m is required
        # alongside cfg because D2 compares against 2*r_eff; cfg without r_eff
        # cannot be validated -> refuse (fail-loud, never skip).
        if cfg is not None:
            if r_eff_m is None:
                raise RnsConfigError(
                    "RnsSource(cfg=...) needs r_eff_m for the D2 startup "
                    "assertion (20 S7A.1); refusing to construct unvalidated")
            run_startup_assertions(cfg, r_eff_m)
        self._cfg = cfg
        self._state: NavState = NavState.IDLE
        self._mission: Optional[Mission] = None
        self._suspend = EstopSuspension()
        self._arrived_pending = False   # W2: arrival latched for the host to take

    def load_mission(self, mission: Mission) -> None:
        """P3/route pushes a mission (goto or path). Transitions IDLE -> FOLLOW.
        The mission carries origin (route|relmove) for report routing."""
        self._mission = mission
        self._state = NavState.FOLLOW

    def clear_mission(self) -> None:
        """Terminal (arrive/fail/cancel) -> back to IDLE, no mission."""
        self._mission = None
        self._state = NavState.IDLE

    def take_arrival(self) -> bool:
        """W2: one-shot arrival latch. True exactly once after a mission
        arrived; the host routes the report by the mission's origin (12 S4.2c.6)
        and this resets. Latch (not callback) keeps RNS free of host callbacks
        (RNS-M-5)."""
        if self._arrived_pending:
            self._arrived_pending = False
            return True
        return False

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
        # REVIEW-FIX (2026-09-09): short-circuit while suspended BEFORE running
        # the follow pipeline -- otherwise Mission.advance() keeps advancing the
        # monotone index as a side effect during estop (output was gated, state
        # was not). A suspended tick must be a true no-op.
        if self._suspend.suspended():
            return None
        return self._run_follow(ctx)

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
            # REVIEW-FIX W2 (2026-09-09): arrival must END the mission, not just
            # zero the velocity. Before this fix the mission stayed loaded, so
            # is_active stayed True and the source held the 900 arbiter slot
            # FOREVER after arriving (every tick a zero candidate). Now: latch
            # the event for the host (take_arrival -> origin-routed report,
            # 12 S4.2c.6), clear to IDLE, emit one final zero candidate.
            self._arrived_pending = True
            self.clear_mission()
            return VelocityCandidate(vx=Mps(0.0), vy=Mps(0.0), wz=0.0)
        # deviation cap (the only cap wireable without perception ctx yet).
        speed_cfg = self._cfg["rns"]["speed"]
        cap = cap_deviation(fs.projection.deviation_m, v_nom,
                            speed_cfg["dev_e0_m"], route_cfg["max_deviation_m"],
                            speed_cfg["dev_g_min"])
        caps = SpeedCaps({"deviation": cap})
        v = caps.limit()
        # REVIEW-FIX B1 (2026-09-09): wz is an angular VELOCITY (rad/s), not a
        # bearing. The old placeholder returned atan2's absolute bearing to R --
        # with R due north it commanded a constant 1.57 rad/s spin regardless of
        # the robot's actual heading, never converging. Correct form: theta_des
        # is the bearing to R, and wz comes from the P law align_omega(theta_des,
        # psi_now, k_yaw, wz_max) (20 S2.5). That needs the CURRENT yaw and the
        # wz limit from ctx; missing either -> None (no output), never a guessed
        # rotation (RNS-M-3: None is the honest no-output value).
        yaw = getattr(ctx, "yaw_rad", None)
        wz_max = getattr(ctx, "wz_max_rps", None)
        if yaw is None or wz_max is None:
            return None
        r = fs.lookahead_point
        theta_des = math.atan2(r[1] - pose[1], r[0] - pose[0])
        wz = align_omega(theta_des, yaw, route_cfg["k_yaw"], wz_max)
        return VelocityCandidate(vx=Mps(v), vy=Mps(0.0), wz=wz)

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
