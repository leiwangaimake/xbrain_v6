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

from .audit import TerminalReporter
from .candidate import (CandidateSelector, candidate_cost, candidates_from_profile,
                        clear_extrapolation, clearance_at, obstacle_points,
                        passes_hard_gates)
from .classify import (behavior_class, dispatch_dynamic, health_speed_capped,
                       usable_velocity)
from .config import RnsConfigError, run_startup_assertions
from .dynamic import DynamicAction, WaitBudget, distance_action, in_corridor
from .grid import (MemoryGrid, profile_age_ms, profile_speed_limited,
                   profile_zero_speed, seg_stale)
from .planner import GuidancePlanner
from .wallfollow import (Side, WallFollowState, can_enter, can_leave,
                         check_failure, inner_corner_stop, keep_distance_omega,
                         record_crossing, select_side, wall_vanished)
from .watchdog import ProgressWatchdog, WatchdogResult, no_progress_failure
from .audit import AuditRecord, RingAudit
from .route import Mission, align_omega, lookahead_distance, wrap_angle
from .speed import (SpeedCaps, cap_deviation, cap_gap_tightness,
                    cap_unknown_ratio)
from .types import (Cell, MissionKind, NavFailReason, NavFailure, NavState, Origin,
                    VelocityCandidate, is_legal_transition)
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
        self._r_eff_m = r_eff_m
        # ── S2 assembly state ──
        self._reporter = TerminalReporter()
        self._failed_pending: Optional[NavFailure] = None
        self._selector: Optional[CandidateSelector] = None
        self._subgoal_world = None          # DETOUR target (world frame)
        self._subgoal_clearance_m = None    # adopted subgoal's gap clearance
        self._detour_dwell = 0              # ticks since DETOUR entry (S6.6)
        self._wait_budget: Optional[WaitBudget] = None
        self._dyn_prev = DynamicAction.RUN
        self._pre_wait_state: NavState = NavState.FOLLOW
        self._slow_since: dict = {}         # track_id -> ms when speed dropped
        self._grid: Optional[MemoryGrid] = None
        self._watchdog: Optional[ProgressWatchdog] = None
        self._wall: Optional[WallFollowState] = None
        self._s_star = float("-inf")     # monotone best arc progress (RNS-N-15)
        self._last_pose = None
        self._last_now = None
        self._last_d_side: Optional[float] = None
        self._wall_corner = False        # concave-corner turn in progress
        self._wall_stall_ticks = 0       # in-wall stall backstop (tick count)
        self._wall_goal_dist_at_hit = 0.0
        self._holo = False
        self._last_R = None                  # lookahead point, for observability
        self._r_star = None                  # guidance point (20 S4A), per tick
        self._r_steer = None                 # known-field steering guide
        self._chain_prefix_m = 0.0           # served chain: confirmed prefix
        self._chain_prefix_end = None
        self._wall_cooldown_until = None     # R* steering mute after wall_exit
        self._wall_entry_dwell = 0           # exit-eval mute after wall_enter
        self.audit = RingAudit(capacity=256)
        if cfg is not None:
            c = cfg["rns"]
            self._selector = CandidateSelector(c["candidate"]["side_hold_ticks"])
            self._wait_budget = WaitBudget(c["dynamic"]["wait_budget_s"])
            m = c["memory"]
            self._grid = MemoryGrid(m["cell_m"], m["ttl_static_s"],
                                    m["ttl_dynamic_s"], m["reset_jump_m"],
                                    m["radius_m"])
            w = c["watchdog"]
            self._watchdog = ProgressWatchdog(int(w["window_s"] * 1000),
                                              w["min_progress_m"])
            # guidance layer (20 S4A): missing config key fails loud here --
            # a silently-absent planner would demote every direction decision
            # back to local heuristics with no error anywhere.
            self._planner = GuidancePlanner(c)
        else:
            self._planner = None

    def load_mission(self, mission: Mission) -> None:
        """P3/route pushes a mission (goto or path). Transitions IDLE -> FOLLOW.
        The mission carries origin (route|relmove) for report routing."""
        self._mission = mission
        self._state = NavState.FOLLOW
        self._reporter.reset_for_new_mission()
        if self._planner is not None:
            # GUIDANCE IS GOTO-ONLY (field bug 2026-09-11, user: "path run
            # stopped following the path"): a goto's reference line is a
            # direction hint and shortcutting it is the optimization; a
            # PATH's polyline IS the task (patrol line, recorded-safe
            # ground) -- R* pulling toward the globally shortest way lifted
            # the robot OFF the line it was ordered to walk. One gate here
            # starves every guidance consumer (R*/steer/chain/no-path) for
            # path missions; avoidance falls back to the v1.0 candidate/
            # wall machinery, whose leave rules RETURN to the line (S7.3).
            # GUIDANCE IS GOTO-ONLY (field bug 2026-09-11 + acceptance
            # sweep): an endpoint-rooted field serves a PATH badly twice
            # over -- steering pulls off the ordered line, and even the
            # detour-side aim drags toward "shortcut to the endpoint"
            # instead of "rejoin the line" (path_rev regressed OK->FAIL
            # when tried). PATH missions run pure line-following; their
            # avoidance uses the candidate/wall machinery whose leave
            # rules return to the line (S7.3).
            if mission.kind == MissionKind.GOTO:
                self._planner.set_task(mission.endpoint, mission.endpoint)
            else:
                self._planner.clear()
        self._subgoal_world = None
        self._dyn_prev = DynamicAction.RUN
        self._wall = None
        self._s_star = float("-inf")
        # REVIEW R1-2 (3-pass audit): cross-order residue. The wall cooldown
        # muted R* steering for the new order's first 2 s; dwell counters
        # and per-track dwell clocks leaked the previous order's state.
        self._wall_cooldown_until = None
        self._wall_entry_dwell = 0
        self._detour_dwell = 0
        self._wall_stall_ticks = 0
        self._last_d_side = None
        self._wall_corner = False
        self._slow_since = {}
        if self._cfg is not None:
            w = self._cfg["rns"]["watchdog"]
            self._watchdog = ProgressWatchdog(int(w["window_s"] * 1000),
                                              w["min_progress_m"])
        if self._cfg is not None:
            c = self._cfg["rns"]
            self._selector = CandidateSelector(c["candidate"]["side_hold_ticks"])
            self._wait_budget = WaitBudget(c["dynamic"]["wait_budget_s"])

    def clear_mission(self) -> None:
        """Terminal (arrive/fail/cancel) -> back to IDLE, no mission."""
        self._mission = None
        self._state = NavState.IDLE
        self._last_R = None
        if self._planner is not None:
            self._planner.clear()           # S4A.4: domain dies with the task

    def take_arrival(self) -> bool:
        """W2: one-shot arrival latch. True exactly once after a mission
        arrived; the host routes the report by the mission's origin (12 S4.2c.6)
        and this resets. Latch (not callback) keeps RNS free of host callbacks
        (RNS-M-5)."""
        if self._arrived_pending:
            self._arrived_pending = False
            return True
        return False

    def take_failure(self) -> "Optional[NavFailure]":
        """One-shot failure latch (mirror of take_arrival): the terminal
        NavFailure reported by this mission, exactly once; the host routes it by
        origin (12 S4.2c.6)."""
        f = self._failed_pending
        self._failed_pending = None
        return f

    def nav_state(self) -> NavState:
        return self._state

    def _fail(self, failure: NavFailure) -> None:
        """Terminal failure: report once (A-FAIL-1 via TerminalReporter), latch
        for the host, clear to IDLE (S9.0.3). Never auto-retries (A-FAIL-2)."""
        rep = self._reporter.report_failure(failure)
        if rep is not None:
            self._failed_pending = rep
        self.clear_mission()

    def _transition(self, to: NavState) -> None:
        """A-ST-1: table-external transitions THROW, never pass silently."""
        if not is_legal_transition(self._state, to):
            raise RuntimeError("illegal NavState transition %s -> %s (A-ST-1)"
                               % (self._state.value, to.value))
        self._state = to

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
        """The assembled tick (S2): perception gating -> follow geometry ->
        dynamic-obstacle rule -> static avoidance state machine -> speed/heading.
        ctx must supply pose_xy/yaw_rad/v_nom_mps/wz_max_rps; perception (a
        PerceptionSnapshot) and now_mono_ms unlock the avoidance stages -- absent
        they degrade to the bare follow spine (pre-P7.1 hosts)."""
        pose = getattr(ctx, "pose_xy", None)
        yaw = getattr(ctx, "yaw_rad", None)
        v_nom = getattr(ctx, "v_nom_mps", None)
        wz_max = getattr(ctx, "wz_max_rps", None)
        if pose is None or yaw is None or v_nom is None or wz_max is None \
                or self._cfg is None:
            return None
        snap = getattr(ctx, "perception", None)
        now = getattr(ctx, "now_mono_ms", None)
        self._holo = bool(getattr(ctx, "holonomic", False))
        cfg = self._cfg["rns"]
        route_cfg = cfg["route"]
        zero = VelocityCandidate(vx=Mps(0.0), vy=Mps(0.0), wz=0.0)
        caps = {"mission": v_nom}

        # ── A. perception age / health gates (20 S3.1, T-50/T-51) ────────────
        profile = snap.profile if snap is not None else None
        if profile is not None and now is not None:
            age = profile_age_ms(profile, now)
            if profile_zero_speed(age, 1000):        # T-51: too old -> zero
                return zero
            if profile_speed_limited(age, 300):      # T-50: aged -> slow
                caps["profile_age"] = 0.5 * v_nom
            if seg_stale(profile, cfg["perception"]["seg_stale_ms"]):
                caps["no_seg"] = cfg["perception"]["no_seg_speed_cap_mps"]
        if snap is not None and snap.status is not None:
            if health_speed_capped(snap.status.invalid_pixel_ratio,
                                   cfg["health"]["invalid_ratio_limit"]):
                # RNS-I-2: sensor-blind cover; cap value shares the no-seg key
                # (a dedicated key is a calibration-time decision).
                caps["health"] = cfg["perception"]["no_seg_speed_cap_mps"]

        # ── B. follow geometry ────────────────────────────────────────────────
        lka = lookahead_distance(v_nom, route_cfg["lookahead_k"],
                                 route_cfg["lookahead_min_m"],
                                 route_cfg["lookahead_max_m"])
        fs = self._mission.advance(pose, lka)
        self._last_R = fs.lookahead_point    # observability: host/UI may draw it
        if fs.arrived:
            self._arrived_pending = True
            self.clear_mission()
            return zero
        # deviation FAILURE removed (user ruling 2026-09-10, 20 S2.7 v1.20):
        # two live runs to a goal BEHIND the car wall both died on max_deviation
        # right as the detour completed -- a legitimate long detour necessarily
        # runs a large e on the way home, and the v1.17 wall-state exemption
        # missed the first FOLLOW tick after leaving the wall. Deviation now
        # only SLOWS (the dev cap in E); boundedness belongs to the watchdog
        # (S7.3A) and the wall criteria (S7.6).

        # ── B2. memory grid + progress bookkeeping (S2b) ─────────────────────
        if profile is not None and now is not None and self._grid is not None:
            self._grid.on_pose(pose)
            self._grid.ingest_profile(profile, pose, yaw, now)
            self._grid.maybe_sweep(pose, now)   # SYNC AUDIT S1: bounded memory
        # guidance layer (20 S4A): advance the budgeted build, then take R*.
        # None -> every consumer below falls back to its v1.0 target.
        self._r_star = None
        self._r_steer = None
        # REVIEW R1-1: reset the chain prefix EVERY tick -- with planner or
        # now absent the previous tick's prefix survived and the blocked
        # branch could chase a stale chain point.
        self._chain_prefix_m = 0.0
        self._chain_prefix_end = None
        if self._planner is not None and now is not None:
            # G2 event trigger: the served descent chain crossing a freshly
            # remembered BLOCKED forces a rebuild now, not at the period.
            if self._planner.chain_cut(self._grid, now):
                self._planner.request_replan()
            self._planner.on_tick(self._grid, pose, now)
            # two grades of guidance (S4A.5 note): guide_point serves the
            # discrete decisions whatever the mode; steering_guide pulls the
            # continuous heading only off a KNOWN (observed-FREE) field.
            self._r_star = self._planner.guide_point(pose)
            self._r_steer = self._planner.steering_guide(pose)
            # line-following (user order 2026-09-11): under an ATTEMPT
            # field, the chain's observed-FREE prefix IS confirmation --
            # steer to its end (never past it). The 96%-unsteered audit
            # showed the plan existed but the legs ignored it.
            self._chain_prefix_m, self._chain_prefix_end = \
                self._planner.chain_free_prefix(self._grid, now)
            if self._r_steer is None and self._chain_prefix_m >= 1.0:
                self._r_steer = self._chain_prefix_end
            # G2 (S4A.3): a PROVEN in-domain no-path terminates the mission
            # with its own reason -- circling until wall_no_progress would
            # bury a provable verdict under a tired-of-trying heuristic.
            if self._planner.domain_no_path(now):
                self._fail(NavFailure(
                    NavFailReason.NO_PATH_IN_DOMAIN,
                    detail={"goal": list(self._mission.endpoint),
                            "unreachable_builds": 2}))
                return zero
        self._s_star = max(self._s_star, fs.projection.s_arc_m)
        step_m = 0.0
        if self._last_pose is not None:
            step_m = math.hypot(pose[0] - self._last_pose[0],
                                pose[1] - self._last_pose[1])
        dt_ms = (now - self._last_now) if (now is not None and
                                           self._last_now is not None) else 50
        self._last_pose = pose
        self._last_now = now

        # target: the subgoal while detouring, else R* under a WALL COOLDOWN
        # (G1 sweeps g1a-g1e settled this): continuous R* pulling is the
        # layer's core win (south->gap_mid 57.8 m -> ~16 m came from pure
        # FOLLOW riding R* around the column -- no wall contact at all), but
        # pulling DURING/RIGHT AFTER a wall walk tangles with the Bug2
        # geometry (churn FAILs). So R* steers in open running only: never
        # in WALL_FOLLOW (that branch has its own laws) and not within the
        # post-leave cooldown, where the plain R lets the leave settle.
        if self._state == NavState.DETOUR and self._subgoal_world is not None:
            target = self._subgoal_world
        elif self._mission.kind == MissionKind.GOTO \
                and self._r_steer is not None and (
                self._wall_cooldown_until is None or now is None
                or now >= self._wall_cooldown_until):
            target = self._r_steer
        else:
            target = fs.lookahead_point

        # ── C. dynamic obstacles (20 S5) ─────────────────────────────────────
        half_w = self._r_eff_m + 0.5    # corridor half-width: r_eff + margin
        blocking = None
        dyn_objs = []                   # world (x, y, r) of DYNAMIC-pile objects
        if snap is not None and snap.objects is not None and now is not None:
            dyn_cfg = cfg["dynamic"]
            stopped = slowed = False
            cy, sy = math.cos(yaw), math.sin(yaw)
            # corridor end: the stop rule speaks in stop/resume DISTANCES, so
            # the checked segment must reach that far -- the lookahead R sits
            # only 1-1.3 m out and TRUNCATED the rule (assembly hole #3: a
            # person at 2.7 m was "not in the corridor" and got detoured
            # instead of waited for, violating the S5.3 semantics).
            tdx, tdy = target[0] - pose[0], target[1] - pose[1]
            tlen = math.hypot(tdx, tdy)
            ext = max(dyn_cfg["resume_dist_m"], tlen)
            if tlen > 1e-6:
                cor_end = (pose[0] + tdx / tlen * ext,
                           pose[1] + tdy / tlen * ext)
            else:
                cor_end = target
            for obj in snap.objects.objects:
                spd = usable_velocity(
                    obj.velocity_frame,
                    math.hypot(obj.velocity_xy[0], obj.velocity_xy[1]),
                    cfg["perception"]["raw_velocity_policy"])
                beh = behavior_class(obj.class_name, cfg["class_map"])
                # static-criterion dwell bookkeeping (20 S5.5): raw-refused
                # speed (None) is conservatively treated as moving.
                eff_spd = spd if spd is not None else 999.0
                if eff_spd < dyn_cfg["v_static_thresh_mps"]:
                    t0 = self._slow_since.setdefault(obj.track_id, now)
                    dwell_s = (now - t0) / 1000.0
                else:
                    self._slow_since.pop(obj.track_id, None)
                    dwell_s = 0.0
                if not dispatch_dynamic(beh, eff_spd, dwell_s,
                           dyn_cfg["v_static_thresh_mps"],
                           dyn_cfg["t_static_dwell_s"]):
                    continue        # static pile: geometry (profile) covers it
                # body-frame centroid -> world; corridor test against target.
                fp = obj.footprint_xy
                if not fp:
                    continue    # REVIEW R1-11: malformed object, no geometry
                bx = sum(q[0] for q in fp) / len(fp)
                by = sum(q[1] for q in fp) / len(fp)
                ox = pose[0] + bx * cy - by * sy
                oy = pose[1] + bx * sy + by * cy
                r_obs = max(math.hypot(q[0] - bx, q[1] - by) for q in fp)
                dyn_objs.append((ox, oy, r_obs))
                if in_corridor(ox, oy, r_obs, pose[0], pose[1],
                               cor_end[0], cor_end[1], half_w):
                    act = distance_action(True, obj.r_near, self._dyn_prev,
                                          dyn_cfg["stop_dist_m"],
                                          dyn_cfg["resume_dist_m"])
                    if act == DynamicAction.STOP:
                        stopped = True
                        blocking = obj
                    elif act == DynamicAction.SLOW:
                        slowed = True
            self._dyn_prev = (DynamicAction.STOP if stopped else
                              DynamicAction.SLOW if slowed else
                              DynamicAction.RUN)
            if stopped:
                if self._state != NavState.WAIT_DYNAMIC:
                    self._pre_wait_state = self._state
                    self._transition(NavState.WAIT_DYNAMIC)
                wf = self._wait_budget.tick(
                    True, now, blocking.track_id if blocking else None,
                    blocking.class_name if blocking else None)
                if wf is not None:                   # RNS-N-16 budget blown
                    self._fail(wf)
                return zero
            self._wait_budget.tick(False, now, None, None)
            if self._state == NavState.WAIT_DYNAMIC:
                self._transition(self._pre_wait_state)
            if slowed:
                caps["dynamic"] = 0.5 * v_nom

        # ── C2. wall-follow tick (S2b -- 20 S7) ──────────────────────────────
        if self._state == NavState.WALL_FOLLOW:
            return self._wall_tick(profile, pose, yaw, fs, now, wz_max, step_m)

        # ── D. static avoidance state machine (20 S6) ────────────────────────
        if profile is not None:
            # profile is class-blind geometry; 20 S6.2 says take the MAX margin
            # of the gap's flanking classes -- unknowable here, so take the max
            # of the table (collision fix #2: structure 0.3 under-margined cars,
            # 195 ticks inside the U54 1 m keep-out on the user's audit).
            margin = max(cfg["clearance"]["margin_by_class"].values())
            gate_base = self._r_eff_m + margin
            blocked = self._ahead_blocked(profile, pose, yaw, target,
                                          cfg["dynamic"]["stop_dist_m"])
            if self._state == NavState.FOLLOW and blocked:
                # line-following first (user order 2026-09-11): a served
                # chain with a long-enough CONFIRMED-FREE prefix beats the
                # FOV candidate contest -- the guide already routed around
                # the blockage; jumping to its prefix end keeps the robot
                # ON the plan instead of degrading into wall-follow (72.5%
                # of ticks pre-fix). Short prefix -> the old path stands.
                chain_ok = False
                if self._mission.kind == MissionKind.GOTO \
                        and self._chain_prefix_m >= 2.0:
                    # the chain point passes the SAME clearance gate as any
                    # candidate subgoal (line1 sweep: skipping it let the
                    # chain ride obstacle edges to -0.012 m) -- one yard-
                    # stick for every subgoal, whatever proposed it.
                    obs_pts = obstacle_points(profile.d_block,
                                              profile.angle_min_rad,
                                              profile.angle_step_rad)
                    sub_body = self._world_to_body(self._chain_prefix_end,
                                                   pose, yaw)
                    chain_clear = clearance_at(sub_body, obs_pts,
                                               profile.range_max_m)
                    chain_ok = chain_clear >= gate_base
                if chain_ok:
                    # falls through to E (speed+heading toward the chain
                    # point), same as the candidate-subgoal branch below.
                    self._subgoal_clearance_m = chain_clear
                    self._subgoal_world = self._chain_prefix_end
                    self._transition(NavState.DETOUR)
                    self._detour_dwell = 0
                    target = self._subgoal_world
                    self.audit.append(AuditRecord(
                        now or 0, "detour_enter",
                        {"subgoal": target, "via": "guide_chain"}))
                elif (sel := self._selector.select(
                        False,
                        self._pick_margin_ladder(profile, pose, yaw, fs, cfg,
                                                 margin, dyn_objs),
                        best_is_current=False)) is not None:
                    self._subgoal_clearance_m = sel.clearance_m
                    self._subgoal_world = self._body_to_world(
                        sel.subgoal, pose, yaw)
                    self._transition(NavState.DETOUR)
                    self._detour_dwell = 0
                    target = self._subgoal_world
                    self.audit.append(AuditRecord(now or 0, "detour_enter",
                                                  {"subgoal": target}))
                else:
                    # hole #6 rescue layer: before falling to wall-follow, ask
                    # once more WITH memory appeal (out-of-FOV subgoals whose
                    # corridor the grid has walked). Inert on first encounter.
                    best = self._pick_candidate(profile, pose, yaw, fs, cfg,
                                                margin, dyn_objs,
                                                memory_appeal=True)
                    if best is None:
                        best = self._pick_candidate(
                            profile, pose, yaw, fs, cfg,
                            cfg["clearance"]["margin_by_class"]["small_object"],
                            dyn_objs, memory_appeal=True)
                    sel = self._selector.select(False, best,
                                                best_is_current=False)
                    if sel is not None:
                        self._subgoal_clearance_m = sel.clearance_m
                        self._subgoal_world = self._body_to_world(
                            sel.subgoal, pose, yaw)
                        self._transition(NavState.DETOUR)
                        self._detour_dwell = 0
                        target = self._subgoal_world
                        self.audit.append(AuditRecord(
                            now or 0, "detour_enter",
                            {"subgoal": target, "via": "memory_appeal"}))
                    elif (esc := self._small_obstacle_escape(
                            profile, pose, yaw, now)) is not None:
                        self._subgoal_clearance_m = None
                        self._subgoal_world = esc
                        self._transition(NavState.DETOUR)
                        self._detour_dwell = 0
                        target = esc
                        self.audit.append(AuditRecord(
                            now or 0, "detour_enter",
                            {"subgoal": esc, "via": "escape"}))
                    elif self._enter_wall(profile, pose, yaw, fs, now):
                        return self._wall_tick(profile, pose, yaw, fs, now,
                                               wz_max, step_m)
                    else:
                        return zero
            elif self._state == NavState.DETOUR:
                self._detour_dwell += 1
                d_sub = math.hypot(pose[0] - self._subgoal_world[0],
                                   pose[1] - self._subgoal_world[1])
                r_clear = not self._ahead_blocked(
                    profile, pose, yaw, fs.lookahead_point,
                    cfg["dynamic"]["stop_dist_m"], unseen_is_blocked=True,
                    now=now)
                # the OPPORTUNISTIC exit (r_clear) waits out a minimum dwell
                # (side_hold_ticks, same S6.6 hysteresis family): a 1-tick
                # enter/exit flip is never meaningful steering, only limit-
                # cycle fuel. Arrival (d_sub) and the infeasible drop below
                # (A-HYS-2) stay immediate -- safety exits take no dwell.
                hold = cfg["candidate"]["side_hold_ticks"]
                if d_sub < 0.7 or (r_clear and self._detour_dwell >= hold):
                    self._subgoal_world = None
                    self._subgoal_clearance_m = None
                    self._selector.select(False, None, False)
                    self._transition(NavState.FOLLOW)
                    target = fs.lookahead_point
                else:
                    obs = obstacle_points(profile.d_block,
                                          profile.angle_min_rad,
                                          profile.angle_step_rad)
                    sub_body = self._world_to_body(self._subgoal_world,
                                                   pose, yaw)
                    feasible = clearance_at(sub_body, obs,
                                            profile.range_max_m) >= gate_base
                    if not feasible:                 # A-HYS-2: drop NOW
                        best = self._pick_candidate(profile, pose, yaw, fs,
                                                    cfg, margin, dyn_objs)
                        sel = self._selector.select(False, best, False)
                        if sel is None:
                            best = self._pick_candidate(
                                profile, pose, yaw, fs, cfg, margin, dyn_objs,
                                memory_appeal=True)
                            sel = self._selector.select(False, best, False)
                        if sel is None:
                            esc = self._small_obstacle_escape(
                                profile, pose, yaw, now)
                            if esc is not None:
                                self._subgoal_clearance_m = None
                                self._subgoal_world = esc
                                self._detour_dwell = 0
                                target = esc
                                self.audit.append(AuditRecord(
                                    now or 0, "detour_enter",
                                    {"subgoal": esc, "via": "escape"}))
                            else:
                                self._subgoal_world = None
                                if self._enter_wall(profile, pose, yaw, fs,
                                                    now):
                                    return self._wall_tick(
                                        profile, pose, yaw, fs, now,
                                        wz_max, step_m)
                                self._transition(NavState.FOLLOW)
                                return zero
                        if sel is not None:
                            self._subgoal_clearance_m = sel.clearance_m
                            self._subgoal_world = self._body_to_world(
                                sel.subgoal, pose, yaw)
                            target = self._subgoal_world

        # ── E. speed + heading toward target ─────────────────────────────────
        # REVIEW R2-3 (3-pass audit 2026-09-11): the S8.1A gap-tightness cap
        # (cap_gap_tightness + gap_g_min, on the books since P4) had NO
        # consumer -- threading a tight gap ran at full detour speed. While
        # DETOURING, rho = the adopted subgoal's clearance over the gate.
        if self._state == NavState.DETOUR \
                and self._subgoal_clearance_m is not None and profile is not None:
            margin_e = max(cfg["clearance"]["margin_by_class"].values())
            rho = self._subgoal_clearance_m / max(1e-6,
                                                  self._r_eff_m + margin_e)
            caps["gap"] = cap_gap_tightness(
                rho, v_nom, cfg["clearance"]["gate_saturate_ratio"],
                cfg["speed"]["gap_g_min"])
        # SYNC AUDIT S2 (2026-09-11): the S8.1A UNKNOWN-share cap existed as
        # three tuned keys with no consumer. Share = fraction of forward-
        # sector bins (+-45 deg) with no FREE evidence; more unknown ahead
        # -> slower, floor unk_g_min.
        if profile is not None:
            n_fwd = 0
            n_unk = 0
            for i in range(profile.n_bins):
                ang = profile.angle_min_rad + i * profile.angle_step_rad
                if abs(ang) > math.pi / 4:
                    continue
                n_fwd += 1
                if profile.d_free[i] is None:
                    n_unk += 1
            if n_fwd:
                sp = cfg["speed"]
                caps["unknown"] = cap_unknown_ratio(
                    n_unk / n_fwd, v_nom, sp["unk_r0"], sp["unk_r1"],
                    sp["unk_g_min"])
        caps["deviation"] = cap_deviation(
            fs.projection.deviation_m, v_nom, cfg["speed"]["dev_e0_m"],
            route_cfg["max_deviation_m"], cfg["speed"]["dev_g_min"])
        theta_des = math.atan2(target[1] - pose[1], target[0] - pose[0])
        wz = align_omega(theta_des, yaw, route_cfg["k_yaw"], wz_max)
        v = SpeedCaps(caps).limit()
        # heading-error speed shaping: CONTINUOUS (cos taper), not a step.
        # The first cut ("err > 1 rad -> clamp to 0.2") was a step function --
        # every steering correction crossed it and the robot lurched
        # (drive-stop-drive stutter, user-reported). cos(err) tapers smoothly:
        # full speed aligned, ~0 speed sideways, floor 0.15 keeps it creeping.
        err = abs(wrap_angle(theta_des - yaw))
        v = v * max(0.15, math.cos(min(err, math.pi / 2)))

        # UNIVERSAL contact fuse (collision audit #2 cont.): the wall-state fuse
        # alone still let the robot graze a car corner in FOLLOW right after
        # leaving the wall (host f() gate covers only +/-10 deg; the corner sat
        # off-axis). ANY live hit closer than 0.7 m inside the +/-60 deg drive
        # cone zeroes v this tick (turn continues; upper layers re-plan). A
        # normal detour never trips this -- its corridor gate guarantees
        # >= 1.1 m -- only an abnormal approach does.
        if profile is not None:
            for i, db in enumerate(profile.d_block):
                if db is None or db >= 0.7:
                    continue
                ang_off = abs(wrap_angle(profile.angle_min_rad
                                         + i * profile.angle_step_rad
                                         + yaw - theta_des))
                if ang_off < math.pi / 3:
                    v = 0.0
                    break

        # ── progress watchdog (20 S7.3A, RNS-N-15) -- placed HERE, after the
        # heading error is known, because its timing domain EXCLUDES ticks
        # whose binding limiter is heading/rtk. FIELD BUG (user, 2026-09-10):
        # the first assembly passed limiter=None always, so the turn-toward-
        # goal phase at mission start (big err, cos-tapered crawl, arc
        # projection flat) COUNTED as lost and false-fired
        # watchdog_no_progress ~8 s after a goto behind the robot.
        if self._watchdog is not None and profile is not None:
            limiter = "heading" if err > 0.9 else SpeedCaps(caps).binding_source()
            boundary = any(d is not None for d in profile.d_block)
            wd = self._watchdog.tick(
                self._s_star, dt_ms, self._state.value,
                self._state == NavState.WAIT_DYNAMIC, limiter, boundary)
            if wd == WatchdogResult.ESCALATE_WALL:
                if self._enter_wall(profile, pose, yaw, fs, now):
                    return self._wall_tick(profile, pose, yaw, fs, now,
                                           wz_max, step_m)
            elif wd == WatchdogResult.REPORT_NO_PROGRESS:
                self._fail(no_progress_failure(limiter))
                return zero
        vy_cmd = 0.0
        if self._holo:
            vy_cmd = self._body_shield_vy(pose, yaw, now)
        # shield vx cap on the FOLLOW exit too (line2 sweep: -0.030 m at
        # the wall's north corner -- FOLLOW rode the guide past the corner
        # at 0.70 m/s with the corner behind the FOV; the wall-tick exits
        # got this cap in the night batch, this exit was the gap). The
        # final-approach is EXEMPT (line3 sweep: a goal parked 0.75 m off a
        # car kept the ring permanently hot and the cap crawled the last
        # meters at 0.15 m/s for 122 s) -- inside 2 m the arrival slowdown
        # and the vy shield own the contact question.
        if fs.dist_to_endpoint_m > 2.0:
            v = min(v, self._shield_vx_cap(pose, now))
        return VelocityCandidate(vx=Mps(v), vy=Mps(vy_cmd), wz=wz)

    # ── assembly helpers ─────────────────────────────────────────────────────
    def _body_shield_vy(self, pose, yaw, now) -> float:
        """vy side-shield (residual-graze fix, user-ordered 2026-09-10): the
        0.073 m graze happened sliding past an already-rounded car corner
        sitting BEHIND the robot -- outside the 90-deg FOV, invisible to every
        profile-based fuse. The corner IS in the memory grid (just walked).
        Scan the grid in a 0.60 m ring around the body; the nearest remembered
        BLOCKED cell pushes a lateral vy AWAY from it, forward speed untouched
        (sidestep, not stop -- M20S is holonomic; the host passes
        ctx.holonomic from models/<chassis>.yaml spec, so a tracked chassis
        disables this with zero code). 0.60 < the wall-follow centre distance
        (d_wall 1.0), so normal wall hugging never trips it."""
        if self._grid is None or now is None:
            return 0.0
        cell = self._grid._cell_m
        # 0.85 not 0.60: the graze debug showed a corner at 0.585 m slipping
        # under a 0.60 ring through the 0.25 m cell quantization; the wall-
        # follow centre distance is 1.0 so hugging still never trips this.
        r_scan = 0.85
        steps = int(r_scan / cell) + 1
        best_d = None
        best_ang = 0.0
        for ix in range(-steps, steps + 1):
            for iy in range(-steps, steps + 1):
                wx = pose[0] + ix * cell
                wy = pose[1] + iy * cell
                if self._grid.read(wx, wy, now) == Cell.BLOCKED:
                    d = math.hypot(wx - pose[0], wy - pose[1])
                    if d < r_scan and (best_d is None or d < best_d):
                        best_d = d
                        best_ang = math.atan2(wy - pose[1], wx - pose[0])
        if best_d is None:
            return 0.0
        rel = wrap_angle(best_ang - yaw)
        push = min(0.35, (r_scan - best_d) * 3.0)
        return -math.sin(rel) * push     # obstacle left -> push right, v.v.

    def _shield_vx_cap(self, pose, now) -> float:
        """Speed cap companion to the vy shield (batch probe 2026-09-10,
        east->gap_s 0.113 m): rounding a corner at the full wall speed, the
        keep-distance PD lags ~0.4 m through the turn and the corner ends up
        BEHIND the body -- live fuses blind, the vy push alone out-muscled.
        A remembered BLOCKED inside the ring caps vx so the turn tightens and
        the push wins. Open ring -> no cap (returns +inf)."""
        d = self._nearest_mem_blocked(pose, now, r_scan=0.85)
        if d is None:
            return float("inf")
        # 0.60 datum + 1.5 slope: the memory dots sit ON the wall face but
        # sparse/quantized, so the ring reads ~0.2 long -- at a read of 0.79
        # (the graze tick) this caps to 0.29; a normal hug (read >= 1.0)
        # caps to >= 0.6, above the wall v_max -- no drag on cruise.
        return max(0.15, (d - 0.60) * 1.5)

    def _nearest_mem_blocked(self, pose, now, r_scan=0.85):
        """Distance to the nearest remembered BLOCKED cell within r_scan, or
        None. Shared by the vy shield and the goto leave-point cleanliness
        check."""
        if self._grid is None or now is None:
            return None
        cell = self._grid._cell_m
        steps = int(r_scan / cell) + 1
        best = None
        for ix in range(-steps, steps + 1):
            for iy in range(-steps, steps + 1):
                wx = pose[0] + ix * cell
                wy = pose[1] + iy * cell
                if self._grid.read(wx, wy, now) == Cell.BLOCKED:
                    d = math.hypot(wx - pose[0], wy - pose[1])
                    if d < r_scan and (best is None or d < best):
                        best = d
        return best

    @staticmethod
    def _body_to_world(p, pose, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return (pose[0] + p[0] * c - p[1] * s, pose[1] + p[0] * s + p[1] * c)

    @staticmethod
    def _world_to_body(p, pose, yaw):
        c, s = math.cos(-yaw), math.sin(-yaw)
        dx, dy = p[0] - pose[0], p[1] - pose[1]
        return (dx * c - dy * s, dx * s + dy * c)

    def _ahead_blocked(self, profile, pose, yaw, target, trigger_m,
                       unseen_is_blocked: bool = False, now=None) -> bool:
        """Is the bearing toward `target` blocked within trigger_m? Checks the
        profile bins in a corridor-width angular window around that bearing.

        unseen_is_blocked separates two DIFFERENT questions (field bug #2,
        user trace 2026-09-10: 254 detour_enters at one spot):
          False -> "do I SEE an obstacle there?" (DETOUR entry: only visible
                   obstacles justify detouring; an out-of-FOV bearing must not
                   trigger avoidance -- else a goal behind the robot deadlocks
                   the start-up turn).
          True  -> "is it CONFIRMED clear there?" (DETOUR exit / wall-follow
                   goal_open). Confirmed = perception UNION memory: an
                   out-of-FOV bearing consults the MEMORY GRID along the ray --
                   a freshly-walked area reads FREE (wall-follow leave points
                   stay reachable, the U-trap escape depends on this), while a
                   never-seen area reads UNKNOWN and unknown is NOT clear
                   (the car-wall DETOUR oscillation: turning toward the subgoal
                   swung R out of the 90-deg FOV and the empty scan window
                   counted as "clear" -- 254 re-entries at one spot)."""
        theta_body = wrap_angle(
            math.atan2(target[1] - pose[1], target[0] - pose[0]) - yaw)
        fov_lo = profile.angle_min_rad
        fov_hi = profile.angle_min_rad + (profile.n_bins - 1) * profile.angle_step_rad
        if theta_body < fov_lo - 0.05 or theta_body > fov_hi + 0.05:
            if not unseen_is_blocked:
                return False           # entry semantics: only SEEN obstacles
            # confirmed-clear semantics: walk the bearing through MEMORY.
            if self._grid is None or now is None:
                return True            # no memory -> cannot confirm -> blocked
            ang = yaw + theta_body
            c, si = math.cos(ang), math.sin(ang)
            # CORRIDOR width, not a single ray (field bug 2026-09-10: 1-tick
            # FOLLOW<->DETOUR limit cycle, 20 s head-shake). The robot's own
            # walked trail reads FREE along the exact ray while the car wall
            # sits 0.5 m beside it -- a 0-width ray "confirmed clear" through
            # a corridor the BODY cannot fit, so DETOUR exited; one tick
            # later the 0.06 rad turn put the same bearing back inside the
            # FOV and the (corridor-width) perception branch said blocked ->
            # DETOUR re-entered, forever. Same half-width as the in-FOV
            # branch: any non-FREE cell inside the swept corridor refutes.
            half_w = self._r_eff_m + 0.3
            nc, ns = -si, c            # unit normal across the bearing
            r = 0.5
            while r <= trigger_m:
                px, py = pose[0] + r * c, pose[1] + r * si
                off = -half_w
                while off <= half_w + 1e-9:
                    cell = self._grid.read(px + nc * off, py + ns * off, now)
                    if cell != Cell.FREE:   # BLOCKED or UNKNOWN: unconfirmed
                        return True
                    off += 0.25
                r += 0.5
            return False               # remembered FREE corridor: clear
        # FULL forward hemisphere, no fixed window (assembly hole #5, user
        # collision audit 2026-09-10): the old +/-15-bin window failed on a
        # narrow slot -- up close, the slot's angular span EXCEEDS the window,
        # so the window saw only through-the-slot rays and blocked never fired;
        # the robot threaded a sub-body gap on pure FOLLOW, bypassing the
        # (correct) candidate corridor gate entirely. The db <= trigger guard
        # stays: distant walls trigger naturally as the robot closes in, and
        # dropping it (an along/lateral decomposition over unlimited range)
        # proved over-sensitive -- it froze ordinary rock detours too.
        half_w = self._r_eff_m + 0.3
        for i in range(profile.n_bins):
            db = profile.d_block[i]
            if db is None or db > trigger_m:
                continue
            ang_off = abs(wrap_angle(profile.angle_min_rad
                                     + i * profile.angle_step_rad - theta_body))
            if ang_off > math.pi / 2:
                continue                      # behind the walk direction
            if db * math.sin(ang_off) < half_w:
                return True
        return False

    def _small_obstacle_escape(self, profile, pose, yaw, now):
        """Dense-field fix (2026-09-11 acceptance, path_rev death lap): the
        snake end probe said end_l=0.5 / end_r=1.0 -- BOTH ends of the
        "wall" within a meter, i.e. an isolated rock -- and wall-follow
        still hugged it, then drifted rock-to-rock through the field until
        wall_budget burned (60 m of lapping pebbles). A boundary whose two
        confirmed ends span < 3 m is NOT a wall: build a DETOUR subgoal
        just past the NEARER end instead (out along the tangent, offset
        away from the obstacle), and let the ordinary detour machinery
        (A-HYS-2 guards included) walk around it. Returns the world subgoal
        or None when this is not a small obstacle / geometry unknown."""
        if self._grid is None or now is None or profile is None:
            return None
        best_db = None
        best_bearing = 0.0
        for i, db in enumerate(profile.d_block):
            if db is not None and (best_db is None or db < best_db):
                best_db = db
                best_bearing = yaw + profile.angle_min_rad \
                    + i * profile.angle_step_rad
        if best_db is None:
            return None
        anchor = (pose[0] + best_db * math.cos(best_bearing),
                  pose[1] + best_db * math.sin(best_bearing))
        t1 = best_bearing + math.pi / 2.0
        t2 = best_bearing - math.pi / 2.0
        e1 = self._grid.wall_end_dist(anchor, t1, now, r_max_m=3.0)
        e2 = self._grid.wall_end_dist(anchor, t2, now, r_max_m=3.0)
        if e1 is None or e2 is None or e1 + e2 >= 3.0:
            return None                     # a real wall (or unconfirmed)
        tang = t1 if e1 <= e2 else t2
        end = e1 if e1 <= e2 else e2
        # subgoal: past the near end along the tangent, pushed AWAY from
        # the obstacle by the keep distance so the corner is not clipped.
        away = math.atan2(pose[1] - anchor[1], pose[0] - anchor[0])
        gx = anchor[0] + math.cos(tang) * (end + 1.2) \
            + math.cos(away) * 0.8
        gy = anchor[1] + math.sin(tang) * (end + 1.2) \
            + math.sin(away) * 0.8
        return (gx, gy)

    def _enter_wall(self, profile, pose, yaw, fs, now) -> bool:
        """S7.2 entry: all candidates gated out AND a BLOCKED boundary exists
        (perception or memory). Chooses the side (goal-side heuristic when
        neither end is visible) and opens the WallFollowState with s_hit = the
        monotone best progress s* (20 S7.3). Returns False -> caller holds."""
        if profile is None:
            return False
        boundary = any(d is not None for d in profile.d_block)
        if not boundary and self._grid is not None and now is not None:
            boundary = self._grid.nearest_blocked_in_sector(
                pose, yaw, -math.pi, math.pi, now, r_max_m=3.0,
                n_rays=12) is not None
        if not can_enter(True, boundary):
            return False
        # HAND-ON-WALL side pick from the WALL TANGENT, not the body axes
        # (field bug #5, 2026-09-10 spy-replay): "walking the wall = going
        # body-left/right" only holds when FACING the wall square-on. Entering
        # at 45 deg (post-DETOUR pose), the body-right bearing projected onto a
        # north-south wall pointed NORTH while the goal lay SOUTH -- an 18 m
        # reverse lap ending in max_deviation "one breath short" (user trace).
        # Correct frame: wall normal = bearing of the nearest blocked bin; the
        # two TANGENTS are normal +/- 90 deg; walk the tangent closer to the
        # bearing of the mission ENDPOINT (not R: facing the wall, R sits on
        # the normal and the two tangents tie), and the hand is which side the
        # wall normal falls on relative to the walk direction.
        best_db = None
        best_bearing = 0.0
        for i, db in enumerate(profile.d_block):
            if db is not None and (best_db is None or db < best_db):
                best_db = db
                best_bearing = yaw + profile.angle_min_rad \
                    + i * profile.angle_step_rad
        if best_db is None:
            return False                    # no visible wall bearing to hug
        # side pick aims at R* when guidance serves (S4A.5): the field
        # already encodes WHICH way around is globally shorter, which is
        # exactly the question the tangent tie-break is trying to answer.
        # PATH missions aim at the LOOKAHEAD on the line instead (dense-
        # field acceptance 2026-09-11): the endpoint sits tens of meters
        # down the polyline and its bearing is locally meaningless -- the
        # path_fwd run picked the 20 m south lap around a wall group
        # because the ENDPOINT lay south-east, while the line's next leg
        # was 8 m around the north end. Following the line means the side
        # pick serves the LINE.
        if self._mission.kind == MissionKind.PATH:
            goal = fs.lookahead_point
        else:
            goal = self._r_star or self._mission.endpoint
        to_goal = math.atan2(goal[1] - pose[1], goal[0] - pose[0])
        t1 = best_bearing + math.pi / 2.0
        t2 = best_bearing - math.pi / 2.0
        walk = t1 if abs(wrap_angle(to_goal - t1)) <= \
            abs(wrap_angle(to_goal - t2)) else t2
        goal_side = (Side.RIGHT if wrap_angle(best_bearing - walk) < 0
                     else Side.LEFT)
        # rules 1/2 of S7.2 (v1.21): probe BOTH tangents for the wall's end in
        # the memory grid, nearer confirmed end wins. Before this the call
        # passed sees_end=False,False and the goal-side heuristic alone chose
        # -- field bug 2026-09-10: re-entering 1 m short of the SOUTH end of a
        # 13 m car wall, the goal bore 0.2 rad north of the tangent tie-line,
        # so the pick flipped NORTH and re-walked the entire wall. The end
        # probe knows "south end 1 m, north end >8 m" and rule 1 keeps south.
        end1 = end2 = None
        if self._grid is not None and now is not None:
            anchor = (pose[0] + best_db * math.cos(best_bearing),
                      pose[1] + best_db * math.sin(best_bearing))
            end1 = self._grid.wall_end_dist(anchor, t1, now)
            end2 = self._grid.wall_end_dist(anchor, t2, now)
        # map tangents to hands: walking t, the wall normal falls on one side
        side1 = (Side.RIGHT if wrap_angle(best_bearing - t1) < 0
                 else Side.LEFT)
        if side1 == Side.LEFT:
            left_end, right_end = end1, end2
        else:
            left_end, right_end = end2, end1
        # rule-2 cost is |X->E| + |E->R'| VERBATIM from 20 S7.2 -- the
        # first cut used the end distance alone and the nearer end won even
        # when rounding it led AWAY from the objective (path_fwd 2026-09-11:
        # end_l=0.5 into a wall funnel beat end_r=5.5 toward the line; a
        # 27 m south lap followed). E = the probe end point along its
        # tangent; R' = `goal` chosen above (lookahead for PATH, R*/endpoint
        # for goto), so the detour-cost comparison serves the mission's own
        # objective.
        big = 1e9    # "no confirmed end" cost for rule-2 comparison
        def _rule2_cost(end_m, tang):
            if end_m is None:
                return big
            ex = anchor[0] + math.cos(tang) * end_m
            ey = anchor[1] + math.sin(tang) * end_m
            return end_m + math.hypot(goal[0] - ex, goal[1] - ey)
        if side1 == Side.LEFT:
            left_tang, right_tang = t1, t2
        else:
            left_tang, right_tang = t2, t1
        side = select_side(
            left_end is not None, right_end is not None,
            _rule2_cost(left_end, left_tang),
            _rule2_cost(right_end, right_tang),
            goal_side, False, False)
        if side is None:
            return False
        if self._planner is not None:
            # event replan (S4A.5): hitting a wall IS new information; the
            # next build folds it in so R* (and any re-entry side pick)
            # reflects the wall rather than the pre-collision straight line.
            self._planner.request_replan()
        self._wall = WallFollowState(side=side, s_hit=self._s_star,
                                     hit_point=(pose[0], pose[1]))
        self._wall_entry_dwell = 0
        self._last_d_side = None
        self._wall_corner = False
        self._wall_stall_ticks = 0
        g = self._mission.endpoint
        self._wall_goal_dist_at_hit = math.hypot(pose[0] - g[0], pose[1] - g[1])
        self._transition(NavState.WALL_FOLLOW)
        self.audit.append(AuditRecord(now or 0, "wall_enter",
                                      {"side": side.value,
                                       "s_hit": self._s_star,
                                       "end_l": left_end, "end_r": right_end}))
        return True

    def _wall_tick(self, profile, pose, yaw, fs, now, wz_max, step_m):
        """One WALL_FOLLOW tick (20 S7): keep-distance PD against the memory
        grid's side sector (the wall is OUT of the 90-deg FOV -- RNS-I-7),
        corner rules, the 2' arc-length leave, D3 crossing records, and the
        three failure criteria."""
        cfg = self._cfg["rns"]
        wf = cfg["wall_follow"]
        w = self._wall
        zero = VelocityCandidate(vx=Mps(0.0), vy=Mps(0.0), wz=0.0)
        # REVIEW R1-20 (3-pass audit 2026-09-11): perception dropout while
        # hugging (snap/profile None) reached the d_free/d_block derefs
        # below and CRASHED the tick. Hugging blind is undefined -- hold
        # zero this tick; the T-51 age gate governs recovery when frames
        # return, and the stall counter above stays honest because this
        # early return never runs it.
        if profile is None:
            return zero
        w.followed_m += step_m
        self._s_star = max(self._s_star, fs.projection.s_arc_m)

        # in-wall STALL backstop (field bug #3, 2026-09-10 recorder): the S7.6
        # criteria are all DISTANCE-metered (followed_m); a robot pinned in
        # place (e.g. by the host speed gate) walks zero meters, so none of
        # them can ever fire -- a silent forever-stall. Track wall-clock in the
        # wall state: no displacement progress for watchdog.window_s -> fail
        # honestly (bounded failure over silent hang, S9.0 discipline).
        # REVIEW R1-12 (3-pass audit 2026-09-11): measured by TICK COUNT,
        # not wall-clock spans. The clock version froze _wall_last_move_ms
        # through WAIT_DYNAMIC / SUSPENDED (those ticks never reach here),
        # so the first tick AFTER resume saw the whole suspension as stall
        # and false-failed instantly (estop hold > 20 s made it certain).
        # Suspended ticks do not run this body, so a counter cannot inflate.
        if step_m > 0.005:
            self._wall_stall_ticks = 0
        else:
            self._wall_stall_ticks += 1
        if self._wall_stall_ticks > int(cfg["watchdog"]["window_s"] * 20):
            self.audit.append(AuditRecord(now or 0, "wall_fail",
                                          {"reason": "stall_in_place"}))
            self._wall = None
            self._fail(NavFailure(NavFailReason.WALL_NO_PROGRESS,
                                  detail={"stall_ticks": self._wall_stall_ticks,
                                          "followed_m": w.followed_m}))
            return zero

        # D3 crossing record: near the line with positive arc gain (S7A.1).
        s_gain = fs.projection.s_arc_m - w.s_hit
        if fs.projection.deviation_m < wf["e_ok_m"] and s_gain > 0.0:
            record_crossing(w, s_gain, fs.projection.s_arc_m)

        # entry dwell (churn fix, g1g audit: 10 wall_enters at ONE s_hit):
        # enter -> leave -> re-enter each ~2 ticks when the leave criterion
        # sits on its edge. A 1-tick flip is never meaningful wall-following;
        # same S6.6 hysteresis family as the DETOUR dwell. Failure checks
        # below stay live -- the dwell only mutes the EXIT evaluation.
        self._wall_entry_dwell += 1
        dwell_ok = self._wall_entry_dwell >= \
            cfg["candidate"]["side_hold_ticks"]
        # exit (S7.3): back on line AND 2' arc progress AND goal dir open.
        # goal_open deliberately does NOT use R* (G1 sweep lesson): while
        # hugging, R* runs ALONG the wall ahead, so an R*-aimed open check is
        # almost always true and the leave rule fires every few steps -- the
        # wall walk fragments into enter/leave churn (4 mini-laps, one
        # wall_no_progress FAIL in the g1b sweep). Leaving means "I can head
        # for the REAL objective now", so the check aims at the plain
        # lookahead R on the reference line.
        goal_open = not self._ahead_blocked(
            profile, pose, yaw, fs.lookahead_point,
            cfg["dynamic"]["stop_dist_m"], unseen_is_blocked=True, now=now)
        # GOTO leave rule (design ruling 2026-09-10, 20 S7.3 v1.19): a goto
        # has no path shape worth returning to -- its reference line (anchor ->
        # goal) may cut straight THROUGH the obstacle, and the back-on-line
        # condition then drags the robot BACK along the wall to touch that dead
        # line before it may leave (user-observed: rounded the car wall with
        # the goal 7.6 m dead ahead, walked AWAY north to e<0.3, U-turned).
        # Classic Bug2 goto criterion instead: strictly closer to the goal
        # than at the hit point AND the goal direction confirmed open. PATH
        # missions keep the full three-condition rule (the line IS the task).
        g = self._mission.endpoint
        d_goal = math.hypot(pose[0] - g[0], pose[1] - g[1])
        if self._mission.kind == MissionKind.GOTO:
            # leave-point cleanliness (graze fix, final cut): leaving while a
            # remembered obstacle sits < 0.7 m off the hull hands FOLLOW a
            # corner-cutting line past an invisible (behind-FOV) corner. Hug a
            # little longer -- the in-wall vy shield walks the body off the
            # corner first -- then release.
            near = self._nearest_mem_blocked(pose, now, r_scan=0.7)
            leave_now = (goal_open and near is None and
                         d_goal < self._wall_goal_dist_at_hit
                         - wf["leave_progress_m"])
        else:
            leave_now = can_leave(fs.projection.deviation_m,
                                  fs.projection.s_arc_m, w.s_hit, goal_open,
                                  wf["e_ok_m"], wf["leave_progress_m"])
        if leave_now and not dwell_ok:
            leave_now = False
        if leave_now:
            # mute R* steering for 2 s: the fresh leave must settle on the
            # plain reference geometry before the global pull resumes, or
            # the pull re-triggers the wall it just left (g1b churn).
            self._wall_cooldown_until = (now or 0) + 2000
            self.audit.append(AuditRecord(now or 0, "wall_exit",
                                          {"s": fs.projection.s_arc_m,
                                           "followed_m": w.followed_m}))
            self._wall = None
            self._transition(NavState.FOLLOW)
            return zero    # next tick follows; this tick stops cleanly

        # failure (S7.6, D3-checked closed loop / no-progress / budget).
        dist_h = math.hypot(pose[0] - w.hit_point[0], pose[1] - w.hit_point[1])
        f = check_failure(w, dist_h, wf["min_loop_m"], wf["no_progress_m"],
                          wf["max_follow_m"], wf["leave_progress_m"])
        if f is not None:
            self.audit.append(AuditRecord(now or 0, "wall_fail",
                                          {"reason": f.reason.value}))
            self._wall = None
            self._fail(f)
            return zero

        # side-sector wall distance from MEMORY (perception U memory, S7.7).
        lo = math.radians(wf["sector_lo_deg"])
        hi = math.radians(wf["sector_hi_deg"])
        if w.side == Side.LEFT:
            a_lo, a_hi = lo, hi
        else:
            a_lo, a_hi = -hi, -lo
        d_side = None
        if self._grid is not None and now is not None:
            d_side = self._grid.nearest_blocked_in_sector(
                pose, yaw, a_lo, a_hi, now, r_max_m=4.0)

        # corners + PD law (S7.7).
        sgn = 1.0 if w.side == Side.LEFT else -1.0
        fwd = [d for i, d in enumerate(profile.d_free)
               if abs(profile.angle_min_rad + i * profile.angle_step_rad) < 0.35
               and d is not None]
        front_min = min(fwd) if fwd else profile.range_max_m
        # concave corner with ENTRY/EXIT hysteresis (debug-found limit cycle:
        # enter at front_stop, spin away, exit the instant front clears, PD
        # turns back in, re-enter -- forever. Exit only when the front is
        # CLEARLY open (2.5x), so the turn commits past the corner).
        if self._wall_corner:
            # exit factor 1.8 (was 2.5): with front_stop raised to 1.35 (host
            # gate zero-line clearance) 2.5x = 3.4 m rarely clears in corridors.
            if front_min > wf["front_stop_m"] * 1.8:
                self._wall_corner = False
            else:
                return VelocityCandidate(vx=Mps(0.0), vy=Mps(0.0),
                                         wz=-sgn * 0.6 * wz_max)
        if inner_corner_stop(front_min, wf["front_stop_m"]):
            self._wall_corner = True     # S7.7-3, hysteresis above
            return VelocityCandidate(vx=Mps(0.0), vy=Mps(0.0),
                                     wz=-sgn * 0.6 * wz_max)
        if wall_vanished(side_sector_has_blocked=(d_side is not None)):
            # convex corner (S7.7-2): the wall left the side sector. Blind
            # "curve toward the wall side" span in place forever once the wall
            # also left the 4 m sector range (behind-the-wall goal debug,
            # 2026-09-10: spun at the south end burning the whole no-progress
            # budget). The MEMORY still knows where the wall is -- steer at its
            # remembered bearing and ROUND the corner instead.
            info = None
            if self._grid is not None and now is not None:
                info = self._grid.nearest_blocked_full(pose, now, r_max_m=6.0)
            # vy shield ON THE ROUNDING RETURNS too (batch probe 2026-09-10,
            # corr->east_mid: -0.152 m overlap AT the wall's south end angle).
            # Rounding steers TOWARD the remembered corner while the corner
            # itself sits outside the +/-60 deg live fuse cone (left-rear at
            # the moment of contact) -- the memory ring shield is the only
            # layer that can see it. Same holo gating as the FOLLOW output.
            vy_r = self._body_shield_vy(pose, yaw, now) if self._holo else 0.0
            if info is not None:
                # steer at the TANGENT of the d_wall circle around the
                # remembered corner, NOT at the corner itself (batch probe
                # 2026-09-10: aiming at the corner cut inside the keep radius
                # -- 0.113 m graze at the south-end angle with the vy shield
                # already at full push). The tangent keeps the rounding
                # radius; sgn picks the tangent that leaves the corner on
                # the wall-hand side.
                d_c, bear_c = info
                off = math.asin(min(1.0, wf["d_wall_m"]
                                    / max(d_c, wf["d_wall_m"])))
                wz = align_omega(bear_c - sgn * off, yaw,
                                 cfg["route"]["k_yaw"], wz_max)
                v_c = min(0.5 * wf["v_max_mps"],
                          self._shield_vx_cap(pose, now))
                return VelocityCandidate(vx=Mps(v_c),
                                         vy=Mps(vy_r), wz=wz)
            v_c = min(0.3 * wf["v_max_mps"], self._shield_vx_cap(pose, now))
            return VelocityCandidate(vx=Mps(v_c), vy=Mps(vy_r),
                                     wz=sgn * 0.5 * wz_max)
        # CONTACT FUSE (collision audit #2, 2026-09-10): the PD keeps distance
        # off the MEMORY grid, whose 0.25 m cells read large at point-like car
        # corners -- the robot grazed a corner to -0.10 m body overlap while
        # d_side still read ~1.0. Do not bet the hull on the PD: any LIVE
        # profile hit closer than 0.75 m in the forward hemisphere stops and
        # turns away immediately.
        near_hit = None
        for i, db in enumerate(profile.d_block):
            if db is not None and db < 0.75 and (near_hit is None
                                                 or db < near_hit):
                near_hit = db
        if near_hit is not None:
            return VelocityCandidate(vx=Mps(0.0), vy=Mps(0.0),
                                     wz=-sgn * 0.6 * wz_max)
        # APPROACH phase (weave fix, user "not decisive, keeps circling"
        # 2026-09-10 debug: entered wall-follow 3.5 m off the wall, then the
        # keep-distance PD and the corner hysteresis fought each other all the
        # way down the wall -- an S-weave that also burned the 40 m
        # no-progress budget). Far off the wall, do NOT weave: steer straight
        # AT the wall (P law on the nearest-hit bearing) and drive; switch to
        # the PD only inside the keep-distance band.
        if d_side > 1.8 * wf["d_wall_m"]:
            best_db = None
            best_bear = yaw
            for i, db in enumerate(profile.d_block):
                if db is not None and (best_db is None or db < best_db):
                    best_db = db
                    best_bear = yaw + profile.angle_min_rad \
                        + i * profile.angle_step_rad
            wz = align_omega(best_bear, yaw, cfg["route"]["k_yaw"], wz_max)
            self._last_d_side = d_side
            v_a = min(wf["v_max_mps"], self._shield_vx_cap(pose, now))
            return VelocityCandidate(vx=Mps(v_a), vy=Mps(0.0),
                                     wz=wz)
        rate = 0.0
        if self._last_d_side is not None:
            rate = (d_side - self._last_d_side) / max(0.02, 0.05)
        self._last_d_side = d_side
        wz = keep_distance_omega(d_side, wf["d_wall_m"], rate, w.side,
                                 wf["kp_wall"], wf["kd_wall"], wz_max)
        # far from the wall: cap the turn-IN rate so approach is a forward
        # spiral, not an in-place spin toward the wall (the other half of the
        # limit cycle). Toward-wall for LEFT is wz>0; for RIGHT wz<0.
        if d_side > 2.0 * wf["d_wall_m"]:
            cap = 0.35 * wz_max
            if sgn > 0:
                wz = min(wz, cap)
            else:
                wz = max(wz, -cap)
        # vy shield in-wall too: at point-like corners the sector read goes
        # unstable and the PD grazes in; the memory ring-scan still sees the
        # corner and side-steps off it (graze debug t902-914).
        vy_w = 0.0
        if self._holo:
            vy_w = self._body_shield_vy(pose, yaw, now)
        # shield vx cap (east->gap_s 0.113 m): full wall speed through a
        # corner lets the PD lag eat the keep distance; a remembered BLOCKED
        # inside the body ring slows the pass. Not holo-gated -- slowing
        # helps a tracked chassis exactly the same.
        v_w = min(wf["v_max_mps"], self._shield_vx_cap(pose, now))
        return VelocityCandidate(vx=Mps(v_w), vy=Mps(vy_w), wz=wz)

    def _pick_margin_ladder(self, profile, pose, yaw, fs, cfg, margin,
                            dyn_objs=()):
        """Margin DEGRADATION ladder (dense-field acceptance 2026-09-11):
        the class-blind profile forces the max class margin (0.6, the car
        fix) -- which walls off every 1.5-2 m gap in a rock field and
        degrades the run into wall-hugging laps around pebbles (baseline:
        both path missions FAILED with 50-60 percent off-line time; three
        goto FAILs). 20 S5.6 margins ARE per-class; blindness argues for
        trying conservative FIRST, then stepping down toward the small-
        object floor (0.2) -- never below it, and U54's 1 m person keep-out
        is untouched (persons ride the DYNAMIC pile, not this gate). The
        gap-tightness cap (S8.1A, wired R2-3) slows the squeeze exactly as
        designed: narrow gap => creep through, not lap around."""
        floors = sorted({margin,
                         cfg["clearance"]["margin_by_class"]["unknown_geom"],
                         cfg["clearance"]["margin_by_class"]["small_object"]},
                        reverse=True)
        for m in floors:
            best = self._pick_candidate(profile, pose, yaw, fs, cfg, m,
                                        dyn_objs)
            if best is not None:
                return best
        return None

    def _pick_candidate(self, profile, pose, yaw, fs, cfg, margin,
                        dyn_objs=(), memory_appeal=False):
        """Build candidates from the profile, gate them, score them, return the
        best feasible (body frame) or None. dyn_objs: world (x, y, r) of the
        DYNAMIC pile -- a candidate whose X->S corridor one of them occupies is
        gated out (A-GAP-6; assembly hole #4: this field was hardwired False,
        so detour candidates could aim AROUND a person -- A-CLS-1 forbids).

        memory_appeal (hole #6, LAYERED design 2026-09-10): when True, a
        candidate rejected only for being out-of-FOV (in_unobserved) may be
        rehabilitated if the memory grid confirms the X->S corridor FREE
        (RNS-I-6 is perception UNION memory). Kept OFF in the normal pick --
        the first cut mixed appealed candidates into the regular contest and
        the cost ordering went haywire (all car-wall regressions blew up:
        walked-ground candidates outscored forward ones and the robot doubled
        back). As a SEPARATE rescue layer between "all candidates gated" and
        "enter wall-follow", the normal behavior is untouched by construction:
        first-encounter ticks have no memory, so the layer is inert there."""
        cand_cfg = cfg["candidate"]
        clear_m = clear_extrapolation(self._r_eff_m, margin,
                                      cand_cfg["subgoal_extra_m"])
        cands = candidates_from_profile(
            profile.d_block, profile.d_free, profile.angle_min_rad,
            profile.angle_step_rad, profile.range_max_m,
            cand_cfg["edge_jump_m"], clear_m)
        # angle costs aim at R* when guidance serves (S4A.5), else at R --
        # this is what makes candidate selection prefer the globally right
        # detour side instead of the straight-at-goal cone.
        aim = self._r_star or fs.lookahead_point
        r_body = self._world_to_body(aim, pose, yaw)
        w = cfg["cost_weights"]
        best = None
        best_cost = math.inf
        for c in cands:
            if memory_appeal and c.in_unobserved \
                    and self._grid is not None:
                sw = self._body_to_world(c.subgoal, pose, yaw)
                dx, dy = sw[0] - pose[0], sw[1] - pose[1]
                dist = math.hypot(dx, dy)
                if dist > 1e-6 and self._last_now is not None:
                    steps = max(2, int(dist / 0.3))
                    if all(self._grid.read(pose[0] + dx * k / steps,
                                           pose[1] + dy * k / steps,
                                           self._last_now) == Cell.FREE
                           for k in range(1, steps + 1)):
                        import dataclasses
                        c = dataclasses.replace(c, in_unobserved=False)
            if dyn_objs:
                s_world = self._body_to_world(c.subgoal, pose, yaw)
                if any(in_corridor(ox, oy, orad, pose[0], pose[1],
                                   s_world[0], s_world[1],
                                   self._r_eff_m + margin)
                       for ox, oy, orad in dyn_objs):
                    continue            # A-GAP-6: dynamic-occupied candidate
            if not passes_hard_gates(c, self._r_eff_m, margin):
                continue
            from .candidate import unknown_ratio_toward
            unk = unknown_ratio_toward(c.subgoal, profile.d_free,
                                       profile.angle_min_rad,
                                       profile.angle_step_rad)
            cost = candidate_cost(
                c, (0.0, 0.0), r_body, unk, False,
                r_eff_m=self._r_eff_m, margin_m=margin,
                gate_saturate_ratio=cfg["clearance"]["gate_saturate_ratio"],
                w_clr=w["clearance"], w_ang=w["angle"], w_len=w["length"],
                w_unk=w["unknown"], w_hys=w["hysteresis"])
            if cost < best_cost:
                best_cost = cost
                best = c
        return best

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
