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
from .classify import behavior_class, health_speed_capped, usable_velocity
from .config import RnsConfigError, run_startup_assertions
from .dynamic import DynamicAction, WaitBudget, distance_action, in_corridor
from .grid import (MemoryGrid, profile_age_ms, profile_speed_limited,
                   profile_zero_speed, seg_stale)
from .wallfollow import (Side, WallFollowState, can_enter, can_leave,
                         check_failure, inner_corner_stop, keep_distance_omega,
                         record_crossing, select_side, wall_vanished)
from .watchdog import ProgressWatchdog, WatchdogResult, no_progress_failure
from .audit import AuditRecord, RingAudit
from .route import Mission, align_omega, lookahead_distance, wrap_angle
from .speed import SpeedCaps, cap_deviation
from .types import (MissionKind, NavFailure, NavState, Origin,
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
        self.audit = RingAudit(capacity=256)
        if cfg is not None:
            c = cfg["rns"]
            self._selector = CandidateSelector(c["candidate"]["side_hold_ticks"])
            self._wait_budget = WaitBudget(c["dynamic"]["wait_budget_s"])
            m = c["memory"]
            self._grid = MemoryGrid(m["cell_m"], m["ttl_static_s"],
                                    m["ttl_dynamic_s"], m["reset_jump_m"])
            w = c["watchdog"]
            self._watchdog = ProgressWatchdog(int(w["window_s"] * 1000),
                                              w["min_progress_m"])

    def load_mission(self, mission: Mission) -> None:
        """P3/route pushes a mission (goto or path). Transitions IDLE -> FOLLOW.
        The mission carries origin (route|relmove) for report routing."""
        self._mission = mission
        self._state = NavState.FOLLOW
        self._reporter.reset_for_new_mission()
        self._subgoal_world = None
        self._dyn_prev = DynamicAction.RUN
        self._wall = None
        self._s_star = float("-inf")
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
        if fs.arrived:
            self._arrived_pending = True
            self.clear_mission()
            return zero
        dev_fail = self._mission.deviation_failure(fs)
        if dev_fail is not None:                     # P1.5 wired (A-DEV-1)
            self._fail(dev_fail)
            return zero

        # ── B2. memory grid + progress bookkeeping (S2b) ─────────────────────
        if profile is not None and now is not None and self._grid is not None:
            self._grid.on_pose(pose)
            self._grid.ingest_profile(profile, pose, yaw, now)
        self._s_star = max(self._s_star, fs.projection.s_arc_m)
        step_m = 0.0
        if self._last_pose is not None:
            step_m = math.hypot(pose[0] - self._last_pose[0],
                                pose[1] - self._last_pose[1])
        dt_ms = (now - self._last_now) if (now is not None and
                                           self._last_now is not None) else 50
        self._last_pose = pose
        self._last_now = now

        # target: the subgoal while detouring, else the lookahead R.
        if self._state == NavState.DETOUR and self._subgoal_world is not None:
            target = self._subgoal_world
        else:
            target = fs.lookahead_point

        # ── C. dynamic obstacles (20 S5) ─────────────────────────────────────
        half_w = self._r_eff_m + 0.5    # corridor half-width: r_eff + margin
        blocking = None
        if snap is not None and snap.objects is not None and now is not None:
            dyn_cfg = cfg["dynamic"]
            stopped = slowed = False
            cy, sy = math.cos(yaw), math.sin(yaw)
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
                from .classify import dispatch_dynamic as _dd
                if not _dd(beh, eff_spd, dwell_s,
                           dyn_cfg["v_static_thresh_mps"],
                           dyn_cfg["t_static_dwell_s"]):
                    continue        # static pile: geometry (profile) covers it
                # body-frame centroid -> world; corridor test against target.
                fp = obj.footprint_xy
                bx = sum(q[0] for q in fp) / len(fp)
                by = sum(q[1] for q in fp) / len(fp)
                ox = pose[0] + bx * cy - by * sy
                oy = pose[1] + bx * sy + by * cy
                r_obs = max(math.hypot(q[0] - bx, q[1] - by) for q in fp)
                if in_corridor(ox, oy, r_obs, pose[0], pose[1],
                               target[0], target[1], half_w):
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

        # ── C3. progress watchdog (S2b -- 20 S7.3A, RNS-N-15) ────────────────
        if self._watchdog is not None and profile is not None:
            boundary = any(d is not None for d in profile.d_block)
            wd = self._watchdog.tick(
                self._s_star, dt_ms, self._state.value,
                self._state == NavState.WAIT_DYNAMIC, None, boundary)
            if wd == WatchdogResult.ESCALATE_WALL:
                if self._enter_wall(profile, pose, yaw, fs, now):
                    return self._wall_tick(profile, pose, yaw, fs, now,
                                           wz_max, step_m)
            elif wd == WatchdogResult.REPORT_NO_PROGRESS:
                self._fail(no_progress_failure("geometry"))
                return zero

        # ── D. static avoidance state machine (20 S6) ────────────────────────
        if profile is not None:
            margin = cfg["clearance"]["margin_by_class"]["structure"]
            gate_base = self._r_eff_m + margin
            blocked = self._ahead_blocked(profile, pose, yaw, target,
                                          cfg["dynamic"]["stop_dist_m"])
            if self._state == NavState.FOLLOW and blocked:
                best = self._pick_candidate(profile, pose, yaw, fs, cfg, margin)
                sel = self._selector.select(False, best, best_is_current=False)
                if sel is not None:
                    self._subgoal_world = self._body_to_world(
                        sel.subgoal, pose, yaw)
                    self._transition(NavState.DETOUR)
                    target = self._subgoal_world
                    self.audit.append(AuditRecord(now or 0, "detour_enter",
                                                  {"subgoal": target}))
                else:
                    # S2b: no feasible candidate -- the S7.2 entry: hug the wall
                    # if there is one; else hold and let the watchdog decide.
                    if self._enter_wall(profile, pose, yaw, fs, now):
                        return self._wall_tick(profile, pose, yaw, fs, now,
                                               wz_max, step_m)
                    return zero
            elif self._state == NavState.DETOUR:
                d_sub = math.hypot(pose[0] - self._subgoal_world[0],
                                   pose[1] - self._subgoal_world[1])
                r_clear = not self._ahead_blocked(
                    profile, pose, yaw, fs.lookahead_point,
                    cfg["dynamic"]["stop_dist_m"])
                if d_sub < 0.7 or r_clear:
                    self._subgoal_world = None
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
                                                    cfg, margin)
                        sel = self._selector.select(False, best, False)
                        if sel is None:
                            self._subgoal_world = None
                            if self._enter_wall(profile, pose, yaw, fs, now):
                                return self._wall_tick(profile, pose, yaw, fs,
                                                       now, wz_max, step_m)
                            self._transition(NavState.FOLLOW)
                            return zero
                        self._subgoal_world = self._body_to_world(
                            sel.subgoal, pose, yaw)
                        target = self._subgoal_world

        # ── E. speed + heading toward target ─────────────────────────────────
        caps["deviation"] = cap_deviation(
            fs.projection.deviation_m, v_nom, cfg["speed"]["dev_e0_m"],
            route_cfg["max_deviation_m"], cfg["speed"]["dev_g_min"])
        theta_des = math.atan2(target[1] - pose[1], target[0] - pose[0])
        wz = align_omega(theta_des, yaw, route_cfg["k_yaw"], wz_max)
        v = SpeedCaps(caps).limit()
        # large heading error: turn before driving (keeps arcs off obstacles;
        # the fine speed gate f(d_free) is the HOST's job, 12 S6.2).
        err = abs(wrap_angle(theta_des - yaw))
        if err > 1.0:
            v = min(v, 0.2 * v_nom)
        return VelocityCandidate(vx=Mps(v), vy=Mps(0.0), wz=wz)

    # ── assembly helpers ─────────────────────────────────────────────────────
    @staticmethod
    def _body_to_world(p, pose, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return (pose[0] + p[0] * c - p[1] * s, pose[1] + p[0] * s + p[1] * c)

    @staticmethod
    def _world_to_body(p, pose, yaw):
        c, s = math.cos(-yaw), math.sin(-yaw)
        dx, dy = p[0] - pose[0], p[1] - pose[1]
        return (dx * c - dy * s, dx * s + dy * c)

    def _ahead_blocked(self, profile, pose, yaw, target, trigger_m) -> bool:
        """Is the bearing toward `target` blocked within trigger_m? Checks the
        profile bins in a corridor-width angular window around that bearing."""
        theta_body = wrap_angle(
            math.atan2(target[1] - pose[1], target[0] - pose[0]) - yaw)
        half_w = self._r_eff_m + 0.3
        n = profile.n_bins
        center = round((theta_body - profile.angle_min_rad)
                       / profile.angle_step_rad)
        for i in range(max(0, center - 15), min(n, center + 16)):
            db = profile.d_block[i]
            if db is None or db > trigger_m:
                continue
            # lateral offset of that hit from the target bearing at range db
            ang_off = abs(wrap_angle(profile.angle_min_rad
                                     + i * profile.angle_step_rad - theta_body))
            if db * math.sin(ang_off) < half_w:
                return True
        return False

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
        r_body = self._world_to_body(fs.lookahead_point, pose, yaw)
        goal_side = Side.LEFT if r_body[1] > 0 else Side.RIGHT
        side = select_side(False, False, 0.0, 0.0, goal_side, False, False)
        if side is None:
            return False
        self._wall = WallFollowState(side=side, s_hit=self._s_star,
                                     hit_point=(pose[0], pose[1]))
        self._last_d_side = None
        self._wall_corner = False
        self._transition(NavState.WALL_FOLLOW)
        self.audit.append(AuditRecord(now or 0, "wall_enter",
                                      {"side": side.value,
                                       "s_hit": self._s_star}))
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
        w.followed_m += step_m
        self._s_star = max(self._s_star, fs.projection.s_arc_m)

        # D3 crossing record: near the line with positive arc gain (S7A.1).
        s_gain = fs.projection.s_arc_m - w.s_hit
        if fs.projection.deviation_m < wf["e_ok_m"] and s_gain > 0.0:
            record_crossing(w, s_gain, fs.projection.s_arc_m)

        # exit (S7.3): back on line AND 2' arc progress AND goal dir open.
        goal_open = not self._ahead_blocked(
            profile, pose, yaw, fs.lookahead_point,
            cfg["dynamic"]["stop_dist_m"])
        if can_leave(fs.projection.deviation_m, fs.projection.s_arc_m,
                     w.s_hit, goal_open, wf["e_ok_m"], wf["leave_progress_m"]):
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
            if front_min > wf["front_stop_m"] * 2.5:
                self._wall_corner = False
            else:
                return VelocityCandidate(vx=Mps(0.0), vy=Mps(0.0),
                                         wz=-sgn * 0.6 * wz_max)
        if inner_corner_stop(front_min, wf["front_stop_m"]):
            self._wall_corner = True     # S7.7-3, hysteresis above
            return VelocityCandidate(vx=Mps(0.0), vy=Mps(0.0),
                                     wz=-sgn * 0.6 * wz_max)
        if wall_vanished(side_sector_has_blocked=(d_side is not None)):
            # convex corner (S7.7-2): the wall left the sector -- curve gently
            # TOWARD the wall side to re-acquire it, slow.
            return VelocityCandidate(vx=Mps(0.3 * wf["v_max_mps"]), vy=Mps(0.0),
                                     wz=sgn * 0.5 * wz_max)
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
        return VelocityCandidate(vx=Mps(wf["v_max_mps"]), vy=Mps(0.0), wz=wz)

    def _pick_candidate(self, profile, pose, yaw, fs, cfg, margin):
        """Build candidates from the profile, gate them, score them, return the
        best feasible (body frame) or None."""
        cand_cfg = cfg["candidate"]
        clear_m = clear_extrapolation(self._r_eff_m, margin,
                                      cand_cfg["subgoal_extra_m"])
        cands = candidates_from_profile(
            profile.d_block, profile.d_free, profile.angle_min_rad,
            profile.angle_step_rad, profile.range_max_m,
            cand_cfg["edge_jump_m"], clear_m)
        r_body = self._world_to_body(fs.lookahead_point, pose, yaw)
        w = cfg["cost_weights"]
        best = None
        best_cost = math.inf
        for c in cands:
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
