"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: planner.py
Brief: Memory-guided planning layer -- budgeted goal-rooted search, R* guide (20 S4A)

Description:
The guidance layer of 20 S4A (v1.24): an incremental pathfinder over a COARSE
view of the memory grid that answers one question each tick -- "which direction
does the globally shortest path leave my position?" -- as a guide point R*.
Without it every direction decision in RNS (candidate scoring, wall side pick,
leave gates) argues from local evidence only (90 deg FOV + goal-bearing
heuristics), which the 2026-09-10 night sweep proved picks the long way around
whenever the better detour lies outside the FOV.

What this file DOES:
  - Coarse view: aggregates 2x2 memory cells (0.25 m) into one 0.5 m planning
    cell, ON DEMAND during search expansion (never a bulk snapshot -- the
    aggregation cost rides inside the search budget).
  - Goal-rooted budgeted Dijkstra (G1): the wavefront grows FROM THE GOAL, a
    few milliseconds per tick, so once the field reaches the robot every later
    R* query is O(1) and robot motion costs nothing. Double-buffered: queries
    are served by the last COMPLETED field while the next one builds.
  - Dual-mode (S4A.3, FAR-style): a known-mode pass (FREE cells only) runs
    first when the goal cell is observed FREE; if it cannot reach the robot,
    an attempt-mode pass (UNKNOWN traversable at kappa x cost) follows. The
    optimism belongs to THIS layer only -- the reactive layer keeps treating
    UNKNOWN conservatively (two-layer semantics, S4A.2).
  - R*: walk the descent field guide_lookahead_m from the robot cell; return
    that point. None whenever the field does not cover the robot yet -- the
    caller falls back to the plain lookahead R (behavior == v1.0, no switch).

What this file does NOT do (boundaries, per S4A):
  - No velocity, no clearance, no contact decisions -- direction only.
  - No persistence: the domain lives for one task (S4A.4 lifecycle ruling);
    pose-jump grid clears (A-MEM-3) empty the coarse view implicitly because
    aggregation reads the live grid.
  - No thread: the budget loop runs inside compute() (RNS-M-4/M-5).

Traps encoded here (do not re-learn):
  - The wavefront must be GOAL-rooted, not robot-rooted: a robot-rooted field
    invalidates on every step of self-motion; goal-rooted survives until the
    WORLD changes (the D* Lite direction choice, [11]).
  - Diagonal steps must cost sqrt(2), not 1 -- a unit-cost 8-connected grid
    zigzags and R* oscillates laterally on open ground.
  - A goal inside an obstacle (memory says BLOCKED) still deserves guidance:
    the field roots at the goal cell regardless; the reactive layer owns the
    final approach and arrival radius handles the last meter.
"""

from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Tuple

from .types import Cell

# 8-connected neighborhood with true metric step costs (see header trap note).
_NBRS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
         (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)))


class GuidancePlanner:
    """Budgeted goal-rooted planner over the memory grid's coarse view.

    Usage from the source: set_task() on mission load, on_tick() every compute
    tick (advances the budgeted search; restarts it on period or when the
    served field is stale), guide_point() for R*, clear() on terminal."""

    def __init__(self, cfg: dict) -> None:
        g = cfg["guidance"]
        self._coarse_m = float(g["coarse_cell_m"])
        self._margin_m = float(g["domain_margin_m"])
        self._cap_m = float(g["domain_cap_m"])
        self._kappa = float(g["unknown_cost_factor"])
        self._budget_nodes = int(g["nodes_per_tick"])
        self._replan_period_ms = int(float(g["replan_period_s"]) * 1000)
        self._guide_ahead_m = float(g["guide_lookahead_m"])
        self._goal: Optional[Tuple[float, float]] = None
        # domain bounds (world, coarse-cell aligned): (min_x, min_y, nx, ny)
        self._dom: Optional[Tuple[float, float, int, int]] = None
        # served (front) field: cell -> cost-to-goal; and the build (back) side
        self._field: Optional[Dict[Tuple[int, int], float]] = None
        self._field_mode: str = "none"          # "known" | "attempt" | "none"
        # REVIEW R1-21 (3-pass audit 2026-09-11): the SERVED field must carry
        # the domain it was built in. on_tick re-anchors self._dom around the
        # moving pose at every rebuild, so cell ids from the NEW domain were
        # used to index the OLD field -- a silent lattice shift that bent R*
        # (masked by the reactive layer, but the guide was wrong).
        self._field_dom: Optional[Tuple[float, float, int, int]] = None
        self._build: Optional[Dict[Tuple[int, int], float]] = None
        self._open: List[Tuple[float, Tuple[int, int]]] = []
        self._build_mode: str = "none"
        self._coarse_cache: Dict[Tuple[int, int], Cell] = {}
        self._last_build_start_ms: Optional[int] = None
        # task clock for the min-try window; ONLY set_task assigned it
        # before (latent AttributeError on any query-before-task path --
        # surfaced by the goto-only gate, where path missions clear()).
        self._task_start_ms: Optional[int] = None
        # G2: consecutive attempt-mode builds that could NOT reach the robot
        # (or its 8-neighborhood). Two in a row -- with a fresh grid read in
        # between -- is the domain no-path proof (S4A.3).
        self._unreachable_builds = 0
        self._last_chain: List[Tuple[int, int]] = []
        # G3 static layer (S4A.4): fence/prior-map polygons rasterized to
        # coarse WORLD cells (independent of the task domain). Consumed by
        # _coarse_state as BLOCKED-dominant. RNS only CONSUMES this -- the
        # polygons come from the curated map side (geo/fence.db, 20 S4A.4
        # lifecycle table); nothing here persists anything.
        self._static_blocked: set = set()

    # ── task lifecycle (S4A.4: task-scoped, never persisted) ─────────────────
    def set_task(self, start_xy, goal_xy) -> None:
        self._goal = (float(goal_xy[0]), float(goal_xy[1]))
        self._make_domain(start_xy, goal_xy)
        self._field = None
        self._field_mode = "none"
        self._build = None
        self._open = []
        self._build_mode = "none"
        self._last_build_start_ms = None
        # verdict state is STRICTLY per-task (user ruling 2026-09-11: every
        # order must TRY; a failure belongs to its own order only). The
        # field bug: these two survived set_task, so one no-path verdict
        # made every later order fail AT TICK ONE -- "the navigator stopped
        # working". Nothing of a past order's verdict may leak forward.
        self._unreachable_builds = 0
        self._last_chain = []
        self._task_start_ms: Optional[int] = None

    def set_static_polygons(self, polygons) -> None:
        """Rasterize keep-out polygons into the static BLOCKED layer
        (world-anchored coarse cells; survives set_task/clear -- the static
        world does not die with a mission). polygons: iterable of vertex
        lists [(x, y), ...]. Point-in-polygon by ray casting at coarse-cell
        centers over each polygon's bounding box."""
        self._static_blocked = set()
        for poly in polygons:
            if len(poly) < 3:
                continue
            xs = [v[0] for v in poly]
            ys = [v[1] for v in poly]
            x = min(xs)
            while x <= max(xs) + 1e-9:
                y = min(ys)
                while y <= max(ys) + 1e-9:
                    if self._point_in_poly(x, y, poly):
                        self._static_blocked.add(
                            (int(x / self._coarse_m),
                             int(y / self._coarse_m)))
                    y += self._coarse_m
                x += self._coarse_m

    @staticmethod
    def _point_in_poly(x, y, poly) -> bool:
        inside = False
        j = len(poly) - 1
        for i in range(len(poly)):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if (yi > y) != (yj > y) and \
                    x < (xj - xi) * (y - yi) / (yj - yi) + xi:
                inside = not inside
            j = i
        return inside

    def clear(self) -> None:
        self._goal = None
        self._task_start_ms = None
        self._dom = None
        self._field = None
        self._field_mode = "none"
        self._build = None
        self._open = []
        self._build_mode = "none"
        self._coarse_cache = {}

    def _make_domain(self, a, b) -> None:
        # task bounding box + margin, side-capped (S4A.5). Cap keeps the cell
        # count bounded (worst 120x120 at 0.5 m) so a single build's total
        # work is a known quantity spread over ticks.
        lo_x = min(a[0], b[0]) - self._margin_m
        hi_x = max(a[0], b[0]) + self._margin_m
        lo_y = min(a[1], b[1]) - self._margin_m
        hi_y = max(a[1], b[1]) + self._margin_m
        cx, cy = (lo_x + hi_x) / 2.0, (lo_y + hi_y) / 2.0
        w = min(hi_x - lo_x, self._cap_m)
        h = min(hi_y - lo_y, self._cap_m)
        lo_x, lo_y = cx - w / 2.0, cy - h / 2.0
        nx = max(1, int(w / self._coarse_m))
        ny = max(1, int(h / self._coarse_m))
        self._dom = (lo_x, lo_y, nx, ny)

    # ── coarse view (2x2 aggregation, on demand) ─────────────────────────────
    def _cell_of(self, xy, dom=None) -> Optional[Tuple[int, int]]:
        dom = dom if dom is not None else self._dom
        if dom is None:
            return None
        lo_x, lo_y, nx, ny = dom
        ix = int((xy[0] - lo_x) / self._coarse_m)
        iy = int((xy[1] - lo_y) / self._coarse_m)
        if 0 <= ix < nx and 0 <= iy < ny:
            return (ix, iy)
        return None

    def _center_of(self, cell, dom=None) -> Tuple[float, float]:
        dom = dom if dom is not None else self._dom
        lo_x, lo_y, _, _ = dom
        return (lo_x + (cell[0] + 0.5) * self._coarse_m,
                lo_y + (cell[1] + 0.5) * self._coarse_m)

    def _coarse_state(self, cell, grid, now_ms) -> Cell:
        """BLOCKED dominates, FREE needs a majority, else UNKNOWN (S4A.5).
        Cached per build (cache dropped when a build starts) so each coarse
        cell reads its 4 fine cells at most once per replan."""
        hit = self._coarse_cache.get(cell)
        if hit is not None:
            return hit
        cx, cy = self._center_of(cell)
        if self._static_blocked and (
                int(cx / self._coarse_m),
                int(cy / self._coarse_m)) in self._static_blocked:
            self._coarse_cache[cell] = Cell.BLOCKED
            return Cell.BLOCKED
        q = self._coarse_m / 4.0
        n_free = 0
        state = Cell.UNKNOWN
        for dx, dy in ((-q, -q), (-q, q), (q, -q), (q, q)):
            st = grid.read(cx + dx, cy + dy, now_ms)
            if st == Cell.BLOCKED:
                state = Cell.BLOCKED
                break
            if st == Cell.FREE:
                n_free += 1
        else:
            state = Cell.FREE if n_free >= 3 else Cell.UNKNOWN
        self._coarse_cache[cell] = state
        return state

    # ── budgeted goal-rooted Dijkstra ────────────────────────────────────────
    def _start_build(self, mode: str, now_ms: int) -> None:
        goal_cell = self._cell_of(self._goal)
        self._build = {}
        self._open = []
        self._build_mode = mode
        self._coarse_cache = {}
        self._last_build_start_ms = now_ms
        if goal_cell is not None:
            self._build[goal_cell] = 0.0
            heapq.heappush(self._open, (0.0, goal_cell))

    def _step_cost(self, state: Cell, base: float) -> Optional[float]:
        """known mode: FREE only. attempt mode: UNKNOWN at kappa x (S4A.3).
        BLOCKED impassable in both -- optimism never walks through a wall."""
        if state == Cell.BLOCKED:
            return None
        if state == Cell.FREE:
            return base
        if self._build_mode == "attempt":
            return base * self._kappa
        return None

    def _inflate(self, cell, grid, now_ms) -> float:
        """Soft obstacle inflation (the Nav2 inflation-layer idea): a cell
        whose 8-neighborhood holds a BLOCKED cell costs 4x. Without it the
        shortest path HUGS obstacle corners -- the first G1 sweep read body
        clearance 0.023 m (sw->hug_wall_e) because a 0.5 m cell center can
        sit 0.25 m off a wall face. Guidance-only: the reactive layer's
        clearances are untouched; this just makes R* prefer the middle of
        gaps over their edges."""
        for dx, dy, _ in _NBRS:
            nb = (cell[0] + dx, cell[1] + dy)
            if not (0 <= nb[0] < self._dom[2] and 0 <= nb[1] < self._dom[3]):
                continue
            if self._coarse_cache.get(nb) == Cell.BLOCKED or \
                    self._coarse_state(nb, grid, now_ms) == Cell.BLOCKED:
                return 4.0
        return 1.0

    def domain_no_path(self, now_ms: int) -> bool:
        """G2 (S4A.3): True when two consecutive ATTEMPT builds -- a grid
        refresh apart -- failed to reach the robot('s neighborhood), AND the
        task has been TRYING for at least the min-try window (user ruling
        2026-09-11: navigation always walks first; a verdict 2.2 s after
        load -- before the memory had even warmed -- froze the whole
        navigator). The remembered BLOCKED set provably separates robot
        from goal: report no_path_in_domain, do not keep circling. mutant:
        always False -> the sealed-goal scene times out -> reddens."""
        if self._task_start_ms is None \
                or now_ms - self._task_start_ms < 10000:
            return False
        return self._unreachable_builds >= 2

    def chain_cut(self, grid, now_ms: int) -> bool:
        """G2 event trigger: a cell on the SERVED descent chain turned
        BLOCKED (fresh grid read, bypassing the build-scoped cache). The
        period gate alone leaves up to replan_period_s of steering into a
        newly-seen wall."""
        if self._field_dom is None:
            return False
        for cell in self._last_chain:
            cx, cy = self._center_of(cell, self._field_dom)
            q = self._coarse_m / 4.0
            for dx, dy in ((-q, -q), (-q, q), (q, -q), (q, q)):
                if grid.read(cx + dx, cy + dy, now_ms) == Cell.BLOCKED:
                    return True
        return False

    def request_replan(self) -> None:
        """Force the next on_tick to start a fresh build (event-driven
        replan: wall entry, discovered blockage). Cheap -- just clears the
        period gate; the budgeted build machinery is unchanged."""
        self._last_build_start_ms = None

    def on_tick(self, grid, pose_xy, now_ms: int) -> None:
        """Advance the build within budget; (re)start it when due. Serving
        (guide_point) always uses the last COMPLETED field."""
        if self._goal is None or self._dom is None or grid is None:
            return
        if self._task_start_ms is None:
            self._task_start_ms = now_ms
        if self._build is None:
            due = (self._last_build_start_ms is None
                   or now_ms - self._last_build_start_ms
                   >= self._replan_period_ms)
            if due:
                # domain refresh keeps the CURRENT pose inside (a detour can
                # exceed the original margin); known-first iff the goal cell
                # is observed FREE, else straight to attempt (S4A.3).
                self._make_domain(pose_xy, self._goal)
                goal_cell = self._cell_of(self._goal)
                mode = "attempt"
                if goal_cell is not None and \
                        self._coarse_state_fresh(goal_cell, grid,
                                                 now_ms) == Cell.FREE:
                    mode = "known"
                self._start_build(mode, now_ms)
            else:
                return
        # DETERMINISTIC budget: a fixed node quota per tick, never wall
        # time. A monotonic-clock budget made the whole layer's timing (and
        # therefore trajectories) depend on host load -- the same scenario
        # read body clearance 0.048 in one run and 0.360 in the next (g1h),
        # which poisons every regression number and would make field
        # behavior irreproducible. ~400 expansions is a few ms of work.
        quota = self._budget_nodes
        opened = self._open
        build = self._build
        robot_cell = self._cell_of(pose_xy)
        while opened:
            if quota <= 0:
                return                       # budget out; resume next tick
            quota -= 1
            cost, cell = heapq.heappop(opened)
            if cost > build.get(cell, math.inf):
                continue                     # stale heap entry
            if cell == robot_cell:
                # EARLY STOP: the wavefront reached the robot -- the descent
                # from here to the goal is now optimal, and that is the only
                # part R*/the polyline read. Finishing the whole domain
                # multiplies build time several-fold for cells nobody asks;
                # a fast field is what lets a post-collision replan overtake
                # events (the g1f south->car_back lesson: the correct detour
                # existed but arrived after wall-follow had taken over).
                opened.clear()
                break
            for dx, dy, w in _NBRS:
                nb = (cell[0] + dx, cell[1] + dy)
                if not (0 <= nb[0] < self._dom[2]
                        and 0 <= nb[1] < self._dom[3]):
                    continue
                sc = self._step_cost(self._coarse_state(nb, grid, now_ms),
                                     w * self._coarse_m)
                if sc is None:
                    continue
                if dx != 0 and dy != 0:
                    # NO corner cutting: a diagonal step needs BOTH orthogonal
                    # neighbors passable. Two BLOCKED cells touching at a
                    # corner otherwise leak the wavefront through the seam --
                    # the sealed-box no-path proof never completed because
                    # the field escaped through every box corner (G2 debug).
                    o1 = self._coarse_state((cell[0] + dx, cell[1]),
                                            grid, now_ms)
                    o2 = self._coarse_state((cell[0], cell[1] + dy),
                                            grid, now_ms)
                    if o1 == Cell.BLOCKED or o2 == Cell.BLOCKED:
                        continue
                nc = cost + sc * self._inflate(nb, grid, now_ms)
                if nc < build.get(nb, math.inf):
                    build[nb] = nc
                    heapq.heappush(opened, (nc, nb))
        # build finished. known mode that cannot reach the robot escalates to
        # attempt (dual-mode, S4A.3); otherwise the build becomes the field.
        # Reachability accepts the robot cell OR any 8-neighbor: a hugging
        # robot can sit on a coarse cell quantized BLOCKED -- demanding the
        # exact cell would false-prove no-path against a wall it is legally
        # following at d_wall.
        robot_cell = self._cell_of(pose_xy)
        reached = False
        if robot_cell is not None:
            # 5x5 neighborhood (1 m radius at 0.5 m cells): in a dense scene
            # a hugging robot's own cell AND its 8-ring can all quantize
            # BLOCKED from walked-past surfaces; demanding them would
            # false-prove no-path mid-hug (106-obstacle field scene).
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    if (robot_cell[0] + dx, robot_cell[1] + dy) in build:
                        reached = True
                        break
                if reached:
                    break
        if self._build_mode == "known" and not reached:
            self._start_build("attempt", now_ms)
            return
        if self._build_mode == "attempt":
            self._unreachable_builds = 0 if reached \
                else self._unreachable_builds + 1
        else:
            self._unreachable_builds = 0
        self._field = build
        self._field_mode = self._build_mode
        self._field_dom = self._dom          # R1-21: field owns its lattice
        self._build = None
        self._open = []
        self._build_mode = "none"

    def _coarse_state_fresh(self, cell, grid, now_ms) -> Cell:
        # bypass the (build-scoped) cache -- used before a build starts.
        self._coarse_cache.pop(cell, None)
        return self._coarse_state(cell, grid, now_ms)

    def _coarse_read_only(self, cell, grid, now_ms, dom=None) -> Cell:
        """Fresh aggregate WITHOUT touching the build cache: per-tick
        consumers (chain prefix) must not poison the snapshot the build is
        expanding on. dom selects the lattice (default: the build domain)."""
        dom = dom if dom is not None else self._dom
        if dom is None:
            return Cell.UNKNOWN
        cx, cy = self._center_of(cell, dom)
        if self._static_blocked and (
                int(cx / self._coarse_m),
                int(cy / self._coarse_m)) in self._static_blocked:
            return Cell.BLOCKED
        q = self._coarse_m / 4.0
        n_free = 0
        for dx, dy in ((-q, -q), (-q, q), (q, -q), (q, q)):
            st = grid.read(cx + dx, cy + dy, now_ms)
            if st == Cell.BLOCKED:
                return Cell.BLOCKED
            if st == Cell.FREE:
                n_free += 1
        return Cell.FREE if n_free >= 3 else Cell.UNKNOWN

    def chain_free_prefix(self, grid, now_ms: int):
        """(prefix_m, end_xy) of the OBSERVED-FREE prefix of the served
        descent chain (line-following iteration, user order 2026-09-11):
        the chain from the robot outward, cut at the first cell that a
        FRESH read does not confirm FREE. This prefix is CONFIRMED ground
        -- steering/subgoals along it never violate "optimism decides,
        confirmation steers" even under an attempt-mode field. Returns
        (0.0, None) when no chain is served."""
        if not self._last_chain or self._field_dom is None:
            return (0.0, None)
        length = 0.0
        end = None
        for cell in self._last_chain:
            if self._coarse_read_only(cell, grid, now_ms,
                                      self._field_dom) != Cell.FREE:
                break
            # clearance leg (line1 sweep: -0.012 m body overlap): FREE alone
            # admits a cell BESIDE a wall -- the chain rode obstacle edges
            # and the prefix subgoal bypassed the candidate clearance gate.
            # A prefix cell must also have NO BLOCKED 8-neighbor, keeping
            # the confirmed line >= one coarse cell off every obstacle.
            beside_blocked = False
            for dx, dy, _ in _NBRS:
                nb = (cell[0] + dx, cell[1] + dy)
                if self._coarse_read_only(nb, grid, now_ms,
                                          self._field_dom) == Cell.BLOCKED:
                    beside_blocked = True
                    break
            if beside_blocked:
                break
            length += self._coarse_m
            end = self._center_of(cell, self._field_dom)
        return (length, end)

    # ── R* guide point + path (viz) ──────────────────────────────────────────
    def _descend(self, cell) -> Optional[Tuple[int, int]]:
        field = self._field
        best = None
        best_c = field.get(cell, math.inf)
        for dx, dy, _ in _NBRS:
            nb = (cell[0] + dx, cell[1] + dy)
            c = field.get(nb)
            if c is not None and c < best_c:
                best_c = c
                best = nb
        return best

    def _los_free(self, a_xy, b_xy) -> bool:
        """Coarse-grid line of sight: sample the segment at half-cell pitch;
        any BLOCKED (or never-expanded, conservatively) cell breaks it. Uses
        the build-scoped coarse cache -- complete after a finished build."""
        dx, dy = b_xy[0] - a_xy[0], b_xy[1] - a_xy[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return True
        n = max(1, int(dist / (self._coarse_m / 2.0)))
        for k in range(1, n + 1):
            p = (a_xy[0] + dx * k / n, a_xy[1] + dy * k / n)
            cell = self._cell_of(p)
            if cell is None:
                return False
            st = self._coarse_cache.get(cell)
            if st is None or st == Cell.BLOCKED:
                return False
        return True

    def steering_guide(self, pose_xy) -> Optional[Tuple[float, float]]:
        """R* for CONTINUOUS steering -- served only from a KNOWN-mode field
        (observed-FREE ground). The g1i deterministic sweep settled the
        principle: optimism decides, confirmation steers. An attempt-mode
        field is an optimistic straight line through UNKNOWN -- steering by
        it is no smarter than the reference line and loses the line's
        geometric stability (south->car_back went 57.8 m -> TIMEOUT under
        attempt-pull). Attempt fields still serve the DISCRETE consumers
        (wall side pick, candidate aim) via guide_point."""
        if self._field_mode != "known":
            return None
        return self.guide_point(pose_xy)

    def guide_point(self, pose_xy) -> Optional[Tuple[float, float]]:
        """R*: the FARTHEST point on the descent path that is LINE-OF-SIGHT
        reachable from the robot (pure-pursuit on the planned path, capped at
        8 m). The g1b/g1c sweeps proved a fixed-lookahead R* is unusable
        while wall-hugging: the guide runs ALONG the wall ahead, the straight
        robot->R* line grazes the wall, the blocked-entry check fires every
        tick and the walk churns enter/leave (one FAIL each sweep, opposite
        halves of the same dilemma). A sighted R* is straight-line clean by
        construction -- heading for it never re-triggers the wall.
        None when the field does not cover the robot -- caller falls back."""
        if self._field is None or self._field_dom is None:
            return None
        cell = self._cell_of(pose_xy, self._field_dom)
        if cell is None or cell not in self._field:
            # the robot moved OUT of the served field's coverage (a known-
            # mode field only spans observed FREE ground; a hugging robot
            # can outrun it). A stale field would return None until the
            # period gate -- seconds of fallback -- so request a rebuild NOW.
            self.request_replan()
            return None
        # descent polyline from the robot cell (bounded)
        pts: List[Tuple[float, float]] = []
        chain: List[Tuple[int, int]] = []
        cur = cell
        for _ in range(int(8.0 / self._coarse_m)):
            nxt = self._descend(cur)
            if nxt is None:
                break
            cur = nxt
            chain.append(cur)
            pts.append(self._center_of(cur, self._field_dom))
        self._last_chain = chain
        if not pts:
            return None                      # no descent: at/around the goal
        # farthest sighted point (scan far -> near; first hit wins)
        for p in reversed(pts):
            if self._los_free(pose_xy, p):
                return p
        return pts[0]                        # nothing sighted: next step still
                                             # carries the field's direction

    def path_points(self, pose_xy, max_pts: int = 240) -> List[Tuple[float, float]]:
        """Full descent polyline for visualization (SIL). Not used for
        control -- R* is; keep it cheap and bounded."""
        if self._field is None or self._field_dom is None:
            return []
        cell = self._cell_of(pose_xy, self._field_dom)
        if cell is None or cell not in self._field:
            return []
        pts = []
        cur = cell
        for _ in range(max_pts):
            nxt = self._descend(cur)
            if nxt is None:
                break
            cur = nxt
            pts.append(self._center_of(cur, self._field_dom))
        return pts
