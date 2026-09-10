"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_planner.py
Brief: Guidance layer -- coarse view, dual-mode field, sighted R* (20 S4A)

Description:
Guards the G1 guidance planner: thin-obstacle coarse aggregation (BLOCKED
dominates), dual-mode known->attempt escalation, the deterministic node
budget, and the two-grade guide split (optimism decides, confirmation
steers). Each test names its mutant (CLAUDE.md 3.3).
"""

from __future__ import annotations

import copy
from pathlib import Path

import yaml

from xbrain.p1_motion.rns.grid import MemoryGrid
from xbrain.p1_motion.rns.planner import GuidancePlanner
from xbrain.p1_motion.rns.types import Cell

_ROOT = Path(__file__).resolve().parents[3]
_CFG = yaml.safe_load((_ROOT / "configs" / "rns.yaml").read_text(
    encoding="utf-8"))["rns"]


def _mk():
    g = MemoryGrid(0.25, 600.0, 2.0, 1.0)
    p = GuidancePlanner(copy.deepcopy(_CFG))
    return g, p


def _paint(g, x0, x1, y0, y1, state, now=1000):
    x = x0
    while x <= x1 + 1e-9:
        y = y0
        while y <= y1 + 1e-9:
            g.write(x, y, state, now)
            y += 0.25
        x += 0.25


def _run_until_field(p, g, pose, ticks=200, now=1000):
    for _ in range(ticks):
        p.on_tick(g, pose, now)
        if p._field is not None:
            return True
    return False


def test_coarse_aggregation_blocked_dominates():
    # a THIN wall (one fine-cell line) must poison its coarse cell: a
    # majority-FREE rule that ignores BLOCKED would let the planner route
    # THROUGH walls. mutant: drop the BLOCKED branch in _coarse_state ->
    # the field crosses the wall line -> reddens (guide crosses x=2).
    g, p = _mk()
    _paint(g, 0.0, 4.0, -2.0, 2.0, Cell.FREE)          # open room
    _paint(g, 2.0, 2.0, -2.0, 2.0, Cell.BLOCKED)       # thin wall at x=2
    p.set_task((0.0, 0.0), (4.0, 0.0))
    assert _run_until_field(p, g, (0.0, 0.0))
    # the wall spans the domain vertically minus margins; the known field
    # must NOT descend straight through x=2 -- expect either no coverage at
    # the robot (known blocked) or a guide that is not the straight line.
    r = p.guide_point((0.0, 0.0))
    if r is not None:
        cell = p._cell_of((2.0, 0.0))
        assert p._coarse_cache.get(cell) == Cell.BLOCKED


def test_dual_mode_escalates_to_attempt():
    # goal observed FREE but a full-width UNKNOWN band separates it from the
    # robot: the known pass cannot reach -> the build must escalate to
    # attempt (S4A.3) and produce a field. mutant: drop the escalation ->
    # field stays None -> reddens.
    g, p = _mk()
    _paint(g, 0.0, 1.0, -1.0, 1.0, Cell.FREE)           # around robot
    _paint(g, 5.0, 6.0, -1.0, 1.0, Cell.FREE)           # around goal
    # 1..5 never observed (UNKNOWN)
    p.set_task((0.5, 0.0), (5.5, 0.0))
    assert _run_until_field(p, g, (0.5, 0.0))
    assert p._field_mode == "attempt"
    assert p.guide_point((0.5, 0.0)) is not None
    # and the steering grade REFUSES an attempt field (optimism decides,
    # confirmation steers -- g1i lesson). mutant: serve steering from
    # attempt -> reddens.
    assert p.steering_guide((0.5, 0.0)) is None


def test_known_mode_serves_steering():
    g, p = _mk()
    _paint(g, 0.0, 6.0, -1.5, 1.5, Cell.FREE)           # fully observed lane
    p.set_task((0.5, 0.0), (5.5, 0.0))
    assert _run_until_field(p, g, (0.5, 0.0))
    assert p._field_mode == "known"
    r = p.steering_guide((0.5, 0.0))
    assert r is not None and r[0] > 0.5                  # pulls toward goal


def test_budget_is_deterministic_node_quota():
    # two identical runs must expand identically per tick (no wall-clock
    # budget): same tick count to field completion. mutant: reintroduce a
    # time-based budget -> tick counts diverge across runs (flaky) -- this
    # test pins at least the EQUALITY of two in-process runs.
    def ticks_to_field():
        g, p = _mk()
        _paint(g, 0.0, 6.0, -1.5, 1.5, Cell.FREE)
        p.set_task((0.5, 0.0), (5.5, 0.0))
        for i in range(200):
            p.on_tick(g, (0.5, 0.0), 1000)
            if p._field is not None:
                return i
        return -1
    a = ticks_to_field()
    b = ticks_to_field()
    assert a == b and a >= 0


def test_sealed_memory_proves_domain_no_path():
    # G2 core proof (S4A.3): the goal cell ringed by remembered BLOCKED on
    # OBSERVED ground -- two consecutive attempt builds cannot reach the
    # robot => domain_no_path. This is the mechanism test; the scene-level
    # sealed-box test only asserts BOUNDED termination (whichever of
    # no_path_in_domain / wall-family fires first -- equivalent verdicts,
    # 20 S9.0.2 note). mutants: (a) allow corner cutting -> the field leaks
    # through ring corners -> never proves -> reddens; (b) domain_no_path
    # always False -> reddens.
    g, p = _mk()
    _paint(g, -3.0, 8.0, -4.0, 4.0, Cell.FREE)          # observed ground
    _paint(g, 2.0, 6.0, 2.0, 2.5, Cell.BLOCKED)         # sealed ring walls
    _paint(g, 2.0, 6.0, -2.5, -2.0, Cell.BLOCKED)
    _paint(g, 2.0, 2.5, -2.5, 2.5, Cell.BLOCKED)
    _paint(g, 5.5, 6.0, -2.5, 2.5, Cell.BLOCKED)
    p.set_task((-2.0, 0.0), (4.0, 0.0))                 # goal inside
    pose = (-2.0, 0.0)
    now = 1000
    for i in range(400):
        now = 1000 + i * 50
        p.on_tick(g, pose, now)
        if p.domain_no_path(now):
            break
    assert p.domain_no_path(now), "sealed ring never proved no-path"
    # PER-TASK verdict (user ruling 2026-09-11, the frozen-navigator bug):
    # a new order must NOT inherit the old order's no-path -- it TRIES.
    # NOTE on mutants: dropping the set_task counter reset alone is now an
    # EQUIVALENT mutant -- the min-try window plus the on-success zeroing
    # cover the leak's damage path (the original field bug predated both).
    # The reset stays as stated intent (CLAUDE.md 7.2.1: equivalent mutants
    # get a note, not a forced assertion); this assert pins the WINDOW leg.
    p.set_task((-2.0, 0.0), (-5.0, 0.0))                # open-ground goal
    assert not p.domain_no_path(now + 50), \
        "no-path verdict leaked into the next order"


def test_diagonal_wall_seam_does_not_leak():
    # corner-cutting pin (G2 debug): a 45-deg thin wall coarsens into a
    # DIAGONAL chain of BLOCKED cells -- every link is a corner seam. With
    # corner cutting allowed, the wavefront slips between diagonal
    # neighbors and the no-path proof never completes. Domain shrunk so the
    # wall spans it fully. mutant: drop the two-orthogonal-neighbors check
    # on diagonal steps -> the field leaks across -> reddens.
    g = MemoryGrid(0.25, 600.0, 2.0, 1.0)
    cfg = copy.deepcopy(_CFG)
    cfg["guidance"]["domain_margin_m"] = 3.0
    p = GuidancePlanner(cfg)
    _paint(g, -6.0, 8.0, -6.0, 8.0, Cell.FREE)          # observed everywhere
    x = -5.0
    while x <= 7.0:                                     # 45-deg wall line
        g.write(x, x - 1.0, Cell.BLOCKED, 1000)
        g.write(x + 0.125, x - 0.875, Cell.BLOCKED, 1000)
        x += 0.25
    p.set_task((0.0, 3.0), (3.0, -2.0))                 # opposite sides
    pose = (0.0, 3.0)
    now = 1000
    for i in range(400):
        now = 1000 + i * 50
        p.on_tick(g, pose, now)
        if p.domain_no_path(now):
            break
    assert p.domain_no_path(now), "field leaked through the diagonal seam"


def test_static_polygon_layer_blocks_planning():
    # G3 (S4A.4): a keep-out polygon (fence/prior map, curated upstream)
    # rasterizes into the static layer and the field routes AROUND it --
    # while the layer survives set_task (static world does not die with a
    # mission). mutant: skip the static check in _coarse_state -> the field
    # crosses the polygon -> reddens.
    g, p = _mk()
    _paint(g, -1.0, 7.0, -4.0, 4.0, Cell.FREE)
    p.set_static_polygons([[(2.0, -3.0), (4.0, -3.0), (4.0, 3.0), (2.0, 3.0)]])
    p.set_task((0.0, 0.0), (6.0, 0.0))
    assert _run_until_field(p, g, (0.0, 0.0))
    chain_cells = []
    cur = p._cell_of((0.0, 0.0))
    for _ in range(60):
        nxt = p._descend(cur)
        if nxt is None:
            break
        cur = nxt
        chain_cells.append(p._center_of(cur))
    assert chain_cells, "no descent at all"
    for (cx, cy) in chain_cells:
        assert not (2.0 < cx < 4.0 and -3.0 < cy < 3.0), \
            "path crossed the keep-out at (%.1f, %.1f)" % (cx, cy)
    # static layer survives a new task (lifecycle: static != mission-scoped)
    p.set_task((0.0, 0.0), (6.0, 2.0))
    assert p._static_blocked, "static layer died with the task"
