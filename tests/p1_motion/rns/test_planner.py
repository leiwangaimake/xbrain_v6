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
