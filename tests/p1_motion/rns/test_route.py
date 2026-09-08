"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_route.py
Brief: Polyline projection / lookahead / monotone-index tests (P1 -- 20 S2)

Description:
Guards route.py's P1.1/P1.2 slice: projection correctness, lookahead arc-length
walk, and the two RNS-N-4 invariants (A-RT-3 monotone index, A-RT-4 windowed
search). Each test names its mutant (CLAUDE.md 3.3).
"""

from __future__ import annotations

import math

import pytest

from xbrain.p1_motion.rns.route import (
    PolylineTracker, lookahead_distance,
)


def _straight(n=11, spacing=1.0):
    # a straight polyline along +x: (0,0),(1,0),...,(10,0)
    return [(i * spacing, 0.0) for i in range(n)]


def test_projection_on_straight_line():
    tr = PolylineTracker(_straight(), search_window=50)
    p = tr.project((3.4, 0.5))
    assert p.seg_index == 3
    assert p.foot[0] == pytest.approx(3.4, abs=1e-9)
    assert p.foot[1] == pytest.approx(0.0, abs=1e-9)
    assert p.deviation_m == pytest.approx(0.5, abs=1e-9)
    assert p.s_arc_m == pytest.approx(3.4, abs=1e-9)


def test_lookahead_walks_arclength_across_segments():
    tr = PolylineTracker(_straight(), search_window=50)
    p = tr.project((2.0, 0.0))
    r = tr.lookahead_point(p, lookahead_m=3.5)
    assert r[0] == pytest.approx(5.5, abs=1e-9)  # 2.0 + 3.5


def test_lookahead_clamps_at_polyline_end():
    tr = PolylineTracker(_straight(), search_window=50)
    p = tr.project((9.0, 0.0))
    r = tr.lookahead_point(p, lookahead_m=5.0)  # would overshoot 10 -> clamps
    assert r == (10.0, 0.0)


def test_index_is_monotone_structurally():
    # A-RT-3: min_index is non-decreasing under ANY position sequence, including
    # a hairpin whose return leg is geometrically nearest a passed segment.
    # Monotonicity is structural (search lower bound = min_index, so best_i >=
    # min_index) -- this test pins the OBSERVABLE contract: feed a U-turn and the
    # index never goes down. A design that lowered the search bound below
    # min_index (the real way to break this) reddens here.
    # Hairpin: out along +x to (4,0), back along a parallel line y=0.1.
    out = [(float(i), 0.0) for i in range(5)]           # seg 0..3
    back = [(4.0 - float(i), 0.1) for i in range(1, 5)]  # seg 4..7 returning
    tr = PolylineTracker(out + back, search_window=20)
    last = -1
    for x in [(0.5, 0.0), (2.5, 0.0), (3.9, 0.0),        # outbound
              (3.0, 0.1), (1.0, 0.1), (0.2, 0.1)]:        # return leg, nearer
        tr.project(x)                                     # to outbound geometry
        assert tr.min_index >= last, "index retreated"
        last = tr.min_index
    # ended on the return leg, index climbed past the outbound segments
    assert tr.min_index >= 4


def test_index_never_below_search_lower_bound():
    # the structural guarantee, stated directly: after any project(), the found
    # segment (and thus min_index) is >= the min_index it started from. mutant:
    # set the loop lower bound to 0 instead of lo -> a nearer earlier segment
    # would be found and min_index would drop -> this reddens.
    tr = PolylineTracker(_straight(n=20), search_window=30)
    tr.project((10.5, 0.0))
    before = tr.min_index
    tr.project((0.5, 3.0))   # geometrically nearest seg 0, but behind min_index
    assert tr.min_index >= before


def test_search_is_windowed_not_full():
    # A-RT-4: with a small window, a point far ahead of the current index is NOT
    # found (search stays in [min_index, min_index+window]). This is what bounds
    # per-tick cost. mutant: search the whole polyline -> it WOULD find seg 40
    # and this assertion (index stays small) reddens.
    pts = _straight(n=100)
    tr = PolylineTracker(pts, search_window=3)
    # robot near segment 40, but index still 0 and window only 3 ahead:
    p = tr.project((40.0, 0.0))
    assert tr.min_index <= 3
    assert p.seg_index <= 3


def test_window_advances_step_by_step():
    # feeding progressively advancing positions, the index climbs within window.
    tr = PolylineTracker(_straight(n=100), search_window=3)
    for x in range(0, 60):
        tr.project((float(x), 0.0))
    # reached the far end by stepping, not by a single full search
    assert tr.min_index >= 55


def test_single_point_polyline_is_goto():
    # RNS-N-1: a single point is a degenerate polyline (goto). Projection gives
    # straight-line deviation, s_arc 0, lookahead is the point itself.
    tr = PolylineTracker([(5.0, 0.0)], search_window=10)
    p = tr.project((2.0, 0.0))
    assert p.deviation_m == pytest.approx(3.0)
    assert p.s_arc_m == 0.0
    assert tr.lookahead_point(p, 2.0) == (5.0, 0.0)


def test_empty_polyline_refused():
    with pytest.raises(ValueError):
        PolylineTracker([], search_window=10)


def test_lookahead_distance_clamps():
    # L = clamp(k*v, L_min, L_max)
    assert lookahead_distance(2.0, k=1.5, l_min=1.0, l_max=4.0) == 3.0
    assert lookahead_distance(0.1, k=1.5, l_min=1.0, l_max=4.0) == 1.0  # floor
    assert lookahead_distance(10.0, k=1.5, l_min=1.0, l_max=4.0) == 4.0  # ceil
