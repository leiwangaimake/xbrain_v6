"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_grid_ring.py
Brief: MemoryGrid.ring_counts -- the rotation permit's annulus census (12 S6A.3.2)

Description:
The census feeding 12 S2.2 step 6b, and it can be wrong in two opposite
directions that this file keeps apart on purpose.

Undercount, the dangerous one: a cell holding an obstacle that the scan never
visits, or a stale FREE that vouches for ground nobody has looked at in five
minutes. Either one hands the rotation permit a clean ring for a ring that is
not clean, and the permit has nothing else to check against.

Overcount, the one that gets the gate switched off in the field: counting cells
outside the annulus, or calling a freshly-seen FREE unknown. Every extra
unknown cell is a tick that turns at 0.3 rad/s instead of at commanded speed,
and every phantom occupied cell is a refused turn.

What this file does NOT cover: the permit's verdict (tests/p1_motion/nav/
test_nav_tick_rotation.py) and the assembly into a RingSample (test_nav_wiring_
ring.py). Here the grid is written by hand so a census failure cannot hide
behind a perception fixture.
"""

from __future__ import annotations

import math

from xbrain.p1_motion.rns.grid import MemoryGrid, RingCounts
from xbrain.p1_motion.rns.types import Cell

CELL = 0.25
NOW = 100_000
FRESH = 200          # grid_age_max_ms, the FREE freshness window


def _grid(cell_m=CELL, ttl_static_s=300.0, ttl_dynamic_s=2.0):
    return MemoryGrid(cell_m, ttl_static_s, ttl_dynamic_s, reset_jump_m=1.0)


def _census(g, *, pose=(0.0, 0.0), yaw=0.0, r_in=0.0, r_out=1.5,
            now=NOW, fresh=FRESH) -> RingCounts:
    return g.ring_counts(pose, yaw, r_in, r_out, now, fresh)


def _paint_disc(g, pose, r_m, state, now, cell_m=CELL):
    """Fill every cell centre within r_m of pose. Written as a helper because
    every test needs a KNOWN ring to start from -- an unwritten grid is all
    UNKNOWN, which is the answer most of these tests are trying not to get by
    accident."""
    n = int(r_m / cell_m) + 2
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            x = pose[0] + i * cell_m
            y = pose[1] + j * cell_m
            if math.hypot(x - pose[0], y - pose[1]) <= r_m:
                g.write(x, y, state, now)


# ---------------------------------------------------------------------------
# The domain: which cells are in A, and which are not.
# ---------------------------------------------------------------------------

def _expected_domain(pose, r_in, r_out, cell_m=CELL):
    """Independent count of |A|, the oracle for the census's total.

    Deliberately a second implementation of the DOMAIN only -- no TTL, no
    states, no bins. That keeps it a real oracle rather than a copy of the code
    under test: it answers "which cells are in the annulus", which is the one
    thing a census cannot be allowed to get approximately right.
    """
    n = 0
    i0 = math.floor((pose[0] - r_out) / cell_m)
    i1 = math.floor((pose[0] + r_out) / cell_m)
    j0 = math.floor((pose[1] - r_out) / cell_m)
    j1 = math.floor((pose[1] + r_out) / cell_m)
    for ix in range(i0, i1 + 1):
        for iy in range(j0, j1 + 1):
            rho = math.hypot((ix + 0.5) * cell_m - pose[0],
                             (iy + 0.5) * cell_m - pose[1])
            if r_in <= rho <= r_out:
                n += 1
    return n


def test_the_domain_is_exactly_the_annulus_no_more_no_less():
    """total must equal |A| exactly, counted independently.

    An approximate assertion is not enough here, and this file learned that the
    hard way: a 24-ray census of a 1.482 m ring at 0.25 m cells visits 120
    cells against a true |A| of 110, which sits inside any tolerance loose
    enough to survive a resolution change. Exact equality is the only form that
    separates "visited every cell once" from "visited a similar number of them".

    That makes this one assertion cover three distinct failures: a ray-cast
    scan (misses the cells between rays -- a person standing there is 15
    degrees of arc wide at this range and fits entirely in the gap), a missing
    radius filter (the bounding square's corners, 4/pi - 1 = 27% extra), and a
    truncating index bound (see the negative-pose case below).

    The pose is deliberately off-grid and negative: a pose whose bounds divide
    evenly into the cell size makes int() and floor() agree, which is exactly
    how the truncation bug survived the first version of this file.
    """
    g = _grid()
    for pose, r_in, r_out in (((0.0, 0.0), 0.0, 1.482),
                              ((-3.1, -2.7), 0.0, 1.482),
                              ((7.3, -1.9), 0.6, 1.482)):
        c = _census(g, pose=pose, r_in=r_in, r_out=r_out)
        assert c.total == _expected_domain(pose, r_in, r_out), (pose, r_in, r_out)


def test_every_cell_of_the_annulus_is_visited():
    """A cell no ray passes through must still be found.

    nearest_blocked_full answers its own question with 24 rays, and reusing
    that shape here is the obvious economy. It is also a fail-open: the gap
    between two rays at 2.4 m is 0.63 m, two and a half cells wide, so an
    obstacle can sit in it entirely. The bearing and range below are picked so
    that no 15-degree ray's stepped samples land in that cell.

    mutant: scan with rays instead of the bounding square -> occupied 0.
    """
    g = _grid()
    ang = math.radians(7.5)
    g.write(2.4 * math.cos(ang), 2.4 * math.sin(ang), Cell.BLOCKED, NOW)
    c = _census(g, r_out=2.5)
    assert c.occupied == 1, "a cell between the rays was never visited"


def test_cells_outside_the_outer_radius_are_not_counted():
    """Overcount guard. The scan walks a SQUARE and filters by radius; drop
    the filter and the corners come in -- 4/pi - 1, about 27% more cells, all
    of them beyond r_check. On a machine whose surroundings are mostly
    unobserved that is 27% more unknown cells for no reason, and an obstacle
    sitting outside the sweep envelope would refuse a turn that is safe.

    mutant: delete the rho2 > r_out2 test -> occupied 1.
    """
    g = _grid()
    _paint_disc(g, (0.0, 0.0), 3.0, Cell.FREE, NOW)
    # inside the bounding square of r_out=1.0 (corner is at 1.41), outside the
    # circle: a cell on the diagonal at 1.2 m.
    d = 1.2 / math.sqrt(2.0)
    g.write(d, d, Cell.BLOCKED, NOW)
    c = _census(g, r_out=1.0)
    assert c.occupied == 0, "a cell beyond r_check was counted"


def test_cells_inside_the_self_mask_are_not_counted():
    """RCG-2's inner radius. 0 is the default and means the whole disc, but a
    positive r_self_mask must actually exclude, or the key does nothing and
    the 12 S6A.2 warning about masking real obstacles hitting the body would
    be describing a setting with no effect.

    mutant: drop the rho2 < r_in2 test -> occupied 1.
    """
    g = _grid()
    _paint_disc(g, (0.0, 0.0), 2.0, Cell.FREE, NOW)
    g.write(0.3, 0.0, Cell.BLOCKED, NOW)
    assert _census(g, r_in=0.0).occupied == 1        # whole disc: seen
    assert _census(g, r_in=0.6).occupied == 0        # masked: not seen


def test_the_three_states_partition_the_domain():
    """occupied + unknown + free == total, always.

    Cheap, and it catches the class of bug where a branch forgets to increment
    anything: the permit reads occ_cells and unknown_cells against thresholds
    and total_cells against zero (RCG-2), so a census whose parts do not add up
    can pass RCG-2 with an annulus it never actually classified.
    """
    g = _grid()
    _paint_disc(g, (0.0, 0.0), 1.0, Cell.FREE, NOW)
    g.write(0.8, 0.0, Cell.BLOCKED, NOW)
    c = _census(g, r_out=1.5)
    assert c.occupied + c.unknown + c.free == c.total
    assert c.total > 0
    assert c.occupied >= 1 and c.unknown >= 1 and c.free >= 1


def test_pose_away_from_the_origin_recentres_the_annulus():
    """The annulus is centred on the POSE, not on the grid origin.

    MemoryGrid is anchored in the localization frame (20 S4.2), so the body
    moves through a fixed grid. An implementation that scanned around (0, 0)
    would be judging the ring at the site origin -- which passes every test
    written with the robot parked there, and refuses or permits at random once
    it drives away.

    mutant: use (0, 0) instead of pose_xy for the centre -> occupied 0 and the
    total collapses to whatever is near the origin.
    """
    g = _grid()
    pose = (12.0, -7.0)
    _paint_disc(g, pose, 2.0, Cell.FREE, NOW)
    g.write(pose[0] + 1.0, pose[1], Cell.BLOCKED, NOW)
    c = _census(g, pose=pose)
    assert c.occupied == 1 and c.free > 0
    # and nothing near the origin leaks in
    assert _census(g, pose=(0.0, 0.0)).free == 0


def test_negative_indices_do_not_lose_a_column():
    """floor(), not int(), on the low index bound.

    int() truncates toward zero, so for a pose west or south of the origin the
    first column/row of the bounding square is dropped. The symptom is a ring
    that is correct on the +x/+y side of the site and quietly missing its edge
    on the other -- exactly the kind of asymmetry nobody reproduces.

    The offsets are 3.1, not 3.0: at 3.0 with r_out 1.5 the low bound divides
    evenly into the cell size, int() and floor() agree, and the bug hides. The
    first version of this test used 3.0 and the truncation mutant survived it.

    mutant: int((cx - r) / cell) instead of floor -> the counts for the two
    poses below stop matching.
    """
    g = _grid()
    for pose in ((3.1, 3.1), (-3.1, -3.1)):
        _paint_disc(g, pose, 2.0, Cell.FREE, NOW)
    east = _census(g, pose=(3.1, 3.1))
    west = _census(g, pose=(-3.1, -3.1))
    assert east.total == west.total
    assert east.free == west.free


# ---------------------------------------------------------------------------
# Freshness: the asymmetric rule (12 S15 #51).
# ---------------------------------------------------------------------------

def test_stale_free_is_counted_unknown_not_free():
    """Memory may refuse this permit; it may not grant it.

    A cell seen empty is evidence about the past. The grid's own TTL keeps it
    readable as FREE for ttl_static_s (300 s today), while RCG-4's lower bound
    only ever priced grid_age_max_ms plus the recheck window -- 350 ms. Letting
    a 300 s old FREE count as free is therefore a fail-open with three orders
    of magnitude of slack in it: a person can walk into the ring and stand
    there for minutes without changing the census.

    mutant: count FREE without the age test -> free stays high and unknown
    stays 0, and a spin over five-minute-old ground reads as permitted.
    """
    g = _grid()
    _paint_disc(g, (0.0, 0.0), 2.0, Cell.FREE, NOW)
    fresh = _census(g, now=NOW)
    stale = _census(g, now=NOW + FRESH + 1)
    assert fresh.free > 0 and fresh.unknown == 0
    assert stale.free == 0
    assert stale.unknown == fresh.free + fresh.unknown
    assert stale.total == fresh.total          # the domain did not change


def test_blocked_survives_the_free_freshness_window():
    """The other half of the asymmetry, and it is not symmetric by accident.

    Downgrading a remembered BLOCKED to UNKNOWN after 200 ms would move that
    cell from "refuse" to "clamp at 0.3 rad/s" -- the memory of an obstacle
    would expire faster than the obstacle does, and the robot would start
    turning into things it saw a quarter of a second ago. BLOCKED stays on the
    grid's own TTL.

    mutant: apply free_max_age_ms to BLOCKED too -> occupied 0 below.
    """
    g = _grid()
    g.write(1.0, 0.0, Cell.BLOCKED, NOW)
    late = _census(g, now=NOW + FRESH * 10)
    assert late.occupied == 1
    # and it does eventually expire, on the grid's TTL, not on ours
    assert _census(g, now=NOW + 301_000).occupied == 0


def test_expired_blocked_becomes_unknown_not_free():
    """When memory does expire, it expires to "I do not know", never to free.

    The TTL exists because a person walks away; it does not mean the cell was
    observed empty. Counting an expired cell as free would make the dynamic
    TTL (2 s) a licence to turn into wherever a person was two seconds ago.
    """
    g = _grid()
    g.write(1.0, 0.0, Cell.BLOCKED, NOW, cls="person")     # dynamic TTL 2 s
    after = _census(g, now=NOW + 2_100)
    assert after.occupied == 0
    assert after.free == 0
    assert after.unknown == after.total


# ---------------------------------------------------------------------------
# blocked_deg: 12 S6A.8 OB-2's diagnostic.
# ---------------------------------------------------------------------------

def test_blocked_deg_is_body_frame_and_follows_yaw():
    """The interval says where the obstacle is relative to the BODY.

    A world-frame bearing here would be read by the field as "it is on my left"
    while the robot faces south, which is worse than no interval at all. Same
    obstacle, two headings, two different reported sectors.

    mutant: report atan2(dy, dx) without subtracting yaw -> the two rows below
    come out identical.
    """
    g = _grid()
    _paint_disc(g, (0.0, 0.0), 2.0, Cell.FREE, NOW)
    g.write(1.0, 0.0, Cell.BLOCKED, NOW)               # due east in world
    ahead = _census(g, yaw=0.0).blocked_deg            # facing east -> front
    left = _census(g, yaw=-math.pi / 2).blocked_deg    # facing south -> left
    assert ahead and left and ahead != left
    assert any(lo <= 0.0 < hi for lo, hi in ahead), ahead
    assert any(lo <= 90.0 < hi for lo, hi in left), left


def test_blocked_deg_merges_adjacent_bins_and_is_empty_when_clear():
    """One obstacle should print as one interval, and a clean ring as none.

    Not merging is only a readability defect, but the empty case is not: an
    implementation that always reported something would make the field chase a
    phantom on every clamped tick, and the clamped tick is now the common one.
    """
    g = _grid()
    _paint_disc(g, (0.0, 0.0), 2.0, Cell.FREE, NOW)
    assert _census(g).blocked_deg == ()
    # a contiguous arc of blocked cells at r = 1.0, spanning ~45 degrees
    for deg in range(0, 46, 3):
        a = math.radians(deg)
        g.write(1.0 * math.cos(a), 1.0 * math.sin(a), Cell.BLOCKED, NOW)
    got = _census(g).blocked_deg
    assert len(got) == 1, got
    lo, hi = got[0]
    assert lo <= 0.0 and hi >= 45.0


# ---------------------------------------------------------------------------
# Budget (CLAUDE.md 4.4). Not a timing assertion -- those are flaky on a shared
# box -- but a cell-count one, which is the quantity 12 S6A.3.1 actually bounds.
# ---------------------------------------------------------------------------

def test_visited_cell_count_matches_the_documented_arithmetic():
    """12 S6A.3.2's complexity note must describe this implementation.

    N = (2*ceil(r_out/cell) + 1)^2 visits, |A| ~ pi*r_out^2/cell^2 counted. The
    doc quotes 169 visits at today's values; if the scan silently grew (a
    padded bound, a second pass) the note would go stale without anything
    failing, and the 20 Hz budget argument would rest on a number nobody
    re-derived.
    """
    g = _grid(cell_m=0.25)
    r_out = 1.482                       # r_robot 0.482 + d_safe 1.00
    square = (2 * math.ceil(r_out / 0.25) + 1) ** 2
    assert square == 169
    c = _census(g, r_out=r_out)
    assert c.total <= square
    # the annulus is pi/4 of its bounding square, within one ring of cells
    ideal = math.pi * r_out * r_out / (0.25 * 0.25)
    assert abs(c.total - ideal) < 0.15 * ideal, (c.total, ideal)
