"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_nav_wiring_ring.py
Brief: NavInputs.ring is produced from the RNS memory grid, and both failure directions hold end to end

Description:
The rotation permit was wired into 12 S2.2 step 6b on 2026-09-29 with nothing
producing its RingSample, so every spin_like tick answered ring_source_
unavailable. This file covers the producer and the two ways it can be wrong,
through the REAL objects: a real RnsSource built from configs/rns.yaml, its
real MemoryGrid, the real _ring_sample of NavRuntime, and the real permit.

Too little: a ring that reports clean ground it never observed, or that misses
an obstacle beside the body. The permit has no second opinion to catch that
with -- free_space.sectors has no producer, so the cross-veto is blind too.

Too much: a ring that reports everything unknown when perception is healthy, or
that reports no ring at all. Both put the machine back where it was before the
correction: unable to turn, which is 12 S14 trap 19's road to the gate being
switched off in the field.

NavRuntime cannot be constructed here (it needs two zenohd routers), so the two
methods under test are called as unbound functions on a minimal holder, the
same convention and for the same reason as test_nav_wiring_estop_epoch. What
the holder supplies is config and a source; the grid, the census, the assembly
and the verdict are all real code.
"""
from __future__ import annotations

import ast
import copy
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Optional, Tuple

import pytest
import yaml

from tests.p1_motion.nav.rot_fixture import rot_limits
from tests.p1_motion.rns.scenes import healthy_status, snapshot, uniform_free
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rns.types import Cell
from xbrain.p1_motion.rotation.rcg import (
    DECISION_LIMIT,
    DECISION_PASS,
    DECISION_REJECT,
    REASON_NO_RING_SOURCE,
    REASON_OCCUPIED_CELLS,
    REASON_PERMITTED,
    REASON_UNKNOWN_CELLS,
    apply_rotation_permit,
)
from xbrain.p1_motion.runtime.nav_wiring import NavRuntime

pytestmark = pytest.mark.no_device

_ROOT = Path(__file__).resolve().parents[3]
_RNS_CFG = yaml.safe_load((_ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))

# 12 S6A.3.3 records this calibration (2026-08-04, from the M20S manual).
# configs/ carries no r_robot key and CLAUDE.md iron rule 3 forbids landing one
# to make something run, so it is a TEST value: its job is to make r_check
# exist so the ring has a radius, not to stand in for a measurement.
R_ROBOT = 0.482
NOW = 500_000
POSE = (11.0, -4.0)


@dataclass
class _Pose:
    """Only the fields _ring_sample reads off the tick's pose view."""
    xy: Optional[Tuple[float, float]] = POSE
    yaw: Optional[float] = 0.0


class _Cfg:
    def __init__(self, r_robot: Optional[float]) -> None:
        self.r_robot_m = r_robot
        self.rot_limits = rot_limits()


class _Src:
    """Stands in for RnsAvoidSource: one attribute, the real RnsSource."""

    def __init__(self, rns: RnsSource) -> None:
        self.rns = rns


class _Holder:
    """What _ring_sample touches, and nothing else.

    _frame_unknown_ratio is bound in rather than reimplemented: _ring_sample
    calls it through self, and a stub version here would mean the ratio this
    file asserts on is not the ratio the robot computes.
    """

    _frame_unknown_ratio = NavRuntime._frame_unknown_ratio

    def __init__(self, rns: RnsSource, r_robot: Optional[float] = R_ROBOT) -> None:
        self._cfg = _Cfg(r_robot)
        self._src = _Src(rns)


def _rns() -> RnsSource:
    return RnsSource(cfg=copy.deepcopy(_RNS_CFG), r_eff_m=0.5)


def _snap(*, capture_ms: int = NOW - 20, unknown_bins: int = 0):
    """A healthy perception snapshot; unknown_bins forces that many d_free
    entries to None so the frame's unknown_ratio can be driven."""
    prof = uniform_free(t_capture_mono_ms=capture_ms, t_seg_mono_ms=capture_ms - 10)
    if unknown_bins:
        d_free = list(prof.d_free)
        for i in range(min(unknown_bins, len(d_free))):
            d_free[i] = None
        prof = replace(prof, d_free=tuple(d_free))
    return snapshot(profile=prof, objects=None,
                    status=healthy_status(t_publish_mono_ms=capture_ms - 50))


def _feed_grid(rns: RnsSource, *, free_r_m: float = 2.5,
               blocked: Optional[Tuple[float, float]] = None,
               now_ms: int = NOW) -> None:
    """Write a known world into the REAL memory grid, then stamp the ingest.

    Writing cells directly rather than running a mission through compute():
    the census is what is under test, and a mission would make every case here
    depend on the follow pipeline's state machine as well. The ingest stamp is
    set through the same attribute _run_follow sets, so the "grid never
    written" gate stays exercised by the test that wants it.
    """
    grid = rns._grid
    cell = grid._cell_m
    n = int(free_r_m / cell) + 2
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            x, y = POSE[0] + i * cell, POSE[1] + j * cell
            if math.hypot(x - POSE[0], y - POSE[1]) <= free_r_m:
                grid.write(x, y, Cell.FREE, now_ms)
    if blocked is not None:
        grid.write(POSE[0] + blocked[0], POSE[1] + blocked[1], Cell.BLOCKED, now_ms)
    rns._grid_ingest_ms = now_ms


def _ring(holder: _Holder, pose: _Pose, snap: Any, now_ms: int = NOW):
    return NavRuntime._ring_sample(holder, pose, snap, now_ms)


def _permit(ring, *, source: str = "nav2_proxy", r_robot: Optional[float] = R_ROBOT):
    """A pure spin through the real permit: vx 0, wz 1.5 rad/s."""
    return apply_rotation_permit(vx_mps=0.0, vy_mps=0.0, wz_radps=1.5,
                                 source=source, limits=rot_limits(),
                                 r_robot_m=r_robot, ring=ring)


# ---------------------------------------------------------------------------
# Criterion 1 -- availability. The real shape of this machine: the RGBD covers
# the forward half, so the rear of the ring is unobserved. That must CLAMP.
# ---------------------------------------------------------------------------

def test_unobserved_rear_clamps_the_spin_instead_of_refusing_it():
    """The forward-half-coverage case, end to end, and the reason for all this.

    Before the RCG-3 correction this tick was a veto, and since the rear is
    never observed the veto was permanent: 18 A09..A12 all reach the chassis
    through nav2_proxy, so "turn around" could not execute, and seeing behind
    requires turning. A clamped 0.3 rad/s turn is what lets the RGBD sweep the
    rear into the grid, at which point an obstacle there becomes BLOCKED and
    the occupied conjunct stops the turn on its own.

    mutant: drop the ingest stamp / return counts from an empty grid -> the
    ring comes back None and this goes red as a veto.
    """
    rns = _rns()
    # a forward half-disc of observed FREE, nothing behind: the real coverage
    grid = rns._grid
    cell = grid._cell_m
    n = int(2.5 / cell) + 2
    for i in range(0, n + 1):
        for j in range(-n, n + 1):
            x, y = POSE[0] + i * cell, POSE[1] + j * cell
            if math.hypot(x - POSE[0], y - POSE[1]) <= 2.5:
                grid.write(x, y, Cell.FREE, NOW)
    rns._grid_ingest_ms = NOW

    ring = _ring(_Holder(rns), _Pose(), _snap())
    assert ring is not None
    assert ring.occ_cells == 0
    assert ring.unknown_cells > 0, "the rear should read as unobserved"
    assert ring.total_cells > ring.unknown_cells, "the front should read free"
    # THE discriminator for where unknown_ratio comes from, and the reason it
    # lives in this case rather than only in the _frame_unknown_ratio unit
    # test: here the frame is fully observed (ratio 0) while half the RING is
    # not. A ratio re-derived from the census would read about 0.5 -- at or
    # over rot_unknown_ratio_max -- and that conjunct is a HARD refusal, so it
    # would quietly cancel the clamp this whole case is about.
    assert ring.unknown_ratio == pytest.approx(0.0)
    assert ring.unknown_ratio < rot_limits().rot_unknown_ratio_max

    ev = _permit(ring)
    assert ev.reason == REASON_UNKNOWN_CELLS
    assert ev.decision == DECISION_LIMIT
    assert ev.wz_out == pytest.approx(rot_limits().wz_blind_radps)


def test_a_fully_observed_clean_ring_permits_untouched():
    """The permitting path exists and is reachable from real grid data.

    Not decoration: every other case here ends in a refusal of some kind, so
    an _ring_sample that returned an all-unknown census -- or a permit that
    returned False -- would satisfy all of them. This is the case that makes
    the others mean something.

    sectors_min_m is the one conjunct that cannot be satisfied from the grid
    (nothing publishes free_space), and since the correction it is blind, so
    the verdict here is the clamp rather than a pass. The assertion is on what
    the RING says -- zero occupied, zero unknown, a non-empty domain -- because
    that is what this file produces.
    """
    rns = _rns()
    _feed_grid(rns)
    ring = _ring(_Holder(rns), _Pose(), _snap())
    assert ring is not None
    assert ring.total_cells > 0
    assert ring.occ_cells == 0
    assert ring.unknown_cells == 0, "fully observed ground read as unobserved"
    # with the sectors channel supplied, the same ring permits outright
    ev = _permit(replace(ring, sectors_min_m=99.0))
    assert ev.reason == REASON_PERMITTED
    assert ev.decision == DECISION_PASS
    assert ev.wz_out == pytest.approx(1.5)


def test_travel_that_turns_is_not_touched_by_any_of_this():
    """12 S6A.4.1: path_follow's normal turning must not enter the gate.

    Carried here as well as in test_nav_tick_rotation because this file is the
    one that changed what the ring says, and the cheapest way to make a ring
    "safe" is to make everything blocked -- which would leave this assertion as
    the only thing standing between the change and a robot that can only drive
    in straight lines (12 S6A.4.1's own correction records that happening).
    """
    rns = _rns()
    _feed_grid(rns, free_r_m=0.4, blocked=(0.9, 0.0))   # a hostile ring
    ring = _ring(_Holder(rns), _Pose(), _snap())
    ev = apply_rotation_permit(vx_mps=1.5, vy_mps=0.0, wz_radps=0.3,
                               source="rns_avoid", limits=rot_limits(),
                               r_robot_m=R_ROBOT, ring=ring)
    assert ev.spin_like is False
    assert ev.decision == DECISION_PASS
    assert ev.wz_out == pytest.approx(0.3)


# ---------------------------------------------------------------------------
# Criterion 2 -- safety. Something IN the ring still refuses, whatever else is
# unobserved around it.
# ---------------------------------------------------------------------------

def test_a_blocked_cell_beside_the_body_still_vetoes():
    """The scenario 11 S10.3.2 R-3 is written about, reproduced from the grid.

    A person standing 0.9 m off the flank, in a ring whose rear is unobserved.
    The correction must not have turned this into a clamped turn: the reason
    has to be the occupied cells, not the unknown ones, and the decision has to
    be the veto.

    mutant: count BLOCKED as unknown in ring_counts (or put OCCUPIED_CELLS in
    BLIND_REASONS) -> the machine sweeps 180 degrees through them at 0.3 rad/s.
    """
    rns = _rns()
    _feed_grid(rns, free_r_m=2.5, blocked=(0.0, 0.9))
    ring = _ring(_Holder(rns), _Pose(), _snap())
    assert ring.occ_cells >= 1
    ev = _permit(ring)
    assert ev.reason == REASON_OCCUPIED_CELLS
    assert ev.decision == DECISION_REJECT
    assert ev.wz_out == 0.0
    # and the field is told WHERE: 12 S6A.8 OB-2's blocked_sector_deg, body
    # frame, with the robot facing +x so the obstacle is off the left flank.
    # Asserted as a quadrant rather than an exact bin: the reported bearing is
    # the CELL CENTRE's, which lands a few degrees off the written point
    # (0.25 m cells), and pinning the bin would make this red on any change of
    # resolution without anything actually being wrong.
    assert ring.blocked_sector_deg
    assert any(45.0 <= lo and hi <= 135.0 for lo, hi in ring.blocked_sector_deg), \
        ring.blocked_sector_deg


def test_stale_memory_cannot_vouch_for_the_ring():
    """A ring observed once and then not looked at again decays to unobserved.

    This is 12 S15 #51 at the wiring level: the grid would happily read those
    cells FREE for ttl_static_s (300 s), and RCG-4's bound never priced more
    than grid_age_max_ms. A census that let memory grant the permit would make
    the gate depend on how long ago the robot drove past.
    """
    rns = _rns()
    _feed_grid(rns, now_ms=NOW)
    lim = rot_limits()
    fresh = _ring(_Holder(rns), _Pose(), _snap())
    later = NOW + lim.grid_age_max_ms + 1
    stale = _ring(_Holder(rns), _Pose(), _snap(capture_ms=later - 20), now_ms=later)
    assert fresh.unknown_cells == 0
    assert stale.unknown_cells == stale.total_cells
    assert stale.total_cells == fresh.total_cells


# ---------------------------------------------------------------------------
# Criterion 4 -- "no data" is not "I looked and could not see".
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("broken", ["no_pose", "no_yaw", "no_profile",
                                    "never_ingested", "no_r_robot"])
def test_no_ring_is_produced_when_the_question_cannot_be_answered(broken):
    """Five ways the ring is genuinely unanswerable, all of them a veto.

    Every one of these is easy to turn into a clamp by accident -- returning an
    all-unknown census instead of None is a one-line difference and looks
    tidier. It is also a fail-open with no floor under it: with no pose there
    is no annulus, with no ingest there is no observation of any kind, and 0.3
    rad/s of blind rotation is not a defensible answer to either.

    no_r_robot is here for a second reason: RCG-1 would refuse anyway, but
    building the annulus off r_robot_fallback_m would put the trigger radius
    into r_check, which 12 S6A.4.1 iron rule (1) forbids by name.
    """
    rns = _rns()
    _feed_grid(rns)
    pose, snap, holder = _Pose(), _snap(), _Holder(rns)
    if broken == "no_pose":
        pose = _Pose(xy=None)
    elif broken == "no_yaw":
        pose = _Pose(yaw=None)
    elif broken == "no_profile":
        snap = snapshot(profile=None, objects=None,
                        status=healthy_status(t_publish_mono_ms=NOW - 50))
    elif broken == "never_ingested":
        rns._grid_ingest_ms = None
    elif broken == "no_r_robot":
        holder = _Holder(rns, r_robot=None)

    assert _ring(holder, pose, snap, NOW) is None
    ev = _permit(None)
    assert ev.decision == DECISION_REJECT
    assert ev.reason == REASON_NO_RING_SOURCE


def test_frame_unknown_ratio_reads_the_profile_not_the_ring():
    """The whole-frame sanity check must not be re-derived from the annulus.

    A ring-derived ratio sits above rot_unknown_ratio_max (0.5) on every tick
    of this machine, because the rear is never observed. That would make the
    unknown_ratio conjunct permanently false -- a hidden hard refusal that
    cancels the RCG-3 correction while reading like a sanity check. Taking it
    from the profile's own None fraction asks the same question of the frame
    that actually arrived.

    mutant: return counts.unknown / counts.total -> the first assertion goes
    red, because a fully observed frame would report the rear as frame noise.
    """
    holder = _Holder(_rns())
    clean = NavRuntime._frame_unknown_ratio(holder, _snap())
    assert clean == pytest.approx(0.0)
    n_bins = len(_snap().profile.d_free)
    half = NavRuntime._frame_unknown_ratio(holder, _snap(unknown_bins=n_bins // 2))
    assert 0.4 < half < 0.6
    assert NavRuntime._frame_unknown_ratio(holder, None) is None


def test_ring_age_follows_the_grid_write_not_the_newest_frame():
    """RC-3's operand is how old the GRID is, not how old perception is.

    They diverge whenever RNS is not ingesting -- suspended by estop or local
    teleop, or with no mission loaded -- and in that window perception keeps
    arriving while nobody writes it down. Reporting perception's age would call
    the census fresh when every cell in it is minutes old, which is the one
    thing RC-3 exists to prevent.

    mutant: age_ms = now - profile.t_capture_mono_ms -> the assertion below
    reports ~20 ms for a grid that has not been touched in a second.
    """
    rns = _rns()
    _feed_grid(rns, now_ms=NOW)
    later = NOW + 1000
    ring = _ring(_Holder(rns), _Pose(), _snap(capture_ms=later - 20), now_ms=later)
    assert ring.age_ms == pytest.approx(1000, abs=1)


def test_the_wiring_reads_down_and_rns_does_not_know_about_rotation():
    """12 S15 #50's guard rails, as a check that survives a refactor.

    The layer crossing is accepted because MemoryGrid is the only per-cell
    structure there is; what keeps it reviewable is the direction. If rns/ ever
    imports rotation/, the navigator has started making decisions for the
    safety gate, and the "read-only query returning a neutral value object"
    story stops being true.
    """
    # Parsed, not grepped. Two reasons, and the second one bit while this test
    # was being written: "rotation" is ordinary RNS prose for the physical act
    # of turning (backup.py weighs in-place rotation against a backup step), so
    # a text scan is permanently red; and the scan would match the sentence in
    # grid.py that SAYS rns must not import rotation -- a criterion whose own
    # wording is inside its scanning surface, which CLAUDE.md 3.2 names as
    # "judgement self-harm" and whose usual end is a relaxed check that catches
    # nothing.
    rns_dir = _ROOT / "xbrain" / "p1_motion" / "rns"
    seen = 0
    for path in sorted(rns_dir.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        seen += 1
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                assert "rotation" not in (node.module or ""), path.name
                assert all(a.name != "RingSample" for a in node.names), path.name
            elif isinstance(node, ast.Import):
                assert all("rotation" not in a.name for a in node.names), path.name
    assert seen > 5, "the rns package glob matched almost nothing"
    nav = (_ROOT / "xbrain" / "p1_motion" / "runtime" / "nav_wiring.py").read_text(
        encoding="utf-8")
    assert "ring=self._ring_sample(pose, snap, now_ms)" in nav
    assert "self._src.rns.ring_counts(" in nav
