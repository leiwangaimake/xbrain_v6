"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_nav_tick_rotation.py
Brief: 12 S2.2 step 6b rotation permit -- both failure directions, judge + wired tick

Description:
This gate is the only thing in V6 that constrains wz (12 S6A.1 walks the other
six layers and each is a no-op on it), and it can fail in two opposite ways,
so the file asserts both and neither half is optional.

Too little: a spin with no clearance goes through and the machine sweeps 180
degrees past whoever is standing off its flank (11 S10.3.2 R-3).

Too much: the permit fires on travel that merely turns, and then patrol cannot
steer at all. 12 S6A.4.1's own correction table records that happening -- the
v0.3 trigger made path_follow read as a sweep on EVERY tick with an
uncalibrated body, which left the robot able to drive only in straight lines,
and 12 S14 trap 19 predicts the result: a gate like that gets switched off in
the field. So test_path_follow_turn_is_not_spin_like and its wired twin are
load-bearing, not decoration.

The four criteria, and what each one would miss on its own:
  1  spin + insufficient clearance -> blocked        (catches "too little")
  2  spin + clean full ring        -> permitted      (catches a return-False stub:
                                                      the permit refuses on this
                                                      machine for want of a ring
                                                      source, so without this the
                                                      judge could BE a stub)
  3  path_follow turn vx=1.5 wz=0.3 -> untouched     (catches "too much")
  4  r_robot uncalibrated          -> never permitted (RCG-1; catches reading 0.0
                                                      as "zero radius, so safe")

Attribution is asserted too, and asserted NEGATIVELY against gate.limiter:
12 S6A.8 OB-1 forbids widening that closed set because every value in it
describes a LINEAR cap, so the correct limiter for a rotation refusal is
whatever the speed gate already said, unchanged. The rotation story leaves as
an event instead (OB-2).
"""
from __future__ import annotations

import copy
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from tests.p1_motion.nav.rot_fixture import rot_limits
from tests.p1_motion.rns.scenes import healthy_status, snapshot, uniform_free
from xbrain.common.enums import GATE_LIMITER
from xbrain.p1_motion.nav.health_factor import HealthView
from xbrain.p1_motion.nav.nav_tick import NavInputs, NavTick, NavTickConfigError
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rotation.rcg import (
    BLIND_REASONS,
    DECISION_LIMIT,
    DECISION_PASS,
    DECISION_REJECT,
    KIND_CLEARANCE_UNCONFIGURED,
    KIND_ROTATION_BLOCKED,
    LIMIT_SOURCES,
    REASON_EMPTY_DOMAIN,
    REASON_GRID_STALE,
    REASON_MARGIN_BELOW_BOUND,
    REASON_NO_RING_SOURCE,
    REASON_OCCUPIED_CELLS,
    REASON_PERMITTED,
    REASON_R_ROBOT_UNCALIBRATED,
    REASON_SECTORS_BELOW_R_CHECK,
    REASON_SECTORS_UNAVAILABLE,
    REASON_SELF_MASK_GE_R_CHECK,
    REASON_UNKNOWN_CELLS,
    REASON_UNKNOWN_RATIO,
    VETO_SOURCES,
    RingSample,
    RotationConfigError,
    RotationLimits,
    apply_rotation_permit,
    effective_radius,
    evaluate_ring,
    rcg4_lower_bound,
    ring_check_radius,
    spin_like,
)
from xbrain.p1_motion.sources.arbiter_p1 import BehaviorSource, P1Arbiter
from xbrain.p1_motion.sources.rns_avoid import RnsAvoidSource

pytestmark = pytest.mark.no_device

_ROOT = Path(__file__).resolve().parents[3]
_CFG = yaml.safe_load((_ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))

OK = HealthView(1.0, True, "patrol", "ok", 100)

# The calibrated body radius 12 S6A.3.3 records (2026-08-04, derived from the
# M20S manual). It is a TEST value only: configs/ carries no r_robot key, and
# CLAUDE.md iron rule 3 forbids landing one just to make something run. Its job
# here is to drive the criteria that need a calibrated body, so that "permitted
# is reachable" and "uncalibrated is never permitted" are two different runs of
# the same code rather than two different code paths.
R_ROBOT_CALIBRATED = 0.482


def _clean_ring(r_check: float, *, occ: int = 0, unknown: int = 0,
                age_ms: int = 100, total: int = 800,
                sectors_min_m: float = 99.0) -> RingSample:
    """A ring that satisfies every conjunct unless a caller spoils one.

    total defaults to ~800 because that is the cell count 12 S2.2's latency row
    computes for r_check at this scale; sectors_min defaults far away so the
    cross-veto is not what a clearance test accidentally trips on.
    """
    return RingSample(occ_cells=occ, unknown_cells=unknown, total_cells=total,
                      unknown_ratio=0.1, age_ms=age_ms, resolution_m=0.1,
                      sectors_min_m=sectors_min_m,
                      blocked_sector_deg=((80.0, 100.0),) if occ or unknown else ())


def _permit(vx, vy, wz, *, source="rns_avoid", r_robot=R_ROBOT_CALIBRATED,
            ring=None, limits=None):
    lim = limits if limits is not None else rot_limits()
    return apply_rotation_permit(vx_mps=vx, vy_mps=vy, wz_radps=wz, source=source,
                                 limits=lim, r_robot_m=r_robot, ring=ring)


# ---------------------------------------------------------------------------
# Criterion 3 FIRST, deliberately. It is the one a fail-safe written with too
# much enthusiasm breaks, and putting it at the top of the file is a reminder
# that "blocked more" is not automatically "safer" here.
# ---------------------------------------------------------------------------

def test_path_follow_turn_is_not_spin_like():
    """12 S6A.4.1 substitution row: vx=2.00, wz=0.30 -> R=6.667 m, no trigger.

    The task's stated case is vx=1.5, wz=0.3 -> R = 5.0 m, and the threshold is
    k_rot * r_eff = 0.5 * 0.482 = 0.241 m on a calibrated body. Both are two
    orders of magnitude clear of it. Asserted on the criterion directly so the
    failure message says "this turn was called a spin" rather than surfacing
    three layers up as a stuck robot.
    """
    lim = rot_limits()
    r_eff = effective_radius(R_ROBOT_CALIBRATED, lim.r_robot_fallback_m)
    assert spin_like(1.5, 0.0, 0.3, wz_eps_radps=lim.wz_eps_radps,
                     k_rot=lim.k_rot, r_eff_m=r_eff) is False
    # The document's own three non-triggering rows, checked verbatim.
    for vx, wz in ((2.00, 0.30), (0.50, 1.00), (0.20, 0.30)):
        assert spin_like(vx, 0.0, wz, wz_eps_radps=lim.wz_eps_radps,
                         k_rot=lim.k_rot, r_eff_m=r_eff) is False


def test_path_follow_turn_passes_the_permit_untouched():
    """Criterion 3 at the permit level: wz survives, and no event is raised.

    The no-event half matters as much as the wz: a warn per tick on normal
    patrol steering buries the real ones, which is the mechanism by which
    12 S14 trap 19 says a gate gets turned off in the field.
    """
    ev = _permit(1.5, 0.0, 0.3, ring=None)
    assert ev.spin_like is False
    assert ev.wz_out == pytest.approx(0.3)
    assert ev.decision == DECISION_PASS
    assert ev.event_kind is None


def test_path_follow_turn_passes_even_with_r_robot_uncalibrated():
    """The v0.3 fail-safe-too-far case, pinned so it cannot come back.

    12 S6A.4.1's correction: the retired trigger had an r_robot <= 0 disjunct,
    which on an uncalibrated body made spin_like identically "|wz| > wz_eps".
    Every path_follow tick then read as a sweep and the robot could only drive
    straight. r_robot=None here IS the uncalibrated body, and the turn must
    still go through: RCG-1 belongs on the verdict, never on the trigger.
    """
    ev = _permit(1.5, 0.0, 0.3, r_robot=None, ring=None)
    assert ev.spin_like is False
    assert ev.wz_out == pytest.approx(0.3)


# ---------------------------------------------------------------------------
# Criterion 1: a spin with no clearance is blocked.
# ---------------------------------------------------------------------------

def test_pure_spin_triggers_even_with_a_little_vx():
    """12 S6A.4.1's bypass row: vx=0.04 with wz=1.5 is still a sweep.

    A linear-velocity deadzone trigger would wave this through -- emit just
    over v_eps and spin as hard as you like. The curvature radius is 0.027 m,
    i.e. the rotation centre is 2.7 cm from base_link, well inside the body.
    """
    lim = rot_limits()
    r_eff = effective_radius(R_ROBOT_CALIBRATED, lim.r_robot_fallback_m)
    assert spin_like(0.04, 0.0, 1.5, wz_eps_radps=lim.wz_eps_radps,
                     k_rot=lim.k_rot, r_eff_m=r_eff) is True
    assert spin_like(0.0, 0.0, 1.5, wz_eps_radps=lim.wz_eps_radps,
                     k_rot=lim.k_rot, r_eff_m=r_eff) is True


def test_spin_with_occupied_cell_is_blocked():
    """One occupied cell in the ring refuses the spin (rot_occ_max is 0)."""
    r_check = R_ROBOT_CALIBRATED + rot_limits().margin_rot_m
    ev = _permit(0.0, 0.0, 1.5, source="nav2_proxy",
                 ring=_clean_ring(r_check, occ=1))
    assert ev.spin_like is True
    assert ev.decision == DECISION_REJECT
    assert ev.reason == REASON_OCCUPIED_CELLS
    assert ev.wz_out == 0.0
    assert ev.event_kind == KIND_ROTATION_BLOCKED
    # RCE-2 makes both of these mandatory in the reject detail, and a
    # fabricated radius there would read as a real measurement.
    assert ev.r_check_m == pytest.approx(r_check)
    assert ev.occ_cells == 1


@pytest.mark.parametrize("source", sorted(LIMIT_SOURCES | VETO_SOURCES))
def test_spin_with_unknown_cell_is_clamped_not_vetoed(source):
    """RCG-3 as corrected 2026-09-29: unknown -> blind clamp, for EVERY source.

    Availability half of the gate, and it is not a nicety. The machine has no
    LiDAR and the RGBD covers the forward half, so the rear of the ring is
    unobserved on every tick of every mission. Under the pre-correction rule
    that is a permanent veto: 18 A09..A12 all reach the chassis through
    nav2_proxy, which 12 S6A.4.2 puts on the veto branch, so "turn around"
    could never execute -- and seeing behind requires turning, which requires
    seeing behind.

    Parametrised over the WHOLE source closed set on purpose. An implementation
    that left the blind case under 12 S6A.4.2's table would pass a test written
    against rns_avoid alone and still veto every autonomous turn, which is the
    only path an operator actually uses (12 S15 #52 records the cost of
    crossing that table, including teleop_cloud).

    mutant: decision = _disposal_for(source) for blind reasons too -> every
    VETO_SOURCES parameter goes red while rns_avoid stays green, which is
    exactly the half-fix this parametrisation exists to catch.
    """
    r_check = R_ROBOT_CALIBRATED + rot_limits().margin_rot_m
    lim = rot_limits()
    ev = _permit(0.0, 0.0, 1.5, source=source,
                 ring=_clean_ring(r_check, unknown=1), limits=lim)
    assert ev.reason == REASON_UNKNOWN_CELLS
    assert ev.decision == DECISION_LIMIT
    assert ev.wz_out == pytest.approx(lim.wz_blind_radps)
    # The sign of the operator's turn must survive the clamp: 12 S6A.4.2's
    # reason for clamping rather than zeroing is that the turn is how you get
    # OUT of the situation, and a clamp that dropped the sign would turn the
    # wrong way.
    back = _permit(0.0, 0.0, -1.5, source=source,
                   ring=_clean_ring(r_check, unknown=1), limits=lim)
    assert back.wz_out == pytest.approx(-lim.wz_blind_radps)
    # Still a warn, not a fault: turning is what clears it, so a retry helps.
    assert ev.event_kind == KIND_ROTATION_BLOCKED


def test_blind_clamp_is_not_a_permit():
    """A clamped tick is still a refusal -- wz is reduced, never passed.

    The failure this pins is subtle and would look like a success in a log: an
    implementation that treated "blind" as "permitted" would leave wz at its
    input value and still report decision limit nowhere. 12 S6A.2's key
    property is that limiting does not shrink the swept annulus, so the clamp
    only buys time to react; it is not permission.
    """
    r_check = R_ROBOT_CALIBRATED + rot_limits().margin_rot_m
    lim = rot_limits()
    v = evaluate_ring(lim, R_ROBOT_CALIBRATED, _clean_ring(r_check, unknown=1))
    assert v.permitted is False
    ev = _permit(0.0, 0.0, 1.5, source="nav2_proxy",
                 ring=_clean_ring(r_check, unknown=1), limits=lim)
    assert ev.wz_out < ev.wz_in
    assert ev.decision != DECISION_PASS


def test_a_hard_refusal_outranks_a_blind_one():
    """"There is something there" must never be reported as "I cannot see".

    Since the two now have different dispositions, the order between the hard
    and blind conjuncts decides whether the tick vetoes or turns at 0.3 rad/s.
    Three rings, each tripping one hard conjunct AND the blind one at the same
    time; every one must name the hard reason.

    mutant: put the unknown_cells test back above unknown_ratio (its position
    before 2026-09-29) -> the unknown_ratio row below goes red.
    """
    lim = rot_limits()
    r_check = R_ROBOT_CALIBRATED + lim.margin_rot_m
    both_cells = _clean_ring(r_check, occ=1, unknown=1)
    bad_frame = replace(_clean_ring(r_check, unknown=1),
                        unknown_ratio=lim.rot_unknown_ratio_max + 0.1)
    near_sector = replace(_clean_ring(r_check, unknown=1),
                          sectors_min_m=r_check - 0.01)
    for ring, want in ((both_cells, REASON_OCCUPIED_CELLS),
                       (bad_frame, REASON_UNKNOWN_RATIO),
                       (near_sector, REASON_SECTORS_BELOW_R_CHECK)):
        v = evaluate_ring(lim, R_ROBOT_CALIBRATED, ring)
        assert v.reason == want, "blind reason outranked a hard one: %s" % v.reason
        assert v.reason not in BLIND_REASONS
        ev = _permit(0.0, 0.0, 1.5, source="nav2_proxy", ring=ring, limits=lim)
        assert ev.decision == DECISION_REJECT and ev.wz_out == 0.0


def test_spin_with_occupied_cell_is_still_vetoed_for_autonomous_sources():
    """Safety half: the 2026-09-29 correction touched unknown and nothing else.

    A ring with something IN it keeps the pre-correction disposition exactly --
    veto for an autonomous source, clamp for the local-teleop / rns_avoid
    exemption of 12 S6A.4.2. Written next to the blind test so that a change
    which relaxed both at once cannot look like a change that relaxed one.
    """
    lim = rot_limits()
    r_check = R_ROBOT_CALIBRATED + lim.margin_rot_m
    ring = _clean_ring(r_check, occ=1)
    for source in sorted(VETO_SOURCES):
        ev = _permit(0.0, 0.0, 1.5, source=source, ring=ring, limits=lim)
        assert ev.reason == REASON_OCCUPIED_CELLS
        assert ev.decision == DECISION_REJECT and ev.wz_out == 0.0
    for source in sorted(LIMIT_SOURCES):
        ev = _permit(0.0, 0.0, 1.5, source=source, ring=ring, limits=lim)
        assert ev.decision == DECISION_LIMIT
        assert ev.wz_out == pytest.approx(lim.wz_blind_radps)


@pytest.mark.parametrize("source", sorted(VETO_SOURCES))
def test_spin_with_no_ring_source_is_vetoed_not_clamped(source):
    """"No data" and "the data says I cannot see" are two different answers.

    Both end in "the permit does not pass", which is exactly why they are easy
    to merge -- and merging them is a fail-open: an absent RingSample means
    nothing at all is known about the ring, not even that the near cells are
    clear, so there is no basis for the 0.3 rad/s that a blind tick gets.

    Parametrised over VETO_SOURCES, which is where the two answers actually
    diverge. The local-teleop family and rns_avoid are clamped here too, but by
    12 S6A.4.2's per-source exemption rather than by the blind branch -- an
    unchanged, pre-2026-09-29 path, pinned in the companion assertion below so
    that "they look the same from outside" cannot hide a merge of the two.

    Reported as unconfigured rather than as a stale grid, because waiting will
    not produce a source.

    mutant: add REASON_NO_RING_SOURCE to BLIND_REASONS -> every parameter here
    goes red, which is the point: a process with no ring producer at all would
    otherwise start turning.
    """
    ev = _permit(0.0, 0.0, 1.5, source=source, ring=None)
    assert ev.decision == DECISION_REJECT
    assert ev.reason == REASON_NO_RING_SOURCE
    assert ev.reason not in BLIND_REASONS
    assert ev.event_kind == KIND_CLEARANCE_UNCONFIGURED
    assert ev.wz_out == 0.0
    # The exempt sources are clamped by the TABLE, not by the blind branch:
    # same wz, different reason, and the fault kind says which.
    for exempt in sorted(LIMIT_SOURCES):
        alt = _permit(0.0, 0.0, 1.5, source=exempt, ring=None)
        assert alt.decision == DECISION_LIMIT
        assert alt.event_kind == KIND_CLEARANCE_UNCONFIGURED
        assert alt.detail_item == REASON_NO_RING_SOURCE


def test_spin_with_stale_grid_is_vetoed_not_clamped():
    """RC-3 (12 S6A.5): older than grid_age_max_ms refuses, never uses.

    Same family as the absent source: a dead perception channel is "no data",
    not "I looked and saw nothing there", so it keeps the veto. The two are
    told apart by whether a RingSample arrived at all, and both must stay out
    of BLIND_REASONS or a perception outage would turn into a licence to spin
    slowly.
    """
    lim = rot_limits()
    r_check = R_ROBOT_CALIBRATED + lim.margin_rot_m
    ev = _permit(0.0, 0.0, 1.5, source="nav2_proxy",
                 ring=_clean_ring(r_check, age_ms=lim.grid_age_max_ms + 1))
    assert ev.decision == DECISION_REJECT
    assert ev.reason == REASON_GRID_STALE
    assert ev.reason not in BLIND_REASONS


def test_sectors_cross_veto_unavailable_is_blind_not_an_abstention():
    """12 S6A.3.2's cross-veto with no value: refuses the permit, then clamps.

    The conjunct is NOT skipped -- that would silently drop half the
    conjunction -- but since 2026-09-29 it lands in the blind branch. The
    reason it had to move with RCG-3: free_space.sectors has no producer
    anywhere in the repository, so sectors_min_m is None on every tick, and one
    permanently-false hard conjunct cancels the whole correction by itself.
    11 S3.1.5.4's own table already reads this case as blind.
    """
    # Built inline rather than via _clean_ring because sectors_min_m=None is
    # the whole point here and a helper default would hide it.
    lim = rot_limits()
    ring = RingSample(occ_cells=0, unknown_cells=0, total_cells=800,
                      unknown_ratio=0.1, age_ms=100, resolution_m=0.1,
                      sectors_min_m=None)
    v = evaluate_ring(lim, R_ROBOT_CALIBRATED, ring)
    assert v.permitted is False
    assert v.reason == REASON_SECTORS_UNAVAILABLE
    ev = _permit(0.0, 0.0, 1.5, source="nav2_proxy", ring=ring, limits=lim)
    assert ev.decision == DECISION_LIMIT
    assert ev.wz_out == pytest.approx(lim.wz_blind_radps)


# ---------------------------------------------------------------------------
# Criterion 2: a clean ring permits. Without this the judge could be a stub.
# ---------------------------------------------------------------------------

def test_spin_with_clean_ring_is_permitted():
    """Every conjunct satisfied -> the spin goes through with wz untouched.

    On this machine the permit always refuses, for three independent reasons
    (no ring source, no calibrated r_robot, no full-circle sectors). A judge
    that simply returned False would satisfy every other test in this file.
    This one is what makes those tests mean anything.
    """
    r_check = R_ROBOT_CALIBRATED + rot_limits().margin_rot_m
    ev = _permit(0.0, 0.0, 1.5, source="nav2_proxy",
                 ring=_clean_ring(r_check, sectors_min_m=r_check + 0.01))
    assert ev.spin_like is True
    assert ev.decision == DECISION_PASS
    assert ev.reason == REASON_PERMITTED
    assert ev.wz_out == pytest.approx(1.5)
    assert ev.event_kind is None


def test_sectors_exactly_at_r_check_still_permits():
    """The cross-veto is ">= r_check", so equality passes.

    Pinned because flipping it to ">" would make the boundary refuse, and a
    boundary that refuses is easy to mistake for correct caution while it is
    really a different rule than the one written down.
    """
    r_check = R_ROBOT_CALIBRATED + rot_limits().margin_rot_m
    v = evaluate_ring(rot_limits(), R_ROBOT_CALIBRATED,
                      _clean_ring(r_check, sectors_min_m=r_check))
    assert v.permitted is True


# ---------------------------------------------------------------------------
# Criterion 4: RCG-1 on the TRUE r_robot.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("r_robot", [None, 0.0, -0.1])
def test_uncalibrated_r_robot_never_permits(r_robot):
    """RCG-1: 0.0 means "not known, so do not turn", never "zero radius".

    Read the other way, r_check collapses to margin_rot alone, the decision
    domain shrinks to a few cells round the origin, and the permit passes
    nearly everything -- 12 S6A.3.3 calls that the section's most likely and
    most costly fail-open. The ring handed in here is otherwise perfect, so the
    only thing that can refuse it is RCG-1.
    """
    r_check = R_ROBOT_CALIBRATED + rot_limits().margin_rot_m
    ring = _clean_ring(r_check, sectors_min_m=r_check + 0.01)
    v = evaluate_ring(rot_limits(), r_robot, ring)
    assert v.permitted is False
    assert v.reason == REASON_R_ROBOT_UNCALIBRATED
    # r_check is undefined without a true r_robot, and echoing None rather than
    # a plausible number is the point: RCE-2 makes r_check_m mandatory in the
    # detail, so a fabricated one would travel as a measurement.
    assert v.r_check_m is None
    assert ring_check_radius(r_robot, rot_limits().margin_rot_m) is None


def test_r_eff_never_reaches_r_check():
    """12 S6A.4.1 iron rule (1), asserted as an inequality on real numbers.

    The fallback exists so the TRIGGER keeps working on an uncalibrated body.
    If it leaked into r_check, an uncalibrated machine would be judged against
    a 1.60 m ring nobody measured, and RCG-1 would have nothing left to refuse.
    """
    lim = rot_limits()
    assert effective_radius(None, lim.r_robot_fallback_m) == pytest.approx(0.60)
    assert ring_check_radius(None, lim.margin_rot_m) is None


def test_effective_radius_is_a_branch_not_a_max():
    """12 S12 v0.7 retired max(r_robot, fallback): the two differ below 0.60.

    A measured 0.30 must stay 0.30. max() would push it to 0.60 and double the
    trigger threshold, widening the trigger domain for no reason -- safe in
    direction but a second implementation shape, and the book allows one.
    """
    assert effective_radius(0.30, 0.60) == pytest.approx(0.30)
    assert effective_radius(0.90, 0.60) == pytest.approx(0.90)


# ---------------------------------------------------------------------------
# RCG-2 and RCG-4: the two invariants about the configuration itself.
# ---------------------------------------------------------------------------

def test_self_mask_reaching_r_check_refuses():
    """RCG-2: an annulus with no cells passes count(blocked)==0 vacuously.

    "No cells to check" has to read as "cannot tell". Anyone raising the inner
    radius to hide the robot's own echo off the grid lands here; 12 S6A.2 says
    the correct fix is a per-sensor self mask in perception instead.
    """
    lim = rot_limits()
    bad = RotationLimits(
        margin_rot_m=lim.margin_rot_m, r_self_mask_m=99.0,
        rot_occ_max=lim.rot_occ_max,
        rot_unknown_max_cells=lim.rot_unknown_max_cells,
        rot_unknown_ratio_max=lim.rot_unknown_ratio_max,
        grid_age_max_ms=lim.grid_age_max_ms, recheck_ticks=lim.recheck_ticks,
        wz_eps_radps=lim.wz_eps_radps, k_rot=lim.k_rot,
        r_robot_fallback_m=lim.r_robot_fallback_m,
        ped_speed_mps=lim.ped_speed_mps, allow_visual_override=False,
        wz_blind_radps=None)
    r_check = R_ROBOT_CALIBRATED + lim.margin_rot_m
    v = evaluate_ring(bad, R_ROBOT_CALIBRATED,
                      _clean_ring(r_check, sectors_min_m=r_check + 0.01))
    assert v.permitted is False
    assert v.reason == REASON_SELF_MASK_GE_R_CHECK


def test_empty_annulus_refuses_instead_of_passing_vacuously():
    """RCG-2's data half: |A| > 0, or the verdict is "cannot tell".

    With no cells in range, count(occupied) == 0 and count(unknown) == 0 are
    both true and every cell-count conjunct passes -- so an implementation
    that only ever checked the counts would permit every spin the moment the
    grid stopped producing cells in range. 12 S6A.3.3 puts it as: "no cells to
    check" must read as "cannot tell", never as "clean".

    mutant: delete the total_cells check -> this ring permits -> red.
    """
    lim = rot_limits()
    r_check = R_ROBOT_CALIBRATED + lim.margin_rot_m
    empty = RingSample(occ_cells=0, unknown_cells=0, total_cells=0,
                       unknown_ratio=0.0, age_ms=100, resolution_m=0.1,
                       sectors_min_m=r_check + 0.01)
    v = evaluate_ring(lim, R_ROBOT_CALIBRATED, empty)
    assert v.permitted is False
    assert v.reason == REASON_EMPTY_DOMAIN


def test_rcg4_violation_refuses_the_whole_permit():
    """RCG-4 is a conjunct, not a comment: breaching the bound refuses.

    12 S12 reverse-solves the headroom and states it: with age 200 ms and 3
    recheck ticks, ped_speed may not exceed 2.65 m/s before the bound passes
    the pinned margin_rot of 1.00. At 3.0 m/s the bound is 1.12 m, so "the ring
    was clean this tick" has already expired by the time the tick acts on it,
    and the permit must say so.

    It refuses with its OWN reason, not with a cell count: 12 S6A.5 is explicit
    that the attribution moves from "something is in the ring" to "the config
    is not self-consistent", and those send the field to different places.

    mutant: drop the RCG-4 comparison -> this ring permits on stale data -> red.
    """
    lim = replace(rot_limits(), ped_speed_mps=3.0)
    r_check = R_ROBOT_CALIBRATED + lim.margin_rot_m
    assert rcg4_lower_bound(lim, 0.1) > lim.margin_rot_m      # the premise
    v = evaluate_ring(lim, R_ROBOT_CALIBRATED,
                      _clean_ring(r_check, sectors_min_m=r_check + 0.01))
    assert v.permitted is False
    assert v.reason == REASON_MARGIN_BELOW_BOUND


def test_rcg4_bound_matches_the_document_arithmetic():
    """12 S6A.3.3's worked bound: 1.5*(0.200 + 3*0.050) + 0.5*0.10*sqrt(2).

    = 0.596 m, and margin_rot is pinned at 1.00 by U54 rule 3, so the invariant
    holds with 0.40 m to spare. Pinned as a number because the bound is what
    decides whether tightening recheck_ticks or ped_speed silently turns the
    permit into a permanent refusal.
    """
    lim = rot_limits()
    assert rcg4_lower_bound(lim, 0.1) == pytest.approx(0.5957, abs=1e-3)
    assert lim.margin_rot_m >= rcg4_lower_bound(lim, 0.1)


def test_allow_visual_override_true_refuses_construction():
    """12 S6A.6's fourth condition cannot be met at the exit layer.

    The bypass has to name the confirming command's origin and cmd_id in its
    event, and step 6b sees a velocity, not a command. 12 S6A.6 says that
    without that record the switch is a silent safety-release channel, so
    construction refuses rather than releasing rotation unrecorded.
    """
    lim = rot_limits()
    with pytest.raises(RotationConfigError, match="allow_visual_override"):
        RotationLimits(
            margin_rot_m=lim.margin_rot_m, r_self_mask_m=lim.r_self_mask_m,
            rot_occ_max=lim.rot_occ_max,
            rot_unknown_max_cells=lim.rot_unknown_max_cells,
            rot_unknown_ratio_max=lim.rot_unknown_ratio_max,
            grid_age_max_ms=lim.grid_age_max_ms,
            recheck_ticks=lim.recheck_ticks, wz_eps_radps=lim.wz_eps_radps,
            k_rot=lim.k_rot, r_robot_fallback_m=lim.r_robot_fallback_m,
            ped_speed_mps=lim.ped_speed_mps, allow_visual_override=True,
            wz_blind_radps=None)


# ---------------------------------------------------------------------------
# 12 S6A.4.2: per-source disposal, and the degrade when the clamp is missing.
# ---------------------------------------------------------------------------

def test_rns_avoid_is_clamped_not_vetoed_when_the_clamp_value_exists():
    """The section's single veto exemption, and the reason it exists.

    Zeroing the turn of the one source whose motive is to get AWAY from the
    obstacle is the safety gate used backwards. RNS reaches this state for
    real: its contact fuse zeroes v on a close hit and lets the turn continue
    so the upper layers can re-plan, which is precisely a spin_like tick.
    """
    r_check = R_ROBOT_CALIBRATED + rot_limits().margin_rot_m
    ev = _permit(0.0, 0.0, 1.5, source="rns_avoid",
                 ring=_clean_ring(r_check, occ=1),
                 limits=rot_limits(wz_blind_radps=0.3))
    assert ev.decision == DECISION_LIMIT
    assert ev.wz_out == pytest.approx(0.3)
    assert ev.event_kind == KIND_ROTATION_BLOCKED


def test_local_teleop_is_clamped_and_cloud_teleop_is_vetoed():
    """v0.7.9 narrowed the human-in-the-loop exemption to LOCAL sources.

    The pad and the keyboard are beside the robot; the cloud operator has a
    delayed video feed and not even that premise. Both in one test so the
    distinction cannot be half-deleted.
    """
    r_check = R_ROBOT_CALIBRATED + rot_limits().margin_rot_m
    ring = _clean_ring(r_check, occ=1)
    lim = rot_limits(wz_blind_radps=0.3)
    local = _permit(0.0, 0.0, 1.5, source="teleop_joystick", ring=ring, limits=lim)
    cloud = _permit(0.0, 0.0, 1.5, source="teleop_cloud", ring=ring, limits=lim)
    assert local.decision == DECISION_LIMIT and local.wz_out == pytest.approx(0.3)
    assert cloud.decision == DECISION_REJECT and cloud.wz_out == 0.0


def test_missing_clamp_value_degrades_limit_to_veto_and_says_why():
    """12 S12 landing plan (2).

    Verbatim: cannot get the clamp value, then do not let it through, and never
    on a guessed one. The detail must name wz_blind_radps rather than the
    ring's own reason, because the operator's next action is completely
    different -- land a config key, not clear the area.

    None is passed explicitly since 2026-09-29. Before that it was the fixture
    default, because nav_cfg hard-coded None; the key now carries a value and
    is read, so None is a modelled degrade rather than the machine's state.
    Keeping the test means the degrade still cannot be deleted by accident.
    """
    r_check = R_ROBOT_CALIBRATED + rot_limits().margin_rot_m
    ev = _permit(0.0, 0.0, 1.5, source="rns_avoid",
                 ring=_clean_ring(r_check, occ=1),
                 limits=rot_limits(wz_blind_radps=None))
    assert ev.decision == DECISION_REJECT
    assert ev.wz_out == 0.0
    assert ev.event_kind == KIND_CLEARANCE_UNCONFIGURED
    assert ev.detail_item == "wz_blind_radps"


def test_unknown_source_raises_rather_than_defaulting():
    """12 S6A.4.2's rns_avoid row demands the source test be explicit.

    Both defaults are wrong: to LIMIT exempts an unknown source from the veto,
    to VETO silently zeroes one the section may have exempted. CLAUDE.md 3.5
    forbids passing a value outside a closed set through either way.
    """
    with pytest.raises(RotationConfigError, match="S6A.4.2"):
        _permit(0.0, 0.0, 1.5, source="a_source_nobody_ruled_on", ring=None)


def test_every_behavior_source_has_a_disposal():
    """Closed-set parity in both directions (CLAUDE.md 7.1 meta test).

    A new BehaviorSource with no row here would raise at 20 Hz on its first
    spin; a stale name in either set would look like coverage that is not
    there. Both sets are checked against the arbiter's enum, not against a
    list retyped in this file.
    """
    sources = {s.value for s in BehaviorSource}
    assert LIMIT_SOURCES & VETO_SOURCES == frozenset()
    assert (LIMIT_SOURCES | VETO_SOURCES) == sources


def test_rotation_reasons_never_enter_the_gate_limiter_closed_set():
    """12 S6A.8 OB-1, asserted as a disjointness rather than trusted.

    Every gate.limiter value describes a LINEAR cap and 11 S9.6.5's argmax over
    the reduction Delta has no meaning for wz, so a rotation token appearing in
    that set would break the attribution for every other limiter too. OB-4
    registers gate.wz_limiter as the proper home, not yet ruled by 11.
    """
    reasons = {REASON_NO_RING_SOURCE, REASON_R_ROBOT_UNCALIBRATED,
               REASON_OCCUPIED_CELLS, REASON_UNKNOWN_CELLS, REASON_GRID_STALE,
               REASON_SECTORS_UNAVAILABLE, REASON_SELF_MASK_GE_R_CHECK}
    assert reasons & set(GATE_LIMITER) == set()
    assert "rotation" not in set(GATE_LIMITER)


# ---------------------------------------------------------------------------
# The wired tick: the same criteria through NavTick at step 6b.
# ---------------------------------------------------------------------------

def _stack(*, wz_blind_radps=None, r_robot=None):
    """The production RnsSource behind NavTick, as test_nav_tick.py builds it."""
    cfg = copy.deepcopy(_CFG)
    rns = RnsSource(cfg=cfg, r_eff_m=0.5)
    src = RnsAvoidSource(rns, cfg["rns"])
    tick = NavTick(src, P1Arbiter(dwell_ms=200), v_nom_mps=1.0,
                   speed_up_hold_ms=3000, d_up_margin_m=0.5, wz_max_rps=1.2,
                   spec_max_vx_mps=2.0, holonomic=True,
                   rot_limits=rot_limits(wz_blind_radps=wz_blind_radps),
                   r_robot_m=r_robot)
    return src, tick


def _inp(now, ring=None):
    perception = snapshot(
        uniform_free(6.0, t_capture_mono_ms=now, t_seg_mono_ms=now - 10),
        None, healthy_status(t_publish_mono_ms=now))
    return NavInputs(now_mono_ms=now, pose_xy=(0.0, 0.0), yaw_rad=0.0,
                     heading_valid=True, i_fix=1.0, i_heading=1.0,
                     perception=perception, health=OK, estop=False,
                     teleop_active=False, ring=ring)


def test_tick_runs_the_permit_every_tick_and_reports_it():
    """NavOutput.rotation is always populated, so nothing has to guess.

    With no mission the holder is hold and the velocity is zero, so the tick is
    not spin_like -- but the permit still ran and still says so. An output that
    left the field None on a quiet tick would be indistinguishable from a tick
    where step 6b was skipped.
    """
    _, tick = _stack()
    out = tick.run(_inp(1000))
    assert out.rotation is not None
    assert out.rotation.spin_like is False
    assert out.rotation.decision == DECISION_PASS


class _FixedCandidateSource:
    """A behaviour source that always offers the same velocity.

    The real RnsSource will not hand over a chosen vx / wz on demand -- what it
    emits depends on a mission, a route and a perception frame -- so the two
    tests below would be asserting the planner as much as the permit. This
    stub drives the SEAM instead: arbitration picks it, step 6b sees exactly
    the velocity under test, and the only thing that can change the published
    wz is the permit. It carries the rns_avoid name because that is the source
    12 S6A.4.2 puts on the LIMIT branch, and because the arbiter has a slot for
    it already.
    """

    name = BehaviorSource.RNS_AVOID.value

    def __init__(self, vx, vy, wz):
        from xbrain.common.types import Mps
        from xbrain.p1_motion.rns.types import NavState, VelocityCandidate
        self._cand = VelocityCandidate(vx=Mps(vx), vy=Mps(vy), wz=wz)
        self._idle = NavState.FOLLOW

    def is_active(self, ctx):
        return True

    def compute(self, ctx):
        return self._cand

    def on_preempted(self, ctx):
        return None

    def on_release(self, ctx):
        return None

    def nav_state(self):
        return self._idle


def _tick_with(vx, vy, wz, *, wz_blind_radps=None, r_robot=None, ring=None):
    """One tick over the stub source, returning the NavOutput."""
    tick = NavTick(_FixedCandidateSource(vx, vy, wz), P1Arbiter(dwell_ms=0),
                   v_nom_mps=2.0, speed_up_hold_ms=3000, d_up_margin_m=0.5,
                   wz_max_rps=2.0, spec_max_vx_mps=2.0, holonomic=True,
                   rot_limits=rot_limits(wz_blind_radps=wz_blind_radps),
                   r_robot_m=r_robot)
    return tick.run(_inp(1000, ring=ring))


def test_tick_zeroes_a_spin_like_candidate():
    """Criterion 1, wired: the published wz is 0, not the candidate's 1.5.

    This is the whole point of the change. Before it, a pure spin reached
    cmd_vel untouched and 11 S10.3.2 R-3's scenario -- somebody 0.5 m off the
    flank, a spoken turn-around, a 180 degree sweep -- had nothing in its way.

    The reason is RCG-1 rather than the missing ring, because the conjunction
    is evaluated config-first and r_robot has no key in any loaded tree. Both
    refusals are live on this machine; which one is NAMED is the evaluation
    order, and it is pinned here so the order cannot drift unnoticed -- the
    field reads this string to decide what to go and fix.

    mutant: drop the `wz = rot.wz_out` line in nav_tick.run (i.e. run the
    permit and ignore its answer) -> the candidate's wz is published -> red
    here while every other test in this file still passes, because they all
    assert on the permit's own return value rather than on what the tick
    published.
    """
    out = _tick_with(0.0, 0.0, 1.5)
    assert out.raw_wz == pytest.approx(1.5)      # the source did ask for it
    assert out.wz == 0.0                          # and step 6b took it away
    assert out.rotation.spin_like is True
    assert out.rotation.decision == DECISION_REJECT
    assert out.rotation.reason == REASON_R_ROBOT_UNCALIBRATED


def test_tick_still_refuses_once_the_body_radius_is_calibrated():
    """The second of the three live refusals, reached by closing the first.

    Landing r_robot alone does not make rotation available, and 12 S6A.3.3's
    2026-08-05 note says so in as many words: RCG-1 is one conjunct. With the
    body measured, the next thing missing is the ring itself, and the reason
    string moves on to say so rather than the tick suddenly permitting.
    """
    out = _tick_with(0.0, 0.0, 1.5, r_robot=R_ROBOT_CALIBRATED)
    assert out.wz == 0.0
    assert out.rotation.reason == REASON_NO_RING_SOURCE


def test_tick_leaves_a_path_follow_turn_alone():
    """Criterion 3, wired: vx=1.5 with wz=0.3 reaches the wire intact.

    The counterweight to the test above, and the one that fails if the permit
    is made too eager. 12 S6A.6 row 5 puts this in writing as a consequence
    that MUST be stated -- travelling turns are explicitly not affected -- and
    12 S6A.4.1's correction records what happened when they were: every patrol
    tick read as a sweep and the robot could only drive in straight lines.

    mutant: restore the retired v0.3 trigger (spin_like = |wz| > wz_eps AND
    (r_robot <= 0 OR R < k_rot*r_eff)) -> with r_robot None this turn is called
    a sweep, wz goes to 0 -> red here, while the blocking tests stay green.
    That asymmetry is the whole reason both directions are asserted.
    """
    out = _tick_with(1.5, 0.0, 0.3)
    assert out.raw_wz == pytest.approx(0.3)
    assert out.wz == pytest.approx(0.3)
    assert out.rotation.spin_like is False
    assert out.rotation.decision == DECISION_PASS


def test_tick_permits_a_spin_when_the_ring_is_clean():
    """Criterion 2, wired: with a calibrated body and a clean ring, wz lives.

    Without this the wired half would be satisfied by a step 6b that zeroed wz
    unconditionally, which is a different bug wearing the same test results.
    """
    r_check = R_ROBOT_CALIBRATED + rot_limits().margin_rot_m
    out = _tick_with(0.0, 0.0, 1.5, r_robot=R_ROBOT_CALIBRATED,
                     ring=_clean_ring(r_check, sectors_min_m=r_check + 0.01))
    assert out.rotation.spin_like is True
    assert out.rotation.decision == DECISION_PASS
    assert out.wz == pytest.approx(1.5)


def test_tick_clamps_rns_avoid_rather_than_zeroing_it():
    """12 S6A.4.2's exemption, wired, because it is the row with a real cost.

    RNS reaches a spin_like tick for real: its contact fuse zeroes v on a close
    hit and deliberately lets the turn continue so the upper layers can
    re-plan. Vetoing that turn leaves the machine stopped in front of the thing
    it was turning away from, which 12 S6A.4.2 calls the safety gate used
    backwards. With the clamp value present it turns, slowly.
    """
    r_check = R_ROBOT_CALIBRATED + rot_limits().margin_rot_m
    out = _tick_with(0.0, 0.0, 1.5, wz_blind_radps=0.3,
                     r_robot=R_ROBOT_CALIBRATED,
                     ring=_clean_ring(r_check, occ=1))
    assert out.rotation.decision == DECISION_LIMIT
    assert out.wz == pytest.approx(0.3)


def test_tick_never_moves_the_rotation_story_into_limiter():
    """12 S6A.8 OB-1, asserted where it would actually be violated.

    A refused spin must not change gate.limiter: the value there describes what
    capped the LINEAR speed, and overwriting it would make the recording say
    the wrong thing about vx as well. Compared against a non-spin tick of the
    same stack so the assertion is "unchanged", not "happens to equal a string
    somebody typed here".
    """
    blocked = _tick_with(0.0, 0.0, 1.5)
    assert blocked.wz == 0.0
    assert blocked.limiter in set(GATE_LIMITER)
    assert "rotation" not in blocked.limiter
    assert all(x in set(GATE_LIMITER) for x in blocked.limiter_all)


def test_tick_refuses_construction_without_the_rotation_block():
    """12 S6A.7 RC-D7 gives the permit no off switch, including this one.

    A NavTick that ran without a RotationLimits when the config block was
    absent would be an enabled:false spelled differently -- one missing section
    and the only gate on wz is gone, silently.
    """
    cfg = copy.deepcopy(_CFG)
    src = RnsAvoidSource(RnsSource(cfg=cfg, r_eff_m=0.5), cfg["rns"])
    with pytest.raises(NavTickConfigError, match="RC-D7"):
        NavTick(src, P1Arbiter(), v_nom_mps=1.0, speed_up_hold_ms=3000,
                d_up_margin_m=0.5, wz_max_rps=1.2, spec_max_vx_mps=2.0,
                holonomic=True, rot_limits=None, r_robot_m=None)
