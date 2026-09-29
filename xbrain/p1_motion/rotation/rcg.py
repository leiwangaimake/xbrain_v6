"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rcg.py
Brief: P1 rotation permit -- spin_like trigger, RC-1 ring verdict, RCG-1..RCG-4 (12 S6A)

Description:
The problem this file solves. A pure spin has no geometric safety gate anywhere
else in V6. 12 S6A.1 walks the six other layers one at a time and every one of
them is a no-op on wz: the speed gate's six terms are all linear caps, so
v_max falling to 0 does not make wz any smaller; hard-fence clipping is
v - max(0, v.n).n, which is the identity map when v is zero; the three-stage
limiter bounds how FAST it turns, never how much area it sweeps; the RNS
corridor is a ray cast along the candidate heading, i.e. translation
passability; and Nav2 runs with simulate_ahead_time 0.0, whose own note says
the loop body is never entered so the costmap is not consulted. 11 S10.3.2 R-3
is the root finding. Without this gate, a person standing 0.5 m off the flank
plus a spoken "turn around" is enough, and 18 A11 pins dyaw at +pi so the
operator can neither choose nor predict the side it sweeps toward.

Which design sections. Trigger: 12 S6A.4.1, verbatim "spin_like = |wz| >
wz_eps_radps AND |v|/|wz| < k_rot x r_eff". Verdict: 12 S6A.3.2, the per-cell
blocked predicate plus the pass conjunction. Invariants: 12 S6A.3.3 RCG-1 to
RCG-4. Per-source disposal: 12 S6A.4.2. Events and their detail.kind values:
12 S6A.8 OB-2. Config block: 12 S12 rotation_clearance.

What this file does NOT do, and where that work lives instead.
  * No Zenoh, no clock read, no config read. It is arithmetic over its
    arguments, because it runs inside the 20 Hz tick at 12 S2.2 step 6b and
    CLAUDE.md 4.4 forbids blocking IO there.
  * It does not own the ring READ. Counting occupied and unknown cells in the
    annulus belongs to whoever holds the occupancy grid; this file consumes the
    counts as a RingSample. 12 S6A.3.1 RC-D2 rules rt/lidar/grid the single
    primary source, and 11's LiDAR single-topic row records that the machine
    has no LiDAR, so today nothing produces a RingSample -- see nav_tick step
    6b for what that means at run time.
  * It is not the V-33 coverage fail-safe in xbrain/common/failsafe/rotation.py.
    That one answers the COMMAND layer (18 A09..A12 / C07 -> E_BUSY) at intent
    time; this one is the per-tick velocity-layer gate. 12 S6A.9 ND-3 wants one
    implementation of the criterion shared by the entry and exit call sites;
    the entry site (12 S4.6.4 RC-1) is still unwired.

The looks-right-but-wrong writings, each one a named prohibition.
  * Feeding r_eff into r_check. 12 S6A.4.1 iron rule (1). r_eff is the TRIGGER
    radius and carries a fallback for an uncalibrated body; r_check is
    r_robot + margin_rot on the TRUE value. Substituting writes "unknown" as
    "known 0.60", the exact fail-open RCG-1 is named for. So effective_radius
    is consumed by spin_like alone, and ring_check_radius refuses a
    non-positive r_robot rather than falling back to anything.
  * Letting RCG-1 pass because r_eff > 0. Iron rule (2): pass is evaluated on
    the true r_robot, always.
  * r_eff = max(r_robot, fallback). 12 S12 v0.7 retired that form. It is not
    equivalent to the branch for 0 < r_robot < fallback: it would push a
    measured 0.30 up to 0.60 and double the trigger threshold for no reason.
    The branch in effective_radius is the only legal shape.
  * A linear-velocity deadzone as the trigger (|v| <= v_eps AND |wz| > wz_eps).
    12 S6A.4.1 rejects it by name: emitting vx = v_eps + delta together with a
    large wz walks around the entire gate while still spinning on the spot.
    The curvature radius has no such seam, which is why 12 S12 deleted the
    v_eps_mps key outright.
  * Reading an unknown cell as free. RCG-3. That is "treat what you did not see
    as empty", and it is why rot_unknown_max_cells carries no tolerance.
  * Letting an empty annulus pass. RCG-2. count(blocked) == 0 is vacuously true
    over an empty set, so "no cells to check" must read as "cannot tell", never
    as "clean".
  * Vetoing rns_avoid's wz. 12 S6A.4.2 keeps that source on the LIMIT branch:
    zeroing the turn of the one source whose motive is to get AWAY from the
    obstacle is the safety gate used backwards.
  * Putting the attribution into gate.limiter. 12 S6A.8 OB-1 forbids extending
    that closed set -- every value in it describes a LINEAR cap, and 11 S9.6.5's
    argmax over the reduction Delta has no meaning for wz. Attribution leaves
    as an event instead (OB-2); OB-4 registers gate.wz_limiter as the known
    gap, not yet ruled by 11.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


class RotationConfigError(ValueError):
    """A rotation_clearance parameter is missing, off-type or out of range.

    Its own type, not ValueError, so a caller can tell a bad rotation block
    from a bad loop parameter without parsing the message. CLAUDE.md 3.1: these
    are safety params, so construction refuses instead of substituting.
    """


# Control period in seconds, 12 S2.2 (20 Hz). RCG-4's second term prices the
# recheck window in seconds, so the tick length is part of that bound's
# arithmetic. It is a structural constant of the loop rather than a tunable:
# changing the rate re-prices the whole 12 S2.2 latency budget, not just here.
T_CTRL_S = 0.05

# Occupancy encoding, 11 S3.9 and 12 S6A.3.2. The two BLOCKING values are not
# numerically adjacent -- 0 is unknown and 2 is occupied with 1 (free) sitting
# between them -- so every "> 0 blocks" or "truthy blocks" shortcut is wrong by
# construction. All three are named so the predicate reads the way the contract
# writes it instead of as an arithmetic trick.
OCC_UNKNOWN = 0
OCC_FREE = 1
OCC_OCCUPIED = 2

# ---------------------------------------------------------------------------
# 12 S6A.8 OB-2: the decision that goes into the event detail. The document
# fixes this closed set verbatim as "decision in { reject, limit, override }";
# PASS is the fourth state only in the sense that it emits NO event at all, so
# it never appears in a detail and never needs to match those three spellings.
# ---------------------------------------------------------------------------
DECISION_PASS = "pass"
DECISION_LIMIT = "limit"
DECISION_REJECT = "reject"

# 12 S6A.8: the three detail.kind values for cat = "motion". This section IS
# their definition site -- 11 keeps no central closed set for motion detail.kind
# and every existing value is declared by the section that emits it.
KIND_ROTATION_BLOCKED = "rotation_blocked"                  # warn
KIND_CLEARANCE_UNCONFIGURED = "rotation_clearance_unconfigured"   # fault

# ---------------------------------------------------------------------------
# Why the verdict has a reason string at all. 12 S6A.8 OB-1 forbids widening
# gate.limiter, and OB-4 records that 11 S3.4's gate block has NO field able to
# answer "why is wz zero this tick". Until gate.wz_limiter is ruled, the event
# detail is the only place the answer can live, so the reason must be specific
# enough to separate "something is in the ring" from "we cannot see the ring"
# from "the config is not self-consistent". 12 S6A.6 note 1 is explicit that
# saying those in one sentence makes the field retry a condition that will
# never clear on its own.
# ---------------------------------------------------------------------------
# The permitting outcome. It carries a name rather than an empty string so a
# log line reads the same shape whichever way the tick went.
REASON_PERMITTED = "permitted"

# No RingSample at all: RC-D2's primary source is absent, not merely stale.
# These are different situations for the operator -- a stale grid clears by
# itself, an absent one never does -- and only one of them is waitable.
REASON_NO_RING_SOURCE = "ring_source_unavailable"

# RCG-1, on the TRUE r_robot. 0.0 means "not known", never "zero radius".
# Read as zero radius, r_check collapses to margin_rot alone and the annulus
# shrinks to a handful of cells around the origin, which passes almost always.
REASON_R_ROBOT_UNCALIBRATED = "r_robot_uncalibrated"

# RCG-2, two distinct ways the decision domain stops being a usable annulus.
# The first is a configuration mistake (inner radius raised to or past the
# outer one, usually while trying to mask the robot's own echo); the second is
# a grid that produced no cells in range at all. Both mean "cannot tell".
REASON_SELF_MASK_GE_R_CHECK = "self_mask_ge_r_check"
REASON_EMPTY_DOMAIN = "decision_domain_empty"

# The two cell counts of 12 S6A.3.2, kept apart because 12 S6A.3.2's note asks
# the field to be able to see "there is something there" vs "it is not
# covered", and 12 S6A.6 turns the same split into two HMI sentences: clear the
# area and retry, against rotation is unavailable and retrying will not help.
REASON_OCCUPIED_CELLS = "occupied_cells_over_max"
REASON_UNKNOWN_CELLS = "unknown_cells_over_max"

# Whole-frame sanity check, same source as 11 S3.9's unknown_ratio guidance.
# It does not replace the per-cell counts above: a frame can be mostly known
# and still have this particular ring blocked, and vice versa.
REASON_UNKNOWN_RATIO = "unknown_ratio_over_max"

# RC-3 (12 S6A.5): the grid is older than rotation_clearance.grid_age_max_ms.
# The threshold is deliberately tighter than the generic 500 ms staleness rule,
# because at 500 ms the RCG-4 bound rises above the pinned margin and the
# permit could not hold anyway.
REASON_GRID_STALE = "grid_stale"

# RCG-4: margin_rot is below the bound its own inputs imply, so "the ring was
# clean this tick" has already expired by the time it is acted on. This is the
# one refusal that is about the configuration rather than the surroundings,
# which is why it does not share a reason with the cell counts.
REASON_MARGIN_BELOW_BOUND = "margin_rot_below_rcg4_bound"

# The 12 S6A.3.2 cross-veto on sectors. Unavailable is a REJECT, not a skip:
# two independent channels are required to agree, and a missing second opinion
# is not an abstention. This is the conjunct that keeps refusing even after a
# body radius is measured, because the sectors span only the forward half.
REASON_SECTORS_UNAVAILABLE = "sectors_full_circle_unavailable"
REASON_SECTORS_BELOW_R_CHECK = "sectors_min_below_r_check"

# ---------------------------------------------------------------------------
# 12 S6A.4.2 disposal by source. The table's axis is "is a human watching THIS
# turn in real time", not the priority number.
#
# LIMIT holds the local teleop family -- 12 S6A.4.2's exemption argues verbatim
# that the pad and the keyboard are both beside the robot -- and rns_avoid,
# whose motive is to get away from the obstacle. That second row is the
# section's only "high priority source exempt from veto", and the same row
# demands the source test be spelled out rather than left to a default, which
# is why these are explicit frozensets and an unrecognised source raises.
LIMIT_SOURCES = frozenset({"teleop_keyboard", "teleop_joystick", "rns_avoid"})

# VETO holds autonomous turns with nobody watching, plus teleop_cloud: v0.7.9
# narrowed the human-in-the-loop exemption to LOCAL sources because the cloud
# operator has a delayed video feed and not even the "standing next to it"
# premise. hold and fence_guard emit no motion at all; they are listed so that
# the union of the two sets covers every BehaviorSource value exactly, which a
# meta test asserts in both directions.
#
# 12 S6A.4.2's table still names path_follow and target_oriented in this
# column. 12 v0.8 folded both into RNS (12 S4.2, mirrored in
# sources/arbiter_p1.py), so those two rows have no source to apply to any
# more; the behaviour they described now arrives under rns_avoid, which the
# same table puts on the LIMIT branch. That divergence is reported, not
# resolved here -- picking a branch for the merged source is a ruling.
VETO_SOURCES = frozenset({"nav2_proxy", "relative_move", "teleop_cloud",
                          "hold", "fence_guard"})


def _finite_number(name: str, value: object) -> float:
    """A finite int/float, or RotationConfigError naming the key.

    bool is rejected first because bool is a subclass of int in Python, and
    True would otherwise sail through as 1.0 -- a silent acceptance is exactly
    what CLAUDE.md 3.1 is about.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)) \
            or not math.isfinite(float(value)):
        raise RotationConfigError(
            "rotation_clearance.%s must be a finite number, got %r" % (name, value))
    return float(value)


@dataclass(frozen=True)
class RotationLimits:
    """The 12 S12 rotation_clearance block, injected whole.

    Every field is required. There is no default anywhere in this class and no
    from-config helper that fills one in: CLAUDE.md 3.1 makes a missing safety
    param an error that names the key, and a plausible-looking stand-in here
    would be indistinguishable from a calibrated value downstream.

    r_robot is deliberately NOT a field. 12 S12 keeps its single definition in
    the RNS geometry section and warns that a private copy would be flagged as
    a duplicate by the freeze line's assertion B. It is passed per call instead.

    wz_blind_radps is Optional on purpose. 12 S12 keeps it out of this block
    and reuses 11 S3.1.5.6's free_space.blind.wz_blind_radps, then rules what
    happens when that key cannot be read: landing plan (2) says a missing,
    non-positive or non-finite value degrades the LIMIT branch to a VETO plus
    one rotation_clearance_unconfigured fault, verbatim "cannot get the clamp
    value, then do not let it through -- never let it through on a guessed
    one". None is that state, modelled rather than defaulted away.
    """

    margin_rot_m: float            # r_check = r_robot + this (12 S6A.3.2)
    r_self_mask_m: float           # ring inner radius; 0.0 = full disc (RC-D3)
    rot_occ_max: int               # 12 S6A.3.2, contract name from 11 S13.8
    rot_unknown_max_cells: int     # RCG-3, no tolerance
    rot_unknown_ratio_max: float   # whole-frame sanity, 11 S3.9
    grid_age_max_ms: int           # RC-3
    recheck_ticks: int             # RC-2, priced by RCG-4's second term
    wz_eps_radps: float            # spin_like's first conjunct
    k_rot: float                   # spin_like's curvature coefficient
    r_robot_fallback_m: float      # TRIGGER only, never r_check
    ped_speed_mps: float           # RCG-4's first term
    allow_visual_override: bool    # 12 S6A.6, must be false here (see below)
    wz_blind_radps: Optional[float]

    def __post_init__(self) -> None:
        """Range-check at construction so the 20 Hz tick never has to.

        The checks below are the ones whose violation would silently widen the
        gate rather than crash it. Negative or non-finite values are rejected
        for every field; zero is allowed only where 12 S12 gives 0 as the
        STRICTEST setting (r_self_mask_m, rot_occ_max, rot_unknown_max_cells),
        never as a stand-in for an unmeasured quantity.
        """
        for name in ("margin_rot_m", "wz_eps_radps", "k_rot",
                     "r_robot_fallback_m", "ped_speed_mps"):
            if _finite_number(name, getattr(self, name)) <= 0.0:
                raise RotationConfigError(
                    "rotation_clearance.%s must be > 0, got %r"
                    % (name, getattr(self, name)))
        for name in ("r_self_mask_m", "rot_unknown_ratio_max"):
            if _finite_number(name, getattr(self, name)) < 0.0:
                raise RotationConfigError(
                    "rotation_clearance.%s must be >= 0, got %r"
                    % (name, getattr(self, name)))
        # The two cell tolerances and the two tick counts are counts, so an int
        # is required: a float here would mean somebody wrote 0.5 cells, and
        # comparing counts against it would round in an unannounced direction.
        for name in ("rot_occ_max", "rot_unknown_max_cells"):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, int) or v < 0:
                raise RotationConfigError(
                    "rotation_clearance.%s must be an int >= 0, got %r" % (name, v))
        for name in ("grid_age_max_ms", "recheck_ticks"):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, int) or v <= 0:
                raise RotationConfigError(
                    "rotation_clearance.%s must be an int > 0, got %r" % (name, v))
        if not isinstance(self.allow_visual_override, bool):
            raise RotationConfigError(
                "rotation_clearance.allow_visual_override must be a bool, got %r"
                % (self.allow_visual_override,))
        # 12 S6A.6 lets allow_visual_override release ONE rotation, but only
        # when all four of its conditions hold together, and the fourth is an
        # event carrying the confirming command's origin and cmd_id. The exit
        # layer has no command identity to carry -- it sees a velocity, not a
        # cmd_id -- so those four cannot be evaluated here. 12 S6A.6 says of
        # exactly that case: without the fourth condition the switch becomes a
        # silent safety-release channel. Refusing to start is the only reading
        # that does not open one, and it is loud rather than quiet.
        if self.allow_visual_override:
            raise RotationConfigError(
                "rotation_clearance.allow_visual_override is true but the four "
                "conditions of 12 S6A.6 cannot be evaluated at the exit layer "
                "(no cmd_id to name in the rotation_visual_override event); "
                "refusing rather than releasing rotation without the record")
        # wz_blind_radps stays Optional, but a present value must be usable.
        # None is the ruled-on missing-key state (12 S12 landing plan (2)); a
        # present-but-nonsense value is a config error, not a degrade, because
        # somebody wrote it on purpose and should be told it is wrong.
        if self.wz_blind_radps is not None:
            if _finite_number("wz_blind_radps", self.wz_blind_radps) <= 0.0:
                raise RotationConfigError(
                    "free_space.blind.wz_blind_radps must be > 0 when present, "
                    "got %r" % (self.wz_blind_radps,))


@dataclass(frozen=True)
class RingSample:
    """One tick's read of the sweep annulus, produced by the grid holder.

    This is the seam 12 S6A.3.1 RC-D2 defines: the judge wants per-cell counts
    over A = { cell | r_self_mask <= rho(cell) <= r_check }, in the body frame,
    and nothing else. Keeping it a plain value object is what lets the verdict
    be tested with a clean ring, a ring with one occupied cell, and an empty
    ring, without a grid or a sensor anywhere in the test.

    occ_cells and unknown_cells are counted SEPARATELY even though both block.
    12 S6A.3.2 asks for a diagnosable signal that tells "there is something
    there" apart from "that direction is not covered", and 12 S6A.6 note 1
    turns the same split into two different HMI sentences with two different
    client behaviours (retry helps / retry never helps).

    sectors_min_m is the 12 S6A.3.2 cross-veto input: the FULL-CIRCLE minimum
    of 11 S3.1.5's sectors. None means the full-circle minimum cannot be formed
    -- which is today's standing state, because V-33 leaves the flanks and rear
    at covered = false and the sectors span only +/-90 degrees. 12 S6A.3.3's
    2026-08-05 note is explicit that this makes the cross-veto conjunct false
    and that this is the design intent, not a defect.
    """

    occ_cells: int                  # count(occ == 2) inside A
    unknown_cells: int              # count(occ == 0 or outside the grid) in A
    total_cells: int                # |A|, the RCG-2 operand
    unknown_ratio: float            # whole-frame, 11 S3.9
    age_ms: int                     # now_mono - grid.frame_mono, 11 S3.0.1
    resolution_m: float             # grid cell size, RCG-4's quantisation term
    sectors_min_m: Optional[float]  # full-circle sectors minimum; None = none
    blocked_sector_deg: Tuple[Tuple[float, float], ...] = ()


@dataclass(frozen=True)
class RotationVerdict:
    """Outcome of the RC-1 conjunction, before any per-source disposal."""

    permitted: bool
    reason: str
    # r_check is None exactly when RCG-1 failed, because r_check is undefined
    # without a true r_robot. Echoing a None rather than a plausible number is
    # the point: 12 S4.6.4 RCE-2 makes r_check_m mandatory in the E_BUSY
    # detail, and a fabricated radius there would read as a real measurement.
    r_check_m: Optional[float]


@dataclass(frozen=True)
class RotationEval:
    """What step 6b did this tick, and everything the event needs.

    wz_out is the only field the control path consumes. The rest exists because
    OB-1 blocks the attribution from riding in gate.limiter, so the event is
    the only carrier and it has to hold the whole story (OB-2's detail list:
    r_check_m, occ_cells, unknown_cells, blocked_sector_deg, grid_age_ms,
    decision).
    """

    spin_like: bool
    decision: str                   # DECISION_* above
    reason: str                     # REASON_* above
    wz_in: float
    wz_out: float
    r_check_m: Optional[float]
    occ_cells: int
    unknown_cells: int
    grid_age_ms: Optional[int]
    blocked_sector_deg: Tuple[Tuple[float, float], ...]
    event_kind: Optional[str]       # None when nothing happened this tick
    detail_item: Optional[str]      # set only for the unconfigured fault


def effective_radius(r_robot_m: Optional[float], r_robot_fallback_m: float) -> float:
    """r_eff for the TRIGGER only (12 S6A.4.1), as a branch and never a max.

    The branch and max(r_robot, fallback) disagree for 0 < r_robot < fallback:
    max would push a measured 0.30 up to 0.60 and double the trigger threshold
    with no justification. 12 S12 retired the max form for that reason and
    permits only this shape, so the whole book has one implementation.

    r_robot is Optional because "the key does not exist" and "the key is 0.0"
    are the same statement in 12 S6A.3.3 -- both mean "not known" -- and
    collapsing them here keeps every caller from having to remember that.

    mutant: return r_robot_m when it is 0.0 -> the trigger threshold becomes 0
    -> |v|/|wz| < 0 is never true -> nothing is ever spin_like -> the gate
    stops existing. That is the fail-open 12 S6A.4.1's correction table calls
    out by name, and test_spin_like_still_triggers_when_r_robot_uncalibrated
    turns it red.
    """
    if r_robot_m is not None and r_robot_m > 0.0:
        return float(r_robot_m)
    return float(r_robot_fallback_m)


def spin_like(vx_mps: float, vy_mps: float, wz_radps: float, *,
              wz_eps_radps: float, k_rot: float, r_eff_m: float) -> bool:
    """12 S6A.4.1: is this tick an in-place sweep, or travel that turns?

    spin_like = |wz| > wz_eps AND |v| / |wz| < k_rot * r_eff

    |v| / |wz| is the instantaneous turn radius R. R smaller than the body
    radius scale means the rotation centre lies INSIDE the body, i.e. the
    machine is sweeping where it stands. That is a physical statement, which is
    why it is the trigger and why the fail-safe for an uncalibrated body is NOT
    applied here (12 S6A.3.3's v0.4 boundary, and 12 S6A.4.1's correction:
    doing it in both places made path_follow's every tick read as a sweep, so
    the robot could only drive in straight lines).

    |v| is the planar speed hypot(vx, vy), not |vx|. On a holonomic chassis a
    crab-walk with vy and no vx is travel, and judging on vx alone would call
    it a spin and stop it.

    12 S6A.4.1's substitution table, with the threshold at k_rot * r_eff =
    0.5 * 0.60 = 0.30 m on an uncalibrated body:

      patrol tracking   vx 2.00  wz 0.30  R = 6.667 m   no trigger
      slow hard turn    vx 0.50  wz 1.00  R = 0.500 m   no trigger
      avoid-tier turn   vx 0.20  wz 0.30  R = 0.667 m   no trigger
      the "add some vx" bypass  vx 0.04  wz 1.50  R = 0.027 m   TRIGGERS
      pure spin         vx 0.00  wz 1.50  R = 0.000 m   TRIGGERS

    The first three are why this is a ratio and not a deadzone: every one of
    them is a turn the robot must be allowed to make, and all three sit two to
    twenty times clear of the threshold rather than just inside it. The fourth
    is why it is a ratio and not "vx below some epsilon": 0.04 m/s with 1.5
    rad/s is a machine spinning on the spot that a deadzone would call travel.

    A caveat worth writing down, because it is not in that table. RNS tapers
    its speed by cos(heading error) with a floor, so a large heading error
    gives a small |v| against a large wz -- a genuinely small R, correctly
    read as a sweep. The table was written when path_follow was a separate
    pure-pursuit source with no such taper, so it does not cover that case.
    The classification is right (the rotation centre really is inside the
    body); what it means for availability is reported, not decided here.

    mutant: swap the < for <= or drop the wz_eps conjunct -> a resting robot
    with wz = 0 divides by zero or reads as spinning -> the hold source starts
    emitting rotation events every tick.
    """
    if abs(wz_radps) <= wz_eps_radps:
        # Below the deadzone there is no rotation to judge, and returning here
        # is also what keeps the division below safe: wz_eps is > 0 by
        # construction (RotationLimits.__post_init__), so |wz| > wz_eps implies
        # |wz| > 0 and the quotient is always finite.
        return False
    return math.hypot(vx_mps, vy_mps) / abs(wz_radps) < k_rot * r_eff_m


def ring_check_radius(r_robot_m: Optional[float], margin_rot_m: float) -> Optional[float]:
    """r_check = r_robot + margin_rot on the TRUE r_robot (12 S6A.3.2).

    Returns None when r_robot is unknown, and that None is load-bearing: it is
    how RCG-1 reaches the verdict without anything in this file ever having a
    number to substitute. 12 S6A.4.1 iron rule (1) forbids r_eff here, and the
    fallback is the only other number in scope, so returning None is the only
    remaining honest answer.

    mutant: fall back to r_robot_fallback_m here -> r_check becomes 1.60 on an
    uncalibrated body and the ring is judged against a radius nobody measured
    -> test_r_check_is_none_when_r_robot_uncalibrated red.
    """
    if r_robot_m is None or r_robot_m <= 0.0:
        return None
    return float(r_robot_m) + float(margin_rot_m)


def rcg4_lower_bound(limits: RotationLimits, resolution_m: float) -> float:
    """RCG-4's lower bound on margin_rot (12 S6A.3.3), not its value.

      margin_rot >= ped_speed * (grid_age_max_ms/1000 + recheck_ticks * T_ctrl)
                    + 0.5 * resolution_m * sqrt(2)

    Term one: the grid was exposed in the past, and a walking person covers
    ground between exposure and use. Term two: RC-2 tolerates recheck_ticks
    consecutive failing ticks before it cancels, and the same person keeps
    walking through that window. Term three: grid quantisation, cell centre
    against true edge.

    This is a bound, not a setting. margin_rot itself is pinned by U54 rule 3
    at the system safety distance and 12 S6A.3.3 says shrinking it is a
    requirements change. What tuning can do is raise the bound until it exceeds
    the pinned value, at which point RCG-4 stops holding and the permit refuses
    -- with the attribution moving from "something in the ring" to "the config
    is not self-consistent", which is why it gets its own reason string.
    """
    travel_s = limits.grid_age_max_ms / 1000.0 + limits.recheck_ticks * T_CTRL_S
    return limits.ped_speed_mps * travel_s + 0.5 * resolution_m * math.sqrt(2.0)


def evaluate_ring(limits: RotationLimits, r_robot_m: Optional[float],
                  ring: Optional[RingSample]) -> RotationVerdict:
    """The RC-1 conjunction of 12 S6A.3.2 plus RCG-1..RCG-4.

    Every conjunct is evaluated from data, none is hard-wired: hand this a
    calibrated r_robot and a clean full-circle ring and it returns permitted.
    That matters more than it looks. On today's machine the permit always
    refuses -- there is no LiDAR, so no RingSample exists, and V-33 leaves
    sectors_min unavailable on top of that -- and a judge that only ever
    refused would be passed just as happily by a one-line "return False". The
    permitting path is what separates this from that stub, so it is asserted.

    Evaluation order is fixed for a reproducible reason string, and runs
    config-level checks before data-level ones: an unusable configuration
    should be reported as such rather than surfacing as whatever the grid
    happened to contain. The verdict itself is order-independent -- it is a
    conjunction -- so the order only decides WHICH failure gets named.

    Worked, on today's machine, to make the three independent refusals visible
    rather than leaving "it always refuses" as one undifferentiated fact:

      RCG-1  r_robot      no key in any loaded tree      -> REFUSE (reported)
      RC-D2  ring sample  no LiDAR, nothing builds one   -> would refuse
      cross  sectors min  forward half only, no full     -> would refuse
             veto                     circle minimum

    Each is closed by different work -- a calibration key, a ring producer, a
    sensor-coverage answer -- so collapsing them into one boolean would hide
    two of the three from whoever comes to close them.
    """
    # RCG-1, first, and on the TRUE r_robot. 0.0 does not mean "zero radius, so
    # always safe"; it means "not known, so do not turn". Reading it the other
    # way shrinks r_check to margin_rot alone and the decision domain collapses
    # to a few cells round the origin, which nearly always passes -- 12 S6A.3.3
    # calls that the most likely and most costly fail-open in the section.
    r_check = ring_check_radius(r_robot_m, limits.margin_rot_m)
    if r_check is None:
        return RotationVerdict(False, REASON_R_ROBOT_UNCALIBRATED, None)

    # RCG-2, config half: 0 <= r_self_mask < r_check. If the inner radius
    # reaches the outer one the annulus is the empty set, count(blocked) == 0
    # holds vacuously, and the permit passes everything for ever.
    if limits.r_self_mask_m >= r_check:
        return RotationVerdict(False, REASON_SELF_MASK_GE_R_CHECK, r_check)

    # RC-D2's failure direction, verbatim: primary source unavailable means
    # refuse. Absent is strictly worse than stale, so it is checked before any
    # field of the sample is touched.
    if ring is None:
        return RotationVerdict(False, REASON_NO_RING_SOURCE, r_check)

    # RCG-4. Checked here rather than at startup because its quantisation term
    # needs the grid's own resolution, which is a run-time fact. 12 S6A.3.3
    # describes the four invariants as a startup self-check; evaluating them
    # every tick is strictly stronger and costs two multiplications.
    if limits.margin_rot_m < rcg4_lower_bound(limits, ring.resolution_m):
        return RotationVerdict(False, REASON_MARGIN_BELOW_BOUND, r_check)

    # RCG-2, data half: |A| > 0. "No cells to check" reads as "cannot tell".
    if ring.total_cells <= 0:
        return RotationVerdict(False, REASON_EMPTY_DOMAIN, r_check)

    # RC-3 (12 S6A.5). Deliberately stricter than the generic T-05 of 500 ms:
    # at 500 ms RCG-4's bound rises above the pinned margin_rot and the permit
    # could never hold anyway. Stale data is refused, never used.
    if ring.age_ms > limits.grid_age_max_ms:
        return RotationVerdict(False, REASON_GRID_STALE, r_check)

    # The two cell counts. Occupied first so that a ring with both gets the
    # more actionable reason -- "there is something there" can be cleared by
    # moving it, "not covered" cannot.
    if ring.occ_cells > limits.rot_occ_max:
        return RotationVerdict(False, REASON_OCCUPIED_CELLS, r_check)

    # RCG-3. Unknown blocks exactly like occupied. Raising the tolerance is
    # equivalent to calling everything unseen empty, which is the whole of the
    # R-3 gap reproduced in one config line.
    if ring.unknown_cells > limits.rot_unknown_max_cells:
        return RotationVerdict(False, REASON_UNKNOWN_CELLS, r_check)

    # Whole-frame sanity check. Cheap, and it does not replace the per-cell
    # test above -- a frame can be mostly known and still have the ring blocked.
    if ring.unknown_ratio > limits.rot_unknown_ratio_max:
        return RotationVerdict(False, REASON_UNKNOWN_RATIO, r_check)

    # The cross-veto of 12 S6A.3.2, written the way the conjunction writes it:
    # "sectors full-circle minimum >= r_check, OR that value is unavailable ->
    # treated as not passing". Two independent channels must both agree, and an
    # absent second opinion is not an abstention.
    if ring.sectors_min_m is None:
        return RotationVerdict(False, REASON_SECTORS_UNAVAILABLE, r_check)
    if ring.sectors_min_m < r_check:
        return RotationVerdict(False, REASON_SECTORS_BELOW_R_CHECK, r_check)

    return RotationVerdict(True, REASON_PERMITTED, r_check)


def _disposal_for(source: str) -> str:
    """12 S6A.4.2's per-source branch, written out rather than defaulted.

    An unrecognised source raises instead of picking a branch. CLAUDE.md 3.5
    forbids passing a value outside a closed set through silently, and here the
    two readings of a default are both wrong: defaulting to LIMIT would exempt
    an unknown source from the veto, and defaulting to VETO would silently
    zero a source that 12 S6A.4.2 may have placed on the limit branch. The
    same row that grants rns_avoid its exemption says the source test must be
    written explicitly and not left to a default.
    """
    if source in LIMIT_SOURCES:
        return DECISION_LIMIT
    if source in VETO_SOURCES:
        return DECISION_REJECT
    raise RotationConfigError(
        "rotation permit has no 12 S6A.4.2 disposal for source %r; add it to "
        "LIMIT_SOURCES or VETO_SOURCES after the section rules which" % (source,))


def apply_rotation_permit(*, vx_mps: float, vy_mps: float, wz_radps: float,
                          source: str, limits: RotationLimits,
                          r_robot_m: Optional[float],
                          ring: Optional[RingSample]) -> RotationEval:
    """12 S2.2 step 6b: the exit-layer rotation permit for one tick.

    Runs after the speed gate and before fence clipping, on the arbitration
    winner's velocity, and only when the tick is spin_like. Returns the wz the
    tick may actually use plus everything the event needs.

    Why the exit layer exists at all, when 11 S4.6.4 only ever put RC-1 at the
    command entry: 12 S6A.4.2 walks the sources that can produce a pure wz and
    finds three with no command channel to intercept -- teleop, the
    target-oriented aim, and the avoidance source. An entry-only check is a
    lock on one of six doors, and the teleop door is the one where the person
    is standing right next to the machine.

    vx and vy are the GATED values, matching the step's position in 12 S2.2.
    A tick whose vx the speed gate has already zeroed is, at this point in the
    chain, genuinely a spin, and judging on the pre-gate candidate would let
    exactly that case through.
    """
    # Trigger first, verdict second, and never the other way round. The order
    # is what keeps the two fail-safes apart: the trigger answers "is this a
    # sweep", which is physics and must stay honest on an uncalibrated body,
    # while the verdict answers "may it proceed", which is where refusing on
    # missing knowledge belongs. 12 S6A.4.1's correction exists because a
    # previous version applied the fail-safe to both and the robot could then
    # only drive in straight lines.
    r_eff = effective_radius(r_robot_m, limits.r_robot_fallback_m)
    spinning = spin_like(vx_mps, vy_mps, wz_radps,
                         wz_eps_radps=limits.wz_eps_radps,
                         k_rot=limits.k_rot, r_eff_m=r_eff)
    if not spinning:
        # Travel that turns does not enter this section at all; its geometric
        # safety is the RNS inflated corridor, whose half width already carries
        # the body radius (12 S6A.4.1 closing note). Emitting no event here is
        # part of that: a warn per tick on normal patrol turning would bury the
        # real ones and get the gate switched off in the field.
        return RotationEval(
            spin_like=False, decision=DECISION_PASS, reason=REASON_PERMITTED,
            wz_in=wz_radps, wz_out=wz_radps, r_check_m=None,
            occ_cells=0, unknown_cells=0, grid_age_ms=None,
            blocked_sector_deg=(), event_kind=None, detail_item=None)

    verdict = evaluate_ring(limits, r_robot_m, ring)
    occ = ring.occ_cells if ring is not None else 0
    unk = ring.unknown_cells if ring is not None else 0
    age = ring.age_ms if ring is not None else None
    sectors = ring.blocked_sector_deg if ring is not None else ()
    if verdict.permitted:
        return RotationEval(
            spin_like=True, decision=DECISION_PASS, reason=REASON_PERMITTED,
            wz_in=wz_radps, wz_out=wz_radps, r_check_m=verdict.r_check_m,
            occ_cells=occ, unknown_cells=unk, grid_age_ms=age,
            blocked_sector_deg=sectors, event_kind=None, detail_item=None)

    decision = _disposal_for(source)
    detail_item: Optional[str] = None
    # 12 S6A.8's two kinds split on whether a retry could ever clear it. The
    # three config-level reasons are persistent (12 S6A.7 RC-D5 routes the same
    # split to E_CAPABILITY rather than E_BUSY at the command layer, because
    # backing off and retrying an unset parameter never improves); everything
    # else is a fact about this tick's surroundings and can clear.
    #
    # REASON_NO_RING_SOURCE is grouped with the persistent ones. 12 S6A.8's
    # table lists a stale grid under rotation_blocked but does not name an
    # absent source, because the section was written when rt/lidar/grid existed.
    # A source that is not merely late but absent cannot be waited out, so the
    # fault kind carries the field's actual situation; the verdict is identical
    # either way, only the label differs. Reported as a registered choice.
    if verdict.reason in (REASON_R_ROBOT_UNCALIBRATED, REASON_SELF_MASK_GE_R_CHECK,
                          REASON_MARGIN_BELOW_BOUND, REASON_NO_RING_SOURCE):
        kind = KIND_CLEARANCE_UNCONFIGURED
        detail_item = verdict.reason
    else:
        kind = KIND_ROTATION_BLOCKED

    if decision == DECISION_LIMIT:
        blind = limits.wz_blind_radps
        if blind is None:
            # 12 S12 landing plan (2), verbatim: cannot get the clamp value ->
            # do not let it through, never let it through on a guessed one. The
            # degrade is a VETO and it reports its own cause rather than the
            # ring's, because the operator's next action differs entirely.
            decision = DECISION_REJECT
            kind = KIND_CLEARANCE_UNCONFIGURED
            detail_item = "wz_blind_radps"
            wz_out = 0.0
        else:
            # Clamp, do not zero. 12 S6A.4.2: taking the operator's wz away
            # leaves him unable to turn OUT of a wall he is against, and taking
            # the avoidance source's wz away refuses to dodge because there is
            # something to dodge.
            wz_out = max(-blind, min(blind, wz_radps))
    else:
        wz_out = 0.0

    return RotationEval(
        spin_like=True, decision=decision, reason=verdict.reason,
        wz_in=wz_radps, wz_out=wz_out, r_check_m=verdict.r_check_m,
        occ_cells=occ, unknown_cells=unk, grid_age_ms=age,
        blocked_sector_deg=sectors, event_kind=kind, detail_item=detail_item)


__all__ = [
    "T_CTRL_S",
    "OCC_UNKNOWN", "OCC_FREE", "OCC_OCCUPIED",
    "DECISION_PASS", "DECISION_LIMIT", "DECISION_REJECT",
    "KIND_ROTATION_BLOCKED", "KIND_CLEARANCE_UNCONFIGURED",
    "REASON_PERMITTED", "REASON_NO_RING_SOURCE", "REASON_R_ROBOT_UNCALIBRATED",
    "REASON_SELF_MASK_GE_R_CHECK", "REASON_EMPTY_DOMAIN",
    "REASON_OCCUPIED_CELLS", "REASON_UNKNOWN_CELLS", "REASON_UNKNOWN_RATIO",
    "REASON_GRID_STALE", "REASON_MARGIN_BELOW_BOUND",
    "REASON_SECTORS_UNAVAILABLE", "REASON_SECTORS_BELOW_R_CHECK",
    "LIMIT_SOURCES", "VETO_SOURCES",
    "RotationConfigError", "RotationLimits", "RingSample",
    "RotationVerdict", "RotationEval",
    "effective_radius", "spin_like", "ring_check_radius", "rcg4_lower_bound",
    "evaluate_ring", "apply_rotation_permit",
]
