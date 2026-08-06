"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rotation.py
Brief: V-33 in-place-rotation fail-safe -- blind sides make every spin E_BUSY

Description:
The problem this solves. The M20S front/rear LiDAR horizontal FOV and mounting
orientation are unknown pending a 云深处 writeup plus an on-site recheck (21 S1,
11 V-33). Until that closes, 11 rules that the sides and rear are treated as
covered=false: the left/right coverage bands carry an empty source list and a
null clearance, so the rotation sweep ring's left and right arcs are永远未知格.
12 S6A.3.2 (RC-1) counts an unknown cell as blocking exactly like an occupied
one, so the rotation permission can never pass, and 18 A09~A12 (turn_left/right/
around/face_heading) and C07 (set_motion_behavior) must all return E_BUSY with a
three-field detail. This module is that fail-safe branch, factored into common/
so the one blocked-rotation verdict has a single home instead of being re-derived
at each caller.

Why in-place rotation and not translation. 11 S10.3.2 R-3 is the root fact: a
pure spin has NO geometric safety gate anywhere else in V6 -- the speed gate caps
linear velocity only, fence clipping is a vector projection that a zero-linear
spin defeats, the three-stage limiter bounds magnitude not swept area, and Nav2
runs with simulate_ahead_time 0.0. So the sweep-ring check IS the only thing
standing between "转身" and a 180-degree sweep through a person standing 0.5 m to
the side. Translation stays protected by f() / corridor / fence and is out of
scope here; the lateral case below is about its CONFIRMATION level, not a block.

Which design sections. Occupancy encoding and the blocked predicate: 12 S6A.3.2
(逐字 "blocked(cell) = occ==2 OR occ==0 未知 OR 越界"). The mandatory reject detail:
12 S4.6.4 RC-1 + RCE-1/RCE-2 (occ_count = 扫掠环内 occ==2 的格数; r_check_m =
r_robot + margin_rot; both 必填, a report with only reason 是契约不合规的). The
verbatim JSON is 12 S4.6.4. The prohibition on every shortcut: 12 S6A.3.2 逐字
"绝不要为此写"只看前半环"的豁免", and 12 S14 trap 15 "把未知格当空闲".

What this file does NOT do. It is not the production RC-1 grid evaluator -- that
belongs to P1 (12 S6A, C++/Python under p1_motion) and runs over a live LidarGrid
every 20 Hz tick. This is a coverage-level fail-safe that models the V-33 state
directly: front covered, sides/rear blind. It also does not lift the block: the
云深处 writeup and on-site recheck would only ever make rotation AVAILABLE, and
21 S1 is explicit that doing so here is out of scope (不得为了让功能可用而豁免).

The looks-right-but-wrong traps, each a real prohibition:
  * Only checking the front arc. If the front LiDAR sees a clear front, skipping
    the blind rear/sides makes the ring "pass" -- 12 calls this the原样重现 of the
    R-3 gap. build_v33_sweep_ring deliberately marks the FRONT free so that a
    front-only reading would wrongly succeed, and the test injects exactly that.
  * Reading an unknown cell as free. 12 S6A.3.2 RCG-3: 未知 == 把没看见的地方当成
    空的 == fail-open. evaluate_rotation_clearance treats OCC_UNKNOWN as blocking.
  * occ_count is honestly 0 here, because nothing is confirmed OCCUPIED -- the
    ring is blocked by unknown coverage, not obstacles. That is not a bug and not
    the "unknown as free" trap: occ_count counts occ==2 by contract (RCE-2), and
    the reason field, not the count, carries the true cause. Do not "fix" it to
    report the unknown count; that would diverge from what RCE-2 defines.
"""

from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

# E_BUSY comes from the shared library, never as a literal (CLAUDE.md 3.5). The
# import binds the string via the errors package __init__, so E_BUSY == "E_BUSY"
# but a typo is an ImportError at load, not a bad code on the wire.
from ..errors import E_BUSY
from .outcome import CONFIRM_L1B, STATUS_REJECTED, FailSafeResult

# Occupancy encoding, 12 S6A.3.2. Three states on the grid, and the two that
# block are NOT adjacent numerically -- 0 is unknown and 2 is occupied, with 1
# (free) between them -- which is why a "> 0 means blocked" shortcut would be
# wrong (it would pass 1/free and block... nothing sensible) and a "!= 1"
# shortcut is the only correct collapse. We keep all three named so the predicate
# reads as the contract writes it rather than as an arithmetic trick.
OCC_UNKNOWN = 0     # 未知: no sensor coverage confirmed this cell either way
OCC_FREE = 1        # 空闲: covered and clear -- the ONLY value that does not block
OCC_OCCUPIED = 2    # 占据: covered and something is there

# The reason token the E_BUSY detail must carry, 12 S13.8 第 (8) 类成因 (RCE-1:
# it is (8) not (7), (7) being approval_queue_full). E_BUSY's own detail requirement is
# "unspecified" in codes.yaml, but this specific cause fixes the reason string, so
# it is a named constant here rather than a literal sprinkled at construction.
REASON_ROTATION_BLOCKED = "rotation_blocked"

# Ring regions. The sweep ring is the annulus [r_self_mask, r_check] the body
# passes through as it spins; here it is modelled as four named arcs so that the
# "只看前半环" fail-open is expressible as "look at FRONT only" and testable. Real
# P1 works per-cell; the fail-safe works per-arc because under V-33 an entire arc
# shares one coverage state (the side band is empty, so the whole side arc is
# unknown), and that is all the block needs to decide.
REGION_FRONT = "front"
REGION_LEFT = "left"
REGION_RIGHT = "right"
REGION_REAR = "rear"

# The intents that route to this block. A09~A12 are the in-place turns and C07
# hands control to a behaviour that turns in place by bearing-error P control
# (12 S6A.4.2 lists target_oriented as a pure-wz source). Source rows:
#   A09 turn_left / A10 turn_right / A11 turn_around / A12 face_heading  18 S3.1
#   C07 set_motion_behavior                                             18 S6/C
# Held as data, cited, so the "一律 E_BUSY" assertion iterates a named set rather
# than a magic list buried in a test. This is the fail-safe's declared coverage,
# not a second copy of the intent registry (that lives in 16 S6.6 / p4_agent).
ROTATION_INTENTS = frozenset({"A09", "A10", "A11", "A12", "C07"})

# The intents that are lateral translation, NOT rotation: they are not blocked,
# but under V-33 they move into a blind direction, so 18 S3.1 fixes them at L1b
# (逐字 "判 L1a 直接违反契约"). Source rows A07 move_left / A08 move_right, 18 S3.1.
LATERAL_INTENTS = frozenset({"A07", "A08"})


@dataclass(frozen=True)
class CoverageBand:
    """One side's LiDAR coverage provenance, 11 S3.1.5 bands.*.

    Under V-33 both the left and right bands are empty: sources is () and d_m is
    None. The two together are what makes the corresponding ring arc unknown, and
    they are kept as fields (not folded into a single bool) because 21 S1 asserts
    on both independently -- sources 恒空 AND d_m 恒 null.
    """

    name: str                       # "left" | "right"
    sources: Tuple[str, ...]        # LiDAR source ids; () == no coverage
    d_m: Optional[float]            # nearest clearance in metres; None == null


@dataclass(frozen=True)
class RingSegment:
    """One arc of the sweep ring with its single coverage state."""

    region: str                     # one of the REGION_* names
    occ: int                        # one of OCC_UNKNOWN / OCC_FREE / OCC_OCCUPIED


@dataclass(frozen=True)
class SweepRing:
    """The modelled sweep ring plus the radius that would have been checked.

    r_check_m is NOT computed here. It is r_robot + margin_rot per 12 S6A.3.2,
    both of which are calibrated safety params resolved from config by the caller
    (CLAUDE.md 3.1 forbids this module owning a default for either). It is carried
    so the E_BUSY detail can echo the exact radius the check used, as RCE-2 makes
    mandatory. A None here is not defaulted to anything -- see build_v33_sweep_ring.
    """

    segments: Tuple[RingSegment, ...]
    r_check_m: float
    bands: Mapping[str, CoverageBand]


@dataclass(frozen=True)
class RotationClearance:
    """The verdict of evaluate_rotation_clearance."""

    blocked: bool                   # True == rotation must not proceed
    occ_count: int                  # cells with occ==2, for the mandatory detail


def build_v33_sweep_ring(r_check_m: float) -> SweepRing:
    """Construct the sweep ring in the V-33 state, for the given check radius.

    The front arc is marked FREE on purpose. It is the worst case for the fail-
    safe, not a lucky one: it proves that a clear, fully-covered front does NOT
    earn a rotation permit while the sides and rear are blind. An implementation
    that only looked at the front would pass this ring, which is the "只看前半环"
    fail-open 12 S6A.3.2 names -- so building the front free is what gives that
    mutant something to get wrong.

    r_check_m is required and echoed, never invented. Passing None would make the
    detail carry a null radius, which RCE-2 forbids; callers resolve r_robot +
    margin_rot first and hand the sum in.
    """
    # Both side bands empty and null -- the literal V-33 posture, 21 S1 (sources
    # 恒空, d_m 恒 null). Kept as real CoverageBand objects, not just a comment,
    # so a test can assert the posture rather than trust it.
    bands = {
        "left": CoverageBand(name="left", sources=(), d_m=None),
        "right": CoverageBand(name="right", sources=(), d_m=None),
    }
    # Front covered and clear; the three blind arcs unknown. count of occupied
    # (occ==2) is therefore zero -- the block is unknown-driven, which is the
    # honest state under V-33 and the reason occ_count comes out 0 downstream.
    segments = (
        RingSegment(region=REGION_FRONT, occ=OCC_FREE),
        RingSegment(region=REGION_LEFT, occ=OCC_UNKNOWN),
        RingSegment(region=REGION_RIGHT, occ=OCC_UNKNOWN),
        RingSegment(region=REGION_REAR, occ=OCC_UNKNOWN),
    )
    return SweepRing(segments=segments, r_check_m=r_check_m, bands=bands)


def evaluate_rotation_clearance(ring: SweepRing) -> RotationClearance:
    """Run the RC-1 blocked predicate over the whole ring.

    This is the single seam the first two mutants attack, so the loop is written
    plainly and completely:
      * it iterates EVERY segment -- narrowing the iterable to the front arc is
        the "只看前半环" fail-open (12 S6A.3.2);
      * OCC_UNKNOWN blocks exactly like OCC_OCCUPIED -- dropping the unknown
        branch is the "把未知格当空闲" fail-open (12 S14 trap 15 / RCG-3).
    Both collapse the verdict to blocked=False under the V-33 ring, which is why
    a single positive assertion (rotation on this ring is blocked) catches either.
    """
    occ_count = 0
    blocked = False
    # No early break. Every occupied cell must be counted for occ_count even after
    # blocked is already True, because RCE-2 requires the COUNT in the detail, not
    # just the boolean -- stopping at the first blocker would under-report it.
    for seg in ring.segments:
        if seg.occ == OCC_OCCUPIED:
            occ_count += 1
            blocked = True
        elif seg.occ == OCC_UNKNOWN:
            # 未知 counts as blocked. This branch is the whole of RCG-3; deleting
            # it is fail-open, and under V-33 (three unknown arcs) deleting it is
            # what would let every spin through.
            blocked = True
        # OCC_FREE: the only non-blocking state. Anything that is not one of the
        # three encoded values never reaches here as free -- it simply is not
        # equal to OCC_OCCUPIED or OCC_UNKNOWN and so does not block, which is why
        # callers must only ever build rings from the OCC_* constants.
    return RotationClearance(blocked=blocked, occ_count=occ_count)


def rotation_failsafe(ring: SweepRing) -> Optional[FailSafeResult]:
    """E_BUSY reject if the ring blocks rotation, else None.

    None means "this fail-safe has nothing to say -- fall through to normal RC-1".
    Under the V-33 ring it is never None; the branch returns None only for a ring
    with no unknown and no occupied arc, which is the state a mutated
    evaluate_rotation_clearance wrongly reports and the tests catch by that
    absence. The three detail keys are all present and mandatory (RCE-2): a report
    carrying only reason is契约不合规 and a peer rejects it as E_SCHEMA.
    """
    clearance = evaluate_rotation_clearance(ring)
    if not clearance.blocked:
        return None
    detail = {
        "reason": REASON_ROTATION_BLOCKED,   # 12 S13.8 (8)
        "occ_count": clearance.occ_count,    # 扫掠环内 occ==2 的格数 (RCE-2)
        "r_check_m": ring.r_check_m,         # r_robot + margin_rot echoed (RCE-2)
    }
    # E_BUSY, not a bespoke E_NO_CLEARANCE: 12 S6A.7 RC-D5 rules the transient
    # "环内有障碍/未知格" case reuses E_BUSY (EC-2 forbids inventing a code). The
    # persistent case (r_robot uncalibrated) is a DIFFERENT branch that returns
    # E_CAPABILITY -- not this one, and deliberately not merged, because E_BUSY's
    # client behaviour is backoff-retry and a retry against missing coverage never
    # clears. V-33 is the transient shape (coverage absent, could return), so it
    # is E_BUSY here.
    return FailSafeResult(status=STATUS_REJECTED, code=E_BUSY, detail=detail)


def lateral_move_confirm_level(intent: str) -> str:
    """A07/A08 lateral move -> L1b, statically, under V-33.

    Not a block: a lateral move is protected by f() / corridor / fence like any
    translation. What V-33 fixes is its CONFIRMATION level. Because the sides are
    blind, a sideways move is a move into an unsensed direction, and 18 S3.1 makes
    that L1b verbatim (判 L1a 直接违反契约, because 11 S3.1.5.3 (4) requires L1
    confirmation with a spoken 该方向无感知 warning). "Statically" means it does
    not depend on a runtime coverage read -- there is no side coverage to read --
    so the level is fixed, and downgrading it to L1a is the mutant this guards.
    """
    if intent not in LATERAL_INTENTS:
        # Fail-loud on a mis-route rather than silently returning a level. A
        # caller that sent a non-lateral intent here has a routing bug, and
        # returning L1b anyway would hide it and mislabel some other command.
        raise ValueError(
            f"lateral_move_confirm_level got {intent!r}, not a lateral intent "
            f"{sorted(LATERAL_INTENTS)} (18 S3.1 A07/A08)"
        )
    return CONFIRM_L1B


__all__ = [
    "OCC_UNKNOWN",
    "OCC_FREE",
    "OCC_OCCUPIED",
    "REASON_ROTATION_BLOCKED",
    "REGION_FRONT",
    "REGION_LEFT",
    "REGION_RIGHT",
    "REGION_REAR",
    "ROTATION_INTENTS",
    "LATERAL_INTENTS",
    "CoverageBand",
    "RingSegment",
    "SweepRing",
    "RotationClearance",
    "build_v33_sweep_ring",
    "evaluate_rotation_clearance",
    "rotation_failsafe",
    "lateral_move_confirm_level",
]
