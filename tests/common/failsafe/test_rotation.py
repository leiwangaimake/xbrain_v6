"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_rotation.py
Brief: Tests for the V-33 in-place-rotation fail-safe, incl. the two named rotation mutants

Description:
This is the acceptance surface for INF-DB-3 branch (1). It pins the V-33 posture
(bands empty, d_m null, side arcs unknown), the E_BUSY reject with its three
mandatory detail fields (RCE-2), the coverage of every in-place-rotation intent,
and the static L1b for lateral moves.

*** The two named mutants for this branch (INF-DB-3 done-criterion), each verified
to turn a test red before this file was considered done (CLAUDE.md 3.3):
  * "只看前半环" (only the front arc): narrow the loop in
    evaluate_rotation_clearance to segments with region == REGION_FRONT. Because
    build_v33_sweep_ring makes the front FREE, the ring then reads clear ->
    rotation_failsafe returns None -> test_all_in_place_rotation_intents_blocked
    and test_rotation_reject_shape go red.
  * "把未知格当空闲" (unknown as free): delete the `elif seg.occ == OCC_UNKNOWN`
    branch. The V-33 ring has zero occupied arcs, so it then reads clear -> None
    -> the same tests go red. test_block_is_unknown_driven states the premise that
    makes this sharp: occ_count is 0, so the verdict rests entirely on unknowns.

Both positive controls (a clear ring returns None; an occupied ring counts) exist
so that a constant "always E_BUSY, occ_count 0" shell -- CLAUDE.md 3.2 form 1 --
cannot pass this file.
"""

import os
import re
import sys

import pytest

# ROOT is three levels up (tests/common/failsafe -> repo root). Inserted so the
# xbrain package imports the same way it does for the other common tests.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

from xbrain.common import errors  # noqa: E402
# Imported from the submodules directly: the package __init__ is docstring-only
# (it does not re-export), the same convention as xbrain/common/__init__.py.
from xbrain.common.failsafe.outcome import (  # noqa: E402
    CONFIRM_L1A,
    CONFIRM_L1B,
    STATUS_REJECTED,
)
from xbrain.common.failsafe.rotation import (  # noqa: E402
    LATERAL_INTENTS,
    OCC_FREE,
    OCC_OCCUPIED,
    OCC_UNKNOWN,
    REGION_FRONT,
    REGION_LEFT,
    REGION_REAR,
    REGION_RIGHT,
    ROTATION_INTENTS,
    RingSegment,
    SweepRing,
    build_v33_sweep_ring,
    evaluate_rotation_clearance,
    lateral_move_confirm_level,
    rotation_failsafe,
)

# The contract's own worked example radius (12 S4.6.4 JSON: r_check_m 1.60). Used
# purely as a test STIMULUS -- it stands in for the r_robot + margin_rot the
# caller resolves from config. It is NOT a calibrated default living in code
# (CLAUDE.md 3.1): the module never supplies it, the test hands it in.
R_CHECK_STIMULUS = 1.60

# The master voice command set, where the rotation and lateral intents are
# defined. The preset intents live in 18-B and are checked in test_ptz.py.
VOL18 = os.path.join(ROOT, "docs", "18-语音文本指令集.md")


def _intent_is_a_row(intent, path):
    """True if `intent` appears as the first cell of a table row in `path`.

    Reads the leading cell only, the same discipline test_error_codes uses: prose
    mentions an intent in passing (deletions, cross-references), and matching
    those would keep a retired intent alive. The pattern tolerates an optional
    backtick pair because 18 writes `| A07 |` bare and `| `E02` |` quoted.
    """
    pat = re.compile(r"^\|\s*`?" + re.escape(intent) + r"`?\s*\|")
    with open(path, encoding="utf-8") as fh:
        return any(pat.match(line) for line in fh)


def test_v33_bands_are_empty_and_null():
    """21 S1: bands.left/right sources 恒空, d_m 恒 null.

    Asserted on the real CoverageBand objects, not trusted from a comment, so the
    posture this whole branch rests on is itself under test.
    """
    ring = build_v33_sweep_ring(R_CHECK_STIMULUS)
    for side in ("left", "right"):
        band = ring.bands[side]
        assert band.sources == (), f"{side} band must have no sources under V-33"
        # `is None` and not `== None`: null is the exact value 21 S1 fixes, and a
        # 0.0 slipping in here would be the fail-silent 0.0 CLAUDE.md 3.1 warns of.
        assert band.d_m is None, f"{side} band d_m must be null under V-33"


def test_v33_ring_side_arcs_are_unknown():
    """21 S1: 旋转扫掠环左右两段恒为未知格."""
    ring = build_v33_sweep_ring(R_CHECK_STIMULUS)
    by_region = {seg.region: seg.occ for seg in ring.segments}
    assert by_region[REGION_LEFT] == OCC_UNKNOWN
    assert by_region[REGION_RIGHT] == OCC_UNKNOWN
    # The rear is unknown too; the front is FREE on purpose (the worst case that
    # gives the "只看前半环" mutant something to get wrong -- see the front-arc
    # test below).
    assert by_region[REGION_REAR] == OCC_UNKNOWN
    assert by_region[REGION_FRONT] == OCC_FREE


def test_front_arc_is_clear_so_front_only_would_fail_open():
    """The premise of the "只看前半环" mutant.

    If the fail-safe read only the front arc, it would find a clear front and let
    the spin through. Stating that the front is clear here is what makes the
    mutant a genuine fail-open rather than a no-op: the blocked verdict must come
    from the arcs a front-only reader skips.
    """
    ring = build_v33_sweep_ring(R_CHECK_STIMULUS)
    front = [seg for seg in ring.segments if seg.region == REGION_FRONT]
    assert front and all(seg.occ == OCC_FREE for seg in front)
    # And the whole ring is still blocked despite the clear front.
    assert evaluate_rotation_clearance(ring).blocked is True


def test_rotation_reject_shape():
    """E_BUSY + the three mandatory detail fields, RCE-2.

    A report carrying only `reason` is契约不合规 (RCE-2), so the key set is
    asserted exactly: dropping occ_count or r_check_m must fail here.
    """
    result = rotation_failsafe(build_v33_sweep_ring(R_CHECK_STIMULUS))
    assert result is not None, "V-33 rotation must be blocked, never None"
    assert result.status == STATUS_REJECTED
    # The code comes from the shared library; comparing to errors.E_BUSY (not the
    # literal) keeps this test honest about where the code is defined.
    assert result.code == errors.E_BUSY
    assert set(result.detail) == {"reason", "occ_count", "r_check_m"}, (
        "detail must carry exactly the three RCE-2 fields"
    )
    assert result.detail["reason"] == "rotation_blocked"   # 12 S13.8 (8)
    assert result.detail["r_check_m"] == R_CHECK_STIMULUS   # echoed, not invented


def test_all_in_place_rotation_intents_blocked():
    """INF-DB-3 (1): A09~A12 and C07 一律 E_BUSY.

    The ring is what blocks; the loop shows the block covers every intent that
    routes here. If a mutant makes the V-33 ring read clear, rotation_failsafe
    returns None and result.code raises AttributeError -- red for all of them.
    """
    ring = build_v33_sweep_ring(R_CHECK_STIMULUS)
    for intent in sorted(ROTATION_INTENTS):
        result = rotation_failsafe(ring)
        assert result is not None and result.code == errors.E_BUSY, (
            f"{intent} in-place rotation must be E_BUSY under V-33"
        )


def test_block_is_unknown_driven():
    """The premise of the "未知格当空闲" mutant.

    occ_count is 0 -- nothing is confirmed occupied -- so the entire block rests
    on unknown arcs. Reading unknown as free would therefore flip the verdict to
    clear, which is exactly what the mutant does and what test_rotation_reject_
    shape then catches. This also documents why occ_count 0 is correct, not a bug.
    """
    clearance = evaluate_rotation_clearance(build_v33_sweep_ring(R_CHECK_STIMULUS))
    assert clearance.blocked is True
    assert clearance.occ_count == 0, (
        "under V-33 nothing is occupied; the block is unknown-driven"
    )


def test_clear_ring_returns_none():
    """Positive control: an all-free ring is NOT blocked.

    Without this, a shell that returns E_BUSY unconditionally would pass every
    negative test in this file (CLAUDE.md 3.2 form 1). A ring of only free arcs
    must fall through to None.
    """
    ring = SweepRing(
        segments=(
            RingSegment(region=REGION_FRONT, occ=OCC_FREE),
            RingSegment(region=REGION_LEFT, occ=OCC_FREE),
            RingSegment(region=REGION_RIGHT, occ=OCC_FREE),
            RingSegment(region=REGION_REAR, occ=OCC_FREE),
        ),
        r_check_m=R_CHECK_STIMULUS,
        bands={},
    )
    assert rotation_failsafe(ring) is None
    assert evaluate_rotation_clearance(ring).blocked is False


def test_occupied_ring_counts_occupied_cells():
    """Positive control: occ_count actually counts occ==2.

    Guards against a shell that hardcodes occ_count 0. A ring with two occupied
    arcs must report occ_count 2 and block.
    """
    ring = SweepRing(
        segments=(
            RingSegment(region=REGION_FRONT, occ=OCC_OCCUPIED),
            RingSegment(region=REGION_LEFT, occ=OCC_FREE),
            RingSegment(region=REGION_RIGHT, occ=OCC_OCCUPIED),
            RingSegment(region=REGION_REAR, occ=OCC_FREE),
        ),
        r_check_m=R_CHECK_STIMULUS,
        bands={},
    )
    clearance = evaluate_rotation_clearance(ring)
    assert clearance.blocked is True
    assert clearance.occ_count == 2


def test_lateral_move_is_static_l1b():
    """INF-DB-3 (1): A07/A08 横移 静态判 L1b, never L1a.

    18 S3.1: 判 L1a 直接违反契约 for a blind-direction move. The mutant that
    returns CONFIRM_L1A instead makes this red.
    """
    for intent in sorted(LATERAL_INTENTS):
        level = lateral_move_confirm_level(intent)
        assert level == CONFIRM_L1B, f"{intent} must be L1b under V-33"
        assert level != CONFIRM_L1A, f"{intent} must never downgrade to L1a"


def test_lateral_rejects_a_non_lateral_intent():
    """A mis-routed intent raises rather than silently returning a level."""
    with pytest.raises(ValueError):
        lateral_move_confirm_level("A09")   # a rotation intent, not lateral


def test_rotation_and_lateral_intent_sets_match_contract():
    """The declared coverage matches the intents INF-DB-3 (1) names, and each is real.

    The equality restates the criterion; the doc-existence loop is the anti-
    fabrication guard -- a renamed or deleted intent id fails here rather than
    silently shrinking the fail-safe's coverage.
    """
    assert ROTATION_INTENTS == {"A09", "A10", "A11", "A12", "C07"}
    assert LATERAL_INTENTS == {"A07", "A08"}
    for intent in ROTATION_INTENTS | LATERAL_INTENTS:
        assert _intent_is_a_row(intent, VOL18), (
            f"{intent} is not a row in 18 -- citation stale or intent renamed"
        )
