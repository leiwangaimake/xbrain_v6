"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_accel_limit.py
Brief: 12 S8.1 stage 2 -- acceleration limited, deceleration never (99 U90 4)

Description:
Stage 2 of 12 S8.1 was never built (NEXT S8.6 P7.3 (3) verbatim "(2) 未建").
99 U90 4 settled the shape it had to take: the ramp constrains acceleration
ONLY, deceleration passes through untouched, and the f-band downshift plus
v_fence are exempt.

These cases pin all three, and the third one in a particular way: there is NO
case here that checks a limiter-name exemption list, because the
implementation deliberately has none. The f-band downshift and v_fence can
only ever LOWER the commanded speed, so "deceleration is unlimited" already
exempts them by construction. A criterion that asserted an exemption list
would be asserting a mechanism the ruling does not need -- and would go green
on an implementation that had the list but got the deceleration case wrong.

WHY THE DECELERATION HALF IS THE LOAD-BEARING ONE: U54 derived the
1.25 / 1.8 / 3.0 band boundaries FROM brake.a_mps2 = 2.5 (11 S9.6.1's
arithmetic-basis table). A ramp that limited deceleration would make the
robot unable to achieve the deceleration its own safety distances were
computed with. That is not a comfort regression, it is the robot being
unable to keep up with its own stopping distance.

MUTANTS, run 2026-10-09, all red (CLAUDE.md S3.3):
  symmetric ramp (limit deceleration too)      -> the deceleration cases
  pass acceleration through unclamped          -> the acceleration cases
  sign change treated as plain acceleration    -> the reversal case
  zero a_max accepted at construction          -> the construction case
  `v_cmd == 0.0` early return removed          -> the stop-now case

WHAT THESE CASES DO NOT COVER:
  * wz. The implementation does not limit it and says why: common.spec has
    no angular-acceleration key. Registered in docs/NEXT.md; there is
    nothing here to test because there is no parameter.
  * Whether a_max = 1.5 (the sim variant's value) is right. That is V-01.
    Every case below passes its own a_max in explicitly.
  * The CtrlLoop wiring -- that is tests/p1_motion/nav/test_ctrl_loop_accel.py.
    This file is the mechanism only.
"""

# pytest for raises/parametrize; the module under test.
import pytest

from xbrain.p1_motion.gate.accel_limit import (
    AccelLimits,
    AccelLimitUnconfigured,
    limit_axis,
)

#: Pure arithmetic over floats. No device, no clock.
pytestmark = pytest.mark.no_device

#: 20 Hz with the sim variant's a_max: budget = 1.5 * 0.05 = 0.075 m/s per
#: tick. Spelled out because every expected value below is built from it.
_LIM = AccelLimits(a_max_mps2=1.5, dt_s=0.05)
_BUDGET = 0.075


def test_the_budget_is_a_max_times_dt():
    """max_step_mps is the product, not a stored third value.

    Asserted because every other case in this file is expressed in terms of
    it: if the budget were computed some other way, the cases would all
    still agree with each other and with nothing in 12 S8.1.
    """
    assert _LIM.max_step_mps == pytest.approx(_BUDGET)


# ---------------------------------------------------------------------------
# Acceleration: limited
# ---------------------------------------------------------------------------

def test_acceleration_from_rest_is_clamped_to_one_budget():
    """0.0 -> 2.0 commanded gives 0.075, not 2.0.

    Starting from rest is the step command 12 S8.1 stage 2 names in as many
    words: "防止阶跃指令冲击本体". Without the clamp the chassis receives a
    full-speed command one tick after standing still.
    """
    assert limit_axis(2.0, 0.0, _LIM) == pytest.approx(_BUDGET)


def test_acceleration_from_speed_adds_at_most_one_budget():
    """1.0 -> 2.0 gives 1.075."""
    assert limit_axis(2.0, 1.0, _LIM) == pytest.approx(1.0 + _BUDGET)


def test_an_in_budget_acceleration_passes_through_unchanged():
    """1.0 -> 1.05 is within budget and must come back bit-identical.

    Not just "close enough": a limiter that recomputed every value as
    v_prev + budget would quietly round every small command up to the full
    budget, accelerating FASTER than asked in the common case.
    """
    assert limit_axis(1.05, 1.0, _LIM) == 1.05


def test_acceleration_is_limited_in_the_negative_direction_too():
    """-1.0 -> -2.0 gives -1.075. Backward is still acceleration.

    The axis is signed and reversing is not the only way to speed up. An
    implementation that compared raw values rather than magnitudes-with-sign
    would let backward acceleration through unlimited.
    """
    assert limit_axis(-2.0, -1.0, _LIM) == pytest.approx(-(1.0 + _BUDGET))


# ---------------------------------------------------------------------------
# Deceleration: never limited (99 U90 4)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("v_cmd,v_prev,why", [
    (0.5, 2.0, "f-band downshift: the gate dropped the cap"),
    (0.0, 2.0, "stop now: v_fence or a veto reduced the cap to zero"),
    (-0.5, -2.0, "same, travelling backward"),
    (0.0, -2.0, "stop now from backward"),
    (1.999, 2.0, "a one-tick trim far smaller than the budget"),
])
def test_deceleration_passes_through_whatever_the_step_size(v_cmd, v_prev, why):
    """Deceleration is returned unchanged, however large the step.

    2.0 -> 0.0 is a step of 2.0 m/s, 26 times the budget. A symmetric ramp
    would turn it into 1.925 and take 27 ticks (1.35 s) to actually stop --
    which is the failure 99 U90 4 describes: the robot cannot achieve the
    deceleration its own safety distances assume.
    """
    assert limit_axis(v_cmd, v_prev, _LIM) == v_cmd, why


def test_commanding_zero_is_never_ramped():
    """Zero is returned as zero from ANY previous speed.

    Separate from the parametrized case above because this is the one the
    vetoes and v_fence all funnel into, and because it is reachable by an
    early return that a mutant can remove while leaving the general
    deceleration path intact.
    """
    for v_prev in (0.0, 0.1, 2.0, -2.0, 1e6):
        assert limit_axis(0.0, v_prev, _LIM) == 0.0, v_prev


# ---------------------------------------------------------------------------
# Sign change: stop at zero this tick
# ---------------------------------------------------------------------------

def test_a_reversal_stops_at_zero_rather_than_crossing():
    """+0.5 -> -2.0 gives 0.0, not -0.425 and not -2.0.

    12 S8.1 does not define this case; the implementation derives it from
    the ruling and says so. The derivation: a reversal contains a
    deceleration to zero (unlimited, so it completes) AND a reverse
    acceleration (limited, so it starts next tick from rest).

    The two wrong answers this rules out:
      -0.425 (= v_prev - budget) would be a symmetric ramp, limiting the
             deceleration half;
      -2.0   would let a full reversal land in one tick, which is a larger
             step than any acceleration this limiter permits.
    """
    assert limit_axis(-2.0, 0.5, _LIM) == 0.0
    # And the tick after, from rest, the reverse acceleration is limited.
    assert limit_axis(-2.0, 0.0, _LIM) == pytest.approx(-_BUDGET)


def test_a_reversal_from_backward_also_stops_at_zero():
    """-0.5 -> +2.0 gives 0.0. Symmetric in the other direction."""
    assert limit_axis(2.0, -0.5, _LIM) == 0.0


# ---------------------------------------------------------------------------
# Construction: no defaults, no silent zero (CLAUDE.md S3.1)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("a_max,dt,what", [
    (0.0, 0.05, "a_max of zero would freeze every axis at its previous value"),
    (-1.5, 0.05, "negative a_max"),
    (1.5, 0.0, "dt of zero makes the budget zero, same freeze"),
    (1.5, -0.05, "negative dt"),
])
def test_construction_refuses_an_unusable_parameter(a_max, dt, what):
    """Non-positive a_max or dt raises AccelLimitUnconfigured.

    a_max = 0.0 is the case worth naming: the budget becomes zero, every
    acceleration is clamped to v_prev, and the robot holds whatever speed it
    had forever with no error anywhere. That is CLAUDE.md S3.1's "0.0
    passing for assigned" exactly, and it is why this is a refusal rather
    than an absorbed edge case.
    """
    with pytest.raises(AccelLimitUnconfigured) as ei:
        AccelLimits(a_max_mps2=a_max, dt_s=dt)
    # The message must name the config key an operator has to go fix.
    assert ("max_accel_mps2" in str(ei.value)
            or "dt_s" in str(ei.value)), what


def test_there_is_no_decel_field_to_misuse():
    """AccelLimits carries no max_decel field.

    An attractive nuisance, deliberately absent: the only thing stage 2
    could do with a deceleration limit is limit deceleration, which 99 U90 4
    forbids. Leaving the field out makes that unavailable rather than merely
    discouraged -- a future edit has to add the field first, which is a
    visible change.
    """
    assert not hasattr(_LIM, "max_decel_mps2")
    assert "decel" not in repr(_LIM)
