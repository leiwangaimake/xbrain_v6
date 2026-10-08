"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: accel_limit.py
Brief: 12 S8.1 stage 2 -- |v - v_prev| <= a_max*dt, ACCELERATION ONLY (99 U90 4)

Description:
12 S8.1 lists three stages in a fixed order and stage 2 is the one that was
never built (NEXT S8.6 P7.3 (3) verbatim "(2) 未建"):

  (1) profile / gait clamp        per-axis amplitude      -- in the speed gate
  (2) |v - v_prev| <= a_max*dt    anti step-command       -- THIS FILE
  (3) axis validity               zero the invalid axes   -- in CtrlLoop

The order is not interchangeable, and 12 S8.1 says why verbatim: "(3) 必须最后,
否则被置零的轴仍会参与 (2) 的变化率计算, 产生虚假的加速度限制".

*** THE RULING THIS FILE IMPLEMENTS (99 U90 4, user 2026-10-08)

The ramp constrains ACCELERATION ONLY. Deceleration is never limited, and the
f-band downshift and v_fence are exempt -- both halves of one ruling, not a
choice between them. The reasoning is physical, not a preference:

  * A ramp protects the machine and the ride. That is an acceleration-side
    concern. Braking harder than comfortable is not a hazard; braking slower
    than commanded is.
  * U54 derived the 1.25 / 1.8 / 3.0 band boundaries FROM brake.a_mps2 = 2.5
    (11 S9.6.1's arithmetic-basis table). Limiting deceleration would mean
    the robot cannot achieve the deceleration its own safety distances were
    computed with -- it would be unable to keep up with its own stopping
    distance.
  * v_fence IS a brake-solve ("can I stop within d_eff"). Putting a ramp on
    a value that already means "the fastest I may go and still stop in time"
    is self-contradictory.

*** WHY "DECELERATION IS NOT LIMITED" ALREADY GIVES THE TWO EXEMPTIONS

The f-band downshift and v_fence can only ever LOWER the commanded speed. So
once deceleration passes through unlimited, both are exempt by construction
-- there is nothing further to special-case, and in particular this file does
NOT inspect gate.limiter to decide. That matters because a limiter-based
exemption list is the shape that rots: a new limiter gets added, nobody adds
it to the list, and the ramp silently starts fighting a safety reduction.
The f-band UPSHIFT is acceleration and is limited, which is correct -- the
upshift already carries its own 3.0 s hysteresis.

*** WHAT THIS FILE DOES NOT TOUCH

  * wz. 12 S8.1 stage 2 is written for "v" without naming axes, but
    common.spec has no angular-acceleration key at all (max_vx_mps /
    max_vy_mps / max_wz_radps / max_accel_mps2 / max_decel_mps2 -- the last
    two are linear). There is no parameter to limit wz with, and inventing
    one would be CLAUDE.md S3.1 in reverse: a safety-adjacent number with no
    calibration behind it. Registered in docs/NEXT.md.
  * Vetoes. 12 S8.1's own note is verbatim: "S6A 的否决 (wz <- 0) 是否决语义,
    立即生效, 不受 (2) 加加速度限幅约束 ... 若把 S6A 的否决塞进 (2) 做斜坡,
    则"拒绝旋转"会变成"先慢慢转一会儿再停"-- 门就漏了". estop and
    SAFE_STOP are the same shape. The caller keeps those OUT of this
    function; CtrlLoop does so by applying the limiter only on the ACTIVE
    branch.

*** SIGN CHANGE: a boundary 12 S8.1 does not define, handled explicitly

A command that reverses sign (+0.5 -> -2.0) contains a deceleration to zero
AND a reverse acceleration. Treating it as one "change of 2.5" would limit
the deceleration half, which the ruling forbids; treating it as unlimited
would let a full reversal land in one tick. So this file stops the tick AT
ZERO: the deceleration half is unlimited and completes, and the reverse
acceleration starts from zero on the next tick under the normal limit.
That is derived from the ruling rather than specified by it -- see the
module's criteria file for the case that pins it.

All parameters injected at construction, no defaults (CLAUDE.md S3.1).
"""

from __future__ import annotations

# dataclass for the frozen parameter object; no typing imports needed --
# every parameter is a plain float and absence is a construction error,
# not an Optional.
from dataclasses import dataclass


class AccelLimitUnconfigured(ValueError):
    """Raised at construction when a_max_mps2 or dt_s is missing/unusable.

    A dedicated type, not a bare ValueError: the caller (nav_wiring) has to
    distinguish "this deployment has not been calibrated yet" -- which is
    the designed state while common.spec.max_accel_mps2 is null (V-01 is
    the calibration item) -- from a programming error. CLAUDE.md S3.1
    forbids defaulting it, so refusing to construct is the only other
    option; what the caller does with the refusal is the caller's call.
    """


@dataclass(frozen=True)
class AccelLimits:
    """Stage-2 parameters. Both required, both validated at construction.

    a_max_mps2 is common.spec.max_accel_mps2 (null on the real machine
    until V-01; models/m20s_sim.yaml carries 1.5 so the mechanism can be
    exercised). dt_s is the control period -- 0.05 at 20 Hz.

    NO max_decel_mps2 field. It would be an attractive nuisance: the only
    thing this file could do with it is limit deceleration, which 99 U90 4
    forbids. Leaving the field out makes that unavailable rather than
    merely discouraged.
    """

    a_max_mps2: float
    dt_s: float

    def __post_init__(self) -> None:
        # Strictly positive, both. Zero a_max would freeze every axis at
        # its previous value -- the robot would hold whatever speed it had
        # and never change it, with no error anywhere. That is exactly the
        # fail-silent shape CLAUDE.md S3.1 describes for 0.0 passing as a
        # calibrated value, so it is refused here rather than absorbed.
        if not (self.a_max_mps2 > 0.0):
            raise AccelLimitUnconfigured(
                "common.spec.max_accel_mps2 must be > 0, got %r"
                % (self.a_max_mps2,))
        if not (self.dt_s > 0.0):
            raise AccelLimitUnconfigured(
                "control period dt_s must be > 0, got %r" % (self.dt_s,))

    @property
    def max_step_mps(self) -> float:
        """The per-tick budget a_max * dt, computed once per use site."""
        # A property rather than a stored field so the two inputs stay the
        # single source: a stored product could drift from them if either
        # were ever made mutable.
        return self.a_max_mps2 * self.dt_s


def limit_axis(v_cmd: float, v_prev: float, lim: AccelLimits) -> float:
    """One axis through 12 S8.1 stage 2. Returns the allowed command.

    Three cases, in the order they are tested:

      deceleration    |v_cmd| <= |v_prev|, same sign   -> v_cmd unchanged
      sign change     v_cmd and v_prev differ in sign  -> 0.0 this tick
      acceleration    otherwise                        -> clamped to
                                                          v_prev +- budget

    `v_prev` is the previously PUBLISHED value, not the previously computed
    one. Those differ on any tick where a veto zeroed the axis, and using
    the computed value would let the ramp resume from a speed the chassis
    never saw -- the robot would jump to it in one tick.
    """
    budget = lim.max_step_mps
    # Same-sign test via product > 0. `v_prev == 0.0` gives a product of
    # 0.0, which is NOT "same sign" -- starting from rest is acceleration
    # and takes the third branch, which is what we want.
    same_sign = (v_cmd * v_prev) > 0.0
    # -- deceleration: unlimited (99 U90 4) --------------------------------
    # Includes commanding exactly zero from any speed, which is the f-band
    # downshift / v_fence / "stop now" case and must not be slowed down.
    #
    # EQUIVALENT MUTANT, noted rather than asserted (CLAUDE.md S7.2.1):
    # removing the `v_cmd == 0.0 or` clause changes nothing observable.
    # Trace it: with v_cmd == 0.0, same_sign is False (the product is 0.0),
    # so this branch is skipped; if v_prev != 0 the sign-change branch below
    # returns 0.0, and if v_prev == 0 the acceleration branch finds
    # |0 - 0| <= budget and returns 0.0. Same answer by three routes.
    # The clause is kept because it states the intent at the top -- a reader
    # looking for "does a stop command get ramped" finds the answer here
    # instead of having to trace two more branches -- and because it is the
    # one route that stays correct if either branch below is ever changed.
    if v_cmd == 0.0 or (same_sign and abs(v_cmd) <= abs(v_prev)):
        return v_cmd
    # -- sign change: stop at zero this tick -------------------------------
    # Not same_sign and v_cmd != 0 and v_prev != 0 means a reversal. The
    # deceleration half runs to completion (unlimited) and the reverse
    # acceleration begins next tick from rest.
    if v_prev != 0.0 and not same_sign:
        return 0.0
    # -- acceleration: clamp the step --------------------------------------
    # From rest (v_prev == 0) this gives +-budget, which is the correct
    # start-up ramp. From speed it adds at most one budget in the direction
    # already travelled.
    if abs(v_cmd - v_prev) <= budget:
        # Already within budget: pass it through rather than recomputing,
        # so an in-budget command is bit-identical to what was asked for.
        return v_cmd
    step = budget if v_cmd > v_prev else -budget
    return v_prev + step
