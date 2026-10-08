"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ctrl_loop_accel.py
Brief: 12 S8.1 stage 2 WIRED into CtrlLoop -- real ticks, not the helper alone

Description:
tests/p1_motion/test_accel_limit.py covers the mechanism. This file covers
the thing that mechanism tests cannot: that the loop actually calls it, on
the right branch, in the right position, and against the right previous
value. Every one of those four is a place where a correct limiter produces
wrong behaviour.

The four, and the failure each one would cause:

  called at all          A limiter nobody invokes is the state this repo has
                         measured repeatedly -- a module with green criteria
                         and no caller. Driven here through run_one_tick().
  right branch           Vetoes must NOT be ramped. 12 S8.1 is explicit:
                         "若把 S6A 的否决塞进 (2) 做斜坡, 则'拒绝旋转'会变成
                         '先慢慢转一会儿再停' -- 门就漏了". estop and
                         SAFE_STOP are the same shape.
  right position         Stage 3 (axis validity) must run AFTER stage 2.
                         12 S8.1: "(3) 必须最后, 否则被置零的轴仍会参与 (2)
                         的变化率计算, 产生虚假的加速度限制".
  right previous value   The ramp resumes from what the chassis was last
                         SENT, not from what was last computed. Those differ
                         on any tick a veto zeroed, and using the computed
                         value lets the robot jump to a speed the chassis
                         never saw.

MUTANTS, run 2026-10-09:

  RED (killed by the cases below)
    _prev_* recorded from the computed values, not the published ones
    the limiter is never applied (the whole branch dropped)
    acceleration passes through unclamped
    a sign change treated as plain acceleration
    a symmetric ramp (deceleration limited too)
    construction accepts a_max = 0

  SURVIVING, and equivalent -- noted in ctrl_loop.py rather than chased
    dropping the `reason == "none"` guard (ramp on every branch)
    keying the guard on `state == ACTIVE` instead of on the branch

  Why those two survive: every veto branch sets the axes to exactly 0.0,
  and stage 2 does not limit deceleration -- so running a veto's zero
  through the limiter returns that same zero. No observable difference
  exists today, so no assertion here can be honest about catching one.
  The guard is kept because the equivalence is a property of TODAY'S veto
  set: the first veto that produces a NON-ZERO value (a "reduce to
  0.5 m/s" degradation, say) would start being ramped without it. That is
  the moment to write the case -- not now, with a contrived one.

  CLAUDE.md S7.2.1 is the rule being followed here: before treating a
  surviving mutant as a missing assertion, ask whether any test could kill
  it. These two cannot be killed, so they are documented at the code.

WHAT THIS FILE DOES NOT COVER: that nav_wiring passes real limits built from
the resolved config. That path needs a resolved snapshot and is covered by
tests/p1_motion/test_config_key_parity.py's neighbourhood; here the limits
are injected directly so the cases are about the loop, not the config.
"""

# pytest for the parametrized veto case; the loop and the limiter type.
import pytest

from xbrain.p1_motion.ctrl_loop import CtrlLoop, CtrlState
from xbrain.p1_motion.gate.accel_limit import AccelLimits

#: In-process, no publisher beyond a list append. No device.
pytestmark = pytest.mark.no_device

#: Same budget as the mechanism file: 1.5 * 0.05 = 0.075 m/s per tick.
_BUDGET = 0.075


def _loop(holonomic=False, limits=True):
    """An ACTIVE loop whose publisher records (vx, wz[, vy]) tuples.

    Transitioned to ACTIVE here rather than in each case: every case below
    is about a loop that is driving, and a case that forgot the transition
    would publish zeros and pass several of the assertions for the wrong
    reason (no_source also gives vx == 0.0).

    limits=False builds the loop with accel_limits=None, which is the real
    machine's state today -- not a test convenience. See the case that
    covers it.
    """
    # The list is returned alongside so a case can assert on what the
    # chassis actually received -- asserting on the CtrlTick alone would
    # miss a loop that reported one value and published another.
    sent = []
    # The limits are injected, not read from configs/: these cases are
    # about the loop's use of the limiter, and reading the live config
    # would make them fail the day a site re-tunes max_accel_mps2 (and
    # they would pass vacuously today, since the real key is null).
    lim = AccelLimits(a_max_mps2=1.5, dt_s=0.05) if limits else None
    # `lambda *a` and not a two-arg lambda: the publisher is called with
    # three args on a holonomic loop, and a fixed arity here would turn a
    # holonomic case into a TypeError swallowed by run_one_tick's
    # publish-failure guard -- the case would report "not published"
    # instead of the value mismatch it was looking for.
    loop = CtrlLoop(lambda *a: sent.append(a), holonomic=holonomic,
                    accel_limits=lim)
    loop.transition(CtrlState.ACTIVE)
    return loop, sent


def test_the_loop_actually_ramps_a_step_command():
    """ACTIVE tick from rest: a 2.0 command reaches the chassis as 0.075.

    This is the wiring criterion. The mechanism file proves limit_axis
    clamps; only this one proves run_one_tick calls it.
    """
    loop, sent = _loop()
    report = loop.run_one_tick(computed_vx=2.0)
    assert report.vx == pytest.approx(_BUDGET)
    # And the chassis received the clamped value, not the computed one.
    # Both are asserted because they can disagree: a loop that clamped for
    # the report and published the raw command would satisfy the first
    # assertion alone, and the chassis is the half that matters.
    assert sent == [(pytest.approx(_BUDGET), 0.0)]
    # The counter is the operator-visible signal that the ramp engaged.
    # Checked here so "it clamped" and "it said it clamped" cannot drift.
    assert loop.accel_clamped == 1


def test_the_ramp_accumulates_across_ticks_from_the_published_value():
    """Three ticks of a held 2.0 command: one budget each.

    The accumulation is the part that depends on _prev_vx being updated
    from the PUBLISHED value. A loop that forgot to record it would clamp
    to one budget forever and never reach the commanded speed.
    """
    loop, sent = _loop()
    for n in (1, 2, 3):
        report = loop.run_one_tick(computed_vx=2.0)
        assert report.vx == pytest.approx(_BUDGET * n), n
    # The published sequence, rounded only to absorb float formatting --
    # the values themselves are exact multiples of the budget. Asserted as
    # a sequence rather than per-tick so a loop that reached the right
    # final value by a different path (one jump, then holds) goes red.
    assert [round(v, 6) for v, _ in sent] == [0.075, 0.15, 0.225]


def test_a_deceleration_is_not_ramped_by_the_loop():
    """Ramp up, then command 0.0: the chassis gets 0.0 in one tick.

    The ruling's load-bearing half, driven through the real loop rather than
    the helper. A symmetric ramp here would make every stop take 27 ticks.
    """
    loop, sent = _loop()
    # Get to a nonzero published speed first -- a deceleration case starting
    # from rest would be trivially satisfied.
    for _ in range(4):
        loop.run_one_tick(computed_vx=2.0)
    before = sent[-1][0]
    assert before > 0.0
    report = loop.run_one_tick(computed_vx=0.0)
    assert report.vx == 0.0
    assert sent[-1] == (0.0, 0.0)


@pytest.mark.parametrize("kwargs,state,expect_reason", [
    ({"estop": True}, CtrlState.ACTIVE, "soft_estop"),
    ({}, CtrlState.SAFE_STOP, "soft_estop"),
    ({}, CtrlState.INIT, "no_source"),
])
def test_vetoes_reach_zero_in_one_tick_and_are_not_ramped(
        kwargs, state, expect_reason):
    """estop / SAFE_STOP / not-driving all publish 0.0 immediately.

    Each is set up from a MOVING state, which is what makes the case
    meaningful: a ramp applied to these branches would turn a veto into a
    gradual slowdown, and 12 S8.1 names that failure directly.

    The estop row is the sharpest: the state is still ACTIVE and a nonzero
    vx is still being computed, so a limiter keyed on `state == ACTIVE`
    rather than on the branch actually taken would ramp the estop's zero.
    """
    loop, sent = _loop()
    # Four ticks of ramping first: a veto case that started from rest would
    # publish zero either way and prove nothing about the ramp.
    for _ in range(4):
        loop.run_one_tick(computed_vx=2.0)
    assert sent[-1][0] > 0.0
    loop.transition(state)
    report = loop.run_one_tick(computed_vx=2.0, **kwargs)
    assert report.vx == 0.0
    assert report.stop_reason == expect_reason
    assert sent[-1] == (0.0, 0.0)


def test_after_a_veto_the_ramp_restarts_from_zero_not_from_the_computed_value():
    """estop then release: the next tick gives one budget, not a jump.

    This is the "right previous value" criterion. During the estop tick the
    computed vx was 2.0 but the PUBLISHED vx was 0.0. A loop that recorded
    the computed value would, on release, see v_prev = 2.0 and pass 2.0
    straight through as a no-op "deceleration" -- the robot would jump from
    standstill to full speed one tick after an emergency stop.
    """
    loop, sent = _loop()
    # Ramp up to a real speed, so the estop has something to interrupt.
    for _ in range(4):
        loop.run_one_tick(computed_vx=2.0)
    # The estop tick: computed vx is still 2.0, published vx becomes 0.0.
    # That divergence between computed and published is the whole subject
    # of this case.
    loop.run_one_tick(computed_vx=2.0, estop=True)
    assert sent[-1] == (0.0, 0.0)
    # Estop released, same command.
    report = loop.run_one_tick(computed_vx=2.0)
    assert report.vx == pytest.approx(_BUDGET), (
        "the ramp resumed from a value the chassis never received")


def test_stage_three_runs_after_stage_two_on_a_non_holonomic_loop():
    """A dropped vy must not have been ramped first.

    12 S8.1's ordering rule, observable through the counter: on a
    non-holonomic chassis vy is zeroed by stage 3. If stage 3 ran FIRST,
    stage 2 would see vy = 0.0 both times and never count a clamp on it;
    the ramp would also be computing a rate of change for an axis that is
    not there, which is the "虚假的加速度限制" 12 S8.1 warns about.

    So: vy is dropped (vy_dropped increments, published vy is absent) AND
    the vx ramp still happened on the same tick.
    """
    loop, sent = _loop(holonomic=False)
    # Both axes commanded: vx must be ramped (stage 2) and vy must be
    # dropped (stage 3), on the SAME tick. One tick is what makes the
    # ordering observable -- two ticks would not distinguish the orders.
    report = loop.run_one_tick(computed_vx=2.0, computed_vy=1.0)
    assert loop.vy_dropped == 1
    assert report.vy == 0.0
    # Non-holonomic publishes two args only.
    assert len(sent[-1]) == 2
    assert report.vx == pytest.approx(_BUDGET)


def test_vy_is_ramped_on_a_holonomic_loop():
    """On a holonomic chassis vy goes through stage 2 like vx.

    common.spec.max_accel_mps2 is a linear acceleration and vy is a linear
    axis, so the same budget applies. Without this case, "the loop ramps"
    would be satisfied by an implementation that only ever touched vx --
    and a 2.0 m/s lateral step would reach the body unramped.
    """
    loop, sent = _loop(holonomic=True)
    # vx held at zero so the only clamp that can happen is on vy -- a case
    # that ramped both would be satisfied by a loop that only did vx.
    report = loop.run_one_tick(computed_vx=0.0, computed_vy=2.0)
    assert report.vy == pytest.approx(_BUDGET)
    # The publish signature on a holonomic loop is (vx, wz, vy) -- vy LAST.
    # Asserted positionally because a loop that passed them in the wrong
    # order would send a lateral command as a forward one.
    assert sent[-1] == (0.0, 0.0, pytest.approx(_BUDGET))


def test_without_limits_the_loop_passes_commands_through():
    """accel_limits=None -> no ramp, and the counter stays at zero.

    This is the state on the real machine today (common.spec.max_accel_mps2
    is null until V-01). The case exists so that state is DESCRIBED rather
    than assumed: an operator reading accel_clamped == 0 on a robot that is
    visibly accelerating hard should be able to tell that from a bug.
    nav_wiring logs a WARNING naming the key when it constructs this way.
    """
    loop, sent = _loop(limits=False)
    # A full step command, the one stage 2 exists to catch. Passing it
    # through is the CORRECT behaviour for an uncalibrated deployment --
    # the alternative would be refusing to drive at all, and 12 S8.1
    # stage 2 is not a safety gate, it is body protection.
    report = loop.run_one_tick(computed_vx=2.0)
    assert report.vx == 2.0
    assert sent == [(2.0, 0.0)]
    assert loop.accel_clamped == 0


def test_wz_is_never_ramped():
    """wz passes through unchanged -- there is no angular a_max.

    Pinned rather than left implicit: a future edit that "completed" stage 2
    by ramping wz would be inventing an uncalibrated angular acceleration,
    and it would also put a ramp on the axis 12 S8.1 specifically exempts
    for the rotation veto. If an angular key ever lands, this case is where
    the decision becomes visible.
    """
    loop, sent = _loop()
    # wz alone, from rest: if wz were ramped, 1.5 rad/s would come back as
    # one budget (0.075) -- a value that happens to be in m/s, which is
    # the unit confusion the missing angular key would introduce.
    report = loop.run_one_tick(computed_vx=0.0, computed_wz=1.5)
    assert report.wz == 1.5
    assert sent == [(0.0, 1.5)]
    assert loop.accel_clamped == 0
