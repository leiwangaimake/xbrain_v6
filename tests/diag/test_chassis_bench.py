"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chassis_bench.py
Brief: The bench console's motion guard stops on every abort it promises, and only then

Description:
scripts/diag/chassis_bench.py drives the real M20S with no obstacle
avoidance; its MotionGuard is the only thing between a typed "fwd 0.5" and a
robot that keeps walking. Each abort the tool's header promises is driven
here by a small kinematic simulation (the robot follows the commanded speed
times a gain, optionally with drift or an offset), so each check has a case
that must end in exactly that reason:

  done            gain 1: reaches the target within tolerance, never above
                  the operator's speed, ramp starts from zero
  stall           gain 0 (blocked, the 2026-10-23 TV wall): stops, and not
                  before STALL_AFTER_S
  wrong_direction gain -1
  over_speed      gain 2.5 (10-23 measured 0.4-0.5 rad/s for 0.2)
  off_axis_drift  sideways creep during a forward walk
  timeout         gain 0.3: slow enough to miss the timeout, fast enough
                  not to count as a stall
  tier1_zeroing   stop_reason not none for TIER1_ZERO_S (a 0.5 s blip is not)
  operator_stop / estop / foreign_cmd_vel / state_stale / odom_invalid

plus the geometry under it: progress is measured along the START heading,
strafe and turn signs follow REP-103, and the continuous yaw survives the
+-pi seam (a 180 degree turn must not read as -179 and stop as "wrong
direction").

No zenoh, no robot: the guard and the geometry are pure functions.
Mutants this file kills are listed in the commit.
"""
from __future__ import annotations

import importlib.util  # load the script as a module
import json  # envelope bytes
import math  # angles
import pathlib  # script path
import sys  # module registry

import pytest  # parametrize, raises

#: The script under test, loaded by path (scripts/ is not a package).
_PATH = pathlib.Path(__file__).resolve().parents[2] / "scripts" / "diag" / "chassis_bench.py"
_spec = importlib.util.spec_from_file_location("chassis_bench", _PATH)  # by path
cb = importlib.util.module_from_spec(_spec)  # the module object
sys.modules["chassis_bench"] = cb  # @dataclass looks its module up here
_spec.loader.exec_module(cb)  # run its top level (no zenoh import there)

pytestmark = pytest.mark.no_device  # runs anywhere


def _flags(**kw):
    """The guard's keyword inputs, all quiet unless overridden."""
    base = dict(operator_stop=False, estop_changed=False, foreign=False,  # no stop
                state_stale=False, odom_valid=True, stop_reason="none")  # healthy
    base.update(kw)  # the one under test
    return base


def _sim(kind="fwd", amount=0.5, speed=0.2, gain=1.0, drift=0.0, until=60.0,
         flags_at=None, noise=None):
    """Run a guard against a robot moving at gain * commanded speed.

    drift: off-axis metres per second while moving. flags_at(t) returns
    extra flags for that tick. noise(i) is added to the REPORTED progress of
    tick i (odom jitter; the robot itself is not moved by it). Returns
    (guard, list of (t, scalar, ended), final progress).
    """
    g = cb.MotionGuard(kind, amount, speed)  # the unit under test
    t, prog, off, out = 100.0, 0.0, 0.0, []  # arbitrary monotonic origin
    scalar = 0.0  # what the stream last sent
    i = 0  # tick index for the noise
    while t < 100.0 + until:  # bounded
        prog += gain * scalar * cb.TICK_S  # the robot follows the last command
        off += drift * cb.TICK_S if scalar > 0 else 0.0  # sideways creep
        seen = prog + (noise(i) if noise else 0.0)  # what odom reports
        i += 1  # next tick
        st = g.step(t, seen, off, **_flags(**(flags_at(t - 100.0) if flags_at else {})))
        out.append((t - 100.0, st.scalar, st.ended))  # record the tick
        if st.ended is not None:  # guard finished
            return g, out, prog
        scalar = st.scalar  # next tick's command
        t += cb.TICK_S  # 20 Hz
    raise AssertionError("guard never ended")  # a guard must always end


# -- the happy path -----------------------------------------------------------

@pytest.mark.parametrize("kind,amount,speed", [
    ("fwd", 0.5, 0.2), ("back", 0.3, 0.15), ("left", 0.4, 0.15),  # translations
    ("right", 0.2, 0.1), ("turnl", math.radians(90), 0.2),  # turns
    ("turnr", math.radians(30), 0.3)])
def test_reaches_the_target_without_exceeding_the_speed(kind, amount, speed):
    """gain 1: done, within tolerance, scalar never above the operator's."""
    g, out, prog = _sim(kind, amount, speed)  # an obedient robot
    assert out[-1][2] == "done"  # finished normally
    assert abs(prog - amount) <= g.tol + speed * cb.TICK_S  # stopped at the target
    assert max(s for _t, s, _e in out) <= speed + 1e-12  # never faster than asked
    assert out[0][1] == 0.0 and out[1][1] < speed  # ramps up from zero


def test_once_ended_it_stays_ended_at_zero():
    """The verdict is a latch: no later tick restarts the motion."""
    g, out, _ = _sim()  # finishes "done"
    again = g.step(999.0, 0.0, 0.0, **_flags())  # far later, progress reset
    assert (again.scalar, again.ended) == (0.0, "done")  # still ended, zero


# -- the robot misbehaving ----------------------------------------------------

def test_a_blocked_robot_is_stopped_as_a_stall_but_not_too_early():
    """gain 0: commanded but not moving (pushing a wall) -> stall."""
    _g, out, _ = _sim(gain=0.0)  # nothing moves
    t_end, _s, why = out[-1]  # when and why
    assert why == "stall"  # named correctly
    assert cb.STALL_AFTER_S <= t_end < cb.STALL_AFTER_S + 0.5  # promptly after the grace


def test_moving_the_wrong_way_stops():
    """gain -1: a sign error anywhere shows up here."""
    assert _sim(gain=-1.0)[1][-1][2] == "wrong_direction"  # last tick names it


def test_running_away_stops_as_over_speed():
    """gain 2.5: faster than asked (10-23 measured 2x on a turn)."""
    _g, out, _ = _sim(kind="turnl", amount=math.radians(170), speed=0.2, gain=2.5)  # runaway
    assert out[-1][2] == "over_speed"  # named correctly


def test_sideways_creep_on_a_forward_walk_stops():
    """0.2 m/s of drift while walking 1.5 m: off_axis_drift."""
    _g, out, _ = _sim(amount=1.5, drift=0.2)  # creeps sideways
    assert out[-1][2] == "off_axis_drift"  # named correctly


def test_too_slow_but_moving_times_out_rather_than_stalls():
    """gain 0.3: above the stall fraction, below finishing in time."""
    g, out, _ = _sim(gain=0.3)  # slow
    assert out[-1][2] == "timeout"  # the timeout caught it
    assert out[-1][0] == pytest.approx(g.timeout_s, abs=0.1)  # at its bound


def test_isolated_odom_spikes_do_not_add_up_to_over_speed():
    """A 0.15 m odom spike every 4th tick: never 5 CONSECUTIVE over-speed ticks.

    Each spike alone reads as over-speed for one tick; the count must reset
    on the normal ticks between them, or jitter alone would stop every walk.
    """
    _g, out, _ = _sim(amount=1.0, noise=lambda i: 0.15 if i % 4 == 3 else 0.0)
    assert out[-1][2] == "done"  # jitter is not a runaway


# -- external stops -------------------------------------------------------------

@pytest.mark.parametrize("flag,value,reason", [
    ("operator_stop", True, "operator_stop"), ("estop_changed", True, "estop"),
    ("foreign", True, "foreign_cmd_vel"), ("state_stale", True, "state_stale"),
    ("odom_valid", False, "odom_invalid")])
def test_each_external_stop_ends_with_zero_at_once(flag, value, reason):
    """Raised at t = 1 s: that very tick commands zero and names the reason."""
    _g, out, _ = _sim(flags_at=lambda t: {flag: value} if t >= 1.0 else {})
    t_end, s, why = out[-1]  # the ending tick
    assert (why, s) == (reason, 0.0)  # reason and zero
    assert t_end == pytest.approx(1.0, abs=cb.TICK_S)  # immediately


def test_tier1_zeroing_stops_after_its_hold_and_a_blip_does_not():
    """stop_reason timeout for 1 s -> abort; a 0.5 s blip is ridden through."""
    _g, out, _ = _sim(flags_at=lambda t: {"stop_reason": "timeout"} if t >= 0.5 else {})
    assert out[-1][2] == "tier1_zeroing:timeout"  # persisted: stopped
    assert out[-1][0] == pytest.approx(0.5 + cb.TIER1_ZERO_S, abs=2 * cb.TICK_S)  # float ticks
    _g, out, _ = _sim(flags_at=lambda t: {"stop_reason": "mode_switching"}
                      if 0.5 <= t < 1.0 else {})  # a short blip
    assert out[-1][2] == "done"  # ridden through
    # Two short blips 1.5 s apart: each is under the hold, so neither may stop
    # the walk -- the clock must restart, not keep counting from the first.
    _g, out, _ = _sim(amount=1.0, flags_at=lambda t: {"stop_reason": "mode_switching"}
                      if (0.5 <= t < 0.9 or 2.0 <= t < 2.4) else {})
    assert out[-1][2] == "done"  # two blips, still ridden through


# -- geometry ---------------------------------------------------------------------

def test_progress_is_along_the_start_heading():
    """Start facing +y (90 deg): moving +y is forward, moving -x is left."""
    start = (1.0, 2.0, math.pi / 2)  # facing +y
    assert cb.progress_of("fwd", start, (1.0, 2.5, math.pi / 2)) == pytest.approx((0.5, 0.0))
    assert cb.progress_of("left", start, (0.7, 2.0, math.pi / 2)) == pytest.approx((0.3, 0.0))
    assert cb.progress_of("right", start, (0.7, 2.0, 0.0))[0] == pytest.approx(-0.3)
    assert cb.progress_of("back", start, (1.0, 2.5, 0.0)) == pytest.approx((-0.5, 0.0))
    p, off = cb.progress_of("turnr", start, (1.1, 2.0, math.pi / 2 - 0.4))  # turned right
    assert (p, off) == (pytest.approx(0.4), pytest.approx(0.1))  # walking is off-axis


@pytest.mark.parametrize("kind,vec", [
    ("fwd", (0.2, 0.0, 0.0)), ("back", (-0.2, 0.0, 0.0)),
    ("left", (0.0, 0.2, 0.0)), ("right", (0.0, -0.2, 0.0)),
    ("turnl", (0.0, 0.0, 0.2)), ("turnr", (0.0, 0.0, -0.2))])
def test_command_vectors_follow_rep103(kind, vec):
    """Left and counter-clockwise are positive; one axis per kind."""
    assert cb.command_vector(kind, 0.2) == vec  # exact vector


def test_a_half_turn_across_the_seam_completes():
    """Turn 180 deg from yaw 120 deg, odom reporting WRAPPED yaw."""
    yaw0 = math.radians(120)  # the seam is 60 deg into the turn
    g = cb.MotionGuard("turnl", math.pi, 0.5)  # the largest turn, fastest speed
    uw = cb.YawUnwrapper(yaw0)  # as run_motion uses it
    yaw, scalar, t, why = yaw0, 0.0, 0.0, None  # robot state
    while why is None and t < 30.0:  # bounded
        yaw = cb.wrap_pi(yaw + scalar * cb.TICK_S)  # odom wraps at +-pi
        prog, _off = cb.progress_of("turnl", (0, 0, yaw0), (0, 0, uw.feed(yaw)))
        st = g.step(t, prog, 0.0, **_flags())  # one tick
        why, scalar, t = st.ended, st.scalar, t + cb.TICK_S  # advance
    assert why == "done"  # not wrong_direction at the seam


def test_the_unwrapper_is_continuous():
    """170 -> -170 deg wrapped is +20 deg of turn, not -340."""
    uw = cb.YawUnwrapper(math.radians(170))  # just before the seam
    assert math.degrees(uw.feed(math.radians(-170))) == pytest.approx(190.0)  # across it


# -- inputs ------------------------------------------------------------------------

@pytest.mark.parametrize("value,ok", [
    (0.2, True), (0.5, True), (0.51, False), (0.0, False), (-0.1, False),
    (float("nan"), False)])
def test_speeds_are_checked_against_the_ceiling(value, ok):
    """Positive and at most the ceiling; NaN refused."""
    if ok:  # accepted as given
        assert cb.check_speed("lin", value, cb.MAX_LIN_MPS) == value  # unchanged
    else:  # refused
        with pytest.raises(ValueError):  # never clamped silently
            cb.check_speed("lin", value, cb.MAX_LIN_MPS)  # the check


def test_the_guard_refuses_a_bad_request():
    """Unknown kind, zero / negative / NaN amount or speed."""
    for args in (("jump", 1.0, 0.1), ("fwd", 0.0, 0.1), ("fwd", 1.0, -0.1),  # kind, zero, negative
                 ("fwd", float("nan"), 0.1)):  # and NaN
        with pytest.raises(ValueError):  # refused at construction
            cb.MotionGuard(*args)  # the request


def test_the_envelope_carries_what_quadruped_requires():
    """read_envelope refuses a frame without v, rid, src, seq, ts, ts_sync, data."""
    d = json.loads(cb.envelope("gj-001", 7, {"vx": 0.0}))  # one frame
    assert {"v", "rid", "src", "seq", "ts", "ts_sync", "data"} <= set(d)  # all present
    assert (d["v"], d["rid"], d["seq"], d["src"]) == (1, "gj-001", 7, cb.BENCH_SRC)  # values
    assert d["ts_sync"] is False  # a boolean, present


def test_pose_needs_an_explicitly_valid_odom():
    """valid must be literally true; a missing field is no pose."""
    assert cb.pose_of({"odom": {"x": 1, "y": 2, "yaw_rad": 0.5, "valid": True}}) == (1.0, 2.0, 0.5)
    assert cb.pose_of({"odom": {"x": 1, "y": 2, "yaw_rad": 0.5, "valid": False}}) is None
    assert cb.pose_of({"odom": {"x": 1, "y": 2, "valid": True}}) is None
    assert cb.pose_of({}) is None  # no odom at all
