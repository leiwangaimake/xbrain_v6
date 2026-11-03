"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_basic_motion_test.py
Brief: the TEMPORARY basic-motion test executor, driven against a simulated chassis odometry

Description:
xbrain/p1_motion/runtime/basic_motion_test.py closes the loop of the six basic
motions on quadruped's odometry while every gate of the standard flow is off
(user ruling 2026-10-26). It had no test of its own: the three fixes made on
the real chassis that day -- wait for navigation, no end-of-turn taper, refuse
rather than truncate an over-limit command -- were checked only by the
operator watching the robot. These cases pin each of them, plus the guard
verdicts an operator reads in the log (done / stall / wrong_direction /
state_stale_or_odom_invalid / preempted).

How the chassis is simulated. _Sim keeps a planar pose and integrates the
executor's body-frame (vx, vy, wz) over one TICK_S per step -- a perfect
chassis that does exactly what it is told. Faults are injected by scaling or
reversing what it does (`gain`), which is all the guard can observe. The pose
goes back in the shape rt/chassis/state carries it: usage_mode plus an odom
block with valid / x / y / yaw_rad.

Each case names the mutant it exists to kill; the mutants were run by hand on
2026-10-26 (edit the module, run this file, restore), and every one of them
turned its case red:
  * clamp instead of refuse (amount = min(abs(val), cap))  -> refuse case red;
  * start in admit() regardless of usage_mode              -> wait case red;
  * never drop a pending relmove                           -> drop case red;
  * TAPER_MIN back to 0.3                                  -> no-taper case red;
  * delete the stall check                                 -> stall case red
    (the motion then ends as "timeout", a different verdict);
  * delete the wrong-direction check                       -> sign-error case
    red (a reversed chassis then drives on until off-axis / timeout);
  * drop the state-age condition of the stale stop         -> stale case red;
  * queue() does not abort the running motion              -> preempt case red.
The no_nav_check mutant also turns the drop case red (the pending relmove
starts before it can be dropped), which is the expected overlap: both cases
are about the same pending slot. Run with __pycache__ cleared before every
run (CLAUDE.md 7.2.1); the original was restored and compared byte for byte.

What these cases do NOT establish: that nav_wiring calls queue / admit / step
in the right order on the live loop (that wiring is three lines and was
exercised on the real chassis), or anything about the real chassis' dynamics.
"""

from __future__ import annotations

import math

import pytest

from xbrain.p1_motion.runtime import basic_motion_test as bmt

pytestmark = pytest.mark.no_device  # pure logic against a simulated odometry


class _Sim:
    """A perfect planar chassis: integrates body-frame velocities per tick.

    gain scales what the chassis really does relative to the command: 1.0 is
    obedient, 0.0 is blocked (or not stepping), -1.0 is a sign error.
    """

    def __init__(self, gain: float = 1.0, usage_mode: str = "navigation") -> None:
        self.x = self.y = self.yaw = 0.0  # odom-frame pose, starts at origin
        self.gain = gain  # how much of the command really happens
        self.usage_mode = usage_mode  # what rt/chassis/state reports

    def state(self) -> dict:
        """The rt/chassis/state body the executor reads."""
        return {"usage_mode": self.usage_mode,  # Tier 1's mode gate input
                "odom": {"valid": True, "x": self.x, "y": self.y, "yaw_rad": self.yaw}}

    def apply(self, v) -> None:
        """Move by one tick of body-frame (vx, vy, wz)."""
        vx, vy, wz = (self.gain * c for c in v)  # what the chassis really does
        c, s = math.cos(self.yaw), math.sin(self.yaw)  # body -> odom rotation
        self.x += (vx * c - vy * s) * bmt.TICK_S  # odom x
        self.y += (vx * s + vy * c) * bmt.TICK_S  # odom y
        self.yaw += wz * bmt.TICK_S  # counter-clockwise positive


#: The control deadzone the executor is built with (NavConfig.cmd_deadzone);
#: test values, below the speeds used here.
DEADZONE = {"x": 0.08, "y": 0.08, "yaw": 0.25}
#: The MO-2 speeds P2 puts in every relmove (99 U104 / U106).
SPEEDS = {"max_speed_mps": 0.2, "max_yaw_rate_radps": 0.3}


def _with_speeds(body):
    """The relmove as P2 sends it: the MO-2 speeds added where the case left them out."""
    return dict(SPEEDS, **body) if isinstance(body, dict) else body


class _P2Fed:
    """The executor, fed relmoves that carry P2's speeds. The cases below are
    about the motion, not the speed source; the speed cases use the bare
    executor (bare_ex)."""

    def __init__(self, inner):
        self._inner = inner  # the real executor

    def __getattr__(self, name):
        return getattr(self._inner, name)  # everything else unchanged

    def start(self, body, *a):
        return self._inner.start(_with_speeds(body), *a)  # with MO-2 speeds

    def queue(self, body, *a):
        return self._inner.queue(_with_speeds(body), *a)  # with MO-2 speeds


@pytest.fixture
def bare_ex():
    """An executor built from the config deadzone, no speeds injected."""
    return bmt.BasicMotionTest(DEADZONE)


@pytest.fixture
def ex(bare_ex):
    """An executor whose relmoves carry P2's speeds (0.2 m/s, 0.3 rad/s)."""
    return _P2Fed(bare_ex)


def _run(ex, sim, t0=0.0, max_ticks=2000):
    """Step the executor against `sim` until the motion ends; returns the list
    of commands it produced (one (vx, vy, wz) per tick that kept going)."""
    out = []  # commands of the ticks that kept going
    t = t0  # simulated monotonic time
    for _ in range(max_ticks):  # bounded: a hang is a failure, not a stall
        if not ex.active:  # the motion ended last tick
            break  # nothing more to drive
        v = ex.step(t, sim.state(), 0.0)  # fresh state every tick
        if ex.active:  # still going after this tick
            out.append(v)  # record it
        sim.apply(v)  # the chassis does it
        t += bmt.TICK_S  # one control period
    return out  # for the per-axis assertions


def test_forward_one_metre_arrives(ex):
    """The baseline: forward 1 m on an obedient chassis ends as done, near
    1 m, and only ever drives the x axis. Without this case every refusal
    case below is satisfied by an executor that never moves anything."""
    sim = _Sim()  # an obedient chassis at the origin
    assert ex.start({"dx_m": 1.0, "cmd_id": "c1"}, sim.state(), 0.0) is None
    cmds = _run(ex, sim)  # run to the end, keep the commands
    assert ex.last_result == "done"  # arrived, no guard fired
    assert abs(sim.x - 1.0) < 0.05 and abs(sim.y) < 1e-9  # straight ahead
    assert all(vy == 0.0 and wz == 0.0 for _vx, vy, wz in cmds)  # one axis only


def test_turn_left_is_counter_clockwise_and_single_axis(ex):
    """Turn left 90 deg: positive dyaw -> positive wz only, ends done near
    +pi/2. Pins the sign convention the operator relies on ("左转")."""
    sim = _Sim()  # an obedient chassis at the origin
    assert ex.start({"dyaw_rad": math.pi / 2}, sim.state(), 0.0) is None
    cmds = _run(ex, sim)  # run to the end, keep the commands
    assert ex.last_result == "done"  # arrived, no guard fired
    assert abs(sim.yaw - math.pi / 2) < math.radians(3.0)  # within 3 deg of 90
    # >= 0 per tick (the ramp's first tick commands exactly 0), > 0 overall.
    assert all(vx == 0.0 and vy == 0.0 and wz >= 0.0 for vx, vy, wz in cmds)
    assert max(wz for _vx, _vy, wz in cmds) > 0.0


def test_over_limit_is_refused_not_truncated(ex):
    """2026-10-26: "右转360度" turned 180 deg because the amount was clamped to
    MAX_ANGLE_RAD. It must be refused whole and nothing may start.
    MUTATION: amount = min(abs(val), cap) -> start() returns None and the
    executor becomes active -> this fails."""
    sim = _Sim()  # an obedient chassis at the origin
    why = ex.start({"dyaw_rad": -2 * math.pi}, sim.state(), 0.0)
    assert why is not None and why.startswith("exceeds_test_limit")
    assert ex.active is False  # nothing started
    why = ex.start({"dx_m": bmt.MAX_DIST_M + 0.5}, sim.state(), 0.0)
    assert why is not None and why.startswith("exceeds_test_limit")
    # Exactly at the limit is still allowed: the boundary is inclusive.
    assert ex.start({"dx_m": bmt.MAX_DIST_M}, sim.state(), 0.0) is None


def test_relmove_waits_for_navigation(ex):
    """2026-10-26: the first turn after stand-up arrived before the chassis
    was in navigation, Tier 1 output zero, and the guard declared a stall.
    A queued relmove must not start until usage_mode is navigation.
    MUTATION: start in admit() without the usage_mode check -> active while
    the chassis is still "normal" -> this fails."""
    sim = _Sim(usage_mode="normal")  # stood up, not yet in navigation
    ex.queue({"dyaw_rad": 0.5}, 0.0)  # sent during stand-up
    for i in range(20):  # one second of ticks in normal mode
        assert ex.admit(i * bmt.TICK_S, sim.state(), 0.0) is None  # waiting
        assert ex.active is False and ex.pending is True  # held, not started
    sim.usage_mode = "navigation"  # the auto-switch lands
    assert ex.admit(1.0, sim.state(), 0.0) == "started"  # starts on the first navigation tick
    assert ex.active is True and ex.pending is False  # moved from pending to running


def test_pending_relmove_is_dropped_after_the_wait_limit(ex):
    """A relmove that never sees navigation is dropped after PENDING_MAX_S
    rather than starting minutes later and surprising the operator.
    MUTATION: never drop -> still pending at PENDING_MAX_S + 1 -> this fails."""
    sim = _Sim(usage_mode="normal")  # stood up, not yet in navigation
    ex.queue({"dx_m": 0.5}, 0.0)  # sent, navigation never comes
    assert ex.admit(bmt.PENDING_MAX_S - 1.0, sim.state(), 0.0) is None  # still within the wait
    assert ex.admit(bmt.PENDING_MAX_S + 1.0, sim.state(), 0.0) == "dropped_not_navigation"  # given up
    assert ex.pending is False  # the slot is empty again
    sim.usage_mode = "navigation"  # too late: nothing is left to start
    assert ex.admit(bmt.PENDING_MAX_S + 2.0, sim.state(), 0.0) is None  # nothing pending
    assert ex.active is False  # nothing started


def test_turn_is_not_tapered_near_the_end(ex):
    """2026-10-26: below ~0.3 rad/s the chassis twists without stepping, so
    the end-of-turn taper turned every turn into a stall. After the ramp the
    commanded wz stays at the set speed until the guard says done.
    MUTATION: TAPER_MIN = 0.3 -> the last ticks command < 0.3 rad/s -> fails."""
    sim = _Sim()  # an obedient chassis at the origin
    assert ex.start({"dyaw_rad": 1.0}, sim.state(), 0.0) is None
    cmds = _run(ex, sim)  # run to the end, keep the commands
    assert ex.last_result == "done"  # arrived, no guard fired
    ramp_ticks = int(round(bmt.RAMP_S / bmt.TICK_S))  # the ramp-up is allowed
    after_ramp = [wz for _vx, _vy, wz in cmds[ramp_ticks:]]  # the steady part of the turn
    assert after_ramp and min(after_ramp) == pytest.approx(0.3)  # never below the set speed


def test_blocked_chassis_ends_as_stall(ex):
    """A chassis that does not move (gain 0) ends as "stall", the verdict the
    operator reads as "blocked or not stepping".
    MUTATION: delete the stall check -> it ends as "timeout" -> this fails."""
    sim = _Sim(gain=0.0)  # blocked / not stepping
    assert ex.start({"dx_m": 1.0}, sim.state(), 0.0) is None
    _run(ex, sim)  # run to the end
    assert ex.last_result == "stall"  # not timeout


def test_sign_error_ends_as_wrong_direction(ex):
    """A chassis moving the opposite way (gain -1) is stopped as
    "wrong_direction", never driven on."""
    sim = _Sim(gain=-1.0)  # a sign error
    assert ex.start({"dy_m": 1.0}, sim.state(), 0.0) is None
    _run(ex, sim)  # run to the end
    assert ex.last_result == "wrong_direction"  # stopped, not driven on


def test_stale_state_stops_at_once(ex):
    """No pose to close the loop on -> stop with zero, never dead-reckon."""
    sim = _Sim()  # an obedient chassis at the origin
    assert ex.start({"dx_m": 1.0}, sim.state(), 0.0) is None
    assert ex.step(0.0, sim.state(), bmt.STATE_STALE_S + 0.1) == (0.0, 0.0, 0.0)
    assert ex.active is False and ex.last_result == "state_stale_or_odom_invalid"


def test_newer_relmove_preempts_the_running_one(ex):
    """queue() while a motion runs aborts it at once ("preempted"); the new
    one waits in pending, the old one does not keep going meanwhile."""
    sim = _Sim()  # an obedient chassis at the origin
    assert ex.start({"dx_m": 1.0}, sim.state(), 0.0) is None
    ex.queue({"dx_m": -0.5}, 0.1)  # a newer command
    assert ex.active is False and ex.last_result == "preempted"
    assert ex.pending is True  # the new one waits


def test_speed_comes_from_the_relmove_not_the_environment(bare_ex):
    """99 U106: forward / strafe at max_speed_mps, turn at max_yaw_rate_radps.
    MUTATION: read max_yaw_rate_radps for a linear axis -> forward at 0.3 -> fails."""
    sim = _Sim()  # an obedient chassis at the origin
    body = {"dx_m": 1.0, "max_speed_mps": 0.25, "max_yaw_rate_radps": 0.6}  # both present
    assert bare_ex.start(body, sim.state(), 0.0) is None
    cmds = _run(bare_ex, sim)  # run to the end
    assert max(vx for vx, _vy, _wz in cmds) == pytest.approx(0.25)  # the move speed, not 0.6


@pytest.mark.parametrize("body, why", [
    ({"dx_m": 1.0}, "speed_missing(max_speed_mps)"),  # no move speed
    ({"dy_m": 1.0, "max_speed_mps": "fast"}, "speed_missing(max_speed_mps)"),  # not a number
    ({"dyaw_rad": 1.0, "max_speed_mps": 0.2}, "speed_missing(max_yaw_rate_radps)"),  # wrong field for a turn
])
def test_relmove_without_its_speed_is_refused(bare_ex, body, why):
    """This module never picks a speed of its own (the env fallback is gone).
    MUTATION: fall back to a constant when the field is missing -> fails."""
    assert bare_ex.start(body, _Sim().state(), 0.0) == why  # refused, nothing moves
    assert not bare_ex.active  # no motion started


@pytest.mark.parametrize("body", [
    {"dy_m": 1.0, "max_speed_mps": 0.08},  # AT the linear deadzone
    {"dyaw_rad": 1.0, "max_yaw_rate_radps": 0.2},  # below the yaw deadzone
])
def test_speed_in_the_deadzone_is_refused(bare_ex, body):
    """2026-11-03 "死区抑制": a speed at or below the control deadzone does
    not step the chassis -- no motion, not a stall later.
    MUTATION: compare with < instead of <= -> the 0.08 case starts -> fails."""
    why = bare_ex.start(body, _Sim().state(), 0.0)
    assert why is not None and why.startswith("speed_in_deadzone")  # refused
    assert not bare_ex.active  # nothing moves


@pytest.mark.parametrize("dz", [{"x": 0.08, "y": 0.08}, {"x": 0.08, "y": 0.0, "yaw": 0.3},
                                {"x": 0.08, "y": 0.08, "yaw": None}])
def test_bad_deadzone_config_refuses_construction(dz):
    """A missing / non-positive axis is a config error at start, not a guess."""
    with pytest.raises(ValueError):
        bmt.BasicMotionTest(dz)


# --- face-heading (A12 "面向东面", 2026-11-04) ------------------------------

@pytest.mark.parametrize("heading_deg, target_deg, turn_deg", [
    (90.0, 0.0, -90.0),      # facing north, "面向东面": right 90
    (100.0, 180.0, 80.0),    # facing just west of north, "面向西面": left 80
    (-170.0, 170.0, -20.0),  # across the +-180 seam: the short way, not +340
    (0.0, -90.0, -90.0),     # facing east, "面向南面": right 90
])
def test_face_heading_turns_the_short_way_from_the_rtk_heading(ex, heading_deg, target_deg, turn_deg):
    """The turn is target - RTK heading taken at start, wrapped; the odometry
    yaw (zero at boot, here 0) is NOT the reference.
    MUTATION: dyaw = target - odom yaw -> the first case turns 0 -> fails."""
    sim = _Sim()  # odometry yaw 0, unrelated to north
    body = {"target_yaw_rad": math.radians(target_deg)}  # what P2 sends for A12
    assert ex.start(body, sim.state(), 0.0, math.radians(heading_deg)) is None  # accepted
    cmds = _run(ex, sim)  # run the turn to its end
    assert ex.last_result == "done"  # no guard fired
    assert abs(math.degrees(sim.yaw) - turn_deg) < 3.0  # the turn it made
    assert all(vx == 0.0 and vy == 0.0 for vx, vy, _wz in cmds)  # turn only


@pytest.mark.parametrize("body, heading, why", [
    ({"target_yaw_rad": 0.0}, None, "no_absolute_heading"),  # COG / lost heading
    ({"target_yaw_rad": 0.0, "dy_m": 1.0}, 0.5, "face_heading_with_motion"),  # TY-1
    ({"target_yaw_rad": 0.0, "dx_m": -1.0}, 0.5, "face_heading_with_motion"),  # TY-6
    ({"target_yaw_rad": float("nan")}, 0.5, "bad_body"),  # not a number
    ({"target_yaw_rad": 0.5}, 0.5, "zero_move"),  # already facing it
])
def test_face_heading_refusals_move_nothing(ex, body, heading, why):
    """MUTATION: fall back to the odometry yaw when no L1 heading -> the first
    case starts a turn -> fails."""
    assert ex.start(body, _Sim().state(), 0.0, heading) == why  # the named refusal
    assert not ex.active  # and nothing runs


# --- heading walk (A17 "向南走一米", 2026-11-04, 99 U108, TY-6) ---------------

def test_heading_walk_turns_then_walks_along_the_target(ex):
    """Facing north (RTK), "向东走一米": a right turn of 90 deg, then 1 m
    forward -- which is east in the odometry frame the turn left behind.
    MUTATION: start the walk without the turn (or never start it) -> the
    end position is not 1 m along the turned heading -> this fails."""
    sim = _Sim()  # odometry yaw 0; RTK says the body faces north
    body = {"target_yaw_rad": 0.0, "dx_m": 1.0}  # what P2 sends for A17 east 1 m
    assert ex.start(body, sim.state(), 0.0, math.pi / 2) is None  # accepted
    cmds = _run(ex, sim)  # both phases run inside this one motion
    assert ex.last_result == "done"  # the walk, not just the turn, ended done
    assert abs(math.degrees(sim.yaw) + 90.0) < 3.0  # turned right 90
    # 1 m along the turned heading (odom -90 deg = -y)
    assert abs(sim.x) < 0.08 and abs(sim.y + 1.0) < 0.08  # not along the old heading
    assert any(wz < 0.0 for _vx, _vy, wz in cmds)  # a right turn happened
    assert any(vx > 0.0 for vx, _vy, _wz in cmds)  # and a forward walk
    assert all(not (vx != 0.0 and wz != 0.0) for vx, _vy, wz in cmds)  # one axis per tick


def test_heading_walk_already_facing_walks_at_once(ex):
    """No turn needed: the walk starts straight away."""
    sim = _Sim()  # odometry at the origin
    assert ex.start({"target_yaw_rad": 0.3, "dx_m": 0.5}, sim.state(), 0.0, 0.3) is None  # RTK == target
    _run(ex, sim)  # walk only
    assert ex.last_result == "done"  # arrived
    assert abs(sim.x - 0.5) < 0.05 and abs(sim.yaw) < 1e-9  # walked, never turned


@pytest.mark.parametrize("body, why", [
    ({"target_yaw_rad": 0.0, "dx_m": bmt.MAX_DIST_M + 0.5}, "exceeds_test_limit"),  # too far
    ({"target_yaw_rad": 0.0, "dx_m": 1.0, "max_speed_mps": 0.05}, "speed_in_deadzone"),  # won't step
])
def test_heading_walk_refused_before_any_turn(bare_ex, body, why):
    """A walk that would be refused never turns the robot first.
    MUTATION: check the walk only when it starts -> start() returns None and
    the robot turns -> this fails."""
    body = dict({"max_yaw_rate_radps": 0.3, "max_speed_mps": 0.2}, **body)  # P2's speeds, overridable
    got = bare_ex.start(body, _Sim().state(), 0.0, math.pi / 2)  # a 90 deg turn would come first
    assert got is not None and got.startswith(why)  # refused whole
    assert not bare_ex.active  # so the robot did not start turning


def test_a_turn_that_fails_does_not_start_the_walk(ex):
    """Only a turn that ends "done" starts the walk (blocked chassis -> stall)."""
    sim = _Sim(gain=0.0)  # a chassis that does not move
    assert ex.start({"target_yaw_rad": 0.0, "dx_m": 1.0}, sim.state(), 0.0, math.pi / 2) is None  # accepted
    _run(ex, sim)  # the turn stalls
    assert ex.last_result == "stall"  # the turn's verdict, reported as such
    assert not ex.active and sim.x == 0.0  # and no walk was started after it
