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


@pytest.fixture
def ex(monkeypatch):
    """An executor with the start script's speeds (0.2 / 0.15 / 0.3)."""
    monkeypatch.setenv("XBRAIN_BASIC_MOTION_LIN", "0.2")  # m/s, as the start script
    monkeypatch.setenv("XBRAIN_BASIC_MOTION_LAT", "0.15")  # m/s
    monkeypatch.setenv("XBRAIN_BASIC_MOTION_ANG", "0.3")  # rad/s, the user-approved turn speed
    return bmt.BasicMotionTest()  # reads the three variables


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
