"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_mode_hold.py
Brief: P1-3 mode hold (99 U99) -- B / D suspends the autonomous source, aborts / refuses relmoves, keeps manual

Description:
cmd/motion/behavior had a producer (p2 command_for_mode) and a contract row
(11 S3.5 P1-3) but no subscriber in P1 until 2026-10-28, so a cloud broadcast
or an alarm never stopped the robot. The user ruled the semantics (99 U99):
suppress AUTONOMOUS motion, keep MANUAL intervention. These cases pin the P1
side through the real NavTick / MissionHost / NavRuntime._on_behavior:
  * hold suspends the RNS source exactly like estop / teleop, and releasing
    it lets the kept mission drive again;
  * a running relmove is ABORTED (preempted, item mode_hold), never paused --
    "go forward 1 m" must not finish by itself after the broadcast;
  * reject_relmove gives P4 a refusal it can speak (E_BUSY, reason mode_hold);
  * _on_behavior accepts only {hold, resume_task} from p2_core and refuses
    anything else without touching the flag;
  * the basic-motion test executor drops a relmove still pending.
Helpers come from the neighbouring NavTick / MissionHost tests so the stacks
are the production ones (same rns.yaml, same arbiter dwell).
"""

from __future__ import annotations  # postponed annotations

import json  # frames as JSON bytes
import math  # kinematics in _step_held
from types import SimpleNamespace  # minimal stand-ins

import pytest  # marks

from tests.p1_motion.nav.test_mission_host import CH_RELMOVE, OK, Sim, _ch
from tests.p1_motion.nav.test_nav_tick import _drive_until_moving, _goto, _inp, _stack
from xbrain.common.errors import E_BUSY  # the refusal code
from xbrain.p1_motion.nav.mission_host import MissionHost  # relmove shell
from xbrain.p1_motion.nav.nav_tick import NavInputs  # tick snapshot
from xbrain.p1_motion.runtime import basic_motion_test as bmt  # test executor
from xbrain.p1_motion.runtime.nav_wiring import NavRuntime  # _on_behavior

pytestmark = pytest.mark.no_device  # pure in-process


def _held(now):  # input with the hold
    """A tick input with only the P2 mode hold set (no estop, no teleop)."""
    inp = _inp(now)  # the neighbouring helper, everything healthy
    return NavInputs(**{**inp.__dict__, "mode_hold": True})  # frozen copy


def test_hold_suspends_the_autonomous_source_and_release_resumes():
    """MUTATION: leave mode_hold out of _suspension_edge -> RNS keeps driving
    under the hold -> the zero-velocity assertion fails."""
    src, tick = _stack()  # production NavTick + RNS source
    _goto(src, 5000)  # an autonomous mission
    assert _drive_until_moving(tick, 5000).vx > 0.0  # it drives
    out = tick.run(_held(6000))  # B / D starts
    assert out.suspended and out.source == "hold"  # RNS suspended
    assert (out.vx, out.vy, out.wz) == (0.0, 0.0, 0.0)  # robot still
    out = _drive_until_moving(tick, 6100)  # hold lifted
    assert out.source == "rns_avoid" and out.vx > 0.0  # mission kept


def _step_held(sim, n=1):  # Sim.step plus mode_hold
    """Sim.step with mode_hold=True (Sim.step itself has no such flag)."""
    emits = []  # collected bodies
    for _ in range(n):  # n ticks
        sim.now += 50  # one 20 Hz tick
        inp = NavInputs(now_mono_ms=sim.now, pose_xy=(sim.x, sim.y),
                        yaw_rad=sim.yaw, heading_valid=True, i_fix=1.0,
                        i_heading=1.0, perception=sim.snap(), health=OK,
                        estop=False, teleop_active=False, ts_wall_s=1.5,
                        mode_hold=True)  # only the hold differs
        out = sim.tick.run(inp)  # the real tick
        emits += sim.host.after_tick(inp, out)  # the real shell rules
        c, s = math.cos(sim.yaw), math.sin(sim.yaw)  # integrate like Sim
        sim.x += (out.vx * c - out.vy * s) * 0.05  # odom x
        sim.y += (out.vx * s + out.vy * c) * 0.05  # odom y
    return emits  # all channels


def test_hold_aborts_a_running_relmove_as_preempted():
    """MUTATION: drop the mode_hold branch in after_tick -> the relmove stays
    in flight (and would resume after the broadcast) -> no abort -> fails."""
    sim = Sim()  # NavTick + MissionHost + kinematics
    sim.relmove(dx=5.0)  # "go forward 5 m"
    sim.step(5)  # under way
    ems = _ch(_step_held(sim), CH_RELMOVE)  # B / D starts
    assert ems and ems[0].body["state"] == "aborted"  # ended, not paused
    assert ems[0].body["abort_reason"] == "preempted"  # 11 S9.3 closed set
    assert ems[0].body["detail"]["item"] == "mode_hold"  # and why


def test_reject_relmove_is_a_speakable_refusal():
    """The body P4 turns into speech (11 S9.3.2A rejected row)."""
    em = MissionHost.reject_relmove({"cmd_id": "rm-9"}, E_BUSY,
                                    {"reason": "mode_hold"})[0]
    assert em.channel == CH_RELMOVE  # cmd/motion/relative_move/status
    assert (em.body["cmd_id"], em.body["state"], em.body["code"]) == (
        "rm-9", "rejected", E_BUSY)
    assert em.body["detail"] == {"reason": "mode_hold"}  # what P4 says


def _behavior(body):  # zenoh sample stand-in
    """A zenoh-like sample carrying `body`."""
    return SimpleNamespace(payload=json.dumps(body).encode("utf-8"))  # bytes payload


def test_on_behavior_sets_and_clears_the_hold_only_from_p2():
    """MUTATION: accept any src / any value -> the forged or misspelt frames
    below flip the flag -> this fails."""
    fake = SimpleNamespace(_mode_hold=False, _counts={})  # the fields it uses
    NavRuntime._on_behavior(fake, _behavior({"src": "p2_core", "behavior": "hold"}))
    assert fake._mode_hold is True  # B / D on
    NavRuntime._on_behavior(fake, _behavior({"src": "p3_task", "behavior": "resume_task"}))
    assert fake._mode_hold is True  # not from p2: ignored
    NavRuntime._on_behavior(fake, _behavior({"src": "p2_core", "behavior": "face_target_stop"}))
    assert fake._mode_hold is True  # out of the R-8 set: ignored
    assert fake._counts["behavior_bad"] == 2  # both counted
    NavRuntime._on_behavior(fake, _behavior({"src": "p2_core", "behavior": "resume_task"}))
    assert fake._mode_hold is False  # B / D over


def test_test_executor_drops_a_pending_relmove(monkeypatch):
    """A relmove waiting for navigation must not start during the hold.
    MUTATION: drop_pending leaves the slot -> still pending -> fails."""
    monkeypatch.setenv("XBRAIN_BASIC_MOTION_LIN", "0.2")  # start-script speeds
    monkeypatch.setenv("XBRAIN_BASIC_MOTION_LAT", "0.15")
    monkeypatch.setenv("XBRAIN_BASIC_MOTION_ANG", "0.3")
    ex = bmt.BasicMotionTest()  # reads the three speeds
    ex.queue({"dx_m": 1.0}, 0.0)  # waiting for navigation
    ex.drop_pending("mode_hold")  # the hold arrives
    assert ex.pending is False  # nothing will start later
