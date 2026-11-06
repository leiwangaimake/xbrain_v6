"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_teleop_drive.py
Brief: 99 U134 -- teleop axes drive the robot at the voice-set speeds, through the shared gate chain

Description:
Until 2026-11-05 a teleop input only suspended RNS: nothing turned its axes
into a velocity, so the robot stood still under every key. The user ruled (99
U134) that the cloud keyboard drives at the voice-set move / turn speed. Each
case names the mutant it kills:
  * velocity.py: axes x move_speed (vx, vy) / x turn_speed (wz), clamped to
    [-1, 1], vy only on a holonomic base, None when no speed is known;
    speeds read from state/mode move_speed.mps / turn_speed.radps;
  * state.py: keyboard_cloud is a closed-set device (network priority, 400 ms)
    and drive_axes returns the winner's axes;
  * nav_tick.py: the teleop candidate wins the slot, goes through the SAME
    speed gate (perception dead / estop -> zero), and the tick reports ACTIVE;
  * nav_wiring.py: _on_mode keeps the speeds, _teleop_drive feeds the tick,
    and _one_tick hands both fields to NavInputs (a static check, as the
    existing nav_wiring_static tests do -- the runtime needs live sessions).
"""
from __future__ import annotations  # postponed annotations

import json  # state/mode frames
import threading  # the tracker lock
from pathlib import Path  # static read of nav_wiring
from types import SimpleNamespace  # a minimal self for the unbound methods

import pytest  # parametrize, marker

from tests.p1_motion.nav.test_nav_tick import OK, _goto, _inp, _stack  # the real tick stack
from xbrain.p1_motion.nav.nav_tick import ctrl_state_for  # state word
from xbrain.p1_motion.ctrl_loop import CtrlState  # ACTIVE / READY
from xbrain.p1_motion.rns.inputs import PerceptionSnapshot  # perception never received
from xbrain.p1_motion.runtime.nav_wiring import NavRuntime  # the two methods under test
from xbrain.p1_motion.sources.arbiter_p1 import BehaviorSource  # source names
from xbrain.p1_motion.teleop.state import TeleopTracker  # arbitration
from xbrain.p1_motion.teleop.velocity import TeleopSpeeds, source_for, speeds_from_mode, teleop_velocity  # under test

pytestmark = pytest.mark.no_device  # pure logic

_SPEEDS = TeleopSpeeds(move_mps=0.5, turn_radps=0.6)  # the voice defaults (99 U106 / U104)


# -- velocity -----------------------------------------------------------------
@pytest.mark.parametrize("axes,holo,want", [
    ({"vx": 1.0}, True, (0.5, 0.0, 0.0)), ({"vx": -1.0}, True, (-0.5, 0.0, 0.0)),  # w / s
    ({"wz": 1.0}, True, (0.0, 0.0, 0.6)), ({"wz": -1.0}, True, (0.0, 0.0, -0.6)),  # a / d
    ({"vy": 1.0}, True, (0.0, 0.5, 0.0)), ({"vy": -1.0}, True, (0.0, -0.5, 0.0)),  # y / u
    ({"vy": 1.0}, False, (0.0, 0.0, 0.0)),  # no strafe on a non-holonomic base
    ({"vx": 3.0}, True, (0.5, 0.0, 0.0)),  # clamped to 1.0
    ({"vx": "x", "wz": True}, True, (0.0, 0.0, 0.0)),  # garbage drives nothing
])
def test_axes_scale_by_the_voice_set_speeds(axes, holo, want):  # the rule
    """MUTATION: scale wz by move_speed -> a / d give 0.5 -> red; drop the
    clamp -> 1.5 m/s -> red."""
    assert teleop_velocity(axes, _SPEEDS, holonomic=holo) == pytest.approx(want)  # m/s, rad/s


def test_no_speed_known_drives_nothing():  # never guess
    """MUTATION: default to some speed -> a velocity -> red."""
    assert teleop_velocity({"vx": 1.0}, None, holonomic=True) is None  # refused


def test_speeds_come_from_state_mode():  # 11 S4.3 fields
    """MUTATION: read move_speed.min_mps -> red."""
    body = {"move_speed": {"mps": 0.7, "min_mps": 0.5, "max_mps": 1.0, "origin": "explicit"},
            "turn_speed": {"radps": 0.8, "min_radps": 0.4, "max_radps": 0.8, "origin": "default"}}
    assert speeds_from_mode(body) == TeleopSpeeds(0.7, 0.8)  # current values
    assert speeds_from_mode({"move_speed": {"mps": 0.7}}) is None  # turn speed missing
    assert speeds_from_mode({"move_speed": {"mps": 0}, "turn_speed": {"radps": 0.6}}) is None  # 0 is not a speed


# -- the tracker ----------------------------------------------------------------
def test_keyboard_cloud_is_a_network_source():  # closed set, 400 ms
    """MUTATION: leave keyboard_cloud out of the set -> observe raises -> red;
    give it 200 ms -> stale at 300 ms -> red."""
    t = TeleopTracker()  # fresh
    t.observe("keyboard_cloud", now_mono_ms=1000, deadman=True, axes={"vx": 1.0})  # a key held
    assert t.drive_axes(1300) == ("keyboard_cloud", {"vx": 1.0})  # still fresh at 300 ms
    assert t.drive_axes(1500) is None  # stale after 400 ms
    t.observe("keyboard_cloud", now_mono_ms=2000, deadman=True, axes={"vx": 1.0})  # again
    t.observe("gamepad", now_mono_ms=2000, deadman=True, axes={"wz": 1.0})  # a local gamepad
    assert t.drive_axes(2400)[0] == "keyboard_cloud"  # hysteresis keeps the first driver (T-2)
    assert source_for("keyboard_cloud") is BehaviorSource.TELEOP_KEYBOARD  # local family


# -- the tick ------------------------------------------------------------------
def test_the_teleop_candidate_drives_through_the_gate():  # NavTick step 4
    """MUTATION: no teleop branch in NavTick -> source hold, vx 0 -> red."""
    _src, tick = _stack()  # real stack
    out = tick.run(_teleop_inp(5000, (0.5, 0.0, 0.0)))  # w held
    assert out.source == "teleop_keyboard" and out.vx == pytest.approx(0.5)  # driven
    assert ctrl_state_for(_teleop_inp(5000, (0.5, 0.0, 0.0)), out, health_ever_ok=True,
                          inputs_ever_ready=True) is CtrlState.ACTIVE  # not zeroed by the CtrlLoop


def test_teleop_shares_the_safety_gate():  # same chain as autonomy
    """MUTATION: bypass the gate for teleop -> vx 0.5 with no perception -> red."""
    _src, tick = _stack()  # real stack
    dead = tick.run(_teleop_inp(5000, (0.5, 0.0, 0.0), perception=PerceptionSnapshot()))  # no camera
    assert dead.raw_vx == pytest.approx(0.5) and dead.vx == 0.0  # asked, refused
    stop = tick.run(_teleop_inp(5100, (0.5, 0.0, 0.0), estop=True))  # soft estop
    assert stop.vx == 0.0  # estop wins


def test_teleop_outranks_a_running_mission():  # manual over autonomy
    """The same input suspends RNS (TR-RNS-1) and drives instead."""
    src, tick = _stack()  # real stack
    _goto(src, 5000)  # a mission
    out = tick.run(_teleop_inp(5000, (0.0, 0.5, 0.0)))  # y held
    assert out.source == "teleop_keyboard" and out.vy == pytest.approx(0.5) and out.suspended  # strafing


def test_a_holder_kept_by_dwell_does_not_drive_another_sources_command():  # attribution
    """The keyboard held the slot a tick ago and is still in the arbiter's
    dwell; this tick the joystick asks. The holder (keyboard) must not drive
    the joystick's command under its own name. MUTATION: drop the
    `teleop_source is holder` check -> vx 0.3 attributed to the keyboard -> red."""
    from dataclasses import replace  # frozen dataclass
    _src, tick = _stack()  # real stack
    tick.run(_teleop_inp(5000, (0.5, 0.0, 0.0)))  # keyboard drives
    out = tick.run(replace(_teleop_inp(5050, (0.3, 0.0, 0.0)),
                           teleop_source=BehaviorSource.TELEOP_JOYSTICK))  # joystick asks
    assert out.source == "teleop_keyboard" and out.raw_vx == 0.0  # dwell holder drives zero


def _teleop_inp(now, cmd, **kw):  # NavInputs with a teleop candidate
    from dataclasses import replace  # frozen dataclass
    return replace(_inp(now, teleop=True, **kw), teleop_cmd=cmd,
                   teleop_source=BehaviorSource.TELEOP_KEYBOARD)


# -- the wiring -----------------------------------------------------------------
def test_on_mode_and_teleop_drive():  # nav_wiring methods on a minimal self
    """MUTATION: _teleop_drive ignores the speeds -> no command / wrong scale -> red."""
    me = SimpleNamespace(_teleop_lock=threading.Lock(), _teleop=TeleopTracker(),
                         _teleop_speeds=None, _cfg=SimpleNamespace(holonomic=True))  # what they read
    me._teleop.observe("keyboard_cloud", now_mono_ms=1000, deadman=True, axes={"wz": -1.0})  # d held
    assert NavRuntime._teleop_drive(me, 1100) == (None, None)  # no state/mode yet: nothing
    frame = json.dumps({"schema": "state_mode_v1", "move_speed": {"mps": 0.5},
                        "turn_speed": {"radps": 0.6}})  # P2 publishes state/mode bare
    NavRuntime._on_mode(me, SimpleNamespace(payload=frame.encode("utf-8")))  # P2's frame
    cmd, src = NavRuntime._teleop_drive(me, 1100)  # now known
    assert cmd == pytest.approx((0.0, 0.0, -0.6)) and src is BehaviorSource.TELEOP_KEYBOARD  # right turn


def test_one_tick_hands_teleop_to_navinputs():  # static wiring check
    """MUTATION: compute the drive but not pass it -> red."""
    src = (Path(__file__).resolve().parents[2] / "xbrain" / "p1_motion" / "runtime"
           / "nav_wiring.py").read_text(encoding="utf-8")  # the module source
    assert "teleop_cmd, teleop_source = self._teleop_drive(now_ms)" in src  # computed per tick
    assert "teleop_cmd=teleop_cmd, teleop_source=teleop_source," in src  # handed to the tick
    assert "teleop_unconditional=self._teleop_unconditional)" in src  # 99 U150 flag handed too
    assert "declare_subscriber(STATE_MODE_TOPIC" in src  # speeds subscribed


# -- 99 U150: the cloud keyboard drives unconditionally ----------------------------
def _cloud_inp(now, cmd, **kw):  # the cloud keyboard driving
    from dataclasses import replace  # frozen dataclass
    return replace(_teleop_inp(now, cmd, **kw), teleop_unconditional=True)


NO_HEALTH = __import__("tests.p1_motion.nav.test_nav_tick", fromlist=["NEVER"]).NEVER  # never granted


@pytest.mark.parametrize("kw", [
    {"heading_valid": False},  # the dual-antenna dropout of the 2026-11-06 bench
    {"i_fix": None, "pose": None},  # no position at all
    {"perception": PerceptionSnapshot()},  # no camera
    {"health": NO_HEALTH},  # health never granted
])
def test_the_cloud_keyboard_turns_whatever_the_inputs_say(kw):  # user ruling
    """An in-place turn (pure wz, a rotation-permit case) at the voice-set
    0.6 rad/s comes out at 0.6 whatever the positioning / heading /
    perception / health say, and the state word lets it through. MUTATION:
    keep the shared gate for keyboard_cloud -> wz 0 on a heading dropout ->
    red; keep the rotation permit -> 0.5 -> red; keep ctrl_state's input
    checks -> SAFE_STOP / WAIT_* -> red."""
    _src, tick = _stack()  # real stack
    inp = _cloud_inp(5000, (0.0, 0.0, 0.6), **kw)  # a held, in-place
    out = tick.run(inp)
    assert out.source == "teleop_keyboard" and out.wz == pytest.approx(0.6)  # unchanged
    assert ctrl_state_for(inp, out, health_ever_ok=True, inputs_ever_ready=True) is CtrlState.ACTIVE


def test_the_cloud_keyboard_keeps_the_estop_and_the_chassis_limits():  # what stays
    """MUTATION: drop the estop check -> 0.6 under an estop -> red; drop the
    chassis clamp -> wz 2.0 / vx 3.0 reach the chassis -> red."""
    _src, tick = _stack()  # real stack (spec max vx 2.0, max wz 1.2)
    stop = tick.run(_cloud_inp(5000, (0.0, 0.0, 0.6), estop=True))  # soft estop
    assert (stop.vx, stop.vy, stop.wz, stop.limiter) == (0.0, 0.0, 0.0, "estop")  # estop wins
    big = tick.run(_cloud_inp(5100, (3.0, 0.0, 2.0)))  # over the chassis limits
    assert big.vx == pytest.approx(2.0) and big.wz == pytest.approx(1.2)  # clamped to the spec


def test_only_the_cloud_keyboard_is_unconditional():  # the user's scope
    """MUTATION: mark keyboard_hmi unconditional too -> red; never set the
    flag -> red."""
    me = SimpleNamespace(_teleop_lock=threading.Lock(), _teleop=TeleopTracker(),
                         _teleop_speeds=TeleopSpeeds(move_mps=0.5, turn_radps=0.6),
                         _cfg=SimpleNamespace(holonomic=True), _teleop_unconditional=False)
    me._teleop.observe("keyboard_cloud", now_mono_ms=1000, deadman=True, axes={"wz": 1.0})
    NavRuntime._teleop_drive(me, 1100)
    assert me._teleop_unconditional is True  # the recording-time keyboard
    hmi = SimpleNamespace(_teleop_lock=threading.Lock(), _teleop=TeleopTracker(),
                          _teleop_speeds=TeleopSpeeds(move_mps=0.5, turn_radps=0.6),
                          _cfg=SimpleNamespace(holonomic=True), _teleop_unconditional=True)
    hmi._teleop.observe("keyboard_hmi", now_mono_ms=1000, deadman=True, axes={"wz": 1.0})
    NavRuntime._teleop_drive(hmi, 1100)
    assert hmi._teleop_unconditional is False  # HMI keeps the shared gate

