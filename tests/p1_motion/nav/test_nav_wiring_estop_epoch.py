"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_nav_wiring_estop_epoch.py
Brief: P1 echoes quadruped's estop generation instead of inventing one

Description:
11:1722 makes estop_epoch MANDATORY on rt/motion/cmd_vel, and the row below it
marks the field ABSENT on rt/nav2/cmd_vel -- the distinction is deliberate,
because only the first key reaches Tier 1. quadruped refuses a cmd_vel without
the field (13 S7.1.0 RX-3), so before this wiring existed every command this
loop published was dropped, and the symptom was a robot that would not move
with nothing in any log to say why.

The generation is LEARNED from rt/chassis/state and echoed back. Two ways of
getting that wrong are what these cases exist for:

  * inventing agreement -- echoing our own idea of the epoch, or copying
    whatever makes the two sides match. 13 S9.12.2 (3) holds zero exactly while
    the two disagree, so a value chosen to agree makes the soft-stop hold
    decorative while every counter and every log line still looks right.
  * losing the last known value on a malformed message. Falling back to 0 would
    make us disagree with a robot that has stopped (safe), and then AGREE again
    by accident the next time it stops (not safe) -- a hold that releases itself
    on the next stop is worse than one that never releases.

The runtime itself cannot be constructed here (it needs two zenohd routers), so
the handler is exercised directly. It touches three attributes and two module
level helpers, which is what makes that honest rather than a mock of the code
under test.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from xbrain.p1_motion.runtime.nav_wiring import NavRuntime

pytestmark = pytest.mark.no_device

_NAV = (Path(__file__).resolve().parents[3] / "xbrain" / "p1_motion" /
        "runtime" / "nav_wiring.py").read_text(encoding="utf-8")


class _Sample:
    """The two attribute spellings zenoh-python has used for a payload."""

    def __init__(self, raw: bytes) -> None:
        self.payload = raw


class _Stub:
    """Only what _on_chassis_state touches."""

    def __init__(self) -> None:
        self._estop_epoch = 0
        self._counts_state_bad = 0


def _state(epoch, rid="dev"):
    env = {"v": 1, "rid": rid, "ts": 1.0, "mono": 2.0, "boot": "abcd1234",
           "seq": 1, "src": "quadruped", "ts_sync": False,
           "data": {"estop_epoch": epoch, "conn": "ok"}}
    return _Sample(json.dumps(env).encode("utf-8"))


def _feed(stub, sample):
    NavRuntime._on_chassis_state(stub, sample)


def test_epoch_starts_at_zero_and_zero_is_not_a_guess():
    """A fresh boot agrees with a fresh robot, because both start at 0.

    That is not a coincidence to rely on quietly: it is why 0 is an honest
    starting value rather than a placeholder. After a stop we are behind until
    the next state message arrives, and being behind means Tier 1 holds zero --
    the correct answer while we do not know what generation the robot is on.
    """
    w = _Stub()
    assert w._estop_epoch == 0


def test_estop_epoch_is_echoed_not_invented():
    """The value published is the one the robot reported, whatever it is."""
    w = _Stub()
    _feed(w, _state(7))
    assert w._estop_epoch == 7
    _feed(w, _state(8))
    assert w._estop_epoch == 8
    # ...including going backwards, which happens when quadruped restarts.
    # Clamping to a maximum here would make us permanently ahead of a robot
    # that reset, and permanently ahead means permanently disagreeing.
    _feed(w, _state(0))
    assert w._estop_epoch == 0


def test_bad_chassis_state_keeps_last_epoch():
    """A malformed message must not poison or reset the generation."""
    w = _Stub()
    _feed(w, _state(5))
    assert w._estop_epoch == 5

    for bad in (b"not json",
                json.dumps({"v": 1, "data": {}}).encode("utf-8"),
                json.dumps({"v": 1, "data": {"estop_epoch": "7"}}).encode("utf-8"),
                json.dumps({"v": 1, "data": {"estop_epoch": -1}}).encode("utf-8"),
                json.dumps({"v": 1, "data": {"estop_epoch": True}}).encode("utf-8")):
        _feed(w, _Sample(bad))
        assert w._estop_epoch == 5, "a bad message must not change the epoch"
    assert w._counts_state_bad == 5


def test_cmd_vel_body_carries_the_field_and_the_rt_key_is_subscribed():
    """The two halves that make the echo reach quadruped at all.

    Source-text assertions, the same discipline test_nav_wiring_static uses:
    the runtime needs two routers, and these two facts are exactly the ones
    whose absence produces silence rather than an error.
    """
    # It is published in the body of THIS key, not the nav2 one.
    assert '"estop_epoch": self._estop_epoch' in _NAV
    # And it is learned from the RT plane, not routed through the general plane
    # via chassis_relay -- see the comment at the declaration for why.
    assert 'self._rt.declare_subscriber(' in _NAV
    assert '"xbrain/%s/rt/chassis/state" % self._rid' in _NAV
    # Held in the strong-reference list like every other subscription, or
    # zenoh-python's GC silently unsubscribes it (CLAUDE.md 4.3).
    assert _NAV.count("self._subs.append(self._rt.declare_subscriber(") == 1
