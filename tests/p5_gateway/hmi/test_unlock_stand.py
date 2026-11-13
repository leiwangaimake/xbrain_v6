"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_unlock_stand.py
Brief: 99 U181 -- an HMI unlock read back clear is followed by one stand; a failed one is not

Description:
The user's ruling of 2026-11-13: the HMI unlock button turning green must also
make the robot stand, and an unlock that fails must not. "Green" is the
state/robot read-back (hes_lock and timeout_lock both clear), never the ack.
Each case names the mutant it kills:
  * the read-back decides: locked / unknown keeps waiting, clear fires once;
  * the window: a clear after it never stands;
  * a refused enable disarms (an unrelated or accepted ack does not);
  * an already standing robot (stand / rl_control) gets no stand;
  * the window is required config, no default;
  * the P5 wiring arms on the published enable, feeds the read-back and the
    ack, and the shipped config carries the key.
"""
from __future__ import annotations  # postponed annotations

import inspect  # static wiring checks: the closures need both planes to run

import pytest  # parametrize, raises
import yaml  # the shipped config

from xbrain.p5_gateway.hmi.unlock_stand import StandAfterUnlock  # under test
from xbrain.p5_gateway.runtime import main_wiring  # the wiring that calls it

pytestmark = pytest.mark.no_device  # pure logic

LOCKED = {"hes_lock": False, "timeout_lock": True, "motion_state": "prone"}   # timeout lock
HES = {"hes_lock": True, "timeout_lock": False, "motion_state": "prone"}      # hardware stop held
CLEAR = {"hes_lock": False, "timeout_lock": False, "motion_state": "prone"}   # unlocked, lying down


def _armed(t0=100.0):
    """A 5 s window armed for enable h-u1 at t0."""
    s = StandAfterUnlock(5000)  # the shipped value
    s.arm("h-u1", t0)  # as _send_uplink does after the publish
    return s  # armed, nothing decided yet


def test_the_read_back_decides_and_fires_once():
    """MUTATION: fire on arm / on the ack -> a locked read-back returns h-u1
    -> red; keep it armed after firing -> a second clear frame stands again
    -> red."""
    s = _armed()  # an unlock just went out
    assert s.on_robot(LOCKED, 100.5) is None  # still locked: wait
    assert s.on_robot(HES, 101.0) is None     # hardware stop held (E-2): wait
    assert s.on_robot(None, 101.5) is None    # no state/robot: unknown, wait
    assert s.on_robot({"hes_lock": False}, 101.6) is None  # a lock field missing: unknown
    assert s.on_robot(CLEAR, 102.0) == "h-u1"  # green: stand
    assert s.on_robot(CLEAR, 102.1) is None   # once only
    assert s.armed is None  # and nothing left armed


def test_a_clear_after_the_window_never_stands():
    """MUTATION: ignore the deadline -> a clear 6 s later (another channel's
    unlock, say) makes this HMI click stand -> red."""
    s = _armed()  # deadline 105.0
    assert s.on_robot(LOCKED, 104.9) is None  # inside, locked
    assert s.on_robot(CLEAR, 105.2) is None   # past 5 s: no stand
    assert s.armed is None                    # and disarmed
    assert s.on_robot(CLEAR, 105.3) is None   # a later clear changes nothing


@pytest.mark.parametrize("cmd_id,result,still_armed", [
    ("h-u1", "rejected", False),   # refused: will not clear
    ("h-u1", "error", False),      # failed downstream
    ("h-u1", "accepted", True),    # accepted is not unlocked (BOOT-L3): keep waiting
    ("h-u1", "duplicate", True),   # a repeat of the same enable: keep waiting
    ("h-other", "rejected", True), # someone else's answer
])
def test_acks(cmd_id, result, still_armed):
    """MUTATION: disarm on any ack -> accepted never stands -> red; ignore
    the cmd_id -> another command's refusal cancels this stand -> red; ignore
    refusals -> a refused unlock followed by a commx unlock stands -> red."""
    s = _armed()  # h-u1 waiting
    s.on_ack(cmd_id, result)  # the answer under test
    assert (s.on_robot(CLEAR, 101.0) == "h-u1") is still_armed  # a clear frame tells which


@pytest.mark.parametrize("state,want", [
    ("rl_control", None),   # where a stand settles (13 S6.2 v1.3)
    ("stand", None),        # standing, before the firmware switches to RL
    ("prone", "h-u1"),      # lying down: the case the user hit
    ("damped_prone", "h-u1"),  # lying down, damped
    (None, "h-u1"),         # not reported: send, quadruped decides
])
def test_an_already_standing_robot_gets_no_stand(state, want):
    """MUTATION: drop the standing check -> rl_control gets a stand -> red;
    check the wrong field -> prone gets none -> red."""
    s = _armed()  # h-u1 waiting
    robot = dict(CLEAR, motion_state=state)  # unlocked, in the posture under test
    assert s.on_robot(robot, 101.0) == want  # stand only when not already standing
    assert s.armed is None  # judged once either way


def test_a_newer_unlock_replaces_the_older():
    """MUTATION: keep the first cmd_id -> the stand carries a stale id -> red."""
    s = _armed()  # h-u1 at 100.0, deadline 105.0
    s.arm("h-u2", 103.0)  # a second click, deadline 108.0
    assert s.on_robot(CLEAR, 107.5) == "h-u2"  # window counted from the newer arm


@pytest.mark.parametrize("bad", [None, 0, -1, True, "5000"])
def test_the_window_is_required(bad):
    """CLAUDE.md 3.1. MUTATION: default the window -> no raise -> red."""
    with pytest.raises(ValueError):  # refused, never defaulted
        StandAfterUnlock(bad)  # missing, zero, negative, a bool, a string


# -- wiring (static: the closures need both planes to run) ------------------------
_SRC = inspect.getsource(main_wiring)  # the P5 wiring module source


def test_the_wiring_arms_feeds_and_publishes():
    """MUTATION: drop any of the four calls -> red."""
    assert 'sau.arm(payload["cmd_id"], time.monotonic())' in _SRC  # on the published enable
    assert "sau.on_robot(body, time.monotonic())" in _SRC          # the state/robot read-back
    assert 'sau.on_ack(cmd_id, body.get("result"))' in _SRC        # the W9 ack
    assert '"action": "stand", "origin": "hmi"' in _SRC            # 11 S9.3.4 stand, origin hmi
    assert 'StandAfterUnlock(\n            hmi_cfg.get("unlock_stand_window_ms"))' in _SRC  # config-built


def test_the_shipped_config_carries_the_window():
    """MUTATION: drop the key -> P5 runs without the follow-up -> red."""
    from pathlib import Path  # repo root from this file
    root = Path(__file__).resolve().parents[3]  # tests/p5_gateway/hmi -> repo
    cfg = yaml.safe_load((root / "configs" / "p5_gateway.yaml").read_text(encoding="utf-8"))  # the source
    StandAfterUnlock(cfg["hmi"]["unlock_stand_window_ms"])  # builds: positive number
