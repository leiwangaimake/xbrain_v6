"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_teleop_state.py
Brief: 11 S12A.9.7 teleop arbitration + state/teleop payload (batch 9)

Description:
The arbitration T-1..T-5 and the message P3 reads as arming criterion 1.

The case that carries this batch is test_unheard_source_is_absent_not_listed: a
gamepad nobody has ever seen must not appear in sources[] at all. P3 asks that
list whether a local e-stop is reachable, and an entry saying "gamepad, stale"
invites the reader to treat absence of evidence as evidence of a device.
"""
from __future__ import annotations

import pytest

from xbrain.p1_motion.teleop.state import (
    LOCAL_ESTOP_DEVICES, SWITCH_HYSTERESIS_MS, TELEOP_DEVICES, TeleopTracker,
    has_local_estop_source,
)

pytestmark = pytest.mark.no_device


def _tracker():
    return TeleopTracker()


# ------------------------------------------------------------- payload -----

def test_empty_tracker_reports_nobody_at_the_controls():
    """The honest answer on a machine with no input device -- and the one the
    arming gate needs. MUTATION: publish nothing until a source appears and P3
    cannot tell "no controller" from "P1 is down"."""
    state = _tracker().build_state(1000)
    assert state["active_source"] == "none"
    assert state["deadman"] is False
    assert state["sources"] == []
    assert state["axes_out"] == {"vx": 0.0, "vy": 0.0, "wz": 0.0}


def test_unheard_source_is_absent_not_listed():
    """*** MUTATION: pre-populate sources[] with every known device marked
    stale -- has_local_estop_source still answers no today, but the list now
    asserts a gamepad exists, and the next reader to check `device in sources`
    instead of `alive` grants the arming gate on a device that was never there.
    """
    t = _tracker()
    t.observe("keyboard_hmi", now_mono_ms=1000, deadman=True)
    devices = [s["device"] for s in t.build_state(1000)["sources"]]
    assert devices == ["keyboard_hmi"]


def test_stale_source_is_reported_but_not_active():
    t = _tracker()
    t.observe("gamepad", now_mono_ms=1000, deadman=True)
    state = t.build_state(1000 + 500)          # gamepad times out at 200 ms
    entry = state["sources"][0]
    assert entry["stale"] is True and entry["alive"] is False
    assert entry["age_ms"] == 500
    assert state["active_source"] == "none"


def test_alive_and_stale_are_both_emitted():
    """S12A.3 tests `alive`, S12A.9.7 defines `stale`. Both go out so neither
    reader depends on the other's spelling. MUTATION: emit only one and the
    other consumer silently reads None -- which, for the arming gate, is a
    device that never counts."""
    t = _tracker()
    t.observe("gamepad", now_mono_ms=1000, deadman=True)
    entry = t.build_state(1000)["sources"][0]
    assert entry["alive"] is True and entry["stale"] is False


# --------------------------------------------------------- arbitration -----

def test_highest_priority_live_source_wins():
    t = _tracker()
    t.observe("keyboard_hmi", now_mono_ms=1000, deadman=True)
    assert t.build_state(1000)["active_source"] == "keyboard_hmi"
    # A gamepad arriving outranks it -- after the hysteresis (T-2).
    t.observe("gamepad", now_mono_ms=1000, deadman=True)
    assert t.build_state(1000)["active_source"] == "keyboard_hmi"
    t.observe("gamepad", now_mono_ms=1000 + SWITCH_HYSTERESIS_MS, deadman=True)
    t.observe("keyboard_hmi", now_mono_ms=1000 + SWITCH_HYSTERESIS_MS,
              deadman=True)
    assert t.build_state(1000 + SWITCH_HYSTERESIS_MS)["active_source"] == \
        "gamepad"


def test_releasing_the_deadman_drops_the_source_immediately():
    """*** Gaining a source waits out the hysteresis; LOSING one does not.

    MUTATION: apply the hysteresis symmetrically -- the robot keeps driving on
    an input the operator has already let go of, for 300 ms, which is exactly
    the window a deadman exists to close.
    """
    t = _tracker()
    t.observe("gamepad", now_mono_ms=1000, deadman=True)
    assert t.build_state(1000)["active_source"] == "gamepad"
    t.observe("gamepad", now_mono_ms=1100, deadman=False)
    assert t.build_state(1100)["active_source"] == "none"


def test_a_released_source_is_dropped_even_when_another_is_available():
    """*** The case that makes the immediate-release rule observable.

    With NO other source, releasing the deadman drops to none through the
    no-source path, so a test that only checks that stays green with the
    release rule deleted. The dangerous shape is: the gamepad is released
    while an HMI stick is live. Without the rule the arbitration waits out the
    hysteresis and keeps reporting gamepad -- a source the operator has let go
    of -- as the one driving, for 300 ms.

    MUTATION: remove the "active source no longer eligible" branch -- this
    reddens and the plain release case does not.
    """
    t = _tracker()
    t.observe("gamepad", now_mono_ms=1000, deadman=True)
    t.observe("keyboard_hmi", now_mono_ms=1000, deadman=True)
    assert t.build_state(1000)["active_source"] == "gamepad"
    # Gamepad released; the HMI stick is still held.
    t.observe("gamepad", now_mono_ms=1100, deadman=False)
    t.observe("keyboard_hmi", now_mono_ms=1100, deadman=True)
    assert t.build_state(1100)["active_source"] == "keyboard_hmi"


def test_a_source_going_stale_drops_it_immediately():
    t = _tracker()
    t.observe("gamepad", now_mono_ms=1000, deadman=True)
    t.build_state(1000)
    assert t.build_state(1000 + 201)["active_source"] == "none"


def test_axes_come_from_the_active_source_only():
    """S12A.9.7: axes_out is what the ACTIVE source asked for. MUTATION: merge
    every source's axes and a stale HMI stick keeps contributing to a command
    the gamepad is driving."""
    t = _tracker()
    t.observe("keyboard_hmi", now_mono_ms=1000, deadman=True,
              axes={"vx": 0.9})
    t.observe("gamepad", now_mono_ms=1000, deadman=True, axes={"vx": 0.2})
    t.observe("gamepad", now_mono_ms=1400, deadman=True, axes={"vx": 0.2})
    t.observe("keyboard_hmi", now_mono_ms=1400, deadman=True,
              axes={"vx": 0.9})
    state = t.build_state(1400)
    assert state["active_source"] == "gamepad"
    assert state["axes_out"]["vx"] == 0.2


def test_mark_seq_counts_rising_edges_monotonically():
    """S12A.9.8: P3 adds a point per increment, idempotently. MUTATION: count
    every frame whose mark flag is set (not just the edge) and a held button
    adds a vertex per frame."""
    t = _tracker()
    t.observe("gamepad", now_mono_ms=1000, deadman=True, mark_edge=True)
    t.observe("gamepad", now_mono_ms=1050, deadman=True, mark_edge=False)
    t.observe("gamepad", now_mono_ms=1100, deadman=True, mark_edge=True)
    assert t.build_state(1100)["mark_seq"] == 2


def test_unknown_device_is_refused():
    """The S12A.9.7 device set is closed and carries a per-device timeout and
    priority; an unknown name has neither. MUTATION: accept it and a source
    with no timeout never goes stale."""
    with pytest.raises(ValueError, match="unknown teleop device"):
        _tracker().observe("joystick", now_mono_ms=1, deadman=True)


# ------------------------------------------------- arming criterion 1 ------

def test_local_estop_devices_exclude_the_networked_keyboard():
    """*** S12A.3 criterion 1 read against the S12A.9.7 device set.

    keyboard_hmi crosses the network; criterion 7 exists because the operator
    needs a key they can physically reach while lateral avoidance and voice
    e-stop are both suppressed. MUTATION: include keyboard_hmi -- a recording
    arms with its only remaining e-stop on the far side of a WiFi link.
    """
    assert LOCAL_ESTOP_DEVICES == {"gamepad", "keyboard_local"}
    assert set(LOCAL_ESTOP_DEVICES) <= set(TELEOP_DEVICES)
    t = _tracker()
    t.observe("keyboard_hmi", now_mono_ms=1000, deadman=True)
    assert has_local_estop_source(t.build_state(1000)) is False
    t.observe("gamepad", now_mono_ms=1000, deadman=True)
    assert has_local_estop_source(t.build_state(1000)) is True


def test_stale_local_source_does_not_satisfy_the_criterion():
    """A gamepad that has stopped reporting is not an e-stop the operator can
    reach. MUTATION: test presence instead of aliveness and a controller
    unplugged mid-session keeps the gate satisfied."""
    t = _tracker()
    t.observe("gamepad", now_mono_ms=1000, deadman=True)
    assert has_local_estop_source(t.build_state(1000)) is True
    assert has_local_estop_source(t.build_state(1000 + 5000)) is False


def test_criterion_falls_back_to_stale_when_alive_is_absent():
    """A producer that emits only the S12A.9.7 spelling must still be read."""
    assert has_local_estop_source(
        {"sources": [{"device": "gamepad", "stale": False}]}) is True
    assert has_local_estop_source(
        {"sources": [{"device": "gamepad", "stale": True}]}) is False
    assert has_local_estop_source(None) is False
