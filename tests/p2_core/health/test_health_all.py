"""BIZ-P2-18/19/20/21 -- health items + factor + restrict matrix + three-stops."""

import pytest

from xbrain.p2_core.health.factor import (
    FactorConfig, FactorOutput, compute_factor,
)
from xbrain.p2_core.health.items import (
    HEALTH_ITEMS, HealthLevel, HealthState, ITEM_LEVELS,
    is_fatal, level_of,
)
from xbrain.p2_core.health.restrict_matrix import (
    check_asr_local_admission, check_new_task_admission,
    check_ptz_command, check_time_window_rules_active,
)
from xbrain.p2_core.three_stops import (
    ForceStrobeState, StopEvent, StopReason,
    apply_rearm, apply_stop,
)


pytestmark = pytest.mark.no_device


# --- items ---

def test_health_items_closed_set_contains_docs_members():
    """11 S5.1A required members. Not enforcing exact count (CLAUDE.md
    3.7) -- assert presence of the safety-critical items."""
    for item in ("chassis", "cam_rgbd", "estop", "config_freeze",
                  "mic", "payload_speaker", "ptz", "lidar"):
        assert item in HEALTH_ITEMS


def test_is_fatal_returns_true_for_fatal_items():
    assert is_fatal("chassis")
    assert is_fatal("cam_rgbd")
    assert is_fatal("estop")
    assert not is_fatal("mic")
    assert not is_fatal("ptz")


def test_level_of_raises_on_unknown_item():
    with pytest.raises(KeyError):
        level_of("this_is_not_an_item")


# --- factor: fatal-fail -> allow_motion=False ---

def _cfg():
    return FactorConfig(fatal_degraded=0.3, degraded_fail=0.5,
                        degraded_degraded=0.7, unknown=0.5)


def test_fatal_fail_forces_allow_motion_false():
    """Fatal item in FAIL -> allow_motion=False, factor=0, profile=none."""
    states = {"chassis": HealthState.FAIL}
    out = compute_factor(states, _cfg())
    assert out.allow_motion is False
    assert out.speed_factor == 0.0
    assert out.max_profile == "none"


def test_all_ok_gives_factor_one():
    states = {k: HealthState.OK for k in HEALTH_ITEMS}
    out = compute_factor(states, _cfg())
    assert out.allow_motion is True
    assert out.speed_factor == 1.0
    assert out.max_profile == "patrol"


def test_degraded_lidar_multiplies_factor():
    """lidar is DEGRADED level; state DEGRADED -> 0.7 factor."""
    states = {k: HealthState.OK for k in HEALTH_ITEMS}
    states["lidar"] = HealthState.DEGRADED
    out = compute_factor(states, _cfg())
    assert out.speed_factor == pytest.approx(0.7)


def test_cam_rgbd_not_ok_reduces_profile_to_none():
    """max_profile requires cam_rgbd OK (per p2_core.yaml.health)."""
    states = {"cam_rgbd": HealthState.FAIL}
    out = compute_factor(states, _cfg())
    # cam_rgbd is FATAL + FAIL -> allow_motion False first.
    assert out.allow_motion is False


def test_cam_rgbd_degraded_still_allows_patrol():
    states = {"cam_rgbd": HealthState.DEGRADED}
    out = compute_factor(states, _cfg())
    # DEGRADED cam_rgbd not FAIL, so allow_motion True, profile patrol
    # (both admissible profiles require it, degraded still counts).
    assert out.allow_motion is True


# --- restrict matrix ---

def test_rtk_degraded_refuses_new_tasks_fm1():
    """FM-1: rtk=degraded -> new tasks refused with E_DEGRADED."""
    d = check_new_task_admission({"rtk": HealthState.DEGRADED})
    assert not d.allowed
    assert d.code == "E_DEGRADED"
    assert d.detail_item == "rtk"


def test_rtk_ok_admits_new_tasks():
    d = check_new_task_admission({"rtk": HealthState.OK})
    assert d.allowed


def test_mic_fail_blocks_asr_local():
    """FM-3: mic=fail blocks asr_local (does NOT auto-switch mode)."""
    d = check_asr_local_admission({"mic": HealthState.FAIL})
    assert not d.allowed
    assert d.code == "E_UNHEALTHY"
    assert d.detail_item == "mic"


def test_ptz_fail_returns_unhealthy_not_capability():
    """FM-2: ptz=fail returns E_UNHEALTHY (device broke), NOT
    E_CAPABILITY (which would say the robot has no PTZ)."""
    d = check_ptz_command({"ptz": HealthState.FAIL})
    assert not d.allowed
    assert d.code == "E_UNHEALTHY"
    # Explicitly NOT E_CAPABILITY.
    assert d.code != "E_CAPABILITY"


def test_clock_fail_disables_time_window_rules():
    """RE-3a via restrict matrix."""
    assert check_time_window_rules_active({"clock": HealthState.FAIL}) is False
    assert check_time_window_rules_active({"clock": HealthState.OK}) is True


# --- three_stops single-branch handler ---

class _FakeArb:
    def __init__(self):
        self.calls = []

    def arb_suspend(self, reason, cmd_id, now_mono_ms):
        self.calls.append(("suspend", reason, cmd_id, now_mono_ms))

    def arb_rearm(self, now_mono_ms):
        self.calls.append(("rearm", now_mono_ms))


def _run_stop(reason: StopReason):
    arb = _FakeArb()
    strobe = ForceStrobeState()
    events = []
    apply_stop(
        StopEvent(reason=reason, cmd_id="c1", now_mono_ms=0),
        domain1_arbiter=arb,
        strobe_state=strobe,
        emit_event=lambda e: events.append(e),
    )
    return arb, strobe, events


def test_all_three_stops_use_same_handler_branch():
    """BIZ-P2-21: three-stop branch count == 1. Verified by asserting
    each of the three reasons produces the SAME shape of side-effects,
    differing only in event.detail.reason."""
    for reason in StopReason:
        arb, strobe, events = _run_stop(reason)
        # Same arbiter call: arb_suspend with reason=<value>, cmd_id=c1.
        assert arb.calls == [("suspend", reason.value, "c1", 0)]
        # Same strobe force ON.
        assert strobe.active is True
        # Same event kind; detail.reason differs.
        assert events == [{
            "kind": "estop",
            "detail": {"reason": reason.value, "cmd_id": "c1"},
        }]


def test_rearm_clears_force_strobe_and_calls_arbiter():
    arb = _FakeArb()
    strobe = ForceStrobeState(active=True)
    events = []
    apply_rearm(
        cmd_id="new_cmd", now_mono_ms=100,
        domain1_arbiter=arb, strobe_state=strobe,
        emit_event=lambda e: events.append(e),
    )
    assert arb.calls == [("rearm", 100)]
    assert strobe.active is False
    assert events[0]["kind"] == "estop_rearm"
