"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_mode_all.py
Brief: mode tests -- mode all

Description:
BIZ-P2-11/12/13 -- mode SM + device map + B-mode timer tests.
"""


import pytest

from xbrain.p2_core.mode.b_mode_timer import BModeTimer
from xbrain.p2_core.mode.device_map import (
    SWITCH_ORDER, is_409_ok, is_device_mode_change, to_device_mode,
)
from xbrain.p2_core.mode.state_machine import (
    ModeState, ModeStateMachine, TransitionRequest, TriggerKind,
)


pytestmark = pytest.mark.no_device


# --- BIZ-P2-11: state machine ---

def test_target_equals_current_returns_accepted_no_work():
    """14 S5.5: transition to current state = accepted, no work, no event."""
    sm = ModeStateMachine(initial=ModeState.IDLE)
    r = sm.request(TransitionRequest(ModeState.IDLE, TriggerKind.CMD))
    assert r.accepted
    assert r.from_state == r.to_state == ModeState.IDLE
    assert r.reason == "already_in_target_state"


def test_transition_committed_updates_state():
    sm = ModeStateMachine(initial=ModeState.IDLE)
    r = sm.request(TransitionRequest(ModeState.ALARM, TriggerKind.CMD))
    assert r.accepted
    assert sm.state == ModeState.ALARM


def test_p1_preflight_reports_all_blocked_not_short_circuit():
    """BIZ-P2-11 spec: blocked[] MUST report ALL conflicting domains,
    not short-circuit on the first one."""
    sm = ModeStateMachine(initial=ModeState.IDLE)
    r = sm.request(
        TransitionRequest(ModeState.ALARM, TriggerKind.AUTONOMOUS),
        blocked_domains=frozenset({"speaker", "payload_light"}),
    )
    assert not r.accepted
    assert r.blocked == frozenset({"speaker", "payload_light"})
    assert r.reason == "preflight_blocked"


def test_min_dwell_only_applies_to_autonomous():
    """MD-1: cmd / timeout / safety triggers EXEMPT."""
    sm = ModeStateMachine(initial=ModeState.IDLE)
    # autonomous with dwell_ok=False -> denied
    r_auto = sm.request(
        TransitionRequest(ModeState.ALARM, TriggerKind.AUTONOMOUS),
        dwell_ok=False,
    )
    assert not r_auto.accepted
    # cmd with dwell_ok=False -> accepted (exempt)
    sm2 = ModeStateMachine(initial=ModeState.IDLE)
    r_cmd = sm2.request(
        TransitionRequest(ModeState.ALARM, TriggerKind.CMD),
        dwell_ok=False,
    )
    assert r_cmd.accepted


def test_cmd_idempotency_returns_replay():
    """Duplicate cmd_id returns first-call result with is_replay=True."""
    sm = ModeStateMachine(initial=ModeState.IDLE)
    r1 = sm.request(TransitionRequest(
        ModeState.ALARM, TriggerKind.CMD, cmd_id="c1"))
    r2 = sm.request(TransitionRequest(
        ModeState.ALARM, TriggerKind.CMD, cmd_id="c1"))
    assert r1.accepted
    assert r2.accepted
    assert r2.is_replay
    assert r2.to_state == r1.to_state


# --- BIZ-P2-12: device map ---

def test_device_map_dialog_variants_all_func1():
    """MM-1: dialog A/C/E all map to func1 -> no /mode call between them."""
    assert to_device_mode(ModeState.DIALOG_A) == "func1"
    assert to_device_mode(ModeState.DIALOG_C) == "func1"
    assert to_device_mode(ModeState.DIALOG_E) == "func1"
    assert to_device_mode(ModeState.IDLE) == "func1"


def test_device_map_broadcast_and_alarm_distinct():
    assert to_device_mode(ModeState.BROADCAST) == "func2"
    assert to_device_mode(ModeState.ALARM) == "deter"


def test_is_device_mode_change_false_for_dialog_variants():
    """A -> C: same device mode -> no POST /mode."""
    assert not is_device_mode_change(ModeState.DIALOG_A, ModeState.DIALOG_C)
    assert not is_device_mode_change(ModeState.IDLE, ModeState.DIALOG_A)


def test_is_device_mode_change_true_for_dialog_to_broadcast():
    assert is_device_mode_change(ModeState.DIALOG_A, ModeState.BROADCAST)


def test_ml2_409_during_switching_is_ok():
    """ML-2: 409 during switching = already in target device mode = OK."""
    assert is_409_ok(409, in_switching_transition=True) is True
    assert is_409_ok(200, in_switching_transition=True) is False
    assert is_409_ok(409, in_switching_transition=False) is False


def test_switch_order_matches_documented_five():
    """ML-5 order. If this drifts from p2_core.yaml.mode.switch_order,
    check_switch_order assertion catches it separately (see
    tests/p2_core/config/test_assertions.py)."""
    assert SWITCH_ORDER == [
        "device_mode", "payload_light", "ptz", "motion", "audio",
    ]


# --- BIZ-P2-13: B-mode timer ---

def test_b_mode_timer_start_and_expire():
    t = BModeTimer(max_duration_s=300.0)
    t.start(now_mono_ms=0)
    assert not t.expired(now_mono_ms=100_000)
    assert not t.expired(now_mono_ms=299_999)
    assert t.expired(now_mono_ms=300_001)


def test_b_mode_timer_not_running_never_expires():
    t = BModeTimer(max_duration_s=300.0)
    assert not t.expired(now_mono_ms=999_999_999)


def test_b_mode_timer_no_reset_method():
    """BCT-2: no reset() method. To restart, must stop() then start().
    (Verifies by checking no reset attribute exists.)"""
    t = BModeTimer(max_duration_s=1.0)
    assert not hasattr(t, "reset"), \
        "BModeTimer must NOT expose reset() -- BCT-2 forbids"


def test_b_mode_timer_stop_clears():
    t = BModeTimer(max_duration_s=1.0)
    t.start(now_mono_ms=0)
    t.stop()
    assert not t.expired(now_mono_ms=999_999)
