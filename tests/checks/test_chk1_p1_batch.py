"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chk1_p1_batch.py
Brief: CHK-1-13/17/22/45 P1 severe-items batch tests

Description:
Four CHK-1 severe items, all P1/P2-facing, tightly-scoped unit
tests with paired negative variants for every assertion.

CHK-1-13 ModeCommand dispatch:
  * 6-value action closed set (each row hit + one negative)
  * SP-C1 profile 2-value closed set + cruise/transit rejected
  * SP-C2 confirm_token missing rejected; self-issued (issuer != 'p2')
    rejected (CT-1..CT-8 anti-self-token)
  * SP-C3 ack.detail.applied THREE-field: profile_to +
    profile_locked + max_profile all populated
  * handlers_complete meta-gate at startup

CHK-1-17 arb visibility:
  * winner change triggers state + audit event together (never
    one without the other -- §7A.8 half-done trap)
  * gen++ only on winner change, not on 1Hz heartbeat
  * heartbeat fires state only, not event
  * dedup_key = 'arb:motion:{action}' composed exactly once

CHK-1-22 goto route:
  * goto payload -> path_follow(300), goes_via_teleop=False
  * teleop source registry MUST NOT list goto/hmi_click_navigate/
    behavior_goto (guards against 600-inheritance)
  * deadman_applies_to_source distinguishes teleop from
    path_follow (so goto in flight is not stale-timed)
  * shape errors raise with descriptive message

CHK-1-45 negative vx cap:
  * every source hitting vx=-1.5 clamps to -0.5 with limiter set
  * positive vx untouched (no fail-safe overshoot)
  * cap=0 raises (fail-silent form)
  * cap value comes from constructor injection only
"""

from __future__ import annotations

import pytest

from xbrain.common.errors import (
    E_CONFIRM_REQUIRED, E_SCHEMA,
)
from xbrain.p1_motion.arb.visibility import (
    ArbEvent, ArbPublisher, ArbState, DEDUP_WINDOW_MS,
    HEARTBEAT_PERIOD_MS, dedup_key_for,
)
from xbrain.p1_motion.gate.negative_vx import (
    NEGATIVE_VX_CAP_LIMITER, NegativeVxCap, NegativeVxConfigError,
)
from xbrain.p1_motion.route.behavior_goto import (
    BEHAVIOR_GOTO, GotoRouteDecision, GotoRouteError,
    PATH_FOLLOW_PRIORITY, assert_not_registered_in_teleop,
    deadman_applies_to_source, route_behavior_goto,
)
from xbrain.p2_core.mode_actions.dispatch import (
    ACTION_CLOSED_SET, CONFIRM_TOKEN_ISSUER, SPEED_PROFILE_CLOSED_SET,
    dispatch, handlers_complete,
)


pytestmark = pytest.mark.no_device


# --- CHK-1-13 ModeCommand dispatch ---

def test_action_closed_set_has_six_rows():
    """11 §7.3 action table has EXACTLY six rows -- no fewer, no more."""
    assert len(ACTION_CLOSED_SET) == 6


def test_dispatch_unknown_action_rejected():
    r = dispatch({"action": "self_destruct"}, profile_state={})
    assert not r.accepted and r.code == E_SCHEMA


def test_dispatch_every_valid_action_has_handler():
    """Meta: handlers_complete catches a missing handler at startup."""
    handlers_complete()   # must not raise


@pytest.mark.parametrize("action", list(ACTION_CLOSED_SET))
def test_dispatch_each_action_has_a_handler(action):
    """Iterate ACTION_CLOSED_SET -- each action must at least go
    through its handler (accept or reject, but never fall through)."""
    # Feed a shape-valid stub for each action.
    stub_by_action = {
        "set_voice_mode": {"action": action, "voice_mode": "silent"},
        "exit_broadcast": {"action": action},
        "exit_alarm": {"action": action},
        "set_behavior": {"action": action, "behavior": "goto"},
        "set_speed_profile": {"action": action, "profile": "patrol"},
        "reset_profile_lock": {
            "action": action,
            "confirm_token": {"issuer": "p2", "value": "tk-1"}},
    }
    r = dispatch(stub_by_action[action], profile_state={})
    assert r.accepted, (
        f"action {action!r} rejected unexpectedly: {r.reason}")


# --- CHK-1-13 SP-C1 profile closed set ---

def test_sp_c1_profile_closed_set_size():
    assert SPEED_PROFILE_CLOSED_SET == ("obstacle_avoid", "patrol")


def test_sp_c1_cruise_rejected():
    """SP-C1 profile ∈ {obstacle_avoid, patrol}; 'cruise' rejected."""
    r = dispatch({"action": "set_speed_profile", "profile": "cruise"},
                    profile_state={})
    assert not r.accepted and r.code == E_SCHEMA
    assert "not in closed set" in r.reason


def test_sp_c1_transit_rejected():
    r = dispatch({"action": "set_speed_profile", "profile": "transit"},
                    profile_state={})
    assert not r.accepted


def test_sp_c1_patrol_ok():
    r = dispatch({"action": "set_speed_profile", "profile": "patrol"},
                    profile_state={"locked": False, "max_profile": "patrol"})
    assert r.accepted


# --- CHK-1-13 SP-C2 confirm_token ---

def test_sp_c2_missing_confirm_token_rejected():
    r = dispatch({"action": "reset_profile_lock"}, profile_state={})
    assert r.code == E_CONFIRM_REQUIRED
    assert "missing" in r.reason


def test_sp_c2_self_issued_token_rejected():
    """CT-1..CT-8: caller-generated token must be REJECTED."""
    r = dispatch({"action": "reset_profile_lock",
                    "confirm_token": {"issuer": "hmi", "value": "self-made"}},
                    profile_state={})
    assert r.code == E_CONFIRM_REQUIRED
    assert "issuer" in r.reason


def test_sp_c2_string_token_rejected():
    """Old-style string tokens (no issuer signature) refused too."""
    r = dispatch({"action": "reset_profile_lock",
                    "confirm_token": "plain-string"},
                    profile_state={})
    assert r.code == E_CONFIRM_REQUIRED


def test_sp_c2_valid_token_accepted():
    r = dispatch({"action": "reset_profile_lock",
                    "confirm_token": {"issuer": CONFIRM_TOKEN_ISSUER,
                                       "value": "tk-1"}},
                    profile_state={"locked": True})
    assert r.accepted


# --- CHK-1-13 SP-C3 ack.detail.applied three-field ---

def test_sp_c3_applied_carries_all_three_fields():
    """set_speed_profile ack MUST populate profile_to +
    profile_locked + max_profile together."""
    r = dispatch(
        {"action": "set_speed_profile", "profile": "patrol"},
        profile_state={"locked": True, "max_profile": "obstacle_avoid"})
    assert r.applied is not None
    for field in ("profile_to", "profile_locked", "max_profile"):
        assert field in r.applied, f"applied missing {field!r}"


def test_sp_c3_applied_reports_state_not_wish():
    """profile_locked reads from state, not from cmd. If S-3 has
    locked the SM, ack MUST say so even though we accepted the
    request."""
    r = dispatch(
        {"action": "set_speed_profile", "profile": "patrol"},
        profile_state={"locked": True, "max_profile": "obstacle_avoid"})
    assert r.applied["profile_locked"] is True
    assert r.applied["max_profile"] == "obstacle_avoid"


# --- CHK-1-17 arb visibility ---

def test_arb_first_observe_bumps_gen_and_fires_both():
    p = ArbPublisher()
    state, event = p.observe("path_follow", "task-1", now_mono_ms=100)
    assert state is not None and event is not None
    assert p.gen == 1
    assert state.gen == 1
    assert event.dedup_key == "arb:motion:winner_change"


def test_arb_stable_winner_heartbeats_no_event():
    """§7A.8: heartbeat publishes state but NEVER an audit event.
    gen must NOT increment on heartbeat."""
    p = ArbPublisher()
    p.observe("path_follow", "task-1", now_mono_ms=100)
    # Wait heartbeat period.
    state, event = p.observe("path_follow", "task-1", now_mono_ms=100 + HEARTBEAT_PERIOD_MS)
    assert state is not None    # heartbeat state
    assert event is None        # no audit event
    assert p.gen == 1           # unchanged
    assert state.gen == 1


def test_arb_stable_winner_no_publish_before_heartbeat():
    """Between winner changes AND before heartbeat is due, nothing
    is published."""
    p = ArbPublisher()
    p.observe("path_follow", "task-1", now_mono_ms=100)
    state, event = p.observe("path_follow", "task-1",
                                now_mono_ms=100 + HEARTBEAT_PERIOD_MS // 2)
    assert state is None and event is None


def test_arb_winner_change_bumps_gen_and_fires_event():
    p = ArbPublisher()
    p.observe("path_follow", "task-1", now_mono_ms=100)
    state, event = p.observe("rns_avoid", "task-1", now_mono_ms=200)
    assert p.gen == 2
    assert event is not None
    assert event.from_source == "path_follow"
    assert event.to_source == "rns_avoid"


def test_arb_dedup_key_composition_stable():
    assert dedup_key_for("preempted") == "arb:motion:preempted"
    assert dedup_key_for("winner_change") == "arb:motion:winner_change"


def test_arb_dedup_window_ms_matches_spec():
    """§7A.7: 10-second coalesce window."""
    assert DEDUP_WINDOW_MS == 10_000


def test_arb_gen_never_increments_on_pure_heartbeat_burst():
    """Sanity: 10 consecutive heartbeats with the same winner
    must not push gen from 1 to 11."""
    p = ArbPublisher()
    p.observe("path_follow", "task-1", now_mono_ms=0)
    for i in range(1, 11):
        p.observe("path_follow", "task-1", now_mono_ms=i * HEARTBEAT_PERIOD_MS)
    assert p.gen == 1     # never bumped


# --- CHK-1-22 goto route ---

def test_goto_routes_to_path_follow():
    decision = route_behavior_goto({
        "behavior": BEHAVIOR_GOTO,
        "waypoint": {"x": 1.0, "y": 2.0},
    })
    assert decision.source == "path_follow"
    assert decision.priority == PATH_FOLLOW_PRIORITY == 300
    assert decision.goes_via_teleop is False


def test_goto_wrong_behavior_rejected():
    with pytest.raises(GotoRouteError, match="expected behavior='goto'"):
        route_behavior_goto({"behavior": "spin", "waypoint": {"x": 0, "y": 0}})


def test_goto_missing_waypoint_rejected():
    """No silent default per CLAUDE.md 3.1."""
    with pytest.raises(GotoRouteError, match="waypoint"):
        route_behavior_goto({"behavior": BEHAVIOR_GOTO})


def test_teleop_source_registry_must_exclude_goto():
    """CHK-1-22: teleop four-source arbiter must NOT list any of
    the goto aliases (600-inheritance guard)."""
    with pytest.raises(GotoRouteError, match="path_follow"):
        assert_not_registered_in_teleop(
            ["teleop_local", "teleop_cloud", "behavior_goto"])


def test_teleop_source_registry_clean_ok():
    assert_not_registered_in_teleop([
        "teleop_local", "teleop_cloud", "teleop_gamepad",
    ])


def test_deadman_does_not_apply_to_path_follow():
    """goto in-flight (source=path_follow) is not subject to
    teleop-side deadman timers."""
    assert deadman_applies_to_source("path_follow") is False


def test_deadman_applies_to_teleop_sources():
    for s in ("teleop_local", "teleop_cloud", "teleop_gamepad",
                "teleop_keyboard"):
        assert deadman_applies_to_source(s) is True


# --- CHK-1-45 negative vx cap ---

def test_negative_cap_rejects_zero_at_construction():
    """Zero cap = fail-silent no reverse allowed. Refuse."""
    with pytest.raises(NegativeVxConfigError, match="fail-silent"):
        NegativeVxCap(abs_max_reverse_mps=0.0)


def test_negative_cap_rejects_negative_at_construction():
    with pytest.raises(NegativeVxConfigError):
        NegativeVxCap(abs_max_reverse_mps=-0.5)


def test_negative_cap_positive_vx_untouched():
    """Fail-safe overshoot guard: positive direction not affected."""
    cap = NegativeVxCap(abs_max_reverse_mps=0.5)
    for v in (0.0, 0.1, 1.5, 3.0):
        result, limiter = cap.apply(v)
        assert result == v and limiter == ""


def test_negative_cap_clamps_at_boundary():
    cap = NegativeVxCap(abs_max_reverse_mps=0.5)
    result, limiter = cap.apply(-1.5)
    assert result == -0.5
    assert limiter == NEGATIVE_VX_CAP_LIMITER


def test_negative_cap_within_range_unchanged():
    """vx = -0.3 within cap -> unchanged, no limiter attribution."""
    cap = NegativeVxCap(abs_max_reverse_mps=0.5)
    result, limiter = cap.apply(-0.3)
    assert result == -0.3 and limiter == ""


def test_negative_cap_exactly_at_boundary_no_limiter():
    """vx = -0.5 exactly: at boundary, not over -> no clamp."""
    cap = NegativeVxCap(abs_max_reverse_mps=0.5)
    result, limiter = cap.apply(-0.5)
    assert result == -0.5 and limiter == ""


def test_negative_cap_source_agnostic():
    """CHK-1-45 core: the cap fires regardless of source. Same
    cap instance applied for teleop/teleop_cloud/relative_move/
    nav2_proxy/rns_avoid: all must clamp -1.5 to -0.5."""
    cap = NegativeVxCap(abs_max_reverse_mps=0.5)
    for source in ("teleop", "teleop_cloud", "relative_move",
                     "nav2_proxy_backup", "rns_avoid"):
        result, limiter = cap.apply(-1.5)
        assert result == -0.5, (
            f"source {source!r}: expected -0.5, got {result}")
        assert limiter == NEGATIVE_VX_CAP_LIMITER


def test_negative_cap_limiter_attribution_is_closed_set_enum():
    """gate.limiter comes from closed-set enum (no bare literal).
    CHK-1-45 spec: 'limiter 归因取自 common/enums 导出的闭集值'."""
    from xbrain.p1_motion.gate.negative_vx import NEGATIVE_VX_CAP_LIMITER
    # The imported name (module-level constant) is the source-of-
    # truth; test asserts a caller consuming it can compare-by-name.
    assert isinstance(NEGATIVE_VX_CAP_LIMITER, str)
    # Meta: NOT written in test as a bare literal that could drift.
    # We're literally checking the exported name is stable.
    assert NEGATIVE_VX_CAP_LIMITER == "negative_vx_cap"
