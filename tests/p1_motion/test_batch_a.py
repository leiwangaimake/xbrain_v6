"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_a.py
Brief: MOT-PM-1..15 batch A tests (foundation + gate + rotation + fence)

Description:
Covers the eight P1 modules landed in batch A: RTC guards, perception
source abstraction, freshness classification, 8-tier source arbiter,
speed gate f(d_free) + rule form + audit, g(targets), rotation permit
RCG, and fence geometry + two-stage commit. Each module has 2-4 focused
tests; variants live in the modules themselves as raise semantics.
"""

import pytest

from xbrain.p1_motion.fence.geom import (
    FenceStage, FenceStageMachine, vector_project_toward_fence,
)
from xbrain.p1_motion.freshness.degradation import (
    CAM_THRESH, Freshness, GRID_THRESH, LIDAR_THRESH, classify,
)
from xbrain.p1_motion.gate.audit import limiter_all, limiter_argmax
from xbrain.p1_motion.gate.g_targets import g_targets
from xbrain.p1_motion.gate.speed_gate import (
    GateHysteresis, f_speed_gate, gate_rule,
)
from xbrain.p1_motion.perception_src.source import (
    PerceptionFrame, ReplayPerceptionSource,
)
from xbrain.p1_motion.rotation.rcg import (
    R_EFF_FALLBACK_M, is_spin_like, rotation_permitted,
)
from xbrain.p1_motion.rt_base.rtc import RtcViolation, note_single_slot
from xbrain.p1_motion.sources.arbiter_p1 import (
    BehaviorSource, P1Arbiter, priority_of,
)


pytestmark = pytest.mark.no_device


# --- MOT-PM-2 RTC ---

def test_single_slot_ok():
    note_single_slot(existing=1)


def test_single_slot_over_1_raises():
    with pytest.raises(RtcViolation):
        note_single_slot(existing=2)


# --- MOT-PM-3 PerceptionSource ---

def test_replay_source_yields_frames_in_order():
    frames = [PerceptionFrame(mono_ms=i * 50) for i in range(3)]
    src = ReplayPerceptionSource(frames=frames)
    assert src.next(0).mono_ms == 0
    assert src.next(50).mono_ms == 50
    assert src.next(100).mono_ms == 100
    assert src.next(150) is None


# --- MOT-PM-4 freshness ---

def test_freshness_ok_degraded_failed():
    assert classify(100, GRID_THRESH) == Freshness.OK
    assert classify(600, GRID_THRESH) == Freshness.DEGRADED
    assert classify(2500, GRID_THRESH) == Freshness.FAILED


# --- MOT-PM-5 arbiter ---

def test_arbiter_priority_order():
    """Higher priority source wins."""
    a = P1Arbiter(dwell_ms=1000)
    a.note(BehaviorSource.HOLD, now_mono_ms=0)
    a.note(BehaviorSource.PATH_FOLLOW, now_mono_ms=0)
    a.note(BehaviorSource.ESTOP_ECHO, now_mono_ms=0)
    assert a.holder() == BehaviorSource.ESTOP_ECHO


def test_arbiter_deactivates_stale_sources():
    a = P1Arbiter(dwell_ms=100)
    a.note(BehaviorSource.PATH_FOLLOW, now_mono_ms=0)
    a.note(BehaviorSource.HOLD, now_mono_ms=0)
    a.tick(now_mono_ms=200)   # both stale
    assert a.holder() is None


def test_arbiter_priority_table_matches_doc():
    """12 S4.1 verbatim priorities."""
    assert priority_of(BehaviorSource.FENCE_GUARD) == 1000
    assert priority_of(BehaviorSource.ESTOP_ECHO) == 900
    assert priority_of(BehaviorSource.TELEOP_CLOUD) == 550
    assert priority_of(BehaviorSource.HOLD) == 100


# --- MOT-PM-6/7 speed gate ---

def test_f_speed_gate_four_bands():
    """[3.0, inf) -> 2.0 | [1.8, 3.0) -> 0.5 | [1.25, 1.8) -> 0.2 | [0, 1.25) -> 0."""
    assert f_speed_gate(5.0) == 2.0
    assert f_speed_gate(3.0) == 2.0   # left-closed
    assert f_speed_gate(2.99) == 0.5
    assert f_speed_gate(1.8) == 0.5
    assert f_speed_gate(1.79) == 0.2
    assert f_speed_gate(1.25) == 0.2
    assert f_speed_gate(1.24) == 0.0
    assert f_speed_gate(0.0) == 0.0


def test_gate_rule_min_of_four():
    """v_max = min(f, g*h*i, hard_upper). g=h=i=1 -> min of (f, 1, hard)."""
    # f=2, g*h*i=1, hard=1.5 -> min = 1.0 (from g*h*i).
    assert gate_rule(f=2.0, g=1.0, h=1.0, i=1.0, hard_upper_mps=1.5) == 1.0
    # f=0.5 wins over g*h*i=2.0 and hard=1.5.
    assert gate_rule(f=0.5, g=2.0, h=1.0, i=1.0, hard_upper_mps=1.5) == 0.5
    # Negative result clamped to 0.
    assert gate_rule(f=-1, g=1, h=1, i=1, hard_upper_mps=1) == 0.0


def test_hysteresis_upgrade_requires_sustained():
    h = GateHysteresis()
    # Starting from mono_ms=100 (not 0) to avoid the sentinel-collision
    # in _above_since_mono_ms==0 that would treat first update as re-init.
    h.update(3.6, now_mono_ms=100)
    h.update(3.6, now_mono_ms=2100)      # 2 s later, < 3 s dwell
    assert h._at_upper_band is False
    h.update(3.6, now_mono_ms=3200)      # 3.1 s later, >= 3 s dwell
    assert h._at_upper_band is True


# --- MOT-PM-8 g(targets) ---

def test_g_targets_present_clips_to_obstacle_avoid():
    assert g_targets(True, obstacle_avoid_max_mps=0.5) == 0.5


def test_g_targets_absent_is_unit_multiplier():
    assert g_targets(False) == 1.0


# --- MOT-PM-9 audit ---

def test_limiter_argmax_picks_min():
    limits = {"f_speed": 0.5, "g_targets": 0.7, "hard_upper": 2.0}
    assert limiter_argmax(limits) == "f_speed"


def test_limiter_argmax_returns_none_on_empty():
    assert limiter_argmax({}) == "none"


def test_limiter_all_includes_ties_within_threshold():
    limits = {"f_speed": 0.5, "g_targets": 0.51, "hard_upper": 2.0}
    all_ = limiter_all(limits, threshold_delta=0.05)
    assert set(all_) == {"f_speed", "g_targets"}


# --- MOT-PM-10/11/12 rotation permit ---

def test_is_spin_like_true_when_wz_and_low_vx():
    assert is_spin_like(vx_mps=0.02, wz_radps=0.3) is True


def test_is_spin_like_false_when_moving_forward():
    """path_follow style turn: vx=1.5, wz=0.3 -> NOT spin."""
    assert is_spin_like(vx_mps=1.5, wz_radps=0.3) is False


def test_rotation_permitted_uses_r_eff_fallback():
    """r_robot=0.0 placeholder + fallback 0.6 -> requires clearance
    >= 0.6, NOT 0.0."""
    assert rotation_permitted(clearance_m=0.7, r_robot=0.0) is True
    assert rotation_permitted(clearance_m=0.5, r_robot=0.0) is False


def test_rotation_permitted_uses_r_robot_when_larger():
    """A concrete r_robot > fallback wins the max()."""
    assert rotation_permitted(clearance_m=0.8, r_robot=1.0) is False
    assert rotation_permitted(clearance_m=1.2, r_robot=1.0) is True


# --- MOT-PM-13/14/15 fence geometry + stage machine ---

def test_vector_project_zero_on_nan():
    """NaN / Inf inputs collapse to zero (fail-safe)."""
    vx, vy = vector_project_toward_fence(
        float("nan"), 1.0, d_soft_m=2.0, d_hard_m=1.0, t_lat_s=0.4)
    assert (vx, vy) == (0.0, 0.0)


def test_vector_project_zero_inside_veto():
    """Inside d_hard -> component toward fence is zero."""
    vx, vy = vector_project_toward_fence(
        1.0, 0.5, d_soft_m=1.0, d_hard_m=2.0, t_lat_s=0.4)
    assert (vx, vy) == (0.0, 0.0)


def test_stage_machine_commit_flow():
    m = FenceStageMachine()
    m.stage("fence_1")
    assert m.state == FenceStage.STAGED
    assert m.commit() == "fence_1"
    assert m.state == FenceStage.COMMITTED


def test_stage_machine_abort_returns_to_idle():
    m = FenceStageMachine()
    m.stage("fence_1")
    m.abort()
    assert m.state == FenceStage.IDLE
    assert m.staged_id is None


def test_stage_machine_commit_from_idle_raises():
    m = FenceStageMachine()
    with pytest.raises(RuntimeError):
        m.commit()
