"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chk_batch.py
Brief: CHK consolidation batch tests (safety disciplines)

Description:
CHK-1-01 single-battery mode: max_vx_eff enters min() only when
active; ETA source flips to actual velocity; cannot exit at
runtime.
CHK-1-09 IR camera health item: 3-consecutive-fail warn latch;
non-blocking; commands rejected with E_IR_UNAVAILABLE when
unhealthy.
CHK-1-12 profile SM: S-1 downshift; S-2 upshift requires ALL 5
conditions; S-3 3-in-window locks the profile; unlock only via
closed-set source strings.
CHK-1-20 visual override: all 4 conditions required; RC-D7 static
scan refuses forbidden clearance-disable keys.
CHK-1-21 creep clamp: 0 < v < v_creep -> 0; startup gate rejects
v_creep >= min band.
CHK-2-25 alias blacklist walks config; renamed old keys raise.
CHK-2-47 closed-set literal scan finds hardcoded tokens.
"""

import pytest

from xbrain.common.checks.alias_blacklist import (
    AliasKeyFound,
    check_source_for_closed_set_literals, scan_config_for_alias,
)
from xbrain.p1_motion.gate.creep import (
    CreepConfigError, apply_creep_clamp, assert_creep_below_gate_min,
)
from xbrain.p1_motion.profile.switch_sm import (
    InvalidProfileTransition, ProfileSwitchSM,
    all_five_upshift_conditions,
)
from xbrain.p1_motion.rotation.visual_override import (
    DETAIL_KIND, ForbiddenClearanceToggle, VisualOverrideDenied,
    check_all_four, rc_d7_scan,
)
from xbrain.p2_core.health.aux.ir_camera import (
    IR_UNAVAILABLE_ERR, IrCameraHealth,
    is_blocking_for_availability, reject_ir_command_when_unhealthy,
)
from xbrain.p2_core.health.aux.single_battery import (
    SingleBatteryMode, choose_velocity_for_eta, compose_max_vx_min,
)


pytestmark = pytest.mark.no_device


# --- CHK-1-01 single battery ---

def test_single_battery_enter_idempotent():
    m = SingleBatteryMode()
    m.enter("second_battery_absent")
    m.enter("later_reason")
    assert m.entered_reason == "second_battery_absent"


def test_single_battery_cannot_exit_at_runtime():
    """Guard against flapping-battery scenario."""
    m = SingleBatteryMode()
    m.enter("some_reason")
    assert m.can_exit_at_runtime() is False


def test_compose_max_vx_min_active():
    """max_vx_eff enters min() when active."""
    assert compose_max_vx_min(spec_max_vx=2.0, spec_max_vx_eff=1.2,
                                 active=True) == 1.2


def test_compose_max_vx_min_inactive():
    """Inactive: normal max_vx returned regardless of eff."""
    assert compose_max_vx_min(spec_max_vx=2.0, spec_max_vx_eff=1.2,
                                 active=False) == 2.0


def test_eta_uses_actual_when_active():
    """When single battery, ETA reads actual (which lags)."""
    assert choose_velocity_for_eta(v_requested=2.0, v_actual=1.5,
                                     active=True) == 1.5


def test_eta_uses_requested_when_inactive():
    assert choose_velocity_for_eta(v_requested=2.0, v_actual=1.5,
                                     active=False) == 2.0


# --- CHK-1-09 IR camera ---

def test_ir_health_three_consecutive_fails():
    h = IrCameraHealth()
    for _ in range(3):
        h.record_probe(False)
    assert h.three_consecutive_fail() is True


def test_ir_health_recent_success_clears():
    h = IrCameraHealth()
    h.record_probe(False)
    h.record_probe(False)
    h.record_probe(True)
    assert h.three_consecutive_fail() is False


def test_ir_command_rejected_when_unhealthy():
    with pytest.raises(RuntimeError, match=IR_UNAVAILABLE_ERR):
        reject_ir_command_when_unhealthy(state="error")


def test_ir_command_ok_when_healthy():
    reject_ir_command_when_unhealthy(state="healthy")


def test_ir_is_non_blocking():
    """BIT-35 ruling: never gates patrol availability."""
    assert is_blocking_for_availability() is False


# --- CHK-1-12 profile SM ---

def test_downshift_drops_one_rung():
    sm = ProfileSwitchSM()
    sm.downshift(now_ms=1000)
    assert sm.current == "creep"


def test_downshift_at_stop_stays_stop():
    sm = ProfileSwitchSM(current="stop")
    sm.downshift(now_ms=1000)
    assert sm.current == "stop"


def test_thrash_locks_profile():
    """S-3: 3 downshifts in window -> PROFILE_LOCKED."""
    sm = ProfileSwitchSM()
    sm.downshift(1000)
    sm.downshift(2000)
    sm.downshift(3000)
    assert sm.locked is True


def test_upshift_blocked_when_locked():
    sm = ProfileSwitchSM(current="stop", locked=True)
    sm.upshift(all_five_conditions=True)
    assert sm.current == "stop"


def test_upshift_requires_all_five():
    sm = ProfileSwitchSM(current="creep")
    sm.upshift(all_five_conditions=False)
    assert sm.current == "creep"


def test_upshift_all_five_moves_up():
    sm = ProfileSwitchSM(current="creep")
    sm.upshift(all_five_conditions=True)
    assert sm.current == "patrol"


def test_unlock_closed_set_source():
    """Any source outside operator_reset / reboot / auto_unlock
    refused."""
    sm = ProfileSwitchSM(locked=True)
    with pytest.raises(InvalidProfileTransition, match="closed set"):
        sm.unlock(source="magic_wand")


def test_unlock_operator_reset_clears():
    sm = ProfileSwitchSM(locked=True)
    sm.unlock(source="operator_reset")
    assert sm.locked is False


def test_all_five_conditions_short_circuits_any_false():
    assert all_five_upshift_conditions(3.0, 2.0, True, True, True,
                                          True) is True
    # any one false ->
    assert all_five_upshift_conditions(1.0, 2.0, True, True, True,
                                          True) is False
    assert all_five_upshift_conditions(3.0, 2.0, False, True, True,
                                          True) is False


# --- CHK-1-20 visual override + RC-D7 ---

def test_v1_missing_rejects():
    with pytest.raises(VisualOverrideDenied, match="V-1"):
        check_all_four(False, True, True, DETAIL_KIND)


def test_v2_missing_rejects():
    with pytest.raises(VisualOverrideDenied, match="V-2"):
        check_all_four(True, False, True, DETAIL_KIND)


def test_v3_missing_rejects():
    with pytest.raises(VisualOverrideDenied, match="V-3"):
        check_all_four(True, True, False, DETAIL_KIND)


def test_v4_missing_rejects():
    """detail.kind must be exactly 'rotation_visual_override'."""
    with pytest.raises(VisualOverrideDenied, match="V-4"):
        check_all_four(True, True, True, "user")


def test_all_four_ok():
    check_all_four(True, True, True, DETAIL_KIND)


def test_rc_d7_scan_rejects_forbidden_key():
    """CLAUDE.md 3.6: no config can disable rotation clearance."""
    cfg = {"rotation_clearance": {"enabled": False}}
    with pytest.raises(ForbiddenClearanceToggle, match="RC-D7"):
        rc_d7_scan(cfg)


def test_rc_d7_scan_nested_key_caught():
    cfg = {"safety": {"rotation": {"permit": {"disabled": True}}}}
    with pytest.raises(ForbiddenClearanceToggle):
        rc_d7_scan(cfg)


def test_rc_d7_scan_clean_config_passes():
    cfg = {"rotation": {"permit_ttl_ms": 500}}
    rc_d7_scan(cfg)     # no raise


# --- CHK-1-21 creep ---

def test_creep_clamp_zeros_small_positive():
    assert apply_creep_clamp(v_max_mps=0.05, v_creep_mps=0.1) == 0.0


def test_creep_clamp_leaves_over_creep_alone():
    assert apply_creep_clamp(0.5, 0.1) == 0.5


def test_creep_clamp_zero_v_max_untouched():
    assert apply_creep_clamp(0.0, 0.1) == 0.0


def test_creep_clamp_rejects_zero_creep():
    with pytest.raises(CreepConfigError, match="> 0"):
        apply_creep_clamp(0.5, 0.0)


def test_startup_gate_rejects_creep_above_min_band():
    """v_creep >= min band would eat every allowance."""
    with pytest.raises(CreepConfigError, match="would clamp"):
        assert_creep_below_gate_min(v_creep_mps=0.3,
                                       band_values=[0.0, 0.2, 0.5, 2.0])


def test_startup_gate_accepts_creep_below_min():
    assert_creep_below_gate_min(v_creep_mps=0.15,
                                   band_values=[0.0, 0.2, 0.5, 2.0])


# --- CHK-2-25 alias blacklist ---

def test_alias_key_at_top_level_raises():
    with pytest.raises(AliasKeyFound, match="rot_occ_max_cells"):
        scan_config_for_alias({"rot_occ_max_cells": 12})


def test_alias_key_nested_raises():
    with pytest.raises(AliasKeyFound, match="fail_ticks"):
        scan_config_for_alias(
            {"rotation": {"clearance": {"fail_ticks": 3}}})


def test_alias_scan_clean_passes():
    scan_config_for_alias(
        {"rotation": {"clearance": {"recheck_ticks": 3,
                                        "rot_occ_max": 12}}})


# --- CHK-2-47 closed-set literal scan ---

def test_closed_set_scan_finds_hardcoded_string():
    src = '''
def foo():
    return "motion"
'''
    hits = check_source_for_closed_set_literals(src)
    # Line 3, token 'motion'
    assert any(h == (3, "motion") for h in hits)


def test_closed_set_scan_ignores_import_line():
    src = 'from xbrain.common.closed_sets import motion, ptz\n'
    hits = check_source_for_closed_set_literals(src)
    assert hits == []


def test_closed_set_scan_no_match_returns_empty():
    src = 'x = 42\n'
    hits = check_source_for_closed_set_literals(src)
    assert hits == []
