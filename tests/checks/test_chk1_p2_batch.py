"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chk1_p2_batch.py
Brief: CHK-1-07/14/33 P2 severe items batch

Description:
Three severe P2 items, each with strong variant coverage per the
CHK-1-* criterion mutation lists.

CHK-1-07 tilt soft-limit
  * boundary values (-90, +30) accepted (inclusive)
  * out-of-range REJECTS (no clamp -- variant 3 guard)
  * TiltLimits inverted range refused at construction
  * detail carries actual + min + max + kind

CHK-1-14 σ hysteresis
  * h = k_h*σ + h_min three-point cross-check (values computed live)
  * entering fires at dist_inside > -h
  * exiting fires at dist_inside < -h
  * exit_persist_s < 4*enter_persist_s refused at construction
  * L3 downgrade: zone_rules_active returns False
  * event detail carries heading_src + zone_rules_disabled

CHK-1-33 hold_ms dead-man
  * fresh nudge refreshes deadline (long pulse keeps moving)
  * silence past deadline fires Stop EXACTLY ONCE per run
  * hold_ms zero at construction refused (fail-silent form)
  * mid-pulse nudge extension keeps armed
"""

from __future__ import annotations

import pytest

from xbrain.common.errors import E_CONFIG_INVALID
from xbrain.p2_core.ptz.hold_ms_renewal import (
    DeadmanState, HoldMsConfigError, create, on_nudge, tick,
)
from xbrain.p2_core.ptz.tilt_limit import (
    TiltLimits, TiltOutOfLimits, check_tilt,
)
from xbrain.p2_core.suspicion.hysteresis import (
    DEGRADE_LEVELS, HysteresisConfig, HysteresisConfigError,
    band_width_m, entering_threshold, event_detail_for_level,
    exiting_threshold, is_entering, is_exiting, zone_rules_active,
)


pytestmark = pytest.mark.no_device


# --- CHK-1-07 tilt limits ---

LIM = TiltLimits(min_deg=-90.0, max_deg=30.0)


def test_tilt_within_range_ok():
    check_tilt(-45.0, LIM)   # no raise
    check_tilt(0.0, LIM)
    check_tilt(20.0, LIM)


def test_tilt_at_min_boundary_ok():
    check_tilt(-90.0, LIM)   # inclusive


def test_tilt_at_max_boundary_ok():
    check_tilt(30.0, LIM)


def test_tilt_below_min_rejected():
    with pytest.raises(TiltOutOfLimits) as excinfo:
        check_tilt(-90.5, LIM)
    e = excinfo.value
    assert e.code == E_CONFIG_INVALID
    assert e.detail["kind"] == "ptz_tilt_out_of_range"
    assert e.detail["actual_deg"] == -90.5
    assert e.detail["min_deg"] == -90.0
    assert e.detail["max_deg"] == 30.0


def test_tilt_above_max_rejected():
    """CHK-1-07 variant 2 guard: +60 (over +30 upper) MUST be
    rejected, not clamped to something 'safe'."""
    with pytest.raises(TiltOutOfLimits):
        check_tilt(60.0, LIM)


def test_tilt_limits_inverted_refused():
    """min >= max would refuse every command silently."""
    with pytest.raises(ValueError, match="min_deg"):
        TiltLimits(min_deg=10.0, max_deg=-5.0)


def test_tilt_limits_equal_refused():
    with pytest.raises(ValueError):
        TiltLimits(min_deg=0.0, max_deg=0.0)


# --- CHK-1-14 hysteresis ---

CFG_STD = HysteresisConfig(k_h=2.0, h_min_m=0.3,
                             enter_persist_s=0.5, exit_persist_s=2.5)


def test_band_width_three_point_cross_check():
    """Formula h = k_h*σ + h_min. Three σ values, three h values,
    computed LIVE (not hardcoded in a comment per CLAUDE.md 3.7)."""
    assert band_width_m(wpos_sigma_m=0.05, cfg=CFG_STD) == \
        pytest.approx(2.0 * 0.05 + 0.3)
    assert band_width_m(0.17, CFG_STD) == pytest.approx(2.0 * 0.17 + 0.3)
    assert band_width_m(0.50, CFG_STD) == pytest.approx(2.0 * 0.50 + 0.3)


def test_hysteresis_entering_gate_direction():
    """CHK-1-14 direction: enter fires when dist_inside > -h.
    Target 0.1 m OUTSIDE region (dist_inside = -0.1) with σ=0.05:
    -h = -0.4, and -0.1 > -0.4 -> enter=True."""
    assert is_entering(dist_inside_m=-0.1, wpos_sigma_m=0.05,
                         cfg=CFG_STD) is True


def test_hysteresis_entering_not_yet_fires_far_outside():
    """Target 1.0 m outside (dist_inside = -1.0), σ=0.05 -> h=0.4.
    -h = -0.4, and -1.0 > -0.4 is False -> not entering."""
    assert is_entering(dist_inside_m=-1.0, wpos_sigma_m=0.05,
                         cfg=CFG_STD) is False


def test_hysteresis_exiting_gate_direction():
    """Exit fires when dist_inside < -h. Target 1.0 m outside
    (dist_inside=-1.0), σ=0.05 -> -h=-0.4; -1.0 < -0.4 -> True."""
    assert is_exiting(dist_inside_m=-1.0, wpos_sigma_m=0.05,
                        cfg=CFG_STD) is True


def test_hysteresis_thresholds_are_symmetric_numerically():
    """Both thresholds return the same NUMERIC value -h; the
    SEMANTIC difference is in the crossing DIRECTION (enter uses
    > , exit uses <). This is the 'symmetric ± h' bug guard."""
    assert entering_threshold(CFG_STD, 0.1) == exiting_threshold(CFG_STD, 0.1)


def test_hysteresis_config_exit_persist_ratio_enforced():
    """exit_persist_s must be >= 4 * enter_persist_s (asymmetric
    hysteresis discipline)."""
    with pytest.raises(HysteresisConfigError, match="exit_persist_s"):
        HysteresisConfig(k_h=2.0, h_min_m=0.3,
                          enter_persist_s=1.0, exit_persist_s=3.0)


def test_hysteresis_zero_h_min_refused():
    """h_min=0 = chatter across boundary."""
    with pytest.raises(HysteresisConfigError, match="h_min_m"):
        HysteresisConfig(k_h=2.0, h_min_m=0.0,
                          enter_persist_s=0.5, exit_persist_s=2.5)


def test_hysteresis_negative_k_h_refused():
    with pytest.raises(HysteresisConfigError, match="k_h"):
        HysteresisConfig(k_h=-1.0, h_min_m=0.3,
                          enter_persist_s=0.5, exit_persist_s=2.5)


# --- CHK-1-14 L1/L2/L3 downgrade ---

def test_degrade_l3_disables_zone_rules():
    assert zone_rules_active("L3") is False


def test_degrade_l1_l2_keep_zone_rules():
    assert zone_rules_active("L1") is True
    assert zone_rules_active("L2") is True


def test_degrade_unknown_level_raises():
    with pytest.raises(HysteresisConfigError, match="unknown degrade level"):
        zone_rules_active("L4")


def test_event_detail_l3_carries_zone_disabled_true():
    d = event_detail_for_level("L3", heading_src="dead_reckon")
    assert d["heading_src"] == "dead_reckon"
    assert d["zone_rules_disabled"] is True


def test_event_detail_l1_carries_zone_disabled_false():
    d = event_detail_for_level("L1", heading_src="rtk_fixed")
    assert d["zone_rules_disabled"] is False


def test_event_detail_both_fields_present():
    """Missing either field leaves the operator without regime
    context. Guard both are populated."""
    for level in DEGRADE_LEVELS:
        d = event_detail_for_level(level, heading_src="rtk")
        assert "heading_src" in d and "zone_rules_disabled" in d


# --- CHK-1-33 hold_ms dead-man ---

def test_hold_ms_zero_refused():
    """Zero hold_ms = no dead-man = uncontrolled pan on link drop."""
    with pytest.raises(HoldMsConfigError, match="fail-silent"):
        create(hold_ms=0)


def test_hold_ms_negative_refused():
    with pytest.raises(HoldMsConfigError):
        create(hold_ms=-100)


def test_deadman_starts_disarmed():
    state = create(hold_ms=800)
    assert state.armed is False
    # Ticking before any nudge fires no Stop.
    assert tick(state, now_mono_ms=1000) is False


def test_nudge_arms_and_extends_deadline():
    state = create(hold_ms=800)
    on_nudge(state, now_mono_ms=0)
    assert state.armed and state.deadline_mono_ms == 800


def test_long_pulse_continuous_motion_zero_stops():
    """CHK-1-33 variant guard: pulse_ms=5000 keeps motion going for
    5 seconds. If we nudge every 200ms (well within hold_ms=800),
    the deadman never fires Stop."""
    state = create(hold_ms=800)
    stop_count = 0
    for now_ms in range(0, 5000, 200):
        on_nudge(state, now_mono_ms=now_ms)
        if tick(state, now_mono_ms=now_ms):
            stop_count += 1
    # At the last tick t=4800, deadline=5600, no stop.
    assert stop_count == 0
    assert state.stops_emitted == 0


def test_silence_after_pulse_fires_stop_once():
    """Once nudges stop, deadman must fire EXACTLY ONE Stop
    within hold_ms."""
    state = create(hold_ms=800)
    on_nudge(state, now_mono_ms=0)
    on_nudge(state, now_mono_ms=200)
    on_nudge(state, now_mono_ms=400)
    # Silence starts at t=400; last deadline was 400+800=1200.
    # Tick before deadline -> no fire.
    assert tick(state, now_mono_ms=1000) is False
    # Tick at deadline -> fire once.
    assert tick(state, now_mono_ms=1200) is True
    # Additional ticks well past deadline -> no more fires (exactly once).
    for later in (1300, 1500, 2000, 3000):
        assert tick(state, now_mono_ms=later) is False
    assert state.stops_emitted == 1


def test_new_nudge_after_stop_rearms():
    """After the Stop fires, a fresh nudge must ARM again."""
    state = create(hold_ms=800)
    on_nudge(state, now_mono_ms=0)
    tick(state, now_mono_ms=1000)   # fires Stop
    assert state.armed is False
    on_nudge(state, now_mono_ms=2000)
    assert state.armed
    assert tick(state, now_mono_ms=2900) is True   # deadline at 2800; fired
    assert state.stops_emitted == 2
