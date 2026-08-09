"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_g.py
Brief: BIZ-P3-20/21/22/26/29/23/24 lifecycle + failure + retention + teach + shutdown + yaml + meta

Description:
Batch G: closes out P3 with the remaining ops-facing modules --
estop freeze/unfreeze permission, six-row failure matrix mapping,
retention window ordering, WAL checkpoint skip in degraded mode,
tombstone hard-delete gate, teach 0.5m dedup, shutdown STOPPING
semantics, and the p3_task.yaml freeze assertions (A/B/C/J) with
the U71 disabled sentinel guard on assertion C.
"""

import pytest

from xbrain.p3_task.config.assertions import (
    FreezeAssertionFailure,
    check_a_no_residuals, check_b_no_alias_keys,
    check_c_retention_and_fence_relation, check_j_config_root,
    run_all_assertions,
)
from xbrain.p3_task.lifecycle.estop import EstopController
from xbrain.p3_task.lifecycle.failure import (
    UnknownFailureKind, classify,
)
from xbrain.p3_task.lifecycle.shutdown import ShutdownController
from xbrain.p3_task.lifecycle.teach import (
    TeachSample, dedup_run, should_keep,
)
from xbrain.p3_task.persistence.retention import (
    RetentionOrderViolation, RetentionWindows,
    should_checkpoint, tombstone_safe_to_hard_delete, validate_windows,
)


pytestmark = pytest.mark.no_device


# --- BIZ-P3-20 estop ---

def test_estop_freeze_blocks_scheduling():
    c = EstopController()
    c.freeze("battery_estop")
    assert c.scheduling_permitted() is False


def test_estop_freeze_idempotent_keeps_first_reason():
    c = EstopController()
    c.freeze("first")
    c.freeze("second")
    assert c.freeze_reason == "first"


def test_estop_unfreeze_rejects_wrong_source():
    """ES-3: only p2_operator may unfreeze; other sources refused."""
    c = EstopController()
    c.freeze("some_reason")
    with pytest.raises(PermissionError, match="not authorized"):
        c.unfreeze("some_other")


def test_estop_unfreeze_from_operator_clears_state():
    c = EstopController()
    c.freeze("some_reason")
    c.unfreeze("p2_operator")
    assert c.scheduling_permitted() is True


# --- BIZ-P3-21 failure matrix ---

def test_failure_process_crash_maps_to_f1():
    r = classify("process_crash")
    assert r.code == "F-1" and r.action == "restart_and_requeue"


def test_failure_db_corrupt_maps_to_f2():
    r = classify("db_corrupt")
    assert r.code == "F-2" and r.action == "refuse_start_wait_human"


def test_failure_disk_full_maps_to_f3():
    r = classify("disk_full")
    assert r.code == "F-3"


def test_failure_unknown_kind_raises():
    """Closed set (CLAUDE.md 3.5): no silent degrade path."""
    with pytest.raises(UnknownFailureKind):
        classify("halfway")


# --- BIZ-P3-22 retention ---

def test_retention_windows_ok_when_ascending():
    validate_windows(RetentionWindows(30, 90, 180))


def test_retention_windows_rejected_when_wrong_order():
    with pytest.raises(RetentionOrderViolation):
        validate_windows(RetentionWindows(180, 90, 30))


def test_retention_windows_equal_boundary_accepted():
    validate_windows(RetentionWindows(30, 30, 30))


def test_should_checkpoint_skipped_in_degraded():
    """When degraded, we do NOT checkpoint (avoid amplifying I/O)."""
    assert should_checkpoint(now_ms=1000, last_ms=0, interval_ms=100,
                                degraded=True) is False


def test_should_checkpoint_fires_at_interval():
    assert should_checkpoint(now_ms=100, last_ms=0, interval_ms=100,
                                degraded=False) is True


def test_tombstone_safe_only_when_no_pending():
    assert tombstone_safe_to_hard_delete(0) is True
    assert tombstone_safe_to_hard_delete(1) is False


# --- BIZ-P3-26 teach ---

def test_teach_first_sample_always_kept():
    s = TeachSample(0.0, 0.0, 0.0)
    assert should_keep(s, last_kept=None, dedup_min_dist_m=0.5) is True


def test_teach_close_sample_dropped():
    prev = TeachSample(0.0, 0.0, 0.0)
    new = TeachSample(0.2, 0.0, 0.0)
    assert should_keep(new, prev, dedup_min_dist_m=0.5) is False


def test_teach_far_sample_kept():
    prev = TeachSample(0.0, 0.0, 0.0)
    new = TeachSample(0.6, 0.0, 0.0)
    assert should_keep(new, prev, dedup_min_dist_m=0.5) is True


def test_teach_dedup_preserves_last():
    """dedup_run must always include the final sample so the
    recorded route ends where the operator ended."""
    stream = [
        TeachSample(0.0, 0.0, 0.0),
        TeachSample(0.1, 0.0, 0.0),
        TeachSample(0.2, 0.0, 0.0),
    ]
    out = dedup_run(stream, dedup_min_dist_m=1.0)
    assert out[0] == stream[0] and out[-1] == stream[-1]


# --- BIZ-P3-29 shutdown ---

def test_shutdown_request_blocks_admission():
    c = ShutdownController()
    assert c.can_admit_new_task() is True
    c.request()
    assert c.can_admit_new_task() is False


def test_shutdown_wait_window_needs_pending():
    c = ShutdownController()
    with pytest.raises(RuntimeError, match="cannot enter"):
        c.enter_wait_window()


def test_shutdown_wait_window_ok_when_pending():
    c = ShutdownController()
    c.request()
    c.enter_wait_window()
    assert c.wait_for_power_off is True


# --- BIZ-P3-23 freeze assertions ---

_GOOD_CONFIG = {
    "retention": {"keep_task_days": 30, "keep_progress_days": 90,
                    "keep_snapshot_days": 180},
    "recording": {"min_dist_m": 0.5},
    "fence": {"fence_close_tol_m": 1.0},
    "charge": {"low_batt_profile": "disabled"},
}


def test_assertion_a_rejects_null():
    bad = {"charge": {"soc_low": None}}
    with pytest.raises(FreezeAssertionFailure, match="explicit null"):
        check_a_no_residuals(bad)


def test_assertion_a_rejects_interpolation_leak():
    bad = {"charge": {"soc_low": "${common.soc_default}"}}
    with pytest.raises(FreezeAssertionFailure, match="interpolation"):
        check_a_no_residuals(bad)


def test_assertion_b_rejects_alias_key():
    """CHK-2-26: enforce_ordering is on the alias blacklist."""
    bad = {"retention": {"enforce_ordering": False, "keep_task_days": 30}}
    with pytest.raises(FreezeAssertionFailure, match="enforce_ordering"):
        check_b_no_alias_keys(bad)


def test_assertion_c_rejects_wrong_order():
    bad = dict(_GOOD_CONFIG)
    bad["retention"] = {"keep_task_days": 180, "keep_progress_days": 90,
                          "keep_snapshot_days": 30}
    with pytest.raises(FreezeAssertionFailure, match="ascending"):
        check_c_retention_and_fence_relation(bad)


def test_assertion_c_disabled_sentinel_skips_fence_clause():
    """U71: with low_batt_profile='disabled', the fence sub-clause
    of C is not evaluated. Missing fence tol becomes tolerable."""
    cfg = {
        "retention": {"keep_task_days": 30, "keep_progress_days": 90,
                        "keep_snapshot_days": 180},
        "charge": {"low_batt_profile": "disabled"},
    }
    check_c_retention_and_fence_relation(cfg)   # no raise


def test_assertion_c_active_profile_needs_fence_relation():
    """When low_batt_profile is NOT disabled, missing
    fence_close_tol_m -> raise."""
    cfg = {
        "retention": {"keep_task_days": 30, "keep_progress_days": 90,
                        "keep_snapshot_days": 180},
        "recording": {"min_dist_m": 0.5},
        "charge": {"low_batt_profile": "active"},
    }
    with pytest.raises(FreezeAssertionFailure):
        check_c_retention_and_fence_relation(cfg)


def test_assertion_j_rejects_wrong_root():
    with pytest.raises(FreezeAssertionFailure, match="not under"):
        check_j_config_root("/etc/xbrain/p3_task.yaml")


def test_assertion_j_accepts_correct_root():
    check_j_config_root("/opt/xbrain_v6/configs/p3_task.yaml")


def test_run_all_assertions_on_good_config():
    run_all_assertions(_GOOD_CONFIG, "/opt/xbrain_v6/configs/p3_task.yaml")
