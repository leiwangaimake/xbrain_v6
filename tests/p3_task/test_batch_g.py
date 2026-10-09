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
    check_a_no_residuals,
    check_b_no_alias_keys,
    check_c_fence_relation,
    check_j_config_root,
    run_all_assertions,
)
from xbrain.p3_task.lifecycle.estop import EstopController
from xbrain.p3_task.lifecycle.failure import (
    UnknownFailureKind,
    classify,
)
from xbrain.p3_task.lifecycle.shutdown import ShutdownController
from xbrain.p3_task.persistence.retention import (
    should_checkpoint,
    tombstone_safe_to_hard_delete,
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


# ===========================================================================
# BIZ-P3-22 retention -- THREE CASES REMOVED 2026-10-09 (RT-2)
# ===========================================================================
#
# Removed: test_retention_windows_ok_when_ascending /
# _rejected_when_wrong_order / _equal_boundary_accepted, plus the
# RetentionWindows / RetentionOrderViolation / validate_windows imports.
#
# They tested a three-window model (keep_task_days / keep_progress_days /
# keep_snapshot_days) whose three key names exist NOWHERE in the fourteen
# volumes. See xbrain/p3_task/persistence/retention.py's header for the
# full reasoning; the short version is that the model was a misreading of
# common.retention, whose three day counts are tasks / events / commands.
#
# *** THE MISREADING IS VISIBLE IN THE DELETED CASES THEMSELVES, which is
# why this block says so rather than just recording a deletion: the first
# case called validate_windows(RetentionWindows(30, 90, 180)). Those are
# the common.retention numbers -- tasks 30, events 90, commands 180. The
# author took the triple from the right place and attached the wrong three
# names to it, and 30 <= 90 <= 180 happens to be ascending, so every case
# passed. A green criterion over an invented model is indistinguishable
# from a correct one until somebody greps the key names.
#
# Nothing replaces them: the ordering they were checking belongs to the
# freeze line (10 S5.4.4 assertion C, implemented in
# xbrain/boot/freeze/assertions/c_cross_file.py and covered by
# tests/boot/freeze/test_b_c_d.py), and 17 S10 v0.7.3 says verbatim it is
# "不由任何一册的运行期代码保证".


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

# The four ENU teach-dedup cases that lived here were REMOVED on 2026-08-20
# with the module they tested (lifecycle/teach.py). They de-duplicated samples
# in ENU metres, and the geo model moved to WGS84 earlier the same day -- so
# the recording path would have needed a projection step whose failure mode is
# an ENU metre landing in a latitude column. Sampling now happens in WGS84 all
# the way through (xbrain/p3_task/teach/sampling.py), which removes the
# conversion rather than guarding it; the replacement cases are in
# tests/p3_task/test_teach_core.py.

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

#: A hand-built config for the four sub-assertions.
#:
#: *** WHAT THIS FIXTURE IS NOT: configs/p3_task.yaml. That file has exactly
#: ONE top-level key today -- `geo` -- so none of the sections these
#: assertions read (`charge`, `recording`, `fence`) exists in it. This dict
#: is therefore NOT a sample of the real file; it is the shape 15 S12
#: specifies, used to exercise the checks. A reader who takes it for the
#: real file would conclude p3_task.yaml is populated. Registered in
#: docs/NEXT.md as RT-1.
#:
#: The `retention` block that used to be here is gone (2026-10-09, RT-2):
#: its three keys were invented, and the check that read them has been
#: removed. See xbrain/p3_task/persistence/retention.py's header.
_GOOD_CONFIG = {
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


# One more case removed here on 2026-10-09 (RT-2): it fed a descending
# retention triple to the check and expected a refusal. Both the triple
# and the check are gone -- the ordering belongs to freeze-line
# assertion C (tests/boot/freeze/test_b_c_d.py covers it, including the
# 15 TC-21 case that the real key-name fix made reachable).

def test_assertion_c_disabled_sentinel_skips_fence_clause():
    """U71: with low_batt_profile='disabled', the fence sub-clause
    of C is not evaluated. Missing fence tol becomes tolerable."""
    cfg = {
        "retention": {"keep_task_days": 30, "keep_progress_days": 90,
                        "keep_snapshot_days": 180},
        "charge": {"low_batt_profile": "disabled"},
    }
    check_c_fence_relation(cfg)   # no raise


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
        check_c_fence_relation(cfg)


def test_assertion_j_rejects_wrong_root():
    with pytest.raises(FreezeAssertionFailure, match="not under"):
        check_j_config_root("/etc/xbrain/p3_task.yaml")


def test_assertion_j_accepts_correct_root():
    check_j_config_root("/opt/xbrain_v6/configs/p3_task.yaml")


def test_run_all_assertions_on_good_config():
    run_all_assertions(_GOOD_CONFIG, "/opt/xbrain_v6/configs/p3_task.yaml")
