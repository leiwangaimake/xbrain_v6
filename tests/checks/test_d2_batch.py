"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_d2_batch.py
Brief: D-2 batch CFG-CF-9 + INF-DP-8 + INF-DP-10 tests

Description:
Refuse-to-boot milestone (variant-a: no code default fallback;
variant-b: safety=0.0 still fails G); observation window minimal
publisher discipline + boot_fail JSONL append + three-state BIT;
orderly shutdown SYS-G gates + P1-last discipline + steady-sync
mode + PWR-S2 banner.
"""

from __future__ import annotations

import json
import os

import pytest

from xbrain.boot.freeze.refuse_to_boot import (
    DefaultFallbackForbidden, FreezeVerdict,
    compose_stdout_lines, refuse_code_default,
    safety_zero_still_fails_g, verdict,
)
from xbrain.common.errors import E_BUSY
from xbrain.p2_core.shutdown.orderly import (
    CLOUD_ACK_MAX_MS, DB_STEADY_SYNC_MODE, ShutdownProgress,
    ShutdownStep, any_cmd_vel_after_p1_exit, assert_pwr_s2_banner,
    check_gates, run_s1_cloud_ack, run_s6_db_checkpoint,
    run_s7_motion_zero_p1_last,
)
from xbrain.p5_gateway.minimal.observation_window import (
    BitObservationState, BootFailRecord, FORBIDDEN_IN_MINIMAL,
    MINIMAL_MODE_PUBLISHERS, MinimalModeSurfaceViolation,
    append_boot_fail_jsonl, assert_minimal_publisher_set,
    classify_bit_observation, read_boot_fail_jsonl,
    transpose_to_event,
)


pytestmark = pytest.mark.no_device


# ---------- CFG-CF-9 refuse to boot ----------

def test_verdict_all_clear_exit_zero():
    v = verdict(missing_files=[], unassigned_keys=[],
                  missing_layer_keys=[])
    assert v.exit_code == 0 and v.stdout_lines == []


def test_verdict_any_missing_file_exits_nonzero():
    v = verdict(missing_files=["/opt/xbrain_v6/configs/p1_motion.yaml"],
                  unassigned_keys=[], missing_layer_keys=[])
    assert v.exit_code == 1
    assert any("missing_file" in ln for ln in v.stdout_lines)
    assert any("assertion J" in ln for ln in v.stdout_lines)


def test_verdict_any_unassigned_key_exits_nonzero():
    v = verdict(missing_files=[],
                  unassigned_keys=["common.spec.max_vx_mps"],
                  missing_layer_keys=[])
    assert v.exit_code == 1
    assert any("common.spec.max_vx_mps" in ln for ln in v.stdout_lines)
    assert any("assertion A" in ln for ln in v.stdout_lines)


def test_verdict_missing_layer_key_lists_it():
    v = verdict(missing_files=[], unassigned_keys=[],
                  missing_layer_keys=["common.motion.profiles.patrol"])
    assert v.exit_code == 1
    assert any("assertion M" in ln for ln in v.stdout_lines)


def test_compose_stdout_lines_sorted_stable():
    """Sorted output = diff-stable between runs."""
    lines = compose_stdout_lines(
        missing_files=["/z.yaml", "/a.yaml"],
        unassigned_keys=[], missing_layer_keys=[])
    # /a.yaml appears before /z.yaml
    a_idx = next(i for i, ln in enumerate(lines) if "/a.yaml" in ln)
    z_idx = next(i for i, ln in enumerate(lines) if "/z.yaml" in ln)
    assert a_idx < z_idx


def test_refuse_code_default_variant_a():
    """CFG-CF-9 variant 1: cannot fall back to a code default."""
    with pytest.raises(DefaultFallbackForbidden, match="unassigned"):
        refuse_code_default("common.spec.max_vx_mps")


def test_safety_zero_still_fails_g_variant_b():
    """CFG-CF-9 variant 2: setting safety=0.0 to bypass assertion A
    doesn't work -- SP-5 still refuses."""
    with pytest.raises(DefaultFallbackForbidden, match="0.0 refused"):
        safety_zero_still_fails_g("common.safety.brake.a_mps2", 0.0)


def test_safety_nonzero_ok():
    """Positive safety value is fine; the guard only fires on 0.0."""
    safety_zero_still_fails_g("common.safety.brake.a_mps2", 2.5)


def test_non_safety_zero_untouched():
    """Non-safety zero values are legal (e.g. count = 0)."""
    safety_zero_still_fails_g("common.priority.task.auto", 0.0)


# ---------- INF-DP-8 minimal-mode publisher discipline ----------

def test_minimal_publisher_set_ok():
    """Sanctioned minimal-mode publishers pass."""
    assert_minimal_publisher_set({"state/link", "state/boot_fail"})


def test_minimal_publisher_empty_ok():
    """Publishing nothing is legal (still in minimal mode)."""
    assert_minimal_publisher_set(set())


def test_minimal_publisher_cmd_motion_rejected():
    """W-1 variant guard: cmd/motion/factor MUST NOT fire from
    minimal mode."""
    with pytest.raises(MinimalModeSurfaceViolation, match="forbidden"):
        assert_minimal_publisher_set({"state/link", "cmd/motion/factor"})


def test_minimal_publisher_any_cmd_motion_key_rejected():
    for key in ("cmd/motion/cmd_vel", "cmd/motion/behavior",
                  "cmd/motion/route", "cmd/ptz", "cmd/payload"):
        with pytest.raises(MinimalModeSurfaceViolation):
            assert_minimal_publisher_set({key})


def test_minimal_publisher_unknown_extras_rejected():
    """Even 'harmless-looking' extras fail; only sanctioned keys."""
    with pytest.raises(MinimalModeSurfaceViolation, match="unknown keys"):
        assert_minimal_publisher_set({"state/link", "state/some_random"})


def test_minimal_mode_publisher_set_matches_expectation():
    assert MINIMAL_MODE_PUBLISHERS == frozenset({
        "state/link", "state/boot_fail", "event/warn/boot",
    })


def test_forbidden_in_minimal_includes_all_cmd_motion():
    for key in ("cmd/motion/cmd_vel", "cmd/motion/factor",
                  "cmd/motion/behavior", "cmd/motion/route"):
        assert key in FORBIDDEN_IN_MINIMAL


# ---------- INF-DP-8 boot_fail JSONL append discipline ----------

def test_jsonl_append_then_read(tmp_path):
    path = str(tmp_path / "boot_fail.jsonl")
    r1 = BootFailRecord(stage="stage_c", code="E_CONFIG_INVALID",
                          boot_id="b1", message="m1")
    r2 = BootFailRecord(stage="stage_c", code="E_STORAGE_CORRUPT",
                          boot_id="b2", message="m2")
    append_boot_fail_jsonl(path, r1)
    append_boot_fail_jsonl(path, r2)
    got = read_boot_fail_jsonl(path)
    assert got == [r1, r2]


def test_jsonl_append_never_overwrites(tmp_path):
    """W-2 variant guard: rewrite mode would lose earlier records."""
    path = str(tmp_path / "boot_fail.jsonl")
    for i in range(5):
        append_boot_fail_jsonl(path, BootFailRecord(
            stage="stage_c", code="E", boot_id=f"b{i}",
            message=f"m{i}"))
    with open(path) as fh:
        lines = fh.readlines()
    assert len(lines) == 5
    # Each line parses cleanly
    for ln in lines:
        json.loads(ln)


def test_jsonl_read_missing_file_empty():
    """No file yet -> empty list, not exception."""
    assert read_boot_fail_jsonl("/nonexistent/boot_fail.jsonl") == []


def test_transpose_to_event_carries_four_fields():
    rec = BootFailRecord(stage="stage_c", code="E_CONFIG_INVALID",
                          boot_id="abc", message="msg")
    ev = transpose_to_event(rec)
    for k in ("stage", "code", "boot_id", "message"):
        assert k in ev["detail"]
    assert ev["kind"] == "event/fault/system"


# ---------- INF-DP-8 W-3 three-state BIT ----------

def test_bit_never_ran():
    """Probe hasn't scheduled BIT yet."""
    r = classify_bit_observation(
        bit_scheduled=False, bit_result=None, result_indicates_pass=False)
    assert r == BitObservationState.NEVER_RAN


def test_bit_ran_no_result():
    """BIT process crashed before producing a result."""
    r = classify_bit_observation(
        bit_scheduled=True, bit_result=None, result_indicates_pass=False)
    assert r == BitObservationState.RAN_NO_RESULT


def test_bit_ran_failed():
    r = classify_bit_observation(
        bit_scheduled=True, bit_result="gpu:fail",
        result_indicates_pass=False)
    assert r == BitObservationState.RAN_FAILED


def test_bit_ran_passed():
    r = classify_bit_observation(
        bit_scheduled=True, bit_result="all_ok",
        result_indicates_pass=True)
    assert r == BitObservationState.RAN_PASSED


def test_bit_three_failure_states_distinct():
    """W-3 variant guard: merging never_ran + ran_no_result into
    'unknown' loses operator context."""
    a = classify_bit_observation(False, None, False)
    b = classify_bit_observation(True, None, False)
    c = classify_bit_observation(True, "fail", False)
    assert a != b != c != a


# ---------- INF-DP-10 SYS-G gates ----------

def test_sysg1_estop_active_refuses():
    v = check_gates(estop_active=True, alarm_active=False,
                     teach_recording=False, charging_critical=False)
    assert not v.accepted and v.code == E_BUSY
    assert "SYS-G1" in v.reason


def test_sysg2_alarm_active_refuses():
    v = check_gates(False, True, False, False)
    assert v.code == E_BUSY and "SYS-G2" in v.reason


def test_sysg3_teach_recording_refuses_no_exemption():
    """v0.7.7 ruling: SYS-G3 does not have an exemption path."""
    v = check_gates(False, False, True, False)
    assert v.code == E_BUSY and "SYS-G3" in v.reason


def test_sysg4_charging_critical_refuses():
    v = check_gates(False, False, False, True)
    assert v.code == E_BUSY and "SYS-G4" in v.reason


def test_all_gates_clear_shutdown_permitted():
    v = check_gates(False, False, False, False)
    assert v.accepted


def test_sysg_priority_first_hit_wins():
    """When multiple gates trip, SYS-G1 is reported (fixed order)."""
    v = check_gates(True, True, True, True)
    assert "SYS-G1" in v.reason


# ---------- INF-DP-10 S1 cloud-ack timeout continues ----------

def test_s1_cloud_ack_timeout_continues():
    """Variant: timeout should NOT abort shutdown; only add to
    skipped[]."""
    p = ShutdownProgress()
    run_s1_cloud_ack(p, cloud_ack_arrived=False,
                      wait_ms=CLOUD_ACK_MAX_MS)
    assert "cloud_ack" in p.skipped
    assert ShutdownStep.S1_CLOUD_ACK.value in p.steps_completed


def test_s1_cloud_ack_arrived_no_skip():
    p = ShutdownProgress()
    run_s1_cloud_ack(p, cloud_ack_arrived=True, wait_ms=1000)
    assert "cloud_ack" not in p.skipped
    assert ShutdownStep.S1_CLOUD_ACK.value in p.steps_completed


def test_s1_cloud_ack_timeout_cap_matches_spec():
    assert CLOUD_ACK_MAX_MS == 5_000


# ---------- INF-DP-10 S6 steady-state sync mode ----------

def test_s6_steady_state_normal_ok():
    p = ShutdownProgress()
    run_s6_db_checkpoint(p, steady_state_sync_mode="NORMAL")
    assert ShutdownStep.S6_DB_CHECKPOINT.value in p.steps_completed


def test_s6_steady_state_full_rejected():
    """Variant: leaving synchronous=FULL as steady state is wrong."""
    p = ShutdownProgress()
    with pytest.raises(ValueError, match="NORMAL"):
        run_s6_db_checkpoint(p, steady_state_sync_mode="FULL")


def test_s6_db_steady_mode_matches_spec():
    assert DB_STEADY_SYNC_MODE == "NORMAL"


# ---------- INF-DP-10 S7 P1-last discipline ----------

def test_s7_p1_exits_after_zero_frame():
    """PWR-S1: zero cmd_vel emitted BEFORE P1 exit."""
    p = ShutdownProgress()
    run_s7_motion_zero_p1_last(p, zero_cmd_mono_ms=1000,
                                 p1_exit_mono_ms=1010)
    # Last cmd_vel frame is a zero.
    last = p.cmd_vel_frames[-1]
    assert last[1] == 0.0 and last[2] == 0.0 and last[3] == 0.0
    # P1 exited after the zero frame.
    assert not any_cmd_vel_after_p1_exit(p)


def test_s7_p1_exit_before_zero_refused():
    """P1 exiting BEFORE emitting the final zero is a defect."""
    p = ShutdownProgress()
    with pytest.raises(ValueError, match="zero first, exit last"):
        run_s7_motion_zero_p1_last(p, zero_cmd_mono_ms=2000,
                                     p1_exit_mono_ms=1000)


def test_any_cmd_vel_after_p1_exit_detected():
    """PWR-S1: no frames may be emitted AFTER P1 exit."""
    p = ShutdownProgress()
    p.cmd_vel_frames.append((1000, 0.0, 0.0, 0.0))
    p.p1_exited_mono_ms = 1010
    # Manually append a stray frame.
    p.cmd_vel_frames.append((1500, 0.0, 0.0, 0.0))
    assert any_cmd_vel_after_p1_exit(p) is True


# ---------- INF-DP-10 PWR-S2 banner ----------

def test_pwr_s2_hmi_banner_requires_unlock_wording():
    p = ShutdownProgress()
    p.hmi_banner = "next boot needs one unlock confirm"
    p.ack_detail = {"note": "next boot needs unlock"}
    assert_pwr_s2_banner(p)   # no raise


def test_pwr_s2_hmi_banner_chinese_ok():
    p = ShutdownProgress()
    p.hmi_banner = "下次开机需要一次解锁确认"
    p.ack_detail = {"note": "解锁确认"}
    assert_pwr_s2_banner(p)


def test_pwr_s2_banner_missing_rejected():
    """Variant: silent omission of the unlock reminder rejected."""
    p = ShutdownProgress()
    p.hmi_banner = "shutdown complete"
    p.ack_detail = {"ok": True}
    with pytest.raises(ValueError, match="unlock"):
        assert_pwr_s2_banner(p)


def test_pwr_s2_ack_detail_missing_rejected():
    p = ShutdownProgress()
    p.hmi_banner = "next boot needs unlock"
    p.ack_detail = {"ok": True}
    with pytest.raises(ValueError, match="unlock"):
        assert_pwr_s2_banner(p)
