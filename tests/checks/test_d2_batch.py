"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_d2_batch.py
Brief: D-2 batch CFG-CF-9 + INF-DP-8 + INF-DP-10 tests

Description:
Refuse-to-boot milestone (CFG-CF-9): the three-section listing and,
since 2026-09-28, the conversion of a real assertion failure into it
plus the freeze entry point that prints it. Also: observation window
minimal publisher discipline + boot_fail JSONL append + three-state
BIT; orderly shutdown SYS-G gates + P1-last discipline + steady-sync
mode + PWR-S2 banner.

The two "variant guard" helpers this file used to cover
(refuse_code_default / safety_zero_still_fails_g) were deleted with
their tests -- see the comment mid-file for why each was a duplicate
of a check that already runs.
"""

from __future__ import annotations

import json

import pytest

from xbrain.boot.freeze.refuse_to_boot import (
    compose_stdout_lines,
    verdict,
    verdict_from_error,
)
from xbrain.common.errors import E_BUSY, E_CONFIG_INVALID, E_QOS_VIOLATION
from xbrain.common.errors.exceptions import XbrainError
from xbrain.p2_core.shutdown.orderly import (
    CLOUD_ACK_MAX_MS,
    DB_STEADY_SYNC_MODE,
    ShutdownProgress,
    ShutdownStep,
    any_cmd_vel_after_p1_exit,
    assert_pwr_s2_banner,
    check_gates,
    run_s1_cloud_ack,
    run_s6_db_checkpoint,
    run_s7_motion_zero_p1_last,
)
from xbrain.p5_gateway.minimal.observation_window import (
    FORBIDDEN_IN_MINIMAL,
    MINIMAL_MODE_PUBLISHERS,
    BitObservationState,
    BootFailRecord,
    MinimalModeSurfaceViolation,
    append_boot_fail_jsonl,
    assert_minimal_publisher_set,
    classify_bit_observation,
    read_boot_fail_jsonl,
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


# The two CFG-CF-9 "variant guard" helpers this file used to exercise
# (refuse_code_default / safety_zero_still_fails_g) were deleted on
# 2026-09-28 together with their tests. Both were runtime functions with
# zero callers that duplicated a check which already runs:
#   * "do not fall back to a code default" is a SOURCE rule, enforced by
#     scripts/lint/no_safety_default.py over the tree, not by a function
#     a freeze-time code path was supposed to remember to call;
#   * "safety filled with 0.0 must still redden" IS assertion G (SP-1 /
#     SP-5, xbrain/boot/freeze/assertions/g_safety_range.py), which is in
#     ASSERT_REGISTRY and therefore actually runs. The deleted helper
#     tested `value == 0.0` on any common.safety.* key, which is neither
#     SP-1 nor SP-5 -- a second, weaker opinion about the same rule.
# CLAUDE.md S9.3: a reserved hook nobody calls is removed at review.


def test_verdict_from_error_assertion_a_key_is_listed():
    """The wiring shape: an assertion A failure must come out of
    verdict_from_error as the 'unassigned key' section, exit 1.

    This is the half that makes the module's other tests mean something.
    compose_stdout_lines() was always green; what was missing until
    2026-09-28 was anything converting a real XbrainError into its
    arguments, so freeze printed a traceback instead of the listing."""
    exc = XbrainError(E_CONFIG_INVALID, "assertion A failed",
                      {"kind": "null_unassigned",
                       "key": "common.safety.t_lat_s"})
    v = verdict_from_error(exc)
    assert v.exit_code == 1
    # The ROW PREFIX, not the word "assertion A": the unclassified branch
    # echoes exc.args[0], which for this exception happens to contain the
    # words "assertion A" too. A mutant that dropped null_unassigned from
    # the A bucket survived the looser wording (measured 2026-09-28), so
    # the assertion pins what only the A bucket emits.
    assert "assertion A: keys unassigned (null placeholder)" in v.stdout_lines
    assert "  unassigned_key: common.safety.t_lat_s" in v.stdout_lines


def test_verdict_from_error_lists_every_key_when_the_raiser_had_them():
    """detail.keys (all of them) must produce one row EACH, not one row.

    10 S5.4.5 asks the refusal to list "unassigned key paths" -- plural.
    Until 2026-09-29 assertion A passed only nulls[0] even though it had
    computed the whole list to decide whether to fail at all, so an
    operator filling in one null re-ran the entire freeze to learn the
    next one. On the tree at the time that was dozens of rounds for a
    list that existed in full on the first run.

    mutant: drop the `keys` branch (back to detail.key only) -> the two
    later rows disappear and this goes red.
    """
    exc = XbrainError(E_CONFIG_INVALID, "assertion A failed",
                      {"kind": "null_unassigned",
                       "key": "common.calib.calib_rev",
                       "keys": ["common.calib.calib_rev",
                                "common.db.fence_db",
                                "common.zenoh.rt_endpoint"],
                       "null_count": 3})
    v = verdict_from_error(exc)
    assert v.exit_code == 1
    for k in ("common.calib.calib_rev", "common.db.fence_db",
              "common.zenoh.rt_endpoint"):
        assert ("  unassigned_key: %s" % k) in v.stdout_lines
    # Exactly three rows: a mutant that appended the first key again (or
    # that listed `key` alongside `keys`) would duplicate a row, and a
    # duplicated key path reads to an operator as two separate problems.
    rows = [ln for ln in v.stdout_lines if ln.startswith("  unassigned_key:")]
    assert len(rows) == 3


def test_verdict_from_error_falls_back_to_single_key():
    """A raiser that has only the first one keeps working unchanged.

    This is the compatibility half of the change above: no assertion is
    REQUIRED to start collecting, and the ones that genuinely cannot
    (their walk stops at the first violation) must not regress into an
    EMPTY listing -- which is the one outcome worse than a single row
    (see this module's "A TRAP WORTH NAMING").

    mutant: make the keys branch unconditional (drop the isinstance /
    non-empty guard) -> detail without `keys` yields no rows and this
    goes red.
    """
    exc = XbrainError(E_CONFIG_INVALID, "assertion A failed",
                      {"kind": "null_unassigned",
                       "key": "common.safety.t_lat_s"})
    v = verdict_from_error(exc)
    assert v.exit_code == 1
    assert "  unassigned_key: common.safety.t_lat_s" in v.stdout_lines


def test_verdict_from_error_assertion_j_uses_absolute_path():
    """A J failure lands in the missing-files section and carries
    detail.path (absolute, because J makes it absolute at raise time)."""
    exc = XbrainError(E_CONFIG_INVALID, "config root check failed",
                      {"kind": "config_file_missing",
                       "path": "/opt/xbrain_v6/configs/p1_motion.yaml"})
    v = verdict_from_error(exc)
    assert v.exit_code == 1
    # Row prefix again, for the reason spelled out on the A test above.
    assert "assertion J: config files missing" in v.stdout_lines
    assert ("  missing_file: /opt/xbrain_v6/configs/p1_motion.yaml"
            in v.stdout_lines)


def test_verdict_from_error_assertion_m_names_the_layer():
    """M rows must name BOTH key and layer -- "which key" without
    "which layer should have supplied it" is not actionable when six
    layers can legally carry it (10 S5.4.3)."""
    exc = XbrainError(E_CONFIG_INVALID, "assertion M failed",
                      {"kind": "required_key_missing",
                       "key": "common.motion.profiles.patrol",
                       "layer": "L1"})
    v = verdict_from_error(exc)
    assert v.exit_code == 1
    assert "assertion M: keys missing from required layer" in v.stdout_lines
    assert ("  missing_layer_key: common.motion.profiles.patrol (layer: L1)"
            in v.stdout_lines)


def test_verdict_from_error_unknown_kind_still_refuses_loudly():
    """The trap the module docstring names: a kind outside the three
    CFG-CF-9 buckets must NOT compose to an empty listing.

    Assertions B..S raise kinds that are deliberately not J/A/M shapes.
    If the bucket dispatch fell through to verdict([], [], []) the
    process would exit 0 -- a refusal that boots -- which is the exact
    fail-silent this whole module exists to prevent."""
    exc = XbrainError(E_QOS_VIOLATION, "assertion F failed",
                      {"kind": "qos_block_on_rt", "key": "rt/motion/cmd_vel"})
    v = verdict_from_error(exc)
    assert v.exit_code == 1
    assert v.stdout_lines, "an unmapped assertion must still print something"
    joined = "\n".join(v.stdout_lines)
    assert E_QOS_VIOLATION in joined
    assert "rt/motion/cmd_vel" in joined


def test_freeze_entrypoint_never_leaks_a_traceback(monkeypatch):
    """CFG-CF-9 (1)+(2) on the REAL tree, stated so it cannot rot.

    Deliberately NOT "exit code is 1": that assertion would go red the
    day configs/ is fully calibrated, and a door that reddens on success
    gets relaxed (CLAUDE.md S3.2 form 2). What must hold in BOTH worlds:
    main() returns an int rather than letting an XbrainError escape, and
    when it returns nonzero the operator got a listing on stdout.

    Red proof: delete the `except XbrainError` block in
    xbrain/boot/freeze/__main__.py and this raises instead of returning,
    which is exactly the behaviour that shipped before 2026-09-28.

    argv is patched rather than passed because main() takes no argv
    parameter -- it is a systemd entry point and argparse reads sys.argv
    directly. Adding a parameter for the test's benefit would put a
    second, test-only way to reach the parser."""
    import io
    import os
    import sys
    import tempfile
    from contextlib import redirect_stdout

    from xbrain.boot.freeze.__main__ import main

    repo = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    with tempfile.TemporaryDirectory() as resolved_root:
        monkeypatch.setattr(sys, "argv", [
            "xbrain.boot.freeze",
            "--config-root", os.path.join(repo, "configs"),
            "--resolved-root", resolved_root,
        ])
        buf = io.StringIO()
        with redirect_stdout(buf):
            rc = main()
    assert isinstance(rc, int)
    if rc != 0:
        assert buf.getvalue().strip(), (
            "freeze refused but printed nothing on stdout; 10 S5.4.5 "
            "requires the failing paths/keys to be listed")


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
