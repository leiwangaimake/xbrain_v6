"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_check.py
Brief: freeze --check -- drift between the recorded snapshot and the sources

Description:
The question this guards is the one an operator actually asks: "I edited
configs/common.yaml on a running machine; is the system using it?" The answer
is no, by design (10 S5.4.1: the freeze expands references once and every
process reads the snapshot), and the hazard is that nothing makes the answer
visible. The structural gates all key on a reboot -- Requires= on the unit,
/run being tmpfs, the MANIFEST.boot_id refusal -- and an edit made without a
reboot passes all three while changing nothing.

Each case here pairs a specific edit with the specific finding it must
produce, because the finding kinds are not interchangeable: an operator who
sees snapshot_tampered has to move an edit into configs/, and one who sees
common_digest_mismatch has to restart the stack. A test suite that only
asserted "some drift was reported" would pass for an implementation that
emitted one kind for everything.

The case that keeps the rest honest is test_no_drift_on_an_untouched_tree: if
--check reported drift against a tree nobody touched, every other case here
would pass for the wrong reason, and the tool would be ignored in the field
within a week.
"""

import json
import os

import yaml

from tests.boot.freeze.test_registry import _scaffold_config_ctx
from xbrain.boot.freeze.check import FINDING_KINDS, Finding, check_freeze
from xbrain.boot.freeze.pipeline import run_freeze

BOOT = "check-test-boot-id"


def _freeze_into(ctx):
    """Run a real freeze into the scaffold's resolved_root (the 'recorded'
    state every case below then compares against)."""
    return run_freeze(
        boot_id=BOOT,
        config_root=ctx["config_root"],
        config_root_overridden=False,
        resolved_root=ctx["resolved_root"],
        context={"skip_files": list(ctx.get("skip_files", []))},
    )


def _check(ctx, boot_id=BOOT):
    return check_freeze(
        config_root=ctx["config_root"],
        resolved_root=ctx["resolved_root"],
        boot_id=boot_id,
        context={"skip_files": list(ctx.get("skip_files", []))},
    )


def _kinds(findings):
    return sorted(f.kind for f in findings)


def _scaffold_with_proc(tmp_path):
    """A scaffold that materialises at least one process, so the snapshot
    cases have a snapshot to talk about."""
    ctx = _scaffold_config_ctx(tmp_path)
    open(os.path.join(ctx["config_root"], "p2_core.yaml"), "w",
         encoding="utf-8").write(
        yaml.safe_dump({"p2_core": {"tick_hz": 20}}, allow_unicode=True))
    return ctx


# --------------------------------------------------------------------------
# The baseline: silence means silence
# --------------------------------------------------------------------------

def test_no_drift_on_an_untouched_tree(tmp_path):
    """Freeze, change nothing, check -> empty.

    Mutant: compare the fresh MANIFEST's gen_ts (a wall clock) as well -> red,
    because every run would report drift and the tool would become noise.
    """
    ctx = _scaffold_with_proc(tmp_path)
    _freeze_into(ctx)
    assert _check(ctx) == []


def test_check_writes_nothing_into_the_resolved_root(tmp_path):
    """--check must not refresh the snapshot it is reporting on.

    A --check that wrote would change the configuration under processes that
    had already read it -- some old, some new, which is the outcome 10 S5.4.1
    exists to prevent and is worse than a stale but consistent set.

    Mutant: point check_freeze's inner run_freeze at resolved_root instead of
    a temp dir -> the MANIFEST mtime and the snapshot bytes move -> red.
    """
    ctx = _scaffold_with_proc(tmp_path)
    _freeze_into(ctx)
    root = ctx["resolved_root"]
    before = {name: open(os.path.join(root, name), "rb").read()
              for name in sorted(os.listdir(root))}
    stats = {name: os.stat(os.path.join(root, name)).st_mtime_ns
             for name in before}
    # Edit a source so the check has something to report -- a no-drift check
    # could pass this case by doing nothing at all.
    with open(os.path.join(ctx["config_root"], "common.yaml"), "a",
              encoding="utf-8") as fh:
        fh.write("# note\n")
    assert _check(ctx) != []
    after = {name: open(os.path.join(root, name), "rb").read()
             for name in sorted(os.listdir(root))}
    assert after == before, "--check modified a file under resolved_root"
    assert stats == {name: os.stat(os.path.join(root, name)).st_mtime_ns
                     for name in before}


# --------------------------------------------------------------------------
# One edit, one finding kind
# --------------------------------------------------------------------------

def test_a_comment_only_source_edit_reports_layer_changed_alone(tmp_path):
    """A comment moves the file's bytes and nothing the system reads.

    So: layer_changed and config_rev_mismatch, but NOT common_digest_mismatch
    and NOT snapshot_stale -- the running processes are already equivalent to
    the source, and telling an operator to restart for this would spend an
    outage on nothing.

    Mutant: hash the parsed tree in the layer inventory -> layer_changed
    disappears -> red. Mutant: fold the raw bytes into common_digest ->
    common_digest_mismatch appears -> red.
    """
    ctx = _scaffold_with_proc(tmp_path)
    _freeze_into(ctx)
    with open(os.path.join(ctx["config_root"], "common.yaml"), "a",
              encoding="utf-8") as fh:
        fh.write("# an operator left a note\n")
    assert _kinds(_check(ctx)) == ["config_rev_mismatch", "layer_changed"]


def test_a_real_common_edit_reports_the_motion_gating_finding(tmp_path):
    """A value change under common.* must produce common_digest_mismatch --
    the finding whose documented consequence is P2 holding "禁止运动"
    (10 S5.4.4).

    Mutant: drop the common_digest comparison from check_freeze -> red.
    """
    ctx = _scaffold_with_proc(tmp_path)
    _freeze_into(ctx)
    path = os.path.join(ctx["config_root"], "common.yaml")
    tree = yaml.safe_load(open(path, encoding="utf-8"))
    tree["common"]["log_level"] = "DEBUG"
    open(path, "w", encoding="utf-8").write(
        yaml.safe_dump(tree, allow_unicode=True))
    kinds = _kinds(_check(ctx))
    assert "common_digest_mismatch" in kinds
    assert "layer_changed" in kinds


def test_a_per_process_edit_reports_snapshot_stale(tmp_path):
    """An L6 source edit reaches exactly one process, and the finding says
    which one.

    It must NOT report common_digest_mismatch: 10 S5.4.4 verbatim
    "私有段变更不应阻塞放行". Mutant: fold the per-proc trees into
    common_digest -> red, and a p2_core tick-rate change would start holding
    the robot at Stage C.
    """
    ctx = _scaffold_with_proc(tmp_path)
    _freeze_into(ctx)
    open(os.path.join(ctx["config_root"], "p2_core.yaml"), "w",
         encoding="utf-8").write(
        yaml.safe_dump({"p2_core": {"tick_hz": 25}}, allow_unicode=True))
    findings = _check(ctx)
    stale = [f for f in findings if f.kind == "snapshot_stale"]
    assert [f.detail["proc"] for f in stale] == ["p2_core"]
    assert "common_digest_mismatch" not in _kinds(findings)


def test_a_hand_edited_snapshot_reports_snapshot_tampered(tmp_path):
    """Iron rule 2: data/run/resolved/ is generated, never edited. The edit
    rolls back at the next boot, and without this finding the rollback is
    discovered as a mystery.

    This is INF-DP-5's first mutant ("改快照不改源") made observable.
    Mutant: compare the fresh snapshot's path instead of the recorded one ->
    the hand edit is never looked at -> red.
    """
    ctx = _scaffold_with_proc(tmp_path)
    manifest = _freeze_into(ctx)
    snap = manifest["processes"]["p2_core"]["path"]
    with open(snap, "a", encoding="utf-8") as fh:
        fh.write("tick_hz: 999\n")
    findings = _check(ctx)
    assert [f.kind for f in findings] == ["snapshot_tampered"]
    assert findings[0].detail["proc"] == "p2_core"
    # The source was NOT touched, so nothing else may fire -- in particular
    # not snapshot_stale, which would send the operator to configs/ for an
    # edit that is not there.
    assert "snapshot_stale" not in _kinds(findings)


def test_a_deleted_snapshot_reports_snapshot_missing(tmp_path):
    """Mutant: treat a missing file as equal to its recorded hash -> red."""
    ctx = _scaffold_with_proc(tmp_path)
    manifest = _freeze_into(ctx)
    os.unlink(manifest["processes"]["p2_core"]["path"])
    assert _kinds(_check(ctx)) == ["snapshot_missing"]


def test_a_new_source_file_reports_layer_added(tmp_path):
    """The FILE SET changing is a different event from a file's bytes
    changing, and config_rev moves for both. Mutant: key the layer comparison
    by list position -> an inserted row reports every later row as changed and
    buries this one -> red.

    The new file is the L4b calib file the scaffold's robot_id (gj-001) names
    but does not have. That is a deliberate choice of vehicle: it is the only
    layer where a file can APPEAR without the freeze refusing, because L2/L3
    are namespace-restricted (S22: "L2 may only write common.spec., common
    .motion.") and any file legal there would collide with common.yaml on
    assertion B. It also exercises the picked-layer path, where an appearing
    file is the realistic case -- a robot gets calibrated.
    """
    ctx = _scaffold_with_proc(tmp_path)
    _freeze_into(ctx)
    open(os.path.join(ctx["config_root"], "calib", "gj-001.yaml"), "w",
         encoding="utf-8").write("common: {}\n")
    kinds = _kinds(_check(ctx))
    assert kinds == ["config_rev_mismatch", "layer_added"]
    assert "layer_changed" not in kinds, (
        "adding a file must not report the untouched files as changed")


# --------------------------------------------------------------------------
# Degenerate inputs: report, do not crash
# --------------------------------------------------------------------------

def test_a_missing_manifest_is_the_only_finding(tmp_path):
    """Nothing downstream is answerable without a baseline, and a list of
    'everything differs' would bury the one fact that matters.

    Mutant: continue past the missing manifest -> a pile of findings derived
    from an empty dict -> red.
    """
    ctx = _scaffold_with_proc(tmp_path)
    assert _kinds(_check(ctx)) == ["manifest_missing"]


def test_an_unparseable_manifest_is_reported_not_raised(tmp_path):
    """Mutant: let json.JSONDecodeError propagate -> red. A diagnostic that
    crashes on the damaged input it was run to diagnose is no diagnostic."""
    ctx = _scaffold_with_proc(tmp_path)
    _freeze_into(ctx)
    open(os.path.join(ctx["resolved_root"], "MANIFEST.json"), "w",
         encoding="utf-8").write("{not json")
    assert _kinds(_check(ctx)) == ["manifest_unreadable"]


def test_a_snapshot_from_a_previous_boot_is_reported(tmp_path):
    """10 S5.4.4 makes this an R-level refusal for a process. --check has to
    survive it to report the rest.

    Mutant: route the manifest read through load_manifest (which runs the
    boot_id gate and raises) -> red, because the check would abort instead of
    reporting.
    """
    ctx = _scaffold_with_proc(tmp_path)
    _freeze_into(ctx)
    findings = _check(ctx, boot_id="a-different-boot")
    assert "boot_id_stale" in _kinds(findings)
    stale = [f for f in findings if f.kind == "boot_id_stale"][0]
    assert stale.detail["recorded"] == BOOT
    assert stale.detail["current"] == "a-different-boot"


def test_sources_that_refuse_to_freeze_are_reported_once(tmp_path):
    """Today's real configs/ are key-position skeletons and refuse (CLAUDE.md
    3.1), so this is the shape a --check on the real tree takes. It must name
    the reason and stop, not emit mismatches derived from a failed pass.

    Mutant: let the XbrainError propagate -> red. Mutant: continue and compare
    against a partial result -> red (the assert on the single finding).
    """
    ctx = _scaffold_with_proc(tmp_path)
    _freeze_into(ctx)
    # Break a reference: ${common.nope} resolves to nothing, which is what an
    # unfilled tree looks like to the resolver.
    open(os.path.join(ctx["config_root"], "p2_core.yaml"), "w",
         encoding="utf-8").write("p2_core:\n  a: \"${common.nope}\"\n")
    findings = _check(ctx)
    assert [f.kind for f in findings] == ["sources_refuse_freeze"]
    assert findings[0].detail["reason"]


# --------------------------------------------------------------------------
# The closed set itself
# --------------------------------------------------------------------------

def test_finding_kinds_are_a_closed_set():
    """CLAUDE.md 3.5: an out-of-set value must raise, never pass through.

    Mutant: drop the membership check in Finding.__init__ -> red. Without it a
    typo reaches an operator as a plausible line and reaches a caller's
    branch as a silent no-match.
    """
    import pytest
    with pytest.raises(AssertionError, match="FINDING_KINDS"):
        Finding("commom_digest_mismatch", proc="p2_core")
    # Every kind is reachable: a name in the tuple that no code path emits is
    # dead weight that a reader would take for a condition worth handling.
    emitted = set()
    for name in FINDING_KINDS:
        emitted.add(Finding(name).kind)
    assert emitted == set(FINDING_KINDS)


def test_finding_lines_are_stable_text(tmp_path):
    """Two runs of the same drift produce the same text, so a diff of two
    reports shows only real differences. Mutant: drop the sorted() on the
    detail keys -> the line order of fields depends on dict insertion and a
    diff becomes unreadable."""
    a = Finding("layer_changed", path="/x/common.yaml", level="L1", was="1",
                now="2")
    b = Finding("layer_changed", now="2", was="1", level="L1",
                path="/x/common.yaml")
    assert a.line() == b.line()
    assert a.line().startswith("DRIFT layer_changed")
