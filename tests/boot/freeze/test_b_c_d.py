"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_b_c_d.py
Brief: CFG-FZ-4 -- B/C/D real-body tests + 4 named variants

Description:
Shares the scaffold shape from test_a_and_m.py; adds L6 file writes
(p2_core / p3_task / p4_agent yaml) needed by C's profile_admission,
low_batt_profile, and B's per-file alias scan.

CFG-FZ-4 named variants:
  (1) p4_agent.yaml with point_min_dist_m: 0.5   -> B red (leaf alias)
  (2) event_days < task_days                     -> C red (retention)
  (3) cruise added to common.motion.profiles     -> C red (deprecated)
  (4) g_person_dist_m added to BLACKLIST         -> meta-test red
      (reverse-entry guard)

Reverse baseline: fully filled tree + minimal L6 files pass B + C + D
all green.
"""

import os
from typing import Any, Dict

import pytest
import yaml

from xbrain.boot.freeze.assertions._alias_table import (
    BLACKLIST, REVERSE_ENTRIES,
)
from xbrain.boot.freeze.assertions.b_no_duplicates import run as b_run
from xbrain.boot.freeze.assertions.c_cross_file import run as c_run
from xbrain.boot.freeze.assertions.d_identity import run as d_run
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Scaffolding
# ---------------------------------------------------------------------------

def _green_common_tree() -> Dict[str, Any]:
    """Filled common tree that satisfies A, M AND all C-checks. All
    fields C reads have consistent values; nothing forbidden."""
    return {
        "common": {
            "robot_id": "gj-001",
            "spec": {
                "max_vx_mps": 2.0, "max_vy_mps": 0.3, "max_wz_radps": 0.4,
                "max_accel_mps2": 1.0, "max_decel_mps2": 2.5,
            },
            "safety": {"t_lat_s": 0.4, "d_safe_m": 1.0},
            "motion": {"profiles": {
                "obstacle_avoid": {"max_mps": 0.5},
                "patrol": {"max_mps": 2.0},
            }},
            "fence": {"soft_margin_min_m": 0.30, "predict_dt_s": 0.45},
            "retention": {
                # Monotone: task <= event <= command.
                "task_days": 30, "event_days": 90, "command_days": 180,
            },
            "recording": {
                # C-3: fence_close_tol == 2 * min_dist.
                "min_dist_m": 0.5, "fence_close_tol_m": 1.0,
            },
        },
    }


def _green_l6_p2() -> Dict[str, Any]:
    """P2 tree: profile_admission keys match common.motion.profiles."""
    return {
        "health": {
            "profile_admission": {
                "obstacle_avoid": True,
                "patrol": True,
            },
        },
    }


def _green_l6_p3() -> Dict[str, Any]:
    """P3 tree: low_batt_profile is an existing profile."""
    return {
        "charge": {"low_batt_profile": "obstacle_avoid"},
    }


def _make_root(tmp_path, common_tree=None,
               p2_tree=None, p3_tree=None, p4_tree=None) -> str:
    """Build a full config root; any tree arg not given uses green defaults."""
    root = tmp_path / "configs"
    root.mkdir()
    (root / "common.yaml").write_text(
        yaml.safe_dump(common_tree if common_tree is not None
                       else _green_common_tree(), allow_unicode=True)
    )
    # J-required L6 files with sensible defaults.
    (root / "p2_core.yaml").write_text(
        yaml.safe_dump(p2_tree if p2_tree is not None else _green_l6_p2(),
                       allow_unicode=True)
    )
    (root / "p3_task.yaml").write_text(
        yaml.safe_dump(p3_tree if p3_tree is not None else _green_l6_p3(),
                       allow_unicode=True)
    )
    (root / "p4_agent.yaml").write_text(
        yaml.safe_dump(p4_tree if p4_tree is not None else {}, allow_unicode=True)
    )
    for name in ("p1_motion.yaml", "p5_gateway.yaml", "quadruped.yaml"):
        (root / name).write_text("# empty L6 placeholder\n")
    (root / "models").mkdir()
    (root / "safety").mkdir()
    return str(root)


# ---------------------------------------------------------------------------
# Reverse baselines
# ---------------------------------------------------------------------------

def test_green_passes_b(tmp_path):
    """B: no L6 files carry `common`, alias names, or dotted alias paths."""
    root = _make_root(tmp_path)
    result = b_run({"config_root": root})
    assert result["status"] == "pass"
    assert result["assertion"] == "B"
    assert result["l6_files_checked"] >= 2   # p2, p3 at least


def test_green_passes_c(tmp_path):
    """C: retention monotone, profiles match, fence_close_tol == 2*min_dist,
    low_batt_profile exists, no deprecated profile."""
    root = _make_root(tmp_path)
    result = c_run({"config_root": root})
    assert result["status"] == "pass"
    assert result["assertion"] == "C"


def test_green_passes_d(tmp_path):
    """D: robot_id matches pattern, no site_id (skipped), no zenoh key template."""
    root = _make_root(tmp_path)
    result = d_run({"config_root": root})
    assert result["status"] == "pass"


# ---------------------------------------------------------------------------
# Variant (1): p4_agent.yaml carries `point_min_dist_m` -> B red
# ---------------------------------------------------------------------------

def test_variant_1_l6_alias_leaf_is_red(tmp_path):
    """CFG-FZ-4 variant (1) verbatim: `point_min_dist_m: 0.5` in
    p4_agent.yaml. The leaf name is on BLACKLIST -- B rejects."""
    p4_bad = {"asr": {"point_min_dist_m": 0.5}}   # nested; leaf still matches
    root = _make_root(tmp_path, p4_tree=p4_bad)
    with pytest.raises(XbrainError) as ei:
        b_run({"config_root": root})
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["kind"] == "l6_alias_name"
    assert ei.value.detail["leaf"] == "point_min_dist_m"
    assert ei.value.detail["file"] == "p4_agent.yaml"


# ---------------------------------------------------------------------------
# Variant: L6 top-level `common` key -> B red (path (1) sibling)
# ---------------------------------------------------------------------------

def test_variant_l6_top_level_common_is_red(tmp_path):
    """B path 1: p4_agent.yaml redefining `common.foo = ...`."""
    p4_bad = {"common": {"foo": "bar"}}
    root = _make_root(tmp_path, p4_tree=p4_bad)
    with pytest.raises(XbrainError) as ei:
        b_run({"config_root": root})
    assert ei.value.detail["kind"] == "l6_common_top_level"


# ---------------------------------------------------------------------------
# Variant (2): event_days < task_days -> C red
# ---------------------------------------------------------------------------

def test_variant_2_retention_not_monotone_is_red(tmp_path):
    """CFG-FZ-4 variant (2) verbatim: event_days < task_days."""
    tree = _green_common_tree()
    tree["common"]["retention"]["event_days"] = 15   # < task_days=30
    root = _make_root(tmp_path, common_tree=tree)
    with pytest.raises(XbrainError) as ei:
        c_run({"config_root": root})
    assert ei.value.detail["kind"] == "retention_not_monotone"
    assert ei.value.detail["which_pair"] == "task > event"


# ---------------------------------------------------------------------------
# Variant (3): cruise added back to common.motion.profiles -> C red
# ---------------------------------------------------------------------------

def test_variant_3_cruise_deprecated_profile_is_red(tmp_path):
    """CFG-FZ-4 variant (3) verbatim: put `cruise` back."""
    tree = _green_common_tree()
    tree["common"]["motion"]["profiles"]["cruise"] = {"max_mps": 3.0}
    root = _make_root(tmp_path, common_tree=tree)
    with pytest.raises(XbrainError) as ei:
        c_run({"config_root": root})
    assert ei.value.detail["kind"] == "deprecated_profile_present"
    assert ei.value.detail["profile"] == "cruise"


# ---------------------------------------------------------------------------
# C additional coverage
# ---------------------------------------------------------------------------

def test_profile_admission_mismatch_is_red(tmp_path):
    """C-2: P2.profile_admission has an extra key not in common.motion.profiles."""
    p2 = _green_l6_p2()
    p2["health"]["profile_admission"]["cruise"] = True   # not in common
    root = _make_root(tmp_path, p2_tree=p2)
    with pytest.raises(XbrainError) as ei:
        c_run({"config_root": root})
    assert ei.value.detail["kind"] == "profile_admission_mismatch"
    assert "cruise" in ei.value.detail["only_in_admission"]


def test_low_batt_profile_missing_is_red(tmp_path):
    """C-4: P3.charge.low_batt_profile names a non-existent profile."""
    p3 = {"charge": {"low_batt_profile": "typo_profile"}}
    root = _make_root(tmp_path, p3_tree=p3)
    with pytest.raises(XbrainError) as ei:
        c_run({"config_root": root})
    assert ei.value.detail["kind"] == "low_batt_profile_missing"
    assert ei.value.detail["low_batt_profile"] == "typo_profile"


def test_fence_close_tol_ratio_wrong_is_red(tmp_path):
    """C-3: fence_close_tol_m must be exactly 2 * min_dist_m."""
    tree = _green_common_tree()
    tree["common"]["recording"]["fence_close_tol_m"] = 1.5   # not 2*0.5=1.0
    root = _make_root(tmp_path, common_tree=tree)
    with pytest.raises(XbrainError) as ei:
        c_run({"config_root": root})
    assert ei.value.detail["kind"] == "fence_close_tol_ratio"


# ---------------------------------------------------------------------------
# D coverage
# ---------------------------------------------------------------------------

def test_d_robot_id_uppercase_is_red(tmp_path):
    """D-1: robot_id GJ-001 (uppercase) violates [a-z0-9_-]."""
    tree = _green_common_tree()
    tree["common"]["robot_id"] = "GJ-001"
    root = _make_root(tmp_path, common_tree=tree)
    with pytest.raises(XbrainError) as ei:
        d_run({"config_root": root})
    assert ei.value.detail["kind"] == "rid_pattern_bad"


def test_d_zenoh_key_template_in_config_is_red(tmp_path):
    """D-3: common.zenoh.key_template must not exist."""
    tree = _green_common_tree()
    tree["common"]["zenoh"] = {"key_template": "xbrain/{rid}/foo"}
    root = _make_root(tmp_path, common_tree=tree)
    with pytest.raises(XbrainError) as ei:
        d_run({"config_root": root})
    assert ei.value.detail["kind"] == "zenoh_key_from_config"


# ---------------------------------------------------------------------------
# Variant (4): reverse-entry guard -- meta-test on BLACKLIST vs REVERSE_ENTRIES
# ---------------------------------------------------------------------------

def test_variant_4_reverse_entries_not_in_blacklist():
    """CFG-FZ-4 variant (4) verbatim: 'g_person_dist_m added to
    alias blacklist' -> meta-test must report 'reverse entry
    misused as blacklist entry'.

    Meta-test: BLACKLIST ∩ REVERSE_ENTRIES must be empty. Adding a
    reverse entry to the blacklist (or vice versa) is a category
    error and this test guards it. If someone future-edits BLACKLIST
    to include g_person_dist_m, this test fails."""
    overlap = BLACKLIST & REVERSE_ENTRIES
    assert not overlap, (
        "BLACKLIST wrongly contains reverse-entry names %s -- these "
        "are legitimately distinct from their same-value common.* keys "
        "(see 10 S5.4.5 末段)" % sorted(overlap)
    )


def test_variant_4_g_person_dist_m_is_reverse_entry():
    """Guard the guard: g_person_dist_m (the CFG-FZ-4 variant target)
    MUST be in REVERSE_ENTRIES. If someone removes it, the variant 4
    test above becomes vacuous."""
    assert "g_person_dist_m" in REVERSE_ENTRIES


# ---------------------------------------------------------------------------
# Wiring guards
# ---------------------------------------------------------------------------

def test_b_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        b_run({})


def test_c_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        c_run({})


def test_d_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        d_run({})
