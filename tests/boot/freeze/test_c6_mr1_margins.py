"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_c6_mr1_margins.py
Brief: CFG-FZ-15 -- C-6 (>=) + MR-1 (==), three variants + baseline

Description:
Both assertions live in one runner. Tests use overlay + p1_motion.yaml
(L6) scaffolds.

CFG-FZ-15 variants:
  M-MR1-a  margin_rot = 0.30 (d_safe stays 1.0)  -> MR-1 red
  M-MR1-b  margin_rot = 1.50                     -> MR-1 red
           (distinguishes '==' from '>=')
  M-MR1-c  margin_rot = 0.30 AND d_safe = 0.30   -> MR-1 GREEN
           (MR-1 is identity, not range -- S-4 handles range)

Reverse: equal + wider-lat pass; missing operand skip.
"""

from typing import Any, Dict

import pytest
import yaml

from xbrain.boot.freeze.assertions.c6_mr1_margins import run
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Scaffolding
# ---------------------------------------------------------------------------

def _green_common() -> Dict[str, Any]:
    """Merged overlay tree with common.safety.d_safe_m = 1.0."""
    return {"common": {"safety": {"d_safe_m": 1.0}}}


def _green_p1() -> Dict[str, Any]:
    """p1_motion.yaml tree satisfying both C-6 and MR-1."""
    return {
        # C-6: margin_lat_m (1.20) >= margin_base_m (1.00) -- pass.
        # Wider lateral than base is legit (slower profiles).
        "corridor": {"margin_lat_m": 1.20, "margin_base_m": 1.00},
        # MR-1: margin_rot_m == d_safe_m 1.0 -- pass.
        "rotation_clearance": {"margin_rot_m": 1.00},
    }


def _make_root(tmp_path, common_tree=None, p1_tree=None) -> str:
    """Build a config root with common.yaml + p1_motion.yaml + stubs."""
    root = tmp_path / "configs"
    root.mkdir()
    (root / "common.yaml").write_text(
        yaml.safe_dump(common_tree if common_tree is not None
                       else _green_common(), allow_unicode=True)
    )
    (root / "p1_motion.yaml").write_text(
        yaml.safe_dump(p1_tree if p1_tree is not None else _green_p1(),
                       allow_unicode=True)
    )
    for name in ("p2_core.yaml", "p3_task.yaml", "p4_agent.yaml",
                 "p5_gateway.yaml", "quadruped.yaml"):
        (root / name).write_text("# empty stub\n")
    (root / "models").mkdir()
    (root / "safety").mkdir()
    return str(root)


# ---------------------------------------------------------------------------
# Baseline: everything equal / wider passes
# ---------------------------------------------------------------------------

def test_green_scaffold_passes(tmp_path):
    """Baseline: lat > base, rot == d_safe. Both checks pass."""
    root = _make_root(tmp_path)
    result = run({"config_root": root})
    assert result["status"] == "pass"
    assert result["assertion"] == "C-6+MR-1"
    assert result["checks_run"] == 2


def test_c6_lat_equal_to_base_passes(tmp_path):
    """C-6 uses >=, so lat == base is legit."""
    p1 = _green_p1()
    p1["corridor"]["margin_lat_m"] = 1.00     # == base
    root = _make_root(tmp_path, p1_tree=p1)
    result = run({"config_root": root})
    assert result["status"] == "pass"


# ---------------------------------------------------------------------------
# CFG-FZ-15 variant M-MR1-a: margin_rot=0.30 -> MR-1 red
# ---------------------------------------------------------------------------

def test_variant_a_rot_below_d_safe_is_mr1_red(tmp_path):
    """M-MR1-a verbatim: margin_rot_m = 0.30, d_safe stays 1.0."""
    p1 = _green_p1()
    p1["rotation_clearance"]["margin_rot_m"] = 0.30
    root = _make_root(tmp_path, p1_tree=p1)
    with pytest.raises(XbrainError) as ei:
        run({"config_root": root})
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["rule"] == "MR-1"
    assert ei.value.detail["lhs"] == 0.30
    assert ei.value.detail["rhs"] == 1.0
    assert ei.value.detail["relation"] == "=="


# ---------------------------------------------------------------------------
# CFG-FZ-15 variant M-MR1-b: margin_rot=1.50 -> MR-1 red (>= vs ==)
# ---------------------------------------------------------------------------

def test_variant_b_rot_above_d_safe_is_mr1_red(tmp_path):
    """M-MR1-b verbatim: margin_rot_m = 1.50, d_safe stays 1.0.
    Distinguishes IDENTITY (==) from GREATER-EQUAL (>=). An
    implementation checking rot >= d_safe would pass this
    (1.5 >= 1.0), but the assertion is ==, so it must red."""
    p1 = _green_p1()
    p1["rotation_clearance"]["margin_rot_m"] = 1.50   # > d_safe
    root = _make_root(tmp_path, p1_tree=p1)
    with pytest.raises(XbrainError) as ei:
        run({"config_root": root})
    assert ei.value.detail["rule"] == "MR-1"
    assert ei.value.detail["lhs"] == 1.50


# ---------------------------------------------------------------------------
# CFG-FZ-15 variant M-MR1-c: rot AND d_safe both 0.30 -> MR-1 GREEN
# ---------------------------------------------------------------------------

def test_variant_c_both_0_30_is_mr1_green(tmp_path):
    """M-MR1-c verbatim: BOTH sides = 0.30. MR-1 is identity, so
    equality passes. The RANGE (d_safe >= 1.0) is S-4's job, not
    MR-1's -- MR-1 must NOT overreach."""
    common = _green_common()
    common["common"]["safety"]["d_safe_m"] = 0.30
    p1 = _green_p1()
    p1["rotation_clearance"]["margin_rot_m"] = 0.30
    root = _make_root(tmp_path, common_tree=common, p1_tree=p1)
    # Should NOT raise -- MR-1 only checks identity.
    result = run({"config_root": root})
    assert result["status"] == "pass"


# ---------------------------------------------------------------------------
# C-6 coverage: lat < base is red
# ---------------------------------------------------------------------------

def test_c6_lat_below_base_is_red(tmp_path):
    """C-6: margin_lat_m < margin_base_m -> red."""
    p1 = _green_p1()
    p1["corridor"]["margin_lat_m"] = 0.80   # < base 1.0
    root = _make_root(tmp_path, p1_tree=p1)
    with pytest.raises(XbrainError) as ei:
        run({"config_root": root})
    assert ei.value.detail["rule"] == "C-6"
    assert ei.value.detail["lhs"] == 0.80
    assert ei.value.detail["rhs"] == 1.00
    assert ei.value.detail["relation"] == ">="


# ---------------------------------------------------------------------------
# Skip semantics: missing operand -> skipped
# ---------------------------------------------------------------------------

def test_missing_rotation_margin_skipped(tmp_path):
    """MR-1 skips when margin_rot_m absent (dev checkout)."""
    p1 = _green_p1()
    del p1["rotation_clearance"]
    root = _make_root(tmp_path, p1_tree=p1)
    # Should NOT raise -- MR-1 skips silently.
    result = run({"config_root": root})
    assert result["status"] == "pass"


def test_missing_d_safe_skipped(tmp_path):
    """MR-1 skips when d_safe_m absent."""
    common = _green_common()
    common["common"]["safety"] = {}
    root = _make_root(tmp_path, common_tree=common)
    result = run({"config_root": root})
    assert result["status"] == "pass"


def test_missing_corridor_skipped(tmp_path):
    """C-6 skips when corridor block absent."""
    p1 = _green_p1()
    del p1["corridor"]
    root = _make_root(tmp_path, p1_tree=p1)
    result = run({"config_root": root})
    assert result["status"] == "pass"


# ---------------------------------------------------------------------------
# Wiring guard
# ---------------------------------------------------------------------------

def test_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run({})
