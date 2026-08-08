"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_n_o_identities.py
Brief: CFG-FZ-8 -- N + O identity assertions, two named variants +
       baselines

Description:
Both N and O compare two configured values for equality; tests build
a merged tree + a p1_motion.yaml file with the two operands, then
run each assertion.

CFG-FZ-8 named variants:
  (1) margin_base_m = 1.1 -> N red (d_safe_m stays 1.0)
  (2) teleop.cloud.priority = 560 -> O red (arb stays 550)

Reverse: equal values pass; missing values skip.
"""

import os
from typing import Any, Dict

import pytest
import yaml

from xbrain.boot.freeze.assertions.n_o_identities import run_n, run_o
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Scaffolding
# ---------------------------------------------------------------------------

def _green_common_tree() -> Dict[str, Any]:
    """Merged overlay tree with common.safety.d_safe_m = 1.0 (U67)."""
    return {"common": {"safety": {"d_safe_m": 1.0}}}


def _green_p1_tree() -> Dict[str, Any]:
    """p1_motion.yaml tree with matching pairs for both N and O."""
    return {
        # N left side.
        "corridor": {"margin_base_m": 1.0},   # == common.safety.d_safe_m
        # O both sides -- 550 per 12 S12 verbatim.
        "teleop": {"cloud": {"priority": 550}},
        "arbitration": {"priorities": {"teleop_cloud": 550}},
    }


def _make_root(tmp_path, common_tree=None, p1_tree=None) -> str:
    """Build a config root with common.yaml + p1_motion.yaml + empty
    L6 stubs so J's required-files check would pass if invoked."""
    root = tmp_path / "configs"
    root.mkdir()
    (root / "common.yaml").write_text(
        yaml.safe_dump(common_tree if common_tree is not None
                       else _green_common_tree(), allow_unicode=True)
    )
    (root / "p1_motion.yaml").write_text(
        yaml.safe_dump(p1_tree if p1_tree is not None else _green_p1_tree(),
                       allow_unicode=True)
    )
    for name in ("p2_core.yaml", "p3_task.yaml", "p4_agent.yaml",
                 "p5_gateway.yaml", "quadruped.yaml"):
        (root / name).write_text("# empty L6 stub\n")
    (root / "models").mkdir()
    (root / "safety").mkdir()
    return str(root)


# ---------------------------------------------------------------------------
# Reverse baselines
# ---------------------------------------------------------------------------

def test_green_passes_n(tmp_path):
    """N passes when margin_base_m == d_safe_m."""
    root = _make_root(tmp_path)
    result = run_n({"config_root": root})
    assert result["status"] == "pass"
    assert result["assertion"] == "N"
    assert result["checked"] is True
    assert result["value"] == 1.0


def test_green_passes_o(tmp_path):
    """O passes when teleop.cloud.priority == arbitration.priorities.teleop_cloud."""
    root = _make_root(tmp_path)
    result = run_o({"config_root": root})
    assert result["status"] == "pass"
    assert result["assertion"] == "O"
    assert result["checked"] is True
    assert result["value"] == 550


# ---------------------------------------------------------------------------
# CFG-FZ-8 variant (1): margin_base_m = 1.1 -> N red
# ---------------------------------------------------------------------------

def test_variant_1_margin_base_1_1_is_n_red(tmp_path):
    """CFG-FZ-8 variant (1) verbatim: margin_base_m = 1.1."""
    p1 = _green_p1_tree()
    p1["corridor"]["margin_base_m"] = 1.1   # != d_safe_m 1.0
    root = _make_root(tmp_path, p1_tree=p1)
    with pytest.raises(XbrainError) as ei:
        run_n({"config_root": root})
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["kind"] == "identity_broken"
    assert ei.value.detail["rule"] == "N"
    assert ei.value.detail["lhs"] == 1.1
    assert ei.value.detail["rhs"] == 1.0


# ---------------------------------------------------------------------------
# CFG-FZ-8 variant (2): teleop.cloud.priority = 560 -> O red
# ---------------------------------------------------------------------------

def test_variant_2_teleop_priority_560_is_o_red(tmp_path):
    """CFG-FZ-8 variant (2) verbatim: teleop.cloud.priority = 560."""
    p1 = _green_p1_tree()
    p1["teleop"]["cloud"]["priority"] = 560   # arb stays 550
    root = _make_root(tmp_path, p1_tree=p1)
    with pytest.raises(XbrainError) as ei:
        run_o({"config_root": root})
    assert ei.value.detail["kind"] == "identity_broken"
    assert ei.value.detail["rule"] == "O"
    assert ei.value.detail["lhs"] == 560
    assert ei.value.detail["rhs"] == 550


# ---------------------------------------------------------------------------
# Reverse mutation: change RHS instead -- both assertions still fire
# ---------------------------------------------------------------------------

def test_n_also_fires_when_d_safe_m_changed(tmp_path):
    """Symmetry: whichever side moves, N reports the mismatch. The
    assertion is equality, so LHS vs RHS is just naming."""
    common = _green_common_tree()
    common["common"]["safety"]["d_safe_m"] = 0.8    # margin_base stays 1.0
    root = _make_root(tmp_path, common_tree=common)
    with pytest.raises(XbrainError) as ei:
        run_n({"config_root": root})
    assert ei.value.detail["rule"] == "N"


def test_o_also_fires_when_arb_side_changed(tmp_path):
    """Same symmetry for O."""
    p1 = _green_p1_tree()
    p1["arbitration"]["priorities"]["teleop_cloud"] = 540
    root = _make_root(tmp_path, p1_tree=p1)
    with pytest.raises(XbrainError) as ei:
        run_o({"config_root": root})
    assert ei.value.detail["rule"] == "O"
    assert ei.value.detail["lhs"] == 550   # teleop.cloud
    assert ei.value.detail["rhs"] == 540   # arb


# ---------------------------------------------------------------------------
# Skip semantics: missing operand -> skipped, not red
# ---------------------------------------------------------------------------

def test_n_skips_when_margin_absent(tmp_path):
    """N should skip if p1_motion.yaml lacks corridor.margin_base_m
    (dev checkout state). A/M handle required-ness elsewhere; N
    only checks equality on values that DO exist."""
    p1 = _green_p1_tree()
    del p1["corridor"]
    root = _make_root(tmp_path, p1_tree=p1)
    result = run_n({"config_root": root})
    assert result["status"] == "pass"
    assert result["checked"] is False


def test_o_skips_when_teleop_absent(tmp_path):
    """O skips if teleop sub-tree not configured."""
    p1 = _green_p1_tree()
    del p1["teleop"]
    root = _make_root(tmp_path, p1_tree=p1)
    result = run_o({"config_root": root})
    assert result["checked"] is False


# ---------------------------------------------------------------------------
# Wiring guards
# ---------------------------------------------------------------------------

def test_n_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run_n({})


def test_o_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run_o({})
