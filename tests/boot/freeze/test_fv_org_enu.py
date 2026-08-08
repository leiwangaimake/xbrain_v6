"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fv_org_enu.py
Brief: CFG-FZ-14 -- FV-ORG-1/-2/-3 four variants (2 must-red, 2 must-
       pass) + baseline

Description:
enu_origin lives across three layers: L1 (null placeholders only),
L4 (real value, sourced from sites/{site_id}.yaml), and the merged
overlay (what runtime reads). Tests exercise each layer combination.

CFG-FZ-14 variants:
  M-ORG-a  {0.0, 0.0, 0.0}                    -> MUST PASS
  M-ORG-b  lat/lon filled, alt null           -> FV-ORG-1 red
  M-ORG-c  L1 filled real, L4 absent          -> FV-ORG-3 red
  M-ORG-d  L1 null placeholders, L4 filled    -> MUST PASS

Baseline: L1 null + L4 real value pass all three sub-checks.
"""

import os
from typing import Any, Dict

import pytest
import yaml

from xbrain.boot.freeze.assertions.fv_org_enu import run
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Scaffolding
# ---------------------------------------------------------------------------

def _l1_null_placeholders() -> Dict[str, Any]:
    """L1 (common.yaml) with three-null placeholder + site_id."""
    return {
        "common": {
            "site_id": "site_test",
            "robot_id": "gj-001",
            "geo": {
                "enu_origin": {"lat": None, "lon": None, "alt": None},
            },
        },
    }


def _l4_real_values() -> Dict[str, Any]:
    """L4 (sites/site_test.yaml) with real enu_origin values."""
    return {
        "common": {
            "geo": {
                "enu_origin": {
                    "lat": 31.2301971,
                    "lon": 121.4732683,
                    "alt": 8.4,
                },
            },
        },
    }


def _make_root(tmp_path, l1=None, l4=None) -> str:
    """Build a config root with L1 + L4 (+ empty L2/L3/L6 stubs)."""
    root = tmp_path / "configs"
    root.mkdir()
    (root / "common.yaml").write_text(
        yaml.safe_dump(l1 if l1 is not None else _l1_null_placeholders(),
                       allow_unicode=True)
    )
    # L2/L3/L4/L4b/L6 empty by default.
    (root / "models").mkdir()
    (root / "safety").mkdir()
    (root / "sites").mkdir()
    (root / "calib").mkdir()
    # L4 file if provided.
    l4_content = l4 if l4 is not None else _l4_real_values()
    (root / "sites" / "site_test.yaml").write_text(
        yaml.safe_dump(l4_content, allow_unicode=True)
    )
    # J-required process files.
    for name in ("p1_motion.yaml", "p2_core.yaml", "p3_task.yaml",
                 "p4_agent.yaml", "p5_gateway.yaml", "quadruped.yaml"):
        (root / name).write_text("# empty L6 stub\n")
    return str(root)


# ---------------------------------------------------------------------------
# Baseline: L1 null + L4 real -> pass (= M-ORG-d, MUST-PASS variant)
# ---------------------------------------------------------------------------

def test_baseline_l1_null_l4_real_passes(tmp_path):
    """M-ORG-d verbatim: L1 null placeholders + L4 real values.
    This IS the normal deployment shape per 10 S5.4.3."""
    root = _make_root(tmp_path)
    result = run({"config_root": root})
    assert result["status"] == "pass"
    assert result["assertion"] == "FV-ORG"
    assert result["site_id"] == "site_test"


# ---------------------------------------------------------------------------
# M-ORG-a MUST PASS: {0.0, 0.0, 0.0} is a legal coordinate
# ---------------------------------------------------------------------------

def test_variant_a_zero_coords_must_pass(tmp_path):
    """M-ORG-a verbatim: enu_origin = {0.0, 0.0, 0.0} in L4.
    0.0 is a legal WGS84 coordinate (Gulf of Guinea intersection);
    rejecting it would be a 'definition masquerading as observation'
    defect (see 10 S5.4.4 M-ORG-a rationale)."""
    l4 = {"common": {"geo": {"enu_origin": {
        "lat": 0.0, "lon": 0.0, "alt": 0.0}}}}
    root = _make_root(tmp_path, l4=l4)
    # Should NOT raise -- 0.0 is legal.
    result = run({"config_root": root})
    assert result["status"] == "pass"


# ---------------------------------------------------------------------------
# M-ORG-b: alt=null while lat/lon filled -> FV-ORG-1 red
# ---------------------------------------------------------------------------

def test_variant_b_alt_null_is_fv_org_1_red(tmp_path):
    """M-ORG-b verbatim: lat/lon filled, alt = null. Distinguishes
    per-component check from object-exists shortcut."""
    l4 = {"common": {"geo": {"enu_origin": {
        "lat": 31.23, "lon": 121.47, "alt": None}}}}
    root = _make_root(tmp_path, l4=l4)
    with pytest.raises(XbrainError) as ei:
        run({"config_root": root})
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["rule"] == "FV-ORG-1"
    assert ei.value.detail["component"] == "alt"


def test_variant_b_lat_null_is_fv_org_1_red(tmp_path):
    """Same rule, different component -- lat = null."""
    l4 = {"common": {"geo": {"enu_origin": {
        "lat": None, "lon": 121.47, "alt": 8.4}}}}
    root = _make_root(tmp_path, l4=l4)
    with pytest.raises(XbrainError) as ei:
        run({"config_root": root})
    assert ei.value.detail["rule"] == "FV-ORG-1"
    assert ei.value.detail["component"] == "lat"


# ---------------------------------------------------------------------------
# M-ORG-c: L1 filled real, L4 absent -> FV-ORG-3 red
# ---------------------------------------------------------------------------

def test_variant_c_l1_filled_l4_absent_is_fv_org_3_red(tmp_path):
    """M-ORG-c verbatim: L1 carries a real enu_origin AND L4 has none.
    L1 must only hold null placeholders; a real value there is a
    site-data-in-shared-layer defect."""
    # L1 with REAL values (violates FV-ORG-3 for L1).
    l1 = {
        "common": {
            "site_id": "site_test",
            "robot_id": "gj-001",
            "geo": {"enu_origin": {"lat": 31.23, "lon": 121.47, "alt": 8.4}},
        },
    }
    # L4 with no enu_origin.
    l4 = {"common": {"geo": {}}}
    root = _make_root(tmp_path, l1=l1, l4=l4)
    with pytest.raises(XbrainError) as ei:
        run({"config_root": root})
    assert ei.value.detail["rule"] == "FV-ORG-3"
    assert ei.value.detail["layer"] == "L1"


def test_l4_missing_enu_origin_is_fv_org_3_red(tmp_path):
    """L4 file exists but has no enu_origin -> FV-ORG-3 red (L4-side)."""
    l4 = {"common": {"geo": {"other_key": "irrelevant"}}}
    root = _make_root(tmp_path, l4=l4)
    with pytest.raises(XbrainError) as ei:
        run({"config_root": root})
    assert ei.value.detail["rule"] == "FV-ORG-3"


# ---------------------------------------------------------------------------
# Extra coverage: L2/L4b/L6 must not carry enu_origin
# ---------------------------------------------------------------------------

def test_l6_carries_enu_origin_is_fv_org_3_red(tmp_path):
    """L6 process config (p1_motion.yaml) with enu_origin -> red."""
    root = _make_root(tmp_path)
    # Overwrite p1_motion.yaml with enu_origin.
    p1_path = os.path.join(root, "p1_motion.yaml")
    with open(p1_path, "w") as f:
        yaml.safe_dump({"common": {"geo": {"enu_origin": {
            "lat": 31.23, "lon": 121.47, "alt": 8.4}}}}, f)
    with pytest.raises(XbrainError) as ei:
        run({"config_root": root})
    assert ei.value.detail["rule"] == "FV-ORG-3"
    assert "L6" in ei.value.detail["layer"]


# ---------------------------------------------------------------------------
# Wiring guard
# ---------------------------------------------------------------------------

def test_fv_org_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run({})
