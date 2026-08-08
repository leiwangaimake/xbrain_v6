"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_a_and_m.py
Brief: CFG-FZ-3 -- assertion A (null/${) + assertion M (required key
       missing); mutants: delete safety.t_lat_s, add null, add unresolved ref

Description:
Both A and M read the same L1~L3 layer files, so the scaffolding is
shared. Each test builds a fake config root with common.yaml + models/
+ safety/, then runs the assertion under scrutiny and asserts the
expected pass/fail shape.

CFG-FZ-3 named variants:
  A: value = null must be caught by A (key path) BEFORE assertion G
     could catch it (range violation). Test injects null into a filled
     tree and asserts A raises with null_unassigned and points at the
     key path.
  A: residual ${common.*} in the resolved tree must be caught.
  M: deleting common.safety.t_lat_s from the tree (row-level delete,
     not null-writing) must make the whole stack refuse to start.

Reverse: a fully-filled tree must pass both A and M green.
"""

import os
from typing import Any, Dict

import pytest
import yaml

from xbrain.boot.freeze.assertions.a_references import run as a_run
from xbrain.boot.freeze.assertions.m_required import (
    _REQUIRED_KEYS, run as m_run,
)
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Scaffolding: build a green common.yaml + models/*.yaml + safety/*.yaml
# ---------------------------------------------------------------------------

def _green_common_tree() -> Dict[str, Any]:
    """Return a dict shaped like a filled common.yaml.

    Every key M requires is present; every value is non-null. Uses
    plausible values (from 10 §5.4.5 examples) but the actual numbers
    do not matter for A/M -- only shape matters.
    """
    return {
        "common": {
            "robot_id": "gj-001",
            "spec": {
                # Values from 10 §5.4.5: max_vx confirmed 2.0 m/s per U54;
                # others are placeholders that pass A (non-null).
                "max_vx_mps": 2.0,
                "max_vy_mps": 0.3,
                "max_wz_radps": 0.4,
                "max_accel_mps2": 1.0,
                "max_decel_mps2": 2.5,
            },
            "safety": {
                # t_lat_s = 0.4 per 10 §6.2 / 5.4.5.
                "t_lat_s": 0.4,
                "d_safe_m": 1.0,
            },
            "motion": {
                "profiles": {
                    "obstacle_avoid": {"max_mps": 0.5},
                    "patrol": {"max_mps": 2.0},
                },
            },
            "fence": {
                "soft_margin_min_m": 0.30,
                "predict_dt_s": 0.45,
            },
        },
    }


def _write_config_tree(root, tree_override: Dict[str, Any] = None) -> str:
    """Write a green (or override-mutated) config tree to `root` and
    return the path.

    root: pathlib.Path from tmp_path.
    tree_override: if given, use this dict for common.yaml instead of
      the default green tree. Callers use this to inject mutations
      (delete a key, set a value to null, etc).

    Layout written:
      root/
        common.yaml         # the tree (green or overridden)
        models/             # empty dir
        safety/             # empty dir
    """
    root.mkdir()
    tree = tree_override if tree_override is not None else _green_common_tree()
    (root / "common.yaml").write_text(yaml.safe_dump(tree, allow_unicode=True))
    (root / "models").mkdir()
    (root / "safety").mkdir()
    return str(root)


# ---------------------------------------------------------------------------
# Reverse baseline: green tree passes both A and M
# ---------------------------------------------------------------------------

def test_green_tree_passes_a(tmp_path):
    """A must pass on a fully-filled tree with no nulls and no ${.
    If this fails, either A over-reports or the green fixture is
    already broken -- fix baseline before touching mutants."""
    root = _write_config_tree(tmp_path / "configs")
    ctx = {"config_root": root}
    result = a_run(ctx)
    assert result["status"] == "pass"
    assert result["assertion"] == "A"
    # A populates ctx for M to reuse.
    assert "overlay" in ctx
    assert "layer_trees" in ctx


def test_green_tree_passes_m(tmp_path):
    """M must pass when every _REQUIRED_KEYS entry is filled by L1."""
    root = _write_config_tree(tmp_path / "configs")
    ctx = {"config_root": root}
    # Run A first to populate ctx, matching pipeline order.
    a_run(ctx)
    result = m_run(ctx)
    assert result["status"] == "pass"
    assert result["assertion"] == "M"
    assert result["required_count"] == len(_REQUIRED_KEYS)


def test_m_runs_standalone_without_a(tmp_path):
    """M's fallback path: called without A having run first. It should
    load layers itself and pass on a green tree. This is the unit-test
    convenience path; production ORD-1 always has A before M."""
    root = _write_config_tree(tmp_path / "configs")
    result = m_run({"config_root": root})           # no overlay preloaded
    assert result["status"] == "pass"


# ---------------------------------------------------------------------------
# Mutant A-1: value = null -> A reports null_unassigned with key path
# ---------------------------------------------------------------------------

def test_mutant_null_value_caught_by_a_not_g(tmp_path):
    """CFG-FZ-3 reverse variant verbatim: 'value = null must be caught
    by A (key path), not by G (range violation)'. Set one leaf to null,
    verify A raises with null_unassigned and detail.key names it.

    This is THE test the CFG-FZ-3 criterion calls out: G would report
    'value out of range' for null and give the wrong operator action;
    A must fire first and name the missing key."""
    tree = _green_common_tree()
    # Blank out common.safety.t_lat_s to simulate 'declared but null'.
    tree["common"]["safety"]["t_lat_s"] = None
    root = _write_config_tree(tmp_path / "configs", tree)
    with pytest.raises(XbrainError) as ei:
        a_run({"config_root": root})
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["kind"] == "null_unassigned"
    # detail.key names the offending path (dotted).
    assert ei.value.detail["key"] == "common.safety.t_lat_s"


# ---------------------------------------------------------------------------
# Mutant A-2: residual ${...} that cannot resolve -> A reports unresolved_ref
# ---------------------------------------------------------------------------

def test_mutant_unresolved_ref_caught_by_a(tmp_path):
    """Inject a ${...} pointing at a non-existent key. A's resolve()
    call must raise ReferenceError_ and A wraps it into
    detail.kind='unresolved_ref'."""
    tree = _green_common_tree()
    # motion.profiles.patrol.max_mps was 2.0; replace with a reference
    # to a key that doesn't exist anywhere.
    tree["common"]["motion"]["profiles"]["patrol"]["max_mps"] = "${common.does_not_exist}"
    root = _write_config_tree(tmp_path / "configs", tree)
    with pytest.raises(XbrainError) as ei:
        a_run({"config_root": root})
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["kind"] == "unresolved_ref"


# ---------------------------------------------------------------------------
# Mutant M: delete common.safety.t_lat_s row entirely -> M reports missing
# ---------------------------------------------------------------------------

def test_mutant_delete_safety_t_lat_s_is_red_by_m(tmp_path):
    """CFG-FZ-3 variant verbatim: 'delete common.safety.t_lat_s row
    (not null-write, actual delete) -> whole stack must refuse to
    start'. This is the case A cannot catch: no null anywhere,
    just a missing row."""
    tree = _green_common_tree()
    # Row-level delete -- key doesn't appear in the file at all.
    del tree["common"]["safety"]["t_lat_s"]
    root = _write_config_tree(tmp_path / "configs", tree)
    # A passes: no nulls, no unresolved refs -- there's just a hole in
    # the coverage that A cannot see (see 10 S5.4.4 M-row commentary).
    ctx = {"config_root": root}
    a_result = a_run(ctx)
    assert a_result["status"] == "pass", "A should pass -- no null present"
    # M catches it.
    with pytest.raises(XbrainError) as ei:
        m_run(ctx)
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["kind"] == "required_key_missing"
    assert ei.value.detail["key"] == "common.safety.t_lat_s"


# ---------------------------------------------------------------------------
# Cross-check: M's required list must include the CFG-FZ-3 variant target
# ---------------------------------------------------------------------------

def test_m_required_list_contains_variant_target():
    """CFG-FZ-3 names common.safety.t_lat_s as the variant target.
    If a future edit drops it from _REQUIRED_KEYS, the delete-mutation
    test above passes (because the row is no longer required) and the
    variant would silently stop firing. This test guards the guard."""
    assert "common.safety.t_lat_s" in _REQUIRED_KEYS


# ---------------------------------------------------------------------------
# Delete an OTHER required key -> M still catches (list-coverage smoke)
# ---------------------------------------------------------------------------

def test_delete_spec_max_vx_mps_is_red_by_m(tmp_path):
    """A required key other than the CFG-FZ-3 variant target: prove M
    fires for the whole list, not just t_lat_s. Otherwise a broken
    implementation that hard-coded 't_lat_s' would still pass the
    variant test above."""
    tree = _green_common_tree()
    del tree["common"]["spec"]["max_vx_mps"]
    root = _write_config_tree(tmp_path / "configs", tree)
    ctx = {"config_root": root}
    a_run(ctx)
    with pytest.raises(XbrainError) as ei:
        m_run(ctx)
    assert ei.value.detail["key"] == "common.spec.max_vx_mps"


# ---------------------------------------------------------------------------
# Wiring guard: missing ctx['config_root'] -> AssertionError (not Xbrain)
# ---------------------------------------------------------------------------

def test_a_requires_config_root_in_ctx():
    """Same guard shape as J -- missing ctx key is a caller bug."""
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        a_run({})


def test_m_requires_config_root_in_ctx():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        m_run({})
