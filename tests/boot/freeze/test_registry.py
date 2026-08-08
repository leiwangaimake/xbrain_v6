"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_registry.py
Brief: CFG-FZ-1 ORD-1 three mutations + framework sanity

Description:
The three ORD-1 mutations the criterion names verbatim:
  ① register a row without adding it to MANIFEST -> forward diff not empty
  ② put a name in MANIFEST that no row backs -> reverse diff not empty
  ③ swap two rows whose declared dep says A must precede M -> validate red

Plus reverse assertions:
  * ASSERT_REGISTRY as committed is topologically valid
  * run_assertions returns exactly one entry per row, in declared order
  * pipeline.build_manifest fills every required MANIFEST field
"""

import dataclasses
import json
import os

import pytest
import yaml

from xbrain.boot.freeze.assertions.j_config_root import _REQUIRED_FILES
from xbrain.boot.freeze.pipeline import (
    MANIFEST_SCHEMA, build_manifest, run_assertions, run_freeze,
)
from xbrain.boot.freeze.registry import (
    ASSERT_REGISTRY, AssertSpec, ordered_assertion_names, registry_names,
    validate_topology,
)


# Extended for CFG-FZ-3 (A/M real bodies): scaffold now writes a filled
# common.yaml + models/ + safety/ so A/M pass green. Framework tests
# that only care about ORD-1 / MANIFEST shape no longer have to know
# A/M schema; they just call this helper.
_GREEN_COMMON_TREE = {
    "common": {
        "robot_id": "gj-001",
        # site_id extended for FV-ORG (CFG-FZ-14) -- names the L4 file.
        "site_id": "site_scaffold",
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
        # geo.enu_origin as null placeholders (L1 shape per FV-ORG-3).
        "geo": {"enu_origin": {"lat": None, "lon": None, "alt": None}},
    },
}

# L4 (sites/site_scaffold.yaml) with real enu_origin -- required by
# FV-ORG-2/-3 (CFG-FZ-14).
_GREEN_L4_TREE = {
    "common": {
        "geo": {"enu_origin": {
            "lat": 31.2301971, "lon": 121.4732683, "alt": 8.4}},
    },
}


def _scaffold_config_ctx(tmp_path):
    """Build a minimal green config tree that passes J + A + M and
    return ctx pointing at it."""
    root = tmp_path / "configs_for_registry_tests"
    root.mkdir()
    # Every J-required file must exist; common.yaml also carries the
    # filled tree M validates. Other required files stay as headers
    # (they're process-scoped and M doesn't check them).
    for name in _REQUIRED_FILES:
        if name == "common.yaml":
            (root / name).write_text(
                yaml.safe_dump(_GREEN_COMMON_TREE, allow_unicode=True)
            )
        else:
            (root / name).write_text("# scaffold for framework tests\n")
    # A/M also load models/ and safety/ directories -- create empty so
    # _read_dir doesn't complain about missing paths.
    (root / "models").mkdir()
    (root / "safety").mkdir()
    # FV-ORG (CFG-FZ-14) needs sites/{site_id}.yaml with real enu_origin.
    sites = root / "sites"
    sites.mkdir()
    (sites / "site_scaffold.yaml").write_text(
        yaml.safe_dump(_GREEN_L4_TREE, allow_unicode=True)
    )
    # calib/ empty (L4b) -- present so J's directory list is complete.
    (root / "calib").mkdir()
    # secrets/ is optional -- skipping it lets J skip its perm check.
    return {"config_root": str(root)}


# --------------------------------------------------------------------------
# Reverse assertions: the committed registry is well-formed
# --------------------------------------------------------------------------

def test_committed_registry_is_topologically_valid():
    """*** Every AssertSpec.depends_on names an earlier assertion. Catches
    a future edit that reorders rows without checking their edges."""
    validate_topology()                          # must not raise


def test_run_assertions_yields_one_result_per_row_in_order(tmp_path):
    """Framework property: iteration order == declaration order, and every
    row's runner produced exactly one entry. Requires a scaffolded ctx
    because J's real body (CFG-FZ-2) now checks config_root."""
    results = run_assertions(_scaffold_config_ctx(tmp_path))
    assert list(results) == list(ordered_assertion_names())
    for spec in ASSERT_REGISTRY:
        assert spec.name in results
        # Stubs emit status=stub; a real runner would emit pass/fail/skip.
        assert "status" in results[spec.name]


def test_every_row_has_a_runner():
    """Never a row without a runner -- run_assertions would raise for it,
    but a mutation that added a `runner=None` new row would fail here at
    load-time before the pipeline ever runs."""
    for spec in ASSERT_REGISTRY:
        assert spec.runner is not None, spec.name


# --------------------------------------------------------------------------
# ORD-1 mutation ③: swap A and M -> topology validation goes red
# --------------------------------------------------------------------------

def test_ord1_mutation_swap_a_and_m_is_caught(monkeypatch):
    """*** Mutation ③: in-place swap so M appears BEFORE A in the registry.
    M declares depends_on=(A,), so validate_topology() must raise -- proof
    that ORD-1 is a real ordering constraint, not just a comment.
    """
    from xbrain.boot.freeze import registry as reg
    rows = list(ASSERT_REGISTRY)
    i_a = next(k for k, s in enumerate(rows) if s.name == "A")
    i_m = next(k for k, s in enumerate(rows) if s.name == "M")
    rows[i_a], rows[i_m] = rows[i_m], rows[i_a]
    monkeypatch.setattr(reg, "ASSERT_REGISTRY", tuple(rows))
    with pytest.raises(AssertionError, match=r"ORD-1"):
        reg.validate_topology()


# --------------------------------------------------------------------------
# ORD-1 mutations ① and ②: bidirectional diff MANIFEST.assertions vs registry
# --------------------------------------------------------------------------

def _diff(reg_names, manifest_assertions):
    """(only-in-registry, only-in-manifest). Empty pair = green."""
    reg_set = set(reg_names)
    man_set = set(manifest_assertions)
    return sorted(reg_set - man_set), sorted(man_set - reg_set)


def test_bidirectional_diff_of_committed_state_is_empty(tmp_path):
    """*** Baseline: run_assertions today produces MANIFEST.assertions whose
    keys equal registry_names(); diff both ways is empty. Every mutation
    below breaks one direction while leaving the other."""
    results = run_assertions(_scaffold_config_ctx(tmp_path))
    only_reg, only_man = _diff(registry_names(), results)
    assert only_reg == [] and only_man == []


def test_ord1_mutation_registry_row_missing_from_manifest_forward_diff(
        monkeypatch, tmp_path):
    """*** Mutation ①: a runner that omits itself (returns nothing usable)
    would leave MANIFEST.assertions short one row. Simulate by returning
    an artefact that DOES NOT include one row's result, and prove the
    diff detects it."""
    real = run_assertions(_scaffold_config_ctx(tmp_path))
    tampered = {k: v for k, v in real.items() if k != "J"}   # drop J
    only_reg, only_man = _diff(registry_names(), tampered)
    assert only_reg == ["J"]
    assert only_man == []


def test_ord1_mutation_manifest_carries_key_no_row_backs_reverse_diff(tmp_path):
    """*** Mutation ②: a MANIFEST that grew a key without a registry row
    behind it. Detected by the reverse diff (manifest \\ registry)."""
    real = run_assertions(_scaffold_config_ctx(tmp_path))
    tampered = dict(real)
    tampered["INVENTED-Z"] = {"status": "pass"}   # not in registry
    only_reg, only_man = _diff(registry_names(), tampered)
    assert only_reg == []
    assert only_man == ["INVENTED-Z"]


# --------------------------------------------------------------------------
# build_manifest / run_freeze framework contracts
# --------------------------------------------------------------------------

def test_build_manifest_populates_every_required_field():
    """MANIFEST must carry every field the S5.4.6 field list + the CFG-FZ-1
    补 (config_rev) names. A caller that omitted one would raise TypeError
    here (all kwargs are required), rather than emit a truncated file."""
    m = build_manifest(
        boot_id="deadbeef",
        config_root="/tmp/x",
        config_root_overridden=False,
        common_digest="cd",
        config_rev="cr",
        calib_rev="unspecified",
        layers=[],
        processes={},
        assertions={"J": {"status": "stub"}},
    )
    for field in ("schema", "boot_id", "config_root",
                  "config_root_overridden", "common_digest", "config_rev",
                  "calib_rev", "layers", "processes", "assertions"):
        assert field in m, field
    assert m["schema"] == MANIFEST_SCHEMA


def test_run_freeze_writes_manifest_json_and_returns_it(tmp_path):
    """End-to-end: run_freeze writes MANIFEST.json under resolved_root and
    returns the same dict. The json on disk parses back to what we
    returned. Uses a scaffolded config_root because CFG-FZ-2 landed
    the real J body which needs the tree to exist."""
    # Scaffold config_root separately from resolved_root so we can
    # pass them to run_freeze without collision.
    ctx_seed = _scaffold_config_ctx(tmp_path)
    resolved = tmp_path / "resolved"
    resolved.mkdir()
    m = run_freeze(
        boot_id="be",
        config_root=ctx_seed["config_root"],
        config_root_overridden=False,
        common_digest="cd",
        config_rev="cr",
        resolved_root=str(resolved),
    )
    manifest_path = resolved / "MANIFEST.json"
    assert manifest_path.exists()
    disk = json.loads(manifest_path.read_text())
    assert disk == m


def test_run_freeze_refuses_missing_resolved_root(tmp_path):
    """CFG-BT-22 owns tmpfs creation; if it is missing we refuse rather
    than mkdir over what might be a mount failure."""
    nonexistent = tmp_path / "does-not-exist"
    with pytest.raises(FileNotFoundError, match=r"CFG-BT-22"):
        run_freeze(
            boot_id="be", config_root="/tmp/x",
            config_root_overridden=False,
            common_digest="cd", config_rev="cr",
            resolved_root=str(nonexistent),
        )


def test_assert_spec_is_frozen():
    """AssertSpec is frozen so a caller cannot mutate a runner at runtime.
    A drift into `dataclass(frozen=False)` would silently allow that."""
    j = next(s for s in ASSERT_REGISTRY if s.name == "J")
    with pytest.raises(dataclasses.FrozenInstanceError):
        j.runner = None                          # type: ignore[misc]
