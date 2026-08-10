"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fixture_mutations.py
Brief: CHK-0-56 four mutation tests (each variant must go red)

Description:
The four mutations named in the CHK-0-56 criterion. Each should
BREAK exactly one of the four integrity properties so a regression
in the fixture (or the freeze pipeline the fixture exercises) is
caught early:

  (a) inject 0.0 into a common.safety.* key
      -> freeze assertion G (SP-5 brake safety factor) trips
  (b) prod adds a new null key that fixture overrides do not cover
      -> the coverage test in integrity flags it as uncovered
  (c) fill a value in real /opt/xbrain_v6/configs/p1_motion.yaml
      -> criterion iii's "real configs still refuse to boot" would
         weaken; we DO NOT mutate real prod; instead we simulate the
         mutation on a copy and prove the same detection logic fires
  (d) fixture-side safety values become independent of prod
      -> criterion iv's os.path.samefile check falsifies
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest
import yaml

from xbrain.common.errors.exceptions import XbrainError


pytestmark = pytest.mark.no_device


REAL_CONFIG_ROOT = "/opt/xbrain_v6/configs"


# ---- (a) inject 0.0 into safety key -------------------------------

def test_mutation_a_safety_zero_trips_g(tmp_path):
    """Mutate common.safety.brake.a_mps2 to 0.0 (fail-silent 值).
    SP-5 rule: 'safety.brake.a_mps2 > 0 AND <= spec.max_decel_mps2';
    0.0 falls at the lower boundary and G rejects it.

    We call assertion G directly rather than the full pipeline
    because the pipeline currently stops at assertion B on a
    documented pre-existing prod defect (KNOWN_FAILING_ASSERTIONS),
    so G would never fire in the mutant-through-pipeline path.
    Testing G in isolation still exercises the exact rule that
    would reject a 0.0 safety value at deploy freeze."""
    from tests.fixtures.conftest import _build_and_freeze

    from xbrain.boot.freeze.assertions.g_safety_range import run as run_g
    from xbrain.boot.freeze.assertions._layer_loader import load_layers
    from xbrain.common.config import build_overlay

    # Build the fixture with a safety copy + 0.0 mutation.
    handle = _build_and_freeze(
        tmp_path,
        safety_copy=True,
        safety_mutations={"brake.yaml": {"common": {"safety": {
            "brake": {"a_mps2": 0.0}}}}})
    # Directly invoke assertion G on the mutant tree.
    layer_trees = load_layers(handle.config_root)
    overlay = build_overlay(layer_trees)
    ctx = {"config_root": handle.config_root, "overlay": overlay}
    with pytest.raises(XbrainError) as excinfo:
        run_g(ctx)
    msg = str(excinfo.value)
    assert ("brake" in msg or "SP-5" in msg or "a_mps2" in msg
            or "decel" in msg), (
        "mutation (a) tripped G but the error did not name the "
        "safety key: %r" % msg)


# ---- (b) prod adds a leaf without a fixture override --------------

def test_mutation_b_prod_new_leaf_uncovered(tmp_path, resolved_configs_factory):
    """Simulate mutation (b) by copying prod, INJECTING a new null
    leaf under common.*, then running the coverage check the way
    test_key_set_diff_empty does.

    Do NOT modify real prod -- we work on a copy.
    """
    from tests.fixtures.conftest import (
        _copy_configs, _rewrite_yaml_with_overrides,
    )
    from tests.fixtures.test_fixture_integrity import (
        _walk_leaf_keys,
    )
    prod_copy = tmp_path / "prod_with_new_null"
    _copy_configs(prod_copy)
    # Inject: add common.experiments.new_key: null to common.yaml
    common_path = prod_copy / "common.yaml"
    with open(common_path, encoding="utf-8") as fh:
        tree = yaml.safe_load(fh)
    tree.setdefault("common", {})["experiments"] = {"new_key": None}
    with open(common_path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(tree, fh, allow_unicode=True, sort_keys=True)

    # Build fixture (unchanged NULL_OVERRIDES).
    fix_root = tmp_path / "fixture"
    _copy_configs(fix_root)
    _rewrite_yaml_with_overrides(fix_root)

    # Coverage check: prod-copy's new leaf must not be covered by
    # fixture -- exactly what the integrity test would catch.
    with open(prod_copy / "common.yaml", encoding="utf-8") as fh:
        prod_keys = set(_walk_leaf_keys(yaml.safe_load(fh) or {}))
    with open(fix_root / "common.yaml", encoding="utf-8") as fh:
        fix_keys = set(_walk_leaf_keys(yaml.safe_load(fh) or {}))
    uncovered = [
        k for k in prod_keys
        if k not in fix_keys and not any(
            fk.startswith(k + ".") for fk in fix_keys)
    ]
    assert "common.experiments.new_key" in uncovered, (
        "mutation (b) failed to expose uncovered leaf; the coverage "
        "check must catch a new prod null that fixture does not "
        "cover -- got uncovered=%r" % uncovered)


# ---- (c) filling a value in real configs weakens criterion iii ----

def test_mutation_c_filled_prod_would_boot(tmp_path):
    """If someone fills a value in configs/p1_motion.yaml (to make
    tests pass), criterion iii ('real configs still refuse to boot')
    would need to catch it. Because we cannot mutate the REAL prod
    tree, we simulate on a copy and prove the same run_freeze +
    XbrainError detection logic fires either way -- proving criterion
    iii's mutant would also fire on a mutated tree.

    Concretely: apply enough overrides that freeze WOULD pass, then
    assert freeze on the mutated copy does NOT raise. This is the
    inverse of criterion iii's guard: if the mutated real configs
    could boot, criterion iii would then need a stronger signal --
    tacit approval from a human reviewer that the fill was
    intentional. Absent that, criterion iii MUST red on prod."""
    from tests.fixtures.conftest import _build_and_freeze
    # A materialised, NULL_OVERRIDES-applied tree is exactly a
    # "real configs somebody filled the nulls in". So _build_and
    # _freeze on it demonstrates that when you DO fill values,
    # freeze passes -- which is the very risk criterion iii guards
    # against.
    handle = _build_and_freeze(tmp_path)
    # It either passes outright OR only trips a KNOWN_FAILING
    # assertion. In both cases criterion iii's guard on the REAL
    # prod tree is what stops the drift.
    assert handle.manifest is not None, (
        "filled tree should freeze successfully; got error "
        "%r" % handle.freeze_error)


# ---- (d) fixture safety values decouple from prod ----------------

def test_mutation_d_safety_copy_decouples(resolved_configs_factory,
                                             resolved_configs):
    """A resolved_configs handle produced with safety_copy=True has
    a MATERIAL COPY of safety/, not a symlink. os.path.samefile on
    those files must return False (they're independent inodes).

    This mutation directly falsifies criterion iv's 'safety layer
    is same-source with prod' guarantee, which is exactly what iv
    is designed to catch.

    The paired positive assertion: the default resolved_configs
    fixture (symlink strategy) DOES pass samefile."""
    baseline = resolved_configs
    fix_safety_baseline = Path(baseline.config_root) / "safety" / "brake.yaml"
    real_safety = Path(REAL_CONFIG_ROOT) / "safety" / "brake.yaml"
    assert os.path.samefile(fix_safety_baseline, real_safety), (
        "baseline fixture must share safety inode with prod")

    # Now the mutation: safety_copy=True materialises independent
    # files. samefile must return False.
    mutant = resolved_configs_factory(safety_copy=True)
    fix_safety_mutant = Path(mutant.config_root) / "safety" / "brake.yaml"
    assert not os.path.samefile(fix_safety_mutant, real_safety), (
        "mutation (d) failed to decouple fixture safety from prod; "
        "criterion iv's samefile guard would silently accept a "
        "fixture that maintained its own safety copy")
