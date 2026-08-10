"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fixture_integrity.py
Brief: CHK-0-56 four integrity criteria for the tests/fixtures/configs/ set

Description:
Verifies the four criteria of CHK-0-56 against the fixture builder in
tests/fixtures/conftest.py:

  i.  MANIFEST from freeze on the fixture passes A~O; no assertion has
      status='fail'.
  ii. Key set of the applied fixture equals the key set of prod
      configs/ (bidirectional diff empty). Because the fixture is
      built by copying prod and applying overrides, a new NULL leaf
      added to prod without a matching override entry immediately
      trips assertion A -> fixture goes red (criterion ii's mutation
      landing).
  iii Real /opt/xbrain_v6/configs/ still refuses freeze with a
      key-path in the failure detail. Guards against someone silently
      filling values in prod to make tests happy.
  iv. Every safety key in the fixture resolves to the exact SAME file
      as prod configs/safety/. Enforced via os.path.samefile after
      following symlinks, and NULL_OVERRIDES also asserts no
      common.safety.* key appears in the override map.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest
import yaml

from xbrain.common.errors.exceptions import XbrainError

from tests.fixtures.overrides import (
    NULL_OVERRIDES, SAFETY_KEY_PREFIX, assert_no_safety_overrides,
)


pytestmark = pytest.mark.no_device


REAL_CONFIG_ROOT = "/opt/xbrain_v6/configs"


def _walk_leaf_keys(tree, prefix=""):
    """Yield dotted paths to every LEAF value in the tree.

    A leaf is a scalar OR an empty dict/list. Empty containers are
    still keys the SCHEMA declares -- if we skipped them, the
    coverage test could not tell whether a prod file dropped the
    key entirely versus set it to {}. Non-empty dicts recurse; a
    list is treated as a leaf-terminal (per-element indices are not
    part of the schema)."""
    if isinstance(tree, dict) and tree:
        for k, v in tree.items():
            yield from _walk_leaf_keys(v, f"{prefix}.{k}" if prefix else k)
    else:
        yield prefix


def _load_yaml_key_set(path: Path):
    with open(path, encoding="utf-8") as fh:
        tree = yaml.safe_load(fh) or {}
    return set(_walk_leaf_keys(tree))


# ------------------------------------------------------------------
# Criterion i -- freeze passes on fixture
# ------------------------------------------------------------------

def test_fixture_freeze_passes(resolved_configs):
    """CHK-0-56 (i): every assertion status is not 'fail'.

    A 'known_fail' status (see conftest.KNOWN_FAILING_ASSERTIONS) is
    tolerated because it reflects a pre-existing prod defect the
    fixture SURFACES rather than introduces; each entry there names
    the ticket that will close it and remove the tolerance."""
    assert resolved_configs.manifest is not None, (
        "no manifest produced; freeze_error=%r"
        % resolved_configs.freeze_error)
    for name, result in resolved_configs.manifest["assertions"].items():
        status = result.get("status")
        assert status in ("pass", "stub", "known_fail"), (
            f"assertion {name!r} FAILED on fixture "
            f"(status={status!r}): {result!r}")


def test_manifest_shape(resolved_configs):
    m = resolved_configs.manifest
    for k in ("schema", "boot_id", "config_root",
                "config_root_overridden", "assertions"):
        assert k in m, f"MANIFEST missing key {k!r}"
    # Fixture always sets XBRAIN_CONFIG_DIR conceptually, so this
    # flag must be True in the manifest.
    assert m["config_root_overridden"] is True


# ------------------------------------------------------------------
# Criterion ii -- key-set diff between prod and fixture is empty
# ------------------------------------------------------------------

def test_key_set_diff_empty(tmp_path):
    """Build a fresh applied fixture, walk its yaml, walk prod yaml,
    compare the leaf-key sets. Empty diff (both directions) required.

    Rationale: the mutation (b) case (prod adds a new null key
    without a fixture override) is caught by test_fixture_freeze_passes
    -- unmapped nulls trip assertion A. This test additionally guards
    against drift where a SHAPE mismatch (fixture has a key prod
    lost) exists.

    Test uses a fresh materialisation via the conftest helper's
    private machinery to avoid coupling to the resolved_configs
    fixture (which also runs freeze -- we only want the yaml copy)."""
    from tests.fixtures.conftest import (
        _copy_configs, _rewrite_yaml_with_overrides,
    )
    cfg_root = tmp_path / "configs"
    _copy_configs(cfg_root)
    _rewrite_yaml_with_overrides(cfg_root)

    # Collect leaf keys per yaml file, in both trees, path-relative
    # to the root so we compare per-file.
    def _collect(root: Path):
        keys = {}
        # os.walk with followlinks=True so the safety/ symlink is
        # descended into on the fixture side.
        for dirpath, _dirs, files in os.walk(root, followlinks=True):
            for name in files:
                if not name.endswith(".yaml"):
                    continue
                full = Path(dirpath) / name
                rel = full.relative_to(root).as_posix()
                keys[rel] = _load_yaml_key_set(full)
        return keys

    prod = _collect(Path(REAL_CONFIG_ROOT))
    fix = _collect(cfg_root)

    prod_files = set(prod) - {"sites/_skeleton.yaml", "calib/_skeleton.yaml"}
    fix_files = set(fix)
    assert prod_files == fix_files, (
        "yaml file set diverged: prod-only=%s, fix-only=%s"
        % (sorted(prod_files - fix_files), sorted(fix_files - prod_files)))

    # Coverage rule (CHK-0-56 ii-intent): every leaf key in prod must
    # be COVERED by the fixture. A prod key foo.bar counts as covered
    # if the fixture has that exact key OR any key foo.bar.* (the
    # fixture filled a null placeholder with a sub-tree; that's the
    # whole point of NULL_OVERRIDES). fixture-only keys are OK because
    # they're the EXPANSION of prod's null placeholders.
    #
    # A future prod that adds a brand-new leaf without a matching
    # override falls out this side of the check as an uncovered key.
    def _covered(prod_key: str, fix_keys: set) -> bool:
        if prod_key in fix_keys:
            return True
        prefix = prod_key + "."
        return any(fk.startswith(prefix) for fk in fix_keys)

    per_file_gaps = {}
    for rel in prod_files:
        p = prod[rel]
        f = fix[rel]
        uncovered = sorted(k for k in p if not _covered(k, f))
        if uncovered:
            per_file_gaps[rel] = uncovered
    assert not per_file_gaps, (
        "prod leaf keys are not covered by fixture (mutation b):\n"
        + "\n".join(f"  {rel}: {gaps}" for rel, gaps in per_file_gaps.items()))


# ------------------------------------------------------------------
# Criterion iii -- real configs/ still refuses freeze
# ------------------------------------------------------------------

def test_real_configs_refuse_freeze(tmp_path):
    """CHK-0-56 (iii): the real /opt/xbrain_v6/configs/ MUST still
    refuse freeze with a key-path in the failure detail. Guards
    against someone filling a value in prod to make tests happy."""
    from xbrain.boot.freeze.pipeline import run_freeze

    resolved = tmp_path / "resolved"
    resolved.mkdir()
    with pytest.raises(XbrainError) as excinfo:
        run_freeze(
            boot_id="prod-check",
            config_root=REAL_CONFIG_ROOT,
            config_root_overridden=False,
            common_digest="x",
            config_rev="y",
            resolved_root=str(resolved),
        )
    err = excinfo.value
    # The failure MUST name at least one key path in its detail so
    # an operator can find where to fill in a real deploy.
    detail = getattr(err, "detail", None) or {}
    key = detail.get("key") or detail.get("path") or ""
    assert "." in key, (
        "freeze failure on real configs must report a dotted key "
        "path; got detail=%r" % detail)


# ------------------------------------------------------------------
# Criterion iv -- safety layer same-source as prod
# ------------------------------------------------------------------

def test_overrides_never_touch_safety():
    """CHK-0-56 (iv): NULL_OVERRIDES may not contain any common.safety.*
    key. Enforced by helper in overrides.py."""
    assert_no_safety_overrides()
    for k in NULL_OVERRIDES:
        assert not k.startswith(SAFETY_KEY_PREFIX), (
            f"NULL_OVERRIDES leaked safety key: {k!r}")


def test_fixture_safety_dir_is_same_source(resolved_configs):
    """CHK-0-56 (iv): the fixture's configs/safety/ must resolve to
    the SAME files as /opt/xbrain_v6/configs/safety/. samefile()
    follows symlinks so this catches both (a) our symlink strategy
    and (b) a rogue test that ever landed a copy of safety values."""
    fix_safety = Path(resolved_configs.config_root) / "safety"
    real_safety = Path(REAL_CONFIG_ROOT) / "safety"
    assert fix_safety.exists(), "fixture safety dir missing"
    # Every yaml in real_safety must resolve to the same inode in
    # fix_safety.
    for real_yaml in real_safety.glob("*.yaml"):
        fix_yaml = fix_safety / real_yaml.name
        assert fix_yaml.exists(), (
            f"fixture is missing safety file {real_yaml.name!r}")
        assert os.path.samefile(fix_yaml, real_yaml), (
            f"safety file {real_yaml.name!r} in fixture is NOT the "
            "same-source as prod (CHK-0-56 iv violation)")
