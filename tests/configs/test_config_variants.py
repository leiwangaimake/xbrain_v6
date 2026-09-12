"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_config_variants.py
Brief: 10 S5.4.7 variant files -- fill-only, ignored without --variant, never in safety/, registered by base name

Description:
The whole point of the variant scheme is that a sim value can never shadow a
real one again (the 2026-09-12 patrol 1.5 vs 2.0 incident). So the first
test walks every configs/**/X_sim.yaml and requires each leaf it assigns to be
null in X.yaml (VAR-2). The others pin the loader: without a variant the sim
files are invisible (VAR-3), with it they overlay (models/m20s_sim.yaml fills
the V-01 nulls), safety/ refuses a variant (VAR-5), sim site/robot ids point at
committed files (VAR-8), and every variant maps to a registered base (VAR-7).
"""
from __future__ import annotations

import os
import shutil
from pathlib import Path
from typing import Any, Dict, Iterator, Tuple

import pytest
import yaml

from xbrain.boot.freeze.assertions._layer_loader import (VARIANTS, load_l6_files,
                                                         load_layers, variant_of,
                                                         variant_sibling)
from xbrain.common.config.schemas.registry import SCHEMAS, variant_base
from xbrain.common.errors.exceptions import XbrainError

pytestmark = pytest.mark.no_device

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "configs"


def _leaves(tree: Any, prefix: str = "") -> Iterator[Tuple[str, Any]]:
    if isinstance(tree, dict) and tree:
        for k, v in tree.items():
            yield from _leaves(v, "%s.%s" % (prefix, k) if prefix else str(k))
    else:
        yield prefix, tree


_MISSING = object()


def _get(tree: Any, dotted: str, missing=_MISSING):
    cur = tree
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return missing
        cur = cur[part]
    return cur


def _base_state(btree: Any, dotted: str) -> str:
    """How the real file declares a sim leaf: 'null' (a null leaf, or a leaf
    under a null placeholder mapping such as qos.profiles: null), 'value' (a
    landed real value -> the sim must not shadow it), or 'undeclared' (no
    such key at all -> the sim would invent a key nobody consumes)."""
    cur = btree
    for part in dotted.split("."):
        if cur is None:
            return "null"
        if not isinstance(cur, dict) or part not in cur:
            return "undeclared"
        cur = cur[part]
    return "null" if cur is None else "value"


def _variant_files():
    return sorted(p for p in CONFIGS.rglob("*_sim.yaml"))


def test_variant_files_exist_and_are_the_committed_set():
    names = sorted(p.relative_to(CONFIGS).as_posix() for p in _variant_files())
    assert names == ["common_sim.yaml", "models/m20s_sim.yaml", "p4_agent_sim.yaml"]


def variant_violations(btree: Any, vtree: Any) -> list:
    """VAR-2 checker: every leaf a variant assigns must be a null placeholder
    of the base. Returns [(dotted, kind)] with kind in {shadow, undeclared,
    null_value}; empty == compliant."""
    out = []
    for dotted, value in _leaves(vtree):
        if value is None:
            out.append((dotted, "null_value"))
            continue
        state = _base_state(btree, dotted)
        if state == "value":
            out.append((dotted, "shadow"))
        elif state == "undeclared":
            out.append((dotted, "undeclared"))
    return out


def test_variant_checker_catches_shadow_undeclared_and_null():
    """The 2026-09-12 incident in miniature: a sim value over a landed real
    one must be flagged; so must a key the base never declares. mutant: make
    _base_state never answer 'value' -> the shadow case passes -> red."""
    base = {"common": {"motion": {"profiles": {"patrol": {"max_mps": 2.0, "sensors": None}}},
                       "qos": {"profiles": None}}}
    sim = {"common": {"motion": {"profiles": {"patrol": {"max_mps": 1.5, "sensors": ["lidar"]}}},
                      "qos": {"profiles": {"Q1": {"depth": 1}}},
                      "made_up": {"key": 3},
                      "audio": None}}
    kinds = dict(variant_violations(base, sim))
    assert kinds["common.motion.profiles.patrol.max_mps"] == "shadow"
    assert kinds["common.made_up.key"] == "undeclared"
    assert kinds["common.audio"] == "null_value"
    assert "common.motion.profiles.patrol.sensors" not in kinds        # fills a null leaf
    assert "common.qos.profiles.Q1.depth" not in kinds                 # expands a null placeholder


def test_every_sim_leaf_fills_a_null_of_its_base():
    """VAR-2 over the committed tree. mutant: put patrol.max_mps back into
    common_sim.yaml -> red."""
    for vf in _variant_files():
        base = vf.with_name(vf.name.replace("_sim.yaml", ".yaml"))
        assert base.is_file(), "variant %s has no base file" % vf
        btree = yaml.safe_load(base.read_text(encoding="utf-8")) or {}
        vtree = yaml.safe_load(vf.read_text(encoding="utf-8")) or {}
        bad = variant_violations(btree, vtree)
        assert not bad, "%s violates 10 S5.4.7 VAR-2: %s" % (vf.name, bad[:6])


def test_no_variant_in_safety_layer():
    assert not list((CONFIGS / "safety").glob("*_sim.yaml"))


def test_variant_helpers():
    assert VARIANTS == ("sim",)
    assert variant_of("m20s_sim.yaml") == "sim" and variant_of("m20s.yaml") is None
    assert variant_sibling("/x/models/m20s.yaml", "sim") == "/x/models/m20s_sim.yaml"
    with pytest.raises(XbrainError):
        variant_sibling("/x/common.yaml", "prod")


def test_loader_ignores_sim_files_without_a_variant_and_overlays_with_it():
    """VAR-3. mutant: drop the variant_of() skip in _read_dir -> the plain walk
    merges m20s_sim.yaml into production -> red."""
    plain = load_layers(str(CONFIGS))
    assert _get(plain["L2"], "common.spec.max_wz_radps") is None
    assert _get(plain["L1"], "common.site_id") is None
    sim = load_layers(str(CONFIGS), variant="sim")
    assert _get(sim["L2"], "common.spec.max_wz_radps") == 1.2
    assert _get(sim["L2"], "common.spec.max_vx_mps") == 2.0          # real value, untouched
    assert _get(sim["L1"], "common.site_id") == "sim"
    assert _get(sim["L1"], "common.motion.profiles.patrol.max_mps") == 2.0   # NOT 1.5
    l6 = load_l6_files(str(CONFIGS), variant="sim")
    assert _get(l6["p4_agent.yaml"], "grammar.max_enum_items") == 7
    assert _get(load_l6_files(str(CONFIGS))["p4_agent.yaml"], "grammar.max_enum_items") is None


def test_safety_layer_refuses_a_variant_file(tmp_path):
    root = tmp_path / "configs"
    shutil.copytree(CONFIGS, root, symlinks=True)
    (root / "safety" / "brake_sim.yaml").write_text("common:\n  safety:\n    brake: {}\n", encoding="utf-8")
    with pytest.raises(XbrainError):
        load_layers(str(root), variant="sim")
    with pytest.raises(XbrainError):
        load_layers(str(root))              # even without a variant: its presence is the defect


def test_sim_site_and_robot_files_are_committed():
    """VAR-8: the ids common_sim.yaml selects must have their L4 / L4b files."""
    sim = yaml.safe_load((CONFIGS / "common_sim.yaml").read_text(encoding="utf-8"))
    site_id = sim["common"]["site_id"]
    robot_id = sim["common"]["robot_id"]
    assert (CONFIGS / "sites" / ("%s.yaml" % site_id)).is_file()
    assert (CONFIGS / "calib" / ("%s.yaml" % robot_id)).is_file()


def test_every_variant_has_a_registered_base_schema():
    for vf in _variant_files():
        rel = vf.relative_to(CONFIGS).as_posix()
        assert variant_base(rel) in SCHEMAS
