"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_l_bit_exemption.py
Brief: CFG-FZ-11 -- L variant (chassis in skip_items) + baseline + parser

Description:
L runs an intersection between two closed sets:
  LEFT   fatal-level items from 11 S5.1A (doc-parsed or ctx-injected)
  RIGHT  skip_items + non_blocking_items from p2_core.yaml bit.quick

CFG-FZ-11 named variant:
  chassis added to skip_items -> L red (chassis is level=fatal)

Reverse: any non-fatal item in either list passes.
Also tests the doc parser against the real 11-接口契约.md so a doc
edit that broke the table shape surfaces here.
"""

import os
from typing import Any, Dict

import pytest
import yaml

from xbrain.boot.freeze.assertions.l_bit_exemption import (
    _FALLBACK_FATAL_ITEMS, _fatal_items_from_map, _parse_bit_levels,
    run,
)
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Scaffolding
# ---------------------------------------------------------------------------

# Injectable level_map for tests -- avoids depending on the real doc
# being at a specific path when unit-testing.
_TEST_LEVEL_MAP = {
    "chassis": "fatal",
    "rtk": "fatal",
    "heading": "fatal",
    "storage": "degraded",
    "network": "warn",
    "ptz": "degraded",
}


def _make_root(tmp_path, p2_bit_quick=None) -> str:
    """Build a config root with p2_core.yaml carrying the bit.quick block."""
    root = tmp_path / "configs"
    root.mkdir()
    (root / "common.yaml").write_text("common:\n  robot_id: gj-001\n")
    p2 = {"bit": {"quick": p2_bit_quick or {}}}
    (root / "p2_core.yaml").write_text(yaml.safe_dump(p2, allow_unicode=True))
    for name in ("p1_motion.yaml", "p3_task.yaml", "p4_agent.yaml",
                 "p5_gateway.yaml", "quadruped.yaml"):
        (root / name).write_text("# empty stub\n")
    (root / "models").mkdir()
    (root / "safety").mkdir()
    return str(root)


def _ctx(root, **extras):
    """Build ctx dict for L, always injecting the test level map."""
    c = {"config_root": root, "bit_level_map": _TEST_LEVEL_MAP}
    c.update(extras)
    return c


# ---------------------------------------------------------------------------
# Baseline: empty lists pass
# ---------------------------------------------------------------------------

def test_empty_bit_lists_pass(tmp_path):
    """No skip / non_blocking items at all -> intersection empty -> pass."""
    root = _make_root(tmp_path, p2_bit_quick={
        "skip_items": [], "non_blocking_items": []})
    result = run(_ctx(root))
    assert result["status"] == "pass"
    assert result["assertion"] == "L"


def test_non_fatal_items_only_passes(tmp_path):
    """storage (degraded) + network (warn) in skip_items -> pass."""
    root = _make_root(tmp_path, p2_bit_quick={
        "skip_items": ["storage", "network"],
        "non_blocking_items": ["ptz"],
    })
    result = run(_ctx(root))
    assert result["status"] == "pass"
    # Sanity: counts reflect what we injected.
    assert result["skip_items_count"] == 2
    assert result["non_blocking_items_count"] == 1


# ---------------------------------------------------------------------------
# CFG-FZ-11 variant: chassis in skip_items -> L red
# ---------------------------------------------------------------------------

def test_variant_chassis_in_skip_items_is_red(tmp_path):
    """CFG-FZ-11 variant verbatim: chassis (fatal) in skip_items.
    L must refuse startup and print the violating item + level."""
    root = _make_root(tmp_path, p2_bit_quick={
        "skip_items": ["chassis"],
        "non_blocking_items": [],
    })
    with pytest.raises(XbrainError) as ei:
        run(_ctx(root))
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["kind"] == "bit_fatal_exempted"
    # entries carries every violating item + level + list.
    entries = ei.value.detail["entries"]
    assert len(entries) == 1
    assert entries[0]["item"] == "chassis"
    assert entries[0]["level"] == "fatal"
    assert entries[0]["list"] == "skip_items"


def test_fatal_in_non_blocking_items_is_red(tmp_path):
    """Same defect, other list: rtk (fatal) in non_blocking_items."""
    root = _make_root(tmp_path, p2_bit_quick={
        "skip_items": [],
        "non_blocking_items": ["rtk"],
    })
    with pytest.raises(XbrainError) as ei:
        run(_ctx(root))
    assert ei.value.detail["entries"][0]["list"] == "non_blocking_items"
    assert ei.value.detail["entries"][0]["item"] == "rtk"


def test_multiple_fatal_all_reported(tmp_path):
    """Multiple fatal items in different lists -> ALL reported in one raise."""
    root = _make_root(tmp_path, p2_bit_quick={
        "skip_items": ["chassis", "heading"],
        "non_blocking_items": ["rtk"],
    })
    with pytest.raises(XbrainError) as ei:
        run(_ctx(root))
    items = {e["item"] for e in ei.value.detail["entries"]}
    assert items == {"chassis", "heading", "rtk"}


# ---------------------------------------------------------------------------
# Skip semantics: bit block absent -> skipped (dev checkout)
# ---------------------------------------------------------------------------

def test_no_bit_block_passes(tmp_path):
    """p2_core.yaml without bit block -> empty lists -> pass."""
    root = _make_root(tmp_path, p2_bit_quick=None)   # no bit at all
    # Overwrite p2_core.yaml with no bit section.
    p2_path = os.path.join(root, "p2_core.yaml")
    with open(p2_path, "w") as f:
        yaml.safe_dump({"other": {}}, f)
    result = run(_ctx(root))
    assert result["status"] == "pass"


# ---------------------------------------------------------------------------
# Fallback: bit_level_map missing -> uses _FALLBACK_FATAL_ITEMS
# ---------------------------------------------------------------------------

def test_fallback_used_when_no_doc_and_no_ctx(tmp_path):
    """No ctx['bit_level_map'] AND no docs/11-接口契约.md at repo root
    -> falls back to _FALLBACK_FATAL_ITEMS. This is the "shipped
    binary without docs/" edge case."""
    root = _make_root(tmp_path, p2_bit_quick={
        "skip_items": ["chassis"], "non_blocking_items": []})
    # Ctx WITHOUT bit_level_map -> exercises the fallback path.
    ctx = {"config_root": root}
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    # Still catches chassis because _FALLBACK_FATAL_ITEMS includes it.
    assert ei.value.detail["entries"][0]["item"] == "chassis"


# ---------------------------------------------------------------------------
# _parse_bit_levels: test against real doc if present
# ---------------------------------------------------------------------------

def test_doc_parser_extracts_real_levels():
    """Parse the real docs/11-接口契约.md and confirm chassis=fatal,
    storage=degraded, network=warn -- three items sampling all three
    levels."""
    # Find the doc via repo root (test file is at tests/boot/freeze/,
    # 4 parents up = repo root).
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(here, "..", "..", ".."))
    doc = os.path.join(repo_root, "docs", "11-接口契约.md")
    if not os.path.isfile(doc):
        pytest.skip("docs/11-接口契约.md not present in checkout")
    level_map = _parse_bit_levels(doc)
    # If parse returned nothing, the section markers or regex broke.
    assert level_map, "doc parser returned empty; check section marks / regex"
    # Sample three items across three levels.
    assert level_map.get("chassis") == "fatal", level_map.get("chassis")
    assert level_map.get("storage") == "degraded", level_map.get("storage")
    assert level_map.get("network") == "warn", level_map.get("network")


def test_doc_parser_fatal_set_matches_fallback():
    """_FALLBACK_FATAL_ITEMS should equal (or subset) the doc-parsed
    fatal set. If they drift, either the doc changed or the fallback
    is stale -- both need updating in lockstep."""
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(here, "..", "..", ".."))
    doc = os.path.join(repo_root, "docs", "11-接口契约.md")
    if not os.path.isfile(doc):
        pytest.skip("docs/11-接口契约.md not present")
    level_map = _parse_bit_levels(doc)
    if not level_map:
        pytest.skip("doc parser returned empty")
    doc_fatal = _fatal_items_from_map(level_map)
    # Fallback must be a subset of the doc-fatal set. (Doc may add
    # new fatals; fallback lags.)
    missing = _FALLBACK_FATAL_ITEMS - doc_fatal
    assert not missing, (
        "_FALLBACK_FATAL_ITEMS contains items not in doc-fatal: %s"
        % missing
    )


# ---------------------------------------------------------------------------
# Wiring guard
# ---------------------------------------------------------------------------

def test_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run({})
