"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: overrides.py
Brief: CHK-0-56 null-key overrides for the fixture config set (L1/L2 only)

Description:
Real /opt/xbrain_v6/configs/ deliberately leaves 60+ leaf values as
null (CLAUDE.md 3.1 "保持这个形态, 不要为了让它跑起来而填数"). The
fixture cannot fill them in the real tree without violating that
rule, so it materialises a COPY of the tree at test-run and applies
this override map to the copy.

Two invariants this file MUST hold, enforced by
tests/fixtures_meta/test_fixture_integrity.py:

  * every dotted key here targets a leaf that is null in the current
    real /opt/xbrain_v6/configs/ (superset that ignores non-null keys
    would silently mask a real value; sub-set would leak nulls into
    the fixture)
  * NO key here begins with 'common.safety.' -- the safety layer
    (10 S5.4.6 ENV-2) is symlinked from real configs at fixture-build
    time; overriding a safety value would break the same-source
    guarantee (CHK-0-56 criterion iv)

Values are the minimum needed to satisfy freeze assertions A + G
(other assertions land as stubs in the current tree per CFG-FZ-1).
Numeric values are consistent with the 12/13/14 spec ranges so
assertion G's SP-1/SP-2/SP-5/SP-11/AS-7 rows pass.
"""

from __future__ import annotations

from typing import Any, Dict


NULL_OVERRIDES: Dict[str, Any] = {}
# 2026-09-12 (10 S5.4.7, user ruling): the null-filling values moved OUT of this
# dict into configs/*_sim.yaml variant files (common_sim.yaml, models/
# m20s_sim.yaml, p4_agent_sim.yaml, sites/sim.yaml, calib/dev.yaml), which the
# freeze line overlays with --variant sim. This dict stays only as the routing
# target for per-test mutation extras (apply_overrides / _rewrite_yaml_with_
# overrides); it must remain EMPTY -- a value here would shadow a real one
# again (the 1.5 m/s patrol incident), and tests/configs/test_config_variants
# pins that every sim value fills a null.

SAFETY_KEY_PREFIX = "common.safety."


def assert_no_safety_overrides() -> None:
    """CHK-0-56 (iv): NULL_OVERRIDES may not touch common.safety.*.
    Called from the meta test; raises with the offending key on any
    violation."""
    for k in NULL_OVERRIDES:
        if k.startswith(SAFETY_KEY_PREFIX):
            raise AssertionError(
                "CHK-0-56 (iv) violation: NULL_OVERRIDES touches "
                "safety key %r; safety layer must remain same-source "
                "with real configs/safety/ (10 S5.4.6 ENV-2)" % k)


def set_by_path(tree: Dict[str, Any], dotted: str, value: Any) -> None:
    """Set a leaf value in a nested dict by dotted path. Creates
    intermediate dicts as needed."""
    parts = dotted.split(".")
    cur = tree
    for p in parts[:-1]:
        nxt = cur.get(p)
        if not isinstance(nxt, dict):
            nxt = {}
            cur[p] = nxt
        cur = nxt
    cur[parts[-1]] = value


def apply_overrides(tree: Dict[str, Any],
                     extras: Dict[str, Any] = None) -> None:
    """Apply NULL_OVERRIDES to `tree` in place. `extras` is an
    optional per-test override map (used by mutation tests to inject
    bogus values)."""
    for k, v in NULL_OVERRIDES.items():
        set_by_path(tree, k, v)
    if extras:
        for k, v in extras.items():
            set_by_path(tree, k, v)


# ---- Per-proc L6 null fills -----------------------------------------
# 2026-08-10 (V-P4-NULLS): p4_agent.yaml declares 12 undecided keys as
# null per CLAUDE.md 3.1 -- freeze assertion A rejects them, and so
# does load_p4_config at process start. For dev voice-loop testing the
# fixture needs to substitute a non-null placeholder for each so
# freeze can complete AND the P4 process boots. Values here are the
# same 'fixture-not-a-*' + 0.987654 markers used in
# tests/p4_agent/test_config_loader.py::FIXTURE_FILL; a real deploy
# still needs Q-P4-2 / Q-P4-3 / D-AI-1..3 decisions before it can
# ship these values. The keys live in P4's own namespace, so they
# route to p4_agent.yaml (NOT common.yaml) per 10 S5.4.3 (L6 owns
# its own top-level namespace).
P4_L6_FILL: Dict[str, Any] = {}   # moved to configs/p4_agent_sim.yaml (10 S5.4.7)
