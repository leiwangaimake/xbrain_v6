"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_f_qos_and_port.py
Brief: CFG-FZ-6 -- assertion F (QoS static) + F' (port identity), three
       variants + baseline + coverage

Description:
F/F' scaffolding: a full QoS document is more work than the trees in
A/M/B/C/D. We use a helper _build_green_qos() that yields a valid
minimal qos doc (five profiles + three ordered bindings + a fallback
last). Each variant mutates one field.

CFG-FZ-6 named variants:
  (1) xbrain/*/cmd/estop bound to Q3_cmd (block) -> A-5 red
  (2) fallback xbrain/*/** moved to head -> ordering red
  (3) fake peer on 7447 -> F' red (identity_bad)

Reverse: default green qos + stub port probe (returns zenohd_router)
pass both F and F'.
"""

import os
from typing import Any, Dict

import pytest
import yaml

from xbrain.boot.freeze.assertions.f_qos_and_port import (
    _EXPECTED_IDENTITY, _FALLBACK_PATTERN, _stub_port_probe, run,
)
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Scaffolding: minimal-but-valid QoS document
# ---------------------------------------------------------------------------

def _build_green_qos() -> Dict[str, Any]:
    """Return a qos doc that passes both load_qos_table and F's extra
    static checks.

    Profile names + values MUST match FROZEN_PROFILES in
    xbrain/common/zenoh/qos.py -- the loader audits against them.
    Real names: Q0_safety / Q1_rt / Q2_state / Q3_cmd / Q4_stream.
    """
    return {
        "profiles": {
            # DROP + real_time + express + reliable + ring/8.
            "Q0_safety": {
                "congestion_control": "drop",
                "priority": "real_time",
                "reliability": "reliable",
                "express": True,
                "handler": {"kind": "ring", "depth": 8},
            },
            # DROP + real_time + express + best_effort + ring/1.
            "Q1_rt": {
                "congestion_control": "drop",
                "priority": "real_time",
                "reliability": "best_effort",
                "express": True,
                "handler": {"kind": "ring", "depth": 1},
            },
            # DROP + data_high + reliable + ring/4.
            "Q2_state": {
                "congestion_control": "drop",
                "priority": "data_high",
                "reliability": "reliable",
                "express": False,
                "handler": {"kind": "ring", "depth": 4},
            },
            # BLOCK + data + reliable + fifo/256. Only block profile.
            "Q3_cmd": {
                "congestion_control": "block",
                "priority": "data",
                "reliability": "reliable",
                "express": False,
                "handler": {"kind": "fifo", "depth": 256},
            },
            # DROP + interactive_high + best_effort + ring/MISSING.
            # Deployment MUST supply depth (11 S2.4.7). Green fixture
            # provides 10.
            "Q4_stream": {
                "congestion_control": "drop",
                "priority": "interactive_high",
                "reliability": "best_effort",
                "express": False,
                "handler": {"kind": "ring", "depth": 10},
            },
        },
        # Ordered bindings. First-match wins; fallback last.
        "bindings": [
            # Estop keys -> Q0_safety.
            {"match": "xbrain/*/cmd/estop", "profile": "Q0_safety"},
            {"match": "xbrain/*/probe/estop/**", "profile": "Q0_safety"},
            # RT plane -> Q1_rt (drop, not block; A-5 clean).
            {"match": "xbrain/*/rt/**", "profile": "Q1_rt"},
            # Fallback last.
            {"match": _FALLBACK_PATTERN, "profile": "Q3_cmd"},
        ],
    }


def _wrap_qos_in_tree(qos_doc: Dict[str, Any]) -> Dict[str, Any]:
    """Wrap qos into a full overlay tree so F can pull it via _get."""
    return {"common": {"qos": qos_doc}}


class _FakeOverlay:
    """Minimal overlay stand-in for tests that populate ctx by hand."""

    def __init__(self, tree):
        self.tree = tree


def _make_ctx(qos_doc=None, port_probe_fn=None,
              config_root="/tmp/nonexistent") -> Dict[str, Any]:
    """Build a ctx dict for F/F' testing."""
    tree = _wrap_qos_in_tree(qos_doc if qos_doc is not None
                             else _build_green_qos())
    ctx: Dict[str, Any] = {
        "config_root": config_root,
        "overlay": _FakeOverlay(tree),
    }
    if port_probe_fn is not None:
        ctx["port_probe_fn"] = port_probe_fn
    return ctx


# ---------------------------------------------------------------------------
# Reverse baseline
# ---------------------------------------------------------------------------

def test_green_qos_and_stub_probe_passes():
    """Default qos + default port probe (stub returns zenohd_router)
    passes both F and F'."""
    result = run(_make_ctx())
    assert result["status"] == "pass"
    assert result["assertion"] == "F"
    assert result["qos_checked"] is True
    assert result["ports_checked"] == 2


def test_missing_qos_is_skipped():
    """No common.qos in tree -> F skips (M is the required-key check).
    F' still runs and passes."""
    ctx = {
        "config_root": "/tmp/x",
        "overlay": _FakeOverlay({"common": {}}),
    }
    result = run(ctx)
    assert result["status"] == "pass"
    assert result["qos_checked"] is False


# ---------------------------------------------------------------------------
# Variant (1): rt/-covering binding on block profile -> A-5 red
# ---------------------------------------------------------------------------

def test_variant_1_a5_block_on_rt_is_red():
    """CFG-FZ-6 variant (1): bind rt/** to Q3_cmd (block profile).
    QOS-C1 would rescue at resolve() time, but F rejects the binding
    itself -- QOS-C1 is a safety net, not a licence."""
    qos = _build_green_qos()
    # Replace the rt binding with a Q3_cmd (block) binding.
    for b in qos["bindings"]:
        if "rt/" in b["match"]:
            b["profile"] = "Q3_cmd"
            break
    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(qos_doc=qos))
    assert ei.value.code == "E_QOS_VIOLATION"
    assert ei.value.detail["kind"] == "a5_block_on_rt"
    assert ei.value.detail["profile_name"] == "Q3_cmd"


# ---------------------------------------------------------------------------
# Variant (2): fallback moved to head -> ordering red
# ---------------------------------------------------------------------------

def test_variant_2_fallback_at_head_is_red():
    """CFG-FZ-6 variant (2): move xbrain/*/** to bindings[0]. First-
    match-wins would then swallow every specific binding below it."""
    qos = _build_green_qos()
    # Rotate bindings so the fallback ends up first.
    bindings = qos["bindings"]
    fallback = [b for b in bindings if b["match"] == _FALLBACK_PATTERN][0]
    rest = [b for b in bindings if b["match"] != _FALLBACK_PATTERN]
    qos["bindings"] = [fallback] + rest
    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(qos_doc=qos))
    assert ei.value.code == "E_QOS_VIOLATION"
    assert ei.value.detail["kind"] == "binding_order_bad"
    assert ei.value.detail["fallback_index"] == 0
    assert ei.value.detail["expected_index"] == len(qos["bindings"]) - 1


# ---------------------------------------------------------------------------
# Variant (3): fake peer on 7447 -> F' red
# ---------------------------------------------------------------------------

def test_variant_3_fake_peer_on_7447_is_red():
    """CFG-FZ-6 variant (3): inject a probe_fn that returns 'fake_peer'
    for port 7447. F' must fire naming that port."""
    def fake_probe(port: int) -> str:
        if port == 7447:
            return "fake_peer"
        return _EXPECTED_IDENTITY

    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(port_probe_fn=fake_probe))
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["kind"] == "port_identity_bad"
    assert ei.value.detail["port"] == 7447
    assert ei.value.detail["actual"] == "fake_peer"


def test_fake_peer_on_7449_is_red():
    """Same as variant (3) but on port 7449 (RT plane) -- confirms F'
    covers both ports, not just 7447."""
    def fake_probe(port: int) -> str:
        if port == 7449:
            return "some_bridge"
        return _EXPECTED_IDENTITY

    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(port_probe_fn=fake_probe))
    assert ei.value.detail["port"] == 7449
    assert ei.value.detail["actual"] == "some_bridge"


# ---------------------------------------------------------------------------
# F coverage: load_qos_table failures wrapped as qos_table_invalid
# ---------------------------------------------------------------------------

def test_bad_qos_shape_is_qos_table_invalid(tmp_path):
    """load_qos_table raises on a bad doc; F wraps as qos_table_invalid."""
    qos = _build_green_qos()
    # Delete a required binding field to trigger loader failure.
    qos["bindings"][0].pop("profile")
    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(qos_doc=qos))
    assert ei.value.detail["kind"] == "qos_table_invalid"
    assert "underlying" in ei.value.detail


# ---------------------------------------------------------------------------
# _stub_port_probe returns the expected identity
# ---------------------------------------------------------------------------

def test_stub_probe_default_returns_zenohd_router():
    """The default probe is a stub that returns zenohd_router. This is
    documented and deliberate; test guards the doc."""
    assert _stub_port_probe(7447) == _EXPECTED_IDENTITY
    assert _stub_port_probe(7449) == _EXPECTED_IDENTITY


# ---------------------------------------------------------------------------
# Wiring guard
# ---------------------------------------------------------------------------

def test_f_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run({})
