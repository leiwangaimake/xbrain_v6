"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_g_safety_range.py
Brief: CFG-FZ-7 -- assertion G, three named variants + baseline +
       per-rule coverage

Description:
G is registry-driven; each row is one rule (SP-1 / SP-2 / SP-5 /
SP-11 / AS-7). Tests exercise the variants CFG-FZ-7 names verbatim
and add a per-rule coverage row so removing a rule from _REGISTRY
does not silently pass.

CFG-FZ-7 named variants:
  (1) gateway.gpu_token.throttle_speed_mps >= spec.max_vx_mps -> SP-11
  (2) gateway.asr.timeout_s = 30.0 -> AS-7
  (3) safety.t_lat_s = 0.2 -> SP-5

Baseline: a well-filled tree passes every registered rule.
"""

from typing import Any, Dict

import pytest

from xbrain.boot.freeze.assertions.g_safety_range import _REGISTRY, run
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Scaffolding: build a green merged tree that satisfies every rule.
# ---------------------------------------------------------------------------

class _FakeOverlay:
    def __init__(self, tree):
        self.tree = tree


def _green_tree() -> Dict[str, Any]:
    """A merged tree that satisfies every rule in _REGISTRY."""
    return {
        "common": {
            "spec": {
                "max_vx_mps": 2.0,
                "max_vy_mps": 0.3,
                "max_wz_radps": 0.4,
                "max_accel_mps2": 1.0,
                "max_decel_mps2": 2.5,
            },
            "safety": {
                "t_lat_s": 0.4,
                "brake": {"k": 1.5, "a_mps2": 2.5},
            },
            "motion": {
                "profiles": {
                    "obstacle_avoid": {"max_mps": 0.5},
                    "patrol": {"max_mps": 2.0},
                },
            },
            "gateway": {
                "gpu_token": {"throttle_speed_mps": 1.0},   # < max_vx = 2.0
                "asr": {"timeout_s": 3.0},                   # <= 5.0
                "llm": {"timeout_s": 4.5},
                "tts": {"timeout_s": 2.5},
            },
        },
    }


def _make_ctx(tree=None) -> Dict[str, Any]:
    return {
        "config_root": "/tmp/nonexistent",
        "overlay": _FakeOverlay(tree if tree is not None else _green_tree()),
    }


# ---------------------------------------------------------------------------
# Baseline: green tree passes every check
# ---------------------------------------------------------------------------

def test_green_tree_passes_all_rules():
    """Every check in _REGISTRY returns without raising. If this ever
    fails, either the green fixture drifted or a new rule was added
    that the fixture does not satisfy."""
    result = run(_make_ctx())
    assert result["status"] == "pass"
    assert result["assertion"] == "G"
    assert result["checks_run"] == len(_REGISTRY)


# ---------------------------------------------------------------------------
# Variant (1): gpu_token.throttle >= spec.max_vx_mps -> SP-11
# ---------------------------------------------------------------------------

def test_variant_1_sp11_throttle_ge_max_vx_is_red():
    """CFG-FZ-7 variant (1) verbatim: throttle >= spec.max_vx."""
    tree = _green_tree()
    tree["common"]["gateway"]["gpu_token"]["throttle_speed_mps"] = 2.0   # == max_vx
    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(tree))
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["rule"] == "SP-11"
    assert ei.value.detail["key"] == "common.gateway.gpu_token.throttle_speed_mps"


def test_variant_1b_sp11_throttle_gt_max_vx_is_red():
    """Same as variant (1) but with >, not just ==."""
    tree = _green_tree()
    tree["common"]["gateway"]["gpu_token"]["throttle_speed_mps"] = 3.0
    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(tree))
    assert ei.value.detail["rule"] == "SP-11"


# ---------------------------------------------------------------------------
# Variant (2): asr.timeout_s = 30.0 -> AS-7
# ---------------------------------------------------------------------------

def test_variant_2_as7_asr_timeout_is_red():
    """CFG-FZ-7 variant (2) verbatim: asr.timeout_s = 30.0."""
    tree = _green_tree()
    tree["common"]["gateway"]["asr"]["timeout_s"] = 30.0
    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(tree))
    assert ei.value.detail["rule"] == "AS-7"
    assert ei.value.detail["key"] == "common.gateway.asr.timeout_s"
    assert ei.value.detail["value"] == 30.0


def test_as7_covers_llm_and_tts():
    """AS-7 applies to LLM and TTS too (three keys share the 5 s bound)."""
    for key, subtree in (("llm", "llm"), ("tts", "tts")):
        tree = _green_tree()
        tree["common"]["gateway"][subtree]["timeout_s"] = 10.0
        with pytest.raises(XbrainError) as ei:
            run(_make_ctx(tree))
        assert ei.value.detail["rule"] == "AS-7"
        assert ei.value.detail["key"] == "common.gateway.%s.timeout_s" % key


# ---------------------------------------------------------------------------
# Variant (3): t_lat_s = 0.2 -> SP-5
# ---------------------------------------------------------------------------

def test_variant_3_sp5_t_lat_below_min_is_red():
    """CFG-FZ-7 variant (3) verbatim: t_lat_s = 0.2 (< 0.4 minimum)."""
    tree = _green_tree()
    tree["common"]["safety"]["t_lat_s"] = 0.2
    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(tree))
    assert ei.value.detail["rule"] == "SP-5"
    assert ei.value.detail["key"] == "common.safety.t_lat_s"


# ---------------------------------------------------------------------------
# Per-rule coverage
# ---------------------------------------------------------------------------

def test_sp1_max_vx_zero_is_red():
    """SP-1: spec.max_vx_mps = 0 (silent v_max = 0 in speed_gate.f())."""
    tree = _green_tree()
    tree["common"]["spec"]["max_vx_mps"] = 0.0
    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(tree))
    assert ei.value.detail["rule"] == "SP-1"
    assert ei.value.detail["key"] == "common.spec.max_vx_mps"


def test_sp1_negative_is_red():
    """SP-1 also rejects negatives."""
    tree = _green_tree()
    tree["common"]["spec"]["max_accel_mps2"] = -1.0
    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(tree))
    assert ei.value.detail["rule"] == "SP-1"
    assert ei.value.detail["key"] == "common.spec.max_accel_mps2"


def test_sp2_max_vx_below_profile_max_is_red():
    """SP-2: spec.max_vx must cover the fastest profile."""
    tree = _green_tree()
    # patrol.max_mps = 2.0; drop spec.max_vx below it.
    tree["common"]["spec"]["max_vx_mps"] = 1.5
    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(tree))
    assert ei.value.detail["rule"] == "SP-2"


def test_sp5_brake_k_below_one_is_red():
    """SP-5 sub-condition: brake.k must be >= 1.0."""
    tree = _green_tree()
    tree["common"]["safety"]["brake"]["k"] = 0.9
    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(tree))
    assert ei.value.detail["rule"] == "SP-5"
    assert ei.value.detail["key"] == "common.safety.brake.k"


def test_sp5_brake_a_exceeds_max_decel_is_red():
    """SP-5 sub-condition: brake.a_mps2 must be <= spec.max_decel_mps2."""
    tree = _green_tree()
    tree["common"]["safety"]["brake"]["a_mps2"] = 3.0    # > max_decel=2.5
    with pytest.raises(XbrainError) as ei:
        run(_make_ctx(tree))
    assert ei.value.detail["rule"] == "SP-5"
    assert ei.value.detail["key"] == "common.safety.brake.a_mps2"


# ---------------------------------------------------------------------------
# Skip-on-None behaviour: G defers to assertion A on null values
# ---------------------------------------------------------------------------

def test_null_spec_is_skipped():
    """SP-1 skips on None (assertion A already caught it upstream)."""
    tree = _green_tree()
    tree["common"]["spec"]["max_vy_mps"] = None
    # Should NOT raise -- A refused this in production; G skips.
    result = run(_make_ctx(tree))
    assert result["status"] == "pass"


# ---------------------------------------------------------------------------
# Meta-test: registry non-empty; every rule listed in ORD-1 verbatim
# ---------------------------------------------------------------------------

def test_registry_has_all_five_verbatim_rules():
    """Ensure the five CFG-FZ-7 named rules are all in _REGISTRY. A
    future edit that dropped SP-11 (say) would silently make variant
    (1) uncatchable; this meta-test fails first."""
    rules = {row.rule for row in _REGISTRY}
    expected = {"SP-1", "SP-2", "SP-5", "SP-11", "AS-7"}
    assert expected <= rules, "missing rules: %s" % (expected - rules)


def test_registry_order_is_stable():
    """Rule ordering is documented (alphabetical within family).
    Test guards against an accidental reshuffling that would make
    first-fail non-deterministic across runs."""
    rules = [row.rule for row in _REGISTRY]
    # Not full alphabetical (SP-1 < SP-11 < SP-2 in string order); rather
    # numeric within SP-* then AS-*.
    assert rules[0] == "SP-1"
    assert rules[-1] == "AS-7"


# ---------------------------------------------------------------------------
# Wiring guard
# ---------------------------------------------------------------------------

def test_g_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run({})
