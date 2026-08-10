"""CFG-FZ-16 S22 layer-namespace tests: three variants + baseline."""

from typing import Any, Dict

import pytest

from xbrain.boot.freeze.assertions.s22_layer_namespace import run
from xbrain.common.errors.exceptions import XbrainError


def _green_layers():
    return {
        "L1": {"common": {"robot_id": "xb-001", "site_id": "site-a"}},
        "L2": {"common": {"spec": {"max_vx_mps": 2.0}}},
        "L3": {"common": {"safety": {"d_safe_m": 1.0, "t_lat_s": 0.4}}},
    }


def _green_l6():
    # 2026-08-10: green L6 no longer requires single-top-level-key ==
    # proc name (see s22_layer_namespace.py header note). Multi-top-level
    # is legal per 10 S5.4.3; the shape below matches every currently
    # committed p*.yaml in configs/.
    return {
        "p1_motion.yaml": {"arbitration": {"priorities": {}}, "logging": {}},
        "quadruped.yaml": {"gait": {}, "safety": {}},
    }


def _ctx(layers=None, l6=None):
    c = {"config_root": "/tmp",
         "layer_trees": layers if layers is not None else _green_layers(),
         "l6_trees": l6 if l6 is not None else _green_l6()}
    return c


def test_green_scaffold_passes():
    result = run(_ctx())
    assert result["status"] == "pass"
    assert result["assertion"] == "S22"


# Variant 1a: speed_profiles at top level in L1
def test_variant_1a_top_level_speed_profiles_in_l1_red():
    layers = _green_layers()
    layers["L1"]["speed_profiles"] = {"patrol": {"max_mps": 2.0}}
    with pytest.raises(XbrainError) as ei:
        run(_ctx(layers=layers))
    assert ei.value.detail["kind"] == "layer_ns_violation"
    assert ei.value.detail["layer"] == "L1"


# Variant 1b: speed_profiles at top level in L2 (same defect, different layer)
def test_variant_1b_top_level_speed_profiles_in_l2_red():
    layers = _green_layers()
    layers["L2"]["speed_profiles"] = {"patrol": {"max_mps": 2.0}}
    with pytest.raises(XbrainError) as ei:
        run(_ctx(layers=layers))
    assert ei.value.detail["kind"] == "layer_ns_violation"
    assert ei.value.detail["layer"] == "L2"


# Variant 2: common.safety.brake in L2 (top-level IS common; naive check misses)
def test_variant_2_common_safety_brake_in_l2_red():
    layers = _green_layers()
    layers["L2"]["common"]["safety"] = {"brake": {"a_mps2": 2.5}}
    with pytest.raises(XbrainError) as ei:
        run(_ctx(layers=layers))
    assert ei.value.detail["kind"] == "layer_ns_violation"
    assert ei.value.detail["layer"] == "L2"


# Variant 3: full gait table in both L1 AND L2 -- S22 does NOT catch,
# because both placements are namespace-legal. B is the enforcer.
def test_variant_3_dup_in_l1_l2_s22_still_passes():
    layers = _green_layers()
    layers["L1"]["common"]["motion"] = {"profiles": {"patrol": {"max_mps": 2.0}}}
    layers["L2"]["common"]["motion"] = {"profiles": {"patrol": {"max_mps": 2.0}}}
    result = run(_ctx(layers=layers))
    assert result["status"] == "pass"


# L6 checks
# 2026-08-10 loosening: 10 S5.4.3 forbids only 'common.* top-level' at L6
# (assertion B owns that rule). S22 no longer enforces single-top-level or
# top-level == proc name. The two old red tests below are inverted to
# document the new looser behaviour; the non-dict test stays red.
def test_l6_multi_top_level_is_legal_after_loosening():
    """L6 with multiple top-level keys is legal per 10 S5.4.3.

    Was 'test_l6_wrong_top_level_key_red' before 2026-08-10; the shape
    that used to trip S22 (a proc yaml whose sole top-level key is not
    the proc name) is now a legitimate config layout -- every
    committed p*.yaml uses it.
    """
    l6 = _green_l6()
    l6["p1_motion.yaml"] = {"arbitration": {}, "logging": {}}
    result = run(_ctx(l6=l6))
    assert result["status"] == "pass"


def test_l6_multiple_top_level_keys_is_legal_after_loosening():
    """Multiple top-level keys is legal; only 'common' top-level is
    reserved (assertion B, not S22)."""
    l6 = _green_l6()
    l6["quadruped.yaml"] = {"gait": {}, "safety": {}, "logging": {}}
    result = run(_ctx(l6=l6))
    assert result["status"] == "pass"


def test_l6_non_dict_top_level_red():
    """Non-dict L6 root (list, scalar) is still S22-red. B's flatten()
    would silently accept a list; S22 is the only defender against it."""
    l6 = _green_l6()
    l6["p3_task.yaml"] = ["not", "a", "dict"]
    with pytest.raises(XbrainError) as ei:
        run(_ctx(l6=l6))
    assert ei.value.detail["kind"] == "l6_top_level_wrong"


def test_l6_empty_file_is_legal():
    l6 = _green_l6()
    l6["p5_gateway.yaml"] = {}
    result = run(_ctx(l6=l6))
    assert result["status"] == "pass"


def test_l6_unknown_filename_is_skipped():
    l6 = _green_l6()
    l6["scratch.yaml"] = {"whatever": {"anything": 1}}
    result = run(_ctx(l6=l6))
    assert result["status"] == "pass"


def test_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run({})
