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
    return {
        "p1_motion.yaml": {"p1_motion": {"foo": 1}},
        "quadruped.yaml": {"quadruped": {"bar": 2}},
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
def test_l6_wrong_top_level_key_red():
    l6 = _green_l6()
    l6["p1_motion.yaml"] = {"p2_core": {"stray": 1}}
    with pytest.raises(XbrainError) as ei:
        run(_ctx(l6=l6))
    assert ei.value.detail["kind"] == "l6_top_level_wrong"
    assert ei.value.detail["expected"] == "p1_motion"
    assert ei.value.detail["actual"] == "p2_core"


def test_l6_multiple_top_level_keys_red():
    l6 = _green_l6()
    l6["quadruped.yaml"] = {"quadruped": {"foo": 1}, "p1_motion": {"bar": 2}}
    with pytest.raises(XbrainError) as ei:
        run(_ctx(l6=l6))
    assert ei.value.detail["kind"] == "l6_multiple_top_levels"


def test_l6_non_dict_top_level_red():
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
