"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_config_refs.py
Brief: The ${common.*} reference axis, one case per rule R-1 ~ R-7

Description:
CFG-CM-8. Each case names its rule and the mutation that turns it red.

*** Two cases carry more weight than the rest:

  test_r3_has_no_default_syntax
      "fall back to a default when the reference misses" is the silent drift
      CFG-40 exists to kill. The TODO names this exact mutation.

  test_r7_anchors_are_caught_in_raw_text
      Anchors are resolved by the YAML parser, so a check written against the
      parsed tree passes on every file that uses them -- worse than no check.
"""

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common.config import refs, unflatten  # noqa: E402
from xbrain.common.config.layers import ConfigLayerError  # noqa: E402

# The S5.4.5 alias table, as used by R-6. Kept here rather than parsed from the
# document because CFG-CM-8's job is the reference axis; binding this list to
# 10 S5.4.5 by metatest belongs to the alias-table item.
ALIAS_BLACKLIST = [
    "margin_soft_m", "soft_margin_m", "fence_margin_m",
    "dedup_min_dist_m", "point_min_dist_m", "sample_min_dist_m",
    "db_path", "events_db", "origin", "enu_ref", "datum_origin",
    "profiles", "profile_table", "speed_profiles",
    "keyword_rules", "intents_path",
    "a_brake_mps2", "decel_mps2", "brake_decel",
    "safety_factor", "brake_safety",
    "t_latency_s", "t_lat",
    "h_camera_m", "cam_height", "ptz_height_m",
]


# ── R-1 引用必须独占整个标量节点 ─────────────────────────────────────────

def test_r1_whole_node_reference_is_accepted():
    tree = unflatten({"common.a": 1.0, "common.b": "${common.a}"})
    assert refs.resolve(tree)["common"]["b"] == 1.0


@pytest.mark.parametrize("bad", [
    "robot-${common.robot_id}",
    "${common.a} ",
    "prefix ${common.a} suffix",
])
def test_r1_string_interpolation_is_rejected(bad):
    """Mutation: accept a reference anywhere in the string => red.

    Interpolation silently degrades an object or int into a string; the schema
    check downstream then sees the wrong type and the cause is three steps away.
    """
    with pytest.raises(ConfigLayerError) as e:
        refs.resolve(unflatten({"common.a": 1, "common.x": bad}))
    assert "R-1" in str(e.value)


# ── R-2 路径只能以 common. 开头 ──────────────────────────────────────────

def test_r2_cross_process_reference_is_rejected():
    with pytest.raises(ConfigLayerError) as e:
        refs.resolve(unflatten({"common.x": "${p2_core.mode.min_dwell_s}"}))
    assert "R-2" in str(e.value)


# ── R-3 不提供默认值语法 ─────────────────────────────────────────────────

@pytest.mark.parametrize("bad", ["${common.missing:-2.5}", "${common.missing?}"])
def test_r3_has_no_default_syntax(bad):
    """*** Mutation the TODO names: implement ${a:-b} as "fall back to b" => red."""
    with pytest.raises(ConfigLayerError) as e:
        refs.resolve(unflatten({"common.x": bad}))
    assert "R-3" in str(e.value)


def test_r3_unresolvable_reference_refuses_to_start():
    """Unresolvable is E_CONFIG_INVALID -- !! never a default, never a None."""
    with pytest.raises(ConfigLayerError) as e:
        refs.resolve(unflatten({"common.x": "${common.nowhere}"}))
    assert e.value.code == "E_CONFIG_INVALID"
    assert "R-3" in str(e.value)


# ── R-4 链长 ≤ 3,且无表达式与算术 ───────────────────────────────────────

def test_r4_alias_chain_within_limit_resolves():
    tree = unflatten({"common.a": 5, "common.b": "${common.a}", "common.c": "${common.b}"})
    assert refs.resolve(tree)["common"]["c"] == 5


def test_r4_chain_too_long_is_rejected():
    tree = unflatten({"common.a": 5, "common.b": "${common.a}",
                      "common.c": "${common.b}", "common.d": "${common.c}",
                      "common.e": "${common.d}"})
    with pytest.raises(ConfigLayerError) as e:
        refs.resolve(tree)
    assert "R-4" in str(e.value)


@pytest.mark.parametrize("bad", ["${common.a + common.b}", "${2 * common.a}",
                                 "${common.a/2}"])
def test_r4_no_arithmetic(bad):
    """* The contract's reason: a calculator in the loader hides safety logic
    inside a configuration file.

    So fence_close_tol_m = 2 x min_dist_m is NOT expressed here -- assertion C
    checks that relation instead.
    """
    with pytest.raises(ConfigLayerError) as e:
        refs.resolve(unflatten({"common.a": 1, "common.b": 2, "common.x": bad}))
    assert "R-4" in str(e.value) or "R-2" in str(e.value)


def test_r4_cycle_does_not_hang():
    """Cycle reporting in full belongs to the cycle-detection item; here we only
    refuse to loop forever."""
    with pytest.raises(ConfigLayerError):
        refs.resolve(unflatten({"common.a": "${common.b}", "common.b": "${common.a}"}))


# ── R-5 与覆盖轴一致 ─────────────────────────────────────────────────────

def test_r5_lists_pass_through_reference_axis_untouched():
    """R-5 is enforced in merge.py; this asserts the reference axis does not
    quietly re-open it by walking into list elements."""
    tree = {"common": {"qos": {"bindings": ["a", "b"]}}}
    assert refs.resolve(tree)["common"]["qos"]["bindings"] == ["a", "b"]


# ── R-6 L6 的两条禁止(契约称其为七条中唯一有执行力的一条)───────────────

def test_r6_l6_may_not_carry_a_common_top_level_key():
    with pytest.raises(ConfigLayerError) as e:
        refs.check_l6({"common": {"safety": {"brake": {"k": 1.2}}}}, ALIAS_BLACKLIST)
    assert "R-6" in str(e.value)


@pytest.mark.parametrize("banned", ["a_brake_mps2", "profiles", "t_lat", "h_camera_m"])
def test_r6_l6_may_not_use_an_aliased_private_name(banned):
    with pytest.raises(ConfigLayerError) as e:
        refs.check_l6(unflatten({f"speed_gate.{banned}": 2.5}), ALIAS_BLACKLIST)
    assert "R-6" in str(e.value)


def test_r6_ordinary_private_keys_are_allowed():
    """Pairs with the two above: a check_l6 that rejected everything would pass
    both negative cases and make every process config invalid."""
    refs.check_l6(unflatten({"p1_motion.loop_hz": 20,
                             "p1_motion.brake.decel": "${common.safety.brake.a_mps2}"}),
                  ALIAS_BLACKLIST)


# ── R-7 锚点必须扫原始文本 ───────────────────────────────────────────────

def test_r7_anchors_are_caught_in_raw_text():
    """*** Mutation: check the parsed tree instead of the text => red.

    The parser resolves anchors, so by the time you hold a dict they are gone
    and every file using them looks clean.
    """
    with pytest.raises(ConfigLayerError) as e:
        refs.check_no_anchors("safety_distance:\n  d_safe_m: &d_safe 1.00\n", "p1.yaml")
    assert "R-7" in str(e.value)

    with pytest.raises(ConfigLayerError) as e:
        refs.check_no_anchors("corridor:\n  margin_base_m: *d_safe\n", "p1.yaml")
    assert "R-7" in str(e.value)


def test_r7_does_not_fire_on_ordinary_text():
    """Pairs with the case above -- a checker that rejected every file would
    satisfy the negative test alone."""
    refs.check_no_anchors("a: 1\nb: \"star * inside a string\"\n# & in a comment\n")


def test_r7_matches_the_real_defect_this_project_had():
    """12 S12 carried exactly this pair until 2026-08-05 (terminal review S25/S30)."""
    real = "safety_distance:\n  d_safe_m: &d_safe 1.00\ncorridor:\n  margin_base_m: *d_safe\n"
    with pytest.raises(ConfigLayerError):
        refs.check_no_anchors(real, "12 S12")


# ── 顺序:引用轴必须在覆盖轴[之后] ─────────────────────────────────────

def test_reference_axis_sees_the_merged_value_not_the_lower_layer():
    """10 S5.4.3: expand only after L0~L5 finish.

    Expanding mid-merge reads a value the site layer has not overridden yet --
    invisible whenever lab and field happen to agree, wrong on one robot.
    """
    from xbrain.common import config
    merged = config.build_overlay({
        "L1": {"common": {"geo": {"enu_origin": None}, "site": {"tag": "lab"}}},
        "L4": {"common": {"geo": {"enu_origin": "field-origin"},
                          "site": {"tag": "field"}}},
    })
    resolved = refs.resolve(config.deep_merge(
        merged.tree, {"common": {"derived": "${common.geo.enu_origin}"}}))
    assert resolved["common"]["derived"] == "field-origin", (
        "引用轴必须看到 L4 覆盖后的值，🚫 不是 L1 的占位"
    )
