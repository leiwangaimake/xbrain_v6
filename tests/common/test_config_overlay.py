"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_config_overlay.py
Brief: The L0~L5 overlay axis, one case per rule of 10 S5.4.3

Description:
CFG-CM-7. Every case below names the rule it guards, and the mutation that turns
it red is written next to it -- CLAUDE.md 3.3: an assertion that has never been
red has not been written.

*** The two cases that matter most are not about merging:

  test_l0_may_not_supply_safety_parameters
      L0 used to be allowed to default ANY key. That meant a completely unfilled
      configs/ still started, on hardcoded values, with assertion A unable to see
      it. A robot running values nobody chose is worse than one that refuses to
      start, because nothing reports it.

  test_null_does_not_override_but_missing_is_different
      null means "declared, unassigned"; a key nobody mentioned means nobody
      thought about it. Assertion A reports the first and cannot report the
      second. Collapsing them removes the only signal that a site layer forgot
      to fill something.
"""

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common import config  # noqa: E402
from xbrain.common.config import ConfigLayerError  # noqa: E402


# ── 合并规则(10 S5.4.3"深合并的粒度"表)────────────────────────────────

def test_map_deep_merges_and_later_leaf_wins():
    """map: 递归深合并,叶子标量后者胜;上层不写的键保留下层值."""
    base = {"common": {"a": {"x": 1, "y": 2}, "b": 3}}
    over = {"common": {"a": {"y": 99}}}
    out = config.deep_merge(base, over)
    assert out["common"]["a"] == {"x": 1, "y": 99}, "上层未写的 x 必须保留"
    assert out["common"]["b"] == 3, "上层未提的兄弟键必须保留"


def test_list_is_replaced_wholesale_not_merged():
    """list: * 整表替换(R-5).

    Mutation: make deep_merge extend or zip lists instead => this goes red.
    qos.bindings is order-sensitive, so an element-wise merge would silently
    reorder it and the binding that ends up first is the one that takes effect.
    """
    base = {"qos": {"bindings": ["a", "b", "c"]}}
    over = {"qos": {"bindings": ["z"]}}
    out = config.deep_merge(base, over)
    assert out["qos"]["bindings"] == ["z"], "list 必须整表替换，🚫 不做元素级合并"


def test_null_does_not_override_but_missing_is_different():
    """null 视为"声明键位但未赋值",不覆盖下层;缺失不参与合并.

    Mutation: treat None like any other value => the first assertion goes red and
    common.yaml's enu_origin placeholder would wipe the site layer's real value.
    """
    base = {"common": {"geo": {"enu_origin": {"lat": 31.0}}, "keep": 7}}
    over = {"common": {"geo": {"enu_origin": None}}}
    out = config.deep_merge(base, over)
    assert out["common"]["geo"]["enu_origin"] == {"lat": 31.0}, "null 🚫 不得覆盖下层"
    assert out["common"]["keep"] == 7, "缺失的键不参与合并"

    # null with nothing underneath stays visible as null, so assertion A can
    # report it by path. Dropping it would make an unfilled key indistinguishable
    # from a key nobody declared.
    out2 = config.deep_merge({}, {"common": {"site": {"name": None}}})
    assert out2["common"]["site"]["name"] is None


def test_unassigned_lists_only_declared_but_unfilled_keys():
    r = config.build_overlay({
        "L1": {"common": {"geo": {"enu_origin": None}, "robot_id": "xbrain-01"}},
    })
    assert r.unassigned() == ["common.geo.enu_origin"]
    assert r.get("common.nobody.mentioned.this") is config.MISSING, (
        "未被任何层提及的键必须返回 MISSING 而不是 None —— None 是「已声明未赋值」"
    )


# ── L0 排除(终审 F7 收窄)────────────────────────────────────────────────

@pytest.mark.parametrize("key", [
    "common.safety.brake.a_mps2",
    "common.spec.max_vx_mps",
    "common.motion.profiles",
    "common.fence.soft_margin_min_m",
])
def test_l0_may_not_supply_safety_parameters(key):
    """*** L0 排除四组命名空间.

    Mutation (the one the TODO names): put common.safety.* back into L0's
    allowance => this goes red. With it allowed, an entirely unfilled configs/
    starts the stack on dataclass defaults and assertion A has nothing to report.
    """
    with pytest.raises(ConfigLayerError) as e:
        config.build_overlay({"L0": config.unflatten({key: 2.5})})
    assert key in str(e.value)


def test_l0_may_still_supply_non_safety_defaults():
    """Pairs with the case above.

    An implementation that rejected every L0 key would satisfy the negative test
    alone -- and would also make the stack unstartable. Both directions needed.
    """
    r = config.build_overlay({"L0": {"common": {"log_level": "info"}}})
    assert r.get("common.log_level") == "info"
    assert r.provenance["common.log_level"] == "L0"


# ── 各层命名空间允许面 ───────────────────────────────────────────────────

def test_l2_may_not_write_outside_spec_and_motion():
    with pytest.raises(ConfigLayerError):
        config.build_overlay({"L2": config.unflatten({"common.safety.brake.k": 1.2})})


def test_l4b_may_only_write_calib():
    with pytest.raises(ConfigLayerError):
        config.build_overlay({"L4b": config.unflatten({"common.geo.datum": "wgs84"})})
    r = config.build_overlay({"L4b": config.unflatten({"common.calib.rev": 3})})
    assert r.get("common.calib.rev") == 3


def test_layer_order_upper_wins():
    r = config.build_overlay({
        "L1": {"common": {"spec": {"max_vx_mps": 1.0}}},
        "L2": {"common": {"spec": {"max_vx_mps": 2.0}}},
    })
    assert r.get("common.spec.max_vx_mps") == 2.0
    assert r.provenance["common.spec.max_vx_mps"] == "L2"


# ── L5 白名单与 ENV 规则 ─────────────────────────────────────────────────

def test_l5_whitelist_is_exactly_three_and_applies():
    assert config.ENV_WHITELIST == {"XBRAIN_ROBOT_ID", "XBRAIN_SITE_ID",
                                    "XBRAIN_LOG_LEVEL"}
    r = config.build_overlay({"L1": {"common": {"robot_id": "from-file"}}},
                             env={"XBRAIN_ROBOT_ID": "from-env"})
    assert r.get("common.robot_id") == "from-env"
    assert r.provenance["common.robot_id"] == "L5"


def test_l5_whitelist_may_not_be_extended():
    """* 白名单三项,不得扩展.

    Mutation: ignore unknown XBRAIN_* instead of raising => red. Ignoring makes a
    typo like XBRAIN_ROBOTID look like it worked.
    """
    with pytest.raises(ConfigLayerError) as e:
        config.env_overlay({"XBRAIN_ROBOTID": "typo"})
    assert "XBRAIN_ROBOTID" in str(e.value)


def test_config_dir_is_not_the_fourth_whitelist_entry():
    """ENV-4: XBRAIN_CONFIG_DIR 不属于 L5,它在 L0~L5 全部之前求值."""
    assert "XBRAIN_CONFIG_DIR" not in config.ENV_WHITELIST
    assert config.env_overlay({"XBRAIN_CONFIG_DIR": "/tmp"}) == {}, (
        "它决定文件从哪来，🚫 不覆盖任何键值"
    )


@pytest.mark.parametrize("bad", ["relative/path", "", "./configs"])
def test_env1_refuses_non_absolute_config_dir_without_falling_back(bad):
    """*** ENV-1: 非绝对路径即拒,!! 不回退到默认根.

    Mutation: return DEFAULT_CONFIG_ROOT on a bad value => red. Silent fallback is
    the worst outcome here: a fixture path with a typo would run against
    production configuration and pass.
    """
    with pytest.raises(ConfigLayerError):
        config.resolve_config_root({"XBRAIN_CONFIG_DIR": bad})


def test_env1_refuses_absolute_but_nonexistent(tmp_path):
    missing = str(tmp_path / "nope")
    with pytest.raises(ConfigLayerError):
        config.resolve_config_root({"XBRAIN_CONFIG_DIR": missing})
    # And accepts a real one -- otherwise a resolve_config_root that raised on
    # everything would pass both negative cases.
    assert config.resolve_config_root({"XBRAIN_CONFIG_DIR": str(tmp_path)}) == str(tmp_path)


def test_env2_safety_root_never_follows_config_dir(tmp_path):
    """*** ENV-2: safety/ 层永远从编译期固化路径读,不跟随 XBRAIN_CONFIG_DIR.

    Mutation (the second one the TODO names): make safety_root honour the env =>
    red. If it followed, pointing the config root at a fixture would swap the
    safety parameters underneath the running system.
    """
    assert config.safety_root({"XBRAIN_CONFIG_DIR": str(tmp_path)}) == config.layers.SAFETY_ROOT
    assert config.safety_root() == config.layers.SAFETY_ROOT
    assert str(tmp_path) not in config.safety_root({"XBRAIN_CONFIG_DIR": str(tmp_path)})


def test_default_root_is_the_plural_absolute_configs():
    """00 CFG-03: 绝对路径 . 复数 . 不接受符号链接."""
    assert config.layers.DEFAULT_CONFIG_ROOT == "/opt/xbrain_v6/configs"
    assert config.resolve_config_root({}) == config.layers.DEFAULT_CONFIG_ROOT
