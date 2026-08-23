"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_locked_keys.py
Brief: CFG-BT-18 FSC-LOCK -- 两组 common.motion.* 的 ConfigCommand 一律拒

Description:
判据点名的变异体只有一条, 但它很精确: "下发一条改 margin_lat_m 的
ConfigCommand 必须回 E_CONFIG_LOCKED -- 一个只按断言 E 五组清单判锁的实现
会放行它, 这正是本条要抓的".

那条要害成立的原因是键路径的形状:
  断言 E 的五组是 common.safety.* / common.spec.* / common.motion.profiles /
  common.qos.* / common.fence.*
  而要锁的键是 common.motion.free_space_corridor.margin_lat_m
它在 common.motion 下, 但[不是] common.motion.profiles. 复用五组清单判锁的
实现于是认为它没锁.

所以本文件的第一条用例就是那个变异体, 且额外断言"这个键确实不在五组里" --
否则这条用例可能因为别的原因通过, 而要害没被验证到.

Boundaries: 不判断这两组该不该锁(那是 10 册主的事, TODO 备注里写着断言 E
第三组是否放宽为 common.motion.* 仍在待裁), 只保证[锁住的确实被拒].
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device

#: 10 S5.4.4 断言 E 的五组安全命名空间, 逐字抄在这里[只为对照].
#: NO 实现不许从这份推导 -- 那正是判据要抓的错误.
_ASSERTION_E_GROUPS = (
    "common.safety.",
    "common.spec.",
    "common.motion.profiles",
    "common.qos.",
    "common.fence.",
)


def test_margin_lat_m_is_rejected():
    """*** 判据点名的变异体, 逐字实现.

    这条通过还不够 -- 下一条要证明它不是因为别的原因通过的.
    """
    from xbrain.common.config.locked_keys import check_config_command
    from xbrain.common.errors import E_CONFIG_LOCKED

    key = "common.motion.free_space_corridor.margin_lat_m"
    verdict = check_config_command(key)
    assert verdict is not None, "改走廊横向余量的 ConfigCommand 被放行了"
    assert verdict["code"] == E_CONFIG_LOCKED
    # detail 必须带键路径: 一条 ConfigCommand 可能带多个键, 只说"被锁了"
    # 让运维不知道是哪个触发的.
    assert verdict["key"] == key


def test_that_key_is_genuinely_outside_the_assertion_e_groups():
    """*** 证明上一条验的确实是那个要害.

    如果 margin_lat_m 恰好落在断言 E 的五组里, 那么"复用五组清单的实现
    会放行它"这个前提就不成立 -- 上一条用例仍会通过, 但它验的是别的东西.

    一条前提已经不成立却照样通过的用例, 与一条什么都不验的用例等价.
    """
    key = "common.motion.free_space_corridor.margin_lat_m"
    inside = [g for g in _ASSERTION_E_GROUPS if key.startswith(g)]
    assert not inside, (
        "%s 现在落进了断言 E 的 %s -- 本条判据的要害前提变了, "
        "需要重新评估 CFG-BT-18" % (key, inside))


def test_both_locked_groups_are_covered():
    """两组都要锁, 各取一个代表键."""
    from xbrain.common.config.locked_keys import check_config_command

    for key in ("common.motion.free_space_corridor.margin_rot_m",
                "common.motion.free_space.d_safe_m"):
        assert check_config_command(key) is not None, "%s 没有被锁" % key


def test_unrelated_keys_are_not_locked():
    """*** 反向: 不该锁的必须放行.

    没有这条, 一个"什么都锁"的实现能通过上面每一条 -- 而它会让所有运行期
    配置下发全部失败, 包括那些设计上明确允许改的.
    """
    from xbrain.common.config.locked_keys import check_config_command

    for key in ("common.motion.profiles.patrol.v_max_mps",
                "common.timezone",
                "p4_agent.asr_timeout_s"):
        assert check_config_command(key) is None, "%s 被误锁了" % key


def test_prefix_match_uses_path_segments_not_string_prefix():
    """*** 前缀必须按[段]比, 不按字符串比.

    "common.motion.free_space" 是 "common.motion.free_space_corridor" 的
    字符串前缀. 今天两组都锁, 所以裸 startswith 的结果碰巧对 -- 而一旦
    其中一组解锁, 那个巧合就变成一次错误的放行.

    这里用一个[两组之外, 但字符串上仍以 free_space 开头]的键来验:
    common.motion.free_spaceship_x 不该被锁.

    MUTATION: 把 is_locked 改成裸 startswith 拼接字符串 -> 这里红.
    """
    from xbrain.common.config.locked_keys import check_config_command

    assert check_config_command("common.motion.free_spaceship_x") is None, (
        "一个只是字符串上以 free_space 开头的键被锁了 -- 前缀匹配没有按段比")


def test_the_locked_prefix_table_is_not_derived_from_assertion_e():
    """*** 守本模块存在的理由.

    如果有人把 LOCKED_PREFIXES 改成从断言 E 的五组推导, 那 margin_lat_m
    会重新被放行 -- 而那正是这条判据要抓的. 这里从两侧钉:
      1. 锁表里必须有 free_space_corridor(它不在五组里);
      2. 锁表里的条目不得全部落在五组内(全落在里面就说明它是复述).
    """
    from xbrain.common.config.locked_keys import LOCKED_PREFIXES

    flat = [".".join(p) for p in LOCKED_PREFIXES]
    assert any("free_space_corridor" in f for f in flat), (
        "锁表里没有 free_space_corridor -- 判据点名的那个键会被放行")
    outside = [f for f in flat
               if not any(f.startswith(g.rstrip(".")) for g in _ASSERTION_E_GROUPS)]
    assert outside, (
        "锁表里每一条都落在断言 E 的五组内 -- 那它只是复述, "
        "而 FSC-LOCK 的意义正是在五组之外再兜一层")


def test_the_error_code_comes_from_the_shared_module():
    """CLAUDE.md 3.5: E_* 由 common/errors 导出, NO 不字符串硬编码.

    硬编码的码字会在大小写/前缀上与真码分叉, 而分叉要到联调才发现 --
    那时的现象是"一个码在一侧不认识".
    """
    import ast
    import pathlib

    from xbrain.common.config import locked_keys

    src = pathlib.Path(locked_keys.__file__).read_text(encoding="utf-8")
    tree = ast.parse(src)
    literals = [n.value for n in ast.walk(tree)
                if isinstance(n, ast.Constant) and isinstance(n.value, str)
                and n.value.startswith("E_")]
    assert not literals, "源码里有 E_* 字符串字面量: %s" % literals
