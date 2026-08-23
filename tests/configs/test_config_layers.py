"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_config_layers.py
Brief: CFG-CF-5/6/8 -- L6 进程私有 yaml, L6 内容表, nav2 参数三层的落地判据

Description:
10 S5.4.3 的分层规则(R-6)只有一条: L6 层的文件[只引用不定义] common.* --
把 common 顶层键写进 p4_agent.yaml, 会让同一个键在两处有值, 而冻结线展开
引用时取哪一个取决于加载顺序.

这条规则此前没有执行体. 一个违反它的文件跑起来一切正常, 直到某天两处的值
分了叉 -- 而那时的现象是"某个进程的参数和别人不一样", 极难倒推.

*** nav2 那一条(CFG-CF-8)守的是一个安全前提, 不是配置洁癖.
11 S10.3 只允许 spin / backup / wait 三个 behavior, 且 simulate_ahead_time
必须是 0.0. 后者的意思是[不做前向仿真] -- 而 12 S6A 已经查明: 速度门六项
全是线速度, 围栏靠向量投影, Nav2 不跑 costmap, 四层都拦不住纯旋转 wz.
多挂一个 behavior 插件就等于多一条不受旋转许可判定器约束的运动通路.

Boundaries: 只做静态校验(键在不在, 值对不对). 不判断值本身标定得准不准 --
未标定的一律写 null 是设计行为(CLAUDE.md 3.1), 不是缺失.
"""
from __future__ import annotations

import pathlib

import pytest
import yaml

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "configs"

#: CFG-CF-5 点名的六个 L6 进程私有文件.
L6_PROCESS = ("p1_motion", "p2_core", "p3_task", "p4_agent",
              "p5_gateway", "quadruped")

#: CFG-CF-6 点名的八个 L6 内容表.
L6_CONTENT = ("ptz_presets", "speech_presets", "restate_templates",
              "intents", "query_templates", "chitchat",
              "asr_dict", "suspicion_rules")


def _load(name):
    path = CONFIGS / (name + ".yaml")
    if not path.is_file():
        return None, path
    return yaml.safe_load(path.read_text(encoding="utf-8")), path


def test_the_six_process_files_exist():
    """守前提: 文件不在时下面的循环会空过.

    "六个文件都没有 common 顶层键"在零个文件的情况下也成立 -- 那是
    CLAUDE.md 3.2 形态1 最直白的形状.
    """
    missing = [n for n in L6_PROCESS if not (CONFIGS / (n + ".yaml")).is_file()]
    assert not missing, "L6 进程私有配置缺失: %s" % missing


def test_no_l6_file_defines_a_common_top_level_key():
    """*** R-6: L6 只引用不定义 common.*.

    定义在两处的键会在冻结线展开时取到哪一个, 取决于加载顺序 -- 而加载
    顺序不是任何人有意设计的东西.

    MUTATION: 往任一 L6 文件里加一个 common: 顶层键 -> 红.
    """
    bad = []
    for name in L6_PROCESS + L6_CONTENT:
        body, path = _load(name)
        if body is None:
            continue                      # 内容表的缺席由下面那条单独报
        if isinstance(body, dict) and "common" in body:
            bad.append(path.name)
    assert not bad, (
        "这些 L6 文件定义了 common 顶层键(R-6 只允许引用): %s" % bad)


def test_l6_files_do_not_use_blacklisted_private_key_names():
    """断言 B: L6 私有键名不得撞上 10 S5.4.5 的别名黑名单.

    黑名单里的名字是[曾经存在过, 后来改名或删掉的]键. 一个仍在用旧名字的
    配置文件, 加载时不会报错 -- 它只是被忽略, 于是那一项回到默认.
    """
    from xbrain.common.checks.alias_blacklist import (
        ALIAS_BLACKLIST, scan_config_for_alias)

    assert ALIAS_BLACKLIST, "别名黑名单是空的 -- 这条检查什么都不查"
    for name in L6_PROCESS:
        body, path = _load(name)
        if body is None:
            continue
        # 抛即失败, 消息里带键路径.
        scan_config_for_alias(body)


def test_a_blacklisted_alias_is_actually_caught():
    """反向: 造一个用了黑名单键名的配置, 必须被抓到.

    没有这条, 一个什么都不查的 scan_config_for_alias 会让上一条通过.
    """
    from xbrain.common.checks.alias_blacklist import (
        ALIAS_BLACKLIST, scan_config_for_alias)

    victim = sorted(ALIAS_BLACKLIST)[0]
    from xbrain.common.checks.alias_blacklist import AliasKeyFound

    # 用具体异常类而不是裸 Exception: 后者连 ImportError 都算"通过",
    # 而那恰恰是第一版踩的坑(函数名写错, 用例照样绿过一半).
    with pytest.raises(AliasKeyFound):
        scan_config_for_alias({"some_section": {victim: 1}})


# --- CFG-CF-8: nav2 只启三个 behavior --------------------------------

def test_nav2_config_exists():
    """behavior_only.yaml 必须在 -- 它是"只启三个 behavior"这条的落点."""
    assert (CONFIGS / "nav2" / "behavior_only.yaml").is_file(), (
        "configs/nav2/behavior_only.yaml 不存在")


def test_nav2_enables_exactly_spin_backup_wait():
    """*** 11 S10.3: 插件表只有 spin / backup / wait 三项.

    多挂一个 behavior 插件 = 多一条运动通路, 而 12 S6A 已经查明纯旋转
    在现有四层里没有几何门(速度门管线速度 / 围栏靠向量投影 / Nav2 不跑
    costmap). 新插件不会自动获得旋转许可判定.

    MUTATION(判据点名): 把第四个 behavior 插件加进表 -> 必须红并报出多出的
    插件名.
    """
    text = (CONFIGS / "nav2" / "behavior_only.yaml").read_text(encoding="utf-8")
    body = yaml.safe_load(text) or {}
    names = _behavior_plugin_names(body)
    if not names:
        pytest.skip("behavior_only.yaml 还没有插件表(骨架状态)")
    assert set(names) == {"spin", "backup", "wait"}, (
        "behavior 插件表不是恰好三项: 多出 %s, 少了 %s"
        % (sorted(set(names) - {"spin", "backup", "wait"}),
           sorted({"spin", "backup", "wait"} - set(names))))


def _behavior_plugin_names(body):
    """从 nav2 参数树里取 behavior_plugins 列表; 取不到返回空."""
    def walk(node):
        if isinstance(node, dict):
            for key, val in node.items():
                if key in ("behavior_plugins", "recovery_plugins"):
                    if isinstance(val, list):
                        return [str(x) for x in val]
                found = walk(val)
                if found:
                    return found
        elif isinstance(node, list):
            for item in node:
                found = walk(item)
                if found:
                    return found
        return []
    return walk(body)


def test_nav2_simulate_ahead_time_is_zero():
    """*** simulate_ahead_time 必须是 0.0.

    非零意味着 Nav2 会做前向仿真并据此拒绝动作 -- 而它拿到的 costmap 在
    本项目里根本没跑. 一个基于空 costmap 的"仿真通过"是假保证, 比不做
    仿真更危险: 它会让人以为这一层在拦.
    """
    text = (CONFIGS / "nav2" / "behavior_only.yaml").read_text(encoding="utf-8")
    body = yaml.safe_load(text) or {}

    def find(node):
        if isinstance(node, dict):
            for key, val in node.items():
                if key == "simulate_ahead_time":
                    return val
                got = find(val)
                if got is not None:
                    return got
        elif isinstance(node, list):
            for item in node:
                got = find(item)
                if got is not None:
                    return got
        return None

    val = find(body)
    if val is None:
        pytest.skip("behavior_only.yaml 还没有 simulate_ahead_time(骨架状态)")
    assert float(val) == 0.0, (
        "simulate_ahead_time=%r -- 11 S10.3 要求 0.0(不做前向仿真)" % val)


def test_no_costmap_node_is_configured():
    """本项目不跑 costmap. 配置里出现 costmap 节点即为多了一条通路."""
    text = (CONFIGS / "nav2" / "behavior_only.yaml").read_text(encoding="utf-8")
    body = yaml.safe_load(text) or {}
    if not body:
        pytest.skip("behavior_only.yaml 是骨架")
    hits = [k for k in body if "costmap" in str(k).lower()]
    assert not hits, "nav2 配置里出现了 costmap 节点: %s" % hits


# --- CFG-CF-7: sites / calib 两层 ------------------------------------

def test_sites_and_calib_layers_have_a_skeleton():
    """L4 / L4b 两层要有骨架文件.

    骨架的作用不是被读, 是[告诉现场要填哪些键]. 没有骨架时, 现场只能从
    报错信息里一个一个试出来 -- 而那些报错来自启动断言, 一次只报一个.
    """
    for sub in ("sites", "calib"):
        d = CONFIGS / sub
        assert d.is_dir(), "configs/%s/ 不存在" % sub
        files = sorted(d.glob("*.yaml"))
        assert files, "configs/%s/ 里一个 yaml 都没有" % sub


def test_uncalibrated_values_are_null_not_zero():
    """*** CLAUDE.md 3.1 的核心: 未标定写 null, NO 不写 0.0.

    0.0 会被判成"已赋值"而放行, 运行期 v_max = min(..., 0) = 0 --
    机器人不动且无任何报错. 那是 fail-silent, 比 fail-safe 差一个量级.

    这里扫 calib 骨架: 出现 0.0 的标定项就是这个错误的形状.
    """
    import re

    for path in sorted((CONFIGS / "calib").glob("*.yaml")):
        for lineno, line in enumerate(
                path.read_text(encoding="utf-8").split("\n"), 1):
            code = line.split("#", 1)[0]
            # 只看标定量: accuracy / err / xyz / rpy 这类.
            if not re.search(r"(accuracy|_err|xyz|rpy|offset)", code):
                continue
            assert not re.search(r":\s*0\.0*\s*$", code), (
                "%s:%d 标定项写了 0.0 -- 未标定应写 null(3.1): %s"
                % (path.name, lineno, code.strip()))
