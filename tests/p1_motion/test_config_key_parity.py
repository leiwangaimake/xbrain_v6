"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_config_key_parity.py
Brief: MOT-PM-33 / CHK-1-23 -- p1_motion 配置键与 12 S12 的双向差集

Description:
12 S12 里有一整块 p1_motion.yaml 的 YAML 示例, 而 configs/p1_motion.yaml
今天是[纯注释](TODO 标着"暂缓 0807", 刻意不落值 -- 按 CLAUDE.md 3.1,
未标定写 null 或干脆不写, 都好过填一个猜的数).

两份东西迟早要对上, 而对不上的方式有两种, 都很隐蔽:
  * 配置里少一个键 -> 该项回到代码默认值(而安全参数按 3.1 根本不该有默认);
  * 配置里多一个键 -> 那个键没有任何消费者, 改它不产生任何效果, 而现场会
    以为自己调了参数.

本文件是那条双向差集的执行体. 它今天是 xfail(strict): 配置还没落值, 差集
必然非空. 落值之后摘掉标记, 它立刻开始起作用 -- strict 保证一旦配置落了值
且恰好对上, XPASS 会失败并提醒摘标记.

*** CHK-1-23 的文档侧前提今天不成立, 如实记录.
判据要 "12 S12 的 rns.corridor: 段内出现且仅出现一次 side_hold_ticks /
k_head_per_rad / lambda 三个键". 实测: 这三个键名在 12 全册[零命中].
也就是说 20 S12.2 交办的那三个键还没有被写进 12. 那是文档侧欠账, 不是
代码问题 -- NO 不在这里替 12 补键, 也不写一条恒红的断言. 见 NEXT SW-25.

Boundaries: 只比键路径集合, 不比值 -- 值该是多少是标定的事(SW-6).
"""
from __future__ import annotations

import pathlib
import re

import pytest
import yaml

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOC = ROOT / "docs" / "12-P1运动域详细设计.md"
CFG = ROOT / "configs" / "p1_motion.yaml"

#: 12 S12 里那块 yaml 的识别标志: 第一行是配置文件的绝对路径注释.
_YAML_MARKER = "# /opt/xbrain_v6/configs/p1_motion.yaml"


def _doc_yaml_text():
    """取出 12 S12 里 p1_motion.yaml 那一块的正文.

    用文件路径注释作锚点而不是小节号: 那一块可能被移动到别的小节, 而
    它开头那行路径注释是它自己的身份.
    """
    text = DOC.read_text(encoding="utf-8")
    hits = text.count(_YAML_MARKER)
    if hits != 1:
        raise AssertionError(
            "12 里 %r 命中 %d 次, 应恰为 1" % (_YAML_MARKER, hits))
    start = text.rindex("```yaml", 0, text.index(_YAML_MARKER))
    end = text.index("```", start + 7)
    return text[start + 7:end]


def _key_paths(node, prefix=""):
    """把嵌套 dict 摊成键路径集合(a.b.c)."""
    out = set()
    if isinstance(node, dict):
        for key, val in node.items():
            path = "%s.%s" % (prefix, key) if prefix else str(key)
            out.add(path)
            out |= _key_paths(val, path)
    return out


def _key_paths_from_text(text):
    """按缩进从 yaml 文本直接提键路径, NO 不经 yaml.safe_load.

    *** 为什么不能用标准解析器.
    12 S12 那块 yaml 里用了别名引用(margin_rot_m: *d_safe), 而锚点 &d_safe
    定义在[另一个代码块]里 -- 单独喂给 safe_load 会抛 ComposerError
    "found undefined alias". 文档里那样写是有意的(一个值在两处引用同一个
    定义), 拆块只是排版.
    我们要的只是键路径集合, 值是什么无所谓, 所以按缩进走一遍就够 --
    比"为了能解析而在测试里补一个假锚点"诚实得多.
    """
    out, stack = set(), []
    for raw in text.split("\n"):
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip() or line.strip().startswith("-"):
            continue
        m = re.match(r"^(\s*)([A-Za-z_][A-Za-z0-9_]*)\s*:", line)
        if not m:
            continue
        indent, key = len(m.group(1)), m.group(2)
        while stack and stack[-1][0] >= indent:
            stack.pop()
        path = ".".join([k for _i, k in stack] + [key])
        out.add(path)
        stack.append((indent, key))
    return out


def test_the_doc_yaml_block_parses():
    """守前提: 取不到那块 yaml 时, 下面的差集会拿空集合去比.

    空集合与空配置比对是"完全一致", 于是一个什么都没读到的门报绿.
    """
    text = _doc_yaml_text()
    assert len(text.strip()) > 200, "取到的 yaml 块太短, 锚点可能错位"
    paths = _key_paths_from_text(text)
    assert len(paths) >= 10, "只解析出 %d 个键路径" % len(paths)


def test_the_config_file_exists():
    """文件必须在 -- 它是纯注释是[有意的], 不存在则是另一回事."""
    assert CFG.is_file(), "configs/p1_motion.yaml 不存在"


def test_config_is_intentionally_unvalued_for_now():
    """*** 记录今天的状态, 并让它一旦改变就被发现.

    p1_motion.yaml 现在是纯注释. 这不是遗漏 -- TODO 标着"暂缓 0807",
    而 CLAUDE.md 3.1 说得很清楚: 未标定的值宁可缺席也不要填一个猜的数,
    因为 0.0 会被判成"已赋值"而放行, 运行期 v_max = min(..., 0) = 0,
    机器人不动且无任何报错.

    一旦有人开始落值, 这条会红, 提醒把下面那条 xfail 的标记摘掉.
    """
    body = yaml.safe_load(CFG.read_text(encoding="utf-8"))
    if body:
        pytest.fail(
            "configs/p1_motion.yaml 开始落值了(顶层键 %s) -- "
            "请摘掉 test_config_keys_match_the_doc 的 xfail 标记, "
            "让双向差集真正生效" % sorted(body))


@pytest.mark.xfail(strict=True, reason=(
    "configs/p1_motion.yaml 今天是纯注释(TODO 暂缓 0807), 双向差集必然非空. "
    "落值后摘掉本标记; strict 保证一旦对上会 XPASS 提醒"))
def test_config_keys_match_the_doc():
    """*** MOT-PM-33 的双向差集本体.

    少一个键 -> 该项回到代码默认值(而安全参数按 3.1 不该有默认);
    多一个键 -> 那个键没有消费者, 改它不产生任何效果, 而现场以为调了参数.

    判据点名的例子是 arbitration.priorities.teleop_cloud -- 少了它,
    云端遥控的优先级会取到默认, 而仲裁优先级决定的是"谁能压过谁".
    """
    want = _key_paths_from_text(_doc_yaml_text())
    have = _key_paths(yaml.safe_load(CFG.read_text(encoding="utf-8")) or {})
    missing = sorted(want - have)
    extra = sorted(have - want)
    assert not missing, "配置里缺这些键(会回到代码默认值): %s" % missing[:8]
    assert not extra, "配置里有 12 S12 未登记的键(没有消费者): %s" % extra[:8]


# --- CHK-1-23: rns.corridor 三键的文档侧 ------------------------------

#: 20 S12.2 交办给 12 的三个 rns.corridor 键.
_RNS_CORRIDOR_KEYS = ("side_hold_ticks", "k_head_per_rad", "lambda")


def test_rns_yaml_is_the_single_config_truth_source():
    """*** CHK-1-23 后继: RNS 配置的唯一运行期真源 = configs/rns.yaml.

    v0.2 时代这条哨兵锁的是 _RNS_CORRIDOR_KEYS 三键(side_hold_ticks /
    k_head_per_rad / lambda)"还没进 12", 提醒键迁移时补配置检查. 那个前提
    已被 #20-3 / v1.0 重写全面取代:
      - k_head_per_rad / lambda 是 v0.2 死键, v1.0 重写已作废(两册零命中);
      - RNS 键的[定义处]在 12 S12.0A(#20-3), [运行期真源]是 configs/rns.yaml;
      - 20 S12.2 是消费侧清单(说明性), 不是运行期加载对象.
    用户 2026-09-09: "以现在的 rns.yaml 为准, 删旧版本, yaml 部署在 configs 下".

    本条改为钉住新真源存在且键结构完整, 而非锁一个已作废的键集.
    """
    import yaml
    rns_yaml = ROOT / "configs" / "rns.yaml"
    assert rns_yaml.is_file(), "configs/rns.yaml (运行期真源) 不存在"
    d = yaml.safe_load(rns_yaml.read_text(encoding="utf-8"))
    top = d["rns"]
    # 键结构与 12 S12.0A / 20 S12.2 一致的顶层段:
    for seg in ("route", "speed", "dynamic", "watchdog", "wall_follow",
                "memory", "perception", "class_map"):
        assert seg in top, "configs/rns.yaml 缺 rns.%s 段" % seg
    # 用户四默认值就位, 其余待整定(不在此逐一断言 null -- 那是 config.py 启动
    # 断言的活):
    assert top["route"]["max_deviation_m"] == 10.0
    assert top["route"]["align_dist_m"] == 5.0
    assert top["dynamic"]["stop_dist_m"] == 3.0
    assert top["follow"]["target_min_dist_m"] == 1.0


def test_rns_yaml_runtime_source_is_unique():
    """判据后继: RNS 配置的[运行期]真源唯一 = configs/rns.yaml.

    原判据禁"20 内第二份 YAML", 本意是防同一量两份[运行期]定义. v1.0 重写后
    真源体系变了: 运行期加载对象只有 configs/rns.yaml 一处; 12 S12.0A(定义处
    说明)和 20 S12.2(消费清单)是[文档], 不被运行期加载. 用户 2026-09-09 定:
    以 configs/rns.yaml 为准.

    本条钉住: configs/ 下 rns 配置只有 rns.yaml 一份(无 rns_*.yaml 副本),
    即运行期不会加载到第二份.
    """
    rns_copies = sorted(p.name for p in (ROOT / "configs").glob("rns*.yaml"))
    assert rns_copies == ["rns.yaml"], (
        "configs/ 下出现 rns 配置副本 %s -- 运行期真源必须唯一(configs/rns.yaml)"
        % rns_copies)
