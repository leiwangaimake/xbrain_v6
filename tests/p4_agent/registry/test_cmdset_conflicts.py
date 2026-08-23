"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cmdset_conflicts.py
Brief: CFG-BT-19 判据(3)(4) -- 级别逐行一致 与 触发词冲突

Description:
CFG-BT-19 要四条加载期断言. (1)(2)(名字双向差集 / 条数一致)已由
GWY-P4-08 的 check_cs_a1 / check_cs_a2 覆盖; 本文件补(3)(4).

*** 一处语义分叉, 必须先说清楚.
16 S0.5 定义的 CS-A3 / CS-A4 是[mission prompt 的 alternation 约束];
而 CFG-BT-19 判据列里的(3)(4)写的是[级别逐行一致]与[触发词冲突] --
同一组编号下是两套不同的东西. 本轮没有把新的两条硬塞进 check_cs_a3 /
check_cs_a4(那会让同一个名字有两种语义), 而是另起了两个函数名.
归属问题记 NEXT SW-23, 由册主裁.

*** 判据(4)在真配置上[当场抓到三组冲突], 而源头在 18 自己.
"停止喊话" 被 C04 exit_broadcast 与 D12 speak_stop 各自列了;
"待命" 被 A04 hold(L0) 与 C06 standby(L1a) 各自列了 -- 级别都不同;
"开始工作" 被 C05 enter_patrol_mode(L1b) 与 H06 wake(L0) 各自列了.
intents.yaml 是如实照抄的, 冲突是从 18 带进来的.

这类冲突的后果不是报错, 是[随机]: 匹配到哪一条取决于遍历顺序, 而遍历
顺序会随 yaml 行序变化. 操作员说同一句话, 今天停喊话明天停播报, 日志里
两次都显示"匹配成功".

Boundaries: 不裁决那三组该归谁(那是语义问题, 见 NEXT SW-23), 只保证
[新出现的冲突一定红], 且已知三组的清单不会悄悄变长.
"""
from __future__ import annotations

import pathlib
import re

import pytest
import yaml

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[3]
INTENTS = ROOT / "configs" / "intents.yaml"
DOC18 = ROOT / "docs" / "18-语音文本指令集.md"

#: 18 指令表的行: | [装饰]ID | `intent` | 触发词 | 槽位 | 级别 | ...
#:
#: *** 为什么不用 registry/cmdset_extractor.py.
#: 那个提取器今天从 18 里提到 [0 条] -- 它的正则要求 ID 紧贴行首的竖线,
#: 而 18 的表里多数行首列带着 "** D12" 这样的强调标记. 也就是说
#: GWY-P4-08 建的那条 CS-A1/CS-A2 通路今天是[空跑]的: 它拿到空集合,
#: 双向差集于是恒空, 断言恒过.
#: 这条发现记在 NEXT SW-23; 本文件不依赖它, 自己解析.
_ROW = re.compile(
    r"^\|[^|]*?\b([A-Z][0-9]{2})\b[^|]*\|\s*`([a-z_0-9]+)`\s*\|"
    r"[^|]*\|[^|]*\|\s*\**\s*(L[0-9][ab]?)\s*\**\s*\|")


def _cmdset_rows():
    """从 18 解析 [{intent, auth}]."""
    out = []
    for line in DOC18.read_text(encoding="utf-8").split("\n"):
        m = _ROW.match(line)
        if m:
            out.append({"id": m.group(1), "intent": m.group(2),
                        "auth": m.group(3)})
    return out


def _yaml_rows():
    return yaml.safe_load(INTENTS.read_text(encoding="utf-8"))["intents"]


def test_the_parser_sees_the_command_table():
    """守前提: 解析到 0 条时下面每条都会空过或误判.

    这不是假想 -- 既有的 cmdset_extractor 正是处在这个状态(见模块头注),
    而它的断言照样"通过". 一个拿到空集合还报绿的门, 是 CLAUDE.md 3.2
    形态1.
    """
    rows = _cmdset_rows()
    assert len(rows) >= 50, "只从 18 解析到 %d 条指令" % len(rows)
    # 同一个 intent 不该在 18 里出现两次 -- 出现了说明表本身重复登记.
    names = [r["intent"] for r in rows]
    assert len(names) == len(set(names)), (
        "18 里有重复登记的 intent: %s"
        % [n for n in names if names.count(n) > 1][:5])


def test_levels_match_row_by_row():
    """*** 判据(3): 级别行数一致[且逐行一致].

    判据自己点名: "NO 只比对行数的实现能过(2)过不了(3)". 行数是个很弱的
    条件 -- 一条本该 L2 二次确认的指令被写成 L0 直接执行, 行数照样一致,
    而那正是级别这个字段存在的全部意义.

    MUTATION: 把 intents.yaml 里任一条的 auth 改掉 -> 这里红.
    """
    from xbrain.p4_agent.registry.startup_assertions import (
        check_intent_levels_match)

    check_intent_levels_match(_yaml_rows(), _cmdset_rows())


def test_a_level_mismatch_is_caught():
    """反向: 造一条不一致, 必须被抓到.

    没有这条, 一个什么都不比的 check_intent_levels_match 也能让上一条通过.
    """
    from xbrain.p4_agent.registry.startup_assertions import (
        CsAssertionError, check_intent_levels_match)

    rows = dict(_yaml_rows())
    cs = _cmdset_rows()
    victim = next(r["intent"] for r in cs if r["intent"] in rows)
    rows[victim] = dict(rows[victim])
    rows[victim]["auth"] = "L9_NOT_A_REAL_LEVEL"
    with pytest.raises(CsAssertionError):
        check_intent_levels_match(rows, cs)


def test_no_new_trigger_word_conflicts():
    """*** 判据(4): 同一触发词映射到不同意图即拒.

    真配置上今天恰好剩下三组已知冲突(源头在 18, 见模块头注). 本条保证
    [不再多出第四组].
    """
    from xbrain.p4_agent.registry.startup_assertions import (
        check_no_trigger_word_conflict)

    check_no_trigger_word_conflict(_yaml_rows())


def test_a_fresh_conflict_is_rejected():
    """造一组新冲突, 必须红 -- 否则已知清单等于把这条断言整个关掉.

    MUTATION: 把 KNOWN_TRIGGER_CONFLICTS 改成"放过一切" -> 这里红.
    """
    from xbrain.p4_agent.registry.startup_assertions import (
        CsAssertionError, check_no_trigger_word_conflict)

    rows = {k: dict(v) for k, v in _yaml_rows().items()}
    names = sorted(rows)[:2]
    # 给两条不相干的意图塞同一个新词.
    for n in names:
        rows[n]["keywords"] = list(rows[n].get("keywords") or []) + [
            "一个刚造出来的重复触发词"]
    with pytest.raises(CsAssertionError):
        check_no_trigger_word_conflict(rows)


def test_known_conflicts_are_exactly_the_reviewed_set():
    """*** 已知清单必须与复核过的那份完全相等.

    这个口子是本文件唯一的豁免. 多一条就多一处"说同一句话结果随机"的
    指令, 而它读起来与其它行毫无区别 -- 能分辨的只有复核的人.

    MUTATION: 往 KNOWN_TRIGGER_CONFLICTS 里加任意一组 -> 红.
    """
    from xbrain.p4_agent.registry import startup_assertions as sa

    expected = {
        ("停止喊话", "exit_broadcast", "speak_stop"),
        ("待命", "hold", "standby"),
        ("开始工作", "enter_patrol_mode", "wake"),
    }
    def _norm(c):
        return (c[0],) + tuple(sorted(c[1:]))

    actual = {_norm(c) for c in sa.KNOWN_TRIGGER_CONFLICTS}
    assert actual == {_norm(c) for c in expected}, (
        "已知冲突集合与复核过的不一致: 多了 %s, 少了 %s"
        % (sorted(actual - {_norm(c) for c in expected}),
           sorted({_norm(c) for c in expected} - actual)))


def test_every_known_conflict_is_real_in_the_doc():
    """*** 已知清单里的每一条都必须能在 18 里查到[两个]出处.

    一条编造的"已知冲突"会让一个真冲突被永久放过. 所以每组都要能在文档
    里找到那个词确实被两条意图各自列了.

    MUTATION: 往清单里加一组文档里不存在的 -> 红.
    """
    from xbrain.p4_agent.registry import startup_assertions as sa

    doc = DOC18.read_text(encoding="utf-8")
    for word, *intents in sa.KNOWN_TRIGGER_CONFLICTS:
        for intent in intents:
            # 该意图那一行必须真的含这个触发词.
            rows = [l for l in doc.split("\n")
                    if ("`%s`" % intent) in l and word in l]
            assert rows, (
                "已知冲突 (%s, %s) 在 18 里查无实据 -- 编造的豁免会让一个"
                "真冲突被永久放过" % (word, intent))


def test_known_conflicts_still_conflict_in_the_config():
    """反向: 一旦某组被裁决并修掉, 它必须从清单里移走.

    否则豁免会比它的理由活得久, 而下一个读的人无法分辨哪些还在等裁决.
    """
    from xbrain.p4_agent.registry import startup_assertions as sa

    rows = _yaml_rows()
    for word, *intents in sa.KNOWN_TRIGGER_CONFLICTS:
        owners = [n for n in intents
                  if word in (rows.get(n, {}).get("keywords") or [])]
        assert len(owners) >= 2, (
            "%r 在 configs/intents.yaml 里已不再冲突(只剩 %s), "
            "应从 KNOWN_TRIGGER_CONFLICTS 移走" % (word, owners))
