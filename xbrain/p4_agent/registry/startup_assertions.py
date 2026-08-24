"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: startup_assertions.py
Brief: GWY-P4-08 -- CS-A1..CS-A4 startup consistency assertions

Description:
16 S0.5 CS-A* four assertions run at P4 startup after intents.yaml,
cmdset_18.json, mission prompts are loaded. Every one refuses
process start if it fails.

  CS-A1  every intent NAME in intents.yaml MUST appear in
         cmdset_18.json's 128-intent closed set (no extra intent
         invented in registry)
  CS-A2  count(intents.yaml rows) == count(cmdset_18.json intents)
  CS-A3  each mission prompt's `intent ::= ...` alternation is a
         SUBSET of the intent closed set
  CS-A4  each mission prompt's alternation size + 1 (unknown) <= 5
         (AI-36 hard limit); one break allowed: M4_follow at 6

* CS-A3 has a 3-step transitional implementation (spec verbatim):
    if a prompt references an intent NOT in the closed set, run in
    'warn'-forced mode instead of refuse: log the mismatch, load the
    prompt with the unknown intent DROPPED from the alternation.
    (This is the transitional path while 18 gets updated.)
"""

from __future__ import annotations

from typing import Dict, FrozenSet, Iterable, List, Set


class CsAssertionError(RuntimeError):
    """A CS-A* assertion failed. Rule name in message."""


def check_cs_a1(intent_names: Iterable[str],
                cmdset_closed_set: FrozenSet[str]) -> None:
    """CS-A1: every intent name in intents.yaml must be in cmdset_18."""
    extras = set(intent_names) - cmdset_closed_set
    if extras:
        raise CsAssertionError(
            "CS-A1: intents.yaml contains name(s) NOT in cmdset_18.json: %s "
            "(closed set has %d entries)"
            % (sorted(extras), len(cmdset_closed_set)))


def check_cs_a2(intents_yaml_count: int,
                cmdset_json_count: int) -> None:
    """CS-A2: count equality."""
    if intents_yaml_count != cmdset_json_count:
        raise CsAssertionError(
            "CS-A2: intents.yaml has %d entries, cmdset_18.json has %d "
            "(counts must match)"
            % (intents_yaml_count, cmdset_json_count))


def check_cs_a3(mission_alternation: List[str],
                cmdset_closed_set: FrozenSet[str],
                mission_name: str = "") -> List[str]:
    """CS-A3: alternation MUST be a subset of the closed set.

    Returns the list of dropped intents (transitional warn mode).
    In strict mode a caller would raise on non-empty return."""
    return [i for i in mission_alternation if i not in cmdset_closed_set]


def check_cs_a4(mission_name: str,
                alternation_size: int) -> None:
    """CS-A4: alternation_size + 1 (unknown) <= 5, with one
    documented break: M4_follow = 6."""
    limit = 5
    if mission_name == "M4_follow":
        limit = 6
    if alternation_size + 1 > limit:
        raise CsAssertionError(
            "CS-A4: mission %s alternation=%d + 1 (unknown) > %d "
            "(AI-36 limit; only M4_follow allowed to break at 6)"
            % (mission_name, alternation_size, limit))


# --- CFG-BT-19 判据(3)(4): 级别逐行一致 + 触发词冲突 ------------------
#
# *** 一处必须写明的语义分叉.
# 16 S0.5 定义的 CS-A3/CS-A4 是[mission prompt 的 alternation 约束](上面那
# 两个函数); 而 CFG-BT-19 判据列里的(3)(4)写的是[级别逐行一致]与[触发词
# 冲突]. 两套东西共用同一组编号, 内容不同.
#
# NO 不把新的两条硬塞进 check_cs_a3 / check_cs_a4 -- 那会让同一个名字下
# 有两种语义, 下一个人读 16 S0.5 会以为它在做别的事. 另起两个名字, 并在
# 这里把分叉记下来. 归属问题(是 16 该改还是 TODO 该改)记 NEXT, 由册主裁.


def check_intent_levels_match(yaml_rows, cmdset_rows) -> None:
    """CFG-BT-19 判据(3): 级别行数一致, 且[逐行]级别一致.

    *** 判据自己点了名: "NO 只比对行数的实现能过(2)过不了(3)".
    行数一致是个很弱的条件 -- 两边各有 128 条而其中一条的级别从 L0 变成
    L2, 行数照样一致. 而级别决定的是[要不要人确认]: 一条本该 L2 二次确认
    的指令被当成 L0 直接执行, 那是设计上明确要防的事.

    yaml_rows:    {intent_name: {"auth": ...}}  来自 intents.yaml
    cmdset_rows:  [{"intent": ..., "auth": ...}] 来自 18 的指令集表
    """
    want = {r["intent"]: r.get("auth") for r in cmdset_rows if r.get("intent")}
    have = {name: row.get("auth") for name, row in yaml_rows.items()}
    # 只比两边都有的那些 -- 名字集合的差集是 CS-A1 的事, 这里不重复报,
    # 否则一次改名会让两条断言同时红, 读的人分不清是哪个问题.
    shared = sorted(set(want) & set(have))
    if not shared:
        raise CsAssertionError(
            "CS-LEVEL: intents.yaml and cmdset_18 share no intent name -- "
            "the two are most likely not the same command set")
    bad = [(n, have[n], want[n]) for n in shared if have[n] != want[n]]
    if bad:
        raise CsAssertionError(
            "CS-LEVEL: these intents disagree in auth level with the 18 "
            "command table (intent, intents.yaml, 18): %s" % bad[:8])


#: 18 自身就存在的触发词歧义, 每条附两个出处. 2026-08-23 实测抓出.
#:
#: *** 这三组不是 intents.yaml 抄错了, 是 18 的指令集表里同一个词被两条
#: 意图各自列了. intents.yaml 如实照抄, 于是冲突被原样带了进来.
#: 归谁是[语义裁决]不是代码能定的事(比如"待命"到底是 A04 原地不动, 还是
#: C06 连任务一起挂起), 所以在这里声明而不是擅自改配置. 见 NEXT SW-23.
#:
#: NO 这不是豁免口: 新出现的冲突照样红(下面的实现只放过这三组), 而这三组
#: 一旦被裁决, 必须从这里移走 -- 与 check_affinity 的 PENDING_DOC_DECISION
#: 同一个规矩.
KNOWN_TRIGGER_CONFLICTS = {
    # 18 C04 exit_broadcast "停止喊话 / 退出喊话" 与 D12 speak_stop
    # "别喊了 / 停止喊话 / 停止播报". 两条都是 L0, 效果不同: 前者退出
    # 喊话模式, 后者只停当前这一句.
    ("停止喊话", "exit_broadcast", "speak_stop"),
    # 18 A04 hold "原地待命 / 别动 / 停在这儿 / 待命"(L0) 与
    # C06 standby "待命 / 休息 / 别工作了"(L1a). 级别都不同 --
    # 一个是旁路关键词(VD-6 安全类不依赖模型), 一个要 L1a 确认.
    ("待命", "hold", "standby"),
    # 18 C05 enter_patrol_mode "进入巡逻模式 / 开始工作"(L1b) 与
    # H06 wake "唤醒 / 醒醒 / 开始工作"(L0).
    ("开始工作", "enter_patrol_mode", "wake"),
}


def check_no_trigger_word_conflict(yaml_rows) -> None:
    """CFG-BT-19 判据(4): 同一触发词映射到不同意图即拒.

    *** 冲突的现实后果不是报错, 是[随机].
    两条意图共用一个触发词时, 匹配到哪一条取决于遍历顺序 -- 而遍历顺序会
    随 yaml 的行序, dict 的插入顺序变化. 操作员说同一句话, 今天开灯明天
    停车, 而日志里两次都显示"匹配成功".

    所以这条必须在[加载期]拒绝, 而不是运行期挑一个.
    """
    owner = {}
    conflicts = []
    for name in sorted(yaml_rows):
        for word in (yaml_rows[name].get("keywords") or []):
            key = str(word).strip()
            if not key:
                continue
            if key in owner and owner[key] != name:
                conflicts.append((key, owner[key], name))
            else:
                owner.setdefault(key, name)
    # 已知的三组按出处放过(见 KNOWN_TRIGGER_CONFLICTS 的说明), 其余一律拒.
    # 归一成有序对再比, 免得 (a,b) 与 (b,a) 被当成两回事.
    def _norm(c):
        return (c[0],) + tuple(sorted(c[1:]))

    known = {_norm(c) for c in KNOWN_TRIGGER_CONFLICTS}
    fresh = [c for c in conflicts if _norm(c) not in known]
    if fresh:
        raise CsAssertionError(
            "CS-TRIGGER: these trigger words are shared by multiple intents; "
            "the match depends on traversal order: %s" % fresh[:8])
