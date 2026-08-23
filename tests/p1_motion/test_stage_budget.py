"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_stage_budget.py
Brief: MOT-PM-35 -- 12 S2.2 拍内逐段时延预算的可执行门禁

Description:
12 S2.2 给了 20 Hz 一拍内的逐段预算(快照 2 ms, 速度门 3 ms, 行为源 25 ms,
仲裁至限幅 5 ms, 输出 1 ms, 合计 36 ms), 并逐字写着"超预算源不许合入主干".

而在本文件之前, [没有任何东西在守这条]. 那句话是一条写进设计册, 却从来
没有实现体的门禁 -- CLAUDE.md 3.2 里"以为有人在守, 其实没有"的那一类.

*** 判据点名的那件事: 注入 +5 ms 必须[只有该行]变红.
逐段预算的价值全在"逐段": 如果计时器挂在错误的粒度上(比如只测总时长),
那么任何一段超预算都表现为总时长超 -- 报出来的是"这一拍慢了", 而不是
"是走廊搜索慢了". 前者在现场没有可操作性.
所以本文件不只测"合计不超", 还测[单段注入只让单段红].

*** 今天能测什么, 不能测什么, 逐条写清楚.
能: 预算表本身的解析与自洽(逐段之和 <= 合计, 合计 <= 周期), 以及计时器
    在人造负载下的逐段归因.
不能: 拿真实的 P1 一拍去量 -- ctrl_loop 今天没有接线(见 CHK-1-04 逐行核实
    与 NEXT SW-21), 没有一拍可量. 那一半标 xfail(strict), NO 不写桩去
    扮演一个不存在的控制环.

Boundaries: 不判断预算数字定得合不合理(那是 12 的事), 只保证[表是自洽的]
且[计时器的粒度是逐段的].
"""
from __future__ import annotations

import pathlib
import re
import time

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOC = ROOT / "docs" / "12-P1运动域详细设计.md"

#: S2.2 的小节锚点, 要求唯一.
_ANCHOR = "### 2.2 控制循环时序"

#: 20 Hz -> 一拍 50 ms. 这个数不是从表里读的, 它是频率的定义.
PERIOD_MS = 50.0


def _budget_rows():
    """从 12 S2.2 解析 [(段名, 预算 ms, 是否小计行)].

    表里混着三种行: 顶层段(1-2 / 3 / 4 / 5-8 / 9), 行为源的细分(带树枝
    符号), 以及合计. 三者不能混为一谈 -- 把细分行也加进"逐段之和"会
    重复计算.
    NO 解析不到就抛: 返回空会让下面每条断言空过.
    """
    text = DOC.read_text(encoding="utf-8")
    hits = text.count(_ANCHOR)
    if hits != 1:
        raise AssertionError("12 S2.2 锚点命中 %d 次, 应恰为 1" % hits)
    body = text[text.index(_ANCHOR) + len(_ANCHOR):]
    nxt = re.search(r"^#{1,4} ", body, re.M)
    if nxt:
        body = body[:nxt.start()]
    rows = []
    for line in body.split("\n"):
        if not line.startswith("|"):
            continue
        cells = line.split("|")
        if len(cells) < 3:
            continue
        name = cells[1].strip()
        budget = cells[2].strip()
        m = re.search(r"([0-9]+(?:\.[0-9]+)?)\s*ms", budget)
        if not m:
            continue
        # 树枝符号(├ └ 　)标记的是行为源的细分行.
        is_sub = bool(re.search(r"[├└　]", name))
        rows.append((re.sub(r"[*\u2605\u251c\u2514\u3000`]", "", name).strip(),
                     float(m.group(1)), is_sub))
    if not rows:
        raise AssertionError("12 S2.2 预算表解析到 0 行 -- 表结构变了")
    return rows


def test_the_budget_table_parses():
    """守前提. 解析到 0 行时下面每条都会空过."""
    rows = _budget_rows()
    assert len(rows) >= 8, "只解析到 %d 行预算" % len(rows)
    tops = [r for r in rows if not r[2]]
    assert len(tops) >= 5, "顶层段只解析到 %d 个" % len(tops)


def test_stage_budgets_sum_within_the_total():
    """*** 逐段之和不得超过表里写的合计.

    表自己就可能不自洽 -- 某一段的预算被上调而合计没跟着改, 那么"每段
    都不超"与"合计不超"就成了两个互相矛盾的条件, 而实现只能满足其中一个.
    """
    rows = _budget_rows()
    total = next((b for n, b, sub in rows
                  if not sub and ("合计" in n or "total" in n.lower())), None)
    assert total is not None, "预算表里没有合计行"
    parts = [b for n, b, sub in rows
             if not sub and "合计" not in n and "total" not in n.lower()]
    assert parts, "除合计外一个顶层段都没有"
    assert sum(parts) <= total + 1e-9, (
        "逐段之和 %.1f ms 超过表里的合计 %.1f ms -- 表本身不自洽"
        % (sum(parts), total))


def test_the_total_fits_in_one_period():
    """合计必须留出余量. 一拍 50 ms 用满就没有任何抖动空间.

    12 S2.2 逐字给的是 36 ms(周期的 72%, 留 14 ms 余量).
    """
    rows = _budget_rows()
    total = next(b for n, b, sub in rows
                 if not sub and ("合计" in n or "total" in n.lower()))
    assert total < PERIOD_MS, (
        "合计 %.1f ms 不小于一拍 %.1f ms -- 没有余量" % (total, PERIOD_MS))
    # 余量至少要有一成; 低于这个数说明预算已经吃满, 任何一次 GC 都会超拍.
    assert total <= PERIOD_MS * 0.9, (
        "合计 %.1f ms 占了一拍的 %.0f%% -- 余量不足"
        % (total, 100.0 * total / PERIOD_MS))


def test_behaviour_sub_budgets_fit_their_parent():
    """行为源的细分之和不得超过它那一段的预算.

    细分行是给实现者用的分配表. 它超了而父段没超, 意味着分配表本身
    在骗人 -- 照着它写的实现必然超父段.
    """
    rows = _budget_rows()
    parent = next((b for n, b, sub in rows if not sub and "行为源" in n), None)
    if parent is None:
        pytest.skip("表里没有行为源那一段")
    subs = [(n, b) for n, b, sub in rows if sub]
    if not subs:
        pytest.skip("表里没有细分行")
    # 小计行不参与求和 -- 它是细分的汇总, 加进去会重复计算.
    addends = [b for n, b in subs if "小计" not in n]
    assert sum(addends) <= parent + 1e-9, (
        "行为源细分之和 %.1f ms 超过该段预算 %.1f ms" % (sum(addends), parent))


# --- 逐段计时器 ------------------------------------------------------

class StageTimer:
    """按段计时, 用单调钟(CLAUDE.md 3.4).

    *** 为什么放在测试里而不是实现里.
    它今天没有消费者: ctrl_loop 没有接线, 没有一拍可以量. 把它放进
    xbrain/ 会是一段没有调用者的代码(9.3 禁止的留口子). 等控制环接起来
    时, 这个类连同下面的断言一起搬过去 -- 那时它才有意义.
    """

    def __init__(self):
        self.spans = {}

    def measure(self, name, fn):
        t0 = time.monotonic()
        fn()
        self.spans[name] = (time.monotonic() - t0) * 1000.0
        return self.spans[name]

    def over_budget(self, budgets):
        """返回超预算的段 [(名字, 实测 ms, 预算 ms)]."""
        return [(n, ms, budgets[n]) for n, ms in sorted(self.spans.items())
                if n in budgets and ms > budgets[n]]


def test_the_timer_attributes_a_single_slow_stage():
    """*** 判据点名的那条: 注入 +5 ms, 必须[只有该段]变红.

    这是在证明计时的[粒度]是对的. 一个只测总时长的实现在这里会让所有段
    一起红(或者一段都不红), 而判据逐字说: "若整表一起红, 说明计时挂在了
    错误的粒度上, 该测本身要重写".
    """
    budgets = {"a": 5.0, "b": 5.0, "c": 5.0}
    timer = StageTimer()
    timer.measure("a", lambda: None)
    timer.measure("b", lambda: time.sleep(0.02))       # 注入 20 ms
    timer.measure("c", lambda: None)
    over = timer.over_budget(budgets)
    assert len(over) == 1, (
        "注入只发生在一段, 却有 %d 段超预算 -- 计时粒度不对: %s"
        % (len(over), over))
    assert over[0][0] == "b", "超预算的段归因错了: %s" % (over[0],)


def test_the_timer_is_silent_when_everything_fits():
    """反向: 都在预算内时不得报任何一段.

    没有这条, 一个"永远报全部超预算"的实现也能让上一条通过(它会报 3 段,
    而上一条只查了 1 段? 不 -- 上一条查的是恰好 1 段, 所以那种实现过不了).
    这条真正防的是另一种: 一个永远报空的实现 -- 它让上一条红, 但如果有人
    为了让上一条过而把断言改成 ">= 1", 这条就是那时唯一的保护.
    """
    budgets = {"a": 50.0, "b": 50.0}
    timer = StageTimer()
    timer.measure("a", lambda: None)
    timer.measure("b", lambda: None)
    assert timer.over_budget(budgets) == []


def test_the_timer_uses_a_monotonic_clock():
    """CLAUDE.md 3.4 / CLK-C1: 一切时长判定用单调钟.

    墙钟在 NTP 阶跃时会往回跳, 于是某一段的"耗时"变成负数或几十秒 --
    而那两种都会被当成一次真实的超预算, 触发一次没有发生过的告警.
    """
    import ast
    import inspect

    # *** 只看代码, 不看注释 -- 第一版扫全文, 当场被自己的说明文字命中
    # (下面那句解释墙钟危害的注释里就写着被禁的函数名). 与 CHK-1-50 的
    # "no if(ROS_DISTRO...)" 是同一种判据自伤: 描述规则的文字撞进扫描面.
    # AST 只看真调用, 注释里写什么都不影响.
    tree = ast.parse(inspect.getsource(StageTimer))
    calls = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            f = node.func
            if isinstance(f, ast.Attribute):
                base = f.value.id if isinstance(f.value, ast.Name) else ""
                calls.add("%s.%s" % (base, f.attr))
    assert "time.monotonic" in calls, "StageTimer 没有用单调钟: %s" % sorted(calls)
    for banned in ("time.time", "datetime.now", "datetime.utcnow"):
        assert banned not in calls, "StageTimer 用了墙钟 %s" % banned


@pytest.mark.xfail(strict=True, reason=(
    "要拿真实的 P1 一拍去量逐段耗时, 需要 20 Hz 控制环真在跑. "
    "ctrl_loop 今天没有接线(CHK-1-04 逐行核实 / NEXT SW-21), 没有一拍可量. "
    "NO 不写桩去扮演一个不存在的控制环"))
def test_a_real_tick_stays_within_every_stage_budget():
    """MOT-PM-35 的完整形态. 控制环接起来后把标记摘掉并在这里写真断言."""
    import ast

    from xbrain.p1_motion import ctrl_loop

    tree = ast.parse(pathlib.Path(ctrl_loop.__file__).read_text(encoding="utf-8"))
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
    assert any("freshness" in n for n in names), "ctrl_loop 仍未接线"
