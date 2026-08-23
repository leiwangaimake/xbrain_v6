"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_factor_participation.py
Brief: BIZ-P2-19 第 0 步 -- speed_factor 参与集必须现场读 11 S5.1A, 不许硬编码

Description:
11 S5.1A 的规范表有一列叫"计入 speed_factor". 哪些健康度项参与速度系数的
计算, 那一列说了算.

判据的第 0 步是一条静态断言: xbrain/p2_core/health/ 的源码里[硬编码的项名
列表与硬编码条数]命中数必须为 0. 理由很直接 -- 一份抄进代码的参与集会在
S5.1A 增删一行时静默过期:

  * 表里新增一项且标"计入", 而代码里的列表没跟上 -> 那一项的故障不再压低
    speed_factor, 机器人带着它全速跑;
  * 表里把某项改成"不计入", 而代码还在算 -> 无故降速, 现场查不出原因.

两种都不报错.

*** 判据逐字点了扫描面: "源码, NO 不含注释与本判据文本".
这条限定不是客套. 本文件与判据文本里都会写出那些项名(要说清在讲什么),
如果扫描面把注释算进去, 那么讨论这条规则的文字本身就会命中 -- 恒红,
然后被人放宽成"包含即可", 于是恒绿. CLAUDE.md 3.2 形态3 的标准剧本.
所以下面的扫描用 AST 取字符串常量, 注释根本不在 AST 里.

Boundaries: 不判断某一项该不该计入(那是 11 的事), 只保证[参与集是现场读
出来的, 不是抄的].
"""
from __future__ import annotations

import ast
import pathlib
import re

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[3]
DOC = ROOT / "docs" / "11-接口契约.md"
HEALTH_DIR = ROOT / "xbrain" / "p2_core" / "health"

_ANCHOR = "### 5.1A"


def spec_rows():
    """从 11 S5.1A 解析 [(项名, 计入 speed_factor, 驱动 allow_motion)].

    NO 解析不到就抛: 返回空会让"参与集与表一致"变成两个空集合相等.
    """
    text = DOC.read_text(encoding="utf-8")
    hits = text.count(_ANCHOR)
    if hits != 1:
        raise AssertionError("11 S5.1A 锚点命中 %d 次, 应恰为 1" % hits)
    body = text[text.index(_ANCHOR) + len(_ANCHOR):]
    nxt = re.search(r"^#{1,4} ", body, re.M)
    if nxt:
        body = body[:nxt.start()]
    rows = []
    for line in body.split("\n"):
        if not line.startswith("|"):
            continue
        cells = line.split("|")
        if len(cells) < 6:
            continue
        m = re.search(r"`([a-z_0-9]+)`", cells[1])
        if not m:
            continue                       # 表头 / 分隔行
        # "计入"与"驱动"两列用的是勾与"否"两种记号.
        def _yes(cell):
            flat = re.sub(r"[*\u2605\s]", "", cell)
            return "✅" in flat or flat in ("是", "Y")
        rows.append((m.group(1), _yes(cells[4]), _yes(cells[5])))
    if not rows:
        raise AssertionError("11 S5.1A 解析到 0 行 -- 表结构变了")
    return rows


def _string_constants(path):
    """源码里的字符串常量(AST), 注释不在其中."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            out.append((node.value, getattr(node, "lineno", 0)))
    return out


def test_the_spec_table_parses():
    """守前提. 解析到 0 行时下面每条都会空过."""
    rows = spec_rows()
    assert len(rows) >= 15, "只解析到 %d 行" % len(rows)
    counted = [n for n, c, _a in rows if c]
    assert counted, "一项都没被标成计入 speed_factor -- 列位可能错了"


def test_no_hardcoded_item_name_list_in_health_sources():
    """*** 判据第 0 步: 硬编码的项名列表命中数 == 0.

    扫描面是 AST 里的字符串常量 -- 注释不在其中(判据逐字要求排除注释,
    否则讨论这条规则的文字会命中自己).

    做法: 找同一个容器字面量里[同时出现三个及以上] S5.1A 项名的地方.
    单独出现一个项名是正常的(比如报错信息里点名某一项), 而三个凑在一起
    几乎只可能是一份抄下来的参与集.

    MUTATION(判据点名): 在 health/ 里写死 7 个项名的列表 -> 这里红.
    """
    names = {n for n, _c, _a in spec_rows()}
    bad = []
    for path in sorted(HEALTH_DIR.rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, (ast.List, ast.Tuple, ast.Set)):
                continue
            elts = [e.value for e in node.elts
                    if isinstance(e, ast.Constant) and isinstance(e.value, str)]
            hit = [e for e in elts if e in names]
            if len(hit) >= 3:
                bad.append("%s:%d %s" % (path.relative_to(ROOT),
                                         node.lineno, sorted(hit)[:6]))
    assert not bad, (
        "health/ 源码里有硬编码的项名列表 -- S5.1A 增删一行时它会静默过期:\n  "
        + "\n  ".join(bad))


def test_no_hardcoded_participation_count():
    """*** 判据点名的另一半: 硬编码条数(7/12/5)命中数 == 0.

    写死条数比写死项名更隐蔽: 它不含任何项名, 读起来像个无害的常量,
    而它同样会在表增删一行时过期 -- 且过期的表现是断言"数量对不上",
    把人引去改那个常量而不是去看表.

    只查[与参与集相关的上下文]里的裸数字: 一个 range(7) 或 timeout=5
    显然无关. 判据给的三个数是 7/12/5.
    """
    suspicious = []
    for path in sorted(HEALTH_DIR.rglob("*.py")):
        src = path.read_text(encoding="utf-8")
        tree = ast.parse(src)
        lines = src.split("\n")
        for node in ast.walk(tree):
            if not isinstance(node, ast.Compare):
                continue
            # 形如 len(xxx) == 7
            if not (isinstance(node.left, ast.Call)
                    and isinstance(node.left.func, ast.Name)
                    and node.left.func.id == "len"):
                continue
            for cmp_node in node.comparators:
                if (isinstance(cmp_node, ast.Constant)
                        and cmp_node.value in (7, 12, 5)):
                    suspicious.append(
                        "%s:%d %s" % (path.relative_to(ROOT), node.lineno,
                                      lines[node.lineno - 1].strip()[:70]))
    assert not suspicious, (
        "health/ 里有对参与集条数的硬编码断言 -- 表增删一行它就过期:\n  "
        + "\n  ".join(suspicious))


def test_the_scan_surface_excludes_comments():
    """*** 守扫描面本身: 注释里出现项名不得命中.

    判据逐字排除注释, 而排除得对不对要证明. 这里造一段[注释里含三个项名]
    的源码, 断言扫描不认它.

    没有这条, 一个扫全文的实现会命中本文件自己的说明文字 -- 恒红,
    然后被放宽成恒绿.
    """
    names = sorted({n for n, _c, _a in spec_rows()})[:3]
    src = "# 参与集含 %s\nX = 1\n" % ", ".join(names)
    tree = ast.parse(src)
    found = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.List, ast.Tuple, ast.Set)):
            found.append(node)
    assert not found, "注释里的项名被当成了容器字面量"
    consts = [n.value for n in ast.walk(tree)
              if isinstance(n, ast.Constant) and isinstance(n.value, str)]
    assert not [c for c in consts if c in names], (
        "注释里的项名进了字符串常量集合 -- 扫描面没有排除注释")


def test_items_that_count_are_not_the_same_as_items_that_block():
    """*** 两列是不同的集合, 混掉会造出一个错误的安全模型.

    S5.1A 里 rtk / heading 是 fatal 但[不计入 speed_factor 也不驱动
    allow_motion]; 而 chassis / clock 两列都是勾. 把"计入"与"驱动"当成
    同一列, rtk 失败就会直接禁止运动 -- 而设计明确保留了遥控 <=0.5 m/s.
    """
    rows = spec_rows()
    counted = {n for n, c, _a in rows if c}
    blocking = {n for n, _c, a in rows if a}
    assert counted and blocking, "两列之一解析为空"
    # 两列不该完全相同 -- 相同就说明列位读错了或表本身塌了.
    assert counted != blocking or len(rows) < 5, (
        "计入 speed_factor 与驱动 allow_motion 两列完全相同 -- 多半读错了列")


def test_fatal_items_that_do_not_block_motion_are_documented():
    """S5.1A 里存在"fatal 但不禁止运动"的项(rtk / heading).

    这看起来矛盾, 其实是设计: 它们 fail 时禁的是[自主运动], 遥控仍以
    <=0.5 m/s 保留 -- 因为完全不能动的机器人无法被开出危险区域.
    这条钉住那个组合确实存在, 免得有人"顺手"把它们改成驱动 allow_motion.
    """
    rows = spec_rows()
    exceptions = [n for n, c, a in rows if not a and not c]
    assert exceptions, (
        "一个 [不计入也不驱动] 的项都没有 -- rtk/heading 那条设计取舍"
        "可能被抹平了")
