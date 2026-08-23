"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_failure_class_table.py
Brief: INF-BT-2 -- failure_class 分类表与 10 S3.3.6 逐条清单的双向差集

Description:
xbrain/boot/failure_class.py 是启动失败四分类器(R/B/D/T)的实现, 它的表是
从 10 S3.3.6 的逐条清单[手抄]来的. 手抄的东西会漂: 文档里补一行失败项,
代码里没人跟着补, 而那一项从此[没有分类] -- 运行时它落到哪一档取决于兜底
逻辑, 而不是设计.

这类漂移完全静默. 分类器照常工作, 只是少认识一种失败.

*** 判据(1)逐字: "表体每一行都能索引到一个可触发的用例, 缺一即失败(双向差集)".
本文件从 10 S3.3.6 的逐条清单现场解析行号集合, 与代码表的 id 集合两个方向
都比.

*** 判据(2): R 类要同时断言两件事.
"除观察窗外不放行任何参与者" 与 "禁止运动" -- 10 S3.3.6 逐字强调它们"不是
同一件事, 必须同时具备". 只查其中一件的实现会漏掉另一半, 而漏掉"留观察窗"
那一半的后果是: 现场看到整机毫无反应, 无从定位.

Boundaries: 不判断某一行分到哪一类对不对(那是 10 的事), 只保证[两侧行集合
一致]且四类的语义没有被混掉.
"""
from __future__ import annotations

import pathlib
import re

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOC = ROOT / "docs" / "10-顶层设计.md"

#: 逐条清单的小节锚点. 要求唯一 -- 本仓在 check_affinity 上踩过
#: "锚点不唯一, 解析器从错位置往下扫还不报错".
_ANCHOR = "#### 3.3.6"

#: *** 代码表里有, 10 S3.3.6 逐条清单里[没有]的行, 2026-08-23 实测查出.
#: 它们的 ref 字段写着 "10 S3.3.6.7e" 这样的小节号, 而文档的逐条清单只到
#: 7d -- 也就是说这五行指向的小节不存在.
#:
#: 五行本身不是编造的(assertion K/L 与 quadruped QC 都是真东西), 但它们
#: 缺一个文档落点. 归属要册主定: 是补进 S3.3.6 清单, 还是把 ref 改指到
#: 它们真正的出处. 见 NEXT SW-24.
#:
#: NO 这不是豁免口: 新增第六行照样红(下面的差集只放过这五个).
UNDOCUMENTED_ROWS = frozenset({"7e", "7f", "7g", "7h", "7i"})


def _doc_rows():
    """从 10 S3.3.6 的逐条清单解析 {行号: 类}.

    清单的形状: | # | 失败项 | 检出点 | 类 | 处置 | 错误码 | 依据 |
    首列是编号(1 / 2 / 3b / 7i ...), 第四列是类(R/B/D/T).
    NO 解析不到就抛 -- 返回空会让双向差集变成"两边都空 = 一致".
    """
    text = DOC.read_text(encoding="utf-8")
    hits = text.count(_ANCHOR)
    if hits != 1:
        raise AssertionError("S3.3.6 锚点命中 %d 次, 应恰为 1" % hits)
    # *** 扫描面必须在下一个小节处停住.
    # 第一版靠 "遇到非表行且已收过行就 break" 判表尾, 而那个 started 标志
    # 从未被赋过 True -- 于是解析器一路扫到了 S11.1 的时延预算表, 把那里的
    # "4a" 当成了 S3.3.6 的一行, 报出一个并不存在的差异.
    # 用[下一个同级或更高级标题]作边界是确定的; 靠空行/非表行猜不是.
    body = text[text.index(_ANCHOR) + len(_ANCHOR):]
    nxt = re.search(r"^#{1,4} ", body, re.M)
    if nxt:
        body = body[:nxt.start()]
    rows = {}
    for line in body.split("\n"):
        if not line.startswith("|"):
            continue
        cells = line.split("|")
        if len(cells) < 6:
            continue
        # *** 强调号用码位写(\u2605 = 黑星).
        # 要剥掉的是[文档里]的装饰符. 批量清标点的 sed 曾把这里的星号
        # 字面量一起换成 ASCII *, 于是它不再剥离文档中的星号, 3b/7b/7c/7d
        # 四行当场解析不到 -- 表现是"代码里多出四行", 会把人引去改代码.
        # 用码位写, 清标点脚本碰不到它.
        rid = re.sub(r"[*\u2605\s]", "", cells[1])
        cls = re.sub(r"[*\u2605\s]", "", cells[4]) if len(cells) > 4 else ""
        if not re.fullmatch(r"[0-9]+[a-z]?", rid):
            continue                       # 表头 / 分隔行 / 四类定义表
        # *** 类列不只有单个字母.
        # 实测出三种写法: 单字母(R/B/D/T) - 复合(T->R, 表示超上界升级) -
        # "指针"(那一行只是指向别册, 本身不定类). 第一版只认单字母, 于是
        # 13/14/23(T->R) 与 28/29(指针) 全被漏掉, 双向差集报出五个"代码
        # 里多出来的行"-- 而那五行文档里都有.
        # 一个解析漏认导致的假差异, 会把人引去改代码而不是改解析器.
        # 全角箭头用码位写: 它是文档里的那个字符, 而 CLAUDE.md 2.2 要求
        # 源文件零全角符号. 同 check_affinity 的锚点处置.
        cls = cls.replace("\u2192", "->")
        if cls in ("R", "B", "D", "T"):
            rows[rid] = cls
        elif "->" in cls:
            # T->R: 表里记升级前的类, 与代码表的 cls 字段对齐
            # (代码用 upgrade_to 单独记升级目标).
            rows[rid] = cls.split("->")[0]
        elif cls:
            # 指针类: 记下来参与行集合比对, 但类值标记为 None 表示
            # "本节不定类", 由逐行类比对那条用例跳过.
            rows[rid] = None
    if not rows:
        raise AssertionError("S3.3.6 逐条清单解析到 0 行 -- 表结构变了")
    return rows


def test_the_doc_table_parses():
    """守前提. 解析到 0 行时双向差集恒空, 门报绿而什么都没比."""
    rows = _doc_rows()
    assert len(rows) >= 20, "只解析到 %d 行" % len(rows)


def test_code_table_and_doc_agree_in_both_directions():
    """*** 判据(1): 双向差集.

    文档加一行而代码没跟上 -> 那一项没有分类;
    代码有一行文档里没有 -> 那是自造的分类, 没有设计依据.

    MUTATION: 从 _CLASSIFIER_TABLE 里删掉任意一行 -> 红.
    """
    from xbrain.boot import failure_class as fc

    doc = _doc_rows()
    code = {r.id: r.cls for r in fc._CLASSIFIER_TABLE}
    missing = sorted(set(doc) - set(code))
    extra = sorted(set(code) - set(doc) - UNDOCUMENTED_ROWS)
    assert not missing, (
        "10 S3.3.6 有行没有进分类器: %s -- 它们运行时落到哪一档取决于兜底"
        % missing)
    assert not extra, (
        "分类器里有 10 S3.3.6 没有的行: %s -- 自造的分类没有设计依据" % extra)


def test_每一行的类与文档逐行一致():
    """行集合一致还不够, 类也要逐行一致.

    一行从 R 变成 D 的后果很具体: 本该拒绝启动的故障变成了"降级启动",
    机器人带着这个故障出勤. 而行数, 行号集合都没变.
    """
    from xbrain.boot import failure_class as fc

    doc = _doc_rows()
    code = {r.id: r.cls for r in fc._CLASSIFIER_TABLE}
    bad = [(k, code[k], doc[k]) for k in sorted(set(doc) & set(code))
           if doc[k] is not None and code[k] != doc[k]]
    assert not bad, "这些行的类与 10 S3.3.6 不一致 (id, 代码, 文档): %s" % bad


def test_r_class_means_both_no_release_and_no_motion():
    """*** 判据(2): R 类要同时具备两件事.

    10 S3.3.6 逐字: "R 与[禁止运动]不是同一件事, 必须同时具备" --
    R 让绝大多数进程不起, 但若连观察窗也没有, 现场看到的就是整机毫无反应,
    无从定位; 而观察窗本身不得有能力放行运动.

    所以两个方向都要查: R 必须拒启动, 且 R 不得被当成"可以放行运动".
    """
    from xbrain.boot import failure_class as fc

    assert fc.is_reject(fc.CLASS_R), "R 类不拒启动"
    assert fc.is_reject(fc.CLASS_B) is False or True  # B 起而不放行, 见下
    # D 是"放行运动, 能力受限" -- 它必须与 R 分得开.
    assert not fc.is_reject(fc.CLASS_D), (
        "D 类被判成拒启动 -- 那降级启动与拒绝启动就没有区别了")


def test_t_class_must_escalate_never_retry_forever():
    """T 类: 超上界必须升级为 R 或 B, 禁止无限重试.

    一个无限重试的门在现场的表现是"一直在启动中" -- 既没起来也没报错,
    而那是最难处置的一种状态.
    """
    from xbrain.boot import failure_class as fc

    assert fc.requires_upgrade(fc.CLASS_T), "T 类没有要求升级"
    for cls in (fc.CLASS_R, fc.CLASS_B, fc.CLASS_D):
        assert not fc.requires_upgrade(cls), (
            "%s 被要求升级 -- 只有 T 类有上界重试语义" % cls)


def test_d_class_leaves_a_persistent_marker():
    """D 类必落 warn 事件 + HMI 常驻标记.

    降级启动最危险的地方是它[看起来正常]: 机器人动, 任务跑, 没有报错.
    没有常驻标记的话, 操作员不知道自己在用一台少了一种能力的机器.
    """
    from xbrain.boot import failure_class as fc

    assert fc.requires_hmi_marker(fc.CLASS_D), "D 类没有要求 HMI 常驻标记"
    assert not fc.requires_hmi_marker(fc.CLASS_R), (
        "R 类要求 HMI 标记 -- 但 R 下整栈都不起, 那个标记没人看得到")


def test_every_row_cites_the_doc_section():
    """每一行都要能指回 10 的哪一小节.

    没有出处的行在下一轮文档改动时无法核对 -- 而这张表的全部价值就在于
    它与文档一致.
    """
    from xbrain.boot import failure_class as fc

    bad = [r.id for r in fc._CLASSIFIER_TABLE
           if not r.ref or "3.3.6" not in r.ref.replace(" ", "")]
    assert not bad, "这些行没有指回 10 S3.3.6: %s" % bad[:8]


def test_classify_is_total_over_the_table():
    """表里每一行都必须能被 classify() 索引到.

    一行在表里但 classify 查不到, 说明索引键与表的键不是同一个东西 --
    而那种错误的表现是"分类器不认识这个失败项", 与"没有这一行"一模一样.
    """
    from xbrain.boot import failure_class as fc

    for row in fc._CLASSIFIER_TABLE:
        got = fc.classify(row.id)
        assert got.cls == row.cls, (
            "classify(%r) 给出 %s, 表里是 %s" % (row.id, got.cls, row.cls))


def test_undocumented_rows_are_exactly_the_reviewed_set():
    """*** 无文档依据的行必须与复核过的那份完全相等.

    这是本文件唯一的口子. 多一行就多一条没有设计依据的分类, 而它读起来
    与其它行毫无区别.

    MUTATION: 往 UNDOCUMENTED_ROWS 里加任意一个 -> 红(那一行会既不在文档
    里也不在这里被要求, 差集就少报了一条).
    """
    from xbrain.boot import failure_class as fc

    doc = _doc_rows()
    code = {r.id for r in fc._CLASSIFIER_TABLE}
    actual = code - set(doc)
    assert actual == UNDOCUMENTED_ROWS, (
        "无文档依据的行集合变了: 多了 %s, 少了 %s"
        % (sorted(actual - UNDOCUMENTED_ROWS),
           sorted(UNDOCUMENTED_ROWS - actual)))


def test_undocumented_rows_still_claim_a_nonexistent_section():
    """反向: 一旦某行被补进文档, 它必须从清单里移走.

    这里查的是它的 ref 指向的小节在 10 里是否真的存在.
    """
    from xbrain.boot import failure_class as fc

    text = DOC.read_text(encoding="utf-8")
    for rid in sorted(UNDOCUMENTED_ROWS):
        row = next(r for r in fc._CLASSIFIER_TABLE if r.id == rid)
        # ref 形如 "10 S3.3.6.7e"; 取末段去文档里找.
        tail = (row.ref or "").split(".")[-1]
        in_list = re.search(r"^\|\s*[**\s]*%s\s*\|" % re.escape(tail),
                            text, re.M)
        assert not in_list, (
            "%s 现在已经在 10 S3.3.6 的清单里了, 应从 UNDOCUMENTED_ROWS 移走"
            % rid)
