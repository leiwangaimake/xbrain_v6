"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_progress_parser.py
Brief: 守 scripts/progress.py 的解析面 -- 它必须看得见 TODO 的每一行

Description:
progress.py 是本项目唯一的自动进度门禁, 而它曾经[结构上看不见任何已完成的
条目]: 它的行正则要求 `任务号` 之后紧跟 |, 而一旦某行被标上完成/部分/暂缓记号,
标记就插在两者之间. 实测 448 行里 252 行因此不可见 -- 也就是说它只看得见[没做
完的那些], 于是无论做了多少都会报接近零(那正是 "DONE 2/250" 的由来).

这种失效是无声的: 脚本不报错, 不警告, 只是少数了一半以上的行. CLAUDE.md 3.2
形态⑥"扫描面不声明" -- 一个数是在什么范围上跑出来的, 必须自己说得清.

本文件把解析面钉住, 三个方向各配变异体:
  1. 解析到的条数 == 表里真实的任务行数(带标记的也算);
  2. 带完成/部分/暂缓记号的行[一定]在解析结果里, 且带证据注的也在;
  3. 状态闭集里, UNMAPPED 与 UNEVALUABLE 都不得被算进 DONE.

Boundaries: 不判断任何一项是否真的完成 -- 那由证据映射与它自己的门禁
(test_todo_evidence.py)负责. 这里只保证[该被看见的行都被看见].
"""
from __future__ import annotations

import importlib.util
import pathlib
import re

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
TODO = ROOT / "docs" / "XBRAIN_V6_TODO.md"


def _progress():
    """Import scripts/progress.py by path -- it is a script, not a package."""
    spec = importlib.util.spec_from_file_location(
        "progress_mod", ROOT / "scripts" / "progress.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _todo_task_rows():
    """Every table row that opens with a backticked task id, marks and all.

    Written independently of progress.py's own regex on purpose: a test that
    reused the implementation's pattern would agree with it by construction,
    including when the pattern is wrong -- which is the failure this file
    exists to catch.
    """
    pat = re.compile(r"^\|\s*`([A-Z]{3}-[A-Za-z0-9-]+)`")
    return [m.group(1) for m in
            (pat.match(l) for l in TODO.read_text(encoding="utf-8").splitlines())
            if m]


def test_parser_sees_every_task_row():
    """*** The regression that made the whole gate useless.

    MUTATION: put the pipe back right after the backtick in progress.py's ROW
    pattern and this drops by the number of marked rows -- which was 252 of 448
    when it was found.
    """
    parsed = {it["id"] for it in _progress().parse()}
    expected = set(_todo_task_rows())
    missed = sorted(expected - parsed)
    assert not missed, (
        "progress.py cannot see %d task rows, e.g. %s -- a checker that skips "
        "what it cannot match under-reports silently"
        % (len(missed), missed[:5]))


def test_marked_rows_are_not_invisible():
    """*** The specific shape of the bug: a row is dropped BECAUSE it is done.

    A gate whose blind spot is exactly the finished work reports near-zero
    forever, and the number looks plausible the whole time.
    """
    marked = []
    # 记号用码位写, 不写字面量: charset_lint 扫的是源文件里的字符, 而这三个
    # 记号正是本用例要匹配的对象 -- 写字面量会让判据句自己撞上扫描面
    # (CLAUDE.md 3.2 形态3 判据自伤).
    marks = "\u2705\u25d0\u23f8"
    pat = re.compile(r"^\|\s*`([A-Z]{3}-[A-Za-z0-9-]+)`\s*([%s])" % marks)
    for line in TODO.read_text(encoding="utf-8").splitlines():
        m = pat.match(line)
        if m:
            marked.append(m.group(1))
    assert marked, "no marked rows at all -- fixture assumption broken"
    parsed = {it["id"] for it in _progress().parse()}
    invisible = sorted(set(marked) - parsed)
    assert not invisible, "marked rows invisible to the parser: %s" % invisible[:5]


def test_evidence_annotated_rows_are_still_parsed():
    """The writeback appends `(证据 ...)` after the mark. That is more text
    between the id and the pipe, so it is the same failure mode one step
    further along -- pinned separately because the two got added months
    apart."""
    parsed = {it["id"] for it in _progress().parse()}
    annotated = [m.group(1) for m in
                 (re.match(r"^\|\s*`([A-Z]{3}-[A-Za-z0-9-]+)`[^|]*证据", l)
                  for l in TODO.read_text(encoding="utf-8").splitlines()) if m]
    if not annotated:
        pytest.skip("no evidence-annotated rows yet")
    assert not sorted(set(annotated) - parsed)


def test_unmapped_and_unevaluable_are_never_done():
    """*** The rule that keeps the number honest.

    Both states mean "the machine could not confirm this", and counting either
    as finished would let the script report 100% on an empty tree
    (CLAUDE.md 3.2 form 1).

    MUTATION: make evaluate() return DONE for an unmapped item and this fails.
    """
    mod = _progress()
    item = {"id": "ZZZ-NO-1", "title": "t", "dir": "", "criterion": "no path here"}
    st, _why = mod.evaluate(item, run_tests=False, evidence={"OTHER-1": ["x"]},
                            cache={})
    assert st != "DONE", "an unmapped item must never be DONE"
    st2, _ = mod.evaluate(item, run_tests=False, evidence=None, cache={})
    assert st2 != "DONE", "an item with no runnable criterion must never be DONE"


#: 允许出现在 NON_FAILURE 里的状态, 以及每条必须能在文档里查到的出处关键词.
#: 与 progress.py 的表分开写: 一份被测者自己维护的白名单证明不了任何事.
_EXPECTED_NON_FAILURE = {
    # CFG-BT-02 曾在这里(WAITING_DECISION, gossip vs RT-C2). 2026-08-23 实测
    # 裁决后移除 -- 豁免不该比它的理由活得久, 这正是本文件另一条用例守的方向.
    "CHK-0-54": ("RELEASE_ONLY", "release"),
}


def test_non_failure_set_is_exactly_what_was_reviewed():
    """*** 这是整套状态分类里最容易被滥用的一格, 所以钉得最死.

    每多一个"不算失败"的状态, 就多一个藏真失败的地方 -- 本仓 charset_lint 的
    头注自己写着"一条不可达的判据会被一路放宽到通过". 一条为了让数字好看而
    加进 NON_FAILURE 的行, 测试套是分辨不出来的; 能分辨的只有复核的人.

    所以本用例要求: 集合必须与这里[逐条列出的, 经过复核的]那份完全相等.
    新增一项 -> 红, 直到有人把它加到这里并说明理由; 删掉一项 -> 也红, 免得
    豁免比它的理由活得久.

    MUTATION: 往 progress.py 的 NON_FAILURE 里加任意一行 -> 立刻红.
    """
    actual = {k: v[0] for k, v in _progress().NON_FAILURE.items()}
    expected = {k: v[0] for k, v in _EXPECTED_NON_FAILURE.items()}
    assert actual == expected, (
        "NON_FAILURE 与已复核的集合不一致: 多了 %s, 少了 %s"
        % (sorted(set(actual) - set(expected)), sorted(set(expected) - set(actual))))


def test_every_non_failure_row_cites_where_the_reason_lives():
    """*** 豁免必须[指得出出处], 否则半年后没人知道它为什么在这里.

    一条写着"暂时先放过"的豁免与一条写着"见 NEXT S7.1A 的三条路"的豁免, 对
    读者的价值差一个量级.

    MUTATION: 把某行的理由改成一句不含出处的话 -> 红.
    """
    for task_id, (status, why) in _progress().NON_FAILURE.items():
        kw = _EXPECTED_NON_FAILURE[task_id][1]
        assert kw.lower() in why.lower(), (
            "%s 的理由 %r 没有指向出处(期望提到 %r)" % (task_id, why, kw))


def test_declared_non_failure_never_counts_as_done():
    """*** 分类的意义是把"流程如此"与"做完了"分开, 不是给它们发通行证.

    MUTATION: 让 evaluate() 对 NON_FAILURE 里的项直接返回 DONE -> 红.
    """
    mod = _progress()
    for task_id, (status, _why) in mod.NON_FAILURE.items():
        assert status != "DONE", "%s 被声明成 DONE 了" % task_id
        assert status in ("WAITING_DECISION", "RELEASE_ONLY"), status


def test_evidence_map_is_actually_consulted():
    """*** Guards the wiring itself: an evidence row must change the verdict.

    Without this, load_evidence() could return {} forever -- the report would
    say "0 hits" and everything would fall back to the prose path, which is the
    state the whole change was meant to leave behind.
    """
    mod = _progress()
    ev = mod.load_evidence()
    assert ev, "evidence map loaded empty; docs/TODO_EVIDENCE.tsv unreadable?"
    rows = mod.parse()
    hits = [it for it in rows if mod.norm_id(it["id"]) in ev]
    assert hits, "no TODO row matched any evidence row -- the id normalisation "\
                 "or the parser is broken"


def test_no_test_file_greps_documents_with_ascii_stars():
    """*** 这条守的是本轮批量清标点时踩的坑.

    多个测试要剥掉文档表格里的装饰符(黑星), 做法是一个字符类正则. 批量清
    标点的 sed 把源码里的黑星字面量一并换成了 ASCII *, 于是那些正则不再
    剥离文档中的黑星 -- 解析立刻少认几行, 而表现是"代码表里多出几行",
    会把人引去改代码而不是改正则.

    正确写法是码位转义(★): 清标点脚本碰不到它, 运行期是同一个字符.

    MUTATION: 把某个测试里的 ★ 换回字面量 -> 这里红.
    """
    import re as _re

    root = pathlib.Path(__file__).resolve().parents[2]
    bad = []
    for path in (root / "tests").rglob("test_*.py"):
        text = path.read_text(encoding="utf-8", errors="replace")
        # 找形如 re.sub(r"[...]", "", ...) 里同时含 * 而不含 ★ 的字符类,
        # 且该文件确实在读 docs/ -- 只有读文档的才需要剥装饰符.
        if "docs" not in text:
            continue
        # *** 本文件必须在扫描面之外.
        # 第一版把示例正则写进了 docstring, 于是本用例[命中了自己] --
        # 恒红, 而恒红的断言最终会被改成恒绿(CLAUDE.md 3.2 形态2/3).
        # 排除自己, 并由下面那条断言证明排除没有写宽.
        if path.resolve() == pathlib.Path(__file__).resolve():
            continue
        for m in _re.finditer(r're\.sub\(r"\[([^\]]*)\]"', text):
            cls = m.group(1)
            if "*" in cls and "\\u2605" not in cls and "★" not in cls:
                bad.append("%s: %s" % (path.name, m.group(0)))
    assert not bad, (
        "这些正则用 ASCII 星号剥文档装饰符, 但文档里是黑星 -- "
        "多半是被批量清标点改坏的: %s" % bad[:4])
    # 扫描面不是空的 -- 否则上面那条恒过.
    scanned = [p for p in (root / "tests").rglob("test_*.py")
               if "docs" in p.read_text(encoding="utf-8", errors="replace")]
    assert len(scanned) >= 3, "只扫到 %d 个读文档的测试" % len(scanned)
