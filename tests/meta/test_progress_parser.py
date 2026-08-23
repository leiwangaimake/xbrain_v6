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
