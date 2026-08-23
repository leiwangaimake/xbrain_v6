"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_todo_evidence.py
Brief: 守 docs/TODO_EVIDENCE.tsv -- 任务号与验证产物两侧都必须真实存在

Description:
证据映射解决的是一个具体故障: TODO 的完成判据写于开工前, 里面点名的测试路径是
[预期文件名]而不是实现. 实测 182 项点了名, 其中 123 项的文件根本不存在, 于是
scripts/progress.py 把它们全判成 IN_PROGRESS, 报出 DONE 2/250 这种无意义的数字.

*** 但一份人工维护的映射本身也会腐烂 -- 而且腐烂方向是最坏的那种: 指向一个
被删掉或改名的测试, 脚本会照样把它算成"有证据", 于是一条已经不存在的验证被当成
仍然成立(CLAUDE.md 3.2 形态①). 所以这份表必须有门禁, 本文件就是.

守三件事, 每条都配了会让它变红的变异体:
  1. 表里的每个任务号在 TODO 里真实存在(orphan 除外, 见下);
  2. 表里点名的每个验证产物在磁盘上真实存在;
  3. orphan 必须[显式声明]-- 一个新出现的, TODO 里没有的任务号会失败, 而不是
     悄悄混进去.

* 关于 orphan: 现有两条(BIZ-P3-40 / BIZ-P3-42)是测试自报了 TODO 里不存在的
任务号. TODO 自己的规矩是"新增任务必须先 grep 确认空号", 所以这两块工作是在任务
表外做的. 保留并显式标记, 是为了将来回填 TODO; NO 不静默丢弃, 也不放任新增.

Boundaries: 本文件不判断"该任务是否真的完成"-- 那要逐条读判据与变异体, 证据表
的头注对此有明确声明. 这里只保证[表本身指向的东西都存在].
"""
from __future__ import annotations

import pathlib
import re

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / "docs" / "TODO_EVIDENCE.tsv"
TODO = ROOT / "docs" / "XBRAIN_V6_TODO.md"

#: The only task ids allowed to be absent from the TODO, each because the work
#: happened outside the task table. A NEW one fails test 3 below -- that is the
#: point of pinning the set rather than allowing any orphan.
DECLARED_ORPHANS = {"BIZ-P3-40", "BIZ-P3-42"}


def _rows():
    """(task_id, files, source, date) for every non-comment line."""
    out = []
    for line in EVIDENCE.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        parts = line.split("\t")
        assert len(parts) == 4, "malformed row (want 4 tab-separated columns): %r" % line
        out.append((parts[0], parts[1].split(";"), parts[2], parts[3]))
    return out


def _todo_ids():
    """Task ids present in the TODO, normalised to two-digit form.

    Normalised because the TODO writes both `BIZ-P3-0` and `GWY-P5-01`; an
    exact string compare would report a third of the table as missing.
    """
    ids = set()
    text = TODO.read_text(encoding="utf-8")
    for m in re.finditer(r"^\|\s*`([A-Z]{3}-[A-Za-z0-9]+)-([0-9]+)`", text, re.M):
        ids.add("%s-%02d" % (m.group(1), int(m.group(2))))
    return ids


def test_the_evidence_file_is_not_empty():
    """*** Guards this file's own premise (CLAUDE.md 3.2 form 1).

    Every other case below iterates the rows. On an empty file they would all
    pass vacuously, and the gate would report green over no evidence at all.
    """
    rows = _rows()
    assert len(rows) >= 50, "only %d rows; the first batch alone covers 55" % len(rows)


def test_every_evidence_file_exists():
    """*** The rot this table is most exposed to: pointing at a test that was
    renamed or deleted.

    That failure is silent by nature -- progress.py would still count the task
    as evidenced, so a verification that no longer exists keeps being reported
    as holding.

    MUTATION: rename any file named here and this goes red.
    """
    missing = []
    for task_id, files, _src, _date in _rows():
        for f in files:
            if not (ROOT / f).exists():
                missing.append("%s -> %s" % (task_id, f))
    assert not missing, "evidence files that no longer exist:\n  " + "\n  ".join(missing)


def test_every_task_id_exists_in_the_todo_or_is_a_declared_orphan():
    """*** A task id that is not in the TODO means one of two things, and both
    need a human: a typo, or work done outside the task table.

    MUTATION: add a row for a made-up id (BIZ-P3-99) without declaring it and
    this goes red -- which is what stops the table from quietly growing ids
    nobody reviewed.
    """
    todo = _todo_ids()
    stray = [t for t, _f, _s, _d in _rows()
             if t not in todo and t not in DECLARED_ORPHANS]
    assert not stray, (
        "task ids not in the TODO and not declared orphans: %s" % sorted(stray))


def test_declared_orphans_are_all_still_orphans():
    """*** The reverse direction, and the half that makes the orphan list shrink.

    Once an orphan is written back into the TODO it must leave this set --
    otherwise the exemption outlives the reason for it, and the next reader
    cannot tell which orphans are real.

    MUTATION: add BIZ-P3-40 to the TODO without removing it here and this goes
    red.
    """
    todo = _todo_ids()
    resolved = sorted(DECLARED_ORPHANS & todo)
    assert not resolved, (
        "these are in the TODO now and must be removed from DECLARED_ORPHANS "
        "(and their rows re-sourced from 'orphan'): %s" % resolved)


def test_orphan_rows_are_marked_as_such():
    """The source column and the declared set must agree, in both directions --
    a row marked orphan that is not declared, or a declared orphan whose row
    claims another source, means the two halves have drifted apart."""
    marked = {t for t, _f, src, _d in _rows() if src == "orphan"}
    assert marked == DECLARED_ORPHANS, (
        "rows marked orphan %s disagree with DECLARED_ORPHANS %s"
        % (sorted(marked), sorted(DECLARED_ORPHANS)))


def test_range_style_brief_ids_are_all_expanded():
    """*** p1_motion 的 Brief 用[区间]写任务号(MOT-PM-1..15), 而前几批的提取
    只认斜杠分隔.

    这个差异是静默的: 把 "1..15" 当成单个 1 读, 结果是少算 14 项而不是报错 --
    提取器不会抱怨, 证据表只是短了一截, 而短了多少没人看得出来.

    本用例把两侧钉在一起: Brief 里出现的每个区间, 其[两端与中间]都必须在证据
    表里有行. MUTATION: 把提取器改回只认 '/', 这里立刻红.
    """
    import re as _re
    briefs = []
    for f in (ROOT / "tests" / "p1_motion").rglob("*.py"):
        m = _re.search(r"^Brief:(.*)$", f.read_text(encoding="utf-8",
                                                    errors="replace"), _re.M)
        if m:
            briefs.append(m.group(1))
    wanted = set()
    for line in briefs:
        for mm in _re.finditer(r"(MOT-PM|MOT-RD)-([0-9]+)\.\.([0-9]+)", line):
            lo, hi = int(mm.group(2)), int(mm.group(3))
            for n in (lo, (lo + hi) // 2, hi):        # 两端 + 中间各取一
                wanted.add("%s-%02d" % (mm.group(1), n))
    if not wanted:
        pytest.skip("no range-style Brief ids present")
    have = {t for t, _f, _s, _d in _rows()}
    assert not sorted(wanted - have), (
        "range-style ids missing from the evidence map -- the extractor is "
        "probably reading '1..15' as a bare 1: %s" % sorted(wanted - have)[:6])


def test_source_column_is_a_closed_set():
    """brief / manual / orphan and nothing else -- an unrecognised source is a
    row nobody can interpret."""
    bad = {src for _t, _f, src, _d in _rows()
           if src not in ("brief", "manual", "orphan")}
    assert not bad, "unknown source values: %s" % sorted(bad)
