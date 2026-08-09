"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_debt_code_cover.py
Brief: doccheck tests -- debt code cover

Description:
INF-DB-1 -- 21册 debt-trace parser + coverage checker + variants.
"""


import json
import os
import subprocess
import sys
from pathlib import Path

import pytest


pytestmark = pytest.mark.no_device


REPO = Path(__file__).parent.parent.parent
CHECKER = REPO / "scripts" / "doccheck" / "debt_code_cover.py"

# Add repo to path so `from scripts...` imports work.
sys.path.insert(0, str(REPO))
from scripts.doccheck.debt_code_cover import (  # noqa: E402
    Debt,
    check_coverage,
    check_fail_loud,
    parse_21,
    _fail_silent_in_positive_context,
    Report,
)


# --- Parser ---------------------------------------------------------

def test_parser_finds_multiple_rows():
    ds = parse_21(str(REPO / "docs" / "21-实测与第三方欠账.md"))
    # No exact count assertion (CLAUDE.md 3.7): just a floor.
    assert len(ds) >= 30, "expected at least 30 rows in 21, got %d" % len(ds)


def test_parser_extracts_book_and_id():
    ds = parse_21(str(REPO / "docs" / "21-实测与第三方欠账.md"))
    # Find a row we know exists (V-02 is in 11册).
    v02 = [d for d in ds if d.id == "V-02"]
    assert v02, "V-02 not parsed"
    assert v02[0].book == "11"
    assert "MotionParam" in v02[0].criteria


def test_parser_skips_header_tables():
    """Rows that are inside the docs' pre-正表 header tables (like the
    §0.2 table) must not be counted as debt rows -- their first cell
    is `11` §15.3 which is book + section, not book + ID."""
    ds = parse_21(str(REPO / "docs" / "21-实测与第三方欠账.md"))
    for d in ds:
        assert "§" not in d.book, \
            "header-table row leaked: %s" % d


# --- Fail-loud detector ---------------------------------------------

def test_fail_silent_in_positive_context_negation_aware():
    """The parser must NOT false-fire on Chinese negations like
    "!! 不得为"先跑起来"而填手感值"."""
    # Negated: should NOT fire.
    negated = '🚫 **不得**为「先跑起来」而填手感值'
    assert _fail_silent_in_positive_context(negated, "先跑起来") is False
    # Positive use: SHOULD fire.
    positive = '暂时先填 0.5 让它先跑起来'
    assert _fail_silent_in_positive_context(positive, "先跑起来") is True


def test_check_fail_loud_flags_positive_fail_silent():
    """Feed an artificial debt row whose default_behavior is genuinely
    fail-silent (no negation)."""
    r = Report()
    r.debts = [Debt(book="本", id="V-99", criteria="", who_tests="",
                    default_behavior="填一个默认值,先跑起来看看",
                    where_to_fix="", line_no=999)]
    check_fail_loud(r.debts, r)
    assert r.fail_silent_hits, "positive fail-silent must be flagged"


def test_check_fail_loud_missing_when_no_loud_word():
    """A default that has no fail-loud word (empty, or just discussion
    without a decision) must be flagged."""
    r = Report()
    r.debts = [Debt(book="本", id="V-99", criteria="", who_tests="",
                    default_behavior="我们再讨论一下",   # no rejection language
                    where_to_fix="", line_no=999)]
    check_fail_loud(r.debts, r)
    assert r.missing_fail_loud


def test_current_21_has_no_fail_silent_positives():
    """POSITIVE: the ACTUAL 21册 (as of this commit) must not contain
    any positive fail-silent default. A regression that adds one --
    or worse, that reintroduces a defaulted safety value -- must fail
    this test loudly."""
    ds = parse_21(str(REPO / "docs" / "21-实测与第三方欠账.md"))
    r = Report()
    r.debts = ds
    check_fail_loud(ds, r)
    assert not r.fail_silent_hits, \
        "21册 has positive fail-silent rows: %s" % r.fail_silent_hits


# --- Coverage bi-diff ----------------------------------------------

def test_coverage_returns_uncovered_when_tests_dir_empty(tmp_path):
    """New checker: an empty tests/debt/ means every debt is uncovered."""
    ds = parse_21(str(REPO / "docs" / "21-实测与第三方欠账.md"))
    r = Report()
    r.debts = ds
    empty_dir = tmp_path / "empty"
    empty_dir.mkdir()
    check_coverage(ds, empty_dir, r)
    assert len(r.uncovered) >= 30
    assert not r.ghost_tests


def test_coverage_bidiff_flags_ghost_test(tmp_path):
    """VARIANT: create a test file for an ID that isn't in the doc.
    That must land in ghost_tests."""
    ds = parse_21(str(REPO / "docs" / "21-实测与第三方欠账.md"))
    r = Report()
    r.debts = ds
    d = tmp_path / "tests"
    d.mkdir()
    (d / "test_v-999.py").write_text("def test_x(): pass\n")
    check_coverage(ds, d, r)
    assert "V-999" in r.ghost_tests


def test_coverage_recognises_present_test_file(tmp_path):
    """POSITIVE: a test file whose stem maps to a real debt ID must
    remove that ID from uncovered."""
    ds = parse_21(str(REPO / "docs" / "21-实测与第三方欠账.md"))
    v02_present = any(d.id == "V-02" for d in ds)
    assert v02_present, "test premise wrong -- V-02 not in 21"

    d = tmp_path / "tests"
    d.mkdir()
    (d / "test_v-02.py").write_text(
        "def test_v_02_default_behavior(): assert True\n")

    r = Report()
    r.debts = ds
    check_coverage(ds, d, r)
    assert "V-02" not in r.uncovered


# --- CLI ------------------------------------------------------------

def test_cli_json_output_is_valid_json():
    r = subprocess.run(
        [sys.executable, str(CHECKER), "--format=json"],
        capture_output=True, text=True, timeout=15)
    assert r.returncode == 0, r.stderr
    doc = json.loads(r.stdout)
    for k in ("total", "uncovered", "ghost_tests",
              "missing_fail_loud", "fail_silent_hits"):
        assert k in doc


def test_cli_strict_coverage_exits_nonzero_on_gaps(tmp_path):
    """--strict-coverage promotes coverage gaps from informational to
    a hard failure. With no debt tests today, strict must exit non-zero."""
    r = subprocess.run(
        [sys.executable, str(CHECKER),
         "--tests-dir", str(tmp_path),
         "--strict-coverage"],
        capture_output=True, text=True, timeout=15)
    # tmp_path is empty; every debt is uncovered; strict => non-zero.
    assert r.returncode != 0


# --- Head-comment lineage -------------------------------------------

def test_checker_head_names_lineage():
    head = "\n".join(CHECKER.read_text().splitlines()[:12])
    assert "INF-DB-1" in head
    assert "上海哈船智能船舶技术有限公司" in head
