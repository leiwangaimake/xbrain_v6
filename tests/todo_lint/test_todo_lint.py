"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_todo_lint.py
Brief: CHK-1-48 -- seven variants (a-g) each turn one rule red, plus a
       reverse-clean baseline that stays green

Description:
Each CHK-1-48 mutation is a real file check: we build a mini TODO table in
tmp_path (five rows across two phases), invoke scripts/todo_lint.py with
--todo pointing at it, and assert which rule reports the injected defect.

The mini TODO is deliberately minimal-but-realistic: two Phase-0 rows +
one Phase-1 row, each carrying task/dir/lang/criterion/deps/blocker cells
identical in shape to the real docs/XBRAIN_V6_TODO.md. Adding rows here
is legitimate; adding rows that trip a rule accidentally means the
mutations tests below need updating too.

Mutations verbatim from CHK-1-48 (2026-08-08 revision):
  (a) ID four-segment `ABC-XY-P4-1` -> rule-1 red (format check)
  (b) duplicate an ID -> rule-2 red (uniqueness)
  (c) dep references a non-existent ID -> rule-3 red (dep existence)
      (original phase-inversion variant is retired -- phase check
      dropped 2026-08-08, replaced by dep-existence variant)
  (d) two rows claim the same FILE (with extension) -> rule-4 red
      (directory-level double-claim is exempt; only file-level is a
      finding)
  (e) criterion carries no mutation-marker keyword and is not on the
      exempt list -> rule-5 red
  (f) criterion says "审查必须报" but names no executable body
      (no scripts/*, no ros2_ws/*, no T-*/M-*/rule-id, no clang-tidy
      etc.) -> rule-6 red
  (g) criterion says "全仓 X" without a self-declaring anchor
      (no grep/find/零命中/豁免/静态扫描/etc.) -> rule-7 red

Plus:
  reverse -- clean mini TODO must exit 0 across all rules
  meta   -- every rule appears in the runner (guard-the-guard)
"""

import os
import subprocess
import sys
import textwrap

import pytest

# Absolute path to the lint script we exercise -- same one that runs in CI
# and the same one the criterion names.
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_HERE))
LINT_SCRIPT = os.path.join(_REPO_ROOT, "scripts", "todo_lint.py")


# ---------------------------------------------------------------------------
# Clean-baseline mini TODO
# ---------------------------------------------------------------------------
#
# Two Phase-0 rows + one Phase-1 row. Each carries an ID that matches
# rule-1's shape, unique names, existing deps, distinct FILE paths in the
# 目录 column, a criterion with a mutation marker, no "审查必须报"
# unaccompanied, no bare "全仓" without an anchor.
#
# textwrap.dedent so the file can be indented in source without leading
# whitespace polluting the header parser.

CLEAN_TODO = textwrap.dedent("""\
    # Mini TODO for CHK-1-48 mutation tests

    ## Phase 0 · foundation

    | ID | 任务 | 目录 | 语言 | 判据 | 依赖 | 阻塞 |
    |---|---|---|---|---|---|---|
    | `ABC-XY-1` | task one | `foo/a.py` | Python | pytest 绿; 变异体: 注入 X ⇒ 必须变红 | — | — |
    | `ABC-XY-2` | task two | `foo/b.py` | Python | ctest 绿; 必须变红 | `ABC-XY-1` | — |

    ## Phase 1 · runtime

    | ID | 任务 | 目录 | 语言 | 判据 | 依赖 | 阻塞 |
    |---|---|---|---|---|---|---|
    | `DEF-ZW-3` | task three | `foo/c.py` | Python | 变异体: 修 Y ⇒ 必须变红 | `ABC-XY-1`, `ABC-XY-2` | — |
    """)


def _write_todo(tmp_path, text):
    """Write text to tmp_path/todo.md and return the path."""
    p = tmp_path / "todo.md"
    p.write_text(text, encoding="utf-8")
    return p


def _run_lint(todo_path):
    """Invoke scripts/todo_lint.py --todo <path> and return
    (exit_code, combined_stdout_stderr)."""
    # Use the same interpreter that runs pytest so venv/deps match. subprocess
    # so we exercise the CLI shape (argparse, main(), exit code) exactly as CI
    # will invoke it -- not the internal Python API.
    result = subprocess.run(
        [sys.executable, LINT_SCRIPT, "--todo", str(todo_path)],
        capture_output=True, text=True,
    )
    return result.returncode, result.stdout + result.stderr


# ---------------------------------------------------------------------------
# Reverse (baseline)
# ---------------------------------------------------------------------------

def test_clean_mini_todo_is_green(tmp_path):
    """Baseline: the clean mini TODO exits 0 with every rule green.
    If this ever fails, one of the seven rules over-reports on innocuous
    content -- fix the rule (or the baseline) before touching the mutants."""
    p = _write_todo(tmp_path, CLEAN_TODO)
    rc, out = _run_lint(p)
    # Fail with captured stdout to make regressions one-step debug.
    assert rc == 0, "clean mini TODO should exit 0; output:\n" + out
    # Sanity: every rule ran (guard-the-guard against a silent no-op).
    for rule in ("rule-1", "rule-2", "rule-3", "rule-4",
                 "rule-5", "rule-6", "rule-7"):
        assert rule in out, "rule %s should have run; out:\n%s" % (rule, out)


# ---------------------------------------------------------------------------
# Mutation (a) -- four-segment ID -> rule-1
# ---------------------------------------------------------------------------

def test_variant_a_four_segment_id_is_red(tmp_path):
    """Verbatim: 'change some ID to four segments (like GWY-CM-P4-1)'.
    A stray extra hyphen segment breaks rule-1's format anchor and the
    line goes to rule-1's finding list."""
    mutated = CLEAN_TODO.replace("`ABC-XY-1`", "`ABC-XY-P4-1`", 1)
    p = _write_todo(tmp_path, mutated)
    rc, out = _run_lint(p)
    assert rc == 1, "mutation (a) should be red; output:\n" + out
    assert "rule-1" in out and "ABC-XY-P4-1" in out


# ---------------------------------------------------------------------------
# Mutation (b) -- duplicate an ID -> rule-2
# ---------------------------------------------------------------------------

def test_variant_b_duplicate_id_is_red(tmp_path):
    """Duplicate the first Phase-0 row wholesale. rule-2 reports the
    second occurrence with a pointer at the first."""
    dup = CLEAN_TODO.replace(
        "| `ABC-XY-2` | task two | `foo/b.py`",
        "| `ABC-XY-1` | duplicate | `foo/d.py`",
        1,
    )
    p = _write_todo(tmp_path, dup)
    rc, out = _run_lint(p)
    assert rc == 1, "mutation (b) should be red; output:\n" + out
    assert "rule-2" in out and "ABC-XY-1" in out


# ---------------------------------------------------------------------------
# Mutation (c) -- dep points at a non-existent ID -> rule-3
# ---------------------------------------------------------------------------

def test_variant_c_dep_does_not_exist_is_red(tmp_path):
    """Point DEF-ZW-3's dep column at an ID no row defines. rule-3
    reports the dangling reference.

    (Original variant was 'CPP-BP-4 depends on CPP-BP-1 without phase
    change -> P0 depends on P3'. 2026-08-08 phase constraint is dropped;
    the equivalent test is now 'dep must exist' -- a non-existent ID.)
    """
    mutated = CLEAN_TODO.replace(
        "| `ABC-XY-1`, `ABC-XY-2` |",
        "| `NONEXIST-XY-99` |",
        1,
    )
    p = _write_todo(tmp_path, mutated)
    rc, out = _run_lint(p)
    assert rc == 1, "mutation (c) should be red; output:\n" + out
    assert "rule-3" in out and "NONEXIST-XY-99" in out


# ---------------------------------------------------------------------------
# Mutation (d) -- two rows claim the same FILE -> rule-4
# ---------------------------------------------------------------------------

def test_variant_d_two_ids_claim_same_file_is_red(tmp_path):
    """Two rows point at `foo/a.py` in the 目录 column. rule-4 reports
    the file with both claimants.

    NOTE: rule-4 only reports FILE-level (with extension); directory-level
    like `foo/` is exempt (the same directory routinely hosts many
    deliverables, one per ID). This test uses a file with extension.
    """
    mutated = CLEAN_TODO.replace(
        "| `ABC-XY-2` | task two | `foo/b.py`",
        "| `ABC-XY-2` | task two | `foo/a.py`",
        1,
    )
    p = _write_todo(tmp_path, mutated)
    rc, out = _run_lint(p)
    assert rc == 1, "mutation (d) should be red; output:\n" + out
    assert "rule-4" in out and "foo/a.py" in out


# ---------------------------------------------------------------------------
# Mutation (e) -- criterion carries no mutation marker -> rule-5
# ---------------------------------------------------------------------------

def test_variant_e_no_mutation_marker_is_red(tmp_path):
    """Strip the criterion of ANY mutation marker keyword. rule-5 reports
    the row.

    We replace ABC-XY-1's criterion cell wholesale with a plain sentence
    that contains none of the accepted keywords (no 变异体/必须变红/
    T-N/M-N/负例/etc). The row is also NOT on the exempt list (no
    "本项不写代码" phrase)."""
    mutated = CLEAN_TODO.replace(
        "pytest 绿; 变异体: 注入 X ⇒ 必须变红",
        "pytest 绿; task is complete when tests pass",
        1,
    )
    p = _write_todo(tmp_path, mutated)
    rc, out = _run_lint(p)
    assert rc == 1, "mutation (e) should be red; output:\n" + out
    assert "rule-5" in out and "ABC-XY-1" in out


# ---------------------------------------------------------------------------
# Mutation (f) -- 审查必须报 without executable body -> rule-6
# ---------------------------------------------------------------------------

def test_variant_f_audit_without_executable_body_is_red(tmp_path):
    """Put '审查必须报' in ABC-XY-1's criterion but name no executable
    body (no scripts/*, no ros2_ws/*, no T-*, no rule-id, no clang-tidy).
    rule-6 reports the row."""
    mutated = CLEAN_TODO.replace(
        "pytest 绿; 变异体: 注入 X ⇒ 必须变红",
        # Deliberately vague: '审查必须报 something bad' with no anchor.
        # Keep a mutation-marker so rule-5 stays green (mutation must be
        # attributable to rule-6 only).
        "must-变红; some-check: 审查必须报 badness",
        1,
    )
    p = _write_todo(tmp_path, mutated)
    rc, out = _run_lint(p)
    assert rc == 1, "mutation (f) should be red; output:\n" + out
    assert "rule-6" in out and "ABC-XY-1" in out


# ---------------------------------------------------------------------------
# Mutation (g) -- 全仓 without a self-declaring anchor -> rule-7
# ---------------------------------------------------------------------------

def test_variant_g_bare_quan_cang_is_red(tmp_path):
    """Put '全仓' in ABC-XY-1's criterion but no anchor (no grep/find/
    零命中/豁免/静态扫描/etc). rule-7 reports the row.

    (Under the widened rule, '全仓 grep' is self-declaring. The mutation
    must use '全仓' followed by NON-anchor phrasing.)
    """
    mutated = CLEAN_TODO.replace(
        "pytest 绿; 变异体: 注入 X ⇒ 必须变红",
        # '全仓 <bare noun>' with no verb -- no anchor, so rule-7 fires.
        # Keep the mutation-marker so rule-5 stays green.
        "变异体: X ⇒ 变红; 全仓 stuff shall be nice",
        1,
    )
    p = _write_todo(tmp_path, mutated)
    rc, out = _run_lint(p)
    assert rc == 1, "mutation (g) should be red; output:\n" + out
    assert "rule-7" in out and "ABC-XY-1" in out


# ---------------------------------------------------------------------------
# Meta: guard-the-guard -- every rule appears in the runner output
# ---------------------------------------------------------------------------

def test_meta_every_rule_appears_in_baseline_output(tmp_path):
    """A rule silently removed from RULES would fail to appear in the
    header printout. This test guards the guard: every rule name we
    expect must appear in the baseline output."""
    p = _write_todo(tmp_path, CLEAN_TODO)
    _, out = _run_lint(p)
    for rule in ("rule-1", "rule-2", "rule-3", "rule-4",
                 "rule-5", "rule-6", "rule-7"):
        assert rule in out, "rule %s missing from output:\n%s" % (rule, out)


# ---------------------------------------------------------------------------
# Live-repo smoke test -- default target must exit 0
# ---------------------------------------------------------------------------

def test_real_repo_todo_exits_zero():
    """The real docs/XBRAIN_V6_TODO.md must exit 0 -- CHK-1-48's own
    exit-0 requirement. If someone adds a row that trips a rule, this
    test goes red BEFORE the row lands, so the fix arrives in the same
    commit as the row that needs it."""
    real_todo = os.path.join(_REPO_ROOT, "docs", "XBRAIN_V6_TODO.md")
    rc, out = _run_lint(real_todo)
    assert rc == 0, "real TODO must exit 0; output:\n" + out
