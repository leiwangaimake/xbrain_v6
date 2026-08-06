"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_lints.py
Brief: Invoke every lint, so "enters the CI gate" stops being an unbacked claim

Description:
What problem this solves. CLAUDE.md 8.2 lists these checks as PR gates and 2.2
and 2.4 say each one 进 CI 门禁. Until this file existed, nothing ran any of
them: they were scripts a person could choose to run, and a rule nobody evaluates
is not a rule. That is the same shape as the QC-1..QC-17 family, which 13 S8.3
attributed to a service that evaluated none of them -- five bad configurations
would have booted green.

CFG-CM-11 is the item that adds the fifth lint and calls it a CI hard rule, so
the runner belongs with it: without one, the new check is a script, not a rule.

*** On comment_ratio specifically, and being exact about what is claimed.
CLAUDE.md 2.4 sets the threshold at 70 percent with no exemption for any tree,
and comment_ratio.py scans xbrain/, scripts/ and tests/. Today xbrain/ meets it
and the other two do not -- a pre-existing gap, not one this item created. So
this file asserts the part that is actually in force (every file under xbrain/)
and MEASURES the rest, printing it rather than hiding it. It does not assert
repo-wide compliance, because that would be false, and it does not silence the
check, because a silenced check is how a gap becomes permanent.

The counts are derived here at run time and written down nowhere -- CLAUDE.md
3.7: a number maintained by hand is a number that goes stale.
"""

import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LINT_DIR = os.path.join(ROOT, "scripts", "lint")

#: Lints whose criterion the whole repository meets today, so the runner can
#: assert exit 0 outright. Each entry is (script, what its zero means).
CLEAN_LINTS = [
    ("charset_lint.py",
     "no emoji or non-ASCII punctuation in any comment, and none in a string "
     "outside the declared frozen trees (CLAUDE.md 2.2)"),
    ("header_lint.py",
     "every file that has a header has all five fields and the Chinese company "
     "name (CLAUDE.md 2.5, 2.5.0)"),
    ("no_config_source_read.py",
     "no unmarked reach for the configuration source under xbrain/, ros2_ws/ or "
     "services/ (CFG-CM-11)"),
    # Added with CFG-CM-12. Its zero is narrower than it looks and the script
    # prints the boundary itself; tests/common/test_clock.py holds the
    # metatests, including the three mutations the item names.
    ("clock_scan.py",
     "no unmarked wall-clock read under common/, xbrain/, ros2_ws/ or "
     "services/, which is the automatable half of 11 S0.2.1 CLK-C1 "
     "(CFG-CM-12)"),
    # Added with CFG-CM-2. Its zero means no E_* is spelled in quotes anywhere a
    # process could send it, which is what makes 11 S13.8 checkable rather than
    # hoped for; tests/common/test_no_literal_ecode.py holds the metatests,
    # including the mutation the item names verbatim.
    ("no_literal_ecode.py",
     "no E_* code spelled as a string literal under xbrain/, ros2_ws/, "
     "services/ or common/, apart from the exemptions declared with a marker "
     "(CFG-CM-2)"),
]

#: Lints that also ship a --self-test. Running it here keeps the probes honest;
#: a self-test nobody invokes rots exactly as fast as a lint nobody invokes.
SELF_TESTED = ["comment_ratio.py", "no_config_source_read.py", "clock_scan.py",
               "no_literal_ecode.py"]


def run_lint(script, *args):
    """(returncode, stdout+stderr) for one lint."""
    proc = subprocess.run([sys.executable, os.path.join(LINT_DIR, script)] + list(args),
                          capture_output=True, text=True)
    return proc.returncode, proc.stdout + proc.stderr


@pytest.mark.parametrize("script,meaning", CLEAN_LINTS,
                         ids=[s for s, _ in CLEAN_LINTS])
def test_lint_passes(script, meaning):
    """*** The gate itself.

    Mutation: introduce any violation the lint is for => red. That is what makes
    this a gate rather than a formality, and each of the three has its own
    mutation coverage in its --self-test or its own metatest.
    """
    rc, out = run_lint(script)
    assert rc == 0, "%s fails; its zero would have meant: %s\n%s" % (script, meaning, out)


@pytest.mark.parametrize("script", SELF_TESTED)
def test_lint_self_test_passes(script):
    """The probes each lint carries for itself.

    comment_ratio.py was silently a no-op through an entire fix once -- the edit
    landed, the output did not change, and nothing signalled. Its --self-test
    exists because of that, and this runs it.
    """
    rc, out = run_lint(script, "--self-test")
    assert rc == 0, out
    assert "PASS" in out, out


def test_every_lint_script_is_invoked_by_this_runner():
    """*** Guards the guard.

    A sixth lint added to scripts/lint/ and wired into nothing would leave this
    file green while the new rule went unevaluated -- which is precisely the
    state this runner was written to end. So the set of scripts on disk is
    compared with the set this file runs.

    charset_fix.py is excluded by name: it is a rewriter, not a checker, and
    running it would modify the tree.
    """
    on_disk = {f for f in os.listdir(LINT_DIR)
               if f.endswith(".py") and f != "charset_fix.py"}
    invoked = {s for s, _ in CLEAN_LINTS} | set(SELF_TESTED) | {"comment_ratio.py"}
    unrun = on_disk - invoked
    assert not unrun, (
        "these lints are not invoked by any test, so nothing evaluates them: %s"
        % sorted(unrun)
    )


def test_comment_ratio_holds_for_the_tree_it_is_enforced_on():
    """*** The part of CLAUDE.md 2.4 that is actually in force today.

    xbrain/ was brought to the threshold deliberately and must stay there.
    Mutation: add a file under xbrain/ below 70 percent => red.
    """
    _rc, out = run_lint("comment_ratio.py")
    offenders = [ln.strip() for ln in out.split("\n")
                 if ln.strip().startswith("LOW") and " xbrain/" in ln]
    assert not offenders, (
        "every file under xbrain/ must meet the CLAUDE.md 2.4 threshold:\n"
        + "\n".join(offenders)
    )


def test_the_comment_ratio_gap_outside_xbrain_is_reported_not_hidden():
    """*** Measures the pre-existing gap instead of asserting it away.

    CLAUDE.md 2.4 states the threshold with no exemption for any tree, and
    comment_ratio.py scans scripts/ and tests/ as well. Neither meets it today.
    That is a real gap and it predates CFG-CM-11.

    This case deliberately does NOT fail. Failing it would make the suite red for
    a decision nobody has taken yet, and the usual response to a permanently red
    assertion is to delete it -- CLAUDE.md 3.2 form 2 becoming form 1. Instead it
    prints what is outstanding, so extending the rule to those trees is a choice
    someone makes with the size in front of them rather than a thing that quietly
    never happens.

    If the decision is taken to enforce it everywhere, this case is replaced by
    an assertion on the whole output, and test_lint_passes gains comment_ratio.
    """
    _rc, out = run_lint("comment_ratio.py")
    outstanding = [ln.strip() for ln in out.split("\n")
                   if ln.strip().startswith("LOW") and " xbrain/" not in ln]
    if outstanding:
        print("\ncomment ratio below the CLAUDE.md 2.4 threshold outside xbrain/ "
              "(pre-existing, not asserted here -- see this test's docstring):")
        for line in outstanding:
            print("  " + line)
    # The one thing that IS asserted: the check still runs and still reports.
    # A comment_ratio.py that printed nothing would make the list empty and this
    # case would read as "nothing outstanding".
    assert "scan surface" in out, "comment_ratio produced no report at all"


def test_no_lint_declares_a_criterion_it_cannot_reach():
    """CLAUDE.md 3.2 form 2: an unreachable criterion gets loosened until it passes.

    Every lint prints a line beginning 'criterion:'. This asserts the line is
    there -- a lint that reports numbers without stating what would count as
    passing leaves the reader to invent one, and the invented one is always the
    one the current output satisfies.
    """
    for script, _meaning in CLEAN_LINTS:
        _rc, out = run_lint(script)
        assert "criterion" in out, "%s prints no criterion" % script
