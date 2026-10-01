#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: comment_ratio.py
Brief: Enforce CLAUDE.md 2.4 -- comment lines / (code + comment) >= 70 percent

Description:
What problem this solves. CLAUDE.md 2.4 sets a floor on comment density, raised
from 25 to 70 percent by the user on 2026-08-06. The section is explicit that the
ratio is a means and not the goal -- the real gate is that no code block goes
unexplained and every block says why, not what -- but a number nothing evaluates
drifts, so this computes it at run time.

Counting rules, chosen so the number cannot be gamed in either direction:
  * blank lines count as neither code nor comment, so padding a file with empty
    lines changes nothing
  * a docstring counts as comment, because in Python that is where the why lives
  * a trailing comment on a code line counts the line once, as comment, since
    the line does carry an explanation
  * a line that is only a string expression and not a docstring counts as code

What it deliberately does NOT do. It does not judge comment quality. A file can
pass this and still fail 2.4, whose real requirement is that comments explain
why. That is a review judgement and this script does not pretend to make it --
reporting a percentage as though it settled the question would be exactly the
kind of number this project keeps catching.

The ratio is printed, never written back into any document (CLAUDE.md 3.7).

================================================================================
THE RATCHET, and why a one-time widening here is not the usual way a ratchet
dies. Read this before touching the snapshot (user ruling, 2026-10-01, option B
-- ratchet rather than a bulk rewrite of the tree).

The state this replaces. The threshold applies to xbrain/, scripts/ and tests/
with no exemption, and the large majority of the files in those trees have never
met it. So this check has been red since the day it was written and on every run
since. A permanently red gate is CLAUDE.md 3.2 form 2: it gets read as noise,
then it gets loosened until it passes, and the loosening is what finally kills
it. The repository has already paid for that shape once -- tests/meta/
test_marker_coverage.py had a gate that was green BY CONSTRUCTION while a debt
ceiling above it stayed red for weeks, and nothing could fire.

What is now enforced, and it is MORE than before, not less:
  * every file NOT in the snapshot must meet the threshold, in all three trees.
    Before this, only xbrain/ was asserted anywhere (tests/common/test_lints.py
    measured scripts/ and tests/ and deliberately did not fail on them). A new
    file under scripts/ or tests/ below the threshold was previously invisible
    and is now red.
  * a snapshot row whose file has RISEN to the threshold is red: the row must
    come out in the same commit. That is what makes the list shrink by itself
    rather than merely stop growing.
  * a snapshot row naming a file that is no longer scanned is red, so a rename
    cannot carry debt along silently.

What holds the line is NOT a number. It is the snapshot file plus those three
rules. Adding a row to the snapshot is a one-line diff with a filename in it,
which is a thing a reviewer can refuse; the debt ceiling in tests/common/
test_comment_ratio.py is the backstop for that edit and may only ever be
lowered.

A snapshot file rather than an inline tuple, for the reason 019f1ec records for
the marker allowlist: hundreds of paths inline get re-sorted and re-wrapped by
every formatter that touches the file, and the diff noise hides an addition.
One path per line makes "somebody added a row" a one-line diff.

*** There is deliberately NO flag that regenerates the snapshot. A --write-debt
option would make the gate self-silencing: the first response to a red run would
be to run it, and the ratchet would be gone with no diff worth reading. Rows go
in by hand or not at all.
================================================================================
"""

import io
import os
import sys
import tokenize

ROOT = "/opt/xbrain_v6"
SOURCE_DIRS = ("xbrain", "scripts", "tests")
THRESHOLD = 0.70

#: The debt snapshot. Resolved from this file's own location rather than from
#: ROOT so the two can be repointed independently: tests/common/
#: test_comment_ratio.py runs the script against a fixture tree AND against a
#: fixture snapshot, and a snapshot derived from ROOT could not be varied on its
#: own. Read inside main(), never at import, for the same reason.
DEBT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "_comment_ratio_debt.txt")


def load_debt():
    """The snapshot as a set of repo-relative paths, or an empty set.

    A missing file is NOT an error and NOT a silent pass: it means every file is
    held to the threshold, which is the strict direction. The report says which
    file was read and how many rows it had, so "the snapshot did not load" can
    never look like "there is no debt".
    """
    try:
        with open(DEBT_PATH, encoding="utf-8") as fh:
            return {ln.strip() for ln in fh if ln.strip()}
    except OSError:
        return set()


def iter_python():
    for top in SOURCE_DIRS:
        base = os.path.join(ROOT, top)
        if not os.path.isdir(base):
            continue
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [d for d in dirnames
                           if d not in ("__pycache__", ".git")
                           and not d.startswith("model")]
            for name in filenames:
                if name.endswith(".py"):
                    yield os.path.join(dirpath, name)


def measure(path):
    """(comment_lines, code_lines) for one file, blanks excluded.

    Uses the tokenizer rather than a line scan so a hash inside a string is not
    mistaken for a comment and a docstring is not mistaken for code.
    """
    src = open(path, encoding="utf-8").read()
    comment_lines = set()
    code_lines = set()

    try:
        tokens = list(tokenize.generate_tokens(io.StringIO(src).readline))
    except (tokenize.TokenError, IndentationError, SyntaxError):
        # An unparsable file is reported rather than silently scored zero: a
        # scorer that treats "could not read" as "no comments" would push the
        # whole project below threshold for a reason nobody can act on.
        return None, None

    prev_significant = None
    for tok in tokens:
        ttype, _string, (srow, _scol), (erow, _ecol), _line = tok
        if ttype == tokenize.COMMENT:
            comment_lines.update(range(srow, erow + 1))
        elif ttype == tokenize.STRING:
            # A string is a docstring when it is the first significant token of
            # a module, class or function body.
            if prev_significant in (None, tokenize.INDENT, tokenize.NEWLINE,
                                    tokenize.DEDENT):
                comment_lines.update(range(srow, erow + 1))
            else:
                code_lines.update(range(srow, erow + 1))
            prev_significant = ttype
        elif ttype in (tokenize.NL, tokenize.NEWLINE, tokenize.INDENT,
                       tokenize.DEDENT, tokenize.ENDMARKER, tokenize.ENCODING):
            # prev MUST advance for NEWLINE, INDENT and DEDENT. Without it, prev
            # stays on the ":" of "def f():" and every function docstring is
            # classified as code -- the tool then under-reports comments on
            # exactly the files that document themselves best, which is the
            # opposite of what it exists to measure.
            #
            # NL is excluded on purpose: it is the non-logical newline that
            # appears inside brackets, so treating it as a boundary would make
            # the second line of a multi-line call look like a docstring
            # position. The same distinction bit the charset fixer, where it
            # rewrote quote characters inside a string literal.
            if ttype != tokenize.NL:
                prev_significant = ttype
            continue
        else:
            code_lines.update(range(srow, erow + 1))
            prev_significant = ttype

    # A line carrying both code and a trailing comment counts as comment: it
    # does explain itself, and counting it as code would penalise the very
    # style 2.4 asks for.
    code_lines -= comment_lines
    return len(comment_lines), len(code_lines)


def self_test():
    """Prove measure() actually distinguishes a docstring from code.

    This exists because the fix that made it do so was first written against the
    wrong variable name and became a silent no-op: the file changed, the tool
    reported the same numbers, and nothing said otherwise. A measuring tool that
    can be broken without any output changing is a tool whose output cannot be
    trusted -- so its own behaviour gets a probe.
    """
    import tempfile
    cases = [
        ("def f():\n    \"\"\"doc\"\"\"\n    return 1\n", 1, 2,
         "function docstring must count as comment"),
        ("x = 1\ny = 2\n", 0, 2, "plain code has no comments"),
        ("# top\nx = 1\n", 1, 1, "hash comment counts"),
        ("f(\n    \"not a docstring\",\n)\n", 0, 3,
         "a string on a continuation line is data, not a docstring"),
    ]
    ok = True
    for src, want_c, want_code, why in cases:
        with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as fh:
            fh.write(src)
            path = fh.name
        got_c, got_code = measure(path)
        os.unlink(path)
        mark = "ok " if (got_c, got_code) == (want_c, want_code) else "FAIL"
        if mark == "FAIL":
            ok = False
        print("  %s %-56s want (%d,%d) got (%s,%s)"
              % (mark, why, want_c, want_code, got_c, got_code))
    print("")
    print("self-test: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    if "--self-test" in sys.argv:
        return self_test()
    debt = load_debt()
    # new_low gates; carried is declared debt; cleared and orphan gate because
    # they are the two ways the snapshot stops shrinking.
    new_low, carried, cleared, unparsable = [], [], [], []
    seen = set()

    print("scan surface: " + ", ".join(SOURCE_DIRS))
    print("  threshold: comment / (comment + code) >= %.0f%%  (CLAUDE.md 2.4)" % (THRESHOLD * 100))
    print("  debt snapshot: %s (%d row(s))"
          % (os.path.relpath(DEBT_PATH, ROOT), len(debt)))
    print("")
    for path in sorted(iter_python()):
        rel = os.path.relpath(path, ROOT)
        seen.add(rel)
        comments, code = measure(path)
        if comments is None:
            print("  %-52s UNPARSABLE" % rel)
            unparsable.append(rel)
            continue
        total = comments + code
        if total == 0:
            continue  # an empty __init__.py has nothing to explain
        ratio = comments / total
        if ratio >= THRESHOLD:
            # A file that has risen out of the snapshot must LEAVE it. Without
            # this the list would only ever stop growing; with it, the list
            # shrinks on its own and the shrink is visible in review.
            mark = "RISEN" if rel in debt else "ok   "
            if rel in debt:
                cleared.append(rel)
        elif rel in debt:
            mark = "debt "
            carried.append(rel)
        else:
            mark = "LOW  "
            new_low.append(rel)
        print("  %s %-50s %5.1f%%  (%d comment / %d code)"
              % (mark, rel, ratio * 100, comments, code))

    # A row that names nothing the walk reached. A rename must not carry its
    # debt along invisibly, so the row is a finding rather than a no-op.
    orphan = sorted(r for r in debt if r not in seen)

    if new_low:
        print("")
        print("  BELOW THRESHOLD and not in the snapshot -- these gate:")
        for rel in new_low:
            print("    %s" % rel)
    if cleared:
        print("")
        print("  RISEN to the threshold while still in the snapshot. Delete")
        print("  these rows from %s in the same commit:"
              % os.path.relpath(DEBT_PATH, ROOT))
        for rel in cleared:
            print("    %s" % rel)
    if orphan:
        print("")
        print("  snapshot rows naming no scanned file (renamed or deleted?).")
        print("  Delete them; a rename must not carry its debt along unseen:")
        for rel in orphan:
            print("    %s" % rel)

    print("")
    print("  gating findings:   %d" % (len(new_low) + len(cleared)
                                       + len(orphan) + len(unparsable)))
    print("  carried debt:      %d" % len(carried))
    print("")
    print("criterion: no file below the threshold EXCEPT the ones the snapshot")
    print("  names, and the snapshot may only shrink. Three ways to fail: a file")
    print("  below the threshold that is not in the snapshot, a row whose file")
    print("  has risen above it, and a row naming nothing.")
    print("  What this does NOT establish: that the carried files are acceptable.")
    print("  They are declared debt, not exemption -- CLAUDE.md 2.4 states the")
    print("  ratio is a means and the real gate is that every block explains WHY,")
    print("  which no scan can decide. The snapshot is the list of files where")
    print("  nobody has done that work yet, and it is allowed to get shorter and")
    print("  nothing else.")
    return 1 if (new_low or cleared or orphan or unparsable) else 0


if __name__ == "__main__":
    sys.exit(main())
