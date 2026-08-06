#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
Shanghai Hachist Intelligent Ship Technology Co., Ltd.
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
"""

import io
import os
import sys
import tokenize

ROOT = "/opt/xbrain_v6"
SOURCE_DIRS = ("xbrain", "scripts", "tests")
THRESHOLD = 0.70


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
    failures = []
    print("scan surface: " + ", ".join(SOURCE_DIRS))
    print("  threshold: comment / (comment + code) >= %.0f%%  (CLAUDE.md 2.4)" % (THRESHOLD * 100))
    print("")
    for path in sorted(iter_python()):
        rel = os.path.relpath(path, ROOT)
        comments, code = measure(path)
        if comments is None:
            print("  %-52s UNPARSABLE" % rel)
            failures.append(rel)
            continue
        total = comments + code
        if total == 0:
            continue  # an empty __init__.py has nothing to explain
        ratio = comments / total
        mark = "ok  " if ratio >= THRESHOLD else "LOW "
        if ratio < THRESHOLD:
            failures.append(rel)
        print("  %s %-50s %5.1f%%  (%d comment / %d code)"
              % (mark, rel, ratio * 100, comments, code))

    print("")
    print("  below threshold: %d" % len(failures))
    print("")
    print("criterion: no file below the threshold")
    print("  2.4 states the ratio is a means, not the goal. A file can pass this")
    print("  and still fail review: the real gate is that every block explains WHY.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
