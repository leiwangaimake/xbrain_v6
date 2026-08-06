#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: charset_fix.py
Brief: Rewrite CJK punctuation and decorative symbols to ASCII, comments only

Description:
What problem this solves. charset_lint.py reports CLAUDE.md 2.2 violations but
deliberately does not fix them. This does the fixing, and it fixes exactly one
class: characters inside comments and docstrings. Those are pure text and a
substitution there cannot change behaviour.

Why it refuses to touch string literals. A CJK comma inside a printed string is
a different defect: CLAUDE.md 2.1 says logs, prints and exception messages must
be entirely English, so the correct repair is a translation, not a character
swap. Swapping punctuation inside an output string would also silently change
what the program prints, and any test asserting on that output would start
failing for a reason unrelated to what was being fixed. So those are reported and
left alone.

Why substitution is not blind. The decorative markers carry a severity scale in
this project -- three stars means "this has bitten us", one star means "worth
knowing". Mapping them all to nothing would delete that signal, so each maps to
an ASCII form that keeps the ranking readable.

Usage:
  charset_fix.py --dry-run [path ...]     report what would change
  charset_fix.py --apply   [path ...]     rewrite in place
With no paths, walks the same source tree charset_lint.py scans.
"""

import io
import os
import sys
import tokenize

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import charset_lint as L  # noqa: E402

# Decorative markers keep their ranking. Three stars is the project's "this has
# bitten us" mark and losing it would flatten the severity scale that the
# comments rely on.
SYMBOL_MAP = {
    chr(0x2605): "*",       # star
    chr(0x2606): "*",       # white star
    chr(0x26a0): "!",       # warning sign, a bare marker
    chr(0x1f6ab): "!!",     # no-entry: a bare marker, not a word.
                            # An earlier draft mapped it to "NEVER" and produced
                            # "NEVER Never write ..." wherever the sentence already
                            # said so. A marker must not compete with the prose.
    chr(0x2705): "OK",
    chr(0x274c): "NO",
    chr(0x1f534): "!",
    chr(0x2611): "[x]",
    chr(0x2610): "[ ]",
    chr(0x2192): "->",
    chr(0x21d2): "=>",
    chr(0x2326): "[del]",
    chr(0x25cf): "*",
    chr(0x25c6): "*",
    chr(0x25a0): "*",
    chr(0x25b2): "^",
    chr(0x25bc): "v",
    chr(0x2714): "OK",
    chr(0x2718): "NO",
    chr(0xfe0f): "",        # variation selector, invisible; drop it
}

PUNCT_MAP = {
    chr(0xff0c): ",", chr(0x3002): ".", chr(0xff1b): ";", chr(0xff1a): ":",
    chr(0xff1f): "?", chr(0xff01): "!",
    chr(0x201c): '"', chr(0x201d): '"', chr(0x2018): "'", chr(0x2019): "'",
    chr(0xff08): "(", chr(0xff09): ")",
    chr(0x3010): "[", chr(0x3011): "]",
    chr(0x300a): "<", chr(0x300b): ">",
    chr(0x3001): ",", chr(0x2014): "--", chr(0x2026): "...", chr(0x00b7): ".",
    chr(0x300c): '"', chr(0x300d): '"',
    chr(0xff5e): "~", chr(0xff0e): ".", chr(0xff1d): "=", chr(0xff06): "&",
    chr(0xff5c): "|",
}

FULL_MAP = dict(SYMBOL_MAP)
FULL_MAP.update(PUNCT_MAP)


def translate(text):
    """Apply the map. Returns the rewritten text."""
    for bad, good in FULL_MAP.items():
        text = text.replace(bad, good)
    return text


def classify_lines(path):
    """(comment_lines, string_lines) -- line numbers holding each kind.

    Uses the tokenizer because a hash inside a string is not a comment and a
    docstring is not code. A line-based guess would rewrite inside string
    literals, which is the one thing this tool must not do.
    """
    src = open(path, encoding="utf-8").read()
    comment, string = set(), set()
    try:
        toks = list(tokenize.generate_tokens(io.StringIO(src).readline))
    except (tokenize.TokenError, IndentationError, SyntaxError):
        return None, None
    prev = None
    for ttype, _s, (srow, _sc), (erow, _ec), _ln in toks:
        if ttype == tokenize.COMMENT:
            comment.update(range(srow, erow + 1))
        elif ttype == tokenize.STRING:
            # Docstrings are comments for our purposes; other strings are data.
            # NL is a NON-logical newline -- it appears inside brackets. Treating
            # it as a statement boundary makes the second line of a multi-line
            # assert message look like a docstring, and translating there rewrote
            # quote characters INSIDE a string literal and broke the file. Only
            # NEWLINE (logical), INDENT and DEDENT open a place a docstring can sit.
            if prev in (None, tokenize.INDENT, tokenize.NEWLINE, tokenize.DEDENT):
                comment.update(range(srow, erow + 1))
            else:
                string.update(range(srow, erow + 1))
            prev = ttype
        elif ttype in (tokenize.NL, tokenize.NEWLINE, tokenize.INDENT,
                       tokenize.DEDENT, tokenize.ENDMARKER, tokenize.ENCODING):
            # prev MUST be updated here. An earlier draft skipped it, so after
            # "def f():" prev stayed at the OP token and every method docstring
            # was classified as a data string and left unfixed -- the fixer
            # reported success while the biggest comments in the file were
            # untouched.
            # NL must NOT update prev: inside brackets it is not a boundary.
            if ttype != tokenize.NL:
                prev = ttype
            continue
        else:
            prev = ttype
    return comment, string


def fix_file(path, apply_changes):
    """Rewrite comment lines only. Returns (fixed, skipped_in_strings)."""
    lines = open(path, encoding="utf-8").read().split("\n")
    comment, string = classify_lines(path)
    if comment is None:
        # Non-Python (shell, C++) has no tokenizer here. Only whole-line
        # comments are rewritten; anything else is left for a human, because
        # guessing at string boundaries is how a fixer corrupts code.
        comment = {i for i, ln in enumerate(lines, 1)
                   if ln.lstrip().startswith(("#", "//", "*", "/*"))}
        string = set()

    fixed = skipped = 0
    for i, line in enumerate(lines, 1):
        hits = sum(1 for ch in line if ch in FULL_MAP)
        if not hits:
            continue
        if i in comment:
            new = translate(line)
            if new != line:
                lines[i - 1] = new
                fixed += hits
        else:
            skipped += hits

    if apply_changes and fixed:
        open(path, "w", encoding="utf-8").write("\n".join(lines))
    return fixed, skipped


def main():
    apply_changes = "--apply" in sys.argv
    paths = [a for a in sys.argv[1:] if not a.startswith("-")]
    # Expand directory arguments. An earlier draft skipped them silently via
    # the isfile check below, so passing a directory reported nothing and looked
    # like the directory was already clean -- a fixer that under-reports is as
    # bad as a linter that does.
    targets = []
    for p in paths:
        full = p if os.path.isabs(p) else os.path.join(L.ROOT, p)
        if os.path.isdir(full):
            targets.extend(f for f in L.iter_sources()
                           if os.path.abspath(f).startswith(os.path.abspath(full) + os.sep))
        else:
            targets.append(full)
    targets = targets or list(L.iter_sources())

    tot_fixed = tot_skipped = 0
    for path in sorted(targets):
        if not os.path.isfile(path):
            continue
        fixed, skipped = fix_file(path, apply_changes)
        if fixed or skipped:
            rel = os.path.relpath(path, L.ROOT)
            print("  %-52s comments %3d  strings %3d" % (rel, fixed, skipped))
            tot_fixed += fixed
            tot_skipped += skipped

    print("")
    print("  %s in comments: %d" % ("fixed" if apply_changes else "would fix", tot_fixed))
    print("  left in string literals:  %d" % tot_skipped)
    print("")
    print("  String literals are NOT rewritten. A CJK comma inside a printed")
    print("  string is a CLAUDE.md 2.1 defect -- logs and messages must be")
    print("  English -- so the repair is a translation, not a character swap,")
    print("  and swapping would change what the program prints.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
