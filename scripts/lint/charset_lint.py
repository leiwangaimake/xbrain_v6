#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
Shanghai Hachist Intelligent Ship Technology Co., Ltd.
File: charset_lint.py
Brief: Enforce CLAUDE.md 2.2 -- ASCII punctuation and no special symbols in source

Description:
What problem this solves. CLAUDE.md 2.2 has forbidden emoji, star markers and
CJK punctuation in source files since the project started, and on 2026-08-06 a
self-check found the rule had been broken in 18 of our own .py files. The
first estimate quoted here was wrong -- a shell-quoting artifact inflated the
CJK-punctuation count. The real figures come from this script, never from a
hand-run grep. A rule nothing evaluates is not a rule, it is a wish.

What it checks, per CLAUDE.md 2.2 and the 2026-08-06 restatement:
  1. no emoji or decorative symbols anywhere in a source file, comments included
  2. ASCII punctuation only, in Chinese comments as well as English ones
  3. Chinese characters themselves stay allowed -- 2.1 lets a file use Chinese
     comments as long as it is consistent; what is banned is CJK punctuation and
     decorative marks, not Chinese words

What it deliberately does NOT do:
  * it does not touch docs/, because 2.2 exempts markdown by design -- the eleven
    design volumes use star and warning markers as a severity scale on purpose
  * it does not check comment ratio, that is comment_ratio.py
  * it does not rewrite anything; a linter that edits code hides what it changed

A trap worth naming. An earlier draft of this scan reported per-file counts only.
Counts tell you a file is dirty but not which line to open, and a developer who
cannot find the offending character gives up and adds an exemption. So every hit
is reported with file, line, column and the character itself.
"""

import os
import re
import sys
import unicodedata

ROOT = "/opt/xbrain_v6"

# Directories that hold source we own. services/ is included: it is our code too.
SOURCE_DIRS = ("xbrain", "scripts", "tests", "common", "ros2_ws", "services")

SOURCE_EXT = (".py", ".c", ".cc", ".cpp", ".h", ".hpp", ".sh", ".bash")

# Decorative symbols the project uses in markdown and forbids in source.
# Listed explicitly rather than by Unicode block: a block test would also catch
# characters that are legitimate in a comment, and an over-broad linter gets
# switched off.
# * Defined by codepoint, not by literal character. A linter written with the
# characters it forbids reports itself on every run; this project has caught
# that self-harming-criterion shape three times. Escapes keep this file clean.
BANNED_SYMBOLS = "".join(chr(c) for c in (
    0x2605, 0x2606, 0x26a0, 0x1f6ab, 0x2705, 0x274c, 0x1f534, 0x2611, 0x2610,
    0x2192, 0x21d2, 0x2326, 0x25cf, 0x25c6, 0x25a0, 0x25b2, 0x25bc, 0x2714, 0x2718,
))

# CJK and full-width punctuation. ASCII replacements exist for every one of them.
BANNED_PUNCT = "".join(chr(c) for c in (
    0xff0c, 0x3002, 0xff1b, 0xff1a, 0xff1f, 0xff01,
    0x201c, 0x201d, 0x2018, 0x2019,
    0xff08, 0xff09, 0x3010, 0x3011, 0x300a, 0x300b,
    0x3001, 0x2014, 0x2026, 0x00b7,
    0x300c, 0x300d, 0xff5e, 0xff0e, 0xff1d, 0xff06, 0xff5c,
))

_BANNED = BANNED_SYMBOLS + BANNED_PUNCT
_BANNED_RE = re.compile("[" + re.escape(_BANNED) + "]")

# Emoji and pictographs beyond the named list, so a symbol nobody thought of
# still gets caught. Restricted to the pictograph planes to avoid false hits on
# ordinary CJK text.
_EMOJI_RE = re.compile(
    "[\U0001f000-\U0001faff\u2600-\u27bf\ufe0f\U0001f1e6-\U0001f1ff]"
)


def iter_sources():
    """Every source file we own, with docs/ and vendor trees excluded."""
    for top in SOURCE_DIRS:
        base = os.path.join(ROOT, top)
        if not os.path.isdir(base):
            continue
        for dirpath, dirnames, filenames in os.walk(base):
            # Skip caches and any vendored model or third-party payload.
            dirnames[:] = [d for d in dirnames
                           if d not in ("__pycache__", ".git", "node_modules")
                           and not d.startswith("model")]
            for name in filenames:
                if name.endswith(SOURCE_EXT):
                    yield os.path.join(dirpath, name)


def scan(path):
    """(lineno, col, char, name) for every banned character in one file."""
    hits = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for lineno, line in enumerate(fh, 1):
            for col, ch in enumerate(line, 1):
                if _BANNED_RE.match(ch) or _EMOJI_RE.match(ch):
                    try:
                        name = unicodedata.name(ch)
                    except ValueError:
                        name = "UNNAMED"
                    hits.append((lineno, col, ch, name))
    return hits


def main():
    show_all = "-v" in sys.argv or "--verbose" in sys.argv
    total, dirty = 0, 0
    print("scan surface: " + ", ".join(SOURCE_DIRS) + " (docs/ exempt per CLAUDE.md 2.2)")
    for path in sorted(iter_sources()):
        hits = scan(path)
        if not hits:
            continue
        dirty += 1
        total += len(hits)
        rel = os.path.relpath(path, ROOT)
        print("  %-52s %d" % (rel, len(hits)))
        for lineno, col, ch, name in (hits if show_all else hits[:3]):
            print("      %s:%d:%d  %r  %s" % (rel, lineno, col, ch, name))
        if not show_all and len(hits) > 3:
            print("      ... %d more (use -v)" % (len(hits) - 3))

    print("")
    print("  files with violations: %d" % dirty)
    print("  total violations:      %d" % total)
    print("")
    print("criterion: total violations == 0")
    print("  ASCII punctuation is required in Chinese comments too (CLAUDE.md 2.2).")
    print("  Chinese WORDS stay allowed; what is banned is CJK punctuation and")
    print("  decorative symbols. Replace , . ; : ? ! ( ) with their ASCII forms.")
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
