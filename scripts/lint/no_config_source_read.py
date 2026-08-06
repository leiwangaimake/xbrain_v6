#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: no_config_source_read.py
Brief: Nothing under xbrain/ ros2_ws/ services/ may reach for the config source

Description:
What problem this solves. 10 S5.4.1 calls "references are not expanded by each
process individually" the most critical structural decision in the configuration
design, and the stated reason is that each process would otherwise resolve them
to different results. That guarantee holds only while every process reads
/run/xbrain/resolved/ and nothing reads /opt/xbrain_v6/configs/ at run time. It
is a whole-tree property, so no amount of care inside one module establishes it;
it needs something that looks at the whole tree, which is this.

Two rules, both enforced:

  RULE 1 -- no unmarked source-path literal. An occurrence of the configuration
  root as a literal path inside the scan surface must carry a line-level
  exemption marker naming why. The permitted reasons are exactly the freeze
  service itself and the source-file evaluation paths of startup assertions J, B
  and K -- those three genuinely evaluate on the L6 SOURCE, before expansion,
  because they exist to catch things that no longer exist afterwards. 13 S8.3
  states this for QC-1: it must distinguish DEFINING a spec. key from REFERENCING
  ${common.spec.*}, and the reference form is gone once expanded.

  RULE 2 -- no reaching for the source root through a symbol. Rule 1 alone is
  evadable without any subterfuge: os.path.join(DEFAULT_CONFIG_ROOT, "x.yaml")
  contains no literal at all. So the names that resolve to the source root are
  themselves restricted to the same exempt set. This is the rule that would
  actually have caught a realistic mistake; rule 1 catches the obvious one.

*** What this CANNOT establish, stated so the pass is not read as more than it
is. A path assembled at run time -- "/opt/xbrain_v6" + "/configs/x.yaml", or a
value arriving from a message -- matches neither rule. The criterion here is
therefore "no unmarked literal and no unmarked symbol use", NOT "no process
reads the configuration source". The latter is not decidable by a scan and
claiming it would be CLAUDE.md 3.2 form 7. The reader in
xbrain/common/config/resolved.py carries the run-time half: it confines every
path it opens to the resolved root, so a manifest naming a source path is
refused there rather than here.

Why this file lives in scripts/ and the scan surface does not include scripts/.
CLAUDE.md 3.2 form 3 is 判据自伤 -- a criterion whose own text is inside the
surface it scans, so its hit count can never reach zero and it gets "fixed" by
being loosened. Keeping the lint outside its own surface is the structural fix,
and tests/common/test_no_config_source_read.py asserts that property rather than
leaving it to whoever next edits SCAN_DIRS.

Run:
  python3 scripts/lint/no_config_source_read.py
  python3 scripts/lint/no_config_source_read.py --self-test
"""

import os
import re
import sys

ROOT = "/opt/xbrain_v6"

#: The scan surface, verbatim from the CFG-CM-11 criterion. scripts/ and tests/
#: are deliberately absent: scripts/ holds the freeze service and this lint, and
#: tests/ must be free to construct a source path in a fixture.
SCAN_DIRS = ("xbrain", "ros2_ws", "services")

SOURCE_EXT = (".py", ".c", ".cc", ".cpp", ".h", ".hpp", ".sh", ".bash", ".yaml",
              ".yml", ".json", ".json5", ".cmake", ".txt")

#: The configuration root, assembled from parts.
#:
#: Being accurate about what this buys, because the first draft of this comment
#: was not: it does NOT make this file free of the literal it searches for -- the
#: Description above spells the path out twice, in prose. What actually keeps
#: this lint out of its own scan surface is SCAN_DIRS, which does not include
#: scripts/, and tests/common/test_no_config_source_read.py asserts that.
#:
#: The split is still worth having. It means the constant survives if someone
#: later adds scripts/ to the surface, and it costs nothing. charset_lint.py
#: reported itself 47 times before it was rewritten this way, which is how the
#: technique got into this code base.
_CONFIG_ROOT_LITERAL = "/opt/" + "xbrain_v6/" + "configs"

#: Symbols that evaluate to the source root. Anything importing or referencing
#: one of these is reaching for the source just as surely as a literal would,
#: and without matching rule 1. Defined in xbrain/common/config/layers.py.
SOURCE_ROOT_SYMBOLS = ("DEFAULT_CONFIG_ROOT", "SAFETY_ROOT", "resolve_config_root",
                       "safety_root")

#: Rule 2 does not apply inside the package that DEFINES these symbols.
#:
#: Why this is narrowing and not loosening. Rule 2 asks "who is reaching for the
#: source root", and the config library is where the reaching is implemented --
#: layers.py refers to its own constants throughout, and the package __init__
#: re-exports them. Applying the rule there produced seventeen findings, every
#: one inside this package and not one of them a process reaching for anything;
#: a checker that fails everything is indistinguishable from a checker that is
#: broken, and the natural response to it is to switch it off. header_lint.py in
#: this repo did exactly that once, flagging 91 correct files.
#:
#: What survives the narrowing is the whole point of rule 2: a consumer writing
#: `from xbrain.common.config import resolve_config_root` is OUTSIDE this tree,
#: so its import line is still a violation. The rule keeps every case it was
#: written for.
#:
#: Declared here with its reason rather than applied silently, on the
#: FROZEN_STRING_TREES precedent in charset_lint.py, and printed in the report so
#: it cannot grow without someone seeing it.
SYMBOL_RULE_EXEMPT_TREES = {
    "xbrain/common/config": "the package that defines the source-root symbols; "
                            "rule 2 is about consumers reaching for them, and "
                            "every use inside here is the implementation itself",
}

#: The permitted exemption reasons, and what each one means. A marker naming
#: anything outside this set is itself a violation -- otherwise the mechanism
#: degrades into "write any word and the check goes quiet".
EXEMPT_TAGS = {
    "freeze": "the freeze service itself, which by definition reads the source "
              "and writes the product (10 S5.4.4, Stage 0c)",
    "J": "startup assertion J -- configuration root and file reachability; it "
         "stats the source files, so it must name them",
    "B": "startup assertion B -- no duplicate definition; evaluated on the L6 "
         "SOURCE because it looks for a `common` top-level key, which the "
         "expansion removes",
    "K": "startup assertion K -- quadruped private config, QC-1 specifically; "
         "13 S8.3 puts QC-1 on the L6 source because it must tell DEFINING a "
         "spec. key from REFERENCING ${common.spec.*}, and the reference form "
         "does not survive expansion",
}

#: The marker itself. Deliberately shouty and greppable: an exemption that a
#: reviewer can miss is not a visible marker, which is what the criterion asks
#: for. Accepted on the offending line or the line immediately above it, because
#: a long line wrapped by a formatter would otherwise lose its marker.
MARKER_RE = re.compile(r"CONFIG-SOURCE-OK\(([A-Za-z]+)\)\s*:\s*(\S.*)")

#: Directories never worth scanning.
SKIP_DIRS = {"__pycache__", ".git", "node_modules", "build", "install", "log"}


def _files_under(base):
    """Scannable files below one directory.

    Shared by iter_sources and by the per-directory count main() prints. One
    walk, so the reported count and the files actually scanned cannot drift
    apart -- a count computed by a second, slightly different walk is a number
    that reassures without measuring.
    """
    for dirpath, dirnames, filenames in os.walk(base):
        dirnames[:] = [d for d in dirnames
                       if d not in SKIP_DIRS and not d.startswith("model")]
        for name in filenames:
            if name.endswith(SOURCE_EXT):
                yield os.path.join(dirpath, name)


def iter_sources():
    """Every file in the scan surface.

    An absent directory is skipped here and REPORTED by main(). A scan surface
    that quietly does not exist produces zero hits, and zero hits from an
    unscanned tree is indistinguishable from zero hits from a clean one.
    """
    for top in SCAN_DIRS:
        base = os.path.join(ROOT, top)
        if not os.path.isdir(base):
            continue
        for path in _files_under(base):
            yield path


def comment_lines_of(path):
    """Line numbers holding comments or docstrings.

    Borrowed from charset_lint.py, which already solved this: telling a comment
    from a string literal needs the tokenizer, and a regex for it gets multi-line
    strings wrong in both directions.
    """
    if not path.endswith(".py"):
        out = set()
        try:
            with open(path, encoding="utf-8", errors="replace") as fh:
                for i, line in enumerate(fh, 1):
                    if line.lstrip().startswith(("#", "//", "*", "/*")):
                        out.add(i)
        except OSError:
            pass
        return out
    sys.path.insert(0, os.path.join(ROOT, "scripts", "lint"))
    try:
        import charset_fix
        commented, _strings = charset_fix.classify_lines(path)
        if commented is not None:
            return commented
    except Exception:
        # Falling back rather than failing: a tokenizer error on one file must
        # not take the whole lint down, and the fallback is conservative in the
        # direction that reports MORE as code.
        pass
    return set()


def marker_for(lines, idx):
    """(tag, reason) from a marker on this line or in the comment block above it.

    Scope of the search, and why it is what it is. Same line first, then upward
    through the CONTIGUOUS run of comment lines directly above, stopping at the
    first line that is not a comment.

    Not just idx-1: a real exemption needs a paragraph to justify itself, and
    DEFAULT_CONFIG_ROOT in layers.py carries eight lines of them. Requiring the
    marker on the last line before the code would push the explanation below the
    marker or force it onto one unreadable line, and an exemption nobody reads is
    the same as a silent one.

    Not "anywhere in the file" either: the block must be attached to this line,
    so a marker written for one occurrence cannot quietly cover a second one
    added forty lines later.
    """
    if idx < len(lines):
        m = MARKER_RE.search(lines[idx])
        if m:
            return m.group(1), m.group(2).strip()

    probe = idx - 1
    while probe >= 0:
        stripped = lines[probe].strip()
        # A blank line ends the block. Comment blocks in this code base are
        # unbroken, and allowing blanks would let the search wander into an
        # unrelated block above.
        if not stripped.startswith(("#", "//", "*", "/*")):
            break
        m = MARKER_RE.search(lines[probe])
        if m:
            return m.group(1), m.group(2).strip()
        probe -= 1
    return None


def symbol_rule_applies(rel_path):
    """Whether rule 2 is in force for this file, or the declared reason it is not."""
    for prefix, why in SYMBOL_RULE_EXEMPT_TREES.items():
        if rel_path == prefix or rel_path.startswith(prefix + os.sep):
            return False, why
    return True, None


def scan_file(path, rel_path=None):
    """(violations, exemptions, prose) for one file.

    Three buckets, not two, and they are reported separately for the same reason
    header_lint.py separates "no header" from "wrong header": they need different
    work, and one number would hide the mix.
    """
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            lines = fh.read().split("\n")
    except OSError as exc:
        return [("?", 0, "cannot read: %s" % exc)], [], []

    commented = comment_lines_of(path)
    # rel_path defaults to the path itself so the self-test, which works in a
    # temp directory, still exercises rule 2. A default of "" would silently put
    # every self-test case outside every exempt tree, which happens to be right
    # here but for the wrong reason.
    symbol_rule, _exempt_why = symbol_rule_applies(rel_path if rel_path else path)
    violations, exemptions, prose = [], [], []

    for i, line in enumerate(lines):
        lineno = i + 1

        # Rule 1: the source-root literal.
        hit_literal = _CONFIG_ROOT_LITERAL in line
        # Rule 2: a symbol that resolves to the source root. Word-boundary
        # matched so a longer name that merely contains one of them does not
        # trigger, and skipped on the line that DEFINES the symbol -- layers.py
        # has to be able to declare it.
        hit_symbol = None
        for sym in (SOURCE_ROOT_SYMBOLS if symbol_rule else ()):
            if re.search(r"\b" + sym + r"\b", line):
                # A definition (`NAME = ...` or `def name(`) is the declaration
                # site, which is exempt by nature: the symbol must exist
                # somewhere. Uses are what this rule is about.
                if re.match(r"\s*(def\s+)?" + sym + r"\s*[:=(]", line):
                    continue
                hit_symbol = sym
                break

        if not hit_literal and not hit_symbol:
            continue

        what = ("source-root literal" if hit_literal
                else "source-root symbol %s" % hit_symbol)

        marker = marker_for(lines, i)
        if marker:
            tag, reason = marker
            if tag not in EXEMPT_TAGS:
                violations.append((path, lineno,
                                   "exemption tag %r is not one of %s"
                                   % (tag, sorted(EXEMPT_TAGS))))
            elif len(reason) < 12:
                violations.append((path, lineno,
                                   "exemption gives no usable reason: %r" % reason))
            else:
                exemptions.append((path, lineno, tag, reason))
            continue

        # An unmarked LITERAL inside a comment is prose, not a read. Reported
        # separately and not failed: a comment cannot open a file, and demanding
        # a marker on explanatory text would push people to delete the
        # explanation instead. An unmarked SYMBOL is a violation wherever it is,
        # because a symbol in a comment is still a name someone can copy.
        if hit_literal and not hit_symbol and lineno in commented:
            prose.append((path, lineno, line.strip()[:90]))
            continue

        violations.append((path, lineno, "unmarked %s" % what))

    return violations, exemptions, prose


SELF_TEST_CASES = [
    # (filename, content, expected violation count, description)
    ("plain.py", 'p = "%s/p1_motion.yaml"\n' % _CONFIG_ROOT_LITERAL, 1,
     "a bare source-path literal is a violation"),
    ("marked.py",
     '# CONFIG-SOURCE-OK(J): assertion J stats the source files by name\n'
     'p = "%s/p1_motion.yaml"\n' % _CONFIG_ROOT_LITERAL, 0,
     "a marker on the preceding line exempts it"),
    ("marked_same_line.py",
     'p = "%s/x.yaml"  # CONFIG-SOURCE-OK(freeze): the freeze service reads source\n'
     % _CONFIG_ROOT_LITERAL, 0,
     "a marker on the same line exempts it"),
    ("bad_tag.py",
     '# CONFIG-SOURCE-OK(whatever): I would rather this did not fail\n'
     'p = "%s/x.yaml"\n' % _CONFIG_ROOT_LITERAL, 1,
     "an invented tag is itself a violation"),
    ("empty_reason.py",
     '# CONFIG-SOURCE-OK(J): ok\n'
     'p = "%s/x.yaml"\n' % _CONFIG_ROOT_LITERAL, 1,
     "a marker with no usable reason does not exempt"),
    ("symbol.py", 'import os\np = os.path.join(DEFAULT_CONFIG_ROOT, "x.yaml")\n', 1,
     "reaching the source root through a symbol is a violation with no literal"),
    ("symbol_def.py", 'DEFAULT_CONFIG_ROOT = "somewhere"\n', 0,
     "the definition site is not a use"),
    ("clean.py", 'p = "/run/xbrain/resolved/p1_motion.yaml"\n', 0,
     "the resolved path is what everything should use"),
]


def self_test():
    """Prove the checker reacts to each case rather than trusting that it does.

    A lint reporting zero on a clean tree is indistinguishable from a lint that
    reports zero on everything. comment_ratio.py in this repo was silently a
    no-op through one whole fix because nothing exercised it; this is the
    cheapest defence against a repeat.
    """
    import tempfile

    ok = True
    with tempfile.TemporaryDirectory() as tmp:
        for name, content, want, why in SELF_TEST_CASES:
            path = os.path.join(tmp, name)
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(content)
            violations, _exempt, _prose = scan_file(path)
            got = len(violations)
            status = "ok " if got == want else "FAIL"
            if got != want:
                ok = False
            print("  %s %-58s want %d got %d" % (status, why, want, got))
    print("")
    print("self-test: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    if "--self-test" in sys.argv:
        return self_test()

    verbose = "-v" in sys.argv or "--verbose" in sys.argv

    print("scan surface: " + ", ".join(SCAN_DIRS))
    # *** Per-directory file counts, printed always and not only on failure.
    #
    # A directory that is absent, or present and empty, contributes zero hits --
    # and zero hits from an unscanned tree is indistinguishable from zero hits
    # from a clean one (CLAUDE.md 3.2 form 6, 扫描面不声明). ros2_ws/ is empty
    # today, so this is not a hypothetical: without these counts the report would
    # read as "three trees checked, all clean" when one of them held no files at
    # all. The count is derived here rather than written down anywhere, per 3.7.
    for top in SCAN_DIRS:
        base = os.path.join(ROOT, top)
        if not os.path.isdir(base):
            print("  NOTE %s/ does not exist; it contributed nothing to this scan"
                  % top)
            continue
        n = sum(1 for _ in _files_under(base))
        if n == 0:
            print("  NOTE %s/ exists but holds no scannable file; it contributed "
                  "nothing to this scan" % top)
        else:
            print("  %-10s %d file(s) scanned" % (top + "/", n))
    print("  rule 1: no unmarked source-root path literal")
    print("  rule 2: no unmarked use of %s" % ", ".join(SOURCE_ROOT_SYMBOLS))
    print("  exemption marker: CONFIG-SOURCE-OK(<tag>): <reason>, tag in %s"
          % sorted(EXEMPT_TAGS))
    for tree, why in sorted(SYMBOL_RULE_EXEMPT_TREES.items()):
        print("  rule 2 not applied under %s/ -- %s" % (tree, why))
    print("")

    violations, exemptions, prose = [], [], []
    for path in sorted(iter_sources()):
        v, e, p = scan_file(path, os.path.relpath(path, ROOT))
        violations.extend(v)
        exemptions.extend(e)
        prose.extend(p)

    for path, lineno, why in violations:
        print("  BAD  %s:%d  %s" % (os.path.relpath(path, ROOT), lineno, why))

    if exemptions:
        print("")
        print("  declared exemptions (each must stay justified):")
        for path, lineno, tag, reason in exemptions:
            print("    %-52s %s: %s"
                  % (os.path.relpath(path, ROOT) + ":" + str(lineno), tag, reason))

    if prose and verbose:
        print("")
        print("  mentions in comments and docstrings (not failed -- prose cannot")
        print("  open a file; listed so they stay visible):")
        for path, lineno, text in prose:
            print("    %s:%d  %s" % (os.path.relpath(path, ROOT), lineno, text))

    print("")
    print("  violations:          %d" % len(violations))
    print("  declared exemptions: %d" % len(exemptions))
    print("  prose mentions:      %d%s"
          % (len(prose), "" if verbose else "   (-v to list)"))
    print("")
    print("criterion: violations == 0")
    print("  This establishes 'no unmarked literal and no unmarked symbol use'.")
    print("  It does NOT establish 'no process reads the configuration source':")
    print("  a path assembled at run time matches neither rule. The run-time half")
    print("  is in xbrain/common/config/resolved.py, which confines every path it")
    print("  opens to the resolved root.")
    return 1 if violations else 0


if __name__ == "__main__":
    sys.exit(main())
