#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: no_literal_ecode.py
Brief: No E_* spelled as a string literal anywhere outside the shared error library

Description:
What problem this solves. CLAUDE.md 3.5 requires every E_* constant to come from
the shared library and forbids hardcoding the string. The failure it prevents is
specific and this project has already named it: a code spelled slightly
differently in two processes compiles, runs, and only shows up during
integration, when the cloud client branches on the code and takes the wrong arm.
Nothing inside one module can establish that property -- it is a whole-tree one
-- so it needs something that looks at the whole tree, which is this.

The two halves of the rule, with the anchors they were actually read from, since
this file exists to enforce them:
  * the set is closed and nobody may invent a member. 11 S13.1, constraint EC-2,
    grep 不得临时造码 -- an ad hoc code cannot be counted by the cloud or
    translated by the HMI.
  * the members are EXPORTED by common/, never spelled as literals. 19 PRC-2,
    grep 由 `common/` 导出, with its mutation PMT-2 next door.
CLAUDE.md's own trap table cites 11 S13.8 for the second half; S13.8 was checked
and it is the group-E code table (E_RESOURCE_* / E_ARB_*), so that pointer does
not lead anywhere and is not repeated here.

THE RULE. On a line that is not a comment, no string literal may contain a token
matching E_ followed by upper-case letters, digits or underscores. Two shapes are
distinguished in the report because they need different fixes, and both fail:

  whole   the literal IS a code, as in _CODE_BUSY = "E_BUSY". This is the wire
          form: the value a peer receives and branches on. It is the shape the
          CFG-CM-2 criterion writes verbatim.
  inside  the code sits within a longer literal, as in "E_CONFIG_INVALID:
          reference cycle". Not a wire value, but still a second copy of the
          spelling, and a second copy is what drifts. Interpolating the exported
          constant produces byte-identical output, so there is no cost to
          holding this to the same standard.

HOW "comes from the shared library" IS DECIDED. A use that came from the export
is a bare NAME -- errors.E_TIMEOUT, or E_TIMEOUT after `from xbrain.common.errors
import E_TIMEOUT` -- and a bare name is not a string literal, so it never matches.
That is the whole mechanism. Anything still spelled in quotes is by construction
a second source of truth for that spelling.

*** WHAT THIS CANNOT ESTABLISH, stated so a green run is not read as more than it
is (CLAUDE.md 3.2 form 7 -- a rule that defines its conclusion into its premise
is unfalsifiable):

  * a code assembled at run time -- "E_" + name, or a value arriving from a
    message or a yaml file -- matches nothing here. The criterion is therefore
    "no unmarked code spelled in a literal", NOT "no process invents a code".
  * a bare name is trusted without checking where it was bound. A module could
    define its own E_BUSY from a computed value and this scan would not see it.
    The realistic form of that mistake, E_BUSY = "E_BUSY", IS caught, because the
    right-hand side is a literal.
  * the success code OK is out of scope. It carries no E_ prefix, and matching a
    bare two-letter token would flag every English "OK" in the tree -- a rule
    that fires everywhere gets switched off, so it is left out deliberately
    rather than left out by accident.
  * only lines are examined, so a code split across two lines of one implicitly
    concatenated string is invisible. Nothing in the tree is written that way.
  * data files are not scanned. xbrain/common/errors/codes.yaml is the single
    source of truth and must contain every code; scanning it would make the
    criterion unreachable, which is how a criterion ends up loosened.

Why this file lives in scripts/ while the scan surface excludes scripts/. This
script necessarily contains the pattern it hunts for, in its own self-test cases
and in this prose. CLAUDE.md 3.2 form 3 is 判据自伤: a criterion whose own text is
inside the surface it scans can never reach zero, and the repair anyone reaches
for is to loosen it. Keeping the lint outside its own surface is the structural
answer, and tests/common/test_no_literal_ecode.py asserts that property instead
of leaving it to whoever next edits SCAN_DIRS. tests/ is excluded for the mirror
reason: pinning errors.E_TIMEOUT == "E_TIMEOUT" is what binds the constant to the
wire value, and it cannot be written without a literal.

Run:
  python3 scripts/lint/no_literal_ecode.py
  python3 scripts/lint/no_literal_ecode.py --self-test
"""

import io
import os
import re
import sys
import tokenize

ROOT = "/opt/xbrain_v6"

#: The scan surface. The CFG-CM-2 criterion says "any .py/.cc/.h", so the surface
#: is every tree that holds shipped source: the Python runtime, the ROS 2 nodes,
#: the AI services, and the deployed C++ headers under common/. common/ is in the
#: list because that is where the generated errors.h will land (CFG-CM-3) and
#: because a rule that covers .h files while scanning no tree holding one would
#: be a rule in name only.
SCAN_DIRS = ("xbrain", "ros2_ws", "services", "common")

#: .cpp and .hpp are here alongside the three the criterion names: they are the
#: same language, and a rule that stops at the file extension is a rule someone
#: evades by renaming a file.
SOURCE_EXT = (".py", ".cc", ".cpp", ".h", ".hpp")

#: A code token. The leading boundary is NOT optional and is the single most
#: load-bearing character in this file: without it, E_[A-Z0-9_]+ matches inside
#: ordinary identifiers -- SAMPLE_RATE contains E_RATE, MODE_NAME contains
#: E_NAME. Measured on this tree before the boundary was added, the pattern
#: reported dozens of hits in services/ that were all constants like
#: ASR_REQUIRE_SAMPLE_RATE. A checker whose findings are mostly noise is a
#: checker that gets switched off.
#:
#: Written as a negative lookbehind rather than \b because \b would also accept a
#: preceding underscore: _E_BUSY is one identifier, not a code.
CODE_RE = re.compile(r"(?<![A-Za-z0-9_])(E_[A-Z0-9_]+)")

#: One single-quoted or double-quoted run on a line, with backslash escapes
#: honoured so a quote inside a literal does not end it early. Deliberately does
#: not model triple quotes: a docstring is classified as a comment before this
#: ever runs, and a non-docstring triple-quoted block is examined line by line,
#: which is stated as a limit above rather than pretended away.
QUOTED_RE = re.compile(r"""(['"])((?:\\.|(?!\1).)*?)\1""")

#: Interpolation spans inside an f-string. Removed from the literal body before
#: the code pattern is applied, because f"...{E_CONFIG_INVALID}..." is the
#: CORRECT form -- the name there is the imported constant, not a spelling. Left
#: in, it would fail exactly the fix this lint asks for, which is the shape of a
#: rule that can never be satisfied.
#:
#: *** Applied ONLY to literals that really carry an f prefix, and that
#: restriction was earned. Stripping braces from every literal blanks out
#: '{"code": "E_BUSY"}' -- a hand-written JSON body, which is the most direct way
#: there is to put a code on the wire. It was found by mutating the docstring
#: classification and watching the docstring case stay green: the case had been
#: passing because of this strip, not because of the classification it claimed to
#: test. CLAUDE.md 3.2 form 1, caught in this file's own probes.
FSTRING_SPAN_RE = re.compile(r"\{[^{}]*\}")

#: String prefix letters Python allows in front of a quote.
STRING_PREFIX_CHARS = "fFrRbBuU"

#: The permitted exemption reasons, each with what it licenses. A closed set: a
#: marker naming anything else is itself a violation, because an open tag set
#: degrades the mechanism into "write any word and the check goes quiet".
#:
#: The "export" tag was earned by CFG-CM-3, exactly as the earlier note here said
#: it would be: the file now exists, so the decision was made with it in front of
#: us rather than reserved in advance (CLAUDE.md 9.3). scripts/gen/gen_errors.py
#: generates common/include/xbrain/errors/errors.h, and a C++ header cannot parse
#: the yaml at compile time -- so the codes MUST be spelled there. That header is
#: the C++ definition point, the analogue of the Python package (which avoids
#: literals only because codes.yaml is a data file this scan does not read). The
#: generator emits the marker on every E_* line; OK carries no E_ prefix and is
#: left unmarked, because an unnecessary marker is itself a violation.
EXEMPT_TAGS = {
    "cycle": "xbrain/common/errors/exceptions.py defines the exception types "
             "that the package __init__ imports while it is still executing, so "
             "importing the generated code names back into it would be an import "
             "cycle. Its own docstring records the same reasoning",
    "as12": "11 S11A.8.2 AS-12 fixes the AI-service refusal body as 429 plus "
            "{code: E_BUSY}; the service cannot obtain the name from the runtime "
            "package because mapping onto the closed set is the gateway's job "
            "(11 S8.13.5, 错误映射(网关唯一实现点)) and services/ does not depend "
            "on xbrain/",
    "export": "common/include/xbrain/errors/errors.h is the generated C++ "
              "deployment export of the E_* closed set (CFG-CM-3), rendered from "
              "xbrain/common/errors/codes.yaml by scripts/gen/gen_errors.py. It "
              "is the C++ definition point, the analogue of the Python package, "
              "not a second copy: a header cannot parse the yaml at compile time, "
              "so the codes must be spelled here. tests/common/"
              "test_cross_lang_codes.py keeps it byte-equal to a fresh render, "
              "and the codes are held correct against 11 S13.4~S13.15 by "
              "tests/common/test_error_codes.py",
}

#: The marker itself. Shouty and greppable on purpose: an exemption a reviewer
#: can miss is not a visible one. Accepted on the offending line or in the
#: comment block immediately above it, so a real justification has room to be a
#: paragraph instead of being crushed onto one line.
MARKER_RE = re.compile(r"ECODE-OK\(([A-Za-z0-9_-]+)\)\s*:\s*(\S.*)")

#: A reason shorter than this is not a reason. Same threshold as
#: no_config_source_read.py, and for the same reason: "ok" must not silence a
#: check that a paragraph is supposed to justify.
MIN_REASON_CHARS = 12

#: Directories never worth scanning.
SKIP_DIRS = {"__pycache__", ".git", "node_modules", "build", "install", "log"}


def _files_under(base):
    """Scannable files below one directory.

    Shared by iter_sources and by the per-directory counts main() prints, so the
    number reported and the files actually read cannot drift apart. A count
    produced by a second, slightly different walk is a number that reassures
    without measuring anything.
    """
    for dirpath, dirnames, filenames in os.walk(base):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for name in filenames:
            if name.endswith(SOURCE_EXT):
                yield os.path.join(dirpath, name)


def iter_sources():
    """Every file in the scan surface.

    An absent tree is skipped here and REPORTED by main(). Zero hits from a tree
    that was never walked is indistinguishable from zero hits from a clean one,
    and ros2_ws/ holds no source today, so this is not hypothetical.
    """
    for top in SCAN_DIRS:
        base = os.path.join(ROOT, top)
        if not os.path.isdir(base):
            continue
        for path in _files_under(base):
            yield path


def python_pieces(source):
    """(code_strings, prose) for Python source, or None if it will not tokenize.

    Each element is (lineno, text). code_strings holds every string literal that
    is NOT a docstring -- the only place a code can be spelled and then travel.
    prose holds comments and docstrings, which cannot reach a peer.

    *** WHY THE TOKENIZER AND NOT LINE CLASSIFICATION. The first version of this
    lint asked only "is this line a comment line" and skipped the whole line if
    so. That is wrong for the single most important shape it has to judge:

        code = "E_SCHEMA"  # a trailing comment

    holds a comment AND a literal, so the line was written off as prose and the
    violation vanished. It was caught here because the marked exemption in
    xbrain/common/errors/exceptions.py stopped being REPORTED as an exemption --
    the run went green for the wrong reason, which is CLAUDE.md 3.2 form 1
    exactly: an implementation that does nothing passes. Working on tokens rather
    than lines removes the ambiguity instead of narrowing it.

    The docstring test -- a string token is a docstring when the token before it
    opens a statement -- is the rule charset_fix.classify_lines uses, and the NL
    caveat is copied with it: NL is a NON-logical newline appearing inside
    brackets, so it must not update prev, or the second line of a multi-line
    call argument would be classified as a docstring.
    """
    try:
        toks = list(tokenize.generate_tokens(io.StringIO(source).readline))
    except (tokenize.TokenError, IndentationError, SyntaxError, ValueError):
        # Deliberately None rather than an empty pair: the caller falls back to
        # the coarse line scan, which over-reports. Returning empty lists would
        # make an untokenizable file look clean, and a file nobody could parse is
        # the last one that should pass silently.
        return None
    code_strings, prose = [], []
    prev = None
    for ttype, text, (srow, _sc), (_erow, _ec), _ln in toks:
        if ttype == tokenize.COMMENT:
            prose.append((srow, text))
        elif ttype == tokenize.STRING:
            if prev in (None, tokenize.INDENT, tokenize.NEWLINE, tokenize.DEDENT):
                prose.append((srow, text))
            else:
                code_strings.append((srow, text))
            prev = ttype
        elif ttype in (tokenize.NL, tokenize.NEWLINE, tokenize.INDENT,
                       tokenize.DEDENT, tokenize.ENDMARKER, tokenize.ENCODING):
            if ttype != tokenize.NL:
                prev = ttype
        else:
            prev = ttype
    return code_strings, prose


def line_pieces(lines):
    """The same pair, derived from lines, for C and C++ and for unparsable Python.

    A whole-line comment is prose; everything else is offered to QUOTED_RE. A
    trailing // comment after code therefore stays in the code half, so a literal
    on that line is still judged -- the mistake described in python_pieces is not
    repeated here. The cost is that a code named inside a trailing comment can be
    reported; that errs towards reporting, which is the safe direction, and it is
    why the Python half does not settle for this.
    """
    code_strings, prose = [], []
    for i, line in enumerate(lines, 1):
        if line.lstrip().startswith(("//", "*", "/*", "#")):
            prose.append((i, line))
        else:
            code_strings.append((i, line))
    return code_strings, prose


def marker_for(lines, idx):
    """(tag, reason) from a marker on this line or the comment block above it.

    Same-line first, then upward through the CONTIGUOUS run of comment lines
    directly above, stopping at the first line that is not a comment. Not just
    idx-1, because an exemption worth granting needs a paragraph to justify
    itself and an explanation nobody reads is the same as a silent skip. Not
    "anywhere in the file" either, so a marker written for one occurrence cannot
    quietly cover a second one added later.
    """
    if idx < len(lines):
        m = MARKER_RE.search(lines[idx])
        if m:
            return m.group(1), m.group(2).strip()
    probe = idx - 1
    while probe >= 0:
        stripped = lines[probe].strip()
        if False:
            break
        m = MARKER_RE.search(lines[probe])
        if m:
            return m.group(1), m.group(2).strip()
        probe -= 1
    return None


def codes_in(text):
    """[(code, shape)] for every code spelled inside a string literal in `text`.

    `text` is one string token when the tokenizer was usable, and one whole line
    otherwise. Either way QUOTED_RE isolates the quoted runs, so a token is
    handled by the same code path as a line and there is only one place where the
    decision "is this a code" is made.

    shape is "whole" when the literal is exactly the code -- the wire form, the
    one a peer receives and branches on -- and "inside" when it sits in a longer
    string. Both are findings; the distinction only makes the printed advice
    match the fix the reader has to make.
    """
    found = []
    # finditer, not search: one line can hold several literals, and the one that
    # matters is rarely the first. `content={"code": _CODE_BUSY, "message": ...}`
    # would be judged on its first literal alone if this stopped at one match.
    for match in QUOTED_RE.finditer(text):
        # group(2) is the body between the quotes; group(1) is the quote
        # character itself, used as the backreference that closes the run. The
        # body is what travels, so the body is what gets searched.
        body = match.group(2)
        # Interpolation is stripped only from an actual f-string: the names
        # inside its braces are the imported constants, which is the correct
        # form. Every other literal keeps its braces, so a hand-written JSON body
        # is still judged on what it contains.
        probe = FSTRING_SPAN_RE.sub("", body) if _is_fstring(text, match.start()) else body
        for code in CODE_RE.findall(probe):
            # The shape is decided on the STRIPPED body, so f"{x}E_BUSY" counts
            # as whole. That is the right answer: what the peer receives is the
            # code and whatever the interpolation produced, and calling it
            # "inside" would understate it.
            found.append((code, "whole" if probe.strip() == code else "inside"))
    return found


def _is_fstring(text, quote_at):
    """Whether the literal opening at `quote_at` carries an f prefix.

    Walks back over the prefix letters and then requires a non-identifier
    character before them. Without that last condition conf["E_X"] reads as an
    f-string -- the f of "conf" sits right against the quote -- and its braces
    would be stripped, which is a hole opened by a variable name.
    """
    k = quote_at
    while k > 0 and text[k - 1] in STRING_PREFIX_CHARS:
        k -= 1
    if k == quote_at:
        return False
    if k > 0 and (text[k - 1].isalnum() or text[k - 1] == "_"):
        return False
    return "f" in text[k:quote_at].lower()


def scan_file(path, rel_path=None):
    """(violations, exemptions, prose) for one file.

    Three buckets rather than two, on the header_lint.py precedent: they need
    different work from a reader, and one number would hide the mix.
    """
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            source = fh.read()
    except OSError as exc:
        return [(path, 0, "cannot read: %s" % exc)], [], []
    lines = source.split("\n")

    # Python gets the tokenizer; everything else, and any Python file that will
    # not parse, gets the coarse line split. The fallback is deliberate and its
    # direction matters: it over-reports rather than under-reports, so a file
    # nobody can parse cannot slip through as clean.
    pieces = python_pieces(source) if path.endswith(".py") else None
    if pieces is None:
        pieces = line_pieces(lines)
    code_strings, prose_pieces = pieces

    # rel_path is accepted for symmetry with no_config_source_read.py and for the
    # report; nothing here is decided by location, because this rule has no
    # per-tree exemption. If one is ever added, it belongs beside EXEMPT_TAGS as
    # declared data, never as an `if` buried in this scan.
    _ = rel_path
    violations, exemptions, prose = [], [], []

    # A comment or docstring mentioning a code is prose. It cannot reach a peer,
    # and demanding a marker on explanatory text pushes people to delete the
    # explanation -- the files in xbrain/common/errors/ carry dozens of such
    # mentions, every one of them the reason a decision was made.
    for lineno, text in prose_pieces:
        for code, _shape in codes_in(text):
            prose.append((path, lineno, code))

    for lineno, text in code_strings:
        for code, shape in codes_in(text):
            # lineno is where the TOKEN starts, which for a multi-line string is
            # its opening quote. That is also where a marker has to sit, so the
            # two agree by construction rather than by luck.
            marker = marker_for(lines, lineno - 1)
            if marker:
                # A marker is not a pass by itself. Both ways it can be wrong --
                # an invented tag and a reason too short to argue with -- are
                # violations in their own right, and they are reported as
                # exemption failures rather than as literal failures so the
                # reader is sent to the right fix.
                tag, reason = marker
                if tag not in EXEMPT_TAGS:
                    violations.append((path, lineno,
                                       "exemption tag %r is not one of %s"
                                       % (tag, sorted(EXEMPT_TAGS))))
                elif len(reason) < MIN_REASON_CHARS:
                    violations.append((path, lineno,
                                       "exemption gives no usable reason: %r" % reason))
                else:
                    exemptions.append((path, lineno, tag, reason))
                continue
            violations.append(
                (path, lineno,
                 "%s spelled as a string literal (%s); import it from "
                 "xbrain/common/errors instead" % (code, shape)))
    # Sorted so two runs over an unchanged tree print identical output. An
    # unstable order makes a diff of two reports look like a change in findings,
    # and a report nobody can diff is a report nobody rereads.
    return sorted(violations), sorted(exemptions), sorted(prose)


#: One probe per way this can be right or wrong. A lint reporting zero on a clean
#: tree is indistinguishable from a lint that reports zero on everything, and
#: comment_ratio.py in this repo was silently a no-op through an entire fix
#: because nothing exercised it.
#:
#: The first case is the mutation CFG-CM-2 names. The literal it writes comes
#: from 19 PMT-2 word for word; the landing point does not -- PMT-2 says
#: perception source and the CFG-CM-2 row says services/asr/, which is where a
#: real source file exists to put it in today.
SELF_TEST_CASES = [
    ("mutation.py", 'def handle():\n    code = "E_SCHEMA"\n    return code\n', 1,
     "the mutation the item names: a bare code literal is a violation"),
    ("imported.py",
     'from xbrain.common.errors import E_SCHEMA\n'
     'def handle():\n    return E_SCHEMA\n', 0,
     "the imported constant is a name, not a literal, and must pass"),
    ("marked.py",
     '# ECODE-OK(cycle): the defining module cannot import its own exports\n'
     'code = "E_SCHEMA"\n', 0,
     "a marker on the preceding line exempts it"),
    ("marked_same_line.py",
     'code = "E_BUSY"  # ECODE-OK(as12): AS-12 fixes this response body shape\n', 0,
     "a marker on the same line exempts it"),
    ("bad_tag.py",
     '# ECODE-OK(whatever): I would rather this did not fail\n'
     'code = "E_SCHEMA"\n', 1,
     "an invented tag is itself a violation"),
    ("empty_reason.py", '# ECODE-OK(cycle): ok\ncode = "E_SCHEMA"\n', 1,
     "a marker with no usable reason does not exempt"),
    ("embedded.py", 'msg = "E_CONFIG_INVALID: reference cycle"\n', 1,
     "a code inside a longer message is a second copy of the spelling"),
    ("fstring.py", 'msg = f"{E_CONFIG_INVALID}: reference cycle"\n', 0,
     "an interpolated constant is the fix, not the defect"),
    # The counterpart, and the reason the f prefix is checked instead of every
    # brace span being stripped: this is a wire body written by hand.
    ("json_body.py", 'body = \'{"code": "E_BUSY"}\'\n', 1,
     "braces in a plain literal are not interpolation, they are JSON"),
    ("prefixed_name.py", 'x = conf["E_BUSY"]\n', 1,
     "a name ending in f before the quote must not read as an f prefix"),
    ("comment.py", '# returns E_BUSY when the decoder is busy\nx = 1\n', 0,
     "a comment naming a code is prose and cannot reach a peer"),
    # *** The regression case. The first version of this lint skipped any line
    # holding a comment, so this shape reported nothing at all and the whole run
    # went green for a reason unrelated to the tree being clean.
    ("trailing.py", 'code = "E_SCHEMA"  # a note, not an exemption marker\n', 1,
     "a trailing comment must not turn the rest of the line into prose"),
    ("docstring.py", '"""Refusal body is {"code": "E_BUSY"} per AS-12."""\nx = 1\n', 0,
     "a docstring quoting a code is documentation, not a wire value"),
    ("comment_quotes.py", 'x = 1  # the {"code": "E_BUSY"} body\n', 0,
     "quotes inside a comment do not make it a literal"),
    ("unparsable.py", 'def broken(:\n    code = "E_SCHEMA"\n', 1,
     "a file that will not tokenize falls back to the coarse scan, not to clean"),
    ("single_quotes.py", "code = 'E_TIMEOUT'\n", 1,
     "quote style does not change what reaches the wire"),
    ("identifier.py", 'RATE = os.environ["ASR_REQUIRE_SAMPLE_RATE"]\n', 0,
     "SAMPLE_RATE contains E_RATE: the leading boundary must reject it"),
    ("cpp.cc", 'const char* k = "E_BUSY";\n', 1,
     "the C++ half of the surface is really scanned"),
    ("cpp_comment.h", '// E_BUSY is returned by the gateway\n', 0,
     "a C++ comment is prose too"),
    # The generated errors.h shape: an E_* named constant carrying the export
    # marker on the same line. This is the line gen_errors.py emits, and it must
    # be exempt -- the header is the C++ definition point, not a second copy.
    ("errors_export.h",
     'inline constexpr std::string_view kEBusy = "E_BUSY";  '
     '// ECODE-OK(export): generated closed-set export\n', 0,
     "the generated C++ error header export is exempt via the export tag"),
    # And the guard on it: the same line WITHOUT the marker is a violation, so the
    # exemption is doing the work rather than the code being invisible for some
    # unrelated reason.
    ("errors_unmarked.h",
     'inline constexpr std::string_view kEBusy = "E_BUSY";\n', 1,
     "an unmarked E_* literal in a common/ header is still a violation"),
]


def self_test():
    """Prove the checker reacts to each case instead of trusting that it does.

    Every probe is written into a temporary directory, never into the tree: a
    crash between writing and deleting would otherwise leave a file spelling a
    code by hand inside services/ or xbrain/, which is both the thing this lint
    forbids and a file someone could later mistake for real code.

    Note the docstring probe is only meaningful because brace spans are no longer
    stripped from ordinary literals. While they were, that probe passed without
    the docstring classification being involved at all.
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
            if got != want:
                ok = False
            print("  %s %-62s want %d got %d"
                  % ("ok " if got == want else "FAIL", why, want, got))
    print("")
    print("self-test: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def _print_surface():
    """The scan surface, with a file count per tree.

    Printed always and not only on failure. A tree that is absent, or present and
    empty, contributes zero hits, and zero hits from an unscanned tree reads the
    same as zero from a clean one -- CLAUDE.md 3.2 form 6, 扫描面不声明. ros2_ws/
    holds no source today, so without these counts the report would say four
    trees checked and all clean when one of them held nothing at all.
    """
    print("scan surface: " + ", ".join(SCAN_DIRS)
          + "   extensions: " + ", ".join(SOURCE_EXT))
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


def _print_criterion():
    """The criterion and, just as important, its boundary.

    A lint that prints numbers without saying what would count as passing leaves
    the reader to invent a criterion, and the invented one is always the one the
    current output already satisfies.
    """
    print("criterion: violations == 0")
    print("  A use that came from the shared library is a bare NAME, so it never")
    print("  matches; anything still in quotes is a second copy of the spelling.")
    print("  This does NOT establish 'no process invents a code': a value")
    print("  assembled at run time, or read from a message, matches nothing here.")
    print("  The success code OK is out of scope -- it has no E_ prefix and")
    print("  matching a bare two-letter token would fire on English prose.")


def main():
    if "--self-test" in sys.argv:
        return self_test()
    verbose = "-v" in sys.argv or "--verbose" in sys.argv

    _print_surface()
    print("  rule: no E_* token inside a string literal; comments and docstrings")
    print("        are prose and are counted separately, never failed")
    print("  exemption marker: ECODE-OK(<tag>): <reason>, tag in %s"
          % sorted(EXEMPT_TAGS))
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
            print("    %-46s %s: %s"
                  % (os.path.relpath(path, ROOT) + ":" + str(lineno), tag,
                     reason[:70]))

    if prose and verbose:
        print("")
        print("  mentions in comments (not failed -- a comment cannot reach a")
        print("  peer; listed so they stay visible):")
        for path, lineno, code in prose:
            print("    %s:%d  %s" % (os.path.relpath(path, ROOT), lineno, code))

    print("")
    print("  violations:          %d" % len(violations))
    print("  declared exemptions: %d" % len(exemptions))
    print("  comment mentions:    %d%s"
          % (len(prose), "" if verbose else "   (-v to list)"))
    print("")
    _print_criterion()
    return 1 if violations else 0


if __name__ == "__main__":
    sys.exit(main())
