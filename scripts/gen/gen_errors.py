#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: gen_errors.py
Brief: Render the deployed C++17 error-code header from xbrain/common/errors/codes.yaml

Description:
What problem this solves. CFG-CM-3 requires the E_* closed set to reach the C++
consumers -- chassis_relay, quadruped, perception, rtk_driver -- as well as the
Python processes, and requires the two languages to share ONE source of truth so
they cannot drift. codes.yaml is that source (U66: hand-written yaml is the truth,
the C++ header is generated from it -- never the reverse, and never "generated
from the doc tables", which would self-certify, CLAUDE.md 3.2 form 7). This script
is the generator. The header it writes is never a SECOND place the codes are
written down -- a hand-maintained C++ copy is the exact failure this whole package
exists to prevent, one language over: a code spelled differently in two processes
compiles, runs, and only surfaces during integration.

Why this reuses the closed_sets_h.py shape rather than inventing one. The enums
half of this batch already solved the same problem -- deploy a hand-maintained
Python closed set to C++ without a second copy -- in scripts/gen/closed_sets_h.py,
with a template beside the script and a --check/--write pair. Following it means a
reader who has read one has read both, and the drift gate and its mutation are
already understood. What differs is only the table shape: codes carry three
columns (code, retryable, detail), not a bare list of members.

Which decisions this follows:
  * the values are read THROUGH the errors package, not by re-parsing the yaml. A
    second parser is a second thing to keep correct, and the one already there
    RAISES on any row it does not recognise instead of skipping it -- the property
    that matters here, because a loader that skips silently shrinks the closed set,
    and a shrunken closed set makes a legitimate code start raising as unknown.
  * importing the package is itself the first validation step: it parses and
    validates codes.yaml at import and raises on a malformed row, so a broken
    table stops this script before it can write a plausible-looking header.

*** What --check establishes, and what it does not (CLAUDE.md 3.2 form 7). It
compares the file on disk with a fresh render and fails on any difference. That
catches a hand edit to the generated header, which is the realistic defect and the
one that leaves the two languages disagreeing. It does NOT establish that the codes
are correct: comparing a generated artifact against its own generator is true by
construction and cannot fail for a reason anybody cares about. Correctness is the
symmetric-difference metatest in tests/common/test_error_codes.py, which compares
codes.yaml with 11 S13.4~S13.15 -- a different assertion, in a different file,
deliberately.

Why the C++ prose lives in errors_h.tmpl rather than in this file. It reads as C++
while being edited, so a stray non-ASCII character in a comment block is visible
where it lands (the header is held to ASCII by charset_lint). And holding the fixed
header prose as Python string appends would put the explanation of the OUTPUT
inside the source of the GENERATOR, where a reader has to reconstruct it from
concatenation order.

What this script deliberately does NOT do:
  * it does not emit the closed sets domain/plane/... . Those are a different
    table and closed_sets_h.py owns their header; merging the two generators to
    save a file would couple an item written to one that is not.
  * it does not emit the meaning column into the header. meaning is Chinese
    contract prose; emitting it would put non-ASCII text into a source file held
    to ASCII, and no C++ consumer on the estop path reads it (see the header's
    own "does NOT do" note).
  * it does not filter the set down. Every code the package exports is emitted; a
    filter is a place to be wrong that buys nothing, since a code added to the
    package and forgotten here would be missing from the C++ side while both files
    still looked maintained.

Run:
  python3 scripts/gen/gen_errors.py --check    # CI: fail on drift
  python3 scripts/gen/gen_errors.py --write    # regenerate after codes.yaml changes
"""

import os
import re
import sys

# The repository root, derived from this file rather than written out, so a copy
# of the tree renders against its own codes.yaml instead of reaching back to the
# original. The mutation test copies the header, and redirects OUT_PATH, so that
# is not hypothetical.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)   # so the import below finds xbrain/ without installation

# Importing the package is also the first validation step: it parses and validates
# codes.yaml at import and raises on any row it does not recognise, so a malformed
# table stops this script before it can write a plausible-looking header from a
# half-parsed file. Read through the package, never by a second yaml parser.
from xbrain.common import errors  # noqa: E402

#: Where the deployed header lands. common/include/ and NOT common/errors/ -- 99
#: U66 fixes this location to align with the digest precedent
#: (common/include/xbrain/digest/). common/CMakeLists.txt globs common/include for
#: the aggregate that tests/common/link_no_ros/ compiles, so a header placed
#: anywhere else under common/ would sit OUTSIDE the no-ROS gate while looking
#: covered by it -- CLAUDE.md 3.2 form 6, a claim made over a scan surface that
#: excludes the thing being claimed about.
OUT_PATH = os.path.join(ROOT, "common", "include", "xbrain", "errors", "errors.h")

#: The C++ file with one placeholder where the codes go. Beside this script rather
#: than under common/: CLAUDE.md 0.2 reserves common/ for deployment artifacts and
#: takes no source, and a template is source.
TEMPLATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "errors_h.tmpl")

#: The placeholder. Chosen so it cannot occur in legal C++ and cannot occur in a
#: rendered code: if the substitution silently failed, the output would not
#: compile, which is louder than a header that quietly lost its table.
PLACEHOLDER = "@@CODES@@"

#: The exemption marker scripts/lint/no_literal_ecode.py requires on a line that
#: spells an E_* code as a literal. This header is the C++ definition point -- the
#: analogue of the Python package -- so the codes MUST be spelled here; a header
#: cannot parse yaml at compile time. The "export" tag was added to that lint's
#: closed tag set for exactly this file (its own docstring deferred the decision
#: to whoever wrote this generator). The reason is short by design: the tag
#: carries the full justification, and repeating it on forty lines would be noise.
ECODE_MARKER = "// ECODE-OK(export): generated closed-set export"

#: The lint's own code pattern, mirrored so a line is marked exactly when the lint
#: would flag it. Deriving the mark from the same rule -- rather than from
#: "startswith E_" -- means the two cannot drift: if the lint's notion of a code
#: ever changes, this uses that notion. OK carries no E_ prefix and so is left
#: unmarked, which is correct: an unnecessary marker is itself a lint violation.
_HAS_ECODE = re.compile(r"(?<![A-Za-z0-9_])E_[A-Z0-9_]+")


def const_name(code):
    """kPascalCase for a code name, per the Google constant convention.

    Derived rather than tabulated, the same choice closed_sets_h.py makes and for
    the same reason: a hand-kept name table is one more thing to forget when a code
    is added, and the failure would be silent -- the constant would simply not
    exist and only a consumer that wanted it would find out. E_TIMEOUT becomes
    kETimeout, OK becomes kOk.
    """
    # capitalize() and not title(): title() would also lower-case the tail of a
    # part that already carried a capital. Every part here is upper-case, so
    # capitalize turns E_ARB_DISABLED into E, Arb, Disabled -> kEArbDisabled.
    return "k" + "".join(part.capitalize() for part in code.split("_"))


def render_codes():
    """Every code as a named constant, then the whole set as one table.

    Sorted by code so the output is stable across runs and machines, and so the
    Python golden test can reproduce the order with a bare sorted(ALL_CODES): an
    unstable order would make --check and the cross-language comparison fail for a
    reason that is not a defect, and a check that cries wolf gets switched off.
    """
    # sorted() and not the package's iteration order: ALL_CODES is a frozenset, so
    # iterating it directly would give a hash-seeded order that changes run to run
    # under PYTHONHASHSEED. The header would then differ between two --write runs
    # on the same input, and --check would report drift that is not a defect. A
    # plain sort by code is reproducible on any machine, and it is the same rule
    # the Python golden test applies, so the two agree without coordinating.
    codes = sorted(errors.ALL_CODES)
    # An empty set is a hard failure, never an empty header. If the package ever
    # loaded nothing, this would otherwise emit a header with no codes at all and
    # every consumer comparison would answer false -- a closed set that rejects
    # everything, arriving without a word. read through the package means this can
    # only happen if the package itself failed to load, which it would have raised
    # on; the guard is here so a future change to the package cannot make it silent.
    if not codes:
        raise ValueError("errors.ALL_CODES is empty -- codes.yaml did not load")

    # The derived names must be unique, or two codes would define the same C++
    # constant and the second would silently win. Nothing downstream would notice:
    # the header compiles, and a consumer naming the shadowed code gets the other
    # one's value. Checked here so a collision stops the generator with both codes
    # named, rather than becoming a wrong-value bug in the field.
    seen = {}
    for code in codes:
        name = const_name(code)
        if name in seen:
            raise ValueError("codes %r and %r both derive constant %r"
                             % (seen[name], code, name))
        seen[name] = code

    # The output is built as a list of lines and joined once at the end. Two
    # blocks, in this order because the second refers to the first: the named
    # constants, then the table that is built out of those names. Emitting the
    # table first would force it to spell the code strings itself, which is the
    # second copy this whole file exists to avoid.
    block = []

    # Block one: the named constants. A consumer names a code by its constant,
    # never by a string literal (CLAUDE.md 3.5); the constant IS the string, so
    # the two cannot diverge. inline, so each translation unit including this
    # header does not get its own copy at a different address -- the same reason
    # closed_sets_h.py gives, and the property C++17 is the first standard to allow
    # on a variable.
    block.append("// One named constant per code, so a C++ consumer names E_TIMEOUT")
    block.append("// rather than spelling it, exactly as the Python package binds")
    block.append("// the name (CLAUDE.md 3.5). The string IS the wire value.")
    for code in codes:
        # The constant definition. This is the ONE line in the header that spells
        # the code as a literal; everywhere else refers to the name below.
        line = ('inline constexpr std::string_view %s = "%s";'
                % (const_name(code), code))
        # Mark the line only when it actually spells an E_* token, which is what
        # no_literal_ecode.py would flag. Derived from the lint's own pattern
        # rather than "startswith E_" so the two cannot drift; OK carries no E_
        # prefix and is left unmarked, because an unnecessary marker is itself a
        # violation of that lint (its exemption set is closed and audited).
        if _HAS_ECODE.search(code):
            line += "  " + ECODE_MARKER
        block.append(line)
    block.append("")   # a blank line between the two blocks, so the header reads

    # Block two: the table. Each row references the NAMED constant for the code --
    # a bare name, not a literal, so no_literal_ecode has nothing to flag on these
    # lines and the code string is written down exactly once, above. retryable and
    # detail are plain strings: they are not E_* codes, so that lint does not touch
    # them, and they are the contract's own classification vocabulary, carried
    # verbatim so a consumer can branch on the exact text the Python side uses.
    block.append("// The whole closed set as one table, in the same sort-by-code")
    block.append("// order the Python golden test iterates, so the two languages")
    block.append("// print byte-identical output without coordinating.")
    block.append("inline constexpr ErrorCode kAllCodes[] = {")
    for code in codes:
        info = errors.info(code)
        # info() is the single lookup point in the package, and it validated
        # retryable and detail against their small closed vocabularies at import
        # time. So a bad value here is impossible -- it would have stopped the
        # import above -- and these two fields need no re-checking before they are
        # embedded. The row order matches the constants block because both iterate
        # the same sorted list.
        block.append('    {%s, "%s", "%s"},'
                     % (const_name(code), info.retryable, info.detail))
    block.append("};")
    # Joined with newlines and returned as one string; render() drops it into the
    # template's single placeholder. Nothing here writes a file -- that is main()'s
    # job, and keeping render pure means --check can compare against it without a
    # temporary file on disk.
    return "\n".join(block)


def render():
    """The whole header as one string.

    Rendered in full and compared as a whole rather than patched in place: a
    generator that edits its previous output cannot tell a stale region from a
    current one, and the stale region is the part that ships.
    """
    with open(TEMPLATE_PATH, encoding="utf-8") as handle:
        template = handle.read()
    # Assert rather than tolerate. A template that lost its placeholder would
    # render a header with the struct but no codes, and every consumer comparison
    # would then be against an empty set -- a closed set that rejects everything,
    # arriving without a word.
    if PLACEHOLDER not in template:
        raise ValueError("template %s has no %s placeholder"
                         % (TEMPLATE_PATH, PLACEHOLDER))
    return template.replace(PLACEHOLDER, render_codes())


def read_current():
    """The header as it is on disk, or None when it has never been written."""
    # None rather than an empty string: the caller reports "missing" and
    # "different" differently, and an empty string would collapse the two into one
    # message that fits neither.
    if not os.path.isfile(OUT_PATH):
        return None
    with open(OUT_PATH, encoding="utf-8") as handle:
        return handle.read()


def print_criterion():
    """The criterion AND its boundary.

    A tool that prints a verdict without saying what would count as passing leaves
    the reader to invent a criterion, and the invented one is always the one the
    current output already satisfies.
    """
    print("criterion: the committed header equals a fresh render of "
          "xbrain/common/errors/codes.yaml.")
    print("  This catches a hand edit to the generated file, which is the")
    print("  realistic defect. It does NOT establish that the codes are correct")
    print("  -- a generated artifact compared with its own generator cannot")
    print("  disagree. Correctness is the symmetric difference against 11")
    print("  S13.4~S13.15, in tests/common/test_error_codes.py.")


def main():
    """--check compares, --write regenerates. Neither is the default.

    Requiring the mode to be named is deliberate: a bare invocation that defaulted
    to writing would let a CI job "check" the file by rewriting it, which passes
    every time and measures nothing.
    """
    # Rendered before the mode is examined, so a broken template or a malformed
    # codes.yaml fails identically under both modes. A --write that rendered lazily
    # could still overwrite a good header with a partial one.
    want = render()
    relative = os.path.relpath(OUT_PATH, ROOT)   # paths printed relative, for grep
    if "--write" in sys.argv:
        os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
        with open(OUT_PATH, "w", encoding="utf-8") as handle:
            handle.write(want)
        print("wrote %s" % relative)
        return 0
    if "--check" in sys.argv:
        have = read_current()
        # Missing and different are reported separately: they need different work
        # from a reader, and one message covering both would send whoever reads it
        # looking for a diff that does not exist.
        if have is None:
            print("MISSING %s -- run with --write" % relative)
            print_criterion()
            return 1
        if have != want:
            print("DRIFT   %s differs from a fresh render" % relative)
            print_criterion()
            return 1
        print("ok      %s matches xbrain/common/errors/codes.yaml" % relative)
        print_criterion()
        return 0
    print("usage: gen_errors.py --check | --write")
    return 2


if __name__ == "__main__":
    sys.exit(main())
