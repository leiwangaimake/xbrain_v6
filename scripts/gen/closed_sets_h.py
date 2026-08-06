#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: closed_sets_h.py
Brief: Render the deployed C++17 closed-set header from xbrain/common/enums/sets.yaml

Description:
What problem this solves. BIZ-CM-0 requires the closed sets of this batch to
reach chassis_relay as well as the Python processes, and CLAUDE.md 5.3 says why
that consumer decides the shape: it sits on the emergency-stop path under CRL-4
and CRL-5, so nothing it includes may drag in rclcpp or any ROS type. A header
is the cheapest way to keep that true. This script exists so that header is
never a SECOND place the values are written down -- a hand-maintained C++ copy
is the failure this whole package exists to prevent, one language over: a value
spelled differently in two processes compiles, runs, and only surfaces during
integration.

Which decisions this follows:
  * the single source of truth is xbrain/common/enums/sets.yaml, the same file
    the Python package loads. It is hand-maintained and bound to the design
    volumes by the symmetric-difference metatest in
    tests/common/test_closed_sets.py, one case per set. That is the arrangement
    already on disk for the error codes, and this reuses it rather than adding a
    second one for the same question.
  * the values are read through the enums package, not by re-parsing the yaml. A
    second parser is a second thing to keep correct, and the one already there
    RAISES on anything it does not recognise instead of skipping it -- which is
    the property that matters here, because a loader that skips silently shrinks
    a closed set, and a shrunken closed set rejects legitimate traffic.

*** What --check establishes, and what it does not (CLAUDE.md 3.2 form 7). It
compares the file on disk with a fresh render and fails on any difference. That
catches a hand edit to the generated header, which is the realistic defect. It
does NOT establish that the values are correct: comparing a generated artifact
against its own generator is true by construction and cannot fail for a reason
anybody cares about. What establishes correctness is the metatest that compares
sets.yaml with the design volumes -- a different assertion, in a different file,
deliberately.

Why the C++ prose lives in closed_sets_h.tmpl rather than in this file. Two
reasons, and the second is the load-bearing one. It reads as C++ while being
edited, so a stray character in a comment block is visible where it lands. And
holding forty lines of C++ prose as forty Python string appends puts the
explanation of the OUTPUT inside the source of the GENERATOR, where a reader
looking for why a paragraph says what it says has to reconstruct it from
concatenation order.

What this script deliberately does NOT do:
  * it does not emit the E_* codes. Those are a different table, four elements
    per row, and CFG-CM-3 owns their header and its cross-language golden test.
    Merging the two generators to save a file would couple an item that is
    written to one that is not.
  * it does not emit a throwing validator. See the template's own header note:
    the C++ consumer nearest to this runs under CRL-4, which forbids dynamic
    allocation, and throwing allocates.
  * it does not filter the sets down to the six BIZ-CM-0 adds. A filter is a
    place to be wrong that buys nothing: a set added to sets.yaml and forgotten
    here would be missing from the C++ side while both files still looked
    maintained.

Run:
  python3 scripts/gen/closed_sets_h.py --check    # CI: fail on drift
  python3 scripts/gen/closed_sets_h.py --write    # regenerate after a set changes
"""

import os
import sys

# The repository root, derived from this file rather than written out, so a copy
# of the tree renders against its own sets.yaml instead of reaching back to the
# original. The mutation tests copy the tree, so that is not hypothetical.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)   # so the import below finds xbrain/ without installation

# Importing the package is also the first validation step: _load() runs at import
# and raises on any line of sets.yaml it does not recognise, so a malformed table
# stops this script before it can write a plausible-looking header from a
# half-parsed file.
from xbrain.common import enums  # noqa: E402

#: Where the deployed header lands. common/include/ and NOT common/errors/:
#: common/CMakeLists.txt globs common/include for the aggregate that
#: tests/common/link_no_ros/ compiles, so a header placed anywhere else under
#: common/ would sit outside the no-ROS gate while looking covered by it. That is
#: CLAUDE.md 3.2 form 6 -- a claim made over a scan surface that excludes the
#: thing being claimed about.
OUT_PATH = os.path.join(ROOT, "common", "include", "xbrain", "enums",
                        "closed_sets.h")

#: The C++ file with one placeholder where the arrays go. Beside this script
#: rather than under common/: CLAUDE.md 0.2 reserves common/ for deployment
#: artifacts and takes no source, and a template is source.
TEMPLATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "closed_sets_h.tmpl")

#: The placeholder. Chosen so it cannot occur in legal C++ and cannot occur in a
#: rendered set: if the substitution silently failed, the output would not
#: compile, which is louder than a header that quietly lost its arrays.
PLACEHOLDER = "@@SETS@@"


def array_name(set_name):
    """kPascalCase for a snake_case set name, per the Google constant convention.

    Derived rather than tabulated. A hand-kept name table is one more thing to
    forget when a set is added, and the failure would be silent on this side:
    the array would simply not exist and only a consumer that wanted it would
    ever find out.
    """
    # capitalize() and not title(): title() would also lower-case the tail of a
    # part that already carried a capital, which matters the day a set name is
    # not pure lower-case snake.
    return "k" + "".join(part.capitalize() for part in set_name.split("_"))


def render_sets():
    """Every set as a C++ array declaration, in one block.

    Sorted by set name so the output is stable across runs and machines: an
    unstable order would make --check fail for a reason that is not a defect,
    and a check that cries wolf gets switched off.
    """
    block = []
    for name in sorted(enums.SET_NAMES):
        closed = enums.get(name)
        # The citation travels with the values, exactly as on the Python side.
        # When a value fails at run time the first question is what the contract
        # says, and the volumes run to tens of thousands of lines; carrying the
        # section and a grep-able anchor turns that into one search. The anchor
        # is a literal string and never a line number (CLAUDE.md 1.1 NUM-4).
        block.append("// " + name + " -- " + closed.source + "   anchor: "
                     + closed.anchor)
        # inline, and that is not decoration: without it every translation unit
        # including this header would get its own copy of every array, and the
        # addresses would differ between them. C++17 is the first standard that
        # allows it on a variable, which ties this output to CPP-1's exact-C++17
        # requirement rather than merely being compatible with it.
        block.append("inline constexpr std::string_view " + array_name(name)
                     + "[] = {")
        # Contract order, not sorted. Sorting gate_limiter would put brake ahead
        # of estop, and the resulting array holds exactly the same strings as the
        # correct one -- so the defect survives any comparison that checks
        # membership alone.
        for value in closed:
            # No escaping step, and none is needed: every member of every set is
            # a lower-case identifier, which the metatest enforces by the shape
            # of the pattern it extracts with. If a contract value ever carries a
            # quote or a backslash, this line is where it has to be handled --
            # today it would emit C++ that does not compile, which is the right
            # failure and better than emitting a silently wrong string.
            block.append('    "' + value + '",')
        block.append("};")
        block.append("")   # blank line between sets, so the output reads as C++
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
    # render a header with no arrays at all, and every Contains() call against it
    # would answer false -- a closed set that rejects everything, arriving
    # without a word.
    if PLACEHOLDER not in template:
        raise ValueError("template %s has no %s placeholder"
                         % (TEMPLATE_PATH, PLACEHOLDER))
    # English message, per CLAUDE.md 2.1, and it names both paths: this runs on a
    # developer machine where the template may have been edited a moment ago.
    return template.replace(PLACEHOLDER, render_sets())


def read_current():
    """The header as it is on disk, or None when it has never been written."""
    # None rather than an empty string: the caller reports "missing" and
    # "different" differently, and an empty string would collapse the two into
    # one message that fits neither.
    if not os.path.isfile(OUT_PATH):
        return None
    with open(OUT_PATH, encoding="utf-8") as handle:
        return handle.read()


def print_criterion():
    """The criterion AND its boundary.

    A tool that prints a verdict without saying what would count as passing
    leaves the reader to invent a criterion, and the invented one is always the
    one the current output already satisfies.
    """
    print("criterion: the committed header equals a fresh render of "
          "xbrain/common/enums/sets.yaml.")
    print("  This catches a hand edit to the generated file, which is the")
    print("  realistic defect. It does NOT establish that the values are")
    print("  correct -- a generated artifact compared with its own generator")
    print("  cannot disagree. Correctness is the symmetric difference against")
    print("  the design volumes, in tests/common/test_closed_sets.py.")


def main():
    """--check compares, --write regenerates. Neither is the default.

    Requiring the mode to be named is deliberate: a bare invocation that
    defaulted to writing would let a CI job "check" the file by rewriting it,
    which passes every time and measures nothing.
    """
    # Rendered before the mode is examined, so a broken template or a malformed
    # sets.yaml fails identically under both modes. A --write that rendered
    # lazily could still overwrite a good header with a partial one.
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
        # Missing and different are reported separately: they need different
        # work from a reader, and one message covering both would send whoever
        # reads it looking for a diff that does not exist.
        if have is None:
            print("MISSING %s -- run with --write" % relative)
            print_criterion()
            return 1
        if have != want:
            print("DRIFT   %s differs from a fresh render" % relative)
            print_criterion()
            return 1
        print("ok      %s matches xbrain/common/enums/sets.yaml" % relative)
        print_criterion()
        return 0
    print("usage: closed_sets_h.py --check | --write")
    return 2


if __name__ == "__main__":
    sys.exit(main())
