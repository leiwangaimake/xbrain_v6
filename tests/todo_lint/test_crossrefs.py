"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_crossrefs.py
Brief: Every TODO task id cited by another document must exist in the TODO

Description:
docs/ cross-references task ids by hand -- NEXT.md alone cites dozens, and 99
and PHASE0_待裁决清单 cite more -- and until 2026-10-01 nothing checked that
the cited id was real.

Why that matters more than it sounds. A citation to an id that was renamed,
or never existed, reads EXACTLY like a working one. The reader greps, finds
nothing, and cannot tell which of three things happened: the item moved, the
item was deleted, or whoever wrote the sentence mistyped. The first two have
an answer somewhere in the repo; the third does not. Without a check, all
three present as the same dead end, and the usual outcome is that the reader
assumes the first and goes looking for a renamed item that is not there.

What this file does NOT establish, stated because a green run should never be
read as more than it is (CLAUDE.md 3.2 form 7):

  * It checks that the cited id EXISTS. It does not check that the sentence
    doing the citing says anything true about it. A paragraph claiming
    BIZ-P3-07 is finished, when the TODO has it open, passes untouched.
  * It reads markdown prose, so an id constructed at render time, or split
    across a line break, is invisible to it.

The TODO's own internal shape -- four-segment ids, duplicate ids, dependency
targets that exist, two ids claiming one file -- belongs to test_todo_lint.py.
This file only looks the other way: inward, from the documents that cite.

Design notes, each of which cost something to learn:

  * The prefix set is DERIVED from the TODO's own ids rather than written
    down here. docs/ carries id families that are NOT TODO items -- 10
    S5.4.0's CFG-ROOT-1..6 is one of them -- and the first version of this
    check used a hand-written regex that swept them in and reported six
    violations against items that were never TODO items at all. A hand list
    fails in both directions: it drags in foreign families today and misses
    a new TODO family tomorrow. Derivation covers a new family on the day it
    appears, with no edit here.

  * Ids are normalised through progress.py's norm_id before comparison. The
    TODO itself writes both BIZ-P3-7 and BIZ-P3-07 -- that function's own
    docstring is where this is recorded -- while NEXT.md consistently writes
    the zero-padded form. Comparing raw strings would report every padded
    citation as dangling: noise, not a finding. The cosmetic half of that
    split (a reader greping the padded form finds nothing in the TODO) is
    real, and is deliberately NOT what this file is for; mechanically
    rewriting several hundred ids to settle it is a change whose risk does
    not pay for its benefit.

  * parse() and norm_id() are imported from progress.py rather than
    reimplemented. A second parser of the same file is a second thing to
    keep correct, and the one that rots is always the one nobody runs.

  * Only docs/*.md is scanned, not code comments. Code that names a task id
    is making a different claim ("this function is what that item asked
    for"), and the thing worth checking there is whether the code does what
    the item says -- which no scan decides. Prose citations are the case
    where mere existence is the whole question, so that is the case this
    file takes.

Relation to the other end of the same subject. progress.py reports UNMAPPED
for a TODO item nobody has recorded evidence for: an item with no artifact.
This file reports the mirror image: a citation with no item. Neither implies
the other, and an item can be both well-evidenced and miscited.
"""

import importlib.util
import os
import pathlib
import re

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOCS = ROOT / "docs"
PROGRESS = ROOT / "scripts" / "progress.py"

#: Documents that are not "citers" and so are not scanned.
#:
#: The TODO is the authority on what exists; scanning it for citations of
#: itself would make every id trivially present and the check vacuous. This
#: is the same self-harm shape CLAUDE.md 3.2 form 3 names -- a criterion
#: inside its own scan surface -- and it is worth spelling out rather than
#: leaving as an obvious omission, because "obvious" is how it comes back.
#:
#: TODO_EVIDENCE.tsv needs no entry here: the loop below only opens *.md, so
#: the tsv is outside the surface by shape rather than by exception. Listing
#: it anyway would suggest the filter is by name, and the next person would
#: add a file to this set instead of asking why it is being scanned at all.
NOT_CITERS = {"XBRAIN_V6_TODO.md"}


def _progress_module():
    """Load scripts/progress.py by path.

    It is a script rather than a package, so there is no import path to it;
    spec_from_file_location is how the rest of this repo's tests reach the
    same kind of file (see tests/deploy/test_affinity_gate.py). The module
    name given here is deliberately not "progress": under pytest's importlib
    mode two modules claiming one name is exactly the collision the repo hit
    before, and a distinct name costs nothing.
    """
    spec = importlib.util.spec_from_file_location("progress_for_crossrefs",
                                                  str(PROGRESS))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _known_ids_and_prefixes():
    """(normalised id set, prefix set, norm_id) read out of the TODO itself.

    The assertions here guard the derivation rather than the subject. If
    parse() returns nothing -- the TODO moved, or its table shape changed --
    then the id set is empty, every citation is "dangling", and the failure
    would point the reader at the citing documents instead of at the parser.
    Failing here instead says the right thing.
    """
    progress = _progress_module()
    items = progress.parse()
    assert items, \
        "progress.parse() returned nothing -- the TODO moved or its shape changed"
    known = {progress.norm_id(item["id"]) for item in items}
    # A prefix is everything before the trailing number: BIZ-P3-07 -> BIZ-P3.
    # Ids whose tail is not a number are skipped rather than guessed at; they
    # contribute no prefix and so widen nothing.
    # norm_id pads toward the two-digit form, so BIZ-P3-7 and BIZ-P3-07 both
    # arrive here as BIZ-P3-07 and the prefix comes out the same either way.
    # The direction only matters for what the SET contains, never for
    # comparison, since both sides of every comparison below go through it.
    prefixes = set()
    for task_id in known:
        head, _, tail = task_id.rpartition("-")
        if tail.isdigit() and head:
            prefixes.add(head)
    assert prefixes, "no task-id prefixes derived from the TODO"
    return known, prefixes, progress.norm_id


def _citations(prefixes):
    """{document: {raw id: first line number}} for every citation under docs/.

    The pattern is built from the derived prefixes, so a token only counts as
    a citation when its family is a family the TODO actually uses. The
    optional trailing letter matches the suffixed ids the TODO does carry
    (BIZ-P2-10a and friends); without it such a citation would be clipped to
    its numeric part and silently compared against the wrong item.
    """
    pattern = re.compile(
        r"\b(?:%s)-[0-9]{1,3}[a-z]?\b"
        % "|".join(sorted(map(re.escape, prefixes))))
    # listdir, not a recursive walk, and that is a decision rather than a
    # shortcut. docs/temp/_* holds the migration material CLAUDE.md 0.2 bans
    # anything from DEPENDING on -- it is scratch, kept only until its content
    # lands in a numbered volume. A dangling citation in there is not a defect
    # to report; it is a sign the material has not been migrated yet, which is
    # the state that directory exists to represent. A recursive walk would
    # turn that into permanent noise, and noise is how a check gets turned off.
    out = {}
    for name in sorted(os.listdir(DOCS)):
        if not name.endswith(".md") or name in NOT_CITERS:
            continue
        # First line number only: a document citing one id in forty places
        # should produce one finding pointing at the first, not forty copies.
        hits = {}
        with open(DOCS / name, encoding="utf-8") as handle:
            for lineno, line in enumerate(handle, 1):
                for match in pattern.finditer(line):
                    hits.setdefault(match.group(0), lineno)
        if hits:
            out[name] = hits
    return out


def test_every_cited_task_id_exists_in_the_todo():
    """*** The criterion: an id cited by any document must be a real TODO id.

    The scan surface is printed unconditionally rather than only on failure.
    A run that scanned nothing and a run that scanned everything and found
    nothing produce the same line otherwise, and this repo has paid for that
    confusion more than once (CLAUDE.md 3.2 form 6).

    The "something cites something" assertion is not ceremony either. If the
    prefix derivation above ever breaks -- a TODO reshape, a parse() change --
    the pattern matches nothing, no citation is collected, and the dangling
    list is trivially empty. That is a permanently green assertion wearing
    this one's name (form 1), and the only thing standing between here and
    there is that one line.

    Mutation (both verified 2026-10-01, __pycache__ cleared each run):
      * cite an id that does not exist => red
      * rename a cited id inside the TODO => red, naming document and line
    The second mutant FAILED TO INJECT on the first attempt -- the shell that
    picked the id came back empty, the deletion that followed matched nothing,
    and the test stayed green. A mutant that did not land proves exactly
    nothing, so the run was redone with the injection self-reported.
    """
    known, prefixes, norm_id = _known_ids_and_prefixes()
    cites = _citations(prefixes)

    print("scan surface: %d id prefixes derived from the TODO, %d known ids"
          % (len(prefixes), len(known)))
    for name in sorted(cites):
        print("  %-44s %d distinct citations" % (name, len(cites[name])))
    assert cites, (
        "no document cites any TODO id -- either the prefix derivation broke "
        "or docs/ stopped cross-referencing, and both make this test vacuous")

    # Every dangling citation is collected before asserting, not reported one
    # at a time. These get fixed in a pass -- a rename typically strands
    # several citations at once -- and a check that surfaces one per run turns
    # one edit into several round trips. The cost of collecting them all is a
    # dictionary lookup per citation, since nothing here runs a test or opens
    # a file twice; this whole check is cheap enough to leave on forever.
    dangling = []
    for name in sorted(cites):
        for raw, lineno in sorted(cites[name].items()):
            if norm_id(raw) not in known:
                dangling.append((name, lineno, raw))
    # The failure names both possibilities rather than guessing between them.
    # A dangling citation is either a rename the citing document did not
    # follow, or a sentence about an item that never existed -- and the fix
    # differs: the first edits the citation, the second questions the whole
    # sentence. A message that assumed one would send half the readers down
    # the wrong path, and this check exists precisely because telling those
    # two apart by hand is what the reader cannot do.
    assert not dangling, (
        "these documents cite task ids that do not exist in "
        "docs/XBRAIN_V6_TODO.md:\n"
        + "\n".join("  %s:%d  %s" % row for row in dangling)
        + "\nEither the id was renamed (fix the citation) or it never existed "
          "(the sentence is about nothing). Comparison runs on the normalised "
          "form, so zero padding is not what is being reported here.")
