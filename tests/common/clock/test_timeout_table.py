"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_timeout_table.py
Brief: Metatest binding xbrain/common/clock/timeouts.yaml to 11 S1.6, both directions

Description:
This is the whole point of the second half of INF-CM-3. The shared timeout
registry and the contract's S1.6 master table are two documents maintained by
different people at different times; nothing but this test keeps them equal.

*** The assertion is a SYMMETRIC difference, not containment. One-directional
containment is the shape this project keeps catching: "every id the registry
holds appears in S1.6" passes happily while the registry is missing an id S1.6
added -- and a timeout that exists in the contract but not in the registry is a
protection nobody wrote down. Both mutation cases below have been run and both
go red (CLAUDE.md 3.3: an assertion that has never been red has not been
written).

*** What this test establishes and what it does NOT (CLAUDE.md 3.2 form 7, and
the S25 dedup ruling on INF-CM-3). The committed copy timeouts.yaml is checked
against a FRESH extraction of S1.6 -- "生成物入库副本 == 重跑输出". That catches
two realistic defects: a hand edit to the registry that drifts from the table
(mutation 2 / 3), and a row added to the table that nobody mirrored into the
registry (mutation 1). It does NOT establish that a threshold is the RIGHT
number for the hardware: comparing the registry against the very table a human
transcribed it from cannot prove the human read the vendor correctly. That
correctness lives with the human reading S1.6, and with the owning process's
config where the runtime value actually comes from -- the registry holds the
contract text for audit, never a number the safety loop divides by.

*** Relationship to the CLK-C1 static scan (the INF-CM-3 row nails this down).
scripts/lint/clock_scan.py catches "a wall clock was READ in the source"; this
test catches "a threshold was copied wrong or an id was left out of the export".
The two do not substitute for each other in either direction: a file can pass
clock_scan while its registry is missing T-11, and the registry can be perfect
while some process reads time.time(). clock_scan.py states the same boundary
from its side in its own Description.

* Scan surface is asserted, not assumed: extract_contract_timeouts reports which
S1.6.x subsections it found, and test_scan_surface_intact fails if that set
shrinks. A parser that silently stops finding subsection C would make the
registry look complete and the symmetric difference look empty (CLAUDE.md 3.2
form 3 / form 6). The two doc anchors that carry the timebase rule -- the A
heading's 全部单调时钟 and T-46's 墙钟 -- are asserted present for the same
reason: the monotonic/wall split is derived from those words, so if they vanish
the derivation is asserting a rule the document no longer states.
"""

import os
import re
import sys
import tempfile

import pytest

# Four dirnames: tests/common/clock/test_timeout_table.py -> repo root. Derived
# from __file__ rather than written out so a copied tree tests itself, which the
# temp-file mutation below relies on.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

from xbrain.common import clock  # noqa: E402

#: The contract volume that owns the master timeout table (11 S1.6). A literal
#: path, not a configurable one: this test has exactly one document to check
#: against, and making it an argument would only add a way to point it at the
#: wrong file.
CONTRACT = os.path.join(ROOT, "docs", "11-接口契约.md")

#: The three subsections of S1.6, named explicitly so a parser that stops
#: finding one fails loudly (test_scan_surface_intact) instead of quietly
#: returning a smaller table. A -> safety loop, B -> link/session/health,
#: C -> non-safety loop; the letter is the group recorded per row.
EXPECTED_GROUPS = frozenset("ABC")

#: Decoration the contract sprinkles through a threshold cell for emphasis and
#: that carries no value: markdown bold, back-ticks, the star/warning/check/ban
#: glyphs used as severity marks, and the U+FE0F variation selector that trails
#: an emoji. Stripped before comparison so that a purely cosmetic edit to a cell
#: -- bolding a number that was already there -- does not read as a value change.
#: This set is the ONE place the normalisation is defined; timeouts.yaml is
#: authored to match it, and drift between the two is exactly what this file
#: reports. Kept deliberately small: every glyph added here is a real edit the
#: test would then stop noticing, so it holds only marks, never digits, units,
#: brackets or Chinese characters.
_DECORATION = set("*`★☆⚠️✅❌\U0001f6ab☑☰")


def _split_row(line):
    """Split a markdown table row on UNESCAPED pipes only.

    A cell containing a backslash-escaped pipe is content, not a delimiter.
    Splitting on the bare character shifts every later column of that row, which
    would silently move a threshold into the wrong slot. Borrowed verbatim from
    tests/common/test_error_codes.py so both metatests read the contract the
    same way -- two slightly different splitters is two chances to be wrong.
    """
    cells, cur, i = [], "", 0
    while i < len(line):
        # A backslash before a pipe means the pipe is literal cell content.
        if line[i] == "\\" and i + 1 < len(line) and line[i + 1] == "|":
            cur += "|"
            i += 2
            continue
        if line[i] == "|":
            cells.append(cur)
            cur = ""
            i += 1
            continue
        cur += line[i]
        i += 1
    cells.append(cur)
    # cells[1:-1] drops the empty strings the outer pipes produce. A row with
    # fewer than two pipes is not a table row and yields nothing.
    return [c.strip() for c in cells[1:-1]] if len(cells) > 2 else []


def normalize_threshold(cell):
    """The in-force threshold token for one S1.6 threshold cell.

    Two mechanical steps, and both are deliberately dumb so the same rule can be
    applied by eye when authoring timeouts.yaml:

      1. Keep only the head, up to the first <br>. The contract puts the value
         first and the justification in the <br> continuations that follow; the
         monster cells (T-45, T-BCAST-MAX) are paragraphs of reasoning after
         that first break, and pulling them in would make every edit to the
         reasoning read as a threshold change.
      2. Drop the decoration glyphs and collapse whitespace. What survives is
         the value plus whatever unit / qualifier the contract wrote inline --
         verbatim, because the moment this function starts deciding that "3 s"
         is the value and "建议收紧 1.5 s" is a note, it is interpreting the
         Chinese, and an interpreter is a place to be subtly wrong. Keeping both
         is the honest, checkable choice: the token changes if EITHER number
         changes, and a human reviews S1.6 to know which one is live.
    """
    head = cell.split("<br>")[0]
    kept = "".join(ch for ch in head if ch not in _DECORATION)
    # \s+ -> single space, then strip: removing a glyph that sat between two
    # spaces leaves a double space, and an unstable amount of whitespace would
    # make the token depend on invisible detail.
    return re.sub(r"\s+", " ", kept).strip()


def extract_contract_timeouts(path=CONTRACT):
    """Parse the S1.6 master table into {id: (group, timebase, threshold)}.

    Returns the mapping AND the set of subsection letters it actually reached,
    so the caller can assert the scan surface did not silently shrink.

    Only the S1.6.1 / S1.6.2 / S1.6.3 table bodies are read. S1.6.4 (the
    worst-case scenario table just below) is full of "T-02 -> T-03 -> ..."
    references and is bounded out on purpose: mining ids from prose would let a
    scenario sentence keep a deleted timeout alive in the set. Patch blocks
    elsewhere in the volume that propose new T-* rows are likewise out of scope
    -- criterion (1) says 表体, the canonical table body, and 11 has a history
    of unmerged patch blocks (MEMORY: 块外正本才是判据).
    """
    lines = open(path, encoding="utf-8").read().split("\n")
    # Subsection headings, e.g. "#### 1.6.1 A - 安全回路(...)". The group letter
    # is captured from the heading, not guessed from the id range, so a row
    # filed under the wrong subsection is visible as a group mismatch.
    heads = []
    for i, ln in enumerate(lines):
        m = re.match(r"^####\s+1\.6\.([123])\s+([ABC])\s*·", ln)
        if m:
            heads.append((m.group(2), i))
    assert heads, "S1.6.1~S1.6.3 subsection headings not found -- parser or document changed"
    # End boundary: the first S1.6.4 heading after the last subsection. Without
    # it, subsection C would run into the scenario table and swallow its prose.
    end = next(
        (i for i, ln in enumerate(lines)
         if i > heads[-1][1] and re.match(r"^####\s+1\.6\.4", ln)),
        len(lines),
    )

    out = {}
    groups = set()
    for k, (grp, start) in enumerate(heads):
        stop = heads[k + 1][1] if k + 1 < len(heads) else end
        groups.add(grp)
        for ln in lines[start:stop]:
            # Only table rows. The heading, the blank lines and the intro
            # blockquote above each table do not start with a pipe.
            if not ln.startswith("|"):
                continue
            cells = _split_row(ln)
            # Columns are: # | 判定方 | 被判对象 | 阈值 | 触发行为 | 出处. A row
            # with fewer than six is the header or the |---| separator, which
            # carry no id and are skipped by the id search below anyway.
            if len(cells) < 6:
                continue
            # The id lives in the first cell, possibly wrapped in bold/back-ticks
            # and trailed by a date note. Take the first T-... token; the first
            # cell never names a second timeout. a-z is allowed so a future
            # suffixed id (T-09a) is captured whole rather than clipped to T-09.
            m = re.search(r"T-[0-9A-Za-z]+(?:-[0-9A-Za-z]+)*", cells[0])
            if not m:
                continue
            tid = m.group(0)
            # Timebase is derived from the document's own stated rule, not from a
            # per-row column (S1.6 has none). The A heading says every safety
            # entry is monotonic and T-46 is singled out as the one wall-clock
            # entry; the whole intro says the clock column is CLOCK_MONOTONIC
            # with no exception but T-46. test_scan_surface_intact asserts both
            # of those anchor phrases still exist, so this derivation is tied to
            # the document rather than asserted from nowhere.
            timebase = "wall" if tid == "T-46" else "monotonic"
            out[tid] = (grp, timebase, normalize_threshold(cells[3]))
    return out, groups


def _export_records():
    """{id: (group, timebase, threshold)} from the shared library side."""
    # Read through the clock package, never by re-parsing timeouts.yaml: a
    # second parser is a second thing to keep correct, and the package's loader
    # already RAISES on anything it does not recognise instead of skipping it.
    return {tid: (t.group, t.timebase, t.threshold) for tid, t in clock.TIMEOUTS.items()}


def a_group_wall_violations(records):
    """Ids in group A whose timebase is not monotonic.

    A free function so both the green test and the mutation test drive the same
    logic: the green one passes the real registry and expects an empty list, the
    red one passes a registry with one A row flipped to wall and expects it back.
    """
    return sorted(tid for tid, (grp, tb, _thr) in records.items()
                  if grp == "A" and tb != "monotonic")


# -- scan surface --------------------------------------------------------------

def test_scan_surface_intact():
    """All three subsections reachable, and the two timebase anchors present.

    Guards the guard. If a heading is renamed and the parser stops seeing
    subsection C, every C timeout vanishes from the contract side and the
    symmetric-difference test below would report them as export-only rather than
    as a parse failure -- which points the reader at the registry instead of at
    the document. The anchor checks guard the timebase derivation: it reads
    "A is monotonic, T-46 is wall" out of these exact words.
    """
    extracted, groups = extract_contract_timeouts()
    assert groups == EXPECTED_GROUPS, (
        f"subsections found {sorted(groups)}, expected {sorted(EXPECTED_GROUPS)} "
        "-- a S1.6.x heading changed, or the parser no longer matches it"
    )
    assert extracted, "no timeouts extracted from S1.6 -- the table body moved or the parser broke"
    text = open(CONTRACT, encoding="utf-8").read()
    # The A-group heading states every safety timeout is monotonic. This is the
    # ground truth criterion (2) rests on; if it is gone, asserting "A is
    # monotonic" asserts a rule the document no longer makes.
    assert "全部单调时钟" in text, (
        "S1.6.1 no longer states 全部单调时钟 -- the monotonic derivation lost its anchor"
    )
    # T-46 is the single wall entry, and the word 墙钟 sits in its row (in the
    # 被判对象 cell, not the threshold cell -- so this checks the whole file, not
    # just cells[3]). If it is gone, T-46 being wall is no longer document-backed.
    assert re.search(r"T-46.*墙钟", text), (
        "T-46 row no longer says 墙钟 -- the single wall-clock derivation lost its anchor"
    )


# -- criterion (1): the committed copy equals a fresh extraction ---------------

def test_committed_copy_matches_contract():
    """*** The core assertion of INF-CM-3's second half (criterion 1).

    Symmetric difference over full records, so a drift in group, timebase OR
    threshold shows up, not only a missing id. "生成物入库副本 == 重跑输出".
    """
    contract, _groups = extract_contract_timeouts()
    export = _export_records()
    only_contract = sorted(set(contract) - set(export))
    only_export = sorted(set(export) - set(contract))
    assert not only_contract, (
        f"S1.6 has timeout ids the registry is missing: {only_contract} -- "
        "add them to xbrain/common/clock/timeouts.yaml"
    )
    assert not only_export, (
        f"registry exports ids S1.6 does not define: {only_export} -- "
        "remove them or amend 11 S1.6 (no inventing a timeout)"
    )
    # Ids agree; now every shared id must agree field for field. Reported as a
    # sorted list of (id, contract, registry) triples so a run is reproducible
    # and a reviewer sees exactly which cell moved.
    mismatches = sorted(
        (tid, contract[tid], export[tid]) for tid in contract if contract[tid] != export[tid]
    )
    assert not mismatches, (
        "registry disagrees with S1.6 on (group, timebase, threshold): "
        f"{mismatches} -- regenerate the token by the rule in normalize_threshold"
    )


def test_threshold_is_carried_for_every_id():
    """Every id carries a non-empty threshold token on both sides.

    A threshold that normalised to "" would make two different timeouts compare
    equal on their value and hide a real difference; and a registry entry with a
    blank threshold is a row someone half-filled. Neither side may be empty.
    """
    contract, _ = extract_contract_timeouts()
    export = _export_records()
    blank_contract = sorted(tid for tid, (_g, _t, thr) in contract.items() if not thr)
    blank_export = sorted(tid for tid, (_g, _t, thr) in export.items() if not thr)
    assert not blank_contract, f"S1.6 rows with an empty threshold cell: {blank_contract}"
    assert not blank_export, f"registry rows with an empty threshold: {blank_export}"


# -- criterion (2): every A-group timeout is monotonic -------------------------

def test_A_group_all_monotonic():
    """Criterion (2): no safety-loop timeout may be a wall clock.

    Evaluated over the registry, which is what a future consumer imports. The A
    subsection heading forbids configuring these as wall clocks 'without
    exception'; a wall clock here reintroduces exactly the RTK-step failure
    S1.6.4 S5/S6 walk through.
    """
    violations = a_group_wall_violations(_export_records())
    assert not violations, (
        f"A-group (safety loop) timeouts marked non-monotonic: {violations} -- "
        "11 S1.6.1 forbids a wall clock here without exception"
    )


# -- positive anchors (an empty shell must not pass) ---------------------------

def test_registry_is_populated_and_grouped():
    """A positive shape check beside the difference tests.

    Without it, a loader that returned {} would sail through every symmetric
    difference (empty vs empty) and every "all A are monotonic" (vacuously
    true). This pins that the registry is non-empty, spans all three groups, and
    that T-46 -- the single wall entry the whole table is built around -- is
    present and is wall. Structural, not a second copy of any threshold number.
    """
    records = _export_records()
    assert records, "registry is empty -- timeouts.yaml did not load"
    groups = {grp for _tid, (grp, _tb, _thr) in records.items()}
    assert groups == EXPECTED_GROUPS, f"registry groups {sorted(groups)} != {sorted(EXPECTED_GROUPS)}"
    walls = sorted(tid for tid, (_g, tb, _t) in records.items() if tb == "wall")
    # Exactly one wall entry, and it is T-46. The contract is explicit that T-46
    # is the ONLY wall-clock row; more than one, or a different one, is a defect.
    assert walls == ["T-46"], f"expected T-46 as the sole wall entry, found {walls}"


def test_unknown_timeout_id_raises():
    """clock.timeout() rejects an id outside the closed table.

    The registry is a closed set like the error codes; a miss is a caller bug,
    not a reason to return a plausible-looking default. Mirrors
    test_error_codes.test_out_of_set_value_raises.
    """
    with pytest.raises(KeyError):
        clock.timeout("T-99999")


# -- mutations: each named mutation, shown to turn a test red ------------------
# CLAUDE.md 3.3 -- an assertion validated only by the positive case is not
# validated. The three mutations the INF-CM-3 row names are injected here and
# each is asserted to fail the check it targets.

def test_mutation_1_new_row_in_contract_is_caught():
    """Mutation 1: add a row to S1.6 without touching the export -> criterion (1) red.

    Injected into a temp COPY of the contract, because the test must leave the
    real document as it found it. A T-90 row is spliced into the S1.6.1 table
    right after T-12 (still inside subsection A, before the S1.6.2 heading), and
    the fresh extraction is expected to carry an id the registry does not.
    """
    lines = open(CONTRACT, encoding="utf-8").read().split("\n")
    # Find the T-12 data row and insert the fabricated row directly after it.
    anchor = next(i for i, ln in enumerate(lines)
                  if ln.startswith("|") and re.search(r"T-12", ln))
    injected = "| **T-90** | test | test | **999 ms** | test | test |"
    mutated = lines[:anchor + 1] + [injected] + lines[anchor + 1:]
    with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False, encoding="utf-8") as fh:
        fh.write("\n".join(mutated))
        tmp = fh.name
    try:
        contract, _groups = extract_contract_timeouts(tmp)
        export = _export_records()
        only_contract = set(contract) - set(export)
        # The mutant MUST be caught. If T-90 does not show up as contract-only,
        # the symmetric-difference test is not actually reading the table.
        assert "T-90" in only_contract, (
            "a new S1.6 row was NOT flagged as missing from the registry -- "
            "criterion (1) would pass an under-populated export"
        )
    finally:
        os.unlink(tmp)


def test_mutation_2_wrong_threshold_in_export_is_caught():
    """Mutation 2: hard-code a registry threshold different from the table -> criterion (1) red.

    The registry side is mutated (one threshold changed) and the same
    field-for-field comparison the core test runs is shown to report it. This is
    the case that makes the threshold part of criterion (1) load-bearing rather
    than decorative -- an ids-only diff would stay green here.
    """
    contract, _ = extract_contract_timeouts()
    export = _export_records()
    # Pick any shared id and corrupt only its threshold.
    victim = sorted(set(contract) & set(export))[0]
    grp, tb, thr = export[victim]
    mutated = dict(export)
    mutated[victim] = (grp, tb, thr + " (tampered)")
    mismatches = [tid for tid in contract if tid in mutated and contract[tid] != mutated[tid]]
    assert victim in mismatches, (
        "a tampered threshold was NOT flagged -- criterion (1) is comparing ids "
        "only and would miss a copied-wrong value"
    )


def test_mutation_3_A_group_marked_wall_fails_criterion_2():
    """Mutation 3: mark an A-group entry wall -> criterion (2) red.

    The registry is mutated so one safety-loop timeout claims a wall clock, and
    a_group_wall_violations -- the exact function test_A_group_all_monotonic
    trusts -- is shown to return it. A check that only ever saw monotonic input
    could be the constant True.
    """
    export = _export_records()
    victim = next(tid for tid, (grp, _tb, _thr) in export.items() if grp == "A")
    grp, _tb, thr = export[victim]
    mutated = dict(export)
    mutated[victim] = (grp, "wall", thr)
    violations = a_group_wall_violations(mutated)
    assert victim in violations, (
        "an A-group timeout flipped to wall was NOT reported -- criterion (2) "
        "would pass a safety timeout backed by a stepping clock"
    )
