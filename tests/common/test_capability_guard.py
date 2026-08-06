"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_capability_guard.py
Brief: INF-DB-4 clause 1 -- the capability-unavailable table matches 21 both ways, and the accepted-mutation goes red

Description:
This binds capability_guard's unavailable table to 21 (measurement/third-party
debt), the source the INF-DB-4 done-criterion names for clause 1. The core is a
SYMMETRIC-difference metatest, the same shape as test_error_codes.py against
11 S13: a row 21 adjudicates unavailable that the table misses, and a debt id in
the table that 21 does not adjudicate, both fail. One-directional containment is
the failure this project keeps catching, so it is not used.

Scope of the scan surface, declared not assumed. The parser reads the two V-class
sections of 21 (section 2, the 11-segment debts, and section 3, the 13-segment
debts) and selects the rows whose fourth column -- the unclosed-default
disposition -- names E_CAPABILITY or E_NOT_IMPLEMENTED. Those are exactly the
adjudicated-unavailable functions. Rows disposed any other way (startup refusal,
warn, null, unknown-marking) are not capability rejections and are correctly out
of surface. A separate test asserts both section headings are still found, so a
document restructure that hid a section would fail loudly instead of quietly
emptying the diff.

The mutation, performed not described (CLAUDE.md 3.3). The INF-DB-4 row names one:
"make an unavailable function return accepted -> clause 1 goes red". Here "return
accepted" is capability_guard returning None (no opinion) for a function that is in
fact unavailable. The test drops one row from the table and observes the guard
answer None for the function that row named -- the exact collapse of "unavailable"
into "available" the guard exists to prevent -- while the control (real table)
returns a rejection. The two together show the rejection assertion discriminates.

What this file does NOT establish, so a green run is not read as more than it is:
  * that the CODES are the right disposition for each function in an absolute
    sense. It checks the table equals 21's fourth column; if 21 itself is wrong,
    that is a 21 defect this cannot see. The one place 21 and 13 disagree (V-59)
    is a recorded conflict, followed per the criterion's named source (21), not
    resolved here.
  * that detail carries the right keys. 11 S13.13 group J gives these two codes no
    detail column, so there is no required key; the detail test asserts that
    vacuity against the contract rather than pretending to check keys that do not
    exist.
"""

import os
import re
import sys

import pytest

# ROOT is three levels up (tests/common/this_file -> repo root), inserted on
# sys.path because there is no conftest.py, matching test_error_codes.py exactly.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common import errors  # noqa: E402
from xbrain.common.errors import capability  # noqa: E402

# The debt ledger. 21 is the source INF-DB-4 clause 1 names, so the metatest reads
# it directly rather than a transcription of it.
DEBT_DOC = os.path.join(ROOT, "docs", "21-实测与第三方欠账.md")

# The two rejection codes, as plain strings for the DOCUMENT scan. tests/ is
# outside no_literal_ecode.py's surface, so spelling them here is allowed and is
# in fact necessary -- the point is to find these tokens in 21's prose, and the
# table under test binds them from the shared library, which is what the code-side
# rule protects. The runtime assertions still compare against errors.E_CAPABILITY
# and errors.E_NOT_IMPLEMENTED so the wire spelling is pinned to the library.
DOC_CODES = ("E_NOT_IMPLEMENTED", "E_CAPABILITY")

# The two V-class section headings that bound the scan. Anchored on the "## N. V "
# prefix so the exact decorated title text may change without breaking the parser,
# while a section being renamed away from "V" (i.e. no longer a debt-class section)
# is caught by test_scan_surface_intact.
SEC2 = re.compile(r"^##\s+2\.\s+V\b")
SEC3 = re.compile(r"^##\s+3\.\s+V\b")
SEC4 = re.compile(r"^##\s+4\.")

# A V-class debt id: V-NN. Two digits, matching 13's own numbering audit
# (grep -oE 'V-[0-9]{2}'); one-digit V-1/V-2/V-3 are a different namespace
# (implementation constraints) that must not be dragged in.
ID_V = re.compile(r"\bV-[0-9]{2}\b")


def _split_row(line):
    """A markdown row's cells, leading/trailing empty cell dropped.

    21's tables carry no escaped pipes in these sections, so a plain split is
    enough; the disposition cell (index 3) is what the codes live in.
    """
    cells = line.split("|")
    if cells and cells[0].strip() == "":
        cells = cells[1:]
    if cells and cells[-1].strip() == "":
        cells = cells[:-1]
    return [c.strip() for c in cells]


def parse_debt_ledger():
    """{debt_id: code} for every capability-rejection row in 21 sections 2 and 3.

    Reads the disposition cell (column 4) of each V-row and keeps the row only when
    that cell names exactly one of the two rejection codes. Also returns which of
    the two section headings were seen, so the scan surface can be asserted intact.
    """
    lines = open(DEBT_DOC, encoding="utf-8").read().split("\n")  # the ledger, whole
    # Locate the section boundaries. The scan runs from the section-2 heading to
    # the section-4 heading (section 4 is M-class, measurement debt, no capability
    # rejections); everything between is sections 2 and 3.
    start = next((i for i, ln in enumerate(lines) if SEC2.match(ln)), None)  # section 2 start
    end = next((i for i, ln in enumerate(lines) if SEC4.match(ln)), None)  # section 4 = stop
    saw_sec2 = start is not None  # recorded so the surface test can assert it
    saw_sec3 = any(SEC3.match(ln) for ln in lines)  # section 3 heading present at all
    out = {}  # debt_id -> code, filled below
    if start is None or end is None:
        # Return what was found so the surface test can report the miss; the diff
        # test guards on the same flags and refuses to run on an empty scan.
        return out, saw_sec2, saw_sec3  # empty map, flags tell the caller why
    for ln in lines[start:end]:  # only the two V-class sections
        if not ln.startswith("|"):  # non-table prose is skipped
            continue
        cells = _split_row(ln)  # split into columns
        # A data row needs at least the id and the four content columns; the header
        # row and the |---| separator do not, so they fall out here.
        if len(cells) < 4:  # header and separator rows are too short
            continue
        ids = ID_V.findall(cells[0])  # the debt id lives in the first column
        if not ids:  # a row with no V id is not a debt row
            continue
        disposition = cells[3]  # column 4: the unclosed-default disposition
        # Which codes the disposition names. Substring tests are exact enough:
        # E_NOT_IMPLEMENTED does not contain E_CAPABILITY and vice versa, so the
        # two are independent.
        named = [c for c in DOC_CODES if c in disposition]  # codes this row names
        if not named:
            continue  # disposed some other way -- not a capability rejection
        # A row naming both codes is ambiguous and must not be guessed at; fail so
        # a human disambiguates 21 rather than this parser picking one.
        assert len(named) == 1, (
            "row %r names both rejection codes; 21 must say one" % cells[0])
        # Exactly one id per number cell in these sections; assert it rather than
        # silently taking the first, so a row that grew a second id is noticed.
        assert len(ids) == 1, (
            "row %r carries more than one V id in its number cell" % cells[0])
        out[ids[0]] = named[0]
    return out, saw_sec2, saw_sec3


def test_scan_surface_intact():
    """Both V-class sections are still reachable by the parser.

    Guards the guard: if section 3 were renamed and the parser stopped seeing it,
    every 13-segment debt would vanish from the doc side and the diff below would
    read the table's rows as table-only rather than as a parse failure. Two known
    anchors are required present so a subtler break -- the section found but its
    rows missed -- is caught too.
    """
    ledger, saw2, saw3 = parse_debt_ledger()
    assert saw2, "21 section 2 (V-class, 11 segment) heading not found -- parser or doc changed"
    assert saw3, "21 section 3 (V-class, 13 segment) heading not found -- parser or doc changed"
    # V-47 sits in section 3 and V-06 in section 2, so requiring both proves the
    # scan reached into each section body, not just past its heading.
    assert "V-47" in ledger, "V-47 not parsed from 21 section 3 -- the row scan is broken"
    assert "V-06" in ledger, "V-06 not parsed from 21 section 2 -- the section-2 scan is broken"


def test_table_matches_ledger_both_directions():
    """*** The core assertion of INF-DB-4 clause 1.

    A symmetric difference on the debt ids, then a code check on the intersection.
    The table's codes are read through the public guard (capability_guard(d).code),
    not by reaching into the module's private dict, so this exercises what a caller
    would actually get.
    """
    ledger, saw2, saw3 = parse_debt_ledger()
    # Do not run the diff on a scan that did not reach both sections: an empty or
    # half-empty ledger would make the "table has extra rows" arm fire for a parse
    # bug rather than a real drift. The surface test owns that failure.
    assert saw2 and saw3, "scan surface incomplete; see test_scan_surface_intact"
    table = {d: errors.capability_guard(d).code for d in errors.CAPABILITY_DEBTS}  # via the public guard
    only_doc = sorted(set(ledger) - set(table))  # rows 21 has that the table lacks
    only_table = sorted(set(table) - set(ledger))  # rows the table has that 21 lacks
    assert not only_doc, (
        "21 adjudicates these unavailable but the guard table misses them: %s -- "
        "add them to xbrain/common/errors/capability.py" % only_doc)
    assert not only_table, (
        "the guard table carries debts 21 does not adjudicate unavailable: %s -- "
        "remove them or amend 21" % only_table)
    # Same id on both sides must carry the same code. This is where the V-59
    # conflict would surface if the table ever switched to 13's E_NOT_IMPLEMENTED:
    # 21's fourth column says E_CAPABILITY, so the table must too.
    mismatch = {d: (ledger[d], table[d]) for d in ledger if ledger[d] != table[d]}
    assert not mismatch, (
        "code disagrees between 21 and the guard table (debt: doc, table): %s"
        % mismatch)


def test_guard_rejects_every_unavailable_function():
    """Clause 1 at run time: each unavailable function returns a rejection.

    The positive half beside the metatest. A rejection is non-None, carries one of
    the two group-J codes from the shared library (so the wire spelling is pinned),
    and names its debt id back. is_failure confirms the code is a real failure, not
    a success dressed up.
    """
    valid = {errors.E_CAPABILITY, errors.E_NOT_IMPLEMENTED}  # the only two group-J codes
    for debt in errors.CAPABILITY_DEBTS:  # every adjudicated-unavailable function
        rej = errors.capability_guard(debt)  # the guard's answer for it
        assert rej is not None, "%s must be rejected, not accepted" % debt  # non-None = rejected
        assert rej.debt == debt, "rejection must name its own debt id"  # traceable back
        assert rej.code in valid, (  # a shared-library code, so the wire spelling is pinned
            "%s -> %r, not one of the group-J rejection codes" % (debt, rej.code))
        assert errors.is_failure(rej.code), (  # a real failure, not a success dressed up
            "%s -> %r which is not a failure code" % (debt, rej.code))


def test_guard_has_no_opinion_off_table():
    """None means not-in-table, and that is distinct from a rejection.

    Two cases: a nonexistent id, and a real 21 debt that is disposed some other way
    (V-01 refuses startup on a null limit, it is not a capability rejection). Both
    must return None -- the guard owns only the rejected path. Without this an
    all-rejecting guard would pass the clause-1 test above while rejecting things it
    should stay silent on.
    """
    assert errors.capability_guard("V-99") is None, "an unknown id is not a rejection"
    assert errors.capability_guard("V-01") is None, (
        "V-01 is disposed by startup refusal, not a capability rejection")
    assert errors.capability_guard("") is None, "the empty string is not a debt id"


def test_detail_is_contract_vacuous_not_invented():
    """detail is empty because the contract mandates no key -- asserted, not assumed.

    11 S13.13 group J gives E_CAPABILITY and E_NOT_IMPLEMENTED no detail column, and
    codes.yaml marks both detail unspecified. So "detail 必填项齐全" (required detail
    keys complete) is vacuously satisfied by an empty detail, and pinning a
    detail.item here would be inventing a key the contract does not define -- worse,
    picking a side in the cross-volume detail.item conflicts the module docstring
    records. This test states that reasoning as an assertion: if 11 ever makes
    either code detail required, detail_requirement changes and this fires, forcing
    the guard to start carrying the key. It is deliberately NOT dressed up as a key
    check, which would be the empty-shell assertion (CLAUDE.md 3.2 form 1) this
    project keeps catching.
    """
    for debt in errors.CAPABILITY_DEBTS:
        rej = errors.capability_guard(debt)
        assert rej.detail == {}, (
            "%s carries a detail key the contract does not require: %r"
            % (debt, rej.detail))
        assert errors.detail_requirement(rej.code) == "unspecified", (
            "%s -> %r is now detail %r in codes.yaml; the guard must start filling "
            "the required key instead of returning an empty detail"
            % (debt, rej.code, errors.detail_requirement(rej.code)))


def test_accepted_mutation_is_caught(monkeypatch):
    """*** The INF-DB-4 mutation, performed (CLAUDE.md 3.3).

    Control: the real table rejects V-47 with E_CAPABILITY. Mutant: drop V-47 from
    the table, and the guard now answers None -- "no opinion", which a caller reads
    as go-ahead. That is an unavailable function returning accepted, exactly clause
    1's named mutation, and it is what would turn
    test_guard_rejects_every_unavailable_function red. monkeypatch restores the
    table after the test, so the mutation cannot leak into another test.
    """
    # Control, on the real module attribute the guard reads at call time.
    control = errors.capability_guard("V-47")  # real table: V-47 is unavailable
    assert control is not None and control.code == errors.E_CAPABILITY  # rejected today

    # Mutant: a copy of the table with V-47 removed, swapped in for the real one.
    # capability_guard looks _UNAVAILABLE up in the module globals on each call, so
    # replacing the module attribute is enough to drive the real function's logic
    # against the mutated data -- not a re-implementation of it.
    mutated = dict(capability._UNAVAILABLE)  # copy so the real table is untouched
    del mutated["V-47"]  # the mutation: drop one unavailable function's row
    monkeypatch.setattr(capability, "_UNAVAILABLE", mutated)  # auto-restored after the test
    assert errors.capability_guard("V-47") is None, (
        "with its row dropped, the guard now accepts V-47 -- the mutation the "
        "clause-1 rejection assertion is meant to catch did not redden it")
