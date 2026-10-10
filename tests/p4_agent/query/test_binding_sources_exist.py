"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_binding_sources_exist.py
Brief: every QueryBinding.source must name a key that exists in 11

Description:
Found 2026-10-10: of the three distinct `source` values in
xbrain/p4_agent/query/sources_g01_g24.py, TWO named keys that do not exist
anywhere in the contract -- `state/battery` (0 hits) and `state/health`
(0 hits). The field they were meant to carry lives elsewhere:
remain_mile_km is on PowerState, published on state/power (11 S4.2, 7 hits).

Nothing caught it because `QueryBinding.source` has NO READER. A grep for
`.source` across xbrain/p4_agent/query/ returns nothing; the whole binding
table is declaration only. So the values were never resolved against
anything, and a key name could be anything at all.

That is the shape SW-28 is about -- an implementation with green tests and
no production caller. This file does not fix the wiring; it makes the
DECLARATIONS checkable on their own terms, so that whoever wires the table
up starts from key names that exist.

WHAT THIS CANNOT DO: it checks that the key NAME appears in the contract,
not that the key carries the field the template interpolates. A binding
pointing at state/pose while its template reads {remain_mile_km} would
pass. Checking that needs the payload schemas per key, which this repo
does not have in machine-readable form -- registered rather than faked.

MUTANTS (run 2026-10-10, all three red):
  * put G03's source back to "state/battery" -> red, naming the binding
  * put G02's source back to "state/battery" -> red
  * point G01 at a plausible-but-absent key   -> red

*** G02 IS WHY THIS FILE EXISTS RATHER THAN A ONE-LINE FIX. The first pass
corrected G03 by hand, because G03 was the binding the investigation
started from. This criterion then went red on G02 -- the SAME wrong key,
in the binding immediately above, which the hand fix had walked straight
past. A criterion finds the second instance; reading finds the first.
"""

# os for the doc path; the binding tables under test.
import os

import pytest

from xbrain.p4_agent.query.sources_g01_g24 import BINDINGS

#: Reads committed files, no device.
pytestmark = pytest.mark.no_device

#: The contract. Two dirname() hops from tests/p4_agent/query/ reach
#: tests/, three reach the repo root.
_CONTRACT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))),
    "docs", "11-接口契约.md")

#: Placeholder sources are excluded from the existence check, and the
#: prefix is spelled here so the exclusion is visible rather than implicit.
#:
#: sources_g01_g24.py fills G04..G23 in a loop with
#: source=f"state/g_generic_{i:02d}" and template "[GNN placeholder]".
#: Those 20 entries exist so assert_bindings_cover_all_24 can report a
#: complete table; they are not claims about any key. Checking them would
#: make this file red for 20 reasons that are all the same reason, and a
#: check that is red for a known reason gets switched off.
#:
#: *** The thing worth noticing is the RATIO, not the exclusion: of the 24
#: G-queries, only a handful carry a real source. That is registered in
#: docs/NEXT.md rather than asserted here -- this file is about whether a
#: declared key exists, not about how many are declared.
_PLACEHOLDER_PREFIX = "state/g_generic_"

#: Sources that are known NOT to exist in the contract, each with the
#: reason it is still here. An entry is a DEBT with a name, not an
#: exemption: the assertion below reports the list on every run, so it
#: cannot quietly grow.
#:
#: state/health -- G24's binding. Its own docstring says it reads
#: `health/time.ts_sync`, and BOTH of those are wrong: health/time has 0
#: hits in the contract, and ts_sync is an ENVELOPE field carried by every
#: message (11 S1.3), not a payload field on some key. So G24 does not
#: have a "source key" in the sense this table assumes, and picking one
#: here would be inventing the design. Registered in docs/NEXT.md.
_KNOWN_ABSENT = {
    "state/health": "G24 reads the envelope ts_sync, not a key payload "
                    "-- see docs/NEXT.md, needs a design decision",
}


def _contract_text() -> str:
    """The contract, read fresh. A few MB, read once per case; the cost
    is irrelevant next to not caching a stale copy across cases."""
    with open(_CONTRACT, encoding="utf-8") as handle:
        return handle.read()


def test_every_binding_source_exists_in_the_contract():
    """Each distinct `source` appears in 11, except the declared debts.

    Substring match on the key name, not a parse of the key tables: the
    contract writes keys in several shapes (`state/power`,
    `xbrain/{rid}/state/power`, inside prose) and a parser tuned to one
    shape would miss the others and report drift that is not there -- the
    failure mode that gets a check relaxed and then ignored.
    """
    text = _contract_text()
    # Collect every offender before asserting. Failing on the first would
    # have reported G02 only after G03 was fixed and the suite re-run --
    # two passes to learn two facts the first pass already held.
    missing = {}
    for intent_id, binding in sorted(BINDINGS.items()):
        src = binding.source
        if src.startswith(_PLACEHOLDER_PREFIX) or src in _KNOWN_ABSENT:
            continue
        if src not in text:
            missing[intent_id] = src
    assert not missing, (
        "these bindings name keys that do not exist in docs/11: %s -- a "
        "source nobody can resolve is a declaration about nothing, and "
        "QueryBinding.source has no reader to notice" % missing)


def test_the_known_absent_list_is_reported_and_minimal():
    """The debt list is printed every run, and every entry is still absent.

    Two halves, and the second is the one that matters: if a key on the
    debt list LANDS in the contract, this case goes red and the entry has
    to come off. Without that, the list is a place where a fixed problem
    goes to be forgotten -- and the next reader cannot tell a real debt
    from a stale one.
    """
    text = _contract_text()
    print("declared-absent sources: %d" % len(_KNOWN_ABSENT))
    for src, why in sorted(_KNOWN_ABSENT.items()):
        print("  %-16s %s" % (src, why))
    # `in text` and not a parse: same reasoning as the case above, and
    # here it errs in the safe direction -- a key mentioned anywhere in
    # the contract is enough to require the debt entry be re-examined.
    landed = sorted(src for src in _KNOWN_ABSENT if src in text)
    assert not landed, (
        "these sources now EXIST in docs/11 and must be removed from "
        "_KNOWN_ABSENT: %s" % landed)
    # And every declared-absent source must actually be in use -- an entry
    # for a source no binding names is dead weight that outlives its
    # binding.
    in_use = {b.source for b in BINDINGS.values()}
    # Placeholders are not "in use" for this purpose, but they are also
    # never on the debt list, so the set difference below is unaffected.
    orphans = sorted(set(_KNOWN_ABSENT) - in_use)
    assert not orphans, (
        "_KNOWN_ABSENT names sources no binding uses: %s" % orphans)
