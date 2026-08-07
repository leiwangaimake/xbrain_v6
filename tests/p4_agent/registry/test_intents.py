"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_intents.py
Brief: GWY-P4-07 -- the intent registry loads clean, and every rule has a red mutation

Description:
This proves the five GWY-P4-07 criteria and, for each, the mutation that turns
it red (CLAUDE.md 3.3: an assertion never seen red is not written):

  1. ID-1 field presence  -- remove any of id/route/auth/slots -> refused,
     with the key path in the message.
  3. ID-3 direction slot  -- move_forward -> [direction, amount, unit] revives
     relative_move -> refused; and the PTZ pair E01/E06 keep their direction
     slot, proving the rule is scoped to the MI-1 set and is not a blanket ban.
  5. CS-A1 closed set     -- register a name not in 18 -> refused; and the
     mirror (drop a real name) is caught too, so the check is bidirectional.
(Criterion 2, ID-2, lives in test_geo_id.py.)

Two oracles, both COMMITTED, neither the gitignored _triage.json:
  * the migrated 16 S6.6 table is the route/slots + name truth source (U76(1),
    self-certifying). The registry must match it exactly, and CS-A1's closed set
    is its name set. If docs/temp/_triage.json is present it is cross-checked as
    a bonus, and skipped otherwise -- a test that ERRORS when a gitignored file
    is absent gets quietly excluded, worse than one that says it skipped.
  * the 16 S8.3A.2 registration table is the auth truth source. Its per-row
    levels are folded to per-id (H01/H03 default to quick/false) and compared to
    the registry's auth, so a fabricated level cannot pass.

Expected counts are DERIVED from those oracles, never written as literals here
(CLAUDE.md 3.7): the histograms are recomputed and compared, so a stale number
cannot hide in this file either.
"""

import copy      # deep copy so one mutation cannot leak into another test
import json      # only the bonus triage cross-check parses json
import os        # path joins and the gitignore-aware skip
import re        # the two doc tables are parsed by regex
import sys       # to put ROOT on the import path

# ROOT is four levels up: tests/p4_agent/registry/<this file>.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))     # /opt/xbrain_v6
sys.path.insert(0, ROOT)                              # so `import xbrain...` resolves

import pytest  # noqa: E402
import yaml    # noqa: E402   # the registry source is yaml

# The public surface under test, plus the closed sets the assertions compare to.
from xbrain.p4_agent.registry import (CONFIRM_LEVELS, MI1_MOTION_INTENTS,  # noqa: E402
                                      REQUIRED_FIELDS, ROUTES,
                                      IntentRegistryError,
                                      check_intents_in_closed_set,
                                      check_no_name_encoded_slot,
                                      load_intent_registry)

# The three on-disk artifacts this test reasons over. Reading configs/ and docs/
# directly is allowed in tests (no_config_source_read.py exempts tests/).
INTENTS_YAML = os.path.join(ROOT, "configs", "intents.yaml")    # registry under test
DOC16 = os.path.join(ROOT, "docs", "16-P4Agent管线详细设计.md")   # the two oracles live here
TRIAGE = os.path.join(ROOT, "docs", "temp", "_prompt_work", "_triage.json")  # gitignored bonus


# --- fixtures / oracles ------------------------------------------------------

def real_mapping():
    """The parsed configs/intents.yaml -- the registry under test.

    A fresh parse on every call, so a test that deep-copies and mutates the
    result cannot poison the structure another test reads. The cost of re-parsing
    a small yaml is nothing next to a cross-test coupling bug.
    """
    with open(INTENTS_YAML, encoding="utf-8") as fh:   # source read, tests exempt
        return yaml.safe_load(fh)                       # {intents: {name: {...}}}


def doc16_text():
    """Volume 16 as text, for the two table parsers below."""
    return open(DOC16, encoding="utf-8").read()          # whole册, read per call


# A migrated 16 S6.6 row looks like `| A01 | estop | bypass | -- |`; the `--`
# in the slots cell means no slots. Anchored so a stray line cannot half-match.
_ROW = re.compile(r"^\|\s*([A-J]\d{2})\s*\|\s*(\w+)\s*\|\s*(\w+)\s*\|\s*(.*?)\s*\|$")


def parse_doc16_registry_table(text):
    """{intent: {'id','route','slots'}} from the committed 16 S6.6 table.

    This is the self-certifying route/slots + name oracle. It is located by its
    unique header and parsed until the first non-row line, so a second table
    elsewhere with a different header cannot be picked up by accident. If the
    header is gone the migration is broken, and that must fail loudly rather than
    return an empty oracle that makes every downstream assertion vacuously pass.
    """
    lines = text.splitlines()                            # line-addressable
    start = None                                         # index of the first data row
    for i, ln in enumerate(lines):                       # scan for the header
        if ln.strip() == "| # | intent | route | slots |":   # the unique header
            start = i + 2                                # skip header + the |---| rule
            break                                        # first match wins
    assert start is not None, "16 S6.6 registry table header not found"  # loud, not soft
    out = {}                                             # intent -> fields
    for ln in lines[start:]:                             # walk the body
        m = _ROW.match(ln.strip())                       # one data row?
        if not m:                                        # first non-row line
            break                                        # ends the table
        iid, intent, route, slots_cell = m.groups()      # the four columns
        slots = [] if slots_cell.strip() == "--" else [  # `--` means empty
            s.strip() for s in slots_cell.split(",") if s.strip()]  # else comma-split
        out[intent] = {"id": iid, "route": route, "slots": slots}   # record row
    return out


# 16 S8.3A.2 auth table: five level rows carrying tokens `A05`, intervals
# `A01`-`A04`, and the split rows H01a/H01b/H03/H03f.
_LEVEL_ROW = re.compile(r"\|\s*[^|]*\*\*(L0|L1a|L1b|L2|L3)\*\*[^|]*\|[^|]*\|(.*)")
_TOKEN = re.compile(r"`([A-J]\d{2}[a-z]?)`")             # a single cmd_ref token
_INTERVAL = re.compile(r"`([A-J]\d{2})`[\u2013\u2014-]+`([A-J]\d{2})`")   # id interval, matches en/em dash + hyphen


def _expand(a, b):
    """Inclusive id interval A01..A04 -> [A01, A02, A03, A04]."""
    lo, hi = int(a[1:]), int(b[1:])                      # digits give the range
    assert a[0] == b[0] and lo <= hi                     # same class, ordered
    return ["%s%02d" % (a[0], n) for n in range(lo, hi + 1)]   # inclusive list


def parse_s8_3a_2_per_id_auth(text):
    """(row_auth, base_auth) from the committed 16 S8.3A.2 registration table.

    row_auth is the 130-row map (cmd_ref -> level, cmd_ref may be H01a/H03f).
    base_auth folds the split rows to their DEFAULT-slot level (16 S8.3A.2:
    scope default quick -> L1a, force_step default false -> L1b), which is
    exactly the per-id base auth the registry stores. Fabricating an auth value
    in the registry would then disagree with this fold and fail the test.
    """
    seg = text[text.find("#### 8.3A.2"):text.find("#### 8.3A.3")]   # just the subsection
    assert seg, "16 S8.3A.2 section not found"           # loud on a bad slice
    row_auth = {}                                        # cmd_ref -> level
    for line in seg.splitlines():                        # each table line
        m = _LEVEL_ROW.match(line)                       # only the five level rows
        if not m:                                        # everything else
            continue                                     # is not an auth row
        level, body = m.group(1), m.group(2)             # the level and its cell
        for a, b in _INTERVAL.findall(body):             # intervals first
            for x in _expand(a, b):                       # expand endpoints
                row_auth[x] = level                       # each id -> this level
        for tok in _TOKEN.findall(_INTERVAL.sub(" ", body)):   # then single tokens
            row_auth[tok] = level                         # (intervals blanked out)
    base = {}                                            # id -> base level
    default_ref = {"H01": "H01a", "H03": "H03"}          # the default-slot rows
    for ref, level in row_auth.items():                  # fold rows to ids
        b = re.match(r"([A-J]\d{2})", ref).group(1)      # strip any a/b/f suffix
        if b in default_ref:                             # split bases handled below
            continue                                     # skip them here
        base[b] = level                                  # ordinary id -> its level
    for b, ref in default_ref.items():                   # now the two split bases
        base[b] = row_auth[ref]                           # take the default row
    return row_auth, base


def cmdset_names():
    """The CS-A1 closed set: intent names from the committed 16 S6.6 table."""
    return set(parse_doc16_registry_table(doc16_text()))   # names only


def mutate(fn):
    """A deep copy of the real mapping with fn applied to its intents dict.

    Deep-copied so one mutation cannot reach another test through the shared
    parsed structure; fn edits the inner `intents` mapping in place.
    """
    m = copy.deepcopy(real_mapping())                    # private copy
    fn(m["intents"])                                     # apply the mutation
    return m


# --- green baseline ----------------------------------------------------------

def test_real_registry_loads_clean():
    """The real configs/intents.yaml passes every check and yields 128 entries.

    128 is asserted as len(the 16 S6.6 table), NOT a literal, so it cannot drift
    from the doc: if 18 adds an intent, both move together or this fails.
    """
    reg = load_intent_registry(real_mapping(), cmdset_names=cmdset_names())   # must not raise
    assert len(reg.names) == len(cmdset_names())         # registry size == doc set size


def test_cs_a1_bidirectional_diff_is_empty_today():
    """CS-A1 runs green today: registry names == the 16 S6.6 closed set, both
    directions. This is the state 16 S0.5 records as passing."""
    reg = load_intent_registry(real_mapping(), cmdset_names=cmdset_names())   # load it
    assert check_intents_in_closed_set(reg.names, cmdset_names()) == []   # no diff either way


def test_route_histogram_matches_the_doc_table():
    """Per-route counts recomputed from the registry equal those recomputed from
    the 16 S6.6 table. Neither side is a literal (CLAUDE.md 3.7)."""
    reg = load_intent_registry(real_mapping(), cmdset_names=cmdset_names())   # load
    doc = parse_doc16_registry_table(doc16_text())       # the oracle rows
    want = {r: 0 for r in ROUTES}                        # expected, seeded at zero
    for v in doc.values():                               # tally the doc rows
        want[v["route"]] += 1                            # one per row
    assert reg.route_histogram() == want                 # registry tally == doc tally


def test_registry_route_and_slots_match_16_s6_6():
    """Migration faithfulness: every registry entry's id/route/slots equals the
    committed 16 S6.6 row -- what makes 16 S6.6 the self-certifying source rather
    than a stale copy of one."""
    reg = load_intent_registry(real_mapping(), cmdset_names=cmdset_names())   # load
    doc = parse_doc16_registry_table(doc16_text())       # oracle
    assert set(reg.names) == set(doc)                    # same name set
    for intent, row in doc.items():                      # then per intent
        e = reg.by_name(intent)                          # the registry entry
        assert e.id == row["id"], intent                 # id agrees
        assert e.route == row["route"], intent           # route agrees
        assert list(e.slots) == row["slots"], intent     # slots agree, in order


def test_auth_matches_16_s8_3a_2():
    """Migration faithfulness for auth: the registry's per-id base level equals
    the folded 16 S8.3A.2 table, and that table's 130-row form is
    self-consistent. A fabricated level cannot survive this."""
    reg = load_intent_registry(real_mapping(), cmdset_names=cmdset_names())   # load
    row_auth, base = parse_s8_3a_2_per_id_auth(doc16_text())   # 134 rows + fold
    doc_ids = {v["id"] for v in parse_doc16_registry_table(doc16_text()).values()}  # S6.6 ids
    assert set(base) == doc_ids                          # fold covers exactly those ids
    # 134 = 132 intents + H01 split + H03 split. Was 130/128 until GWY-P4-23
    # landed D17/D18/E09/E10 (2026-08-07); CS-A3's declared histogram moved with it.
    assert len(row_auth) == 134
    for e in reg.entries:                                # every registry entry
        assert e.auth == base[e.id], (e.name, e.id)      # base auth == folded doc level


def test_split_intents_carry_auth_by_slot():
    """H01/H03 keep the slot-level table so the 130-row form is reconstructible
    from the registry alone, and its levels are validated (a typo refuses)."""
    reg = load_intent_registry(real_mapping(), cmdset_names=cmdset_names())   # load
    run_bit = reg.by_name("run_bit")                     # H01
    assert run_bit.auth_by_slot == {"scope": {"quick": "L1a", "deep": "L2"}}  # quick base, deep L2
    sts = reg.by_name("set_time_sync")                   # H03
    assert set(sts.auth_by_slot["force_step"].values()) == {"L1b", "L2"}  # false base, true L2


@pytest.mark.skipif(not os.path.exists(TRIAGE),
                    reason="docs/temp/_triage.json is gitignored; bonus cross-check only")
def test_bonus_cross_check_against_triage_when_present():
    """When the migration source is on disk, confirm the doc table STILL carries
    the migrated rows faithfully. Skipped (not failed) when absent, so CI without
    docs/temp stays green -- nothing at run time depends on this file (U76(1)).

    Subset, not equality, since 2026-08-07: _triage.json is FROZEN at the 128
    rows that were migrated, and the doc table has legitimately grown past it
    (GWY-P4-23 added D17/D18/E09/E10 directly to the self-certifying source;
    back-filling the _ file would recreate a dependency on it, which CLAUDE.md
    0.2 forbids). The claim this test still holds is migration faithfulness --
    every migrated row is present and unchanged -- plus the growth being exactly
    the intents that arrived after the migration, so an accidental deletion of a
    migrated row and an unexplained extra both still fail.
    """
    tr = json.load(open(TRIAGE))["triage"]               # the frozen migration source
    doc = parse_doc16_registry_table(doc16_text())       # the (growing) doc table
    migrated = {e["intent"] for e in tr}                 # the 128 migrated names
    assert migrated <= set(doc)                          # none of them vanished
    # Growth is enumerated, not open-ended: exactly the post-migration intents.
    assert set(doc) - migrated == {
        "set_light_bright", "set_strobe_mode",           # 18-A D17/D18
        "set_ptz_speed", "ptz_move_deg",                 # 18-B E09/E10
    }
    for e in tr:                                         # and per migrated record
        row = doc[e["intent"]]                            # the doc row
        assert row["id"] == e["id"]                       # id reproduced
        assert row["route"] == e["route"]                 # route reproduced
        assert row["slots"] == list(e["slots"])           # slots reproduced


# --- criterion 1 / ID-1: field presence, with the red mutation ---------------

@pytest.mark.parametrize("field", REQUIRED_FIELDS)
def test_mutation_missing_field_refuses_with_key_path(field):
    """Remove one required field from move_forward -> refused, key path named.

    Parametrised over all four so none is left uncovered; the message must carry
    intents.move_forward.<field>, because 'a field is missing' without the key
    is unactionable across 128 rows and the criterion says to print it.
    """
    m = mutate(lambda it: it["move_forward"].pop(field))   # drop one field
    with pytest.raises(IntentRegistryError) as ei:       # startup must refuse
        load_intent_registry(m, cmdset_names=cmdset_names())   # on the hole
    assert "intents.move_forward.%s" % field in str(ei.value)  # exact key path


def test_mutation_null_field_is_a_hole_too():
    """auth: null is a hole, same as absent -- 0/''/[] would NOT be (they are
    assigned values). A null here is an uncalibrated placeholder, and startup
    must refuse rather than run on it (CLAUDE.md 3.1)."""
    m = mutate(lambda it: it["move_forward"].__setitem__("auth", None))   # null it
    with pytest.raises(IntentRegistryError) as ei:       # refuse
        load_intent_registry(m, cmdset_names=cmdset_names())   # on the null
    assert "intents.move_forward.auth" in str(ei.value)  # key path named


def test_empty_slots_list_is_not_a_hole():
    """The bypass intents legitimately have slots: [] -- proving the hole test
    is `is None`, not `not v`. If [] were a hole, estop would refuse to load,
    the exact over-eager mistake the loader documents."""
    reg = load_intent_registry(real_mapping(), cmdset_names=cmdset_names())   # loads fine
    assert reg.by_name("estop").slots == ()              # empty, not a reported hole


# --- criteria 3+4 / ID-3: direction slot, with the red mutation --------------

def test_mutation_direction_slot_revives_relative_move():
    """criterion 4: move_forward -> [direction, amount, unit] is refused (ID-3).

    This is the relative_move{direction,...} shape MI-1 deleted. The message
    names the intent and the offending slot, so the fix is obvious.
    """
    m = mutate(lambda it: it["move_forward"].__setitem__(   # give it a direction slot
        "slots", ["direction", "amount", "unit"]))
    with pytest.raises(IntentRegistryError) as ei:       # ID-3 refuses
        load_intent_registry(m, cmdset_names=cmdset_names())
    msg = str(ei.value)                                  # the refusal text
    assert "move_forward" in msg and "direction" in msg  # both are named


def test_id3_is_scoped_not_blanket_ptz_keeps_direction():
    """The real registry passes ID-3 even though E01/E06 carry a direction slot.

    This is the load-bearing counter-case: a blanket 'no direction slot' ban
    would reject ptz_move/ptz_zoom and the whole registry would fail to load.
    ID-3 is scoped to MI1_MOTION_INTENTS, and PTZ is outside it.
    """
    raw = real_mapping()["intents"]                      # the real entries
    assert "direction" in raw["ptz_move"]["slots"]       # guard: E01 really has it
    assert "direction" in raw["ptz_zoom"]["slots"]       # guard: E06 really has it
    assert "ptz_move" not in MI1_MOTION_INTENTS          # and PTZ is out of scope
    assert check_no_name_encoded_slot(raw) == []         # so ID-3 finds nothing


def test_id3_fires_for_every_motion_intent():
    """Each of the eight MI-1 intents rejects a direction slot, not just
    move_forward -- a rule guarding only one would let the same defect in through
    any of the other seven."""
    for name in sorted(MI1_MOTION_INTENTS):              # all eight
        m = mutate(lambda it, n=name: it[n].__setitem__(  # append direction
            "slots", list(it[n]["slots"]) + ["direction"]))
        with pytest.raises(IntentRegistryError) as ei:   # each refuses
            load_intent_registry(m, cmdset_names=cmdset_names())
        assert name in str(ei.value)                     # naming the intent


# --- criterion 5 / CS-A1: name closed set, with the red mutation -------------

def test_mutation_non_18_intent_fails_cs_a1():
    """criterion 5: registering a name not in 18 is refused.

    fly_to_moon is not in the 18 closed set; the loader, given the closed set as
    oracle, reports it in only_in_registry. The fake entry uses a well-FORMED id
    (J99) that is simply not a real 18 id, so the ONLY thing wrong is the off-set
    name: the refusal is CS-A1's, not the id-shape check firing first.
    """
    m = mutate(lambda it: it.__setitem__(                # add a coined intent
        "fly_to_moon", {"id": "J99", "route": "fastpath", "auth": "L0", "slots": []}))
    with pytest.raises(IntentRegistryError) as ei:       # CS-A1 refuses
        load_intent_registry(m, cmdset_names=cmdset_names())
    assert "fly_to_moon" in str(ei.value) and "CS-A1" in str(ei.value)  # rule + name


def test_cs_a1_is_bidirectional_dropping_a_name_is_caught():
    """The mirror direction: a name in the 18 set but absent from the registry
    is also refused. A subset test would pass this broken file; CS-A1 is a
    two-way diff (16 S0.5), and this is the half people forget."""
    reg_names = set(real_mapping()["intents"])           # the real names
    reg_names.discard("move_forward")                    # drop one real intent
    problems = check_intents_in_closed_set(reg_names, cmdset_names())   # diff vs 18
    assert problems and any("move_forward" in p for p in problems)   # caught on the other side


def test_cs_a1_not_enforced_without_an_oracle():
    """With cmdset_names=None the loader does NOT invent a closed set: a coined
    name loads. A documented deferral, not a silent pass -- the runtime must pass
    the cmdset once configs/cmdset_18.json (GWY-P4-09) exists. The entry is
    otherwise well-formed, so CS-A1 is the only check that would object, and with
    no oracle it does not run."""
    m = mutate(lambda it: it.__setitem__(                # coined but well-formed
        "fly_to_moon", {"id": "J99", "route": "fastpath", "auth": "L0", "slots": []}))
    reg = load_intent_registry(m, cmdset_names=None)     # no oracle -> CS-A1 skipped
    assert "fly_to_moon" in reg.names                    # so it loads


# --- closed-set robustness (route / auth / auth_by_slot) ---------------------

def test_bad_route_value_is_refused():
    """A route outside the four-value set is refused, not passed through
    (11 S13.6). 'patrol' is a plausible-looking wrong value."""
    m = mutate(lambda it: it["move_forward"].__setitem__("route", "patrol"))   # bad route
    with pytest.raises(IntentRegistryError) as ei:       # refuse
        load_intent_registry(m, cmdset_names=cmdset_names())
    assert "route" in str(ei.value)                      # the field is named


def test_bad_auth_level_is_refused():
    """An auth outside CONFIRM_LEVELS is refused. L4 does not exist."""
    m = mutate(lambda it: it["move_forward"].__setitem__("auth", "L4"))   # bad level
    with pytest.raises(IntentRegistryError) as ei:       # refuse
        load_intent_registry(m, cmdset_names=cmdset_names())
    assert "auth" in str(ei.value)                       # the field is named


def test_bad_auth_by_slot_level_is_refused():
    """A typo in a split intent's slot-level table refuses startup, rather than
    surfacing later as a wrong confirmation level at 16 S8.3A.2."""
    def break_it(it):                                    # L9 is not a level
        it["run_bit"]["auth_by_slot"] = {"scope": {"quick": "L1a", "deep": "L9"}}
    m = mutate(break_it)                                 # apply it
    with pytest.raises(IntentRegistryError) as ei:       # refuse
        load_intent_registry(m, cmdset_names=cmdset_names())
    assert "auth_by_slot" in str(ei.value)               # the nested path is named


def test_confirm_levels_and_routes_are_the_documented_sets():
    """Guard the local closed sets against silent edits: their contents are the
    16 S0.3.1 / S6.6 values, in the documented order (messages print them so)."""
    assert CONFIRM_LEVELS == ("L0", "L1a", "L1b", "L2", "L3")   # 16 S0.3.1
    assert ROUTES == ("bypass", "fastpath", "fastpath_then_llm", "llm")   # 16 S6.6
