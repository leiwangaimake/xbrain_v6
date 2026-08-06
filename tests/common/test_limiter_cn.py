"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_limiter_cn.py
Brief: RS-LIM startup assertion and the limiter_cn value binding, CFG-CM-6

Description:
What this guards. xbrain/common/enums exports LIMITER_CN, the gate.limiter ->
Chinese broadcast wording, and assert_limiter_cn_matches_gate_limiter, the RS-LIM
startup check (16 S8.3B.3). RS-LIM refuses to start unless the map's key set and
the 11 S3.4 gate.limiter closed set are equal in BOTH directions. This file is
the acceptance suite for that item, and it is two suites in one:

  1. The RS-LIM key check itself, with the two named mutations that must turn it
     red -- a new closed-set value with no Chinese, and a Chinese key with no
     closed-set value -- plus the guard that a one-way containment check would
     have missed the first of those. That miss is not hypothetical: heading and
     clock were exactly this gap until 99 U76(6) supplied their wording, and
     16 S8.3B.3 was written to keep the check red until they did.

  2. The VALUE binding, which the RS-LIM key check does not cover. RS-LIM proves
     every limiter has SOME Chinese; it says nothing about whether the Chinese is
     the RIGHT Chinese. So, in the same shape test_closed_sets.py binds sets.yaml
     back to 11, this binds every LIMITER_CN value back to the doc that authored
     it: the twelve non-heading/clock values to 16 S8.3, and heading/clock to
     99 U76(6). A hand transcription that nothing checks against its source is
     the defect this whole package exists to prevent.

Why the doc anchors here are content, not line numbers (NUM-4). The docs shift
under edits -- the 16 map row moved by tens of lines during this item's own
authoring -- so every extractor keys off a greppable string in the row, and each
one raises rather than returning an empty result when its anchor is gone. An
extractor that silently finds nothing would make the binding pass by comparing
two empty maps, which is CLAUDE.md 3.2 form 1.

Not-obvious separator handling. The 16 map is written key -> value with a real
arrow and CJK corner brackets; those characters are built by codepoint here
rather than typed, both so charset_lint has nothing to report and so this file
does not carry the very glyphs it reads out of the document.
"""

import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common import enums  # noqa: E402
from xbrain.common.errors import E_CONFIG_INVALID, XbrainError  # noqa: E402

# The two authoring documents, read whole. 16 owns the twelve everyday values;
# 99 owns heading and clock. Kept as separate reads because they are two
# different authorities and a conflict between them is a real finding, not a
# parse error to smooth over.
DOC16 = os.path.join(ROOT, "docs", "16-P4Agent管线详细设计.md")
DOC99 = os.path.join(ROOT, "docs", "99-决策记录.md")

# Separators built by codepoint, never typed. U+2192 is the -> arrow, U+300C and
# U+300D are the CJK corner brackets the 16 row wraps each value in. Typing them
# would make this test file carry characters charset_lint flags in source, and
# would make a reader of THIS file believe the glyphs belong to it rather than to
# the document it is quoting.
_ARROW = chr(0x2192)
_LBRACKET = chr(0x300c)
_RBRACKET = chr(0x300d)

# `key` -> [value]. The value is everything up to the closing bracket, so a
# value is taken verbatim and a missing bracket yields no match rather than a
# run-on capture. The key class is lower-case identifiers, the shape every
# gate.limiter value has.
_PAIR_RE = re.compile(
    "`([a-z_]+)`\\s*" + re.escape(_ARROW) + "\\s*"
    + re.escape(_LBRACKET) + "([^" + re.escape(_RBRACKET) + "]+)" + re.escape(_RBRACKET)
)

# heading = <hanzi> / clock = <hanzi>, the form 99 U76(6) writes the ruling in.
# The character class is the CJK ideograph block written as escapes, so the
# pattern is ASCII in the source while still matching Chinese words. It requires
# both halves on one line, which is what makes the match unambiguous -- "heading
# =" alone occurs in prose elsewhere in 99.
_HEADING_CLOCK_RE = re.compile(
    r"heading\s*=\s*([一-鿿]+)\s*/\s*clock\s*=\s*([一-鿿]+)"
)


def _doc16_inline_map():
    """The {limiter: Chinese} map inline in 16 S8.3, or raise if the row is gone.

    Keyed off a row that carries free_space, none and both bracket/arrow glyphs,
    so a table restatement elsewhere in 16 that lacks the full map is not
    mistaken for it. If more than one row qualifies the anchor has become
    ambiguous and a human must look, so that raises too rather than guessing
    which copy is current -- the same stance test_closed_sets.py takes on 11's
    duplicated tables.
    """
    lines = open(DOC16, encoding="utf-8").read().split("\n")
    rows = [ln for ln in lines
            if "free_space" in ln and "none" in ln
            and _ARROW in ln and _LBRACKET in ln]
    if not rows:
        raise AssertionError(
            "16 S8.3 inline limiter map row not found -- its wording or glyphs "
            "changed; re-derive the anchor before trusting this suite")
    if len(rows) > 1:
        raise AssertionError(
            "16 S8.3 inline limiter map matched more than one row; the anchor is "
            "ambiguous, check which is the current definition")
    pairs = dict(_PAIR_RE.findall(rows[0]))
    assert pairs, "16 S8.3 map row matched but no `key` -> value pairs parsed"
    return pairs


def _doc99_heading_clock():
    """(heading_cn, clock_cn) from 99 U76(6), or raise if the ruling is gone.

    Exactly one line must carry the paired form; zero means the ruling moved or
    was reworded, more than one means the anchor no longer identifies it. Either
    way the right outcome is a loud stop, because the value the library ships for
    these two must trace to a ruling and not to this test's memory of one.
    """
    lines = open(DOC99, encoding="utf-8").read().split("\n")
    hits = [m.groups() for ln in lines for m in [_HEADING_CLOCK_RE.search(ln)] if m]
    if len(hits) != 1:
        raise AssertionError(
            "99 U76(6) heading/clock ruling did not match exactly one line "
            f"(matched {len(hits)}); the anchor 'heading = ... / clock = ...' "
            "moved or was reworded")
    return hits[0]


def _doc_limiter_map():
    """The full authored map: 16 S8.3 for the twelve, 99 U76(6) for heading/clock.

    heading and clock are added from the ruling. Should 16 ever take the two
    values over (Q-P4-29), the row would already carry them; in that case the two
    sources MUST agree, and a disagreement is surfaced here rather than silently
    preferring one -- two documents defining the same value differently is the
    kind of split this project treats as a finding.
    """
    m = _doc16_inline_map()
    heading_cn, clock_cn = _doc99_heading_clock()
    for key, ruled in (("heading", heading_cn), ("clock", clock_cn)):
        if key in m:
            assert m[key] == ruled, (
                f"16 S8.3 and 99 U76(6) disagree on {key}: {m[key]!r} vs "
                f"{ruled!r} -- resolve the split in the documents first")
        else:
            m[key] = ruled
    return m


# ---------------------------------------------------------------------------
# RS-LIM, the key-set check. 16 S8.3B.3.
# ---------------------------------------------------------------------------

def test_rs_lim_passes_for_the_shipped_pair():
    """The positive half, without which "raises on everything" would pass.

    CLAUDE.md 3.2 form 1: an assertion that always raised would satisfy every
    mutation case below and be catastrophically wrong -- the module would not
    import. The shipped map and closed set are equal both ways, so the assertion
    returns None. That it ALSO runs at import means this test reaching the call
    at all already proves the module loaded with the pair consistent.
    """
    assert enums.assert_limiter_cn_matches_gate_limiter(
        enums.LIMITER_CN.keys(), enums.GATE_LIMITER.values) is None


def test_limiter_cn_covers_exactly_the_gate_limiter_set():
    """The invariant RS-LIM guarantees, stated directly. Both directions.

    Set equality rather than one-way containment on purpose: this is the property
    16 S8.3B.3 demands and the one the mutation cases below prove the assertion
    actually enforces.
    """
    assert set(enums.LIMITER_CN) == set(enums.GATE_LIMITER.values)


def test_forward_mutation_a_new_limiter_without_chinese_goes_red():
    """*** 16 S8.3B.3 variant one, run rather than described.

    Inject a value into the 11 S3.4 closed set and add no Chinese for it: RS-LIM
    must refuse to start. power_cut is a plausible future limiter, chosen so it
    cannot collide with a real member. This is the case the TODO row names as the
    mutation, and the one a one-way "keys subset of the set" check cannot see.
    """
    injected = list(enums.GATE_LIMITER.values) + ["power_cut"]
    assert "power_cut" not in enums.LIMITER_CN, "probe must not be a real key"
    with pytest.raises(XbrainError) as caught:
        enums.assert_limiter_cn_matches_gate_limiter(
            enums.LIMITER_CN.keys(), injected)
    assert caught.value.code == E_CONFIG_INVALID
    # Named on the correct side of the diff: the value with no Chinese, not the
    # other list. A message that only said "differ" would not tell the operator
    # which side to fix.
    assert "power_cut" in str(caught.value)


def test_reverse_mutation_a_chinese_key_outside_the_set_goes_red():
    """*** 16 S8.3B.3 variant two -- the half a one-way check omits by design.

    A Chinese key for a value the closed set does not define: a typo, or wording
    left behind after a value was removed from 11 S3.4. The section says in so
    many words that this direction must also go red, and that both must pass for
    the item to count as implemented.
    """
    ghost = set(enums.LIMITER_CN) | {"ghost_limiter"}
    assert "ghost_limiter" not in set(enums.GATE_LIMITER.values), (
        "probe must not be a real limiter value")
    with pytest.raises(XbrainError) as caught:
        enums.assert_limiter_cn_matches_gate_limiter(
            ghost, enums.GATE_LIMITER.values)
    assert caught.value.code == E_CONFIG_INVALID
    assert "ghost_limiter" in str(caught.value)


def test_one_way_inclusion_would_miss_the_missing_chinese_case():
    """*** Pins "must not be implemented as one-way containment" (16 S8.3B.3).

    This models the exact historical gap: the map lacks heading and clock while
    the closed set has them -- the live state before 99 U76(6) supplied the two
    words. A check written as "map keys are a subset of the closed set" is
    SATISFIED by that crippled map, stays green, and ships a robot that cannot
    say why heading or clock slowed it down. RS-LIM subtracts the other way as
    well and refuses to start. Both facts are asserted here, so the test fails if
    anyone ever relaxes RS-LIM into the one-way form it forbids.
    """
    crippled = set(enums.LIMITER_CN) - {"heading", "clock"}
    full = list(enums.GATE_LIMITER.values)
    # One-way containment holds -- the crippled map IS a subset -- so the weaker
    # check would pass. This line is the proof that the weaker check is unsafe,
    # not a requirement on our code.
    assert crippled <= set(full)
    # The bidirectional check does not pass, and it names the two values the weak
    # check would have dropped.
    with pytest.raises(XbrainError) as caught:
        enums.assert_limiter_cn_matches_gate_limiter(crippled, full)
    assert caught.value.code == E_CONFIG_INVALID
    assert "heading" in str(caught.value) and "clock" in str(caught.value)


def test_rs_lim_raises_the_config_refusal_code():
    """The code identity, so a caller branching on it treats RS-LIM like the rest
    of the config-refusal family (a bad reference, a failed schema check). It is
    E_CONFIG_INVALID, group L, retryable no -- not a generic internal error that
    would send the reader looking in the wrong place."""
    with pytest.raises(XbrainError) as caught:
        enums.assert_limiter_cn_matches_gate_limiter(
            enums.LIMITER_CN.keys(), list(enums.GATE_LIMITER.values) + ["x_extra"])
    assert caught.value.code == E_CONFIG_INVALID


# ---------------------------------------------------------------------------
# The map object itself.
# ---------------------------------------------------------------------------

def test_limiter_cn_is_immutable():
    """A process-wide singleton must be read-only.

    Every importer shares this one object, so a writable map would let a single
    module rewrite a broadcast reason for all the others -- the failure mode
    ClosedSet.values avoids by handing back a tuple. Both a write and a delete
    must raise; checking only the write would leave del as a hole.
    """
    with pytest.raises(TypeError):
        enums.LIMITER_CN["estop"] = "x"        # type: ignore[index]
    with pytest.raises(TypeError):
        del enums.LIMITER_CN["estop"]           # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# The VALUE binding. RS-LIM checks the keys; this checks the words.
# ---------------------------------------------------------------------------

def test_every_chinese_value_matches_its_authoring_doc():
    """*** The binding RS-LIM does not provide: right keys is not right words.

    Same shape as test_closed_sets.py: the library and the documents are written
    separately and this is what keeps them equal. The twelve everyday values come
    from 16 S8.3, heading and clock from 99 U76(6); the merged map must equal
    LIMITER_CN exactly, key and value.
    """
    doc = _doc_limiter_map()
    lib = dict(enums.LIMITER_CN)
    from_docs = sorted((k, doc[k]) for k in doc if doc.get(k) != lib.get(k))
    from_lib = sorted((k, lib[k]) for k in lib if lib.get(k) != doc.get(k))
    assert doc == lib, (
        "LIMITER_CN and its authoring docs disagree. In the docs but not "
        f"matching the library: {from_docs}; in the library but not matching the "
        f"docs: {from_lib}. 16 S8.3 owns the twelve, 99 U76(6) owns heading and "
        "clock -- correct the side that is wrong, do not edit this test to agree")


def test_the_doc_extractors_are_not_silently_empty():
    """Pairs with the binding above, aimed at the silent-empty failure.

    If either extractor found nothing the binding would fail with a confusing
    message, or -- were both sides empty -- pass on a comparison of two empty
    maps. These anchors are the stable ones: three values that have been in
    16 S8.3 since it was written, and the two the ruling supplied.
    """
    doc = _doc_limiter_map()
    for from16 in ("free_space", "none", "estop"):
        assert doc.get(from16), f"16 S8.3 extractor produced no value for {from16}"
    for from99 in ("heading", "clock"):
        assert doc.get(from99), f"99 U76(6) extractor produced no value for {from99}"


def test_a_wrong_chinese_value_would_be_caught_by_the_binding():
    """*** The mutation for the binding, performed rather than described.

    A copy of the map with one value corrupted must stop equalling the docs, or
    the binding could not detect a transcription error. Run against a copy -- the
    shipped map is immutable anyway -- so no tracked state is touched and an
    interrupted run leaves nothing broken.
    """
    doc = _doc_limiter_map()
    mutated = dict(enums.LIMITER_CN)
    # A one-character corruption, the realistic typo. It must break the equality
    # the case above relies on.
    mutated["free_space"] = mutated["free_space"] + "X"
    assert mutated != doc, (
        "a corrupted Chinese value still matched the docs; the binding cannot "
        "detect a transcription error and the case above is vacuous")


def test_heading_and_clock_trace_to_the_ruling_not_to_this_file():
    """The two values 16 S8.3B.3 left red until a ruling filled them.

    99 U76(6) is that ruling. Asserting the library value equals the ruling
    verbatim means a later well-meaning reword of either the library or the
    ruling, without the other, is caught here rather than shipping two spellings.
    """
    heading_cn, clock_cn = _doc99_heading_clock()
    assert enums.LIMITER_CN["heading"] == heading_cn
    assert enums.LIMITER_CN["clock"] == clock_cn
