"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_qos_resolve.py
Brief: INF-ZN-3 -- the 11 S2.4.7 bindings resolved, vector by vector, plus its
       four mutants

Description:
What these cases are worth. Every QoS defect this project has already found was
INVISIBLE in operation. 11 S2.4.7 records the v0.3 repair in its own comment:
rt/safety/estop and probe/estop/{ping,pong} had fallen through into the rt/**
and xbrain/*/** fallbacks, so the emergency-stop path lost real_time and express
on one plane and picked up block on the other. Nothing stopped working. Estop
still arrived, from a lower-priority queue and behind a wait with no upper bound,
and 11 S2.2.12 calls that the most serious row its own audit turned up. There is
no run-time symptom to assert on, so the resolution itself is the test.

Why the vectors live in a JSON file and not in this module. Two reasons, and the
second is the important one. The file is also the fixture the mutants are applied
to, so the document under test and the expectations about it stay side by side.
And the expectations are derived from the bindings array by hand -- if they were
computed from the module under test they would agree with it by construction,
which is CLAUDE.md 3.2 form 7: a claim that cannot have a counterexample.

*** What holds the fixture honest. test_fixture_bindings_are_verbatim reads the
json5 block out of 11 and requires the fixture's twenty-five bindings to appear
in it, in order, with their set clauses spelled the same way; the profile table
and rt_override get the same treatment. Without that, this file would be testing
the resolver against a copy of the bindings that had quietly drifted from the
contract, and every case would still be green.

*** What these cases do NOT establish, stated so a green run is not read as more
than it is:
  * Nothing here opens a Zenoh session or publishes anything. The knobs are
    asserted as values; that Zenoh applies them as described is QoS-T1 to
    QoS-T8 in 11 S2.4.9, all of which are still pending T7.
  * Nothing here checks that a key is registered in the S2.2 key table. A
    well-formed unregistered key resolves happily; INF-ZN-4 owns that.
  * Nothing here checks a process's declarations against the S2.4.8 anti-pattern
    table. That is INF-ZN-5 and INF-ZN-6.
  * The key-expression matcher is cross-checked against zenoh.KeyExpr only where
    eclipse-zenoh is installed. Where it is not, the skip says which claim went
    unverified, because a skipped cross-check and a passing one look the same in
    a summary line and this project has been caught reading one as the other.

The four mutants the item's criterion names are cases in this file rather than
edits somebody has to remember to make. Each builds a mutated document, then
runs the SAME assertion helper the green case uses and requires it to fail --
which is the mechanical form of CLAUDE.md 3.3: an assertion that has never been
red has not been written.

Beyond those four, the following edits were injected into the implementation and
each was confirmed to turn this suite red before being reverted. They are listed
because a reader deciding whether to trust these cases needs to know which
defects they were actually shown to catch, not which ones they were meant to:
  * resolve() falling back to the last binding instead of raising;
  * parse_full_key() dropping its root check;
  * the loader consuming rt_override from the document instead of the constant;
  * require_handler_depth() returning 0 instead of raising;
  * a single * allowed to cross a separator, as fnmatch would have it;
  * last-match-wins instead of first-match-wins;
  * one knob flipped in the frozen table;
  * QOS-C1 never applied;
  * the frozen-profile audit removed;
  * two separate edits to the C++ table, caught by the cross-language module.
One injected edit did NOT turn the suite red on the first attempt: removing
reliability from the pair of forbidden set-clause fields, which then fell through
to the unknown-field branch and still raised. That is a real gap in the
assertion, not in the implementation -- the two failures need different messages
because one is a misspelling and the other is a safety prohibition -- so
test_mutant_2 now asserts on which of the two fired.
"""

import copy
import json
import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

from xbrain.common.config import MISSING  # noqa: E402
from xbrain.common.errors import E_CONFIG_INVALID, E_QOS_VIOLATION  # noqa: E402
from xbrain.common.errors.exceptions import ClosedSetViolation  # noqa: E402
from xbrain.common.zenoh.qos import (FROZEN_PROFILES,  # noqa: E402
                                     RT_OVERRIDE, QosConfigError, QosViolation,
                                     key_expr_matches, load_qos_table,
                                     parse_full_key)

#: The fixture: the S2.4.7 document plus one expectation per golden key.
GOLDEN_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "golden",
                           "qos_bindings_vectors.json")
# Loaded at import rather than in a fixture, because several parametrize lists
# below are built from it and pytest evaluates those at collection time. A
# fixture would be available too late and the ids would have to be written out
# by hand -- which is one more copy of the key list to keep in step.
with open(GOLDEN_PATH, encoding="utf-8") as handle:
    GOLDEN = json.load(handle)

#: The S2.4.7 document -- profiles, the ordered bindings, rt_override -- and the
#: hand-derived expectation for each golden key. Split into two names so that
#: every mutant below reads as "edit the document, re-check the vectors", which
#: is the shape the item's criterion is written in.
DOCUMENT = GOLDEN["qos"]
VECTORS = GOLDEN["vectors"]

#: The contract itself, read by the three transcription cases. They are the only
#: thing standing between this file and a fixture that has quietly drifted, so
#: the path is a module constant rather than repeated in each of them.
CONTRACT = os.path.join(ROOT, "docs", "11-接口契约.md")

#: eclipse-zenoh is only needed by the cross-check. Imported the same way the
#: session-factory tests do it -- a named exception, never importorskip, so the
#: rest of the module keeps running where the client is absent. importorskip
#: would take the nineteen golden vectors down with it for a reason that has
#: nothing to do with them.
try:
    import zenoh  # noqa: E402
except ImportError:
    zenoh = None


# build and mutated are the only two ways a case gets a table. Keeping them
# together means a mutant cannot accidentally build from the unmutated document
# -- which would pass, silently, and prove nothing at all.
def build(document=None):
    """A QosTable over the fixture, or over a mutated copy of it.

    The default argument is the shared document, not a copy: the green cases
    must run against the fixture exactly as it sits on disk, and a copy would
    hide an accidental mutation by an earlier case.
    """
    return load_qos_table(DOCUMENT if document is None else document)


def mutated():
    """A deep copy of the fixture document, safe to edit.

    Deep, not shallow: the bindings are dicts inside a list, and a shallow copy
    would let one mutant's edit reach the next case through the shared entries.
    A test suite whose cases contaminate each other in order is the worst kind
    to debug, because it passes when the cases are run one at a time.

    Nothing here writes the fixture back to disk, and nothing should. The file is
    the transcription of 11 S2.4.7; a mutant that persisted its edit would leave
    every later run testing a contract that does not exist, and the
    transcription cases would be the only ones to notice.
    """
    return copy.deepcopy(DOCUMENT)


# check_vector -- the one assertion the green case and all four mutants share.
#
# It is a helper rather than an inlined block precisely so the mutants can drive
# it. A mutant that asserted something of its own would prove that the mutated
# document behaves differently; it would NOT prove that the golden case goes red,
# which is what CLAUDE.md 3.3 asks for.
#
# Every assert carries the key as its message. Without that, a parametrised
# failure reports a line number inside this helper and the reader has to go back
# to the test id to find out which of nineteen keys moved.
def check_vector(table, vector):
    """Assert one golden vector against a table. Raises AssertionError if not."""
    got = table.resolve(vector["key"])
    # The binding identity is asserted first and separately. Two bindings that
    # name the same profile give identical knobs, so a resolver that picked the
    # wrong one would pass every knob comparison below -- and picking the wrong
    # one is exactly the v0.3 defect this file exists for. rt/chassis/state is
    # covered by two rules that both give Q2_state, so for that key the index is
    # the only thing that can tell the two apart.
    assert got.binding_index == vector["binding_index"], vector["key"]
    assert got.binding_match == vector["binding_match"], vector["key"]
    # The profile name is the row BEFORE any set clause or override, which is
    # why it is asserted alongside the knobs rather than instead of them.
    assert got.profile_name == vector["profile"], vector["key"]
    # The five knobs, each asserted separately rather than as one tuple
    # comparison: a tuple failure prints both tuples and leaves the reader to
    # diff them by eye, which is exactly the moment a second difference gets
    # missed.
    assert got.congestion_control == vector["congestion_control"], vector["key"]
    assert got.priority == vector["priority"], vector["key"]
    assert got.reliability == vector["reliability"], vector["key"]
    # Identity against the bool, not a truthiness comparison: express is the one
    # field where a 0 or a "false" would compare equal under == and serialise
    # into json5 as something that is not a boolean at all.
    assert got.express is vector["express"], vector["key"]
    assert got.handler.kind == vector["handler"]["kind"], vector["key"]
    want_depth = vector["handler"]["depth"]
    if want_depth is None:
        # null in the fixture means the deployment has not supplied this depth.
        # MISSING and not None on our side, because None is a value a document
        # can legitimately carry and MISSING is not; see xbrain/common/config.
        # Comparing with == would pass for a resolver that returned None, which
        # is the state this project keeps closing.
        assert got.handler.depth is MISSING, vector["key"]
    else:
        assert got.handler.depth == want_depth, vector["key"]
    # Whether QOS-C1 fired is asserted for every key, not only the one it
    # applies to. An override that fired somewhere it should not would otherwise
    # be invisible: on a Q1 key it would change nothing observable, and it would
    # sit there until a profile changed underneath it.
    assert got.rt_override_applied is vector["rt_override_applied"], vector["key"]


# ---------------------------------------------------------------------------
# The golden vectors.
#
# Nineteen keys, chosen by the item's criterion rather than by this file, and
# they are not a sample: between them they exercise every distinct shape a
# binding can have. Six land on the safety group at the front. Five land on a
# rule carrying a set clause, which is the only way priority or depth can differ
# from the profile. Two land on a rule that is neither the first nor the last
# that could have matched, and those two are the reason binding_index is part of
# the result at all. One lands on the fallback, one on the plane-wide rt/**
# fallback and is then rewritten by QOS-C1, and one carries a depth the
# deployment has not supplied.
#
# A vector table that covered only the obvious keys would be green against a
# resolver with no ordering at all, since most keys have exactly one candidate
# rule.
# ---------------------------------------------------------------------------

# Parametrised by key so a failure names the key in the test id. A single case
# looping over the vectors would stop at the first failure and report one key,
# leaving the reader to find out whether the rest also moved -- and after a
# binding reorder, most of them will have.
@pytest.mark.parametrize("vector", VECTORS, ids=[v["key"] for v in VECTORS])
def test_golden_vector(vector):
    """Each key resolves exactly as the 11 S2.4.7 bindings say it must.

    What a red here means, in decreasing order of likelihood: a binding was
    reordered, a set clause was edited, the frozen table changed, or the matcher
    stopped agreeing with Zenoh. The first three are document edits and the
    transcription cases below will be red as well; if this case is red and those
    are green, the resolver is what changed.

    Nineteen cases and not one, because the item's criterion lists the coverage
    key by key and a single looping case would report only the first one to move.
    """
    check_vector(build(), vector)


def test_every_binding_the_criterion_calls_non_obvious_is_covered():
    """The two keys whose resolution is not guessable are both in the fixture.

    rt/chassis/state matches TWO rules -- rt/**/state and rt/chassis/** -- and
    the first one written wins, which is the only reason it is Q2 and not
    something a reader would derive from the key's own name. rt/behavior/request
    matches no specific rule at all and reaches Q3 through the rt/** fallback,
    then gets rewritten by QOS-C1.

    Both are the cases a hand-written vector table is most likely to omit,
    precisely because they are the ones whose answer nobody is sure of. Asserting
    their presence stops a later tidy-up from removing them as duplicates of the
    keys that resolve the obvious way.
    """
    keys = {v["key"] for v in VECTORS}
    assert "xbrain/dog-01/rt/chassis/state" in keys
    assert "xbrain/dog-01/rt/behavior/request" in keys


# A property over the whole vector table rather than one more per-key
# expectation. The per-key vectors already pin each rt key's congestion control,
# so this adds nothing to a green run -- what it adds is a red run that names the
# RULE instead of a key, which is the difference between "rt/behavior/request
# moved" and "QOS-C1 is not being applied".
def test_qos_c1_holds_for_every_rt_key():
    """No key on the RT plane resolves to block. 11 S2.4.3 QOS-C1.

    Stated over the resolutions rather than over the bindings, because block can
    arrive two ways: from a profile that carries it, and from a binding that
    names such a profile. QOS-C1 forbids the outcome, not the route.

    This is the case that goes red if the override is ever made conditional --
    on a flag, on a profile name, on anything a deployment can reach. It was
    confirmed red against an implementation with the override disabled.
    """
    table = build()
    rt_keys = [v["key"] for v in VECTORS if v["key"].split("/")[2] == "rt"]
    # An empty list would make the loop below vacuously true, which is CLAUDE.md
    # 3.2 form 1 -- an assertion a do-nothing fixture passes. The guard costs one
    # line and removes the whole failure mode.
    assert rt_keys
    for key in rt_keys:
        assert table.resolve(key).congestion_control == "drop", key


# ---------------------------------------------------------------------------
# The fixture is held to the contract.
#
# These three cases are the load-bearing ones in this file, and they are the only
# ones that read 11 rather than the fixture. Everything above asserts that the
# resolver agrees with the fixture; only these assert that the fixture agrees
# with the contract. Delete them and the suite becomes a very thorough check that
# this project agrees with itself.
#
# They grep rather than parse. A json5 parser for a block embedded in markdown
# with prose around it is a second thing to keep correct, and its failure mode --
# silently matching nothing -- is the one that leaves the whole check vacuous.
# The regexes below anchor on punctuation the contract writes verbatim, and each
# one asserts a count or an ordered equality so that matching nothing fails.
# ---------------------------------------------------------------------------

def read_contract():
    """The whole contract as text.

    Read on every call rather than cached at import. It is a few megabytes and
    three cases use it, so the cost is irrelevant; the benefit is that no case
    can be affected by an earlier one having read a different version of the
    file, which matters while the document is still being edited.
    """
    with open(CONTRACT, encoding="utf-8") as handle:
        return handle.read()


def bindings_block():
    """Just the bindings array out of the S2.4.7 json5 block.

    Bounded on purpose. The same match expressions appear elsewhere in 11 as
    prose examples -- S8.13 quotes the state/** rule when arguing that a new key
    needs no new binding -- and an unbounded scan picks those up as extra rows,
    reporting a drift that does not exist. A test that reports a drift that does
    not exist gets relaxed by whoever hits it, and then it reports nothing.
    """
    text = read_contract()
    start = text.index("    bindings: [")
    end = text.index("\n    ],", start)
    return text[start:end]


def test_fixture_bindings_are_verbatim():
    """Every fixture binding appears in 11 S2.4.7, in order, set clause included.

    Without this case the fixture would be an unchecked copy: it could drift from
    the contract and every vector in this file would stay green, because the
    vectors were derived from the fixture. This is what makes the golden document
    a transcription rather than a second source.
    """
    block = bindings_block()
    # The order comparison first, because it is the property the whole design
    # rests on. Comparing sets would pass for a fixture whose safety group had
    # been moved to the end -- which is mutant 1, deliberately.
    pairs = re.findall(r'\{\s*match:\s*"([^"]+)",\s*profile:\s*"(\w+)"', block)
    assert pairs == [(b["match"], b["profile"]) for b in DOCUMENT["bindings"]]
    for binding in DOCUMENT["bindings"]:
        pattern = r'\{\s*match:\s*"%s",\s*profile:\s*"%s"' % (
            re.escape(binding["match"]), re.escape(binding["profile"]))
        clause = binding.get("set")
        if clause is not None:
            # The set clause is rebuilt from the fixture and searched for, so a
            # depth or a priority that changed on either side fails here. Only
            # the two forms S2.4.7 actually uses are handled; a fixture carrying
            # anything else raises KeyError below rather than passing unchecked,
            # which is the failure direction to prefer.
            inner = []
            if "priority" in clause:
                inner.append(r'priority:\s*"%s"' % re.escape(clause["priority"]))
            if "handler" in clause:
                inner.append(r'handler:\s*\{\s*kind:\s*"%s",\s*depth:\s*%d\s*\}'
                             % (re.escape(clause["handler"]["kind"]),
                                clause["handler"]["depth"]))
            pattern += r',\s*set:\s*\{\s*' + r',\s*'.join(inner) + r'\s*\}'
        # The closing brace is part of the pattern so that a rule with an extra
        # field the fixture omits cannot match as a prefix.
        pattern += r'\s*\}'
        assert re.search(pattern, block), binding["match"]


def test_frozen_profiles_are_verbatim():
    """FROZEN_PROFILES in the module equals the profiles block in 11 S2.4.7.

    The module transcribes the table instead of parsing the markdown, which is
    the same trade the session factory makes for its endpoints: a parser for
    prose-wrapped json5 is a second thing to keep correct. A transcription nobody
    compares against its source is a constant that drifts, so the comparison is
    here rather than in a comment claiming the two agree.
    """
    rows = re.findall(
        r'^\s*(Q\w+):\s*\{\s*congestion_control:\s*"(\w+)",\s*'
        r'priority:\s*"(\w+)",\s*reliability:\s*"(\w+)",\s*'
        r'express:\s*(true|false),\s*handler:\s*\{\s*kind:\s*"(\w+)",\s*'
        r'depth:\s*(\d+)\s*\}\s*\}', read_contract(), re.M)
    # Bidirectional: the contract must not carry a profile the module lacks, and
    # the module must not carry one the contract dropped. A one-way containment
    # check would let an invented sixth profile live in the module forever, and
    # an invented profile routes around every argument in S2.4.5 since none of
    # them covers it.
    assert {r[0] for r in rows} == set(FROZEN_PROFILES)
    for name, congestion, priority, reliability, express, kind, depth in rows:
        profile = FROZEN_PROFILES[name]
        assert profile.congestion_control == congestion
        assert profile.priority == priority
        assert profile.reliability == reliability
        # The contract writes json5 booleans as text, so the comparison converts
        # rather than trusting truthiness -- "false" is a non-empty string.
        assert profile.express is (express == "true")
        assert profile.handler.kind == kind
        if depth == "0":
            # The sentinel. The contract writes 0 and the module holds MISSING,
            # which is the whole point: a 0 that survived into the module would
            # size a ring buffer to nothing and log nothing about it. This branch
            # is the only place the translation is asserted in either direction.
            assert profile.handler.depth is MISSING
        else:
            assert profile.handler.depth == int(depth)


def test_rt_override_is_verbatim():
    """RT_OVERRIDE equals the rt_override block in 11 S2.4.7.

    This is the constant the field table says is 硬编码在实现中, so it is the one
    value in this package that no configuration can correct. If it drifts from
    the contract, nothing else in the system will notice: the override still
    fires, still reports itself as applied, and still produces a working
    publisher -- from the wrong queue.
    """
    found = re.findall(
        r'rt_override:\s*\{\s*congestion_control:\s*"(\w+)",\s*'
        r'priority:\s*"(\w+)",\s*handler:\s*\{\s*kind:\s*"(\w+)",\s*'
        r'depth:\s*(\d+)\s*\}\s*\}', read_contract())
    # Exactly one occurrence. Two would mean the section had been duplicated
    # during an edit, and this test would then be comparing against whichever
    # copy the regex happened to reach first.
    assert len(found) == 1
    congestion, priority, kind, depth = found[0]
    assert RT_OVERRIDE.congestion_control == congestion
    assert RT_OVERRIDE.priority == priority
    assert RT_OVERRIDE.handler.kind == kind
    assert RT_OVERRIDE.handler.depth == int(depth)


# ---------------------------------------------------------------------------
# The matcher.
#
# Three cases, covering three different ways to be wrong. The zenoh cross-check
# covers "does it agree with the implementation that actually routes"; the
# zero-chunk case covers a property no golden key exercises; and the separator
# case covers the specific mistake a reader reaching for fnmatch would make.
#
# Only the first needs eclipse-zenoh, so the other two keep running on a host
# without it -- which is the host this suite is most likely to be run on.
# ---------------------------------------------------------------------------

@pytest.mark.skipif(
    zenoh is None,
    reason="eclipse-zenoh is not installed, so the hand-written key-expression "
           "matcher was NOT compared against the one Zenoh actually uses; the "
           "vectors passing does not establish that the two agree, and a "
           "disagreement would show up on the robot as a key carrying different "
           "QoS than this file says it does")
def test_matcher_agrees_with_zenoh():
    """Every pattern-key pair resolves the same way in both matchers.

    This is the case that makes the hand-written matcher defensible at all. The
    resolver cannot use zenoh.KeyExpr directly -- the module has to keep working
    where the client is absent, for the same reason the session factory does --
    so the only alternative to comparing them is asserting by hand that the two
    agree, which is the kind of claim this project has been caught making
    wrongly.

    Both directions of the zenoh API are checked. includes() is the operation
    this resolver means -- does the pattern cover this concrete key -- and
    intersects() is the one a reader is more likely to reach for; for a key with
    no wildcards they must coincide, and asserting both catches a golden key that
    accidentally acquired a wildcard.
    """
    keys = [v["key"] for v in VECTORS]
    for binding in DOCUMENT["bindings"]:
        pattern = zenoh.KeyExpr(binding["match"])
        for key in keys:
            expr = zenoh.KeyExpr(key)
            ours = key_expr_matches(binding["match"], key)
            assert ours is pattern.includes(expr), (binding["match"], key)
            assert ours is pattern.intersects(expr), (binding["match"], key)


# Written against the matcher directly rather than through a table. A binding
# whose only purpose is to exercise the zero-chunk case would have to be added to
# the fixture, and the fixture is a transcription of 11 -- it must not grow rows
# the contract does not have, or the transcription cases become the thing that
# has to be relaxed.
def test_double_star_spans_zero_chunks():
    """xbrain/*/rt/safety/** covers xbrain/{rid}/rt/safety itself.

    Asserted separately because it is the property a naive matcher gets wrong,
    and getting it wrong is silent: rt/safety would fall past the safety group
    into rt/**, land in Q3-rt, and lose real_time and express -- which is the
    exact defect S2.4.7's v0.3 comment records having already happened once.

    It is also the property the zenoh cross-check above cannot carry on its own,
    since no golden key exercises the zero-chunk case.
    """
    assert key_expr_matches("xbrain/*/rt/safety/**", "xbrain/dog-01/rt/safety")


def test_single_star_does_not_cross_a_slash():
    """xbrain/*/cmd/estop does not cover xbrain/{rid}/cmd/estop/ack.

    This is what separates a chunk matcher from fnmatch. If * crossed the
    separator, the ack would inherit the estop rule -- which happens to give the
    same profile today, so the defect would sit there unnoticed until a rule
    ordering changed and moved something else. Confirmed red against a matcher
    edited to give * the fnmatch behaviour.
    """
    assert not key_expr_matches("xbrain/*/cmd/estop",
                                "xbrain/dog-01/cmd/estop/ack")


# ---------------------------------------------------------------------------
# The unsupplied Q4 depth.
#
# The two cases here are opposite halves of one rule and neither is sufficient
# alone. Without the refusal, a resolver could return 0 and every audio chunk
# would be dropped by a ring buffer sized to nothing. Without the acceptance, an
# audit written too strictly would refuse the one edit a deployment is actually
# required to make, and the only way to finish the configuration would be to
# weaken the audit -- which is how a safety check becomes a warning.
# ---------------------------------------------------------------------------

def test_q4_depth_must_be_supplied_by_deployment():
    """audio/broadcast resolves, and asking for its queue depth refuses.

    Both halves matter. Resolving must succeed, because the profile IS Q4 and the
    item's criterion says so; asking for the depth must raise, because 11 S2.4.7
    gives 0 the meaning "no default" and S13.15 lists this condition under
    E_QOS_VIOLATION by name. Returning any number here would be the CLAUDE.md 3.1
    failure in its exact shape -- a value that passes every is-it-assigned check
    and then sizes an audio jitter buffer nobody calibrated.
    """
    got = build().resolve("xbrain/dog-01/audio/broadcast")
    assert got.profile_name == "Q4_stream"
    assert got.handler.depth is MISSING
    with pytest.raises(QosViolation) as caught:
        got.require_handler_depth()
    # The code is compared against the imported name, never a literal. A test
    # that spelled "E_QOS_VIOLATION" itself would pass against a module that had
    # invented a code of its own, which is the drift CLAUDE.md 3.5 describes.
    assert caught.value.code == E_QOS_VIOLATION


def test_a_supplied_q4_depth_is_used():
    """A deployment may fill in the depth the frozen table leaves unsupplied.

    The number below is arbitrary and deliberately implausible. 11 S2.4.2 does
    now compute N = 10 for Q4, and it is NOT used here on purpose: the same
    paragraph keeps the value pending QoS-T8, and a plausible number in a test is
    one copy-paste away from being a plausible number in configs/. This case is
    about the mechanism, not the value.

    It is also the case that stops the frozen-profile audit from being written
    too strictly. Supplying a depth where the table has none is the one edit a
    deployment is REQUIRED to make, so an audit that refused it would block the
    only correct way to finish the configuration.
    """
    document = mutated()
    document["profiles"]["Q4_stream"]["handler"]["depth"] = 3
    got = build(document).resolve("xbrain/dog-01/audio/broadcast")
    assert got.require_handler_depth() == 3


# ---------------------------------------------------------------------------
# The four mutants the item's criterion names.
#
# CLAUDE.md 3.3: an assertion is accepted only by injecting something that must
# violate it and watching it go red. These four are in the suite rather than in a
# reviewer's notes, so they are re-run on every commit and cannot rot into a
# claim that somebody once tried them.
#
# Mutants 1 and 4 drive check_vector -- the same helper the green cases use --
# through pytest.raises(AssertionError), which is the only construction that
# demonstrates the GOLDEN case going red rather than some private assertion of
# the mutant's own. Mutants 2 and 3 cannot take that shape: 2 is a document the
# loader must refuse outright, so there is no resolution left to check, and 3 is
# a document whose resolutions must be IDENTICAL, so its assertion is the green
# one repeated over every vector.
# ---------------------------------------------------------------------------

# Mutant 1. The criterion: move the six safety bindings after xbrain/*/rt/**,
# and rt/safety/estop must resolve to something other than Q0.
#
# This is the one mutant that reproduces a defect that really happened. S2.4.7's
# own comment records it: before v0.3 the safety keys were not in front, and the
# emergency-stop path silently lost real_time and express while continuing to
# deliver every message.
def test_mutant_1_safety_bindings_moved_after_the_rt_fallback():
    """Reordering the safety group behind rt/** breaks the estop vector.

    The reordering is done by rebuilding the list rather than by swapping two
    indices, so the mutant survives a fixture that grows a binding. It also
    keeps the six safety rules in their original relative order, which matters:
    the point being tested is that they are now behind rt/**, not that they were
    shuffled among themselves.

    Only rt/safety/estop is checked against check_vector. The other five safety
    keys are on the general plane, where rt/** cannot reach them, so they still
    resolve to Q0 -- which is itself worth knowing, because it means a reordering
    like this one breaks part of the safety group and leaves the rest working.
    """
    document = mutated()
    bindings = document["bindings"]
    # The six rows S2.4.7 groups under 安全链路, identified by the profile they
    # name rather than by index. An index list would silently select the wrong
    # rows if the fixture ever grew a binding, and the mutant would then be
    # testing something else while still passing.
    safety = [b for b in bindings if b["profile"] == "Q0_safety"]
    assert len(safety) == 6
    rest = [b for b in bindings if b["profile"] != "Q0_safety"]
    fallback_at = [i for i, b in enumerate(rest)
                   if b["match"] == "xbrain/*/rt/**"][0]
    document["bindings"] = rest[:fallback_at + 1] + safety + rest[fallback_at + 1:]
    table = build(document)
    # The resolution changes as the criterion says: no longer Q0.
    got = table.resolve("xbrain/dog-01/rt/safety/estop")
    assert got.profile_name != "Q0_safety"
    # And it lands in the fallback, rewritten by QOS-C1 -- the historical defect
    # exactly. Asserting the destination and not merely "not Q0" is what makes
    # this a reproduction rather than a smoke test: it shows the estop path ends
    # up in the general command queue with interactive_high and no express.
    assert got.profile_name == "Q3_cmd"
    assert got.rt_override_applied is True
    assert got.priority != "real_time"
    assert got.express is False
    # The green assertion, run against the mutant, must fail. This is the part
    # CLAUDE.md 3.3 asks for: the vector has now been red, in this suite, on this
    # run -- not in a reviewer's recollection of having tried it once.
    vector = [v for v in VECTORS if v["key"] == "xbrain/dog-01/rt/safety/estop"][0]
    with pytest.raises(AssertionError):
        check_vector(table, vector)


# Mutant 2. The criterion: a binding whose set overrides reliability or
# congestion_control must be refused with E_CONFIG_INVALID.
#
# Parametrised over both fields because they are forbidden for different reasons
# and a loader could easily block one and forget the other.
@pytest.mark.parametrize("field,value", [
    # best_effort on the event fallback: the replay cursor in S2.4.5 第 6 条
    # advances past an event that never left the machine.
    ("reliability", "best_effort"),
    # drop reaches the same end by discarding at the queue instead of the link.
    ("congestion_control", "drop"),
])
def test_mutant_2_a_set_clause_overriding_a_safety_knob_is_refused(field, value):
    """set: { reliability | congestion_control } raises E_CONFIG_INVALID.

    The values chosen are the damaging direction in each case. best_effort on the
    event fallback breaks the replay cursor in S2.4.5 第 6 条: the event is
    marked sent, the cursor moves past it, and it is gone with nobody informed.
    drop does the same by another route.

    *** The message assertion is not decoration. Removing reliability from the
    forbidden pair was injected as a mutation and this case stayed GREEN: the
    field fell through to the unknown-field branch and still raised
    E_CONFIG_INVALID. The two failures need different messages -- one is a
    misspelling, the other a safety prohibition -- and the deployment engineer
    reading it acts differently on each, so the case now asserts which fired.
    """
    document = mutated()
    fallback = [b for b in document["bindings"]
                if b["match"] == "xbrain/*/**"][0]
    fallback["set"] = {field: value}
    with pytest.raises(QosConfigError) as caught:
        build(document)
    assert caught.value.code == E_CONFIG_INVALID
    message = str(caught.value)
    # The field must be named, because a deployment engineer reading this has
    # twenty-five bindings to choose from.
    assert field in message
    # And it must be reported as forbidden rather than as unknown. This is the
    # assertion the injected mutation defeated.
    assert "forbidden" in message


# Mutant 3. The criterion: delete rt_override from the document and the
# resolution must not change, which is what proves it is hard-coded.
#
# Note what this case does NOT do: it does not assert an exception. If the loader
# refused a document without rt_override, this mutant could not distinguish a
# hard-coded override from a consumed one -- both would raise, and the criterion
# would be satisfied by an implementation that fails the property it is testing.
def test_mutant_3_deleting_rt_override_changes_nothing():
    """A document with no rt_override resolves identically to one with it.

    This is the only one of the four mutants whose expected outcome is that
    nothing happens, which makes it the easiest to write in a way that proves
    nothing. Two things stop that here: every vector is re-checked rather than
    one, and the overridden key's handler is compared against RT_OVERRIDE by
    field afterwards -- so an implementation that quietly substituted the
    matched profile's FIFO(256) would fail even though the profile name and the
    congestion control still looked right.
    """
    document = mutated()
    del document["rt_override"]
    table = build(document)
    assert table.rt_override_in_config is None
    # Every vector, not only the overridden one. A resolver that read the
    # configured block would most plausibly fall back to some default when it is
    # absent, and that default might coincide on the one key a spot check looked
    # at. Confirmed red against an implementation edited to do exactly that.
    for vector in VECTORS:
        check_vector(table, vector)
    # And specifically: the key that goes through the override still does, with
    # the override's own handler and not the matched profile's.
    got = table.resolve("xbrain/dog-01/rt/behavior/request")
    assert got.rt_override_applied is True
    assert got.handler.kind == RT_OVERRIDE.handler.kind
    assert got.handler.depth == RT_OVERRIDE.handler.depth


def test_a_contradictory_rt_override_is_refused():
    """A document that disagrees with the hard-coded override is refused.

    The other half of mutant 3, and the reason tolerating its absence is not the
    same as ignoring it. 11 S2.4.7 says the configured value exists 仅供审计比对;
    a deployment that writes something else believes the stack does something it
    does not, and that belief is what a person acts on during commissioning --
    they will read the file, not the source.
    """
    document = mutated()
    document["rt_override"]["priority"] = "background"
    with pytest.raises(QosConfigError) as caught:
        build(document)
    assert caught.value.code == E_CONFIG_INVALID


# Mutant 4. The criterion, added this round: a golden key written WITHOUT the
# xbrain/{rid}/ prefix must be rejected, not silently bucketed into the fallback.
#
# Why it needed adding. Every binding is written xbrain/*/..., so a bare key
# matches none of the specific rules. The naive expectation is that it lands in
# xbrain/*/** and comes back Q3 -- plausible, wrong, and invisible. In fact it
# matches nothing at all, so a resolver without a door check would either return
# a made-up default or fail somewhere far less informative.
@pytest.mark.parametrize("key", [
    # The one a reader writes from memory, because it is how 11 S2.2 prose and
    # this project's own conversation refer to the key.
    "cmd/estop",
    # The same shape on the other plane, so the check cannot be passing because
    # of something particular to cmd/.
    "rt/safety/estop",
    # The key whose depth is unsupplied. A resolver that got past the door would
    # fail here for a completely different reason, and the reader would chase
    # the depth instead of the missing root.
    "audio/broadcast",
])
def test_mutant_4_a_bare_key_is_rejected(key):
    """A key without the xbrain/{robot_id}/ root raises rather than resolving.

    Three keys and not one: the first is the one a reader would write from
    memory, the second is on the other plane, and the third is the one whose
    depth is unsupplied -- so a resolver that got past the door check would fail
    three different ways further in, and only one of them loudly.

    The criterion added this mutant in a later round, and the reason is worth
    keeping: the reviewer's first expectation was that a bare key would fall into
    the xbrain/*/** fallback and come back Q3. That is wrong -- the fallback also
    requires the root -- and the case below asserts it directly, because an
    expectation that plausible will be formed again.
    """
    table = build()
    with pytest.raises(QosViolation) as caught:
        table.resolve(key)
    assert caught.value.code == E_QOS_VIOLATION
    # The message must say what is wrong with the key, not merely that it failed.
    # "xbrain" appearing in it is what tells the reader the root is the problem
    # rather than the plane or the name.
    assert "xbrain" in str(caught.value)


def test_mutant_4_the_bare_key_would_not_even_hit_the_fallback():
    """The bare form matches no binding at all, so no default could be right.

    Asserted directly against the matcher, because the point of mutant 4 is
    easily mis-stated as "the fallback would have caught it". It would not: the
    fallback is xbrain/*/**, which requires the root chunk. A resolver that
    "helpfully" prefixed a wildcard would be answering for a key the caller
    cannot publish on, and the answer would look entirely ordinary.
    """
    for binding in DOCUMENT["bindings"]:
        assert not key_expr_matches(binding["match"], "cmd/estop")


# ---------------------------------------------------------------------------
# Refusals that have no vector, because their correct outcome is an exception.
#
# Each of these is a document or a key that must not produce an answer. They are
# worth as much as the vectors: 11 S2.4.0 is an argument about what happens when
# a key ends up with QoS nobody chose, and every case below is a route by which
# that could happen quietly. A resolver that answered any of them would look
# entirely healthy from the outside.
#
# All of them assert the error CODE and not just the type. The type is ours and
# could be renamed; the code is what crosses a process boundary and what the
# cloud branches on, and 11 S13.15 gives both rows retryable = no.
# ---------------------------------------------------------------------------

def test_no_matching_binding_raises_rather_than_defaulting():
    """Remove the fallback and an unmatched key refuses instead of defaulting.

    11 S2.4.0 is entirely about what the Zenoh binding defaults do to this system
    -- block, data, reliable, FIFO(256) -- so falling back to them is the one
    outcome that must never happen. The mutation here is removing the last rule,
    which is also the realistic way it would happen: someone tidies a binding
    they believe is dead. Confirmed red against a resolver edited to fall back to
    the last binding instead of raising.
    """
    document = mutated()
    document["bindings"] = [b for b in document["bindings"]
                            if b["match"] != "xbrain/*/**"]
    with pytest.raises(QosViolation) as caught:
        build(document).resolve("xbrain/dog-01/health/bit")
    assert caught.value.code == E_QOS_VIOLATION


# Both sections, because they are null for the same reason and would be filled
# in by different people: the profile table comes from the contract, the bindings
# from whoever owns the deployment.
@pytest.mark.parametrize("field", ["profiles", "bindings"])
def test_a_null_section_refuses_and_names_the_key_path(field):
    """qos.profiles: null and qos.bindings: null both refuse to load.

    This is not a hypothetical document. It is the state of configs/common.yaml
    today: both key positions are declared and neither is assigned, so the stack
    refuses to start. CLAUDE.md 3.1 calls that the expected behaviour, and this
    case exists to keep it that way -- the tempting repair is a default profile
    set, which would let the stack start carrying QoS nobody chose.

    The key path is asserted as well as the code, because the operator holding
    the failure has nineteen files under configs/ and the message is the only
    thing that says which one is incomplete.
    """
    document = mutated()
    document[field] = None
    with pytest.raises(QosConfigError) as caught:
        build(document)
    assert caught.value.code == E_CONFIG_INVALID
    assert "qos.%s" % field in str(caught.value)


@pytest.mark.parametrize("name,field,value", [
    # The unbounded wait S2.4.5 第 7 条 removes from the emergency-stop path.
    ("Q0_safety", "congestion_control", "block"),
    # The back-pressure the event replay cursor depends on, deleted.
    ("Q3_cmd", "congestion_control", "drop"),
    # cmd_vel behind every other class of traffic on the link.
    ("Q1_rt", "priority", "background"),
    # Every state key off the batching timer at once, changing the packet
    # economics of the whole general plane for no stated reason.
    ("Q2_state", "express", True),
])
def test_a_deployment_may_not_redefine_a_frozen_profile(name, field, value):
    """Editing a frozen profile in the document is refused. 14.2 F-11.

    Each row is a change with a stated consequence rather than a random edit:
    block on Q0 is the unbounded wait S2.4.5 第 7 条 removes from the estop path;
    drop on Q3 deletes the back-pressure the event replay cursor depends on;
    background on Q1 puts cmd_vel behind everything else on the link; express on
    Q2 changes the packet economics of every state key at once.

    Without this audit the word "frozen" would have no mechanism behind it: every
    vector in this file would still pass, because the vectors assert what the
    document says and the document would be the thing that had changed.
    """
    document = mutated()
    document["profiles"][name][field] = value
    with pytest.raises(QosConfigError) as caught:
        build(document)
    assert caught.value.code == E_CONFIG_INVALID
    assert name in str(caught.value)


def test_an_extra_profile_is_refused():
    """A profile 11 S2.4.2 does not define cannot be added by deployment.

    The extra profile is a copy of an existing one, so it is internally valid in
    every respect -- every knob is a legal value and the handler is well formed.
    The only thing wrong with it is that the contract does not have it, which is
    exactly the failure a per-field validator cannot see.


    Closed sets raise on out-of-set values (11 S13.6, CLAUDE.md 3.5). A sixth
    profile would also route around every argument in S2.4.5, since none of them
    covers it -- and the argument, not the value, is what makes a profile
    defensible.
    """
    document = mutated()
    document["profiles"]["Q5_custom"] = copy.deepcopy(
        document["profiles"]["Q3_cmd"])
    with pytest.raises(QosConfigError) as caught:
        build(document)
    assert caught.value.code == E_CONFIG_INVALID


def test_a_set_clause_may_restate_the_handler_kind_but_not_change_it():
    """kind: ring on a ring profile is accepted; kind: fifo on it is refused.

    Both halves are asserted because the two sentences in S2.4.7 pull in
    opposite directions here -- the field table lists only priority and
    handler.depth as overridable, while the frozen bindings themselves write
    handler.kind on three rows. Accepting a kind that restates the profile's is
    the only reading under which both hold; accepting one that changes it would
    make the field table's list meaningless, and refusing the restatement would
    make the contract's own bindings unloadable.

    See the module header in xbrain/common/zenoh/qos.py for the full note. This
    case is the executable form of it, so a later revision that means to allow a
    real kind change has one place to come and change.
    """
    document = mutated()
    binding = [b for b in document["bindings"]
               if b["match"] == "xbrain/*/rt/teleop/input"][0]
    # Q1_rt is a ring profile, so this restates and must load.
    binding["set"] = {"handler": {"kind": "ring", "depth": 2}}
    assert build(document).resolve(
        "xbrain/dog-01/rt/teleop/input").handler.depth == 2
    # And the change must not.
    binding["set"] = {"handler": {"kind": "fifo", "depth": 2}}
    with pytest.raises(QosConfigError) as caught:
        build(document)
    assert caught.value.code == E_CONFIG_INVALID


# The remaining refusals are not named by the criterion. They are here because
# each one is a document a person could plausibly write, and each would produce a
# working system carrying QoS nobody chose.
def test_a_binding_not_rooted_at_xbrain_is_refused():
    """A match expression that can never match any key is refused at load.

    11 S2.2.12 already records one binding left behind under a renamed key and
    calls it 死配置, 应删除. Dead configuration costs nothing at run time and is
    invisible in every test, which is precisely why it survives; load time is the
    only moment anything is in a position to notice it.

    The rule is inserted at the FRONT, where it would do the most damage if it
    did match -- so a loader that accepted it would also be shown to have let a
    rule ahead of the safety group.
    """
    document = mutated()
    document["bindings"].insert(0, {"match": "cmd/estop", "profile": "Q0_safety"})
    with pytest.raises(QosConfigError) as caught:
        build(document)
    assert caught.value.code == E_CONFIG_INVALID


@pytest.mark.parametrize("key,why", [
    # A trailing separator, which is what string concatenation with an empty
    # last segment produces.
    ("xbrain/dog-01/cmd/estop/", "trailing slash leaves an empty chunk"),
    # An unset robot_id joined into the template. The key still has the right
    # number of separators, so a length check alone would pass it.
    ("xbrain//cmd/estop", "empty robot_id"),
    # A plane with nothing on it. Three chunks is a prefix, not a key.
    ("xbrain/dog-01/cmd", "no name after the plane"),
    # Well formed in every respect except the charset, so it would resolve
    # perfectly and address a robot that does not exist.
    ("xbrain/DOG-01/cmd/estop", "robot_id outside [a-z0-9_-] from 11 S2.1"),
    # A match expression handed in where a key belongs, which would otherwise
    # get one answer for a whole family of keys.
    ("xbrain/dog-01/*/estop", "a wildcard is not a publishable key, 11 S2.2 W-4"),
])
def test_a_malformed_key_is_refused(key, why):
    """Keys that are not S2.1-shaped raise instead of resolving.

    why is carried through into the failure message rather than left as a
    comment, because these five keys look almost identical in a test id and the
    reader of a failure needs to know which shape stopped being rejected.

    The uppercase robot_id is the one worth keeping when someone trims this list:
    it is well formed in every respect except the charset, so it would resolve
    perfectly and address a robot that does not exist.
    """
    with pytest.raises(QosViolation) as caught:
        build().resolve(key)
    assert caught.value.code == E_QOS_VIOLATION, why


# Last, and separate from the malformed-key cases above, because the failure has
# a different type and a different owner: the plane closed set belongs to
# xbrain/common/enums and is shared with every other consumer of the key space.
def test_an_unregistered_plane_raises_a_closed_set_violation():
    """A plane outside the eight values of 11 S2.1 is refused by the closed set.

    Raised as ClosedSetViolation and not as a QoS error on purpose: the failure
    is that the key names a plane that does not exist, and the closed-set type
    says which set was violated. Letting it through to the bindings would give it
    the xbrain/*/** fallback and a working publisher on a plane nothing routes --
    the publisher would come up, report success, and be heard by nobody.
    """
    with pytest.raises(ClosedSetViolation):
        parse_full_key("xbrain/dog-01/telemetry/thing")
