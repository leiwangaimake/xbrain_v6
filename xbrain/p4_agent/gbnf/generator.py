"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: generator.py
Brief: GWY-P4-12 -- GBNF constraint generator (R1..R4 + GB-1c/GB-1d + I3/I4)

Description:
16 S7 GBNF constrains LLM output to a valid intent+slots shape.
Generator invariants (R1..R4):
  R1 intent alternation is EXACTLY the mission's declared set
  R2 slot enum values are TAKEN from live SQLite (not hardcoded)
  R3 no slot value that isn't in the closed set at generation time
  R4 empty candidate set -> the ENTIRE production is not emitted
     (NOT emitted as `intent ::=`; the LLM's grammar has NO way
     to reach an intent that has no valid slots)

GB-1c: mission grammar MUST NOT include an intent whose route is
       'fastpath' (fastpath intents bypass LLM entirely; a fastpath
       intent in mission grammar is dead code that also opens the
       door to a wrong LLM interpretation)
GB-1d: intent id in grammar MUST equal intent id in cmdset_18.json
       (verbatim; drift = the LLM emits an id no consumer knows)

I3: mission grammar's `intent ::= a | b | c` MUST match what the
    mission's few-shot examples teach (else few-shot vs grammar
    disagree at gen time -- silent failure mode)
I4: alternation must NOT contain the sentinel 'unknown' -- 'unknown'
    is what the model chooses when NOTHING matches, not one of the
    matches
"""

from __future__ import annotations

from typing import Dict, FrozenSet, List, Set


class GbnfInvariantError(RuntimeError):
    """A GBNF invariant was violated. The message names the specific rule."""


def check_r1_alternation_matches_mission_set(
    emitted: List[str], mission_set: FrozenSet[str]) -> None:
    """R1: emitted alternation MUST equal mission_set."""
    a = set(emitted)
    if a != mission_set:
        raise GbnfInvariantError(
            "R1: emitted intent alternation %s != mission set %s "
            "(sym-diff %s)"
            % (sorted(a), sorted(mission_set), sorted(a ^ mission_set)))


def check_r4_no_empty_production(alternation: List[str]) -> None:
    """R4: an empty alternation MUST NOT be emitted as a production
    (would let the LLM emit `intent ::=` and pick nothing valid).
    Caller must catch this BEFORE writing the grammar file."""
    if not alternation:
        raise GbnfInvariantError(
            "R4: empty intent alternation must not be emitted as "
            "a production; caller should skip the intent rule entirely")


def check_gb_1c_no_fastpath_in_mission_grammar(
    intent_routes: Dict[str, str],
    mission_alternation: List[str]) -> None:
    """GB-1c: mission grammar MUST NOT contain fastpath intents."""
    bad = [i for i in mission_alternation
           if intent_routes.get(i) == "fastpath"]
    if bad:
        raise GbnfInvariantError(
            "GB-1c: fastpath intent(s) %s in mission alternation. "
            "fastpath bypasses LLM; a fastpath intent in mission "
            "grammar is dead code AND lets LLM misroute a match."
            % bad)


def check_gb_1d_ids_match_cmdset(
    grammar_ids: Dict[str, str],
    cmdset_ids: Dict[str, str]) -> None:
    """GB-1d: intent NAME -> ID mapping in grammar MUST equal
    cmdset_18.json. Any drift is silent breakage downstream."""
    for name, grammar_id in grammar_ids.items():
        if name not in cmdset_ids:
            raise GbnfInvariantError(
                "GB-1d: intent %r in grammar has no cmdset entry" % name)
        if cmdset_ids[name] != grammar_id:
            raise GbnfInvariantError(
                "GB-1d: intent %r grammar id=%r != cmdset id=%r"
                % (name, grammar_id, cmdset_ids[name]))


def check_i4_no_unknown_in_alternation(alternation: List[str]) -> None:
    """I4: 'unknown' is a MODEL choice on no-match, NOT an alternation
    member. Presence would let the LLM 'match' unknown when a real
    intent was available."""
    if "unknown" in alternation:
        raise GbnfInvariantError(
            "I4: 'unknown' MUST NOT be in the intent alternation "
            "(it is the fallback picked when no alternation entry matches)")


def check_i3_grammar_matches_few_shots(
    grammar_alt: FrozenSet[str],
    few_shot_intents: FrozenSet[str]) -> None:
    """I3: alternation and few-shot examples must agree."""
    if grammar_alt != few_shot_intents:
        raise GbnfInvariantError(
            "I3: mission grammar alternation %s disagrees with "
            "few-shot examples %s (sym-diff %s)"
            % (sorted(grammar_alt), sorted(few_shot_intents),
               sorted(grammar_alt ^ few_shot_intents)))
