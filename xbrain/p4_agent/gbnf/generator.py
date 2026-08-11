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


# --- GWY-P4-37 (32.E) generate_grammar executor -------------------------

# 16 S5.1 / AI-36: a 3B model's tool-selection accuracy degrades sharply
# past ~5-8 tools, so a mission grammar exposes only its OWN intents, at
# most 5. The count budget of 5 INCLUDES the implicit unknown fallback
# (16 S6.7 hard constraint "<=5 counting unknown"); unknown is never in
# the alternation (I4), so the emitted alternation is at most 5 -- and
# only M4_follow reaches 5 business intents (the one documented 6-total
# exception). Handing generate_grammar the full 128-intent registry is
# exactly the failure this rejects.
MAX_MISSION_INTENTS = 5


def project_mission_intents(registry, mission_intent_names: List[str]):
    """Project a mission's intent NAMES onto the registry.

    Returns (alternation, intent_routes): the ordered intent-name list and
    a name->route map. Every name MUST exist in the registry (by_name
    raises otherwise -- a mission that names a non-intent is a config bug,
    not something to silently drop). The route map lets generate_grammar's
    GB-1c reject a fastpath intent that leaked into a mission set.
    """
    alternation: List[str] = []
    routes: Dict[str, str] = {}
    for name in mission_intent_names:
        entry = registry.by_name(name)   # raises on unknown name
        alternation.append(name)
        routes[name] = entry.route
    return alternation, routes


def _gbnf_intent_rule(alternation: List[str]) -> str:
    """Emit the `intent ::= "a" | "b"` production. Names are GBNF string
    literals, so each is wrapped in escaped double quotes."""
    alts = " | ".join('"\\"%s\\""' % name for name in alternation)
    return "intent ::= " + alts


def generate_grammar(
    alternation: List[str],
    intent_routes: Dict[str, str],
) -> str:
    """Build the mission GBNF that constrains the LLM's `intent` choice.

    Enforces the generator invariants BEFORE emitting anything:
      * R4  empty alternation is not a production
      * I4  'unknown' is not an alternation member (it is the no-match
            fallback the model picks, not a listed choice)
      * GB-1c  no fastpath intent in a mission grammar (fastpath bypasses
            the LLM; its presence here is dead code that also lets the LLM
            misroute a match)
      * AI-36 / S5.1  alternation size <= MAX_MISSION_INTENTS

    Returns the GBNF text. Raises GbnfInvariantError on any violation --
    the grammar is never emitted half-valid.
    """
    check_r4_no_empty_production(alternation)
    check_i4_no_unknown_in_alternation(alternation)
    check_gb_1c_no_fastpath_in_mission_grammar(intent_routes, alternation)
    if len(set(alternation)) > MAX_MISSION_INTENTS:
        raise GbnfInvariantError(
            "AI-36/S5.1: mission alternation has %d intents (> %d). A "
            "mission must expose only its own <=5 intents, never the full "
            "registry -- a 3B model degrades past ~5 tools."
            % (len(set(alternation)), MAX_MISSION_INTENTS))
    # Minimal root: the object must carry an intent; slots are filled by
    # the fastpath regex for closed-set enums (16 S8.0.4), so the mission
    # grammar constrains the intent choice and leaves slots to a permissive
    # object the validation layer (V1..V7) then checks.
    root = 'root ::= "{" ws "\\"intent\\":" ws intent (ws "," ws rest)? ws "}"'
    lines = [
        root,
        _gbnf_intent_rule(alternation),
        'rest ::= [^}]*',           # slots and remainder; validated downstream
        'ws ::= [ \\t\\n]*',
    ]
    return "\n".join(lines) + "\n"
