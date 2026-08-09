"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: startup_assertions.py
Brief: GWY-P4-08 -- CS-A1..CS-A4 startup consistency assertions

Description:
16 S0.5 CS-A* four assertions run at P4 startup after intents.yaml,
cmdset_18.json, mission prompts are loaded. Every one refuses
process start if it fails.

  CS-A1  every intent NAME in intents.yaml MUST appear in
         cmdset_18.json's 128-intent closed set (no extra intent
         invented in registry)
  CS-A2  count(intents.yaml rows) == count(cmdset_18.json intents)
  CS-A3  each mission prompt's `intent ::= ...` alternation is a
         SUBSET of the intent closed set
  CS-A4  each mission prompt's alternation size + 1 (unknown) <= 5
         (AI-36 hard limit); one break allowed: M4_follow at 6

★ CS-A3 has a 3-step transitional implementation (spec verbatim):
    if a prompt references an intent NOT in the closed set, run in
    'warn'-forced mode instead of refuse: log the mismatch, load the
    prompt with the unknown intent DROPPED from the alternation.
    (This is the transitional path while 18 gets updated.)
"""

from __future__ import annotations

from typing import Dict, FrozenSet, Iterable, List, Set


class CsAssertionError(RuntimeError):
    """A CS-A* assertion failed. Rule name in message."""


def check_cs_a1(intent_names: Iterable[str],
                cmdset_closed_set: FrozenSet[str]) -> None:
    """CS-A1: every intent name in intents.yaml must be in cmdset_18."""
    extras = set(intent_names) - cmdset_closed_set
    if extras:
        raise CsAssertionError(
            "CS-A1: intents.yaml contains name(s) NOT in cmdset_18.json: %s "
            "(closed set has %d entries)"
            % (sorted(extras), len(cmdset_closed_set)))


def check_cs_a2(intents_yaml_count: int,
                cmdset_json_count: int) -> None:
    """CS-A2: count equality."""
    if intents_yaml_count != cmdset_json_count:
        raise CsAssertionError(
            "CS-A2: intents.yaml has %d entries, cmdset_18.json has %d "
            "(counts must match)"
            % (intents_yaml_count, cmdset_json_count))


def check_cs_a3(mission_alternation: List[str],
                cmdset_closed_set: FrozenSet[str],
                mission_name: str = "") -> List[str]:
    """CS-A3: alternation MUST be a subset of the closed set.

    Returns the list of dropped intents (transitional warn mode).
    In strict mode a caller would raise on non-empty return."""
    return [i for i in mission_alternation if i not in cmdset_closed_set]


def check_cs_a4(mission_name: str,
                alternation_size: int) -> None:
    """CS-A4: alternation_size + 1 (unknown) <= 5, with one
    documented break: M4_follow = 6."""
    limit = 5
    if mission_name == "M4_follow":
        limit = 6
    if alternation_size + 1 > limit:
        raise CsAssertionError(
            "CS-A4: mission %s alternation=%d + 1 (unknown) > %d "
            "(AI-36 limit; only M4_follow allowed to break at 6)"
            % (mission_name, alternation_size, limit))
