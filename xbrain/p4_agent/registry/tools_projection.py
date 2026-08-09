"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: tools_projection.py
Brief: GWY-P4-27 -- tools table projected from intents.yaml (<= 5 per mission)

Description:
16 S10 tools: LLM sees them as callable functions. The tools table
is DERIVED from intents.yaml (per-mission projection) rather than
hand-authored -- hand version drifts.

Three consistency assertions:
  T-1  tools set per mission is SUBSET of the mission's intent
       alternation (no tool without a matching intent)
  T-2  tools per mission <= 5 (AI-36 limit; M4_follow allowed 6)
  T-3  each tool's JSON schema slot names == intents.yaml slots
       for that intent (schema drift is the whole point of the
       projection)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, FrozenSet, List, Set


class ToolProjectionError(RuntimeError):
    """T-1/T-2/T-3 violation."""


def check_t1_tools_subset_of_alternation(
    mission_name: str,
    tools_projected: FrozenSet[str],
    alternation: FrozenSet[str],
) -> None:
    extras = tools_projected - alternation
    if extras:
        raise ToolProjectionError(
            "T-1 (%s): tool(s) %s have no matching intent in alternation %s"
            % (mission_name, sorted(extras), sorted(alternation)))


def check_t2_max_five_tools(mission_name: str, count: int) -> None:
    limit = 6 if mission_name == "M4_follow" else 5
    if count > limit:
        raise ToolProjectionError(
            "T-2 (%s): %d tools > %d (AI-36 limit)"
            % (mission_name, count, limit))


def check_t3_schema_slots_match(
    intent_name: str,
    tool_schema_slots: FrozenSet[str],
    intent_slots: FrozenSet[str],
) -> None:
    if tool_schema_slots != intent_slots:
        raise ToolProjectionError(
            "T-3 (%s): tool schema slots %s != intent slots %s"
            % (intent_name, sorted(tool_schema_slots), sorted(intent_slots)))
