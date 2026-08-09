"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: assembler.py
Brief: GWY-P4-10 -- 4-layer prompt assembler + trim sequence + history

Description:
16 S6 4-layer prompt structure:
  L1 fixed system persona (never trimmed)
  L2 mission block (M1..M11 depending on classified mission)
  L3 few-shot examples for current mission (trimmable)
  L4 history (trimmable per policy)

Trim order (16 S6.5) when tokens exceed budget:
  history_tail -> few_shots_tail -> few_shots_head -> mission_body

History policy (16 S14 prompt.history.enable_on):
  Three closed-set scenarios: [] (all off) | ['clarify'] |
  ['clarify','recent']. Unknown values REFUSE STARTUP (§14).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import FrozenSet, List, Optional


_HISTORY_SCENARIOS: FrozenSet[str] = frozenset({"clarify", "recent"})


class PromptSchemaError(RuntimeError):
    """history.enable_on had an unknown value or list shape wrong."""


def check_history_enable_on(value: object) -> None:
    """16 S14: enable_on MUST be a list; entries MUST be in
    {clarify, recent}. Empty list = all off (valid)."""
    if not isinstance(value, list):
        raise PromptSchemaError(
            "history.enable_on must be a list (empty for all off); got %s"
            % type(value).__name__)
    for v in value:
        if v not in _HISTORY_SCENARIOS:
            raise PromptSchemaError(
                "history.enable_on entry %r not in %s"
                % (v, sorted(_HISTORY_SCENARIOS)))


@dataclass
class PromptLayers:
    """The four layers before assembly."""
    system: str = ""
    mission: str = ""
    few_shots: List[str] = field(default_factory=list)
    history: List[str] = field(default_factory=list)

    def total_char_count(self) -> int:
        return (len(self.system) + len(self.mission)
                + sum(len(s) for s in self.few_shots)
                + sum(len(h) for h in self.history))


def trim_to_budget(layers: PromptLayers, char_budget: int) -> PromptLayers:
    """Apply the trim sequence (history_tail -> few_shots_tail ->
    few_shots_head -> mission_body -- system NEVER trimmed) until
    total is within budget.

    Returns a NEW PromptLayers; input is untouched.
    """
    hist = list(layers.history)
    shots = list(layers.few_shots)
    mission = layers.mission
    def total() -> int:
        return (len(layers.system) + len(mission)
                + sum(len(s) for s in shots)
                + sum(len(h) for h in hist))
    # Step 1: pop history from the tail.
    while total() > char_budget and hist:
        hist.pop()
    # Step 2: pop few_shots from the tail.
    while total() > char_budget and len(shots) > 1:
        shots.pop()
    # Step 3: pop few_shots from the head (keep at least 1).
    while total() > char_budget and len(shots) > 1:
        shots.pop(0)
    # Step 4: trim mission (last resort; may leave prompt useless
    # but keeps under budget).
    if total() > char_budget:
        excess = total() - char_budget
        mission = mission[:max(0, len(mission) - excess)]
    return PromptLayers(
        system=layers.system,
        mission=mission,
        few_shots=shots,
        history=hist,
    )


def assemble(layers: PromptLayers) -> str:
    """Concatenate the four layers into one prompt string.

    Layout: system \n\n mission \n\n [few_shot_i \n]* \n history_i \n]*
    """
    parts = []
    if layers.system:
        parts.append(layers.system)
    if layers.mission:
        parts.append(layers.mission)
    parts.extend(layers.few_shots)
    parts.extend(layers.history)
    return "\n\n".join(p for p in parts if p)
