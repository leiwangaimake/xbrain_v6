"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: safety_keyword_gate.py
Brief: CHK-1-29 P4 tier-1 keyword classifier covers 7 safety-critical intents

Description:
16 §5.3 requires the tier-1 (keyword + rules) classifier to
recognise ALL seven safety-critical intents. These MUST land at
tier-1 (llm_request_count == 0) and complete in <1 ms:

  B09  emergency stop / trigger E-stop
  C01  hold position
  C03  slow to creep
  C04  stop task immediately
  B07  drop safe-mode
  D12  speak_stop (stop broadcasting)
  D13  cancel current alarm

The seven-item list is NOT hand-maintained here as a second copy.
It's PROJECTED from intents.yaml at process startup by taking
{route: fastpath} ∩ {safety_critical: true}. This module owns the
projection + a meta-test guard that the resulting set equals the
frozen SAFETY_INTENT_IDS below (bidirectional).

Rationale: a spec change that adds an eighth safety intent to
16 §5.3 without matching intents.yaml -> projection diverges from
SAFETY_INTENT_IDS -> the guard reddens BEFORE the tier-1 classifier
silently misroutes to LLM.
"""

from __future__ import annotations

from typing import Iterable, Set


SAFETY_INTENT_IDS = frozenset({
    "B09", "C01", "C03", "C04", "B07", "D12", "D13",
})


class SafetyRoutingViolation(Exception):
    """A safety-critical intent slipped past the tier-1 classifier
    into an LLM route -- either a coding bug or an intents.yaml
    drift."""


def project_safety_ids_from_intents_yaml(intents: Iterable[dict]) -> Set[str]:
    """Given the parsed intents.yaml entries (each a dict with 'id',
    'route', 'safety_critical'), return the set of ids that match
    fastpath + safety_critical."""
    out = set()
    for it in intents:
        if it.get("route") == "fastpath" and it.get("safety_critical"):
            out.add(it["id"])
    return out


def assert_projection_matches_frozen(intents: Iterable[dict]) -> None:
    """Meta-check: the projection MUST equal the frozen set.
    Bidirectional diff empty."""
    projected = project_safety_ids_from_intents_yaml(intents)
    missing_in_yaml = SAFETY_INTENT_IDS - projected
    extra_in_yaml = projected - SAFETY_INTENT_IDS
    if missing_in_yaml or extra_in_yaml:
        raise SafetyRoutingViolation(
            "safety-critical intent projection diverged from spec: "
            f"missing_in_yaml={sorted(missing_in_yaml)}, "
            f"extra_in_yaml={sorted(extra_in_yaml)}")


def check_intent_is_fastpath(intent_id: str,
                               classified_route: str,
                               llm_request_count: int) -> None:
    """Runtime check: safety intent must land in fastpath with
    zero LLM calls. Called at the tier-1 -> route emission point."""
    if intent_id not in SAFETY_INTENT_IDS:
        return   # not our concern
    if classified_route != "fastpath":
        raise SafetyRoutingViolation(
            f"safety intent {intent_id!r} classified to "
            f"{classified_route!r}, expected 'fastpath'")
    if llm_request_count != 0:
        raise SafetyRoutingViolation(
            f"safety intent {intent_id!r} triggered {llm_request_count} "
            f"LLM request(s); tier-1 must complete without LLM")
