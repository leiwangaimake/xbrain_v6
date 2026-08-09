"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: checks.py
Brief: GWY-P4-14 -- 7 deterministic validation rules + single exit table

Description:
16 S8 checks run AFTER GBNF-constrained decode but BEFORE dispatch.
Any check fails -> intent rejected with a specific error code.

Seven rules per 16 S8:
  V1  slot value in the closed set the grammar declared
  V2  numeric slots within declared range (e.g., 0 <= volume <= 100)
  V3  every REQUIRED slot present (no missing keys)
  V4  no EXTRA slots (LLM invented a slot not in the schema)
  V5  slot type matches declaration (int vs str vs bool)
  V6  cross-slot invariants (e.g., duration end > start)
  V7  intent name matches an entry in cmdset_18.json

Every failure returns ValidationResult(accepted=False, code=X). One
'exit table' per rule -> one code (not aggregation): the first
failing rule short-circuits and names the specific rule.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, FrozenSet, Optional


class ValidationRule:
    V1 = "V1_slot_value_out_of_closed_set"
    V2 = "V2_numeric_out_of_range"
    V3 = "V3_required_slot_missing"
    V4 = "V4_extra_slot"
    V5 = "V5_slot_type_mismatch"
    V6 = "V6_cross_slot_invariant"
    V7 = "V7_intent_not_in_cmdset"


@dataclass(frozen=True)
class ValidationResult:
    accepted: bool
    code: str = ""       # one of ValidationRule.V* on reject
    detail: str = ""


@dataclass
class SlotSchema:
    """Per-intent slot schema."""
    required: FrozenSet[str]
    all_slots: FrozenSet[str]
    types: Dict[str, type]            # slot name -> expected type
    closed_sets: Dict[str, FrozenSet[str]]   # slot name -> allowed values
    numeric_ranges: Dict[str, tuple]         # slot name -> (min, max)


def validate(
    intent: str,
    slots: Dict[str, Any],
    schema: SlotSchema,
    cmdset_names: FrozenSet[str],
) -> ValidationResult:
    """Run V7 -> V4 -> V3 -> V5 -> V1 -> V2 order (fail-fast, first
    error wins). V6 caller-supplied if applicable."""
    # V7 first: unknown intent is the most severe -- no schema even
    # applies to the value.
    if intent not in cmdset_names:
        return ValidationResult(
            accepted=False, code=ValidationRule.V7,
            detail="intent %r not in cmdset_18.json" % intent)

    # V4 extra slots (before V3 so 'extra AND missing' fails on extra).
    extras = set(slots.keys()) - schema.all_slots
    if extras:
        return ValidationResult(
            accepted=False, code=ValidationRule.V4,
            detail="extra slot(s) %s" % sorted(extras))

    # V3 required present.
    missing = schema.required - set(slots.keys())
    if missing:
        return ValidationResult(
            accepted=False, code=ValidationRule.V3,
            detail="required slot(s) missing: %s" % sorted(missing))

    # V5 types.
    for name, val in slots.items():
        expected = schema.types.get(name)
        if expected and not isinstance(val, expected):
            return ValidationResult(
                accepted=False, code=ValidationRule.V5,
                detail="slot %r expected %s, got %s"
                       % (name, expected.__name__, type(val).__name__))

    # V1 closed-set values.
    for name, val in slots.items():
        cs = schema.closed_sets.get(name)
        if cs and val not in cs:
            return ValidationResult(
                accepted=False, code=ValidationRule.V1,
                detail="slot %r value %r not in closed set %s"
                       % (name, val, sorted(cs)))

    # V2 numeric ranges.
    for name, val in slots.items():
        rng = schema.numeric_ranges.get(name)
        if rng and isinstance(val, (int, float)):
            lo, hi = rng
            if val < lo or val > hi:
                return ValidationResult(
                    accepted=False, code=ValidationRule.V2,
                    detail="slot %r value %r out of range [%s, %s]"
                           % (name, val, lo, hi))

    return ValidationResult(accepted=True)
