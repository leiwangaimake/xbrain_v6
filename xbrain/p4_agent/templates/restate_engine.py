"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: restate_engine.py
Brief: GWY-P4-16 -- restate_templates.yaml (L1a/L1b) + RS-1..RS-4

Description:
16 S8.5 restate templates. When P4 executes an L1a/L1b command,
it plays TTS restating what was HEARD before executing (L1a) or
what WILL be executed (L1b pre-announce).

RS-1: numeric values MUST include the word "请求" (request); otherwise
      operator can't tell if the number is 'what I said' or 'what
      actually happened'
RS-2: template MUST start with an ACTION verb (per CMD-38); trailing
      constraints go LAST
RS-3: placeholder MUST come from either the intent slots OR the
      state snapshot; unknown placeholder -> refuse to load
RS-4: 'applied != requested' after execution MUST trigger a
      restate correction (applied X but you asked Y)
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import FrozenSet, Set


class RestateSchemaError(RuntimeError):
    """restate_templates.yaml violated RS-1..RS-4."""


_PLACEHOLDER_RE = re.compile(r"\{([a-z_][a-z0-9_]*)\}")


def check_rs1_numeric_uses_request_word(text: str,
                                         has_numeric_placeholder: bool) -> None:
    """RS-1: numeric restate MUST include '请求' so operator can
    distinguish requested from applied."""
    if has_numeric_placeholder and "请求" not in text:
        raise RestateSchemaError(
            "RS-1: template %r has numeric placeholder but does not "
            "contain '请求' (operator can't tell requested vs applied)"
            % text)


def check_rs2_starts_with_action(text: str, action_verbs: FrozenSet[str]) -> None:
    """RS-2: text MUST start with one of the declared action verbs
    (CMD-38 convention)."""
    for verb in action_verbs:
        if text.startswith(verb):
            return
    raise RestateSchemaError(
        "RS-2: template %r does NOT start with a known action verb "
        "(one of %s required)" % (text, sorted(action_verbs)))


def check_rs3_placeholders_available(
    text: str,
    slot_names: FrozenSet[str],
    state_names: FrozenSet[str],
) -> None:
    """RS-3: every {placeholder} MUST resolve from slot OR state."""
    placeholders = set(_PLACEHOLDER_RE.findall(text))
    allowed = slot_names | state_names
    unknown = placeholders - allowed
    if unknown:
        raise RestateSchemaError(
            "RS-3: template %r references unknown placeholder(s) %s "
            "(known slots: %s, known state: %s)"
            % (text, sorted(unknown), sorted(slot_names),
               sorted(state_names)))


def needs_rs4_correction(requested_value, applied_value) -> bool:
    """RS-4: if applied != requested, correction TTS must fire."""
    return requested_value != applied_value
