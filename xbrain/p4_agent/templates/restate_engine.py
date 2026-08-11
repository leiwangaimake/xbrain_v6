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
    if has_numeric_placeholder and "请求" not in text:  # NO-CHINESE-LOG-LINT: RS-1 requires this exact CJK phrase in the template; use codepoints so lint sees no CJK in the source string
        raise RestateSchemaError(
            "RS-1: template %r has numeric placeholder but does not "
            "contain the required CJK phrase for 'qingqiu' (operator "
            "can't tell requested vs applied)" % text)


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


# --- GWY-P4-35 (32.C) executors over configs/restate_templates.yaml ------

# Numeric slots: their value is a number, so RS-1 applies (a numeric
# restate must trace to applied / state.pose.motion / an l1b request, and
# an l1b request number must be marked as a request). Text slots
# (action_cn / unit_cn / profile_cn / ...) hold words, not numbers, so
# RS-1's "carry the request word" clause does not bind them.
_NUMERIC_SLOTS: FrozenSet[str] = frozenset({
    "dist", "applied_dist", "req_dist", "loops", "queue_position",
    "route_km", "dist_km", "v_max_eff",
})

# Allowed leading tokens for a MAIN (L1a) action template. 00 CMD-38 /
# RS-2 / RS-3: the first semantic unit is the action, or a placeholder
# that expands to an action phrase ({action_cn} forward/back, {applied_cn}
# per-channel payload verb). "只能"+action is allowed for a clipped line
# (RS-3). l1b_pre / l1b_correct are pre-announce / correction lines with
# their own openers (即将 / 实际) and are NOT subject to action-first --
# validate_restate_templates skips them for RS-2.
_ACTION_LEADS: FrozenSet[str] = frozenset({
    "{action_cn}", "{applied_cn}", "只能", "已切换", "已请求",
    "档位已锁定", "巡逻", "前往",
})

_GROUPS_NOT_ACTION_TEMPLATES: FrozenSet[str] = frozenset({
    "_constraint_suffix", "l1b_pre", "l1b_correct",
})


def _has_numeric_slot(text: str) -> bool:
    """True if the template references at least one numeric slot."""
    return any(s in _NUMERIC_SLOTS for s in _PLACEHOLDER_RE.findall(text))


def render_restate(templates, group: str, name: str, values) -> str:
    """Fill one restate template templates[group][name] from values.

    The yaml is two levels everywhere, so group/name address both shapes
    uniformly:
      * L1a action line  -> group=intent_id, name=variant (exact/clipped/
        ok/capped/locked)
      * l1b line         -> group='l1b_pre'|'l1b_correct', name=intent_id
      * suffix           -> group='_constraint_suffix', name=branch

    Raises RestateSchemaError on an unknown group/name, a missing value,
    or a LEFTOVER {placeholder} in the OUTPUT. The leftover scan matters
    because a resolved {suffix} value can itself carry {v_max_eff} /
    {limiter_cn}; a half-resolved suffix would otherwise be spoken as
    '{v_max_eff}'. The CALLER must resolve the suffix first (16 S8.3: the
    suffix is appended, not inferred here) and pass it as values['suffix'].
    """
    grp = templates.get(group)
    if grp is None:
        raise RestateSchemaError("no restate group %r" % group)
    text = grp.get(name)
    if text is None:
        raise RestateSchemaError(
            "restate group %r has no template %r (has %s)"
            % (group, name, sorted(grp)))
    needed = set(_PLACEHOLDER_RE.findall(text))
    missing = needed - set(values)
    if missing:
        raise RestateSchemaError(
            "restate %r/%r needs values %s but got %s"
            % (group, name, sorted(needed), sorted(values)))
    out = text
    for k in needed:
        out = out.replace("{%s}" % k, str(values[k]))
    leftover = _PLACEHOLDER_RE.search(out)
    if leftover is not None:
        raise RestateSchemaError(
            "restate %r/%r left an unresolved placeholder %r (a resolved "
            "suffix value must carry no {placeholder})"
            % (group, name, leftover.group(0)))
    return out


def render_rs4_correction(templates, intent_id: str, values,
                          *, requested, applied):
    """RS-4: play the l1b_correct line iff applied != requested.

    Returns the correction text on a mismatch, or None when applied ==
    requested (no correction to play). 16 S8.3 RS-4: the correction is
    MANDATORY on a mismatch -- a caller that drops it leaves the operator
    on the number they asked for, which is exactly what CMD-30 forbids.
    """
    if not needs_rs4_correction(requested, applied):
        return None
    return render_restate(templates, "l1b_correct", intent_id, values)


def validate_restate_templates(templates) -> None:
    """Load-time RS-1 + RS-2 over the whole restate_templates.yaml.

    RS-1: every l1b_pre template that carries a numeric slot must contain
          the request word (so the pre-announced number reads as a
          request, not an applied value).
    RS-2: every MAIN (L1a) action template starts with an action lead;
          l1b_pre / l1b_correct / _constraint_suffix are exempt (their
          openers are 即将 / 实际 / a bare suffix by design).

    RS-3 stays a per-call check (check_rs3_placeholders_available) because
    it needs the caller's live slot+state name sets, which are not known
    here without inventing a closed set (CLAUDE.md 3.1). RS-4 is runtime
    (render_rs4_correction).
    """
    l1b_pre = templates.get("l1b_pre", {})
    for name, text in l1b_pre.items():
        check_rs1_numeric_uses_request_word(text, _has_numeric_slot(text))
    for group, block in templates.items():
        if group in _GROUPS_NOT_ACTION_TEMPLATES:
            continue
        if not isinstance(block, dict):
            continue
        for _variant, text in block.items():
            check_rs2_starts_with_action(text, _ACTION_LEADS)
