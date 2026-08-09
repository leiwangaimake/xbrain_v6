"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: whitelist_gate.py
Brief: BIZ-P2-0 -- publisher / subscriber whitelist gate at startup

Description:
Compares the SET of Zenoh keys P2 actually declared (via P2Publisher
+ P2Subscriber) against P2_CORE_PUB / P2_CORE_SUB from
xbrain/common/zenoh/whitelists.py at Stage 3 startup, before P2's
main loop begins. Any key P2 declared that is NOT in the whitelist
raises WhitelistViolation -- the process refuses to start.

This is the P2-side executable form of the DB1-1 constraint
"p2_core never publishes cmd/task": if someone accidentally writes
publisher.declare('cmd/task') the gate refuses the whole boot rather
than letting a stray publish reach the wire and corrupt P3's
subscriber budget (11 S1.1.6 verbatim).

Direction of the check:
  * declared_pubs SUBSET-OF P2_CORE_PUB    (raises on extras)
  * declared_subs SUBSET-OF P2_CORE_SUB    (raises on extras)
  * The REVERSE direction (whitelist has keys P2 did NOT declare) is
    NOT enforced here: some registered keys are conditional (they
    activate only in certain modes), so a missing declaration is not
    a bug per se. That check lives in the per-mode BIT (BIZ-P2-23).

Key normalisation:
  * Wildcard keys like `state/arb/{domain}` are matched by prefix +
    template-slot pattern equivalence, not by literal string. A
    real declared key `state/arb/motion` matches the template
    `state/arb/{domain}`. Any key that doesn't match either a literal
    or a template in the whitelist is a violation.
"""

from __future__ import annotations

import re
from typing import Iterable, List, Set, Tuple

from xbrain.common.errors.exceptions import XbrainError
from xbrain.common.zenoh.whitelists import P2_CORE_PUB, P2_CORE_SUB


class WhitelistViolation(XbrainError):
    """A publisher or subscriber key was declared that is NOT in the
    P2 whitelist. Raise-only at Stage 3 startup; do not let the
    process transition to RUNNING with an unwhitelisted key alive."""


def _template_to_regex(template: str) -> re.Pattern:
    """Turn `state/arb/{domain}` into a regex matching `state/arb/motion`
    (any single path segment)."""
    # Escape the literal parts, replace {word} placeholders with [^/]+.
    parts = re.split(r"(\{[a-z_][a-z0-9_]*\})", template)
    out = []
    for p in parts:
        if p.startswith("{") and p.endswith("}"):
            out.append(r"[^/]+")
        else:
            out.append(re.escape(p))
    return re.compile("^" + "".join(out) + "$")


def _split_whitelist(items: Iterable[str]) -> Tuple[Set[str], List[re.Pattern]]:
    """Split a whitelist into literal keys and template regexes.

    Literal keys: exact string match.
    Templates:    regexes (compiled once) that match a live key with
                  path-segment substitution."""
    literals: Set[str] = set()
    templates: List[re.Pattern] = []
    for key in items:
        if "{" in key:
            templates.append(_template_to_regex(key))
        else:
            literals.add(key)
    return literals, templates


def _match_key(key: str, literals: Set[str],
               templates: List[re.Pattern]) -> bool:
    if key in literals:
        return True
    for pat in templates:
        if pat.match(key):
            return True
    return False


def check_pub_keys(declared: Iterable[str]) -> None:
    """Raise WhitelistViolation if any declared publisher key is
    outside P2_CORE_PUB (literal or template-matched)."""
    literals, templates = _split_whitelist(P2_CORE_PUB)
    bad: List[str] = []
    for k in declared:
        if not _match_key(k, literals, templates):
            bad.append(k)
    if bad:
        raise WhitelistViolation(
            "E_CONFIG_INVALID",
            "p2_core declared publisher key(s) NOT in P2_CORE_PUB "
            "whitelist: %s. Registered pubs are: %s (see 11 S1.1.6 + "
            "xbrain/common/zenoh/whitelists.py)"
            % (sorted(bad), sorted(P2_CORE_PUB)),
        )


def check_sub_keys(declared: Iterable[str]) -> None:
    """Raise WhitelistViolation if any declared subscriber key is
    outside P2_CORE_SUB."""
    literals, templates = _split_whitelist(P2_CORE_SUB)
    bad: List[str] = []
    for k in declared:
        if not _match_key(k, literals, templates):
            bad.append(k)
    if bad:
        raise WhitelistViolation(
            "E_CONFIG_INVALID",
            "p2_core declared subscriber key(s) NOT in P2_CORE_SUB "
            "whitelist: %s. Registered subs are: %s"
            % (sorted(bad), sorted(P2_CORE_SUB)),
        )
