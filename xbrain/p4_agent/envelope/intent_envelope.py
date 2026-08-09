"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: intent_envelope.py
Brief: GWY-P4-13 -- IntentEnvelope + 7 structural invariants EV-1..EV-7

Description:
16 S8 IntentEnvelope carries one classified intent + slots from
P4 to the appropriate consumer (P2 for cmd/motion/intent, P3 for
cmd/task/create, etc.). Seven invariants:

  EV-1 required fields present (id/intent/route/auth/slots/level/cmd_id)
  EV-2 route in closed set {fastpath, llm, bypass, fastpath_then_llm}
  EV-3 auth in closed set {L0, L1a, L1b, L2, L3}
  EV-4 level in closed set {L0, L1a, L1b, L2, L3} (routing level;
       may differ from auth on transition -- see 16 CL-*)
  EV-5 cmd_id UUID-shaped (hex)
  EV-6 slots MUST be a mapping (dict), NEVER a list
  EV-7 latency_class in closed set + consistent with route (see P4-26)
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, Optional


_ROUTE_SET: FrozenSet[str] = frozenset({
    "fastpath", "llm", "bypass", "fastpath_then_llm",
})
_AUTH_SET: FrozenSet[str] = frozenset({"L0", "L1a", "L1b", "L2", "L3"})
_LEVEL_SET: FrozenSet[str] = _AUTH_SET
_LATENCY_CLASS: FrozenSet[str] = frozenset({
    "fastpath", "llm", "bypass",
})

_UUID_HEX_RE = re.compile(r"^[0-9a-f]{8}(-?[0-9a-f]{4}){3}-?[0-9a-f]{12}$")


class EnvelopeSchemaError(RuntimeError):
    """One of EV-1..EV-7 failed."""


@dataclass(frozen=True)
class IntentEnvelope:
    """The wire shape flowing from P4 to consumers."""
    id: str                   # 18 S13.1 id (A05, E01, ...)
    intent: str               # intent name (move_forward, ...)
    route: str                # closed set
    auth: str                 # closed set
    level: str                # closed set
    slots: Dict[str, Any]     # NEVER a list
    cmd_id: str               # UUID hex
    latency_class: str        # closed set + route-consistent

    def __post_init__(self) -> None:
        _check_required(self)
        _check_closed_sets(self)
        _check_slots_dict(self)
        _check_uuid(self)
        _check_latency_class_matches_route(self)


def _check_required(e: "IntentEnvelope") -> None:
    for f in ("id", "intent", "route", "auth", "level",
              "slots", "cmd_id", "latency_class"):
        v = getattr(e, f)
        if v is None or (isinstance(v, str) and not v):
            raise EnvelopeSchemaError(
                "EV-1: field %r missing or empty" % f)


def _check_closed_sets(e: "IntentEnvelope") -> None:
    if e.route not in _ROUTE_SET:
        raise EnvelopeSchemaError(
            "EV-2: route %r not in %s" % (e.route, sorted(_ROUTE_SET)))
    if e.auth not in _AUTH_SET:
        raise EnvelopeSchemaError(
            "EV-3: auth %r not in %s" % (e.auth, sorted(_AUTH_SET)))
    if e.level not in _LEVEL_SET:
        raise EnvelopeSchemaError(
            "EV-4: level %r not in %s" % (e.level, sorted(_LEVEL_SET)))
    if e.latency_class not in _LATENCY_CLASS:
        raise EnvelopeSchemaError(
            "EV-7: latency_class %r not in %s"
            % (e.latency_class, sorted(_LATENCY_CLASS)))


def _check_slots_dict(e: "IntentEnvelope") -> None:
    if not isinstance(e.slots, dict):
        raise EnvelopeSchemaError(
            "EV-6: slots MUST be a dict; got %s" % type(e.slots).__name__)


def _check_uuid(e: "IntentEnvelope") -> None:
    if not _UUID_HEX_RE.match(e.cmd_id.lower().replace("-", "")):
        # Try again with dashes preserved (pattern above tolerates both).
        if not _UUID_HEX_RE.match(e.cmd_id.lower()):
            raise EnvelopeSchemaError(
                "EV-5: cmd_id %r not UUID-shaped hex" % e.cmd_id)


def _check_latency_class_matches_route(e: "IntentEnvelope") -> None:
    """EV-7 additional: latency_class must be consistent with route.
      fastpath route         -> latency_class = fastpath
      llm route              -> latency_class = llm
      bypass route           -> latency_class = bypass
      fastpath_then_llm      -> latency_class = fastpath (the LLM leg
                                is parallel and doesn't block the
                                fastpath dispatch that is latency-
                                critical)
    """
    expected = {
        "fastpath": "fastpath",
        "llm": "llm",
        "bypass": "bypass",
        "fastpath_then_llm": "fastpath",
    }.get(e.route)
    if expected and e.latency_class != expected:
        raise EnvelopeSchemaError(
            "EV-7 consistency: route=%s expects latency_class=%s; got %s"
            % (e.route, expected, e.latency_class))
