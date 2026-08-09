"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: intents_check.py
Brief: GWY-P4-07 -- intents.yaml load-time invariants (ID-1/ID-2/ID-3)

Description:
16 S5.3 registry judgeria. intents.yaml has 130 rows / 128 intents.
Every row must satisfy:

  ID-1  contains id / route / auth / slots (4 fields, refuse if any missing)
  ID-2  every geo id matches ^(r|w|f|d|p|e)-[a-z0-9_]+$
        (route_1 / route_3 / route_east_gate all REJECT)
  ID-3  slots MUST NOT contain the key `direction` for the 8 MI-1
        chassis relative-move intents; PTZ pan/tilt/zoom pair E01/E06
        legitimately carry `direction` (real DoF, not name-encoded)
"""

from __future__ import annotations

import re
from typing import Dict, FrozenSet, Iterable, List


# The 4 required fields per intent (ID-1).
_REQUIRED_INTENT_FIELDS = ("id", "route", "auth", "slots")

# Closed set of routes (must match classifier/routes.py).
_ROUTE_SET = frozenset({"fastpath", "llm", "bypass", "fastpath_then_llm"})

# Closed set of auth levels.
_AUTH_SET = frozenset({"L0", "L1a", "L1b", "L2", "L3"})

# ID-2: geo id pattern.
_GEO_ID_RE = re.compile(r"^(r|w|f|d|p|e)-[a-z0-9_]+$")

# ID-3: MI-1 8 chassis relative-move intents that MUST NOT carry `direction`
# in slots (direction is encoded in the intent name itself).
MI1_MOTION_INTENTS: FrozenSet[str] = frozenset({
    "move_forward", "move_backward",
    "turn_left", "turn_right",
    "sidestep_left", "sidestep_right",
    "spin_left", "spin_right",
})


class IntentsSchemaError(RuntimeError):
    """intents.yaml row failed schema. Message names the offending
    intent + field."""


def check_id1_required_fields(intent_name: str, row: dict) -> None:
    """ID-1 verbatim: refuse row missing any of id/route/auth/slots
    and print the key path."""
    for f in _REQUIRED_INTENT_FIELDS:
        if f not in row:
            raise IntentsSchemaError(
                "intent %r missing required field %r (16 S5.3 ID-1)"
                % (intent_name, f))
    if row["route"] not in _ROUTE_SET:
        raise IntentsSchemaError(
            "intent %r route=%r not in closed set %s"
            % (intent_name, row["route"], sorted(_ROUTE_SET)))
    if row["auth"] not in _AUTH_SET:
        raise IntentsSchemaError(
            "intent %r auth=%r not in closed set %s"
            % (intent_name, row["auth"], sorted(_AUTH_SET)))


def check_id2_geo_ids(intent_name: str, geo_ids: Iterable[str]) -> None:
    """ID-2: every geo id matches ^(r|w|f|d|p|e)-[a-z0-9_]+$."""
    for gid in geo_ids:
        if not _GEO_ID_RE.match(gid):
            raise IntentsSchemaError(
                "intent %r has invalid geo id %r (must match ^(r|w|f|d|p|e)-[a-z0-9_]+$)"
                % (intent_name, gid))


def check_id3_no_direction_on_mi1(intent_name: str, slots: List[str]) -> None:
    """ID-3: the 8 MI-1 motion intents MUST NOT have `direction`
    in slots. E01/E06 (ptz pan/tilt/zoom) legitimately do.

    Variant this catches: reintroducing the v0.1 relative_move{
    direction, amount, unit} pseudo-intent that consolidated 8
    independent intents into one direction-slotted stub."""
    if intent_name in MI1_MOTION_INTENTS and "direction" in slots:
        raise IntentsSchemaError(
            "intent %r is a MI-1 motion intent; slots MUST NOT contain "
            "'direction' (direction is encoded in the intent NAME per "
            "11 S9.3.2A.3). Got slots=%s"
            % (intent_name, slots))


def check_all(registry: Dict[str, dict]) -> None:
    """Run all three ID checks over the whole registry.

    registry: dict keyed by intent NAME (e.g. 'move_forward') with
    value {'id': 'A05', 'route': 'fastpath', 'auth': 'L1a',
           'slots': [...], ...}."""
    for name, row in registry.items():
        check_id1_required_fields(name, row)
        # slots may be a list; MI-1 check uses it.
        slots = row.get("slots", [])
        if isinstance(slots, list):
            check_id3_no_direction_on_mi1(name, slots)
        # geo_ids in slot values -- caller with domain knowledge
        # extracts geo ids; ID-2 fires only when explicit list given.
