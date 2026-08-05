"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: The non-error closed sets, exported for every XBRAIN runtime process

Description:
Six sets live here: plane, domain, event_category, gate_limiter, stop_reason,
task_state. CLAUDE.md 3.5 forbids their literals outside this package for the
same reason as the error codes -- a value spelled differently in two processes
compiles, runs, and only surfaces during integration.

★★★ Out-of-set values raise ClosedSetViolation. 11 S13.6 requires it in so many
words: no silent pass-through, and no "interpret the unknown value as something
close". The contract even names the tempting version it forbids -- degrading an
unrecognised PTZ action to jog -- because that turns a contract violation into
motion the operator did not ask for.

★ Two of the sets are ORDERED, not just membership: gate_limiter's order is the
attribution priority (estop wins over everything) and stop_reason's order is the
decision order of 11 S9.12.2. Losing the order silently changes which cause gets
reported, so the metatest checks sequence for those two, not just membership.
"""

import os
from typing import Dict, FrozenSet, List, Tuple

from ..errors.exceptions import ClosedSetViolation

_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sets.yaml")


class ClosedSet:
    """One closed set, with the contract location it was generated from."""

    __slots__ = ("name", "source", "anchor", "note", "ordered", "_values", "_frozen")

    def __init__(self, name: str, source: str, anchor: str, note: str,
                 ordered: bool, values: List[str]):
        self.name = name
        self.source = source
        self.anchor = anchor
        self.note = note
        self.ordered = ordered
        self._values = tuple(values)
        self._frozen = frozenset(values)

    @property
    def values(self) -> Tuple[str, ...]:
        """In contract order. Meaningful when .ordered is True."""
        return self._values

    def __contains__(self, v: object) -> bool:
        return v in self._frozen

    def __iter__(self):
        return iter(self._values)

    def __len__(self) -> int:
        return len(self._values)

    def parse(self, v: str) -> str:
        """Return v if it is in the set, else raise.

        🚫 Never add a fallback here. 11 S13.6 forbids both silent pass-through
        and degrading to a nearby value; either one hides a contract violation
        behind something that looks like normal operation.
        """
        if v not in self._frozen:
            raise ClosedSetViolation(self.name, v)
        return v

    def index(self, v: str) -> int:
        """Position in contract order -- the priority for gate_limiter.

        Raises on an unordered set rather than returning a meaningless number:
        a caller asking for the priority of a plane has misunderstood something,
        and a silent answer would let that misunderstanding propagate.
        """
        if not self.ordered:
            raise ValueError(f"closed set {self.name!r} carries no meaningful order")
        return self._values.index(self.parse(v))


def _load() -> Dict[str, ClosedSet]:
    """Parse sets.yaml without a yaml dependency.

    This package is imported by every runtime process, including ones that start
    before any virtualenv is guaranteed, so it deliberately has no third-party
    imports. It raises on any line it does not recognise rather than skipping it:
    a loader that skips silently shrinks the closed set, which is the exact
    failure this module exists to prevent. That is not hypothetical -- the
    generator for this file first dropped nine of task_state's twelve values
    because its row filter treated an empty leading cell as a separator row.
    """
    out: Dict[str, ClosedSet] = {}
    cur: Dict[str, object] = {}
    in_sets = False
    reading_values = False

    def flush():
        if cur:
            out[cur["name"]] = ClosedSet(
                str(cur["name"]), str(cur.get("source", "")), str(cur.get("anchor", "")),
                str(cur.get("note", "")), bool(cur.get("ordered")),
                list(cur.get("values", [])),
            )

    with open(_YAML, encoding="utf-8") as fh:
        for lineno, raw in enumerate(fh, 1):
            line = "" if raw.lstrip().startswith("#") else raw.rstrip()
            if not line.strip():
                continue
            if line.rstrip() == "sets:":
                in_sets = True
                continue
            if not in_sets:
                continue
            indent = len(line) - len(line.lstrip())
            body = line.strip()
            if indent == 2 and body.endswith(":"):
                flush()
                cur = {"name": body[:-1], "values": []}
                reading_values = False
                continue
            if indent == 4:
                if body == "values:":
                    reading_values = True
                    continue
                reading_values = False
                if ":" not in body:
                    raise ValueError(f"{_YAML}:{lineno}: unparsable line {body!r}")
                k, v = body.split(":", 1)
                v = v.split("#", 1)[0].strip().strip('"')
                cur[k.strip()] = (v == "true") if k.strip() == "ordered" else v
                continue
            if indent == 6 and reading_values and body.startswith("- "):
                cur["values"].append(body[2:].strip())  # type: ignore[union-attr]
                continue
            raise ValueError(f"{_YAML}:{lineno}: unexpected line {line!r}")
    flush()
    if not out:
        raise ValueError(f"{_YAML}: no sets parsed")
    return out


_SETS: Dict[str, ClosedSet] = _load()

#: Names of every closed set defined here. 🚫 Never write the count (3.7).
SET_NAMES: FrozenSet[str] = frozenset(_SETS)

PLANE = _SETS["plane"]
DOMAIN = _SETS["domain"]
EVENT_CATEGORY = _SETS["event_category"]
GATE_LIMITER = _SETS["gate_limiter"]
STOP_REASON = _SETS["stop_reason"]
TASK_STATE = _SETS["task_state"]

__all__ = ["ClosedSet", "ClosedSetViolation", "SET_NAMES", "get", "parse_enum",
           "PLANE", "DOMAIN", "EVENT_CATEGORY", "GATE_LIMITER", "STOP_REASON",
           "TASK_STATE"]


def get(name: str) -> ClosedSet:
    """The set by name. Raises on an unknown set name, not just an unknown value."""
    try:
        return _SETS[name]
    except KeyError:
        raise ClosedSetViolation("__set_name__", name) from None


def parse_enum(set_name: str, value: str) -> str:
    """Validate value against the named set. Raises ClosedSetViolation otherwise."""
    return get(set_name).parse(value)
