"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_id.py
Brief: ID-2 -- the geographic-object id shape, and the validator for it

Description:
What this solves. 16 S5.3 ID-2 fixes one shape for every geographic-object id
the system names -- a route, a waypoint, a fence, a dock, a preset phrase, an
event: a single-letter prefix, a hyphen, then a slug. The v0.1 form route_1 /
route_east_gate is wholesale invalid, and 16 S7.0.2 uses that exact form as its
worked example of an I3 defect -- a silent rewrite that the S8 seven-point
value checks can never catch, because a well-formed-but-wrong id passes every
one of them. The only place it can be caught is at the shape, which is here.

Why this is a validator and not a property of intents.yaml. The registry
(intents.py) holds slot NAMES, never slot VALUES, so no geographic id appears in
it -- ID-1/ID-3 are about the registry, ID-2 is about the id strings that flow
through the few-shot examples (16 S6.7.x), the mission enumeration emitters and,
at run time, the slot values a model fills. This module is the one definition of
"is this a legal geo id" those callers share. Kept next to the registry because
the three ID rules are one family (16 S5.3) and a reader looking for ID-2 will
look here first.

Why the prefix set lives here and not in xbrain/common/enums/. That package
mirrors the contract's closed sets from 11 S13 and nothing else, so a metatest
can diff it back against 11. This set is defined by 16 S5.3, which 11 does not
own; adding it there would make the metatest compare 11 against something 11
never said (the same reasoning config/loader.py gives for HISTORY_SCENARIOS).
It is written down here with its anchor instead.

A note on what looks right but is wrong. Do NOT relax the hyphen: route_1 and
route_east_gate both begin with the letter r, and a checker that accepted r
followed by anything would pass both -- which is the precise mistake ID-2
exists to stop. The character after the prefix MUST be the hyphen; r-east_gate
is legal, route_east_gate is not, and they differ by exactly that.
"""

import re
from typing import Tuple

from ...common.errors import E_SCHEMA
from ...common.errors.exceptions import XbrainError

#: The six geographic-object id prefixes, verbatim from 16 S5.3 ID-2:
#:   r- 路径 (route) . w- 位置点 (waypoint) . f- 围栏 (fence)
#:   d- 充电桩 (dock) . p- 预设语句 (preset) . e- 事件 (event)
#:
#: A tuple, not a set: it is printed in the failure message and an unordered set
#: would print in a different order from run to run, making two identical
#: failures look like two different ones. Membership is not on any hot path here,
#: so the tuple costs nothing.
GEO_ID_PREFIXES: Tuple[str, ...] = ("r", "w", "f", "d", "p", "e")

#: The full shape. Anchored at both ends so a trailing or leading stray
#: character is a rejection, not a partial match. The slug is [a-z0-9_]+ verbatim
#: from the 16 S5.3 ID-2 / GWY-P4-07 criterion 2 regex -- lower-case, digits and
#: underscore, at least one character. Uppercase is deliberately excluded: the
#: ids are machine-generated slugs, and allowing case would let r-Main and
#: r-main name what a human reads as one place and the store treats as two.
_GEO_ID_RE = re.compile(r"^(?:" + "|".join(GEO_ID_PREFIXES) + r")-[a-z0-9_]+$")


class GeoIdError(XbrainError):
    """A string offered as a geographic-object id that is not one.

    Carries E_SCHEMA, the closed-set code from the shared library (never a
    literal, CLAUDE.md 3.5): a malformed id arriving on a boundary is a schema
    violation of the slot that should have held a <prefix>-<slug>, and 11 S13.6
    forbids passing it through or repairing it to something nearby.
    """

    def __init__(self, value: str):
        # The message names the value and the rule, so an operator reading the
        # log knows both what was rejected and why -- not just "bad id".
        super().__init__(
            "geographic-object id %r does not match ID-2 shape "
            "<prefix>-<slug> with prefix in %s and slug [a-z0-9_]+ "
            "(16 S5.3). route_1 / route_east_gate style is invalid: the "
            "character after the prefix must be a hyphen"
            % (value, list(GEO_ID_PREFIXES)))
        self.code = E_SCHEMA
        self.value = value


def is_valid_geo_object_id(value: object) -> bool:
    """True iff value is a well-formed geographic-object id (ID-2).

    A pure predicate, for callers that want to branch rather than raise -- a
    few-shot linter counting offenders, a mission builder filtering a candidate
    list. Non-str inputs return False rather than raising: the question "is this
    a geo id" has a defensible answer (no) for a number or None, and forcing the
    caller to guard the type first would just move the same check outward.
    """
    # isinstance first: re.match on a non-str raises TypeError, and a predicate
    # named is_valid_* must answer the question, not explode on an odd input.
    if not isinstance(value, str):
        return False
    return _GEO_ID_RE.match(value) is not None


def validate_geo_object_id(value: str) -> str:
    """Return value if it is a legal geo id, else raise GeoIdError.

    The boundary form, welded to the assignment site the way enums.parse() is:
        target = validate_geo_object_id(slots["waypoint"])
    so the check cannot be dropped by a later edit without the assignment
    noticing. Returns the input unchanged -- it is a validator, not a
    normaliser: it does not lower-case, trim or repair, because a tolerant
    version lets r-Main and r-main coexist across processes until the day they
    must match (the same discipline enums.ClosedSet.parse states for wire
    values).
    """
    if not is_valid_geo_object_id(value):
        raise GeoIdError(value if isinstance(value, str) else repr(value))
    return value
