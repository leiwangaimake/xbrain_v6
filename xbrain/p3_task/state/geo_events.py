"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_events.py
Brief: 11 S6.2 geo events -- geo CRUD audit -> event/{sev}/geo

Description:
The 11 S6.2 `geo` row is info/warn, channel normal, and is the audit trail of who
changed the map. The appliers (ingest/geo_write, ingest/geo_delete) already decide
WHICH audit record a command produced and hand back (sev, detail.type, detail)
triples on ApplyResult.events; this module is the PURE mapping from one such triple
to the (key, body) the wiring publishes -- the same split as task_events.

Two things the appliers deliberately do NOT do, and this module does:

  * severity is a property of the event type, not a choice. S6.2 splits the nine
    types into a fixed info half and a fixed warn half, so a triple whose sev
    disagrees with its type is a producer bug and is refused here rather than
    published -- an audit stream carrying a value the cloud cannot classify is
    harder to chase than a missing line.

  * `detail.type` is stamped here. The appliers fill the object type under
    `geo_type` (the sets.yaml closed-set name, common.enums.GEO_TYPE); S6.2 needs
    `type` to be the EVENT type. Stamping in one place keeps the two from being
    written half each and drifting apart.

S7.10 is explicit that this stream carries no synchronisation duty: nothing
downstream may rebuild geo state from these events, they exist for the HMI
activity list and the cloud audit.
"""

from __future__ import annotations

from typing import Any, Dict, Tuple


#: 11 S6.2 category for every event this module renders.
GEO_CATEGORY = "geo"

#: 11 S6.2 event/{sev}/geo detail.type closed set, info half + warn half.
#:
#: Not promoted into sets.yaml with the other geo sets: every member carries a
#: dot, and the shared extractor's VALUE_RE matches backticked [a-z_]+ only, so
#: adding them would need a second value shape in the metatest. The binding to
#: the contract is kept instead by a case in test_geo_write that reads the S6.2
#: row directly -- so the set is still checked against 11, just not from there.
GEO_EVENT_INFO = frozenset({"geo.created", "geo.updated", "geo.deleted",
                            "geo.renamed"})
GEO_EVENT_WARN = frozenset({"geo.conflict", "geo.force_overwrite",
                            "geo.route_changed", "geo.route_deleted",
                            "geo.route_remap_failed"})


def geo_event_severity(etype: str) -> str:
    """The 11 S6.2 severity for one geo event type.

    Raises ValueError for a type outside the closed set: S6.2 hardcodes the nine,
    and there is no default half to fall back to (3.5 -- an out-of-set value must
    not be silently interpreted as the nearer neighbour).
    """
    if etype in GEO_EVENT_INFO:
        return "info"
    if etype in GEO_EVENT_WARN:
        return "warn"
    raise ValueError(
        "geo event type %r is not in the 11 S6.2 closed set" % (etype,))


def render_geo_event(sev: str, etype: str, detail: Dict[str, Any],
                     eid: str) -> Tuple[str, Dict[str, Any]]:
    """One ApplyResult.events triple -> (key, body) for event/{sev}/geo.

    sev is checked against the type rather than trusted: see the module docstring.
    The caller's detail dict is never mutated -- an applier that reuses it for its
    ack would otherwise ship an event field to the cloud in the ack shape.
    """
    expected = geo_event_severity(etype)
    if sev != expected:
        raise ValueError(
            "geo event %r is %s-half in 11 S6.2, got sev=%r"
            % (etype, expected, sev))
    key = "event/%s/%s" % (sev, GEO_CATEGORY)
    body = {
        "eid": eid,
        "title": "geo %s %s" % (detail.get("geo_id"), etype),
        "detail": dict(detail, type=etype),
        "src": "p3_task",
        "ts": 0.0,
    }
    return key, body
