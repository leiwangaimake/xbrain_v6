"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_events.py
Brief: 11 S6.2 geo events -- closed set, severity pairing, wiring

Description:
Two halves, and the second is the one that matters. The pure renderer is checked
directly. The WIRING is checked by reading p3's real source, because the defect
this file was written for was invisible to every fake: the appliers built the
audit triples correctly, p5's event/** subscriber persisted correctly, and
handle_geo_payload's on_event defaulted to None in between -- so every map edit
committed, acked accepted, and left no audit line. A fake that calls the callback
proves nothing about whether the wiring ever passes one (3.2 form 1).
"""

from __future__ import annotations

import inspect

import pytest

from xbrain.p3_task.runtime import main_wiring
from xbrain.p3_task.state import geo_events
from xbrain.p3_task.state.geo_events import (
    GEO_CATEGORY,
    GEO_EVENT_INFO,
    GEO_EVENT_WARN,
    geo_event_severity,
    render_geo_event,
)

# INF-TS-1: 纯单测, 不碰设备(无 zenohd / 无底盘 / 无 ORIN 专属硬件).
pytestmark = pytest.mark.no_device

#: The ts the renderer calls below hand in. 11 S6.1 Event.ts is now a REQUIRED
#: argument; it was a hard-coded 0.0 inside the renderer until 2026-09-30. Fixed
#: and clearly non-zero so an assertion pins the value that was passed rather
#: than "some plausible clock reading".
FIXED_TS = 1787000000.0


# ----------------------------------------------------------------- closed set

def test_the_severity_of_a_geo_event_is_read_off_its_type_not_chosen():
    """11 S6.2 splits the nine types into a fixed info half and a fixed warn half.
    MUTATION: return "info" unconditionally -> the warn half goes red."""
    for etype in GEO_EVENT_INFO:
        assert geo_event_severity(etype) == "info", etype
    for etype in GEO_EVENT_WARN:
        assert geo_event_severity(etype) == "warn", etype


def test_a_type_outside_the_s6_2_closed_set_is_refused_not_guessed():
    """3.5: no default half. MUTATION: return "info" for the unknown case."""
    with pytest.raises(ValueError) as exc:
        geo_event_severity("geo.archived")
    assert "closed set" in str(exc.value)
    assert "geo.archived" in str(exc.value)


def test_the_two_halves_do_not_overlap():
    """A type in both halves would make the severity ambiguous and the check
    above pass by accident. MUTATION: move geo.updated into GEO_EVENT_WARN."""
    assert not (GEO_EVENT_INFO & GEO_EVENT_WARN)


# ------------------------------------------------------------------- renderer

def test_a_producer_whose_severity_disagrees_with_its_type_is_refused():
    """An applier that labels geo.created as warn is a producer bug; publishing it
    puts a value the cloud cannot classify into the audit trail.
    MUTATION: drop the sev != expected branch -> this goes red."""
    with pytest.raises(ValueError) as exc:
        render_geo_event("warn", "geo.created", {"geo_id": "r-a"}, "geo-x-1",
                         FIXED_TS)
    assert "info-half" in str(exc.value) or "info" in str(exc.value)


def test_detail_type_is_the_event_type_and_geo_type_keeps_the_object_type():
    """11 S6.2 requires detail.type to be the EVENT type. The appliers put the
    geometry type under geo_type. MUTATION: stamp type=detail["geo_type"], or
    drop the stamping -> both go red."""
    key, body = render_geo_event(
        "info", "geo.created",
        {"geo_id": "r-patrol_a", "geo_type": "route", "state": "active"},
        "geo-abc-7", FIXED_TS)
    assert body["detail"]["type"] == "geo.created"
    assert body["detail"]["geo_type"] == "route"
    assert body["detail"]["geo_id"] == "r-patrol_a"
    assert key == "event/info/geo"


def test_the_key_carries_the_severity_that_was_validated():
    """MUTATION: hardcode event/info/geo -> the warn case goes red."""
    key, _ = render_geo_event(
        "warn", "geo.route_deleted", {"geo_id": "r-old", "refs": 0}, "geo-a-1",
        FIXED_TS)
    assert key == "event/warn/geo"
    assert key.endswith("/" + GEO_CATEGORY)


def test_the_callers_detail_dict_is_not_mutated():
    """The appliers hand back the same dict they may reuse; stamping in place
    would leak an event-only field into an ack.
    MUTATION: detail["type"] = etype; body = {..., "detail": detail} -> red."""
    detail = {"geo_id": "w-p1", "geo_type": "waypoint"}
    render_geo_event("info", "geo.created", detail, "geo-a-1", FIXED_TS)
    assert "type" not in detail


def test_the_eid_the_wiring_minted_is_the_eid_published():
    """record.db has UNIQUE(eid); a renderer that minted its own would collide
    with the wiring's boot-token scheme. MUTATION: overwrite eid -> red."""
    _, body = render_geo_event("info", "geo.deleted", {"geo_id": "f-a"},
                               "geo-deadbe-42", FIXED_TS)
    assert body["eid"] == "geo-deadbe-42"


def test_the_event_declares_p3_as_its_source():
    """11 S6.1 src is the producing process; p5 persists it verbatim.
    MUTATION: src="p5_gateway" -> red."""
    _, body = render_geo_event("info", "geo.renamed",
                               {"geo_id": "r-a", "name": "x"}, "geo-a-1",
                               FIXED_TS)
    assert body["src"] == "p3_task"


# --------------------------------------------------------------------- wiring

def _call_text(src: str, opening: str) -> str:
    """The source of one call, from `opening` to ITS closing paren. Slicing at the
    first ")" would stop inside a nested call (geo_queue.get_nowait()) and read as
    "the argument is absent" for an argument that is simply further along."""
    start = src.index(opening)
    depth = 0
    for i in range(start + len(opening) - 1, len(src)):
        if src[i] == "(":
            depth += 1
        elif src[i] == ")":
            depth -= 1
            if depth == 0:
                return src[start:i + 1]
    raise AssertionError("unbalanced parentheses after %r" % (opening,))



def test_p3_actually_passes_on_event_when_applying_a_geo_command():
    """THE defect. handle_geo_payload drops every audit event when on_event is
    left at its None default, and the drop is silent: the write commits, the ack
    says accepted, and record.db gains nothing. Asserted against p3's real source
    because no fake can observe an argument that is never passed.
    MUTATION: delete on_event=_emit_geo_event from the call -> red."""
    src = inspect.getsource(main_wiring._amain)
    assert "handle_geo_payload(" in src, "the geo apply call moved; retarget this"
    call = _call_text(src, "handle_geo_payload(")
    assert "on_event=" in call, (
        "p3 applies cmd/geo without on_event -- every map edit is unaudited")


def test_the_wiring_publishes_through_the_shared_renderer():
    """The closed-set check and the detail.type stamping live in geo_events; a
    wiring that assembled its own body would bypass both and the tests above
    would still pass. MUTATION: inline a gen.put with a hand-built dict -> red."""
    src = inspect.getsource(main_wiring._amain)
    call = _call_text(src, "render_geo_event(")
    # Bound AND consumed: a wiring that called the renderer and then published a
    # hand-built dict would satisfy a bare "is it referenced" check.
    assert "key, body = render_geo_event(" in src, (
        "the renderer result is not bound; the wiring may be assembling its own")
    put = _call_text(src[src.index(call) + len(call):], "gen.put(")
    assert "key" in put and "body" in put, (
        "gen.put does not publish what render_geo_event returned: %s" % put)


def test_the_geo_eid_carries_a_boot_token():
    """A bare seq restarts at 0 each p3 start while record.db persists, so the
    first geo edit after a restart hits UNIQUE(eid) and the DAO degrades the row
    to JSONL. Same lesson the device_health_bridge audit already paid for.
    MUTATION: drop _geo_evt_boot from the eid -> red."""
    src = inspect.getsource(main_wiring._amain)
    assert "_geo_evt_boot" in src
    assert '"geo-%s-%d" % (_geo_evt_boot' in src


def test_the_event_ts_is_the_ts_the_wiring_passed_in_not_a_constant():
    """11 S6.1 Event.ts must say when the map was changed, not when p5 saw it.

    The renderer hard-coded ts=0.0 until 2026-09-30 and nothing caught it: p5's
    _normalise_event writes `d.get("ts") or data.get("ts") or now.timestamp()`
    and 0.0 is FALSY, so record.db stored p5's RECEIVE time. The geo stream is
    the audit trail of who changed the map (S6.2 / S7.10), so the one field that
    answers "when" was being supplied by the wrong process -- and it leaned on
    the value being accidentally falsy, so tightening that `or` into `is None`
    (the correct fix on the p5 side) would have produced real 1970 stamps.

    MUTATION: put ts=0.0 back in the body -> red. Give the parameter a default
    and stop passing it from the wiring -> red at the call, which is the point
    of it having no default.
    """
    _, body = render_geo_event("info", "geo.created",
                               {"geo_id": "r-a", "geo_type": "route"},
                               "geo-a-1", FIXED_TS)
    assert body["ts"] == FIXED_TS, (
        "geo event carries ts=%r; a constant ts hands the event time to p5"
        % body["ts"])


def test_the_renderer_stays_pure_and_reads_no_clock():
    """ts is passed IN, so this module must not import a clock at all.

    The split matters twice. It keeps the renderer callable with no clock (its
    docstring calls it the PURE mapping, and every test above relies on that),
    and it puts the wall reading in the wiring layer, which is the same division
    12 S6A.8 PUB-1 draws for the rotation events. A module that reads its own
    clock also cannot be asserted against a fixed value, so the test above would
    have to weaken to "greater than zero" -- and ts=0.0 plus an off-by-one is
    exactly what that weaker form fails to distinguish.

    MUTATION: import time in geo_events and default ts to time.time() -> red.
    """
    src = inspect.getsource(geo_events)
    for banned in ("import time", "time.time(", "datetime.now(", "monotonic("):
        assert banned not in src, (
            "geo_events reads a clock (%r); ts is the wiring's to supply"
            % banned)


def test_the_wiring_passes_a_real_clock_and_not_a_literal():
    """The renderer is only as good as what the one production caller hands it.

    Asserted on p3's real source because no fake can observe the argument the
    wiring chose: a wiring that passed 0.0 would satisfy every renderer test
    above, since those supply their own FIXED_TS.

    MUTATION: change time.time() to 0.0 at the call site -> red.
    """
    src = inspect.getsource(main_wiring._amain)
    call = _call_text(src, "render_geo_event(")
    assert "time.time()" in call, (
        "the wiring does not pass a wall reading to render_geo_event: %s" % call)
    assert "0.0" not in call, "a literal ts is back at the geo call site"


def test_the_geo_event_carries_no_dedup_key_without_a_window():
    """The pairing that silently swallows events is key-without-window.

    11 S6.2's geo row specifies sev, channel normal and the nine detail.type
    values, and gives neither a dedup_key nor a dedup_window_s; S7.10 adds that
    the stream carries no synchronisation duty. p5 reaches _try_merge only
    `if ev.get("dedup_key")`, so a window alone would never be read, while a key
    alone makes record_dao fold every later event onto the first un-abandoned
    row (dedup_count rises, the cloud sees one line). Inventing either is a
    contract change, not an implementation choice (CLAUDE.md S3.7).

    MUTATION: add "dedup_key": "geo:%s" % etype to the body -> red.
    """
    _, body = render_geo_event("info", "geo.deleted", {"geo_id": "f-a"},
                               "geo-a-1", FIXED_TS)
    if "dedup_key" in body:
        assert body.get("dedup_window_s") is not None, (
            "the geo event grew a dedup_key with no dedup_window_s -- p5 merges "
            "that combination unconditionally and the audit trail collapses to "
            "one row")
