"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_normalise.py
Brief: _normalise_event -- relative + absolute event keys (batch 7 regression)

Description:
The persist path went silent in the first ORIN live run because _normalise_event
parsed sev/cat at FIXED offsets that only matched the absolute contract key
(xbrain/{rid}/event/{sev}/{cat}); the dev bus uses relative keys
(event/{sev}/{cat}), so sev/cat came back None and every event was skipped. These
tests pin BOTH schemes so that regression cannot return silently. Mutations paired
per 3.3.
"""

import pytest

from xbrain.p5_gateway.runtime.main_wiring import (
    _event_seg_index,
    _first_present,
    _normalise_event,
)

pytestmark = pytest.mark.no_device


def test_relative_key_parses_sev_cat(monkeypatch):
    monkeypatch.setenv("XBRAIN_ROBOT_ID", "dev")
    ev = _normalise_event("event/alarm/intrusion",
                          {"eid": "e1", "title": "x", "detail": {"a": 1}})
    # MUTATION: fixed offset segs[4]/segs[5] (absolute-only) -> sev/cat None -> None.
    assert ev is not None
    assert (ev["sev"], ev["cat"], ev["rid"]) == ("alarm", "intrusion", "dev")


def test_absolute_key_parses_sev_cat_rid():
    ev = _normalise_event("xbrain/m20s-001/event/warn/task",
                          {"eid": "e1", "title": "x", "detail": {}})
    assert (ev["sev"], ev["cat"], ev["rid"]) == ("warn", "task", "m20s-001")


def test_missing_essentials_returns_none(monkeypatch):
    monkeypatch.delenv("XBRAIN_ROBOT_ID", raising=False)
    # No rid anywhere (relative key + no env + no payload rid) -> None.
    assert _normalise_event("event/warn/task", {"eid": "e1"}) is None
    # No eid -> None.
    monkeypatch.setenv("XBRAIN_ROBOT_ID", "dev")
    assert _normalise_event("event/warn/task", {"title": "x"}) is None


def test_event_seg_index():
    assert _event_seg_index(["event", "warn", "task"]) == 0
    assert _event_seg_index(["xbrain", "dev", "event", "warn", "task"]) == 2
    assert _event_seg_index(["state", "link"]) == -1


def test_detail_defaults_to_empty_dict(monkeypatch):
    monkeypatch.setenv("XBRAIN_ROBOT_ID", "dev")
    ev = _normalise_event("event/info/mode_change", {"eid": "e1"})
    assert ev["detail"] == {} and ev["title"] == ""


# ------------------------------------------------- Event.ts: 0.0 must survive

def test_a_producer_ts_of_zero_is_not_silently_replaced(monkeypatch):
    """THE C-5 defect. ts was `d.get("ts") or data.get("ts") or now.timestamp()`.

    `or` fires on any FALSY value, so a producer's 0.0 was replaced by p5's
    RECEIVE time -- silently, and in the direction that hides the bug. That is
    how three producers (p2 device_health_bridge, p3 geo_events, p3 task events)
    shipped a hard-coded ts=0.0 for months with a fully green suite: record.db
    held a plausible timestamp, the merge arithmetic saw an increasing ts, and
    nothing anywhere said the event time was invented by the wrong process.

    Keeping 0.0 is the point, not a regression: 1970 in the HMI is a visible bug
    someone chases within the hour. A silently substituted value is not.

    Precondition for this tightening, checked 2026-09-30: no publisher onto
    event/** sends ts=0.0 any more (the three above were fixed first; grep for
    '"ts": 0.0' over xbrain/ is empty).

    MUTATION: put the `or` chain back -> red (ts becomes now, not 0.0).
    """
    monkeypatch.setenv("XBRAIN_ROBOT_ID", "dev")
    ev = _normalise_event("event/warn/task",
                          {"eid": "e1", "title": "x", "ts": 0.0})
    assert ev is not None
    assert ev["ts"] == 0.0, (
        "a producer ts of 0.0 was rewritten to %r -- the substitution that hid "
        "three hard-coded producers" % ev["ts"])


def test_an_absent_ts_still_falls_back_to_the_receive_clock(monkeypatch):
    """The other half: tightening must not turn 'absent' into None in the row.

    record.db's ts column is the dedup comparison value, so a None there would
    break _try_merge rather than merely mis-date the row. A producer that sends
    no ts at all (some cloud-relayed shapes) must still get a reading, which is
    what makes `is None` the correct test rather than dropping the fallback.

    MUTATION: delete the now.timestamp() fallback -> red (ts is None).
    """
    monkeypatch.setenv("XBRAIN_ROBOT_ID", "dev")
    ev = _normalise_event("event/warn/task", {"eid": "e1", "title": "x"})
    assert ev is not None
    assert isinstance(ev["ts"], float) and ev["ts"] > 0.0, (
        "an event with no ts got %r; the fallback is what keeps record_dao's "
        "merge comparison arithmetic valid" % ev["ts"])


def test_the_envelope_ts_still_wins_over_the_inner_body(monkeypatch):
    """Precedence is unchanged: envelope first, then data, then the local clock.

    Asserted because switching from `or` to a helper is exactly the kind of edit
    that reverses an order by accident, and the reversal is invisible until two
    differing ts values appear on one message.

    MUTATION: swap the first two arguments -> red.
    """
    monkeypatch.setenv("XBRAIN_ROBOT_ID", "dev")
    ev = _normalise_event(
        "event/warn/task",
        {"eid": "e1", "title": "x", "ts": 11.0,
         "data": {"eid": "e1", "ts": 22.0}})
    assert ev["ts"] == 11.0


def test_first_present_distinguishes_absent_from_falsy():
    """The helper itself, so the property cannot regress via an `or` rewrite.

    MUTATION: implement it as `for v in candidates: if v: return v` -> red on
    the 0.0 and the 0 cases, which are the two that matter here.
    """
    assert _first_present(None, 0.0, 9.0) == 0.0
    assert _first_present(None, None, 0) == 0
    assert _first_present(None, None, 7.5) == 7.5
    assert _first_present(1.0, 2.0) == 1.0
    # All absent -> None, so a caller that forgot a fallback gets None rather
    # than a value invented here.
    assert _first_present(None, None) is None
