"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_change_events.py
Brief: event/{warn|fault}/health -- one event per held health item change (11 S6.2)

Description:
Pins xbrain/p2_core/health/change_events.py, the source of the device health
entries in the HMI event list (user ruling 2026-11-12: every device health change
is recorded, one entry per change). Each test names the mutant that turns it red.
The traps a do-nothing or over-eager tracker would fall into:
  * startup flood: every item's first state reported as a change;
  * never-known items (compute, gpu: no source) reported as "unknown";
  * a 1 Hz flap reported twice;
  * a change to fail filed as warn (it must be fault, 11 S6.2 health row).
"""

from __future__ import annotations  # postponed annotations

import pytest  # parametrize / raises

from xbrain.p2_core.health.change_events import (  # the unit under test
    EVENT_TYPE,  # detail.type the HMI words on
    HealthChangeTracker,  # baseline + hold judgement
    health_change_event,  # the 11 S6.1 Event builder
)


def _it(state, level="optional", detail=None):
    """One health/summary item body, shaped like aggregate.build_summary's."""
    return {"state": state, "level": level, "kind": "degradable", "detail": detail}  # S5.1 item


def test_first_known_state_is_a_silent_baseline():
    """MUTATION: report the first sample -> 2 events at startup -> red."""
    t = HealthChangeTracker(3.0)  # the configured hold
    # a fail at start-up is the baseline too: the stack starting is not news
    assert t.observe({"speaker": _it("ok"), "cam_rgbd": _it("fail")}, 0.0) == []  # baseline
    assert t.observe({"speaker": _it("ok"), "cam_rgbd": _it("fail")}, 10.0) == []  # unchanged


def test_a_held_change_is_reported_once():
    """MUTATION: hold ignored (report at once) -> red at t=1; reported state not
    advanced -> a second event at t=5 -> red."""
    t = HealthChangeTracker(3.0)  # 3 s hold
    t.observe({"speaker": _it("ok")}, 0.0)  # baseline ok
    assert t.observe({"speaker": _it("fail", "fatal", "no ack")}, 1.0) == []  # hold starts
    assert t.observe({"speaker": _it("fail", "fatal", "no ack")}, 3.9) == []  # 2.9 s: not yet
    assert t.observe({"speaker": _it("fail", "fatal", "no ack")}, 4.0) == [  # 3.0 s: due
        {"device": "speaker", "from": "ok", "to": "fail", "level": "fatal",  # the transition
         "from_reason": None, "reason": "no ack"}]  # detail carried as reason
    assert t.observe({"speaker": _it("fail", "fatal")}, 5.0) == []  # reported once only
    # and the recovery is a second change, measured from the reported fail
    t.observe({"speaker": _it("ok")}, 6.0)  # recovery hold starts
    assert t.observe({"speaker": _it("ok")}, 9.0)[0]["to"] == "ok"  # recovery reported


def test_a_flap_inside_the_hold_is_nothing():
    """MUTATION: pending not cleared on return to the reported state -> the
    old pending start makes t=4.5 fire -> red."""
    t = HealthChangeTracker(3.0)  # 3 s hold
    t.observe({"speaker": _it("ok")}, 0.0)  # baseline ok
    t.observe({"speaker": _it("fail")}, 1.0)  # a fail starts the hold
    assert t.observe({"speaker": _it("ok")}, 2.0) == []  # back to ok: cancelled
    assert t.observe({"speaker": _it("fail")}, 4.5) == []  # hold restarts here
    assert t.observe({"speaker": _it("fail")}, 7.5)[0]["to"] == "fail"  # 3 s after 4.5


def test_a_different_new_state_restarts_the_hold():
    """MUTATION: keep the first pending start when the target changes -> the
    warn->fail hop fires at t=4 as 'fail' -> red."""
    t = HealthChangeTracker(3.0)  # 3 s hold
    t.observe({"cam_rgbd": _it("ok")}, 0.0)  # baseline ok
    t.observe({"cam_rgbd": _it("degraded")}, 1.0)  # heading for degraded
    assert t.observe({"cam_rgbd": _it("fail")}, 3.0) == []  # new target: hold restarts
    assert t.observe({"cam_rgbd": _it("fail")}, 5.9) == []  # 2.9 s on fail
    assert t.observe({"cam_rgbd": _it("fail")}, 6.0)[0] == {  # 3 s on fail
        "device": "cam_rgbd", "from": "ok", "to": "fail", "level": "optional",  # from the baseline
        "from_reason": None, "reason": None}  # no detail given


def test_never_known_items_stay_silent_but_a_known_one_reports_unknown():
    """MUTATION: unknown accepted as a baseline -> gpu unknown->ok would report
    (and compute would baseline) -> red; unknown never reported -> a camera
    whose feed stops is invisible -> red."""
    t = HealthChangeTracker(0.0)  # no hold: judge the rule alone
    assert t.observe({"gpu": _it("unknown"), "cam": _it("ok")}, 0.0) == []  # cam baseline
    assert t.observe({"gpu": _it("ok"), "cam": _it("unknown")}, 1.0) == [  # cam lost
        {"device": "cam", "from": "ok", "to": "unknown", "level": "optional",  # reported
         "from_reason": None, "reason": None}]  # gpu absent: its ok was a baseline
    # gpu's first KNOWN state (ok at t=1) was its silent baseline
    assert t.observe({"gpu": _it("ok"), "cam": _it("unknown")}, 2.0) == []  # steady


def test_malformed_items_say_nothing():
    """MUTATION: a non-dict body raising -> the 1 Hz publish loop dies -> red."""
    t = HealthChangeTracker(0.0)  # no hold
    assert t.observe({"a": None, "b": {"state": 3}, "c": "ok"}, 0.0) == []  # all ignored


@pytest.mark.parametrize("bad", [None, -1.0, True, "3"])  # null, negative, bool, text
def test_hold_must_be_a_non_negative_number(bad):
    """CLAUDE.md 3.1: no default; a null from an unfrozen product must raise."""
    with pytest.raises(ValueError):  # p2 refuses to start
        HealthChangeTracker(bad)  # the constructor judges


def test_event_shape_and_severity():
    """MUTATION: every change filed as warn (or as fault) -> red; cat not
    health / type not health.item_changed -> the HMI wording and the cloud
    fault-source mapping miss it -> red."""
    chg = {"device": "speaker", "from": "ok", "to": "fail", "level": "fatal",  # one change
           "reason": "no ack"}  # as observe() returns it
    ev = health_change_event(chg, eid="health-p2-abc-1", ts=123.0)  # build
    assert (ev["sev"], ev["cat"], ev["channel"], ev["code"]) == (  # 11 S6.2 health row
        "fault", "health", "normal", "HEALTH_ITEM_CHANGED")  # fail -> fault
    assert ev["detail"] == {"type": EVENT_TYPE, "device": "speaker", "from": "ok",  # detail
                            "to": "fail", "from_reason": None,  # the cause being left
                            "level": "fatal", "reason": "no ack"}  # whole
    assert (ev["eid"], ev["ts"], ev["src"]) == ("health-p2-abc-1", 123.0, "p2_core")  # ids
    assert EVENT_TYPE == "health.item_changed"  # the HMI eventText key
    back = health_change_event(dict(chg, **{"from": "fail", "to": "ok"}), eid="e", ts=1.0)  # recovery
    assert back["sev"] == "warn"  # S6.2 has no info for health
    assert health_change_event(dict(chg, to="degraded"), eid="e", ts=1.0)["sev"] == "warn"  # not fail
