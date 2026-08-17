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
    _event_seg_index, _normalise_event,
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
