"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_teach_group.py
Brief: HMI recording indicator -- state/teach projected read-only (11 S12A.5)

Description:
The HMI shows a recording; it cannot start, stop or save one. 11 S12.1.1's
upstream whitelist is a closed set of five types and teach is not among them
(frozen item F-8), and W6 teleop was removed entirely -- so a browser that could
start a recording still could not drive the robot along the route.

The cases worth having are the two ways this can lie: reporting `idle` when
nothing has been heard (an operator recording by voice would see a page that
looks the same as an idle one), and carrying the full point sequence into a 1 Hz
topic.
"""
from __future__ import annotations

import pytest

from xbrain.p5_gateway.hmi.data_readers import build_snapshot, teach_group

pytestmark = pytest.mark.no_device

_RECORDING = {
    "schema": "teach_state_v1",
    "session": {"session_id": "ts-b0-0001", "state": "recording",
                "kind": "route", "name_hint": "东门路线", "elapsed_s": 41.5},
    "stats": {"point_count": 23, "manual_count": 2, "length_m": 47.5,
              "dropped_by_quality": 1,
              "last_point": {"seq": 23, "lat": 34.6971, "lon": 135.5051,
                             "quality": "rtk_fixed", "manual": False}},
    "control": {"driver": "gamepad"},
    "warn": ["no_driver"],
    "validation": None,
}


def test_absent_teach_state_is_unavailable_not_idle():
    """*** The two differ and the difference is what the operator sees.

    `idle` means P3 said there is no session. `unavailable` means nobody said
    anything -- P3 down, or the key not flowing. MUTATION: default to
    {"state": "idle", "available": True} -- an operator recording by voice sees
    a page identical to an idle one, and concludes the recording did not start.
    """
    assert teach_group(None) == {"available": False}
    assert teach_group({})["available"] is False


def test_recording_projects_the_stats_the_indicator_needs():
    g = teach_group(_RECORDING)
    assert g["available"] is True and g["state"] == "recording"
    assert g["session_id"] == "ts-b0-0001" and g["kind"] == "route"
    assert g["point_count"] == 23 and g["length_m"] == 47.5
    assert g["dropped_by_quality"] == 1
    assert g["last_point"]["seq"] == 23
    assert g["warn"] == ["no_driver"]


def test_the_full_point_sequence_never_enters_the_group():
    """S12A.5 keeps the point list out of a 1 Hz topic (2000 points would blow
    it up) and publishes stats plus the last point. MUTATION: pass the whole
    TeachState through -- the group grows without bound as a recording runs, on
    a topic that is pushed every second to every open tab."""
    fat = {**_RECORDING,
           "stats": {**_RECORDING["stats"],
                     "points": [{"lat": 1.0, "lon": 2.0}] * 2000}}
    g = teach_group(fat)
    assert "points" not in g
    assert set(g) == {"available", "state", "session_id", "kind", "name_hint",
                      "elapsed_s", "point_count", "length_m",
                      "dropped_by_quality", "last_point", "warn", "validation"}


def test_validation_rides_through_for_the_naming_dialog():
    """S12A.7's result is what the save dialog has to show: "closed with a 6 m
    gap" is the difference between a fence the operator accepts and one they
    re-record."""
    finalizing = {**_RECORDING,
                  "session": {**_RECORDING["session"], "state": "finalizing"},
                  "validation": {"ok": True,
                                 "issues": [{"code": "close_gap_large",
                                             "gap_m": 6.2}]}}
    g = teach_group(finalizing)
    assert g["state"] == "finalizing"
    assert g["validation"]["issues"][0]["code"] == "close_gap_large"


def test_snapshot_carries_the_teach_group():
    """It must reach the browser through the same snapshot/delta path as every
    other group -- a group the delta does not know about is one the frontend
    never receives after the first keyframe."""
    snap = build_snapshot(teach=_RECORDING)
    assert snap["teach"]["state"] == "recording"
    assert build_snapshot()["teach"] == {"available": False}
