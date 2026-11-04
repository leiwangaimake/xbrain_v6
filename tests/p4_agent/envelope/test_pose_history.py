"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_pose_history.py
Brief: the 18 PS-6 pose_snap is the pose at the first word, not the pose when the command is built

Description:
99 U109. PoseHistory keeps the state/pose frames P4 receives and answers
with the one at (or just before) the operator's first word, plus the PS-6
fields P3 judges: fix, heading, stillness over the second BEFORE the word,
and the age from the word to sending. Each case names the mutant it kills.
"""
from __future__ import annotations

import pytest  # marker

from xbrain.p4_agent.envelope.pose_snap import PoseHistory  # unit under test

pytestmark = pytest.mark.no_device  # pure


def _pose(lat, speed=0.0, fix="rtk_fixed"):
    """One state/pose frame as p1 publishes it (11 S3.0 envelope)."""
    return {"data": {"lat": lat, "lon": 135.5, "alt": 5.0, "heading_rad": 0.5,
                     "heading_valid": True, "heading_level": 1,
                     "fix_type": fix, "i_fix": 1.0, "i_heading": 1.0,
                     "speed_mps": speed}}  # PoseState fields


def _fed(frames):
    """A history fed with (receive ms, lat, speed) at 10 Hz."""
    h = PoseHistory(max_age_ms=2000)  # the --query-state-max-age-ms value
    for t, lat, speed in frames:  # in order
        h.feed(t, _pose(lat, speed))  # one frame
    return h  # ready


def test_the_pose_at_the_first_word_not_the_latest():
    """The robot walked on after the word. MUTATION: take the newest frame
    and the point moves with the LLM's latency (18 S0.4: up to 9.6 m)."""
    h = _fed([(t, 34.0 + t / 1e6, 0.0) for t in range(0, 6000, 100)])  # moving north
    snap = h.snapshot(t_first_mono_ms=1050, now_mono_ms=5000)  # spoke at 1.05 s
    assert snap["lat"] == 34.0 + 1000 / 1e6  # the frame at 1.0 s
    assert snap["snap_mono_s"] == 1.05  # PS-6: seconds, the first word
    assert snap["snap_age_ms"] == 3950  # word -> send
    assert snap["alt_m"] == 5.0 and snap["fix_type"] == "rtk_fixed"  # PS-6 names


def test_no_frame_near_the_word_means_no_snapshot():
    """MUTATION: accept any frame and a dead pose feed stores a stale point."""
    h = _fed([(0, 34.0, 0.0)])  # one old frame
    assert h.snapshot(t_first_mono_ms=5000, now_mono_ms=6000) is None  # 5 s off
    assert PoseHistory(2000).snapshot(100, 200) is None  # nothing fed


def test_no_position_means_no_snapshot():
    h = PoseHistory(max_age_ms=2000)  # fresh
    h.feed(0, {"data": {"lat": None, "lon": None, "fix_type": "no_fix"}})  # no fix
    assert h.snapshot(t_first_mono_ms=50, now_mono_ms=100) is None  # nothing to store


def test_history_starting_after_the_word_uses_the_first_frame():
    h = _fed([(t, 34.0, 0.0) for t in range(500, 1500, 100)])  # began at 0.5 s
    assert h.snapshot(t_first_mono_ms=300, now_mono_ms=2000)["lat"] == 34.0  # first frame


def test_stillness_is_judged_over_the_second_before_the_word():
    """PS-6 still_1s_ok. MUTATION: judge after the word (P3's view) and a
    dock captured while rolling to a stop passes."""
    still = _fed([(t, 34.0, 0.0) for t in range(0, 2100, 100)])  # stood still
    assert still.snapshot(2000, 2500)["still_1s_ok"] is True  # still
    rolling = _fed([(t, 34.0, 0.5 if t > 1500 else 0.0)
                    for t in range(0, 2100, 100)])  # moving before the word
    assert rolling.snapshot(2000, 2500)["still_1s_ok"] is False  # not still
    short = _fed([(t, 34.0, 0.0) for t in range(1600, 2100, 100)])  # 0.4 s of data
    assert short.snapshot(2000, 2500)["still_1s_ok"] is False  # PS-5: < 1 s
