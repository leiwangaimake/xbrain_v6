"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch7.py
Brief: p4_agent tests -- batch7

Description:
GWY-P4-24/25/26/27/28/30/31 batch 7 tests (final P4 batch).
"""


import pytest

from xbrain.p4_agent.envelope.pose_snap import (
    Pose, PoseRing, PoseSnap,
)
from xbrain.p4_agent.registry.channel_permission import (
    allowed_channels, is_channel_allowed,
)
from xbrain.p4_agent.registry.d_class import (
    DRangeError, LightWhich, SchemaError,
    route_brightness, route_lights_on, route_redblue_pattern, route_volume,
)
from xbrain.p4_agent.registry.rulings_18b import (
    CumulativeDrift, E09SessionTier, E09TierError,
    check_r3_within_range, check_r5_restate_no_forbidden,
    resolve_e09_tier,
)
from xbrain.p4_agent.registry.time_expr import (
    TimeExpr, TimeParseError, parse, parse_at_local, parse_delay,
)
from xbrain.p4_agent.registry.tools_projection import (
    ToolProjectionError,
    check_t1_tools_subset_of_alternation,
    check_t2_max_five_tools, check_t3_schema_slots_match,
)


pytestmark = pytest.mark.no_device


# --- P4-24 channel permission CH-18-1 ---

def test_channel_permission_local_mic_for_A():
    assert is_channel_allowed("A", "local_mic")
    assert not is_channel_allowed("A", "cloud_voice")


def test_channel_permission_G_allows_all():
    for ch in ["local_mic", "cloud_voice", "hmi_text", "wecom_text"]:
        assert is_channel_allowed("G", ch)


def test_channel_permission_unknown_origin_denies():
    assert not is_channel_allowed("Z", "any_channel")


# --- P4-25 pose_snap PS-1..PS-6 ---

def test_pose_ring_evicts_oldest_when_full():
    r = PoseRing(capacity=3)
    for i in range(5):
        r.push(Pose(x=i, y=0, heading_deg=0, mono_ms=i))
    assert len(r._buf) == 3


def test_still_1s_false_when_less_than_1s_data():
    """PS-5: less than 1 s history -> not still."""
    r = PoseRing(capacity=100)
    r.push(Pose(x=0, y=0, heading_deg=0, mono_ms=0))
    r.push(Pose(x=0, y=0, heading_deg=0, mono_ms=500))
    assert r.is_still_1s(now_mono_ms=500) is False


def test_still_1s_true_when_stable():
    r = PoseRing(capacity=100)
    for i in range(11):
        r.push(Pose(x=0, y=0, heading_deg=0, mono_ms=i * 100))
    assert r.is_still_1s(now_mono_ms=1000) is True


def test_still_1s_false_when_moving():
    r = PoseRing(capacity=100)
    for i in range(11):
        r.push(Pose(x=i * 0.5, y=0, heading_deg=0, mono_ms=i * 100))
    assert r.is_still_1s(now_mono_ms=1000) is False


# --- P4-27 tools projection T-1/2/3 ---

def test_t1_tools_subset_of_alternation():
    check_t1_tools_subset_of_alternation(
        "M3", frozenset({"stop", "goto"}),
        frozenset({"stop", "goto", "spin"}))


def test_t1_tool_without_intent_raises():
    with pytest.raises(ToolProjectionError):
        check_t1_tools_subset_of_alternation(
            "M3", frozenset({"stop", "ghost"}),
            frozenset({"stop"}))


def test_t2_over_5_tools_raises_for_non_m4():
    with pytest.raises(ToolProjectionError):
        check_t2_max_five_tools("M3_nav", count=6)


def test_t2_m4_follow_allows_6():
    check_t2_max_five_tools("M4_follow", count=6)


def test_t3_schema_slot_mismatch_raises():
    with pytest.raises(ToolProjectionError):
        check_t3_schema_slots_match(
            "goto",
            tool_schema_slots=frozenset({"waypoint"}),
            intent_slots=frozenset({"waypoint", "hurry"}))


# --- P4-28 D-class routing PL-1..PL-6 + D01 dual-channel ---

def test_route_brightness_zero_maps_to_off():
    assert route_brightness(0) == "D07_lights_off"


def test_route_brightness_valid_range():
    assert route_brightness(50) == "D02_set_brightness"


def test_route_brightness_over_range_rejects():
    """PL-3: > 100 -> reject, NOT silently clip."""
    with pytest.raises(DRangeError):
        route_brightness(150)


def test_route_redblue_pattern_zero_rejects():
    """PL-5: pattern 0 is 'off' which is D07, not D18."""
    with pytest.raises(DRangeError):
        route_redblue_pattern(0)


def test_route_redblue_pattern_range():
    assert route_redblue_pattern(1) == "D18_set_pattern"
    with pytest.raises(DRangeError):
        route_redblue_pattern(17)


def test_route_volume_range():
    # 2026-08-11: set_volume is D10 (not the old D14; 18-A reserved D14-16
    # for payload_tilt_*).
    assert route_volume(50) == "D10_set_volume"
    with pytest.raises(DRangeError):
        route_volume(150)


def test_route_lights_on_requires_which():
    with pytest.raises(SchemaError):
        route_lights_on("")


def test_route_lights_on_dual_channel():
    assert route_lights_on("searchlight") == "D01_searchlight"
    assert route_lights_on("chassis") == "D01_chassis"


def test_route_lights_on_rejects_unknown_which():
    with pytest.raises(SchemaError):
        route_lights_on("magic")


# --- P4-30 time expression ---

def test_parse_delay_arabic():
    r = parse_delay("30 秒后")
    assert r.delay_seconds == 30


def test_parse_delay_chinese_digits():
    r = parse_delay("五秒后")
    assert r.delay_seconds == 5


def test_parse_at_local_afternoon():
    r = parse_at_local("下午 3 点")
    assert r.at_local == "15:00"


def test_parse_at_local_with_minutes():
    r = parse_at_local("上午 9 点 30 分")
    assert r.at_local == "09:30"


def test_parse_at_local_tomorrow():
    r = parse_at_local("明天 上午 8 点")
    assert r.day_offset == 1


def test_parse_unparseable_raises():
    with pytest.raises(TimeParseError):
        parse("unrecognized time thing")


# --- P4-31 18-B rulings ---

def test_r1_e09_requires_tier_first_time():
    s = E09SessionTier()
    with pytest.raises(E09TierError):
        resolve_e09_tier(s, requested_tier=None)


def test_r1_e09_uses_session_tier_on_subsequent_calls():
    s = E09SessionTier()
    resolve_e09_tier(s, requested_tier="mid")
    # Second call with no requested -> reuse.
    assert resolve_e09_tier(s, requested_tier=None) == "mid"


def test_r3_over_range_rejects_no_clip():
    """R-3: 云台转 400 度 -> reject; NOT clip to 355 (or whatever)."""
    with pytest.raises(ValueError) as ei:
        check_r3_within_range(400.0, lower=-355.0, upper=355.0)
    assert "R-3" in str(ei.value)
    assert "refused" in str(ei.value).lower()


def test_r5_restate_forbidden_word_rejects():
    """R-5: restate containing '已转' is banned."""
    with pytest.raises(ValueError) as ei:
        check_r5_restate_no_forbidden("已转 30 度")
    assert "R-5" in str(ei.value)


def test_r5_ok_restate_passes():
    check_r5_restate_no_forbidden("请求转 30 度")


def test_r4_cumulative_drift_warns_at_limit():
    d = CumulativeDrift()
    # 5 x 60 = 300; limit = 200 -> warn on 4th (240 > 200).
    for _ in range(3):
        assert d.add(60, 0, pan_limit=200, tilt_limit=200) is None
    warn = d.add(60, 0, pan_limit=200, tilt_limit=200)
    assert warn is not None
    assert "R-4" in warn
