"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_payload_slots.py
Brief: fastpath slot parsers for D17 level / D18 mode / D10 volume

Description:
Tests the closed-set slot extraction (16 S8.0.4). Each carries a mutation
guard per CLAUDE.md 3.3.
"""
from __future__ import annotations

import pytest

from xbrain.p4_agent.slots.payload_slots import (
    parse_light_level, parse_strobe_mode, parse_volume,
)

pytestmark = pytest.mark.no_device


# -- D17 brightness level (closed set, 18-A S1.1) ------------------------

def test_light_level_absolute():
    assert parse_light_level("最亮") == "max"
    assert parse_light_level("调到最暗") == "min"
    assert parse_light_level("亮度一半") == "mid"


def test_light_level_relative():
    assert parse_light_level("亮一点") == "up"
    assert parse_light_level("暗一点") == "down"


def test_light_level_longest_first_not_confused():
    """MUTATION guard: '亮一点点' is high, '亮一点' is up (18-A S1.1). A
    shortest-first match would collapse both to up."""
    assert parse_light_level("亮一点点也行") == "high"
    assert parse_light_level("亮一点") == "up"


def test_light_level_none_when_no_keyword():
    assert parse_light_level("你好") is None


# -- D18 strobe mode (18-A S1.2) -----------------------------------------

def test_strobe_mode_explicit():
    assert parse_strobe_mode("用第三种模式") == 3
    assert parse_strobe_mode("换成5") == 5


def test_strobe_mode_empty_means_cycle():
    # '换一种'/'换个样式' -> None -> p2 cycles current+1.
    assert parse_strobe_mode("换一种闪法") is None
    assert parse_strobe_mode("换个警灯样式") is None


def test_strobe_mode_redblue_selects_pattern_1():
    # '切换到红蓝爆闪模式' -> mode 1 (red-blue), a way back from a pure-red
    # pattern (2026-08-11 ORIN).
    assert parse_strobe_mode("切换到红蓝爆闪模式") == 1
    assert parse_strobe_mode("红蓝模式") == 1


def test_strobe_mode_zero_and_out_of_range_rejected():
    """MUTATION guard: mode 0 is off (D07), not a pattern; >16 is out of
    range. Both yield None (cycle) rather than an invalid pattern."""
    assert parse_strobe_mode("用第0种") is None
    assert parse_strobe_mode("换成17") is None


# -- D10 volume (18 S6.4) ------------------------------------------------

def test_volume_absolute():
    assert parse_volume("音量调到最大") == {"abs": 100}
    assert parse_volume("音量调到最小") == {"abs": 0}
    assert parse_volume("静音") == {"abs": 0}
    assert parse_volume("音量一半") == {"abs": 50}


def test_volume_relative_both_word_orders():
    """2026-08-11 ORIN: '大点声'/'音量大一点' missed. Both word orders map."""
    assert parse_volume("大声点") == {"rel": 25}
    assert parse_volume("大点声") == {"rel": 25}
    assert parse_volume("音量大一点") == {"rel": 25}
    assert parse_volume("小声点") == {"rel": -25}
    assert parse_volume("小点声") == {"rel": -25}


def test_volume_none_when_no_keyword():
    assert parse_volume("前进一米") is None
