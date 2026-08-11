"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ptz_slots.py
Brief: PTZ fastpath slot parsers (E01 direction/amount, E06 zoom, E09 speed)

Description:
Tests the closed-set PTZ slot extraction (16 S8.0.4). Each carries a
mutation guard per CLAUDE.md 3.3.
"""
from __future__ import annotations

import pytest

from xbrain.p4_agent.slots.ptz_slots import (
    parse_ptz_amount, parse_ptz_direction, parse_ptz_speed_level,
    parse_zoom_direction,
)

pytestmark = pytest.mark.no_device


# -- E01 direction -------------------------------------------------------

def test_direction_four_ways():
    assert parse_ptz_direction("云台向左") == "left"
    assert parse_ptz_direction("往上看") == "up"
    assert parse_ptz_direction("看右边一点") == "right"
    assert parse_ptz_direction("低头") == "down"


def test_direction_none_when_absent():
    assert parse_ptz_direction("你好") is None


# -- E06 zoom direction --------------------------------------------------

def test_zoom_in_out():
    assert parse_zoom_direction("拉近") == "in"
    assert parse_zoom_direction("放大点") == "in"
    assert parse_zoom_direction("拉远") == "out"
    assert parse_zoom_direction("缩小") == "out"


# -- amount (shared E01/E06) --------------------------------------------

def test_amount_small_normal_large():
    """MUTATION guard: '一点' -> small (350ms pulse), '大幅' -> large
    (2800ms), plain -> normal (1000ms). Collapsing these would make every
    move the same length."""
    assert parse_ptz_amount("向左一点") == "small"
    assert parse_ptz_amount("向左大幅转") == "large"
    assert parse_ptz_amount("云台向左") == "normal"


# -- E09 speed level -----------------------------------------------------

def test_speed_level_longest_first():
    assert parse_ptz_speed_level("转速最快") == "fast"
    assert parse_ptz_speed_level("转速最慢") == "slow"
    assert parse_ptz_speed_level("转速快一点") == "up"
    assert parse_ptz_speed_level("转速慢一点") == "down"
    assert parse_ptz_speed_level("恢复正常转速") == "normal"
