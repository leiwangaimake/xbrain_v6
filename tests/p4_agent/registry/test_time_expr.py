"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_time_expr.py
Brief: GWY-P4-30 time expressions -- the prefixes a timed goto uses, and their absolute time

Description:
99 U110 wired time_expr for the first time (timed PTP gotos). These pin the
phrases, the two parse defects fixed on the way ("十五秒" read as 5, a time
in the past accepted), and resolve_epoch in the site timezone. The clock is
a fixed epoch -- the module never reads one.
"""
from __future__ import annotations  # postponed annotations

from datetime import datetime  # building the expected epoch
from zoneinfo import ZoneInfo  # site timezone

import pytest  # parametrize, raises, marker

from xbrain.p4_agent.registry.time_expr import (  # units under test
    TimeExpr,  # the result
    TimeParseError,  # refusals
    parse_delay,  # whole-string delay
    resolve_epoch,  # absolute time
    split_time_prefix,  # the prefix form
)

pytestmark = pytest.mark.no_device  # pure

_TZ = "Asia/Shanghai"  # the site zone used below
#: 2026-11-04 10:00:00 local, the fixed "now".
_NOW = datetime(2026, 11, 4, 10, 0, tzinfo=ZoneInfo(_TZ)).timestamp()  # fixed now


@pytest.mark.parametrize("text,seconds", [  # relative phrases
    ("30秒后去", 30), ("十五秒后去", 15), ("5分钟后去", 300), ("5分后去", 300),
    ("2小时后去", 7200), ("2个小时后去", 7200), ("半小时后去", 1800),
    ("半个小时之后去", 1800), ("一个半小时后去", 5400), ("两小时以后去", 7200),
])
def test_relative_prefixes(text, seconds):  # case body
    """MUTATION: the old 十-deleting digit map reads "十五秒" as 5."""
    when, rest = split_time_prefix(text)  # one split
    assert when.delay_seconds == seconds and rest == "去"  # value + remainder


@pytest.mark.parametrize("text,at,day", [  # clock phrases
    ("下午3点去", "15:00", 0), ("下午三点半去", "15:30", 0), ("15:30去", "15:30", 0),
    ("晚上八点去", "20:00", 0), ("今晚8点去", "20:00", 0), ("中午12点去", "12:00", 0),
    ("中午1点去", "13:00", 0), ("明天14点去", "14:00", 1),
    ("后天上午9点15分去", "09:15", 2), ("明天早上8点一刻去", "08:15", 1),
])
def test_clock_prefixes(text, at, day):  # case body
    when, rest = split_time_prefix(text)  # one split
    assert (when.at_local, when.day_offset, rest) == (at, day, "去")  # parsed


def test_no_time_leaves_the_text_alone():  # no time
    """A bare unit without 后 is not a schedule, and a name is not a time."""
    assert split_time_prefix("5分钟去") == (None, "5分钟去")  # no 后
    assert split_time_prefix("1号测试点")[0] is None  # 1号 is not 1点


def test_whole_string_delay_keeps_its_old_forms():  # old forms
    assert parse_delay("10 s").delay_seconds == 10  # the old unit form
    assert parse_delay("5分钟后去") is None  # whole string only


def test_resolve_relative_and_tomorrow():  # absolute times
    assert resolve_epoch(TimeExpr(delay_seconds=300), _NOW, _TZ) == _NOW + 300  # relative
    expect = datetime(2026, 11, 5, 14, 0, tzinfo=ZoneInfo(_TZ)).timestamp()  # tomorrow 14:00
    assert resolve_epoch(TimeExpr(at_local="14:00", day_offset=1), _NOW, _TZ) == expect  # resolved


def test_a_passed_time_is_refused_not_moved():  # the past
    """MUTATION: roll "09:00 today" (it is 10:00) to tomorrow -- a guess."""
    with pytest.raises(TimeParseError, match="passed"):  # refused
        resolve_epoch(TimeExpr(at_local="09:00"), _NOW, _TZ)  # 09:00 < 10:00


def test_out_of_bounds_is_refused():  # bounds
    """16 S8.0.4: delay_s <= 86400, day_offset <= 7."""
    with pytest.raises(TimeParseError):  # over a day
        resolve_epoch(TimeExpr(delay_seconds=86401), _NOW, _TZ)  # 86401 s
    with pytest.raises(TimeParseError):  # over a week
        resolve_epoch(TimeExpr(at_local="08:00", day_offset=8), _NOW, _TZ)  # 8 days
