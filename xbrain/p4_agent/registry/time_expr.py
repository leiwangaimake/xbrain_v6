"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: time_expr.py
Brief: GWY-P4-30 -- deterministic time expression normalization

Description:
Voice commands include time expressions:
  delay_s     "30 秒后 ..." / "5分钟后" / "2个小时后" / "半小时后"
  at_local    "下午 3 点 ..." / "15:30" / "晚上八点半"
  day_offset  "明天 早上 ..." / "后天下午3点" (with at_local)

These MUST parse deterministically -- LLM ambiguity is banned here.
The parser is regex-based; every numeral goes through
slots.motion_slots.parse_number, the one numeral reader in P4.

99 U110 (2026-11-04) wired it for the first time: a timed PTP command
("下午3点去1号测试点") puts split_time_prefix in front of the goto
parser, and resolve_epoch turns the result into the 11 S7.2
task.not_before_ts. Until then nothing called this module, and the one
caller-shaped thing that existed (B03 patrol_schedule) dropped its time.
The same day fixed two parse defects the wiring would have shipped:
"十五秒" read as 5 (the old code deleted every 十) and "明天" alone set
nothing past today; "昨天" is no longer accepted -- a command cannot be
scheduled in the past.

CLAUDE.md 3.4: never reads its own clock. resolve_epoch takes the
current epoch from the caller (the orchestrator's injected wall clock,
which by the 2026-10-26 user ruling is the robot's system time).
"""

from __future__ import annotations

import re  # the time phrase scanner
from dataclasses import dataclass  # a frozen result
from datetime import datetime, time as dtime, timedelta  # local wall arithmetic
from typing import Optional, Tuple  # plain values

from xbrain.p4_agent.slots.motion_slots import parse_number  # one numeral reader


@dataclass(frozen=True)
class TimeExpr:
    """Parsed time expression. Exactly one of delay_s / at_local
    is set; day_offset is optional (only meaningful with at_local)."""
    delay_seconds: Optional[int] = None
    at_local: Optional[str] = None      # 'HH:MM'
    day_offset: int = 0                 # 0=today, 1=tomorrow, 2=the day after


class TimeParseError(RuntimeError):
    """Text did not parse as a supported time expression, or names a moment
    that cannot be scheduled (already past, too far ahead)."""


#: 16 S8.0.4 business upper bounds: delay_s <= 86400, day_offset <= 7.
DELAY_MAX_SECONDS = 86400  # one day, 16 S8.0.4
DAY_OFFSET_MAX = 7  # a week, 16 S8.0.4

#: An integer numeral: Arabic or Chinese (parse_number decides validity).
_NUM = r"[\d零一二两三四五六七八九十百]+"

#: Seconds per spoken unit.
_UNIT_SECONDS = {"秒": 1, "秒钟": 1, "s": 1, "分": 60, "分钟": 60,
                 "小时": 3600, "钟头": 3600}  # unit word -> seconds

#: "N <unit> 后", "N个半小时后", "半小时后". The suffix is optional only for
#: parse_delay's whole-string form (the old "10 s" input).
_DELAY_RE = re.compile(
    r"^(?:(?P<n>" + _NUM + r")(?P<half1>个半)?|(?P<half0>半))?"  # count, 个半, or 半
    r"个?(?P<unit>秒钟|秒|s|分钟|分|小时|钟头)(?P<half2>半)?"  # unit, trailing 半
    r"(?P<suffix>以后|之后|过后|后)?")  # 后 / 之后 / 以后 / 过后

#: Day words -> offset. 今晚 is a day word AND a part of day.
_DAY = {"今天": 0, "今晚": 0, "明天": 1, "明晚": 1, "后天": 2}  # day offset

#: Parts of the day that move a 1-11 hour into the afternoon / evening.
_PM = ("下午", "傍晚", "晚上", "今晚", "明晚", "夜里", "夜间")  # +12 when h < 12

_AT_LOCAL_RE = re.compile(
    r"^(?P<off>今天|明天|后天|今晚|明晚)?"  # day
    r"(?P<ap>凌晨|早上|早晨|上午|中午|下午|傍晚|晚上|夜里|夜间)?"  # part of day
    r"(?P<h>" + _NUM + r")(?:点钟|点|时|:)"  # hour
    r"(?P<m>一刻|三刻|半|" + _NUM + r")?(?:分钟|分)?(?P<zheng>整)?")  # minute; 一刻 before the numeral 一


def _int(token: str) -> Optional[int]:
    """A whole number from a numeral token, else None."""
    value = parse_number(token)  # Arabic or Chinese
    if value is None or value != int(value):  # malformed or fractional
        return None  # refuse, never guess
    return int(value)  # the integer


def _clean(text: str) -> str:
    """Whitespace removed; the full-width colon written as ':'."""
    return re.sub(r"\s+", "", text or "").replace(chr(0xFF1A), ":")  # spaces out; full-width colon


def _match_delay(t: str, need_suffix: bool) -> Optional[Tuple[TimeExpr, int]]:
    """(expr, consumed length) for a delay at the start of t, else None."""
    m = _DELAY_RE.match(t)
    if not m or (need_suffix and not m.group("suffix")):
        return None  # not a delay phrase here
    if m.group("half0"):
        count = 0.5  # 半小时 / 半分钟
    elif m.group("n"):
        n = _int(m.group("n"))
        if n is None:
            return None  # not a numeral
        count = n + (0.5 if m.group("half1") or m.group("half2") else 0.0)  # 个半 / 小时半
    else:
        return None  # a bare unit is not a time
    seconds = int(count * _UNIT_SECONDS[m.group("unit")])  # whole seconds
    if seconds <= 0:
        return None  # "0秒后" is not a schedule
    return TimeExpr(delay_seconds=seconds), m.end()  # parsed + consumed


def _match_at_local(t: str) -> Optional[Tuple[TimeExpr, int]]:
    """(expr, consumed length) for a clock time at the start of t, else None."""
    m = _AT_LOCAL_RE.match(t)
    if not m:
        return None  # no clock time here
    h = _int(m.group("h"))
    if h is None:
        return None  # hour is not a numeral
    off, ap = m.group("off") or "", m.group("ap") or ""
    if (ap in _PM or off in ("今晚", "明晚")) and h < 12:
        h += 12  # 下午3点 -> 15
    elif ap == "中午" and h < 11:
        h += 12  # 中午1点 -> 13; 中午12点 stays 12
    elif ap in ("凌晨", "早上", "早晨", "上午") and h == 12:
        h = 0  # 凌晨12点 -> 00
    minute = 0
    mtok = m.group("m")
    if mtok:
        minute = {"半": 30, "一刻": 15, "三刻": 45}.get(mtok)  # word minutes
        if minute is None:
            minute = _int(mtok)  # a numeral minute
    if minute is None or not (0 <= h < 24 and 0 <= minute < 60):
        return None  # out of range: refuse
    return (TimeExpr(at_local="%02d:%02d" % (h, minute),
                     day_offset=_DAY.get(off, 0)), m.end())  # parsed + consumed


def parse_delay(text: str) -> Optional[TimeExpr]:
    """'30 秒后' / '五秒后' / '5分钟后' / '半小时后' / '10 s' (whole string)."""
    t = _clean(text)
    hit = _match_delay(t, need_suffix=False)
    return hit[0] if hit and hit[1] == len(t) else None  # whole string only


def parse_at_local(text: str) -> Optional[TimeExpr]:
    """'下午 3 点', '明天 上午 9 点 30', '15:30' (whole string)."""
    t = _clean(text)
    hit = _match_at_local(t)
    return hit[0] if hit and hit[1] == len(t) else None  # whole string only


def parse(text: str) -> TimeExpr:
    """Try delay first, then at_local. Raise TimeParseError if
    neither matches."""
    r = parse_delay(text)
    if r is not None:
        return r
    r = parse_at_local(text)
    if r is not None:
        return r
    raise TimeParseError("cannot parse %r as time expression" % text)


def split_time_prefix(text: str) -> Tuple[Optional[TimeExpr], str]:
    """(time, rest) when text STARTS with a time phrase, else (None, text).

    A delay needs its 后 here ("5分钟后去..."): a bare "5分钟" at the start
    of a command is not evidently a schedule. Whitespace is dropped from the
    rest, the same normalisation the phrase parsers apply.
    """
    t = _clean(text)
    hit = _match_delay(t, need_suffix=True) or _match_at_local(t)  # either form
    if hit is None:
        return None, text  # untouched: no time here
    return hit[0], t[hit[1]:]  # the phrase and what follows it


def resolve_epoch(expr: TimeExpr, now_epoch_s: float,
                  tz_name: Optional[str]) -> float:
    """The absolute moment (epoch seconds) a TimeExpr names, from now.

    A clock time is read in the site timezone (common.timezone; the system
    zone when unset). Refused, never moved: a clock time today that has
    already passed (the operator meant something, and "tomorrow" is a guess),
    a delay over a day or a day offset over a week (16 S8.0.4 bounds).
    """
    if expr.delay_seconds is not None:
        if expr.delay_seconds > DELAY_MAX_SECONDS:
            raise TimeParseError("delay over %d s" % DELAY_MAX_SECONDS)
        return now_epoch_s + expr.delay_seconds  # relative
    if expr.at_local is None:
        raise TimeParseError("empty time expression")
    if not 0 <= expr.day_offset <= DAY_OFFSET_MAX:
        raise TimeParseError("day offset %d outside 0..%d"
                             % (expr.day_offset, DAY_OFFSET_MAX))
    if tz_name:
        from zoneinfo import ZoneInfo  # stdlib; imported where used
        local_now = datetime.fromtimestamp(now_epoch_s, ZoneInfo(tz_name))
    else:
        local_now = datetime.fromtimestamp(now_epoch_s).astimezone()  # system zone
    hh, mm = (int(x) for x in expr.at_local.split(":"))  # 'HH:MM'
    day = local_now.date() + timedelta(days=expr.day_offset)  # the target day
    target = datetime.combine(day, dtime(hh, mm), tzinfo=local_now.tzinfo)
    if target.timestamp() <= now_epoch_s:
        raise TimeParseError("that time has already passed")
    return target.timestamp()  # absolute epoch seconds
