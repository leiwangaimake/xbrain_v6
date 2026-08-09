"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: time_expr.py
Brief: GWY-P4-30 -- deterministic time expression normalization

Description:
Voice commands include time expressions:
  delay_s     "30 秒后 ..."           relative seconds
  at_local    "下午 3 点 ..."          absolute local time HH:MM
  day_offset  "明天 早上 ..."          day offset + at_local

These MUST parse deterministically -- LLM ambiguity is banned here.
The parser is regex-based over Chinese numerals + digits.

CLAUDE.md 3.4: uses base timestamps injected as monotonic ms;
never reads its own clock.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class TimeExpr:
    """Parsed time expression. Exactly one of delay_s / at_local
    is set; day_offset is optional (only meaningful with at_local)."""
    delay_s: Optional[int] = None
    at_local: Optional[str] = None      # 'HH:MM'
    day_offset: int = 0                 # 0=today, 1=tomorrow, -1=yesterday


class TimeParseError(RuntimeError):
    """Text did not parse as a supported time expression."""


# Chinese digit -> Arabic digit map (single-char).
_CN_DIGIT = {
    "零": "0", "一": "1", "二": "2", "两": "2", "三": "3", "四": "4",
    "五": "5", "六": "6", "七": "7", "八": "8", "九": "9",
}


def _cn_to_arabic(s: str) -> str:
    return "".join(_CN_DIGIT.get(c, c) for c in s)


_DELAY_RE = re.compile(
    r"^(?P<n>[\d一二三四五六七八九十两]+)\s*(秒|s)后?$"
)


def parse_delay(text: str) -> Optional[TimeExpr]:
    """'30 秒后' / '5 秒后' / '10 s'."""
    t = text.strip()
    m = _DELAY_RE.match(t)
    if not m:
        return None
    n_txt = _cn_to_arabic(m.group("n").replace("十", ""))
    if not n_txt.isdigit():
        return None
    return TimeExpr(delay_s=int(n_txt))


_AT_LOCAL_RE = re.compile(
    r"^(?P<off>今天|明天|昨天)?\s*"
    r"(?P<ap>上午|下午|中午)?\s*"
    r"(?P<h>[\d一二三四五六七八九十]+)\s*(?:点|:)?\s*"
    r"(?P<m>[\d一二三四五六七八九十]+)?\s*"
    r"(?P<mtail>分)?$"
)


def parse_at_local(text: str) -> Optional[TimeExpr]:
    """'下午 3 点', '明天 上午 9 点 30', '15:30' 等."""
    t = text.strip()
    m = _AT_LOCAL_RE.match(t)
    if not m:
        return None
    h_txt = _cn_to_arabic(m.group("h").replace("十", ""))
    if not h_txt.isdigit():
        return None
    h = int(h_txt)
    ap = m.group("ap") or ""
    if ap in ("下午", "中午") and h < 12:
        h += 12
    if ap == "上午" and h == 12:
        h = 0
    if not (0 <= h < 24):
        return None
    minute = 0
    m_txt = m.group("m")
    if m_txt:
        mm = _cn_to_arabic(m_txt.replace("十", ""))
        if mm.isdigit():
            minute = int(mm)
    if not (0 <= minute < 60):
        return None
    day_offset = {"今天": 0, "明天": 1, "昨天": -1}.get(
        m.group("off") or "今天", 0)
    return TimeExpr(
        at_local="%02d:%02d" % (h, minute),
        day_offset=day_offset,
    )


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
