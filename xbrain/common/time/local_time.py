"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: local_time.py
Brief: Site-timezone local-time formatting for DISPLAY (HMI / TTS G24 / CHS-A)

Description:
The one place UTC is turned into a site-local string for DISPLAY. The internal
wall clock is UTC (envelope ts) and every timeout/period uses the monotonic clock
(CLK-C1), so timezone is a pure display concern -- it never touches safety. The
site timezone is common.timezone (a single source, per-deployment for
internationalisation); this module takes it as a parameter so it stays pure and
testable, and it RAISES on a bad zone name rather than silently falling back to
UTC (a misconfigured site tz must surface, not quietly mislead the operator).

Uses the stdlib zoneinfo (Python 3.9+) against the system tz database.
"""

from __future__ import annotations

from datetime import datetime, timezone
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

# Spoken weekday, Monday=0 (datetime.weekday()). Chinese text is data, not
# punctuation (CLAUDE.md 2.2 forbids Chinese punctuation, not Chinese words).
_WEEKDAY_CN = ["周一", "周二", "周三", "周四", "周五", "周六", "周日"]


def is_valid_tz(tz_name: str) -> bool:
    """True iff tz_name is a resolvable IANA zone on this host."""
    try:
        ZoneInfo(tz_name)
        return True
    except (ZoneInfoNotFoundError, ValueError):
        return False


def format_local(utc_s: float, tz_name: str) -> str:
    """UTC epoch seconds -> 'YYYY-MM-DD HH:MM:SS' in tz_name. Raises on a bad zone
    name (fail loud, never silently show UTC)."""
    tz = ZoneInfo(tz_name)
    dt = datetime.fromtimestamp(utc_s, tz=timezone.utc).astimezone(tz)
    return dt.strftime("%Y-%m-%d %H:%M:%S")


def format_spoken(utc_s: float, tz_name: str) -> str:
    """A TTS-friendly local time for G24, e.g. '8月16日 周日 14点05分'."""
    tz = ZoneInfo(tz_name)
    dt = datetime.fromtimestamp(utc_s, tz=timezone.utc).astimezone(tz)
    return "%d月%d日 %s %d点%02d分" % (
        dt.month, dt.day, _WEEKDAY_CN[dt.weekday()], dt.hour, dt.minute)
