"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: time_sync_answer.py
Brief: H03 "校时" -- answer with the clock sync status from state/clock, the chronyd fact, and the current time to the second
Description:
What problem this solves
------------------------
Field test 2026-10-30 (R009): "校时" answered "收到" and nothing happened --
nothing in the repo consumes cmd/system (11 S7.15). The interim fix answered
"校时功能还没有接通执行端", honest but useless.

User ruling 2026-10-30 (99 U102): no sudo. "校时" answers the current clock
sync status -- "时钟已通过 NTP 同步" or "未同步" -- says the system keeps
itself in sync with chronyd, and speaks "当前时间XX时XX分XX秒". A FORCED
time sync (force_step, a wall-clock step) stays honestly "未接通".

Why p4 answers instead of a p5 executor (11 S7.15.3 names p5)
------------------------------------------------------------
The contract's default time_sync does ONE active thing: ask chronyd to poll now
(`chronyc burst` / `online`), which needs root -- excluded by the ruling. What
is left is reading ClockStatus, and CLK-A1 makes rtk_driver its only judge,
mirrored by p1 as state/clock. p5 would read that same topic and send it back
in a SystemAck; p4 already caches it (G47) and speaks the reply itself. So no
cmd/system is published, and an unconsumed command (SYS-3) cannot arise.

Rules
-----
  * sync is spoken as "已同步" only for an explicit sync == true in a FRESH
    state/clock (CLK-A3 fail-safe, the same rule as G47); stale or missing
    state/clock answers the no_data branch, never a guess;
  * the source is named only from the 11 S3.11 closed set (ntp / rtk / rtc);
    an unknown spelling says "已同步" without a source;
  * the chronyd clause is a READING: chronyd is looked up in /proc (any user
    may read /proc/<pid>/comm). This is not a sync judgment (CLK-A2 forbids
    those): it only says whether the service that keeps the clock is running;
  * the time is the robot's system time (`date`), in the site timezone, the
    same source G24 speaks since the 2026-10-26 ruling.

Boundary: reads the cache and /proc, returns text. No publishing.
"""
from __future__ import annotations  # postponed annotations

import os  # /proc lookup for chronyd
import time  # the system wall clock
from datetime import datetime, timezone  # spoken local time
from typing import Callable, Mapping, Optional  # signatures
from zoneinfo import ZoneInfo  # site timezone

from xbrain.p4_agent.state.cache import StateCache  # freshness-aware cache
from xbrain.p4_agent.state.query_data import QueryAnswer  # the shared answer type
from xbrain.p4_agent.templates.query_engine import render_reply  # 16 S8.2 step 3

#: ClockStatus.source (11 S3.11, closed set minus "none") -> spoken name.
SOURCE_NAMES = {"ntp": "NTP", "rtk": "RTK 授时", "rtc": "本机实时时钟"}  # spoken labels

#: Words that mark a forced time sync (force_step: a wall-clock step).
FORCE_WORDS = ("强制", "强行", "跳过来")  # "强制校时" / "强行校时" / "时间跳过来" (99 U199)

TEMPLATE = "time_sync"  # configs/query_templates.yaml block name


def wall_now() -> float:
    """The system wall clock, in one place so tests can replace it."""
    # WALL-CLOCK-OK(record): H03 speaks it to a person; never an age or timeout.
    return time.time()  # what `date` prints


def chronyd_running(proc_root: str = "/proc") -> bool:
    """True when a process named chronyd is alive (read from /proc/<pid>/comm)."""
    try:  # /proc may be unreadable in a sandbox
        pids = os.listdir(proc_root)  # one entry per process
    except OSError:  # no /proc: not a reading
        return False  # say it is not running rather than guess
    for pid in pids:  # every process
        if not pid.isdigit():  # skip /proc/self, /proc/meminfo, ...
            continue  # not a process
        try:  # the process may exit while we look
            with open(os.path.join(proc_root, pid, "comm"), encoding="utf-8") as fh:  # its name
                if fh.read().strip() == "chronyd":  # the time daemon
                    return True  # found it
        except OSError:  # gone or unreadable
            continue  # next process
    return False  # no chronyd


def spoken_clock(utc_s: float, tz_name: Optional[str]) -> str:
    """'19时38分46秒' in the site timezone (the system local zone when unset)."""
    tz = ZoneInfo(tz_name) if tz_name else None  # None = system local zone
    dt = datetime.fromtimestamp(utc_s, tz=timezone.utc).astimezone(tz)  # local time
    return "%d时%d分%d秒" % (dt.hour, dt.minute, dt.second)  # hour, minute, second


def _clock(cache: StateCache, now_mono_ms: int, max_age_ms: int) -> Optional[Mapping]:
    """Fresh state/clock data object (envelope unwrapped), or None."""
    v = cache.get_fresh("state/clock", now_mono_ms, max_age_ms)  # stale -> None
    if not isinstance(v, Mapping):  # nothing usable
        return None  # no_data branch
    inner = v.get("data")  # p1 publishes it enveloped (11 S3.0)
    return inner if isinstance(inner, Mapping) else v  # data object


def time_sync_answer(cache: StateCache, templates: Mapping, text: str, now_mono_ms: int, *,
                     max_age_ms: int, tz_name: Optional[str],
                     now_fn: Callable[[], float] = wall_now,
                     daemon_fn: Callable[[], bool] = chronyd_running) -> QueryAnswer:
    """The H03 reply: sync status + chronyd clause + current time; forced -> not wired."""
    values = {"now": spoken_clock(now_fn(), tz_name)}  # the time, always spoken
    if any(w in text for w in FORCE_WORDS):  # a wall-clock step was asked for
        # 99 U199: "强行对时" said only "not wired" while "时间跳过来" said
        # "已同步" -- one request, two stories. Now: not done, plus the real state.
        plain = time_sync_answer(cache, templates, "", now_mono_ms, max_age_ms=max_age_ms,
                                 tz_name=tz_name, now_fn=now_fn, daemon_fn=daemon_fn)  # status
        return QueryAnswer(False, render_reply(templates, TEMPLATE, "force_unwired",
                                               {"status": plain.text}))  # not wired + state
    values["daemon"] = render_reply(templates, TEMPLATE,  # the chronyd clause
                                    "daemon_on" if daemon_fn() else "daemon_off", {})  # read, not assumed
    d = _clock(cache, now_mono_ms, max_age_ms)  # rtk_driver's judgment, via p1
    if d is None:  # stale or never received
        return QueryAnswer(False, render_reply(templates, TEMPLATE, "no_data", values))  # honest
    if d.get("sync") is not True:  # anything but an explicit true
        return QueryAnswer(True, render_reply(templates, TEMPLATE, "unsynced", values))  # fail-safe
    source = SOURCE_NAMES.get(d.get("source"))  # closed-set name or None
    if source is None:  # unknown spelling
        return QueryAnswer(True, render_reply(templates, TEMPLATE, "synced_plain", values))  # no source
    values["source"] = source  # NTP / RTK 授时 / 本机实时时钟
    return QueryAnswer(True, render_reply(templates, TEMPLATE, "synced", values))  # synced


__all__ = ["time_sync_answer", "chronyd_running", "spoken_clock", "wall_now",
           "SOURCE_NAMES", "FORCE_WORDS", "TEMPLATE"]
