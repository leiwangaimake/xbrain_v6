"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_query_data_time.py
Brief: G24 query_time answer -- the robot's system time in the site timezone

Description:
G24 'what time is it' speaks the robot's SYSTEM time (what `date` prints) in the
site timezone (common.timezone), and the host zone when unset.

*** User ruling 2026-10-26 replaced the former 18 S9.5 hard branch, which
answered "当前时钟未同步, 时间可能不准" whenever state/clock was unsynced, stale
or missing -- on the delivered robot (no RTK sky indoors, NTP possibly
unreachable) that was the answer to every ask. The load-bearing mutants:
  * the SITE timezone is applied (Shanghai vs Tokyo differ by an hour);
  * the system clock is really read (40 s later -> the minute advances);
  * no unsync branch survives (unsynced / anchor-less / stale / missing
    state/clock all answer the time);
  * make_time_query_fn owns G24 only, so it composes with battery/RTK fns.
Tests replace query_data._wall_now so the expected strings are deterministic.
"""

from __future__ import annotations

from xbrain.p4_agent.runtime.orchestrator_turn import (
    compose_query_fns,
    make_time_query_fn,
)
from xbrain.p4_agent.state import query_data as qd
from xbrain.p4_agent.state.cache import StateCache

# 2023-11-14T22:13:20Z -> Shanghai 06:13, Tokyo 07:13 (same 2023-11-15).
_UTC = 1700000000.0
# now_mono_ms 1000 -> 1.0 s; mono_ref 1.0 s -> delta 0 -> current UTC == utc_ref.
_NOW_MS = 1000


def _clock(sync, **extra):
    """A state/clock data dict as p1's mirror_clock publishes it (sync/source +
    the (mono_ref, utc_ref) anchor). Anchor chosen so the monotonic delta is 0."""
    d = {"sync": sync, "source": "ntp" if sync else "none",
         "mono_ref": 1.0, "utc_ref": _UTC}
    d.update(extra)
    return d


def _cache(key, data, at):
    c = StateCache()
    c.update(key, {"v": 1, "data": data}, at)   # enveloped, like p1 publishes
    return c


def test_time_uses_site_zone(monkeypatch):
    """System time in the SITE zone: Shanghai and Tokyo differ by one hour for
    the same epoch -> the zone is really applied. MUTATION: ignore tz_name ->
    the two collapse -> this fails."""
    monkeypatch.setattr(qd, "_wall_now", lambda: _UTC)  # a fixed system time
    c = StateCache()
    sh = qd.time_answer(c, "Asia/Shanghai", _NOW_MS, max_age_ms=5000)
    tk = qd.time_answer(c, "Asia/Tokyo", _NOW_MS, max_age_ms=5000)
    assert sh.known is True and sh.text == "11月15日 周三 6点13分"
    assert tk.known is True and tk.text == "11月15日 周三 7点13分"


def test_time_reads_the_system_clock(monkeypatch):
    """User ruling 2026-10-26: G24 speaks the robot's own `date`. 40 s later on
    the system clock -> the minute advances. MUTATION: answer a fixed / cached
    value -> stays 6点13分 -> this fails."""
    monkeypatch.setattr(qd, "_wall_now", lambda: _UTC + 40)  # 06:13:20 + 40 s
    ans = qd.time_answer(StateCache(), "Asia/Shanghai", _NOW_MS, max_age_ms=5000)
    assert ans.known is True and ans.text == "11月15日 周三 6点14分"


def test_time_is_answered_whatever_state_clock_says(monkeypatch):
    """The former 18 S9.5 unsync warning is gone (user ruling 2026-10-26):
    unsynced, anchor-less, stale or missing state/clock all still answer the
    system time. MUTATION: keep any 'clock not synced' branch -> one of these
    answers the warning -> this fails."""
    monkeypatch.setattr(qd, "_wall_now", lambda: _UTC)  # a fixed system time
    caches = [_cache("state/clock", _clock(False), 1000),           # unsynced
              _cache("state/clock", {"sync": True, "source": "ntp"}, 1000),  # no anchor
              _cache("state/clock", _clock(True), 1000),            # stale below
              StateCache()]                                          # missing
    for c in caches:
        ans = qd.time_answer(c, "Asia/Shanghai", 10**7, max_age_ms=5000)
        assert ans.known is True and ans.text == "11月15日 周三 6点13分"
        assert "未同步" not in ans.text


def test_time_without_site_zone_uses_the_host_zone(monkeypatch):
    """common.timezone unset -> the host's local zone, never an exception."""
    monkeypatch.setattr(qd, "_wall_now", lambda: _UTC)  # a fixed system time
    ans = qd.time_answer(StateCache(), None, _NOW_MS, max_age_ms=5000)
    assert ans.known is True and "点" in ans.text


class _Entry:
    def __init__(self, iid):
        self.id = iid


def test_time_query_fn_owns_g24_only():
    c = _cache("state/clock", _clock(True), 1000)
    # Wide window so the monotonic 'now' inside make_time_query_fn stays fresh
    # relative to the 'at'=1000 we stamped (its own now_mono is much larger).
    qf = make_time_query_fn(c, "Asia/Shanghai", max_age_ms=10**12)
    assert qf(_Entry("G24")) is not None            # time owns G24
    assert qf(_Entry("G02")) is None                # battery's id, not time's
    assert qf(_Entry("G43")) is None                # RTK's id, not time's
    composed = compose_query_fns(
        [lambda e: "BAT" if e.id == "G02" else None, qf])
    assert composed(_Entry("G02")) == "BAT"
    assert composed(_Entry("G24")) is not None
