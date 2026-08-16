"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_query_data_time.py
Brief: G24 query_time answer -- hard unsync branch + site-timezone local time

Description:
G24 'what time is it' has a HARD safety-shaped branch (18 S9.5): when the clock
is NOT synced (or state/clock is stale / never received), the answer is the
unsync warning, NEVER a possibly-wrong time. The wall clock steps at RTK cold
start, so answering the time then would be wrong at the worst moment. The
load-bearing mutants (CLAUDE.md 3.3):

  * unsynced -> warning. A mutant that answers the formatted time regardless of
    sync fails test_time_unsynced_is_warning and test_time_missing_clock.
  * the SITE timezone is applied. Shanghai (+8) and Tokyo (+9) differ by an hour
    for the same utc epoch; a mutant that ignores tz_name collapses them and
    fails test_time_synced_uses_site_zone.
  * make_time_query_fn owns G24 only, so it composes with battery/RTK fns.

The current UTC is reconstructed from state/clock's (mono_ref, utc_ref) anchor as
utc_ref + (now_mono - mono_ref). Vectors pick mono_ref so the delta is 0 (or a
known small value), so no wall-clock read makes the expected string deterministic
across hosts and timezones.
"""

from __future__ import annotations

from xbrain.p4_agent.runtime.orchestrator_turn import (
    compose_query_fns, make_time_query_fn,
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


def test_time_synced_uses_site_zone():
    """Synced clock -> spoken local time in the SITE zone, reconstructed from the
    anchor. Shanghai and Tokyo differ by one hour for the same utc -> the zone is
    really applied."""
    c = _cache("state/clock", _clock(True), 1000)
    sh = qd.time_answer(c, "Asia/Shanghai", _NOW_MS, max_age_ms=5000)
    tk = qd.time_answer(c, "Asia/Tokyo", _NOW_MS, max_age_ms=5000)
    assert sh.known is True and sh.text == "11月15日 周三 6点13分"
    assert tk.known is True and tk.text == "11月15日 周三 7点13分"


def test_time_monotonic_delta_advances_the_clock():
    """The reconstruction really adds the monotonic delta: 40 s later (now_mono
    +40000 ms) the spoken minute advances. MUTATION: ignore the delta and answer
    utc_ref verbatim -> stays 6点13分 -> this fails."""
    c = _cache("state/clock", _clock(True), 1000)
    # 40 s of elapsed monotonic time: 06:13:20 + 40 s = 06:14:00.
    ans = qd.time_answer(c, "Asia/Shanghai", _NOW_MS + 40000, max_age_ms=10**9)
    assert ans.known is True and ans.text == "11月15日 周三 6点14分"


def test_time_unsynced_is_warning():
    """HARD BRANCH: sync=False -> warning, never the time. MUTATION: answer the
    formatted time regardless of sync -> this fails."""
    c = _cache("state/clock", _clock(False), 1000)
    ans = qd.time_answer(c, "Asia/Shanghai", _NOW_MS, max_age_ms=5000)
    assert ans.known is False and "未同步" in ans.text


def test_time_synced_without_anchor_is_warning():
    """Synced but the (mono_ref, utc_ref) anchor is absent (older publisher) ->
    warning, never a wall-clock-read fabrication. MUTATION: fall back to a local
    time.time() read here -> this would answer a time -> fails."""
    c = _cache("state/clock", {"sync": True, "source": "ntp"}, 1000)
    ans = qd.time_answer(c, "Asia/Shanghai", _NOW_MS, max_age_ms=5000)
    assert ans.known is False and "未同步" in ans.text


def test_time_stale_clock_is_warning():
    """A 9 s old clock (> 5 s) is treated as unsynced (unknown), not the time."""
    c = _cache("state/clock", _clock(True), 1000)
    ans = qd.time_answer(c, "Asia/Shanghai", 10000, max_age_ms=5000)
    assert ans.known is False and "未同步" in ans.text


def test_time_missing_clock():
    """No state/clock at all -> warning (fail-safe), never a fabricated time."""
    c = StateCache()
    ans = qd.time_answer(c, "Asia/Shanghai", _NOW_MS, max_age_ms=5000)
    assert ans.known is False and "未同步" in ans.text


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
