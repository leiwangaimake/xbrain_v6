"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_query_data_rtk.py
Brief: F5 runtime G43-G47 answers from live state (fresh vs stale) + dispatch

Description:
The G43-G47 answers must read the LIVE state/pose + state/clock and, when the
reading is stale, return the honest 'unknown' text -- never a last-known RTK
value spoken as current (16 S8.2.1 QT shadow rule). The load-bearing mutants
(CLAUDE.md 3.3): a stale state/pose must answer 'unknown' (not the old fix_type),
and the RTK query_fn must own G43-G47 while leaving G02 to the battery fn.
"""

from __future__ import annotations

import time

from xbrain.p4_agent.runtime.orchestrator_turn import (
    compose_query_fns, make_rtk_query_fn,
)
from xbrain.p4_agent.state import query_data as qd
from xbrain.p4_agent.state.cache import StateCache


def _cache(key, data, at):
    c = StateCache()
    c.update(key, {"v": 1, "data": data}, at)   # enveloped, like p1 publishes
    return c


def test_rtk_fix_fresh():
    c = _cache("state/pose", {"fix_type": "rtk_fixed"}, 1000)
    ans = qd.rtk_fix_answer(c, 1000, max_age_ms=5000)
    assert ans.known is True and "厘米级" in ans.text


def test_rtk_fix_stale_is_unknown():
    # 3.3 QT shadow mutant: a 9 s old pose (> 5 s) MUST answer 'unknown', never
    # the stale rtk_fixed spoken as current.
    c = _cache("state/pose", {"fix_type": "rtk_fixed"}, 1000)
    ans = qd.rtk_fix_answer(c, 10000, max_age_ms=5000)
    assert ans.known is False and "读不到" in ans.text


def test_heading_status_h1():
    # H-1: heading_valid=False answers 'no heading' even with level 1.
    c = _cache("state/pose", {"heading_valid": False, "heading_level": 1}, 1000)
    ans = qd.heading_status_answer(c, 1000, max_age_ms=5000)
    assert ans.known is True and ans.text == "当前没有可用航向"


def test_clock_sync_fresh():
    c = _cache("state/clock", {"sync": True, "source": "rtk"}, 1000)
    ans = qd.clock_sync_answer(c, 1000, max_age_ms=5000)
    assert "已同步" in ans.text


def test_satellites_and_source():
    c = _cache("state/pose", {"num_satellites": 22, "heading_source": "dual_antenna"}, 1000)
    assert "22" in qd.satellites_answer(c, 1000, max_age_ms=5000).text
    assert "双天线" in qd.heading_source_answer(c, 1000, max_age_ms=5000).text


class _Entry:
    def __init__(self, iid):
        self.id = iid


def test_rtk_query_fn_owns_g43_g47_not_g02():
    now = int(time.monotonic() * 1000)
    c = _cache("state/pose", {"fix_type": "single"}, now)
    qf = make_rtk_query_fn(c, max_age_ms=600000)   # wide window so 'now' is fresh
    assert qf(_Entry("G43")) is not None            # RTK owns G43
    assert qf(_Entry("G45")) is not None
    assert qf(_Entry("G02")) is None                # battery's id, not RTK's
    # compose: a stub battery fn + the RTK fn; each keeps its ids.
    composed = compose_query_fns([lambda e: "BAT" if e.id == "G02" else None, qf])
    assert composed(_Entry("G02")) == "BAT"
    assert composed(_Entry("G43")) is not None
