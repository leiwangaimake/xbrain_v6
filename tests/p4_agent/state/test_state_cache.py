"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_state_cache.py
Brief: GWY-P4-39 (32.G) -- G-query reads live state; stale -> unknown

Description:
Tests the freshness-aware state cache and the battery query adapter. Each
criterion carries a mutation that must turn red per CLAUDE.md 3.3:
query_battery reads the LIVE cache value (not a stub), and a stale reading
answers 'unknown' (not the last value).
"""
from __future__ import annotations

import pytest
import yaml

from xbrain.p4_agent.state.cache import STATE_TOPICS, StateCache
from xbrain.p4_agent.state.query_data import QueryAnswer, battery_answer

pytestmark = pytest.mark.no_device

_TEMPLATES_PATH = "/opt/xbrain_v6/configs/query_templates.yaml"


def _templates():
    with open(_TEMPLATES_PATH, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


# -- cache freshness core -------------------------------------------------

def test_get_fresh_returns_value_within_age():
    c = StateCache()
    c.update("state/power", {"soc": 80, "range_km": 5}, now_mono_ms=1000)
    assert c.get_fresh("state/power", now_mono_ms=1500, max_age_ms=1000) == {
        "soc": 80, "range_km": 5}


def test_get_fresh_returns_none_when_stale():
    c = StateCache()
    c.update("state/power", {"soc": 80, "range_km": 5}, now_mono_ms=1000)
    # 1200 ms later, threshold 1000 -> stale -> None.
    assert c.get_fresh("state/power", now_mono_ms=2200, max_age_ms=1000) is None


def test_get_fresh_none_when_never_received():
    c = StateCache()
    assert c.get_fresh("state/power", now_mono_ms=1000, max_age_ms=1000) is None


def test_state_topics_no_wildcard():
    # WL-G3: explicit keys only, never a wildcard subscription.
    for k in STATE_TOPICS:
        assert "*" not in k


# -- criterion 1: query_battery reads LIVE cache, not a stub -------------

def test_battery_reads_live_soc_from_cache():
    """The answer must reflect the SOC that is in the cache. MUTATION A: a
    render that returned a hardcoded stub SOC would not track the cache
    value here."""
    t = _templates()
    c = StateCache()
    c.update("state/power", {"soc": 73, "range_km": 4}, now_mono_ms=1000)
    ans = battery_answer(c, t, now_mono_ms=1200, max_age_ms=1000,
                         low_soc_threshold=20)
    assert ans.known is True
    assert "73" in ans.text                 # the LIVE soc, not a stub
    assert "4" in ans.text                   # the LIVE range


def test_battery_tracks_a_different_live_value():
    """Same call, different cache value -> different answer. If the SOC
    were hardcoded, both tests would print the same number (mutation A)."""
    t = _templates()
    c = StateCache()
    c.update("state/power", {"soc": 55, "range_km": 3}, now_mono_ms=1000)
    ans = battery_answer(c, t, now_mono_ms=1200, max_age_ms=1000,
                         low_soc_threshold=20)
    assert "55" in ans.text
    assert "73" not in ans.text


def test_battery_low_branch_below_threshold():
    t = _templates()
    c = StateCache()
    c.update("state/power", {"soc": 15, "range_km": 1}, now_mono_ms=1000)
    ans = battery_answer(c, t, now_mono_ms=1200, max_age_ms=1000,
                         low_soc_threshold=20)
    assert ans.known is True
    assert "15" in ans.text
    assert "建议尽快充电" in ans.text          # the low-branch advice


# -- criterion 2: stale state answers unknown, not the last value --------

def test_battery_stale_answers_unknown_not_last_value():
    """MUTATION B guard: a stale reading must answer 'unknown', NOT the
    last-known soc. If the adapter returned the cached value regardless of
    age, '80' would leak into the answer here."""
    t = _templates()
    c = StateCache()
    c.update("state/power", {"soc": 80, "range_km": 5}, now_mono_ms=1000)
    # 3 s later, threshold 1 s -> stale.
    ans = battery_answer(c, t, now_mono_ms=4000, max_age_ms=1000,
                         low_soc_threshold=20)
    assert ans.known is False
    assert "80" not in ans.text              # the stale value did NOT leak
    assert "读不到" in ans.text                # honest unknown reply


def test_battery_missing_source_answers_unknown():
    t = _templates()
    c = StateCache()                         # nothing ever received
    ans = battery_answer(c, t, now_mono_ms=1000, max_age_ms=1000,
                         low_soc_threshold=20)
    assert ans.known is False


def test_battery_missing_field_raises_not_fabricates():
    """A value the cache accepted but missing a required field is a
    producer contract break -> raise, never fabricate a number."""
    t = _templates()
    c = StateCache()
    c.update("state/power", {"soc": 50}, now_mono_ms=1000)   # no range_km
    with pytest.raises(KeyError):
        battery_answer(c, t, now_mono_ms=1200, max_age_ms=1000,
                       low_soc_threshold=20)   # ok branch needs range_km
