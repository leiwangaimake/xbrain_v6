"""BIZ-P2-14 + P2-15 -- rules loader + target ledger tests."""

import pytest

from xbrain.p2_core.suspicion.rules_loader import (
    Rule, Ruleset, RulesSchemaError,
    filter_by_night_patrol, filter_by_ts_sync, parse_ruleset,
)
from xbrain.p2_core.suspicion.target_track import TargetLedger


pytestmark = pytest.mark.no_device


# --- rules_loader parse ---

def test_empty_yaml_produces_empty_ruleset():
    rs = parse_ruleset("")
    assert rs.rules == []


def test_valid_rule_parses():
    yml = """
rules:
  - id: r1
    when: {kind: target_seen, class: person}
    then: {action: log}
"""
    rs = parse_ruleset(yml)
    assert len(rs.rules) == 1
    assert rs.rules[0].id == "r1"


def test_rule_missing_id_raises():
    yml = """
rules:
  - when: {}
    then: {}
"""
    with pytest.raises(RulesSchemaError):
        parse_ruleset(yml)


def test_rule_missing_then_raises():
    yml = """
rules:
  - id: r1
    when: {}
"""
    with pytest.raises(RulesSchemaError):
        parse_ruleset(yml)


def test_non_mapping_root_raises():
    """A list at root, not a mapping."""
    with pytest.raises(RulesSchemaError):
        parse_ruleset("- a\n- b\n")


def test_time_window_carried_through():
    yml = """
rules:
  - id: r_night
    when: {}
    then: {}
    time_window: {start: "22:00", end: "05:00"}
"""
    rs = parse_ruleset(yml)
    assert rs.rules[0].time_window == {"start": "22:00", "end": "05:00"}


# --- filter helpers ---

def test_filter_by_night_patrol_drops_when_disabled():
    """RE-7: rules that require night_patrol are dropped when it's off."""
    rs = Ruleset(rules=[
        Rule(id="r1", when={}, then={}),
        Rule(id="r2_night", when={}, then={}, requires_night_patrol=True),
    ])
    filtered = filter_by_night_patrol(rs, night_patrol_enabled=False)
    assert {r.id for r in filtered.rules} == {"r1"}


def test_filter_by_night_patrol_keeps_when_enabled():
    rs = Ruleset(rules=[
        Rule(id="r2_night", when={}, then={}, requires_night_patrol=True),
    ])
    filtered = filter_by_night_patrol(rs, night_patrol_enabled=True)
    assert len(filtered.rules) == 1


def test_filter_by_ts_sync_drops_time_window_rules_when_unsynced():
    """RE-3a: rules with time_window need ts_sync=true."""
    rs = Ruleset(rules=[
        Rule(id="r_always", when={}, then={}),
        Rule(id="r_night", when={}, then={},
             time_window={"start": "22:00", "end": "05:00"}),
    ])
    filtered = filter_by_ts_sync(rs, ts_sync=False)
    assert {r.id for r in filtered.rules} == {"r_always"}


# --- target ledger ---

def test_observe_creates_new_track():
    lg = TargetLedger(lost_confirm_frames=15, track_ttl_ms=30_000)
    lg.observe(1, now_mono_ms=0)
    assert 1 in lg.active_ids()


def test_absent_frames_confirm_lost_after_n():
    lg = TargetLedger(lost_confirm_frames=3, track_ttl_ms=30_000)
    lg.observe(1, now_mono_ms=0)
    lg.tick_frame_absent([1])   # 1
    lg.tick_frame_absent([1])   # 2
    newly_lost = lg.tick_frame_absent([1])   # 3 -> confirmed
    assert newly_lost == [1]
    assert 1 not in lg.active_ids()


def test_reobserve_after_absent_resets_counter():
    lg = TargetLedger(lost_confirm_frames=3, track_ttl_ms=30_000)
    lg.observe(1, now_mono_ms=0)
    lg.tick_frame_absent([1])
    lg.tick_frame_absent([1])
    # Re-observe before threshold -> counter resets.
    lg.observe(1, now_mono_ms=100)
    newly = lg.tick_frame_absent([1])
    assert newly == []   # not lost yet


def test_ttl_evicts_stale_track():
    lg = TargetLedger(lost_confirm_frames=15, track_ttl_ms=1000)
    lg.observe(1, now_mono_ms=0)
    evicted = lg.evict_ttl(now_mono_ms=1500)
    assert 1 in evicted
    assert 1 not in lg.active_ids()
