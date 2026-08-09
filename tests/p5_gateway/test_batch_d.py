"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_d.py
Brief: GWY-P5-11/12/14/15/16 deadman + probe + state/link + fence cache + bandwidth

Description:
Batch D: WS deadman fires at most once per session (WD-3), payload
matches HMI schema (WD-4); estop link_probe budget 10ms enforced
per-packet, 3 consecutive misses raise; state/link N-1/N-2 debounce
transitions; reason 6-value closed set; fence cache stale flag;
bandwidth ledger REFUSES boost + borrow (UG-1/UG-2 non-negotiable).
"""

import pytest

from xbrain.p5_gateway.bandwidth.ledger import (
    BandwidthBoostForbidden, Ledger, PLANES, UnknownPlane,
)
from xbrain.p5_gateway.fence.cache import (
    DirectDbAccessForbidden, FenceCache, refuse_direct_db_read,
)
from xbrain.p5_gateway.hmi.deadman import (
    DeadmanTracker, build_deadman_payload, within_response_window,
)
from xbrain.p5_gateway.link.probe import (
    LinkProbeMissed, MISSED_THRESHOLD, PROBE_BUDGET_MS, ProbeStats,
    record_miss, record_receive, record_send,
)
from xbrain.p5_gateway.link.state_link import (
    LINK_REASONS, LinkStatus,
    apply_down_debounce, apply_up_debounce,
)


pytestmark = pytest.mark.no_device


# --- GWY-P5-11 deadman ---

def test_deadman_fires_first_time():
    t = DeadmanTracker()
    assert t.should_fire("sess1") is True


def test_deadman_dedupes_second_time():
    """WD-3: at most once per session."""
    t = DeadmanTracker()
    t.mark_fired("sess1")
    assert t.should_fire("sess1") is False


def test_deadman_payload_shape():
    """WD-4: identical to HMI's own release message."""
    p = build_deadman_payload()
    assert p["deadman"] is False
    assert p["vx"] == 0.0 and p["vy"] == 0.0 and p["wz"] == 0.0


def test_deadman_within_response_window():
    assert within_response_window(now_ms=50, disconnect_ms=0,
                                    response_ms=100) is True


def test_deadman_response_window_exceeded():
    assert within_response_window(now_ms=200, disconnect_ms=0,
                                    response_ms=100) is False


# --- GWY-P5-12 link probe ---

def test_probe_normal_rtt_ok():
    s = ProbeStats()
    record_send(s)
    record_receive(s, rtt_ms=5.0)
    assert s.received == 1 and s.missed_streak == 0


def test_probe_over_budget_raises():
    s = ProbeStats()
    record_send(s)
    with pytest.raises(LinkProbeMissed, match="exceeds budget"):
        record_receive(s, rtt_ms=PROBE_BUDGET_MS + 1.0)


def test_probe_three_misses_raise():
    s = ProbeStats()
    for _ in range(MISSED_THRESHOLD - 1):
        record_send(s)
        record_miss(s)   # no raise yet
    record_send(s)
    with pytest.raises(LinkProbeMissed, match="consecutive"):
        record_miss(s)


def test_probe_receive_resets_miss_streak():
    s = ProbeStats(missed_streak=2)
    record_send(s)
    record_receive(s, rtt_ms=3.0)
    assert s.missed_streak == 0


# --- GWY-P5-14 state/link debounce ---

def test_link_reasons_closed_set_of_six():
    assert LINK_REASONS == {
        "healthy", "gateway_down", "target_down",
        "handshake_fail", "stale", "manually_disabled",
    }


def test_up_debounce_delays_publish():
    """N-1: first sighting of up doesn't publish; second after
    debounce does."""
    s = LinkStatus(name="cloud", up=False, since_ms=0, reason="target_down")
    changed = apply_up_debounce(s, now_ms=100, debounce_ms=1000,
                                  reason="healthy")
    assert changed is False and s.up is False    # first sighting
    changed = apply_up_debounce(s, now_ms=1200, debounce_ms=1000,
                                  reason="healthy")
    assert changed is True and s.up is True


def test_down_debounce_delays_publish():
    s = LinkStatus(name="cloud", up=True, since_ms=0, reason="healthy")
    changed = apply_down_debounce(s, now_ms=100, debounce_ms=1000,
                                    reason="stale")
    assert changed is False and s.up is True
    changed = apply_down_debounce(s, now_ms=1200, debounce_ms=1000,
                                    reason="stale")
    assert changed is True and s.up is False


def test_up_debounce_unknown_reason_raises():
    s = LinkStatus(name="cloud", up=False, since_ms=0, reason="target_down")
    with pytest.raises(ValueError, match="closed set"):
        apply_up_debounce(s, 100, 1000, reason="halfway")


# --- GWY-P5-15 fence cache ---

def test_fence_cache_on_update():
    c = FenceCache()
    c.on_update([{"id": "f1"}], now_ms=100)
    assert c.fences[0]["id"] == "f1"


def test_fence_cache_stale_after():
    """P5F-2: stale flag when older than threshold."""
    c = FenceCache()
    c.on_update([{"id": "f1"}], now_ms=0)
    fences, stale = c.snapshot(now_ms=200, stale_after_ms=100)
    assert stale is True and fences[0]["id"] == "f1"


def test_fence_cache_fresh_not_stale():
    c = FenceCache()
    c.on_update([{"id": "f1"}], now_ms=0)
    fences, stale = c.snapshot(now_ms=50, stale_after_ms=100)
    assert stale is False


def test_fence_direct_db_forbidden():
    """P5F-3: p5 must not read fence.db directly (owner is p3)."""
    with pytest.raises(DirectDbAccessForbidden):
        refuse_direct_db_read("query_polygon")


# --- GWY-P5-16 bandwidth ledger ---

def test_ledger_planes_closed_set():
    assert PLANES == frozenset({"control", "data", "media"})


def test_ledger_rejects_unknown_plane_at_construction():
    with pytest.raises(UnknownPlane):
        Ledger(budgets={"halfway": 100})


def test_ledger_rejects_unknown_plane_at_record():
    l = Ledger(budgets={"control": 1000})
    with pytest.raises(UnknownPlane):
        l.record("halfway", 100)


def test_ledger_over_budget_true_when_exceeded():
    l = Ledger(budgets={"control": 100})
    l.record("control", 150)
    assert l.over_budget("control") is True


def test_ledger_over_budget_false_when_within():
    l = Ledger(budgets={"control": 100})
    l.record("control", 50)
    assert l.over_budget("control") is False


def test_ledger_raise_budget_refused():
    """UG-2: dynamic budget raise is refused, ALWAYS."""
    l = Ledger(budgets={"control": 100})
    with pytest.raises(BandwidthBoostForbidden, match="cannot raise"):
        l.raise_budget("control", 200)


def test_ledger_borrow_between_planes_refused():
    """UG-1: cannot borrow across planes."""
    l = Ledger(budgets={"control": 100, "data": 100})
    with pytest.raises(BandwidthBoostForbidden, match="cannot borrow"):
        l.borrow_from("data", "control")


def test_ledger_reset_clears_usage():
    l = Ledger(budgets={"control": 100})
    l.record("control", 50)
    l.reset()
    assert l.over_budget("control") is False
