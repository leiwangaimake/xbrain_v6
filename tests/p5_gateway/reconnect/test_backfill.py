"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_backfill.py
Brief: 17 S3.5 backfill pure-logic tests -- rate limiter, 4:1 weight, EventReplay

Description:
Batch 3. The rate limiter enforces backfill.rate_eps_total against an injected
monotonic timeline (no real sleep), the 4:1 weighted interleave orders alarm
ahead of normal without starving normal, and the EventReplay begin/item/end
shapes match 17 S3.5.3 (item verbatim, R-1). Each load-bearing assertion has a
mutation that reddens it (3.3).
"""

import pytest

from xbrain.p5_gateway.reconnect.replay import (
    DEFAULT_ALARM_WEIGHT, RateLimiter, build_begin, build_end, build_item,
    weighted_interleave,
)


pytestmark = pytest.mark.no_device


# --- rate limiter (token bucket, injected monotonic time) ---

def test_rate_limiter_caps_burst_then_refills():
    rl = RateLimiter(rate_eps=20)
    # A fresh bucket holds one second of tokens (cap=20): 20 immediate grants...
    assert all(rl.take(0.0) for _ in range(20))
    # ...and the 21st in the same instant is denied (MUTATION: no rate -> True).
    assert rl.take(0.0) is False
    # Half a second later, 10 tokens have refilled.
    assert rl.take(0.5) is True


def test_rate_limiter_refill_is_bounded_by_capacity():
    rl = RateLimiter(rate_eps=20)
    for _ in range(20):
        rl.take(0.0)
    # A long idle does not let the bucket exceed capacity (no unbounded dump).
    assert all(rl.take(100.0) for _ in range(20))
    assert rl.take(100.0) is False


def test_rate_limiter_rejects_nonpositive_rate():
    with pytest.raises(ValueError):
        RateLimiter(rate_eps=0)


# --- 4:1 weighted interleave ---

def _items(ch, n):
    return [{"ch_seq": i, "ch": ch} for i in range(1, n + 1)]


def test_weight_is_four_alarm_to_one_normal():
    order = weighted_interleave(_items("a", 8), _items("n", 2))
    chans = [c for c, _ in order]
    # First round: 4 alarm then 1 normal. MUTATION: weight=1 -> a,n,a,n...
    assert chans[:5] == ["alarm", "alarm", "alarm", "alarm", "normal"]
    assert DEFAULT_ALARM_WEIGHT == 4


def test_alarm_empty_lets_normal_drain_fully():
    order = weighted_interleave([], _items("n", 3))
    # Not throttled to 1-per-round: all three normal, in ch_seq order.
    assert [it["ch_seq"] for _, it in order] == [1, 2, 3]
    assert all(c == "normal" for c, _ in order)


def test_normal_empty_lets_alarm_drain_fully():
    order = weighted_interleave(_items("a", 6), [])
    assert [it["ch_seq"] for _, it in order] == [1, 2, 3, 4, 5, 6]
    assert all(c == "alarm" for c, _ in order)


def test_within_channel_strict_chseq_order():
    order = weighted_interleave(_items("a", 5), _items("n", 5))
    alarm_seqs = [it["ch_seq"] for c, it in order if c == "alarm"]
    normal_seqs = [it["ch_seq"] for c, it in order if c == "normal"]
    assert alarm_seqs == sorted(alarm_seqs)
    assert normal_seqs == sorted(normal_seqs)


def test_weight_zero_rejected():
    with pytest.raises(ValueError):
        weighted_interleave(_items("a", 1), _items("n", 1), alarm_weight=0)


# --- EventReplay messages (17 S3.5.3) ---

def test_build_begin_shape():
    m = build_begin("bf-1", "alarm", from_ch_seq=102, to_ch_seq=137, count=36,
                    ts_first=1.0, ts_last=2.0, outage_since_ts=0.5,
                    outage_duration_s=7205.9)
    assert m["kind"] == "begin" and m["channel"] == "alarm"
    assert m["from_ch_seq"] == 102 and m["to_ch_seq"] == 137
    assert m["outage"]["duration_s"] == 7205.9


def test_build_item_passes_event_verbatim():
    ev = {"eid": "e1", "channel": "alarm", "sev": "alarm", "ts": 9.9}
    m = build_item("bf-1", "alarm", ev)
    # R-1: the embedded event is the SAME object, no field rewritten.
    assert m["event"] is ev and m["kind"] == "item"


def test_build_item_rejects_channel_mismatch():
    # V-2: three-way channel agreement. MUTATION: drop the check -> cloud gets an
    # event whose channel disagrees with its replay key.
    with pytest.raises(ValueError):
        build_item("bf-1", "alarm", {"eid": "e1", "channel": "normal"})


def test_build_end_shape():
    m = build_end("bf-1", "normal", sent=30, given_up=1, ts_first=1.0,
                  ts_last=2.0, current_ch_seq=200)
    assert m["kind"] == "end" and m["sent"] == 30 and m["given_up"] == 1
    assert m["current_ch_seq"] == 200
