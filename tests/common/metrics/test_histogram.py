"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_histogram.py
Brief: INF-OB-1 primitive -- exact percentiles, the empty-is-not-zero rule, the
       sliding window, and monotonic timing, each with its mutant

Description:
Covers the LatencyHistogram / MetricRegistry primitive. The CI gate, the
per-module registration metatest and the P1-loop check are blocked (bench + timed
loops), so this file covers only what exists -- and says so, rather than asserting
a gate that has nothing to run against.
"""

import pytest

from xbrain.common.metrics import LatencyHistogram, MetricRegistry, REGISTRY
from xbrain.common.metrics import histogram as histmod


def test_percentile_is_exact_nearest_rank():
    """p95 of 1..100 is the 95th value. Nearest-rank returns an OBSERVED sample."""
    h = LatencyHistogram("t")
    for i in range(1, 101):
        h.record(float(i))
    assert h.report_p50() == 50.0
    assert h.report_p95() == 95.0
    assert h.report_p99() == 99.0
    assert h.report_max() == 100.0


def test_percentile_of_empty_raises_not_zero():
    """*** The empty window must raise, never return 0.

    Mutation: return 0.0 for an empty histogram and this goes red -- 0 reads as
    'instant', the most dangerous wrong answer for a latency, and a budget check
    against it would pass while measuring nothing (CLAUDE.md 3.2 form 1).
    """
    h = LatencyHistogram("empty")
    assert h.count() == 0
    with pytest.raises(ValueError):
        h.report_p95()


def test_negative_sample_is_rejected():
    """A negative latency can only come from a wall clock or a bug; reject it so
    it cannot drag a percentile below zero."""
    h = LatencyHistogram("neg")
    with pytest.raises(ValueError):
        h.record(-0.001)


def test_window_slides_and_reports_recent():
    """*** Bounded window: old samples are evicted, so the report is RECENT.

    Fill capacity with a slow value, then capacity with a fast one; the slow ones
    must be gone. Mutation: an unbounded list keeps the slow samples and p95 stays
    high -- this asserts the eviction really happened.
    """
    h = LatencyHistogram("slide", capacity=100)
    for _ in range(100):
        h.record(1.0)                       # window full of 1.0
    assert h.report_p95() == 1.0
    for _ in range(100):
        h.record(0.01)                      # overwrite every 1.0 with 0.01
    assert h.count() == 100                 # still bounded
    assert h.report_max() == 0.01           # the slow samples are gone


def test_sorted_view_stays_correct_across_eviction():
    """A regression guard on the ring+sorted bookkeeping: after wrap-around the
    percentile must still match a brute-force sort of the live samples."""
    h = LatencyHistogram("mix", capacity=5)
    for v in [5.0, 3.0, 8.0, 1.0, 9.0, 2.0, 7.0]:   # last 5 live: 8,1,9,2,7
        h.record(v)
    live = sorted([8.0, 1.0, 9.0, 2.0, 7.0])
    assert h.report_max() == live[-1]
    assert h.report_p50() == live[2]        # nearest-rank p50 of 5 = 3rd value


def test_capacity_must_be_positive():
    """A zero window could hold no sample; fail at construction, not later."""
    with pytest.raises(ValueError):
        LatencyHistogram("bad", capacity=0)


def test_timer_records_monotonic_elapsed(monkeypatch):
    """*** time() records the MONOTONIC elapsed, and does so on exit.

    mono_now_s is patched to a controlled sequence so the recorded sample is
    exact. Mutation: time on the wall clock and a wall-step between enter and exit
    would land here -- clock_scan forbids that statically and this pins it
    dynamically.
    """
    ticks = iter([10.0, 10.25])             # enter, exit -> 0.25 s elapsed
    monkeypatch.setattr(histmod, "mono_now_s", lambda: next(ticks))
    h = LatencyHistogram("timed")
    with h.time():
        pass
    assert h.count() == 1
    assert h.report_p95() == pytest.approx(0.25)


def test_timer_records_even_when_the_body_raises(monkeypatch):
    """A slow path that also threw is the one worth measuring, so the sample is
    recorded and the exception still propagates."""
    ticks = iter([1.0, 1.5])
    monkeypatch.setattr(histmod, "mono_now_s", lambda: next(ticks))
    h = LatencyHistogram("timed_err")
    with pytest.raises(RuntimeError):
        with h.time():
            raise RuntimeError("boom")
    assert h.count() == 1                    # recorded despite the raise
    assert h.report_max() == pytest.approx(0.5)


def test_registry_get_or_create_is_singleton_by_name():
    """Two call sites in one module must report into the SAME window, not two
    half-full ones."""
    reg = MetricRegistry()
    a = reg.get_or_create("p1_loop")
    b = reg.get_or_create("p1_loop")
    assert a is b
    assert reg.names() == ["p1_loop"]


def test_registry_get_unknown_raises():
    """An unknown metric name is a bug in the enumerator, not a reason to hand
    back a fresh empty metric that then reports 0."""
    reg = MetricRegistry()
    with pytest.raises(KeyError):
        reg.get("never_registered")


def test_default_registry_is_shared():
    """REGISTRY is the process-wide default a module records into at import."""
    assert isinstance(REGISTRY, MetricRegistry)
