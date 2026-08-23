"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_threads.py
Brief: BIZ-P2-1 p2_core thread budget -- per-tick budget + consecutive-overrun block

Description:
*** Brief 由占位串改写(2026-08-23). 原值是按路径自动生成的
"p2_core tests -- threads" -- 既没说清本文件测什么, 也无法据以索引任务号, 于是 P2 是唯一
无法自动提取证据映射的子系统(CLAUDE.md 2.5 要求 Brief 一行说清).
BIZ-P2-1 -- main-loop budget + BLOCKED + fault continuation tests.
"""


import pytest

from xbrain.p2_core.threads import (
    BLOCKED_AFTER_OVER_BUDGET_TICKS,
    MAIN_TICK_BUDGET_MS,
    MainLoop,
    TickReport,
)


pytestmark = pytest.mark.no_device


class _FakeClock:
    """Deterministic ms clock; steps by `advance` per tick."""

    def __init__(self, advance_ms):
        self.now = 0
        self._advance = advance_ms

    def __call__(self):
        # Called at t0 and t1; we advance BETWEEN the two calls
        # so a tick's measured duration = _advance.
        val = self.now
        self.now += self._advance
        return val


def _make_loop(tick_advance_ms, tick_body, on_fault=None, on_blocked=None):
    clock = _FakeClock(tick_advance_ms)
    return MainLoop(
        clock_ms=clock,
        tick_body=tick_body,
        on_fault=on_fault or (lambda m: None),
        on_blocked=on_blocked or (lambda: None),
    )


# --- Under-budget tick reports over_budget=False ------------------

def test_under_budget_tick_ok():
    loop = _make_loop(tick_advance_ms=10, tick_body=lambda: None)
    r = loop.run_one_tick()
    assert r.over_budget is False
    assert r.duration_ms == 10
    assert r.faulted is False
    assert loop.blocked is False


# --- Over-budget single tick reports over_budget=True, not blocked -

def test_over_budget_single_tick_marks_but_does_not_block():
    loop = _make_loop(tick_advance_ms=60, tick_body=lambda: None)
    r = loop.run_one_tick()
    assert r.over_budget is True
    assert loop.blocked is False   # only 1 in a row


# --- Consecutive over-budget triggers BLOCKED ---------------------

def test_n_consecutive_over_budget_triggers_blocked_transition():
    """POSITIVE (spec 補): after BLOCKED_AFTER_OVER_BUDGET_TICKS
    consecutive over-budget ticks, MainLoop transitions to BLOCKED
    and calls on_blocked exactly once."""
    blocked_calls = []
    loop = _make_loop(
        tick_advance_ms=60,   # every tick over budget
        tick_body=lambda: None,
        on_blocked=lambda: blocked_calls.append(True),
    )
    for _ in range(BLOCKED_AFTER_OVER_BUDGET_TICKS):
        loop.run_one_tick()

    assert loop.blocked is True
    assert blocked_calls == [True], \
        "on_blocked must be called EXACTLY ONCE (not per subsequent tick)"


def test_on_blocked_only_fires_once_even_if_more_ticks_over_budget():
    """VARIANT: after BLOCKED, further over-budget ticks must not
    re-invoke on_blocked (would be a duplicate downgrade event)."""
    calls = []
    loop = _make_loop(
        tick_advance_ms=60,
        tick_body=lambda: None,
        on_blocked=lambda: calls.append(True),
    )
    for _ in range(BLOCKED_AFTER_OVER_BUDGET_TICKS + 5):
        loop.run_one_tick()
    assert calls == [True]


def test_single_ok_tick_resets_consecutive_counter():
    """VARIANT: an under-budget tick between over-budget ticks resets
    the counter -- 3 over, 1 ok, 3 over = 6 over total but NOT
    blocked (broken into runs of 3+3, neither hits the N=3 threshold
    after the reset)."""
    # BLOCKED_AFTER_OVER_BUDGET_TICKS = 3.
    # Sequence: over, over, ok, over, over. Only 2 in a row: NO blocked.
    times = [60, 60, 10, 60, 60]
    calls = []

    class _Seq:
        def __init__(self, seq):
            self.seq = list(seq)
            self.now = 0

        def __call__(self):
            # Two calls per tick: t0 then t1 with delta = next value.
            val = self.now
            if self.seq:
                self.now += self.seq.pop(0)
            return val

    loop = MainLoop(
        clock_ms=_Seq(times),
        tick_body=lambda: None,
        on_fault=lambda m: None,
        on_blocked=lambda: calls.append(True),
    )
    for _ in range(len(times)):
        loop.run_one_tick()
    assert loop.blocked is False
    assert calls == []


# --- Fault in tick body: caught, logged, loop continues -----------

def test_raise_in_tick_body_captured_by_on_fault():
    """POSITIVE (CLAUDE.md 4.4): raise inside tick body must be
    caught, faulted=True in report, on_fault called with the message,
    loop remains alive so next tick can run."""
    faults = []

    def bad_body():
        raise RuntimeError("kaboom")

    loop = _make_loop(
        tick_advance_ms=10,
        tick_body=bad_body,
        on_fault=lambda m: faults.append(m),
    )
    r = loop.run_one_tick()
    assert r.faulted is True
    assert "kaboom" in r.fault
    assert len(faults) == 1
    assert "kaboom" in faults[0]

    # And a second tick still runs (loop is not dead).
    r2 = loop.run_one_tick()
    assert r2.tick_no == 2


def test_fault_sink_that_itself_raises_does_not_kill_loop():
    """Robustness: on_fault raising must not propagate."""

    def bad_body():
        raise RuntimeError("kaboom")

    def bad_sink(m):
        raise RuntimeError("sink broke too")

    loop = _make_loop(
        tick_advance_ms=10,
        tick_body=bad_body,
        on_fault=bad_sink,
    )
    # Must not raise despite both body and sink raising.
    loop.run_one_tick()
    loop.run_one_tick()


# --- Positive: over-budget WITHOUT continuous does not block -----

def test_over_budget_with_gap_never_blocks():
    """VARIANT of the spec supplement: only CONSECUTIVE over-budget
    ticks count. A single ok tick in between resets the counter.

    Uses a per-tick clock that returns (now, now+advance) then jumps
    to the next slot; each element in `durations_ms` is that tick's
    measured duration in isolation."""
    calls = []
    durations_ms = [60, 10, 60, 60, 10, 60, 60, 10, 60, 60]

    class _PerTickClock:
        """Two calls per tick: first returns baseline, second returns
        baseline + this tick's duration. Advance baseline BETWEEN ticks."""

        def __init__(self, seq):
            self.seq = list(seq)
            self.baseline = 0
            self.state = 0  # 0 = first call this tick, 1 = second

        def __call__(self):
            if self.state == 0:
                self.state = 1
                return self.baseline
            # second call
            self.state = 0
            d = self.seq.pop(0)
            r = self.baseline + d
            self.baseline = r
            return r

    loop = MainLoop(
        clock_ms=_PerTickClock(durations_ms),
        tick_body=lambda: None,
        on_fault=lambda m: None,
        on_blocked=lambda: calls.append(True),
    )
    for _ in range(len(durations_ms)):
        loop.run_one_tick()
    # Longest consecutive over-budget run: 2 (60,60) < BLOCKED_AFTER=3.
    assert loop.blocked is False, \
        "blocked history=%s" % [(h.tick_no, h.duration_ms) for h in loop.history]
    assert calls == []


# --- Meta: budget constant matches doc (14 S2.3 P-1: 50 ms) -------

def test_budget_constant_matches_doc():
    assert MAIN_TICK_BUDGET_MS == 50
    assert BLOCKED_AFTER_OVER_BUDGET_TICKS == 3
