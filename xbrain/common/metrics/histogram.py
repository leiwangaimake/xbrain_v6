"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: histogram.py
Brief: INF-OB-1 latency primitive -- a monotonic-clock timer with exact p50/p95/
       p99/max over a bounded sample window, and a registry to enumerate them

Description:
Every module with a latency budget (11 / 10 timing budgets, the P1 20 Hz loop,
CRL-5) needs to measure its own timing and expose the quantile the budget is
stated against. INF-OB-1 wants that measurement to be uniform: one primitive, so
"module X's p95" means the same thing everywhere, and a registry, so a CI gate
(and the "every module exposes its timing" metatest) can enumerate what exists.

This file is the PRIMITIVE half of INF-OB-1, and only that. It is deliberately
usable before the pieces it feeds exist:
  * timing uses the monotonic clock (mono_now_s, CLK-C1) -- a wall-clock step
    must never turn into a negative or huge latency sample. clock_scan.py forbids
    the wall clock here statically; this uses mono_now_s so it passes for real.
  * it holds a bounded window of recent samples and computes an EXACT nearest-
    rank percentile over them -- not an approximation, because at the sample
    counts a control loop produces (thousands), exact is cheap and a t-digest's
    error would be one more thing to reason about at 3am.

What is NOT here, because it is blocked, not forgotten (stated so the gap is a
decision -- CLAUDE.md 3.2):
  * scripts/ci/latency_gate.py comparing p95 against a budget table: the TS-8
    bench artifact it would read does not exist yet, and the "10 S11.1 budget
    table" the item names is actually the process list -- the per-hop budget is
    still scattered across 11 (急停 200 ms, P1 50 ms, CRL-5 200 us). Building the
    gate now would be a green shell with nothing to compare (CLAUDE.md 3.2
    form 1).
  * the metatest "every implemented module registered its timing metric": there
    is no timed loop yet (P1 is Phase 1), so it would be vacuous today.
  * the P1-loop-specific P99 <= 60 ms / max <= 100 ms check: P1 does not exist.
The registry below is what those three will enumerate when they land.
"""

import contextlib
import math
from collections import deque
from typing import Deque, Dict, Iterator, List

from xbrain.common.clock import mono_now_s

__all__ = ["LatencyHistogram", "MetricRegistry", "REGISTRY"]

#: Default window size. Not a safety parameter (common.safety.*), so a code
#: default is allowed here (CLAUDE.md 3.1 governs the safety namespaces): it
#: bounds memory and quantile accuracy, not a control decision. A control loop at
#: 20-50 Hz fills a few thousand in a couple of minutes, enough for a stable p99.
_DEFAULT_CAPACITY = 4096


class LatencyHistogram:
    """A named latency metric: record seconds, read p50/p95/p99/max.

    Bounded to the most recent `capacity` samples (a sliding window), so a
    long-running process reports RECENT latency rather than a lifetime average
    that never recovers from one early stall. Not thread-safe: a module records
    from its own single loop thread, the same discipline the arbiter and the
    envelope writer use.
    """

    def __init__(self, name: str, capacity: int = _DEFAULT_CAPACITY) -> None:
        # name identifies the metric to the registry and the future CI gate, so
        # "p1_loop"'s p95 is findable by that key.
        self.name = name
        if capacity <= 0:
            # A zero/negative window could never hold a sample, so every quantile
            # would be empty -- fail loud at construction, not silently later.
            raise ValueError("capacity must be positive, got %d" % capacity)
        # A deque with maxlen IS the sliding window: append is O(1) and evicts the
        # oldest automatically, so record() -- the hot path, called every control
        # cycle -- does no shifting. The samples are sorted lazily on read
        # instead, because reads (a report) are rare next to writes and sorting a
        # few thousand floats then is cheaper than keeping a sorted list in step
        # on every append (a list insort is O(n) from the shift).
        self._samples: Deque[float] = deque(maxlen=capacity)

    def record(self, value_s: float) -> None:
        """Record one latency sample, in seconds.

        A negative sample is rejected: it can only come from a wall-clock read
        (which this module does not do) or a bug, and letting it in would drag a
        percentile below zero and mask the real distribution.
        """
        if value_s < 0:
            raise ValueError("latency sample must be >= 0, got %r" % value_s)
        self._samples.append(value_s)       # O(1); maxlen evicts the oldest

    @contextlib.contextmanager
    def time(self) -> Iterator[None]:
        """Context manager: record the monotonic elapsed seconds on exit.

        Usage:
            with metric.time():
                do_the_thing()
        Uses mono_now_s at both ends, so a wall-clock step between them cannot
        produce a bogus sample (CLK-C1).
        """
        start = mono_now_s()                # monotonic start (CLK-C1)
        try:
            yield
        finally:
            # finally, not else: a slow path that ALSO threw is exactly the one
            # to measure. The exception still propagates past this block.
            self.record(mono_now_s() - start)

    def count(self) -> int:
        """How many samples the window currently holds."""
        return len(self._samples)

    def percentile(self, q: float) -> float:
        """The exact nearest-rank percentile q (0..100) over the window.

        Nearest-rank, not linear interpolation: for a latency budget "p95 <= X"
        the question is "is the 95th-percentile sample within budget", and the
        nearest-rank sample IS an observed value, not an interpolated one that
        never occurred. Raises on an empty window rather than returning 0, because
        0 would read as "instant", the most dangerous wrong answer for a latency.
        """
        n = len(self._samples)
        if n == 0:
            # Empty -> raise, never 0 (see the docstring: 0 reads as "instant").
            raise ValueError("percentile of an empty histogram %r" % self.name)
        if not 0 <= q <= 100:
            # Out of range is a caller bug, not a value to clamp silently.
            raise ValueError("percentile q must be in [0, 100], got %r" % q)
        ordered = sorted(self._samples)         # sort lazily, on read (see __init__)
        # Nearest-rank: rank = ceil(q/100 * n), 1-indexed. ceil (not round or
        # floor) so p100 lands on n and p95 rounds UP toward the tail -- a budget
        # asks "is the 95th within X", and rounding down would answer for a
        # slightly better percentile than the one named.
        rank = math.ceil((q / 100.0) * n)
        # Clamp: q=0 gives rank 0, which would index -1 (the MAX) -- the exact
        # opposite of p0. min(n,..) guards the p100 edge on tiny windows.
        rank = max(1, min(n, rank))
        return ordered[rank - 1]                # 1-indexed rank -> 0-indexed list

    def report_p50(self) -> float:
        """Median latency (seconds)."""
        return self.percentile(50)

    def report_p95(self) -> float:
        """95th-percentile latency (seconds) -- the value most budgets name."""
        return self.percentile(95)

    def report_p99(self) -> float:
        """99th-percentile latency (seconds) -- the P1 loop budget's quantile."""
        return self.percentile(99)

    def report_max(self) -> float:
        """The worst sample in the window (seconds). Raises on empty, like
        percentile -- a max of nothing is not 0."""
        return self.percentile(100)


class MetricRegistry:
    """A name -> LatencyHistogram map, so something can enumerate every module's
    timing metric.

    This is what the (blocked) "every implemented module registered its metric"
    metatest and the (blocked) CI gate will iterate. get_or_create keeps a
    module's metric a singleton by name, so two call sites in one module report
    into the same window instead of two half-full ones.
    """

    def __init__(self) -> None:
        # name -> histogram. Plain dict: registration happens at import on one
        # thread, so no lock is needed (same single-thread discipline as elsewhere).
        self._metrics: Dict[str, LatencyHistogram] = {}

    def get_or_create(self, name: str,
                      capacity: int = _DEFAULT_CAPACITY) -> LatencyHistogram:
        """The histogram named `name`, creating it once if absent.

        get-or-create, not create: two call sites in one module asking for the
        same name must get the SAME window, or each records half the samples and
        both p95s are wrong (see the class docstring).
        """
        hist = self._metrics.get(name)
        if hist is None:
            # First request for this name: build it and remember it.
            hist = LatencyHistogram(name, capacity)
            self._metrics[name] = hist
        return hist                             # subsequent calls return the same one

    def names(self) -> List[str]:
        """Every registered metric name, sorted, for enumeration.

        Sorted so the (future) CI gate and metatest iterate deterministically --
        an unstable order would make their output diff for no real change.
        """
        return sorted(self._metrics)

    def get(self, name: str) -> LatencyHistogram:
        """The histogram named `name`, or raise -- an unknown name is a bug in the
        enumerator, not something to paper over with a fresh empty metric that
        would then report a misleading 0."""
        return self._metrics[name]              # KeyError on an unknown name, on purpose


#: The process-wide default registry. A module does
#: `REGISTRY.get_or_create("p1_loop")` at import and records into it each cycle.
REGISTRY = MetricRegistry()
