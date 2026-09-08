"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_perf.py
Brief: A-PERF-1/2 tick-budget assertions (P6 -- 20 S10.2 / #20-8)

Description:
A-PERF-1 (per-cell vectorized) and A-PERF-2 (whole-tick within budget) are the
performance assertions. Per #20-8 / RNS_TODO, they are xfail(strict=True) until
the tick is assembled and profiled on the target -- they must NOT be dropped from
CI (a silently-removed perf assertion is worse than a red one). They flip to real
timing checks when source.compute() is wired end to end (P7) and run on the ORIN.

xfail(strict=True): if the body ever unexpectedly PASSES (e.g. someone stubs it
to always succeed), strict xfail turns that into a FAILURE -- you cannot make a
perf assertion green by making it trivial.
"""

from __future__ import annotations

import pytest


@pytest.mark.xfail(strict=True, reason="#20-8: whole-tick timing lands at P7 "
                   "(source.compute wired end-to-end, profiled on ORIN)")
def test_perf_whole_tick_within_budget_A_PERF_2():
    # A-PERF-2: the WHOLE tick (not a single function) must be within the S10.2
    # budget, with a blocking open() injected reddening it. Requires the wired
    # compute() -- not available until P7. Deliberately fails now.
    raise AssertionError("tick not yet wired end-to-end (P7); #20-8")


@pytest.mark.xfail(strict=True, reason="#20-8: per-cell vectorization measured "
                   "against the naive double-for at P7 on ORIN")
def test_perf_per_cell_vectorized_A_PERF_1():
    # A-PERF-1: per-cell ops vectorized; the naive double-for must exceed budget.
    # Requires the assembled grid on the target. Deliberately fails now.
    raise AssertionError("per-cell timing not yet measurable (P7); #20-8")
