"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: threads.py
Brief: BIZ-P2-1 -- p2_core 5-thread model with realtime budget

Description:
14 S2.1 defines P2's process shape as five threads:

  main        10 Hz   arbiter tick, mode SM, health aggregation
  fast        50 Hz   preempt + estop only
  rx          event   Zenoh subscriber callbacks
  tx          10 Hz   publish state/mode, health/*, state/arb, rt/audio/gate
  payload_io  async   HTTP/WS to payload-service (never blocks main/fast)

Real-time budget (14 S2.3):
  P-1  main tick P99 <= 50 ms (50% of 10 Hz period)
  P-2  health/factor publish rate >= 1 Hz stable
  P-3  ANY payload-service call posted from main/fast is a defect --
       it MUST go through payload_io, which times out at T-PAY-1 = 300 ms

This module owns the tick scheduler + per-tick budget check + BLOCKED
transition on 3 consecutive over-budget ticks (BIZ-P2-1 spec 補).

The threads themselves are pluggable: production wires them to real
threading.Thread with SCHED_FIFO priorities; tests use a synchronous
FakeScheduler that ticks in-process. Both flow through the same
budget-check logic so a spec variant (over-budget -> BLOCKED after
N consecutive) is validated without spinning wall-clock time.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional


# 14 S2.3 P-1: per-tick budget in milliseconds.
MAIN_TICK_BUDGET_MS = 50

# BIZ-P2-1 supplement: how many consecutive over-budget ticks before
# the loop enters BLOCKED (§10). 3 is the doc value.
BLOCKED_AFTER_OVER_BUDGET_TICKS = 3


class LoopFault(RuntimeError):
    """Raised by the tick body's exception path. Caller (main loop)
    logs a fault event AND continues the next tick per CLAUDE.md 4.4:
    a raise inside the loop is one-tick zero speed + fault, NOT a
    process crash."""


@dataclass
class TickReport:
    """One tick's outcome for observability + budget accounting."""
    tick_no: int
    duration_ms: float
    over_budget: bool
    faulted: bool
    fault: Optional[str] = None


@dataclass
class MainLoop:
    """The 10 Hz main thread abstract state machine.

    Not the actual thread -- this class holds the tick semantics
    (budget check, consecutive over-budget counter, BLOCKED trip,
    fault-continue) so tests can drive it directly.
    """
    # Injected clock in ms so tests do not spin real time.
    clock_ms: Callable[[], int]
    # Injected body: called once per tick. May raise; caller catches.
    tick_body: Callable[[], None]
    # Injected fault sink: called with a str on any exception in tick_body.
    on_fault: Callable[[str], None]
    # Injected transition to BLOCKED when N consecutive over-budget ticks.
    on_blocked: Callable[[], None]

    # -- private counters
    _tick_no: int = 0
    _consecutive_over_budget: int = 0
    _blocked: bool = False
    _history: List[TickReport] = field(default_factory=list)

    def run_one_tick(self) -> TickReport:
        """Execute one tick body under the budget guard.

        Returns a TickReport for observability. On over-budget or
        exception, records the outcome; on N consecutive over-budget
        ticks, calls on_blocked (idempotent -- only once)."""
        self._tick_no += 1
        t0 = self.clock_ms()
        faulted = False
        fault_msg: Optional[str] = None
        try:
            self.tick_body()
        except Exception as exc:
            # CLAUDE.md 4.4: raise inside loop = zero-speed this tick +
            # fault event + next tick keeps running. NEVER let it
            # bubble past this method.
            faulted = True
            fault_msg = "%s: %s" % (type(exc).__name__, exc)
            try:
                self.on_fault(fault_msg)
            except Exception:
                # fault sink itself failed; log path is best-effort.
                pass
        t1 = self.clock_ms()
        duration_ms = float(t1 - t0)
        over_budget = duration_ms > MAIN_TICK_BUDGET_MS

        if over_budget:
            self._consecutive_over_budget += 1
            if (self._consecutive_over_budget
                    >= BLOCKED_AFTER_OVER_BUDGET_TICKS
                    and not self._blocked):
                self._blocked = True
                try:
                    self.on_blocked()
                except Exception:
                    pass
        else:
            self._consecutive_over_budget = 0

        report = TickReport(
            tick_no=self._tick_no,
            duration_ms=duration_ms,
            over_budget=over_budget,
            faulted=faulted,
            fault=fault_msg,
        )
        self._history.append(report)
        return report

    @property
    def blocked(self) -> bool:
        return self._blocked

    @property
    def history(self) -> List[TickReport]:
        return list(self._history)

    def reset_consecutive_over_budget(self) -> None:
        """After BLOCKED recovery, callers reset the counter so a
        fresh over-budget run does not immediately re-trip."""
        self._consecutive_over_budget = 0
