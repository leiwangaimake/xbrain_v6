"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ctrl_loop.py
Brief: MOT-PM-1 P1 20 Hz ctrl loop 10-step timing + lifecycle SM

Description:
The P1 main loop runs at 20 Hz. Each tick executes 10 deterministic
steps (freshness -> arbiter tick -> gate -> rotation -> fence -> ...)
in a fixed order. The lifecycle SM has 6 states:
  INIT / WAIT_INPUT / WAIT_GRANT / READY / ACTIVE / SAFE_STOP

Every state MUST emit cmd_vel each tick (zero-vel where not moving)
so chassis_relay never sees a gap. A missed tick becomes cmd_age
> 200 ms at chassis Tier 1 and triggers timeout_lock; the loop's
first job is 'always publish something'.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, List, Optional


class CtrlState(str, Enum):
    INIT = "init"
    WAIT_INPUT = "wait_input"
    WAIT_GRANT = "wait_grant"
    READY = "ready"
    ACTIVE = "active"
    SAFE_STOP = "safe_stop"


@dataclass
class CtrlTick:
    """One tick's outcome for observability.

    stop_reason is 本拍零速的唯一归因 (11 S4.1). It is the closed-set value
    (STOP_REASON) explaining WHY this tick is zero, or "none" when driving.
    P1-21 requires it on every soft-estop tick; without it a zeroed tick and
    a not-yet-driving tick look identical downstream.
    """
    tick_no: int
    state: str
    published_cmd_vel: bool
    vx: float
    wz: float
    stop_reason: str = "none"


class CtrlLoop:
    """Deterministic 10-step per-tick executor.

    Fixed step order (per 12 S2.2):
      1. freshness classify
      2. arbiter tick (deactivation)
      3. holder read
      4. gate compute (f + g + h + i)
      5. rotation permit check
      6. fence project
      7. limiter argmax
      8. safety veto
      9. cmd_vel assemble
      10. publish (single sink)

    Every state emits cmd_vel EVERY TICK (chassis Tier 1 requires it).
    """

    def __init__(self, publish_cmd_vel: Callable[[float, float], None]) -> None:
        self._state = CtrlState.INIT
        self._publish = publish_cmd_vel
        self._tick_no = 0
        self._history: List[CtrlTick] = []

    @property
    def state(self) -> str:
        return self._state.value

    def transition(self, to: CtrlState) -> None:
        """Move to `to` at the START of a tick. No mid-tick transitions."""
        self._state = to

    def run_one_tick(self,
                     computed_vx: float = 0.0,
                     computed_wz: float = 0.0,
                     estop: bool = False) -> CtrlTick:
        """Execute one tick. In non-ACTIVE states vx/wz are zero;
        ACTIVE uses computed values. Always publishes.

        *** estop wins over everything (11 S9.12.2, common/enums 逐字
        "estop wins over everything"). A soft-estop tick is zero-vel
        REGARDLESS of state -- even ACTIVE with a nonzero computed_vx --
        and its stop_reason is "soft_estop". This is P1-21's 本拍零速: the
        estop latch set by the cmd/estop callback is read here every tick,
        so a stale ACTIVE state cannot leak one more nonzero cmd_vel through.

        NO the estop check must come FIRST, before the ACTIVE branch. If it
        were folded in after, an ACTIVE tick would compute vx/wz and a later
        estop check would have to remember to overwrite them -- one forgotten
        path and a moving robot ignores the estop. Ordering it first makes
        that impossible.
        """
        self._tick_no += 1
        if estop:
            # 本拍零速, 归因 soft_estop. Highest precedence, checked first.
            vx, wz, reason = 0.0, 0.0, "soft_estop"
        elif self._state == CtrlState.ACTIVE:
            vx, wz, reason = computed_vx, computed_wz, "none"
        elif self._state == CtrlState.SAFE_STOP:
            vx, wz, reason = 0.0, 0.0, "soft_estop"   # emergency zero-vel
        else:
            # not yet driving; publish zero. no_source distinguishes "nothing
            # to drive" from "estop stopped me" -- both zero, different cause.
            vx, wz, reason = 0.0, 0.0, "no_source"
        # ALWAYS publish -- chassis Tier 1 requires it.
        published = True
        try:
            self._publish(vx, wz)
        except Exception:
            # NEVER let a publish failure kill the loop; caller may
            # log the fault, but the tick still counts.
            published = False
        report = CtrlTick(
            tick_no=self._tick_no, state=self._state.value,
            published_cmd_vel=published,
            vx=vx, wz=wz, stop_reason=reason,
        )
        self._history.append(report)
        return report

    @property
    def history(self) -> List[CtrlTick]:
        return list(self._history)
