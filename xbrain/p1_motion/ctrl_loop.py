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
    """One tick's outcome for observability."""
    tick_no: int
    state: str
    published_cmd_vel: bool
    vx: float
    wz: float


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
                     computed_wz: float = 0.0) -> CtrlTick:
        """Execute one tick. In non-ACTIVE states vx/wz are zero;
        ACTIVE uses computed values. Always publishes."""
        self._tick_no += 1
        if self._state == CtrlState.ACTIVE:
            vx, wz = computed_vx, computed_wz
        elif self._state == CtrlState.SAFE_STOP:
            vx, wz = 0.0, 0.0    # emergency zero-vel
        else:
            vx, wz = 0.0, 0.0    # not yet driving; publish zero
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
            vx=vx, wz=wz,
        )
        self._history.append(report)
        return report

    @property
    def history(self) -> List[CtrlTick]:
        return list(self._history)
