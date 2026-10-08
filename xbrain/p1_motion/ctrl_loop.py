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

import collections
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Deque, List, Optional

# 12 S8.1 stage 2. Imported at module level, not locally: the
# limiter is part of the tick's fixed step order, not an optional
# extra, and a local import would hide that from a reader of the
# imports block.
from xbrain.p1_motion.gate.accel_limit import (
    AccelLimits,
    limit_axis,
)

#: ticks kept for observability (60 s at 20 Hz). Bounded: the P7.2 loop runs
#: for the process lifetime and an unbounded list grew by 20 records a second.
HISTORY_MAX = 1200


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
    vy: float = 0.0          # lateral (holonomic only); zero on every stop


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

    def __init__(self, publish_cmd_vel: Callable[..., None], *,
                 holonomic: bool = False,
                 accel_limits: Optional[AccelLimits] = None) -> None:
        """publish_cmd_vel(vx, wz) on a non-holonomic chassis; (vx, wz, vy)
        when holonomic=True (P7.2: M20S spec.holonomic is true and the RNS
        candidate carries a body-shield vy). Default False is the conservative
        direction: a caller that never asked for lateral never gets it, and
        any computed_vy it passes is dropped and counted (12 S4.7.3 TS-4).

        accel_limits is 12 S8.1 stage 2 (|v - v_prev| <= a_max*dt,
        acceleration only -- 99 U90 4). None means the stage is NOT applied,
        which is the state on the real machine today: common.spec.
        max_accel_mps2 is null until V-01 calibrates it, and CLAUDE.md S3.1
        forbids defaulting a safety parameter. The caller decides what to do
        about that -- nav_wiring passes limits when the resolved config has
        them and None when it does not.

        NO None is not "limits off because we feel like it": AccelLimits
        refuses to construct on a missing or non-positive a_max, so a caller
        cannot accidentally get a no-op limiter that looks configured. The
        two states are "configured and applied" or "absent and visibly
        absent"; there is no third.
        """
        self._state = CtrlState.INIT
        self._publish = publish_cmd_vel
        self._holonomic = bool(holonomic)
        self.vy_dropped = 0
        self._tick_no = 0
        self._accel = accel_limits
        # Previously PUBLISHED values, which is what stage 2 ramps from.
        # Starting at 0.0 is correct for INIT: the chassis has had no
        # command yet, so the first ACTIVE tick ramps up from rest.
        self._prev_vx = 0.0
        self._prev_vy = 0.0
        # Counter, not a flag: an operator reading "the ramp engaged 4000
        # times in a minute" learns something a boolean cannot say. Public
        # for the same reason vy_dropped is.
        self.accel_clamped = 0
        self._history: Deque[CtrlTick] = collections.deque(maxlen=HISTORY_MAX)

    @property
    def state(self) -> str:
        return self._state.value

    def transition(self, to: CtrlState) -> None:
        """Move to `to` at the START of a tick. No mid-tick transitions."""
        self._state = to

    def run_one_tick(self,
                     computed_vx: float = 0.0,
                     computed_wz: float = 0.0,
                     estop: bool = False,
                     computed_vy: float = 0.0) -> CtrlTick:
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
            vx, vy, wz, reason = 0.0, 0.0, 0.0, "soft_estop"
        elif self._state == CtrlState.ACTIVE:
            vx, vy, wz, reason = computed_vx, computed_vy, computed_wz, "none"
        elif self._state == CtrlState.SAFE_STOP:
            vx, vy, wz, reason = 0.0, 0.0, 0.0, "soft_estop"   # emergency zero-vel
        else:
            # not yet driving; publish zero. no_source distinguishes "nothing
            # to drive" from "estop stopped me" -- both zero, different cause.
            vx, vy, wz, reason = 0.0, 0.0, 0.0, "no_source"
        # -- 12 S8.1 stage 2: |v - v_prev| <= a_max*dt ---------------------
        # Applied ONLY on the ACTIVE branch. estop / SAFE_STOP / no_source
        # are veto semantics and 12 S8.1 is explicit that vetoes bypass the
        # ramp: "若把 S6A 的否决塞进 (2) 做斜坡, 则'拒绝旋转'会变成'先慢慢转
        # 一会儿再停' -- 门就漏了". Those branches reach zero in one tick.
        #
        # NO this check is `reason == "none"` and not `self._state == ACTIVE`:
        # the two are not the same thing. An ACTIVE tick with estop set took
        # the estop branch above, and testing the state here would re-apply
        # the ramp to a zero the estop just produced.
        #
        # *** TWO EQUIVALENT MUTANTS, measured 2026-10-09 and noted here
        # rather than chased with assertions (CLAUDE.md S7.2.1). Both of
        # these survive, and no test can kill them TODAY:
        #
        #   (a) dropping `reason == "none"` so the ramp runs on every branch
        #   (b) keying on `self._state == ACTIVE` instead of on the branch
        #
        # Why they are equivalent: every veto branch above sets the axes to
        # exactly 0.0, and stage 2 does not limit deceleration -- so passing
        # a veto's zero through limit_axis returns that same zero. The guard
        # changes nothing while all vetoes are zero-valued.
        #
        # Why the guard stays anyway, and this is the part that matters: the
        # equivalence is a property of TODAY'S veto set, not of the design.
        # The moment any veto produces a NON-ZERO value -- a "reduce to
        # 0.5 m/s" degradation, say -- (a) and (b) stop being equivalent and
        # start ramping a safety reduction, which is exactly the failure
        # 12 S8.1 names: "若把 S6A 的否决塞进 (2) 做斜坡, 则'拒绝旋转'会变成
        # '先慢慢转一会儿再停' -- 门就漏了". The guard is what makes that
        # future veto correct by construction instead of by luck.
        #
        # So: do NOT remove it on the grounds that no test covers it, and do
        # NOT add a contrived assertion to make it look covered. A new
        # non-zero veto is what makes it testable, and that is the moment to
        # write the case.
        #
        # wz is NOT passed through: common.spec has no angular-acceleration
        # key, so there is no parameter to limit it with (see accel_limit.py).
        if self._accel is not None and reason == "none":
            limited_vx = limit_axis(vx, self._prev_vx, self._accel)
            limited_vy = limit_axis(vy, self._prev_vy, self._accel)
            if limited_vx != vx or limited_vy != vy:
                self.accel_clamped += 1
            vx, vy = limited_vx, limited_vy
        # vy rides the SAME branch as vx/wz above, so estop / SAFE_STOP /
        # not-driving zero it too -- a lateral leak under estop would be the
        # exact failure the estop-first ordering exists to prevent.
        if vy != 0.0 and not self._holonomic:
            vy = 0.0                    # 12 S4.7.3 TS-4: no lateral axis
            self.vy_dropped += 1
        # ALWAYS publish -- chassis Tier 1 requires it.
        published = True
        try:
            if self._holonomic:
                self._publish(vx, wz, vy)
            else:
                self._publish(vx, wz)
        except Exception:
            # NEVER let a publish failure kill the loop; caller may
            # log the fault, but the tick still counts.
            published = False
        # Record what the chassis actually received, for the next tick's
        # ramp. AFTER the vy-drop above, so a non-holonomic chassis ramps
        # from the zero it really got rather than from a vy nobody sent.
        #
        # Updated even when `published` is False: a publish that threw still
        # may have reached the wire, and assuming it did not would let the
        # next tick jump by a full command. The conservative direction here
        # is to believe the chassis saw it.
        self._prev_vx = vx
        self._prev_vy = vy
        report = CtrlTick(
            tick_no=self._tick_no, state=self._state.value,
            published_cmd_vel=published,
            vx=vx, wz=wz, stop_reason=reason, vy=vy,
        )
        self._history.append(report)
        return report

    @property
    def history(self) -> List[CtrlTick]:
        return list(self._history)
