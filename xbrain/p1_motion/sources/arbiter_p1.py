"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: arbiter_p1.py
Brief: MOT-PM-5 P1-internal behavior source arbiter (12 S4.2 v0.8)

Description:
The sources (fence_guard / teleop_* / rns_avoid / nav2_proxy / teleop_cloud / relative_move + hold) compete for the P1 output slot; highest active priority wins. Sources not seen for dwell_ms deactivate; a fresh note() re-activates. Separate from xbrain/common/arbiter -- P1 sources are process-local with no cross-process story. PM1.3 (2026-09-09) removed path_follow/target_oriented/estop_echo and moved rns_avoid to 900 (12 v0.8, #20-1/#20-9).
"""



from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional


class BehaviorSource(str, Enum):
    """Behavior source closed set (12 S4.2 v0.8).

    PM1.3 (2026-09-09, #20-1/#20-9): RNS became the system's single navigation
    source (RNS-M-6). Three sources were REMOVED here to match 12 v0.8:
      - path_follow: polyline following moved into RNS (route.py); its route
        pointer consume (12 S3.5A) is RNS's now.
      - target_oriented: folded into RNS follow_target (reserved #20-13).
      - estop_echo: estop is NOT a behavior source in 12 v0.8 -- it is cmd/estop
        zeroing the output directly (P1-21). Removing it also clears the 900
        collision it had with rns_avoid's new priority (user ruling 2026-09-09).
    rns_avoid moved 700 -> 900: it is navigation itself now, not an avoidance
    overlay. Its relation to teleop is TR-RNS-1 (12 S4.2c.5), not the number."""
    FENCE_GUARD = "fence_guard"           # 1000 - veto only, produces no motion
    TELEOP_KEYBOARD = "teleop_keyboard"   # 800  - local (teleop family)
    TELEOP_JOYSTICK = "teleop_joystick"   # 750
    RNS_AVOID = "rns_avoid"               # 900  - the single navigation source
    NAV2_PROXY = "nav2_proxy"             # 600  - Nav2 delegate (spin/backup/wait)
    TELEOP_CLOUD = "teleop_cloud"         # 550  - cloud tel-op
    RELATIVE_MOVE = "relative_move"       # 500  - shell only (translates to goto)
    HOLD = "hold"                         # 100  - always-alive zero-vel


# 12 S4.2 v0.8 priorities. rns_avoid is 900 (below fence_guard 1000 only). The
# teleop_keyboard/joystick 800/750 are the local-teleop family; TR-RNS-1 governs
# whether teleop preempts RNS (12 S4.2c.5), not the raw number comparison.
_PRIORITY = {
    BehaviorSource.FENCE_GUARD: 1000,
    BehaviorSource.RNS_AVOID: 900,
    BehaviorSource.TELEOP_KEYBOARD: 800,
    BehaviorSource.TELEOP_JOYSTICK: 750,
    BehaviorSource.NAV2_PROXY: 600,
    BehaviorSource.TELEOP_CLOUD: 550,
    BehaviorSource.RELATIVE_MOVE: 500,
    BehaviorSource.HOLD: 100,
}


def priority_of(source: BehaviorSource) -> int:
    return _PRIORITY[source]


@dataclass
class SourceState:
    active: bool = False
    last_hit_mono_ms: int = 0


class P1Arbiter:
    """Highest-priority active source wins. Deactivation hysteresis:
    a source not seen for `dwell_ms` transitions active->inactive
    (12 S4 失活迟滞)."""

    def __init__(self, dwell_ms: int = 200) -> None:
        self._dwell_ms = dwell_ms
        self._states: Dict[BehaviorSource, SourceState] = {
            s: SourceState() for s in BehaviorSource
        }

    def note(self, source: BehaviorSource, now_mono_ms: int) -> None:
        st = self._states[source]
        st.active = True
        st.last_hit_mono_ms = now_mono_ms

    def tick(self, now_mono_ms: int) -> None:
        for s, st in self._states.items():
            if st.active and (now_mono_ms - st.last_hit_mono_ms) > self._dwell_ms:
                st.active = False

    def snapshot(self) -> Dict[BehaviorSource, SourceState]:
        """A copy of every source's (active, last_hit) for the 11 S7A.5.1
        state/arb/motion body (sources[].alive). Copies, so a reader cannot
        mutate the arbiter's own records."""
        return {s: SourceState(st.active, st.last_hit_mono_ms)
                for s, st in self._states.items()}

    def holder(self) -> Optional[BehaviorSource]:
        best: Optional[BehaviorSource] = None
        best_pri = -1
        for s, st in self._states.items():
            if st.active and priority_of(s) > best_pri:
                best = s
                best_pri = priority_of(s)
        return best
