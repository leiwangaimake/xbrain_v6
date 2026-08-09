"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: arbiter_p1.py
Brief: MOT-PM-5 P1-internal 8-tier behavior source arbiter

Description:
The eight sources (fence_guard / estop_echo / teleop_* / rns / nav2_proxy / path_follow / relative_move / target_oriented + hold) compete for the P1 output slot. Highest active priority wins. Sources that haven't published in dwell_ms deactivate; a fresh note() re-activates. This is separate from the arbiter framework in xbrain/common/arbiter -- P1 sources are process-local and have no cross-process arbitration story.
"""



from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional


class BehaviorSource(str, Enum):
    """8-tier closed set, verbatim from 12 S4.1."""
    FENCE_GUARD = "fence_guard"           # 1000 - veto only, produces no motion
    ESTOP_ECHO = "estop_echo"             # 900  - zero-vel echo of estop
    TELEOP_KEYBOARD = "teleop_keyboard"   # 800  - local
    TELEOP_JOYSTICK = "teleop_joystick"   # 750
    RNS_AVOID = "rns_avoid"               # 700  - reactive nav
    NAV2_PROXY = "nav2_proxy"             # 600  - Nav2 delegate
    TELEOP_CLOUD = "teleop_cloud"         # 550  - cloud tel-op
    PATH_FOLLOW = "path_follow"           # 500  - path follower
    RELATIVE_MOVE = "relative_move"       # 450  - one-shot MI-1
    TARGET_ORIENTED = "target_oriented"   # 400  - face target
    HOLD = "hold"                         # 100  - always-alive zero-vel


_PRIORITY = {
    BehaviorSource.FENCE_GUARD: 1000,
    BehaviorSource.ESTOP_ECHO: 900,
    BehaviorSource.TELEOP_KEYBOARD: 800,
    BehaviorSource.TELEOP_JOYSTICK: 750,
    BehaviorSource.RNS_AVOID: 700,
    BehaviorSource.NAV2_PROXY: 600,
    BehaviorSource.TELEOP_CLOUD: 550,
    BehaviorSource.PATH_FOLLOW: 500,
    BehaviorSource.RELATIVE_MOVE: 450,
    BehaviorSource.TARGET_ORIENTED: 400,
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

    def holder(self) -> Optional[BehaviorSource]:
        best: Optional[BehaviorSource] = None
        best_pri = -1
        for s, st in self._states.items():
            if st.active and priority_of(s) > best_pri:
                best = s
                best_pri = priority_of(s)
        return best
