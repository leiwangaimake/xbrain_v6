"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: switch_sm.py
Brief: CHK-1-12 §9.6.4 profile switch SM (S-1 downshift, S-2 five-condition upshift, S-3 PROFILE_LOCKED)

Description:
12 §9.6.4 speed profile state machine. Three transitions:

  S-1  DOWNSHIFT   any downshift condition fires -> profile drops
                    one step (patrol -> creep, creep -> stop)
  S-2  UPSHIFT     ALL FIVE conditions satisfied AND held for
                    T_up_s -> profile rises one step
  S-3  PROFILE_LOCKED  a repeat 'thrash' pattern (>= N downshifts
                        within window_s) locks the profile at the
                        lowest observed rung until ONE of THREE
                        explicit unlocks fires:
                          U-1  operator reset_profile_lock command
                          U-2  reboot
                          U-3  configured 'auto_unlock_after_s' elapsed
                                (may be disabled by setting to 0)

The five upshift conditions:
  C1  d_free >= d_up
  C2  no route replan in the last replan_free_s
  C3  no arbiter loss-of-holder in last T_arb_s
  C4  clock health OK
  C5  no active teleop override
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


PROFILES = ("stop", "creep", "patrol")


class InvalidProfileTransition(Exception):
    pass


@dataclass
class ProfileSwitchSM:
    current: str = "patrol"
    locked: bool = False
    lock_at_rung: str = ""
    downshifts_recent_ms: List[int] = field(default_factory=list)
    thrash_window_ms: int = 10000
    thrash_count_threshold: int = 3

    def downshift(self, now_ms: int) -> None:
        """S-1: drop one rung. Feed the thrash tracker."""
        ix = PROFILES.index(self.current)
        if ix > 0:
            self.current = PROFILES[ix - 1]
        self.downshifts_recent_ms.append(now_ms)
        # Prune older than window.
        cutoff = now_ms - self.thrash_window_ms
        self.downshifts_recent_ms = [
            t for t in self.downshifts_recent_ms if t >= cutoff]
        if len(self.downshifts_recent_ms) >= self.thrash_count_threshold:
            self.locked = True
            if not self.lock_at_rung:
                self.lock_at_rung = self.current

    def upshift(self, all_five_conditions: bool) -> None:
        """S-2: rise one rung only if all five conditions AND not locked."""
        if self.locked:
            return
        if not all_five_conditions:
            return
        ix = PROFILES.index(self.current)
        if ix < len(PROFILES) - 1:
            self.current = PROFILES[ix + 1]

    def unlock(self, source: str) -> None:
        """S-3 U-1 / U-2 / U-3 -- the three legal unlock paths.
        source must be one of these strings; any other value refused
        (per CLAUDE.md 3.6 no-toggle discipline).
        Note: reboot is expressed as 'reboot' -- not a runtime call,
        but for symmetry with the closed set."""
        if source not in ("operator_reset", "reboot", "auto_unlock"):
            raise InvalidProfileTransition(
                f"unlock source {source!r} not in closed set")
        self.locked = False
        self.lock_at_rung = ""
        self.downshifts_recent_ms.clear()


def all_five_upshift_conditions(d_free_m: float, d_up_m: float,
                                  replan_free: bool,
                                  arb_holder_stable: bool,
                                  clock_health_ok: bool,
                                  no_teleop_override: bool) -> bool:
    """C1..C5 as one predicate. Any single false -> block upshift."""
    return (d_free_m >= d_up_m
            and replan_free
            and arb_holder_stable
            and clock_health_ok
            and no_teleop_override)
