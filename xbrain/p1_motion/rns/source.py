"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: source.py
Brief: RNS behavior-source shell (20 S1.1 RNS-M-3/M-7 / 12 S4.2c.1)

Description:
The ONLY outward face of the RNS module (RNS-M-3): the behavior-source interface
(is_active / compute / on_preempted / on_input_lost) that 12 S4.1 arbitrates.
Everything else in rns/ is reached only through here; no other module reads the
RNS grid or corridor (that would be a second d_free truth source, D-29).

P0.4 status: SHELL. is_active() returns False unconditionally and compute()
returns None, so wiring this into p1's arbiter does not change robot behavior
(RNS_TODO P0.4: "is_active recanted false, no smoke"). The tick body lands
across P1..P6; each phase flips on its slice.

Lifecycle contract this shell owns (kept here from day one so later phases fill
in, not restructure):
  - RNS-M-7 estop/preempt: on_preempted -> SUSPENDED, mission KEPT; release ->
    re-evaluate from FOLLOW, NEVER replay the frozen pre-suspend intent (the
    "release == recover last velocity" bug is what A-ES-2 kills).
  - 12 S4.2c.1 origin: mission carries origin (route|relmove); on failure the
    report channel is chosen by origin, not by mission category. source.py is
    where that choice is made (route -> path_progress.state; relmove -> status).
  - RNS-M-3: compute returns a VelocityCandidate OR None; None is the normal
    "no output this tick" value, distinct from a zero candidate.
"""

from __future__ import annotations

from typing import Optional

from .types import MissionKind, NavState, Origin, VelocityCandidate


class RnsSource:
    """P1 behavior source `rns_avoid` (12 S4.2 priority 900). Process-local, no
    Zenoh session, no thread (RNS-M-1/M-4/M-5). Constructed once; ticked from the
    p1 ctrl loop with the tick snapshot."""

    def __init__(self) -> None:
        # SHELL: no mission, idle. Real construction (grid, config, submodules)
        # lands in P0.5/P1 -- kept minimal so an unwired instance is inert.
        self._state: NavState = NavState.IDLE
        self._mission_kind: Optional[MissionKind] = None
        self._origin: Optional[Origin] = None

    def is_active(self, ctx) -> bool:
        """12 S4.1: active iff a navigation mission is in flight and not
        SUSPENDED (20 S9.0.1). SHELL: no mission is ever loaded yet, so this is
        unconditionally False -- wiring this source in changes nothing."""
        return False

    def compute(self, ctx) -> Optional[VelocityCandidate]:
        """RNS-M-3: candidate or None. SHELL: always None (no output). The tick
        body is filled in P1..P6."""
        return None

    def on_preempted(self, ctx) -> None:
        """RNS-M-7: teleop/estop took the slot. Suspend, KEEP mission. SHELL:
        record the state transition only; SUSPENDED is non-terminal."""
        self._state = NavState.SUSPENDED

    def on_input_lost(self, ctx) -> None:
        """A required input (12 abort_reason input_lost family) went stale.
        SHELL: no-op; the real handling (which reason, which carrier) lands with
        inputs.py (P2) and the failure path (P6)."""
        return None
