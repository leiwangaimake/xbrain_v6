"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: level_routing.py
Brief: GWY-P4-17 -- CL-1/CL-2/CL-3 + LB-1/LB-2 level routing

Description:
16 S8.3A level routing between L1a and L1b. The two levels differ
in timing:
  L1a  restate BEFORE executing (operator hears what will happen,
       has a chance to cancel by not confirming)
  L1b  execute first, restate AFTER (operator hears what just
       happened; no cancel window)

CL-1: intent's DEFAULT level is from cmdset_18.json auth field
CL-2: session can UPGRADE L1a -> L1b (operator asked "just do it,
      don't confirm") but NEVER downgrade L1b -> L1a
CL-3: routing decision is made at intent-classify time, then FROZEN
      for the rest of the turn; no re-routing based on failure

LB-1: L1a turn timeline: RESTATE -> WAIT(gate) -> if confirmed, EXECUTE
LB-2: L1b turn timeline: EXECUTE -> RESTATE (parallel)
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Level(str, Enum):
    L1a = "L1a"
    L1b = "L1b"


@dataclass
class SessionUpgrade:
    """Session-level L1a -> L1b upgrade preference."""
    upgraded_to_l1b: bool = False


class LevelRoutingError(RuntimeError):
    """Violation of CL-2 (attempted downgrade) or CL-3 (re-route
    mid-turn)."""


def resolve_level(default_level: Level,
                  session: SessionUpgrade) -> Level:
    """CL-1/2: return the effective level for this turn.

    Upgrade path: default L1a + session upgraded -> L1b.
    Default L1b + session upgraded -> stays L1b.
    """
    if default_level == Level.L1a and session.upgraded_to_l1b:
        return Level.L1b
    return default_level


def upgrade_to_l1b(session: SessionUpgrade) -> None:
    """Set the session-level L1a -> L1b upgrade."""
    session.upgraded_to_l1b = True


def try_downgrade_to_l1a(session: SessionUpgrade) -> None:
    """CL-2: downgrade is forbidden."""
    if session.upgraded_to_l1b:
        raise LevelRoutingError(
            "CL-2 violation: cannot downgrade session L1b -> L1a; "
            "'don't confirm' is a monotonic preference within a session")
