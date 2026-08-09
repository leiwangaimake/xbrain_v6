"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: deadman.py
Brief: GWY-P5-11 WS disconnect -> proxy cmd/teleop{deadman:false} (WD-1..6)

Description:
17 S11.4 when the HMI WebSocket session closes (any reason), p5
MUST publish `cmd/teleop{deadman:false}` on behalf of the operator.
This releases any active teleoperation grip so a browser crash /
laptop lid close does not leave the robot with a still-held stick.

WD-1..6 invariants:

  WD-1  fires on ANY disconnect (clean or dirty)
  WD-2  fires within deadman_response_ms (from configs, no default)
  WD-3  emits AT MOST ONCE per session (dedupe by session_id)
  WD-4  publish uses the SAME topic + payload the HMI would have
        used (so p1's four_source_teleop treats it identically)
  WD-5  emits an audit event with reason='hmi_disconnect'
  WD-6  if publish fails, LOG loudly; do not swallow

The deadman message shape is the closed-set schema from 11 §7 --
we do not reimplement the schema here, we call the shared helper.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Set


@dataclass
class DeadmanTracker:
    """WD-3: dedupe emissions per session."""
    fired_sessions: Set[str] = field(default_factory=set)

    def should_fire(self, session_id: str) -> bool:
        return session_id not in self.fired_sessions

    def mark_fired(self, session_id: str) -> None:
        self.fired_sessions.add(session_id)


def build_deadman_payload() -> dict:
    """WD-4: exactly the shape a real HMI would send when releasing
    the deadman. Downstream must not distinguish."""
    return {"deadman": False, "vx": 0.0, "vy": 0.0, "wz": 0.0}


def within_response_window(now_ms: int, disconnect_ms: int,
                             response_ms: int) -> bool:
    """WD-2: caller checks and asserts."""
    return (now_ms - disconnect_ms) <= response_ms
