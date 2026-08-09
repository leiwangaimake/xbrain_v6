"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: state_machines.py
Brief: GWY-P4-18 -- session slots + L2 confirm SM + L3 approval SM + Recording SM

Description:
16 S11 session state machines. Three co-existing SMs per session:

  L2 confirm     (operator button acknowledge; short-lived 10 s)
  L3 approval    (cloud confirm_token; long-lived 60 s)
  RecordingDialog (path/fence recording session; multi-turn)

Chitchat whitelist: within a session, only registered chitchat
intents can interrupt an active L2/L3 wait without cancelling it
(so a operator saying "还有多久" mid-L2 doesn't kill the confirm).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import FrozenSet, List, Optional


class L2ConfirmState(str, Enum):
    IDLE = "idle"
    WAITING = "waiting"
    CONFIRMED = "confirmed"
    TIMED_OUT = "timed_out"


class L3ApprovalState(str, Enum):
    IDLE = "idle"
    PENDING_TOKEN = "pending_token"
    APPROVED = "approved"
    STALE = "stale"


class RecordingState(str, Enum):
    IDLE = "idle"
    RECORDING = "recording"
    FINISHED = "finished"


@dataclass
class L2Slot:
    """One outstanding L2 confirm request."""
    state: L2ConfirmState = L2ConfirmState.IDLE
    started_mono_ms: int = 0
    timeout_ms: int = 10_000        # 10 s per 16 S11

    def request(self, now_mono_ms: int) -> None:
        self.state = L2ConfirmState.WAITING
        self.started_mono_ms = now_mono_ms

    def tick(self, now_mono_ms: int) -> None:
        """Advance the SM based on elapsed time."""
        if self.state == L2ConfirmState.WAITING:
            if now_mono_ms - self.started_mono_ms > self.timeout_ms:
                self.state = L2ConfirmState.TIMED_OUT

    def confirm(self) -> None:
        if self.state != L2ConfirmState.WAITING:
            return
        self.state = L2ConfirmState.CONFIRMED


@dataclass
class L3Slot:
    """One outstanding L3 approval (cloud token)."""
    state: L3ApprovalState = L3ApprovalState.IDLE
    started_mono_ms: int = 0
    timeout_ms: int = 60_000

    def request(self, now_mono_ms: int) -> None:
        self.state = L3ApprovalState.PENDING_TOKEN
        self.started_mono_ms = now_mono_ms

    def tick(self, now_mono_ms: int) -> None:
        if self.state == L3ApprovalState.PENDING_TOKEN:
            if now_mono_ms - self.started_mono_ms > self.timeout_ms:
                self.state = L3ApprovalState.STALE

    def approve(self) -> None:
        if self.state != L3ApprovalState.PENDING_TOKEN:
            return
        self.state = L3ApprovalState.APPROVED


@dataclass
class RecordingSlot:
    state: RecordingState = RecordingState.IDLE
    waypoints: List = field(default_factory=list)

    def start(self) -> None:
        self.state = RecordingState.RECORDING
        self.waypoints.clear()

    def add_waypoint(self, wp) -> None:
        if self.state == RecordingState.RECORDING:
            self.waypoints.append(wp)

    def finish(self) -> None:
        if self.state == RecordingState.RECORDING:
            self.state = RecordingState.FINISHED


# Chitchat white list per 16 S11. Interrupting an L2/L3 wait with
# one of these does NOT cancel the wait.
CHITCHAT_WHITELIST: FrozenSet[str] = frozenset({
    "how_long_left", "current_status", "confirm_again",
})


def is_chitchat_interrupt(intent_name: str) -> bool:
    return intent_name in CHITCHAT_WHITELIST
