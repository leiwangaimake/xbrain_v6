"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: hold_ms_renewal.py
Brief: CHK-1-33 PTZ long-pulse dead-man (hold_ms) renewal (PTZ-D1)

Description:
PTZ-D1: E10 pulse_ms can well exceed the E01 2800 ms upper bound.
A naive dead-man that just uses a fixed 800 ms hold cuts a
"turn 90 degrees" motion after a few hundred ms; a naive
"just-received-a-nudge -> hold forever" opens an unbounded
pan (uncontrolled path).

The right pattern (PTZ-D1):
  * dead-man interval is HOLD_MS (config, no default).
  * each fresh nudge REFRESHES the deadline: deadline = now + hold_ms
  * if now >= deadline AND no fresh nudge arrived -> emit exactly
    ONE Stop and stop rearming until the next nudge
  * pulse_ms parameter (from the higher-level intent) does NOT
    become hold_ms directly -- the two are orthogonal (pulse_ms
    tells P4's requester how long they intend to move; hold_ms
    tells the deadman how much silence to tolerate)

Time is measured with the monotonic clock ONLY (CLAUDE.md 3.4).
"""

from __future__ import annotations

from dataclasses import dataclass


class HoldMsConfigError(Exception):
    """Zero/negative hold_ms is fail-silent (no dead-man = uncontrolled)."""


@dataclass
class DeadmanState:
    """Tracks whether the pan is currently ARMED (has a live deadline)
    and how many Stops have been emitted since arming."""
    hold_ms: int
    deadline_mono_ms: int = 0
    armed: bool = False
    stops_emitted: int = 0


def create(hold_ms: int) -> DeadmanState:
    """Injected hold_ms; zero / negative refused (CLAUDE.md 3.1)."""
    if hold_ms <= 0:
        raise HoldMsConfigError(
            "hold_ms must be > 0, got %r (fail-silent form: no dead-man)"
            % hold_ms)
    return DeadmanState(hold_ms=hold_ms)


def on_nudge(state: DeadmanState, now_mono_ms: int) -> None:
    """Fresh nudge arrived from cmd/ptz: refresh deadline."""
    state.deadline_mono_ms = now_mono_ms + state.hold_ms
    state.armed = True


def tick(state: DeadmanState, now_mono_ms: int) -> bool:
    """Called once per control tick. Returns True if a Stop must
    be emitted NOW (deadline reached AND armed).

    Emits EXACTLY ONCE per silence run: after Stop, state.armed
    goes False; the next fresh nudge re-arms it. This prevents
    a stuck-idle deadman from continuously spamming Stop calls."""
    if not state.armed:
        return False
    if now_mono_ms < state.deadline_mono_ms:
        return False
    # Deadline reached; fire once.
    state.armed = False
    state.stops_emitted += 1
    return True
