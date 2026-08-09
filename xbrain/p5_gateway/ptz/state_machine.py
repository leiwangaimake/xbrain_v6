"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: state_machine.py
Brief: GWY-P5-22 ptz_input independent process (HOLD_FIRST / HOLD_REPEAT / REPEAT_ARMED + 5 Hz)

Description:
17 S19 ptz_input is a separate process from p5. Directly reads evdev
so we don't pay Zenoh round-trip on operator input, but publishes
its state to a topic p5 consumes.

State machine (per axis: pan, tilt, zoom):

  IDLE         no input
  HOLD_FIRST   first frame the button is down; publish one 'step'
  HOLD_REPEAT  button still down after hold_first_delay_ms; publish
               repeat lease renewals at 5 Hz
  REPEAT_ARMED transient state between HOLD_FIRST and HOLD_REPEAT
               so the delay is not accidentally shortened by fast
               polls

Lease renewal at 5 Hz FIXED clock -- not tied to key repeat rate.
This ensures the receiving payload can time out cleanly even if
the OS keyboard-repeat rate varies wildly.
"""

from __future__ import annotations

from enum import Enum


class PtzButtonState(str, Enum):
    IDLE = "idle"
    HOLD_FIRST = "hold_first"
    REPEAT_ARMED = "repeat_armed"
    HOLD_REPEAT = "hold_repeat"


ALLOWED: dict = {
    PtzButtonState.IDLE:         {PtzButtonState.HOLD_FIRST},
    PtzButtonState.HOLD_FIRST:   {PtzButtonState.REPEAT_ARMED,
                                    PtzButtonState.IDLE},
    PtzButtonState.REPEAT_ARMED: {PtzButtonState.HOLD_REPEAT,
                                    PtzButtonState.IDLE},
    PtzButtonState.HOLD_REPEAT:  {PtzButtonState.IDLE},
}


LEASE_RENEWAL_HZ = 5


class InvalidPtzTransition(Exception):
    pass


def transition(from_state: PtzButtonState,
                 to_state: PtzButtonState) -> PtzButtonState:
    if to_state not in ALLOWED[from_state]:
        raise InvalidPtzTransition(
            f"{from_state.value!r} -> {to_state.value!r}")
    return to_state


def lease_renewal_period_ms() -> int:
    """5 Hz fixed -> 200 ms period."""
    return 1000 // LEASE_RENEWAL_HZ
