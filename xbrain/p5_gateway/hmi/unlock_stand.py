"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: unlock_stand.py
Brief: HMI chassis unlock -> stand once the read-back shows the lock cleared (99 U181)

Description:
The user's ruling of 2026-11-13: when the HMI chassis-lock button turns green
after an unlock, the robot must also stand, so it can move at once. Before
this an operator unlocked, then had to send a separate "stand" before any
motion worked, and that tripped up several field tests.

What "unlock succeeded" means here is the same thing that turns the button
green: the state/robot READ-BACK with hes_lock and timeout_lock both clear
(data_readers.chassis_lock_group, 11 S9.12.2, BOOT-L3). An accepted W9 ack is
NOT success -- quadruped accepts the frame and still keeps hes_lock while the
hardware stop is pressed (11 S9.3.4 E-2). So:

  arm(cmd_id, now)     the W9 enable went out (runtime/main_wiring _send_uplink)
  on_ack(cmd_id, res)  a rejected / error answer to THAT enable disarms it
  on_robot(robot, now) the read-back: unlocked within the window -> return the
                       enable's cmd_id ONCE (the caller publishes
                       cmd/chassis/ctrl{stand, origin hmi}); still locked when
                       the window ends -> disarm, nothing is sent

The stand is skipped when the read-back already says the robot stands
(motion_state "stand", or "rl_control", which is where a stand settles: 13
S6.2 v1.3 measured the firmware switching itself to RL control in the same
millisecond). Commanding stand on a robot already in RL control could drop it
out of the state autonomous motion needs, for nothing.

What it does NOT do: it never unlocks, never retries a failed stand, and acts
only on unlocks the HMI itself sent -- a commx / cloud unlock is a different
channel with its own operator, and the user asked for the HMI button. The
window is required config (hmi.unlock_stand_window_ms) with no code default;
an unlock that clears after the window gets no stand, which is the safe side
(the operator stands it by hand, as before).

Thread note: arm runs on the web-server thread, on_robot on a Zenoh (Rust)
callback thread, so all state sits behind one lock.
"""
from __future__ import annotations  # postponed annotations

import threading  # arm and on_robot run on different threads
from typing import Any, Dict, Optional  # type hints

from xbrain.p5_gateway.hmi.data_readers import chassis_lock_group  # the button's own read-back rule

#: Read-back motion states in which the robot already stands (see Description).
STANDING_STATES = frozenset({"stand", "rl_control"})

#: W9 ack results that mean the enable was not taken (11 S7.7 result closed set).
FAILED_RESULTS = frozenset({"rejected", "error"})


class StandAfterUnlock:
    """One pending HMI unlock at a time; a newer unlock replaces the older one."""

    def __init__(self, window_ms: Any) -> None:
        # Required, positive, no default (CLAUDE.md 3.1 discipline): a missing
        # window would either never stand or stand on an unlock long after it.
        # bool is an int in Python, so it is refused by name: True would read as 1 ms.
        if isinstance(window_ms, bool) or not isinstance(window_ms, (int, float)) or window_ms <= 0:
            raise ValueError("hmi.unlock_stand_window_ms must be a positive number, got %r"
                             % (window_ms,))  # the key path, so the operator knows what to fix
        self._window_s = float(window_ms) / 1000.0  # config is ms, the clock is monotonic seconds
        self._lock = threading.Lock()  # web-server thread vs Zenoh callback thread
        self._cmd_id: Optional[str] = None   # the armed enable's cmd_id
        self._deadline = 0.0                 # monotonic seconds

    @property
    def armed(self) -> Optional[str]:
        """The cmd_id waiting for its read-back, or None."""
        with self._lock:  # read under the lock: on_robot may be clearing it
            return self._cmd_id  # None once fired, expired or refused

    def arm(self, cmd_id: str, now_mono_s: float) -> None:
        """The HMI enable went out. The window starts at the publish, not at
        the ack: the read-back can arrive before the ack does."""
        with self._lock:  # both fields change together
            self._cmd_id = cmd_id  # a second click replaces the first: only the latest stands
            self._deadline = now_mono_s + self._window_s  # monotonic (CLK-C1), never wall clock

    def on_ack(self, cmd_id: Any, result: Any) -> None:
        """A refused enable will not clear the lock; disarm so a later unlock
        from another channel inside the window does not make THIS one stand."""
        with self._lock:  # same lock as on_robot: an ack and a frame can race
            # accepted / duplicate keep waiting: accepted is not unlocked (BOOT-L3)
            if cmd_id == self._cmd_id and result in FAILED_RESULTS:
                self._cmd_id = None  # refused or failed: this unlock will never clear

    def on_robot(self, robot: Optional[Dict[str, Any]], now_mono_s: float) -> Optional[str]:
        """The cmd_id to stand for, at most once per arm; None otherwise."""
        with self._lock:  # one frame decides at most once, even if two race
            if self._cmd_id is None:
                return None  # nothing armed: the common case on every state/robot frame
            if now_mono_s > self._deadline:
                self._cmd_id = None          # never cleared in time: no stand
                return None
            cl = chassis_lock_group(robot)
            if not cl["available"] or cl["locked"] is not False:
                return None                  # still locked or unknown: keep waiting
            cmd_id, self._cmd_id = self._cmd_id, None   # once, whatever follows
            if (robot or {}).get("motion_state") in STANDING_STATES:
                return None                  # already standing: nothing to do
            return cmd_id
