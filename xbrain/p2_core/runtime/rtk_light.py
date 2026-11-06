"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rtk_light.py
Brief: 11 S3.3.4 RL-3 light half -- red/blue warning on a positioning loss, off on recovery

Description:
The problem. RL-3 asks for a visible alarm when positioning is lost, so the
operator walking up to a robot that stopped in the field can tell "it lost
its fix" from "it is waiting". Until 2026-11-06 nothing lit anything: the
loss itself was never published. P3 now publishes event/alarm/rtk and
event/info/rtk (99 U142); this module turns those into the payload light
command P2 sends (POST /lights through ModeDeviceSync.set_lights).

Which light. The red/blue double flash, at the same pattern number D / B
mode use (arbiter.domains.payload_light.deter_redblue_mode). NOT the
searchlight strobe: 00 S5.1.1 keeps the fill / search lights steady for
illumination and gives warning flashes to red/blue (CLAUDE.md 10 trap
table); RL-3's "载荷爆闪" is read through that rule.

Boundaries:
  * B / D mode owns the red/blue light while it is on (14 S4.3). Nothing is
    sent then: fighting the mode for the lamp would flicker it, and the mode
    is already flashing. P2's mode exit consults rtk_alarm_on() so leaving
    B / D during a loss keeps the light on instead of switching it off.
  * The chassis light (11 S9.4.1 blink / white) has no publisher anywhere in
    this build; it is NOT faked here (CLAUDE.md 3.2). The payload light is
    the part RL-3 can have today.
  * No clock, no I/O: the caller sends the body.
"""
from __future__ import annotations  # postponed annotations

from typing import Any, Dict, Optional  # plain value types

#: detail.type values (11 S3.3.4 closed set) that switch the light.
LOST = "rtk_lost"  # light on
RECOVERED = "heading_recovered"  # light off


class RtkLight:
    """Remembers whether a loss is in progress and builds the light body."""

    def __init__(self, redblue_mode: int) -> None:
        self._mode = int(redblue_mode)  # deter_redblue_mode, the D / B pattern
        self.on = False  # a loss is in progress (survives a B / D episode)

    def on_event(self, detail: Any, mode_owns_light: bool) -> Optional[Dict[str, Any]]:
        """The /lights body for one rtk event detail, or None (nothing to send).
        mode_owns_light: B or D mode is on (it drives red/blue itself)."""
        kind = detail.get("type") if isinstance(detail, dict) else None  # which event
        if kind == LOST:
            self.on = True  # remembered even when the mode owns the lamp now
        elif kind == RECOVERED:
            self.on = False  # loss over
        else:
            return None  # heading_degraded / malformed: no light change
        if mode_owns_light:
            return None  # B / D flashes already; do not fight it
        return {"redblue": self._mode if self.on else 0}  # partial body (U39)

    def after_mode_exit(self) -> Dict[str, Any]:
        """The red/blue level to leave behind when B / D ends: still flashing
        while a loss lasts, off otherwise (was always off before 99 U142)."""
        return {"redblue": self._mode if self.on else 0}  # partial body
