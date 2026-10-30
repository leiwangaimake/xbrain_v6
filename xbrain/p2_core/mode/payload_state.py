"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: payload_state.py
Brief: state/mode.payload (11 S4.3) -- the payload's ACTUAL state, built from the device readback plus what P2 itself holds

Description:
What problem this solves
------------------------
11 S4.3 designed state/mode.payload {siren, strobe, light, light_effective,
volume_pct, speaking, updated_ts} so an operator can see what the payload is
doing ("未填 = 保持不变" is otherwise unverifiable, S7.5), and 11 S7.16 binds
the G25-G30 voice queries to it. P2 never published it: on 2026-10-30 the
robot's state/mode carried six fields and no payload object, so "音量多大",
"灯开着吗", "警笛响着吗" could only answer "暂时查不到" (field test R004-R006).

Where each field comes from (the honest source for each, not the commanded one)
------------------------------------------------------------------------------
  light / light_effective   payload-service GET /status lights.searchlight --
                            the device readback (QT-2: on/off is b7 alone).
                            No auto lighting runs (PAY-03 not wired), so the
                            level is "on" / "off" and light_src is "manual".
  strobe                    /status lights.redblue != 0 -- "爆闪" means the
                            red/blue lamp (00 PAY-02c), and redblue is read back.
  siren                     the D-mode deterrent loop is running (the siren
                            lives only inside /deter, 18 PL-4 / G-3).
  volume_pct                P2's own volume record (VolumeSetting.effective()):
                            the device has no volume readback ([99] absent).
  speaking                  the speaker domain has a holder (TTS / broadcast /
                            alarm), from the live arbiter.
  updated_ts                wall clock of the last CHANGE of any field above
                            (display only), not of every rebuild.

Rules
-----
  * no /status answer (payload-service down or slow) -> build() returns None
    and the payload object is LEFT OUT of state/mode: a last-known light would
    be a guess dressed as a reading (11 S4.3 "当前实际通断"; CLAUDE.md 3.2);
  * the out-of-range readback (a non-bool searchlight, a non-int redblue) is
    treated as no answer, never coerced.

Boundary: pure (inputs -> dict). The P2 main loop reads /status and the other
inputs once a second and hands the result to ModeFace.
"""
from __future__ import annotations  # postponed annotations

from typing import Any, Dict, Mapping, Optional


class PayloadStateTracker:
    """Builds 11 S4.3 state/mode.payload; remembers when it last changed."""

    def __init__(self) -> None:  # one query answer:  init  
        self._last: Optional[Dict[str, Any]] = None  # last published core fields
        self._updated_ts: Optional[float] = None  # wall time of the last change

    def build(self, status: Optional[Mapping[str, Any]], *, volume_pct: int,  # one query answer: build
              siren_on: bool, speaking: bool, now_wall: float) -> Optional[Dict[str, Any]]:
        """The payload object, or None when the device did not answer."""
        if not isinstance(status, Mapping):  # guard: nothing usable
            return None  # payload-service silent: leave the object out
        lights = status.get("lights")  # value used below
        if not isinstance(lights, Mapping):  # guard: nothing usable
            return None  # no lights readback
        search = lights.get("searchlight")  # value used below
        redblue = lights.get("redblue")  # value used below
        if not isinstance(search, bool) or isinstance(redblue, bool) or not isinstance(redblue, int):  # guard: nothing usable
            return None  # malformed readback: not a reading
        core = {"siren": bool(siren_on), "strobe": redblue != 0,  # value used below
                "light": "on" if search else "off", "light_effective": search,  # id -> spoken label
                "light_src": "manual", "volume_pct": int(volume_pct),  # id -> spoken label
                "speaking": bool(speaking)}
        if core != self._last:  # branch on the live value
            self._last = dict(core)  # remember the new state
            self._updated_ts = now_wall  # it changed now
        out = dict(core)  # local read of the state field
        out["updated_ts"] = self._updated_ts
        return out  # result to the caller


__all__ = ["PayloadStateTracker"]  # value used below
