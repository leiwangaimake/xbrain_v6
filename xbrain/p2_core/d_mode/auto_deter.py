"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: auto_deter.py
Brief: 99 U188 -- the autonomous alarm's siren + red/blue for N seconds, then one spoken warning, robot keeps moving

Description:
What problem this solves
------------------------
The payload device sounds its siren only inside its `deter` mode (services/
payload core/deter.py: siren clip, red/blue flash, steady searchlight), and
P2 enters that mode only as part of system mode ALARM (D), whose motion
behaviour is `hold` (p2_core.yaml mode_motion.d_alarm) -- the robot stops.
The user ruled for the autonomous alarm (99 U188, 2026-11-13): "不停车, 也响
警笛". So this module borrows the DEVICE's deter mode for duration_sec while
the SYSTEM mode stays IDLE (no hold, the patrol / navigation goes on), then
puts the device back and speaks the warning once.

Sequence
--------
  start   only in system IDLE (device func1): POST /mode deter, then POST
          /deter {on, mode: d_mode.redblue_mode, siren_level, tts_reps: 0}
          -- tts_reps 0 is siren + lights only; the deter loop speaks AFTER a
          whole siren clip (about 6 s) and the cloud's duration_sec is 1..20
          s, so its own warnings would usually be cut off. A refused /deter
          puts the device straight back (nothing is left half-armed).
  stop    after duration_sec: POST /deter {on: false}, POST /mode <the mode
          the current system mode maps to>, then the speak callback with the
          warning text (14 S7.3.3 warn_01, p2_core.yaml auto_alarm.speech_text)
          -- /tts is accepted in func1, not in deter.
  abort   the system mode left IDLE while borrowing (an operator started a
          dialog / broadcast / D): into ALARM the D entry has re-armed the
          device itself, so only the ownership is dropped; into anything else
          the siren is switched off and the device moved to that mode's
          device mode (IDLE -> DIALOG is func1 -> func1, for which the
          transition itself sends no /mode, so without this the siren would
          keep going).
  reconcile  while borrowing, the 5 s device reconcile must aim at `deter`,
          not at func1 -- otherwise it would end the siren after up to 5 s
          (reconcile_target()).

What it does NOT do
-------------------
No judgment (suspicion/auto_alarm.py), no events (runtime/alarm_wiring.py),
no system-mode change, no motion command. It reads no clock: every method
takes the caller's monotonic ms. The payload device being absent (sent for
repair, 2026-11-12) only makes the calls fail and get logged; the event is
still published by the wiring.
"""

from __future__ import annotations  # postponed annotations

import logging  # failures are logged, never raised
from typing import Callable, Optional  # hints

_logger = logging.getLogger("xbrain.p2.auto_deter")  # P2 logger tree


class AutoDeter:
    """Borrow the device's deter mode for a few seconds. Main thread only."""

    def __init__(self, device, redblue_mode: int, default_siren_level: float,
                 speech_text: str, speak: Callable[[str], None]) -> None:
        # device = DeviceModeSync (apply / deter); redblue_mode and the default
        # siren level are the tuned d_mode values; speak hands the text to the
        # speaker domain on a worker thread (it blocks for the playback)
        self._dev = device  # payload REST client
        self._redblue = int(redblue_mode)  # 1..16, validated by DeterPlan
        self._default_level = float(default_siren_level)  # 0..1, d_mode.siren_level
        self._text = speech_text  # the warning spoken afterwards
        self._speak = speak  # non-blocking for the caller
        self._until_ms: Optional[int] = None  # None = not borrowing

    @property
    def running(self) -> bool:
        """True while the device is borrowed."""
        return self._until_ms is not None  # set by start, cleared by stop / abort

    def reconcile_target(self, mapped: str) -> str:
        """The device mode the 5 s reconcile should hold: deter while borrowing."""
        return "deter" if self.running else mapped  # see the module docstring

    def start(self, now_ms: int, siren_level_pct: Optional[float], duration_s: float,
              system_idle: bool) -> str:
        """Begin; returns the outcome for the event (started / busy / not_idle / refused)."""
        if self.running:  # one at a time: a second zone's alarm rides on this one
            return "busy"
        if not system_idle:  # an operator mode owns the device
            return "not_idle"
        level = (self._default_level if siren_level_pct is None
                 else max(0.0, min(1.0, float(siren_level_pct) / 100.0)))  # cloud 0..100 -> 0..1
        res = self._dev.apply("deter")  # device into deter (409 = already there)
        if not res.ok:  # device absent / refused
            _logger.warning("p2 auto alarm: /mode deter refused (%s, HTTP %d)", res.reason, res.status)
            return "refused"
        res = self._dev.deter({"on": True, "mode": self._redblue, "siren_level": level,
                               "tts_reps": 0})  # siren + lights only
        if not res.ok:  # not armed: put the device back at once
            _logger.warning("p2 auto alarm: /deter refused (%s, HTTP %d)", res.reason, res.status)
            self._dev.apply("func1")  # IDLE's device mode (device_map MM-1)
            return "refused"
        self._until_ms = now_ms + int(round(duration_s * 1000))  # stop time
        _logger.info("p2 auto alarm: siren on for %.1f s (level %.2f)", duration_s, level)
        return "started"

    def tick(self, now_ms: int, system_idle: bool, system_is_alarm: bool, mapped: str) -> None:
        """Every main-loop pass: stop when due, or give the device up when the
        system mode moved on. mapped = the current system mode's device mode."""
        if not self.running:  # nothing borrowed
            return
        if not system_idle:  # an operator mode took over
            self._until_ms = None  # ownership dropped first: no second stop
            if system_is_alarm:  # D re-armed the device itself
                return
            self._off(mapped)  # siren off, device to the new mode's device mode
            return
        if now_ms < self._until_ms:  # still sounding
            return
        self._until_ms = None  # done
        self._off(mapped)  # back to func1
        self._speak(self._text)  # then the spoken warning, once

    def _off(self, mapped: str) -> None:
        """POST /deter off then /mode mapped; failures are logged (the reconcile repairs the mode)."""
        res = self._dev.deter({"on": False})  # siren and lights off
        if not res.ok:  # a /mode switch below tears the loop down anyway
            _logger.warning("p2 auto alarm: /deter off refused (%s, HTTP %d)", res.reason, res.status)
        res = self._dev.apply(mapped)  # back where the system mode wants it
        if not res.ok:  # reconcile tries again within 5 s
            _logger.warning("p2 auto alarm: /mode %s refused (%s, HTTP %d)", mapped, res.reason, res.status)


__all__ = ["AutoDeter"]  # public surface
