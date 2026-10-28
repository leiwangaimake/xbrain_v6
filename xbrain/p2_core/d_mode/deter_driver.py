"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: deter_driver.py
Brief: D mode on the payload device (14 S7.3.2) -- arm /deter, strobe off, DT-5 speech cut-off, re-arm after drift

Description:
What problem this solves
------------------------
Until 2026-10-28 (99 U99, second batch C2) entering D mode did nothing on the
device: no /deter was ever sent, so the robot "in alarm" made no sound. Worse,
the 5 s device reconcile then moved payload-service into `deter` (the mapped
device mode of ALARM) WITHOUT arming it -- a silent deter mode in which /play
and /tts are refused. d_mode/sequencer.py had been written for this but was
never called, and its step list disagrees with 14 S7.3.2 (it confirms the
strobe pattern instead of switching the strobe off, and numbers the steps
DT-1..DT-4, which are constraint ids, not steps). This module follows the
14 S7.3.2 sequence literally.

The sequence (14 S7.3.2, after POST /mode deter has returned 200 / 409)
----------------------------------------------------------------------
  entry 2  POST /deter {on: true, mode: redblue_mode, siren_level, tts_reps}
           -- a refusal (409: device not in deter, 5xx, no answer) is a
           failed entry: the caller refuses the switch and rolls back (ML-3).
  entry 3  POST /lights {strobe: false} (GL-8a). The payload deter loop
           already sends strobe off itself since PER-42, so this is the
           belt to that braces -- a failure is logged, never fatal.
  DT-5     after speech_max_repeats x loop_s (10 x 20.3 s ~ 203 s) re-POST
           /deter with tts_reps 0: the siren and the red/blue go on, the
           spoken warning stops. tts_reps is "per cycle", max_repeats is
           "in total" -- two different numbers, P2 owns the second one.
  exit     POST /deter {on: false} when the caller asks for it (exit to a
           non-B/D mode, sent BEFORE the /mode switch so the siren stops even
           if the switch fails); towards B the /mode func2 teardown stops the
           loop anyway and the explicit off would only collect a 409.
  re-arm   the heartbeat reconcile found the device out of deter and put it
           back: the loop died with the mode switch, so arm it again with the
           CURRENT speech state (a re-arm after DT-5 must not restart speech).

What it does NOT do: it does not POST /mode (DeviceModeSync.apply, owned by
the transition callback), does not touch the arbiter or the mic gate, and
reads no clock -- every method takes the caller's monotonic ms.
"""

from __future__ import annotations  # postponed annotations

import logging  # failures are logged, never raised
from dataclasses import dataclass  # the frozen plan
from typing import Any, Callable, Dict, Optional  # type hints only

_logger = logging.getLogger("xbrain.p2.d_mode")  # P2 logger tree

#: Retry spacing for a refused DT-5 re-POST: the main loop runs at 10 Hz and
#: must not hammer a payload-service that just refused.
CUTOFF_RETRY_MS = 5000  # same order as the 5 s device reconcile


@dataclass(frozen=True)  # read once at startup
class DeterPlan:
    """The five p2_core.yaml d_mode values P2 puts on the wire or times by."""
    redblue_mode: int          # /deter mode, 1..16 (== payload_light.deter_redblue_mode)
    siren_level: float         # /deter siren_level, 0..1 (real-device 0.45)
    tts_reps: int              # /deter tts_reps, warnings per cycle (3)
    speech_max_repeats: int    # DT-5: cycles before the speech stops (10)
    loop_s: float              # one cycle's length, DT-5 timing only (20.3)

    @classmethod  # the only constructor production uses
    def from_config(cls, get: Callable[[str], Any]) -> "DeterPlan":
        """Build from the resolved p2_core snapshot. get(key) returns the
        dotted key's value or None; a missing or out-of-range value raises
        ValueError naming the key -- P2 must not start D mode on a guess
        (CLAUDE.md 3.6)."""
        def need(key: str, kind: type) -> Any:  # one required key
            v = get(key)  # dotted path into p2_core.yaml
            if v is None or isinstance(v, bool) or not isinstance(v, (int, float)):
                raise ValueError("p2_core config: %s missing or not a number" % key)
            return kind(v)  # int / float as the wire wants
        plan = cls(redblue_mode=need("d_mode.redblue_mode", int),
                   siren_level=need("d_mode.siren_level", float),
                   tts_reps=need("d_mode.tts_reps", int),
                   speech_max_repeats=need("d_mode.speech_max_repeats", int),
                   loop_s=need("d_mode.loop_s", float))
        if not 1 <= plan.redblue_mode <= 16:  # payload DeterCommand range; 0 = off is banned
            raise ValueError("p2_core config: d_mode.redblue_mode %r not in 1..16"
                             % plan.redblue_mode)
        if not 0.0 <= plan.siren_level <= 1.0:  # payload DeterCommand range
            raise ValueError("p2_core config: d_mode.siren_level %r not in 0..1"
                             % plan.siren_level)
        if plan.tts_reps < 1 or plan.speech_max_repeats < 1 or plan.loop_s <= 0:
            raise ValueError("p2_core config: d_mode tts_reps / speech_max_repeats"
                             " must be >= 1 and loop_s > 0")
        if get("d_mode.deter_texts_supported") is not False:  # DT-7 one-key flag
            # payload /deter has no texts field (DeterCommand: on / mode /
            # siren_level / tts_reps); true would mean sending a field the
            # device rejects with 422. Flip it only together with that change.
            raise ValueError("p2_core config: d_mode.deter_texts_supported must be "
                             "false -- payload /deter has no texts[] (14 DT-7)")
        return plan  # validated

    def arm_body(self, speech: bool) -> Dict[str, Any]:  # the POST /deter body
        """POST /deter body; speech False is the DT-5 siren-only re-arm."""
        return {"on": True, "mode": self.redblue_mode,
                "siren_level": self.siren_level,
                "tts_reps": self.tts_reps if speech else 0}  # 0 = siren + lights only

    @property  # derived, so it cannot drift from the two inputs
    def cutoff_ms(self) -> int:  # DT-5 deadline after entry
        """DT-5: speech stops this long after entry."""
        return int(round(self.speech_max_repeats * self.loop_s * 1000))  # ~203 s


class DeterDriver:
    """The device half of D mode. Owned by the P2 main thread (single-threaded)."""

    def __init__(self, device, plan: DeterPlan) -> None:  # device = DeviceModeSync
        """device needs deter(body) and set_lights(body) returning SyncResult."""
        self._dev = device  # the payload REST client
        self._plan = plan  # validated config
        self._entered_ms: Optional[int] = None  # None = not in D
        self._speech = True  # False after DT-5
        self._retry_at_ms = 0  # DT-5 re-POST backoff

    @property  # the main loop asks
    def running(self) -> bool:  # True between enter and exit
        """True between a successful enter() and exit()."""
        return self._entered_ms is not None  # entered and not exited

    @property  # for logs and state
    def speech_on(self) -> bool:  # False once DT-5 fired
        """False once DT-5 has cut the speech."""
        return self._speech  # current wire value

    def enter(self, now_mono_ms: int):  # entry steps 2 and 3
        """Arm the deterrent (entry 2) and switch the strobe off (entry 3).
        Returns the /deter SyncResult; not ok = the caller refuses D (ML-3)."""
        res = self._dev.deter(self._plan.arm_body(True))  # entry 2
        if not res.ok:  # nothing armed: no state change here
            return res  # the caller rolls back
        self._lights_strobe_off()  # entry 3
        self._entered_ms = now_mono_ms  # DT-5 starts now
        self._speech = True  # a fresh D speaks again
        self._retry_at_ms = 0  # no pending retry
        _logger.info("p2 D mode armed: %s, speech cut-off in %d ms",
                     self._plan.arm_body(True), self._plan.cutoff_ms)
        return res  # ok

    def exit(self, *, disarm: bool) -> None:  # leaving D
        """Leave D. disarm=True sends /deter {on:false} first (exit to a
        non-B/D mode); False when a /mode switch already tore the loop down."""
        if self._entered_ms is None:  # not in D: nothing to undo
            return  # idempotent
        self._entered_ms = None  # stop DT-5 first: no re-POST after this
        if disarm:  # the siren must stop even if /mode then fails
            res = self._dev.deter({"on": False})  # exit 1
            if not res.ok:  # logged; /mode teardown is the second stop
                _logger.error("p2 D exit: /deter off refused (%s, HTTP %d)",
                              res.reason, res.status)

    def tick(self, now_mono_ms: int) -> bool:  # called every main-loop pass
        """DT-5: True on the pass the speech was cut (siren + lights stay)."""
        if self._entered_ms is None or not self._speech:  # not in D / already cut
            return False  # nothing due
        if now_mono_ms - self._entered_ms < self._plan.cutoff_ms:  # not yet
            return False  # still speaking
        if now_mono_ms < self._retry_at_ms:  # a refused re-POST backs off
            return False  # wait
        res = self._dev.deter(self._plan.arm_body(False))  # siren-only re-arm
        if not res.ok:  # retry later, keep speaking meanwhile
            self._retry_at_ms = now_mono_ms + CUTOFF_RETRY_MS  # back off
            _logger.error("p2 D DT-5 speech cut-off refused (%s, HTTP %d); retry in %d ms",
                          res.reason, res.status, CUTOFF_RETRY_MS)
            return False  # not cut yet
        self._speech = False  # cut
        self._lights_strobe_off()  # the restart re-sent the device lights
        _logger.info("p2 D DT-5: speech stopped after %d ms, siren + red/blue continue",
                     now_mono_ms - self._entered_ms)
        return True  # fired

    def rearm(self) -> bool:  # after reconcile put the device back into deter
        """Re-arm with the CURRENT speech state; True when the device took it."""
        if self._entered_ms is None:  # not in D: nothing to re-arm
            return False  # no-op
        res = self._dev.deter(self._plan.arm_body(self._speech))  # same fields as now
        if res.ok:  # armed again
            self._lights_strobe_off()  # as on entry
            _logger.warning("p2 D re-armed after device drift (speech=%s)", self._speech)
        else:  # the next reconcile will try again
            _logger.error("p2 D re-arm refused (%s, HTTP %d)", res.reason, res.status)
        return res.ok  # for the caller's log

    def _lights_strobe_off(self) -> None:  # GL-8a
        """POST /lights {strobe:false}; a failure is an indicator fault only."""
        res = self._dev.set_lights({"strobe": False})  # one partial POST
        if not res.ok:  # logged by DeviceModeSync too
            _logger.warning("p2 D strobe-off refused (%s)", res.reason)


__all__ = ["DeterPlan", "DeterDriver", "CUTOFF_RETRY_MS"]  # public surface
