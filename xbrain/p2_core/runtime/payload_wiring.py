"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: payload_wiring.py
Brief: p2_core cmd/payload subscriber -> payload-service /lights

Description:
p4_agent's intent dispatch sends payload-hardware intents (D06/D07
red-blue warning lamp on/off, and future D01-D03 searchlight / D04-D05
warning siren) as JSON envelopes on GEN plane topic cmd/payload.
p2_core owns the payload domain (14 S4) and translates each envelope
into an HTTP call to the local payload-service (127.0.0.1:18080),
which in turn drives the GZH-2 device on 8529/8519.

Why route through Zenoh instead of a direct p4->payload-service HTTP
call: the payload domain has an owner (p2), and future arbitration
(other subscribers on cmd/payload -- teleop, cloud command, HMI)
must all funnel through one process. That is the reason cmd/payload
exists as a topic per 11 S8.
"""

from __future__ import annotations

import json
import logging
import threading
from dataclasses import dataclass

_logger = logging.getLogger("xbrain.p2.payload")

CMD_PAYLOAD_TOPIC = "cmd/payload"

# Brightness sent when D01 turns the searchlight on, so 开灯 visibly
# illuminates instead of resuming a possibly-zero remembered level. A UX
# level, NOT a safety value: 14 GL-2 MSG_BRIGHT range is 0..30, and 30 is
# the device's own remembered default (payload memory note). Fine-grained
# brightness is D17 set_light_bright, a separate intent.
_LIGHT_ON_BRIGHT = 30

# D17 level enum -> MSG_BRIGHT value (18-A S1.1 initial values; field-tuned
# per actual illuminance, not safety values). up/down are relative +/-step.
_BRIGHT_LEVEL = {"max": 30, "high": 22, "mid": 15, "low": 8, "min": 1}
_BRIGHT_STEP = 7
_BRIGHT_FLOOR, _BRIGHT_CEIL = 1, 30      # 18-A S1.1: up ceil 30, down floor 1
_REDBLUE_MAX = 16                        # 14 S4.3.0 REDBLUE_MAX (16 patterns)
_VOL_FLOOR, _VOL_CEIL = 0, 100           # 18 S6.4 volume 0..100


@dataclass
class PayloadWiringConfig:
    """All fields required at construction."""
    payload_base_url: str          # http://127.0.0.1:18080
    http_timeout_s: float


class PayloadDomain:
    """Serialises payload HTTP calls behind one lock so a rapid
    intent burst can't overlap two /lights requests to the device
    (payload-service itself is single-connection to the 8529 socket).

    Only reacts to a small closed set of intent ids today. Any
    unknown intent is LOGGED and dropped -- a silent no-op would
    hide the classifier producing an id p2 doesn't know how to
    handle; a raise would kill the callback thread and stop future
    dispatches. Log + count keeps the loop alive and observable."""

    def __init__(self, cfg: PayloadWiringConfig) -> None:
        self._cfg = cfg
        self._lock = threading.Lock()
        self.calls_made = 0
        self.calls_dropped = 0
        self.errors: list = []
        # Locally tracked device state for the relative/cycle intents. The
        # unit has no [99] volume readback and 0x25 does not report the
        # red/blue PATTERN index (only on/off), so 'change to next pattern'
        # (D18) and 'louder/dimmer' (D10/D17 up/down) resolve against what
        # WE last set. Brightness starts at the D01 on-value.
        self._last_strobe_mode = 0           # 0 -> first cycle yields 1
        self._last_bright = _LIGHT_ON_BRIGHT
        self._last_volume = 50               # initial; first abs/rel adjusts

    def handle_envelope(self, payload_bytes: bytes) -> None:
        """Callback body; runs from a worker thread hand-off, NOT the
        Rust Zenoh thread (see main_wiring for the trampoline)."""
        try:
            env = json.loads(payload_bytes.decode("utf-8"))
        except Exception as exc:      # noqa: BLE001
            _logger.warning("cmd/payload parse fail: %s", exc)
            return
        with self._lock:
            self._dispatch(env)

    def _dispatch(self, env: dict) -> None:
        """Route by intent id, reading the slot value from the envelope
        (level/mode/volume). Deliberately verbose so the log names WHICH
        intent triggered the HTTP call."""
        from xbrain.p4_agent.ai_client.lights_client import (
            LightsClientError, set_redblue, set_searchlight, set_volume,
        )
        intent_id = env.get("intent_id", "")
        base = self._cfg.payload_base_url
        to = self._cfg.http_timeout_s
        try:
            if intent_id == "D06":
                # strobe_on == red/blue warning lamp on. Resume the last
                # pattern (or 1 if never set).
                pat = self._last_strobe_mode or 1
                r = set_redblue(base_url=base, on=True, pattern=pat,
                                timeout_s=to)
                self._last_strobe_mode = pat
                self.calls_made += 1
                _logger.info("payload D06 redblue ON pattern=%d -> %s", pat, r)
            elif intent_id == "D07":
                r = set_redblue(base_url=base, on=False, timeout_s=to)
                self.calls_made += 1
                _logger.info("payload D07 redblue OFF -> %s", r)
            elif intent_id == "D18":
                # set_strobe_mode: explicit mode, or cycle current+1 (18-A
                # S1.2). mode 0 never reaches here (parser rejects it).
                mode = env.get("mode")
                if mode is None:
                    mode = (self._last_strobe_mode % _REDBLUE_MAX) + 1
                r = set_redblue(base_url=base, on=True, pattern=int(mode),
                                timeout_s=to)
                self._last_strobe_mode = int(mode)
                self.calls_made += 1
                _logger.info("payload D18 redblue pattern -> %d -> %s",
                             mode, r)
            elif intent_id == "D01":
                # light_on == searchlight (照明灯) on. Visible brightness so
                # it illuminates (2026-08-11 ORIN: D01 was dropped, lamp
                # never lit). Not a safety value (14 GL-2 range 0..30).
                r = set_searchlight(base_url=base, on=True,
                                    bright=_LIGHT_ON_BRIGHT, timeout_s=to)
                self._last_bright = _LIGHT_ON_BRIGHT
                self.calls_made += 1
                _logger.info("payload D01 searchlight ON -> %s", r)
            elif intent_id == "D02":
                r = set_searchlight(base_url=base, on=False, timeout_s=to)
                self.calls_made += 1
                _logger.info("payload D02 searchlight OFF -> %s", r)
            elif intent_id == "D17":
                # set_light_bright: level enum -> MSG_BRIGHT (18-A S1.1).
                level = env.get("level")
                if not level:
                    self.calls_dropped += 1
                    _logger.warning("payload D17 no level slot; dropped")
                    return
                bright = self._resolve_bright(level)
                r = set_searchlight(base_url=base, on=True, bright=bright,
                                    timeout_s=to)
                self._last_bright = bright
                self.calls_made += 1
                _logger.info("payload D17 bright level=%s -> %d -> %s",
                             level, bright, r)
            elif intent_id == "D10":
                # set_volume: {'abs':N} or {'rel':D} -> 0..100 (18 S6.4).
                vol_slot = env.get("volume")
                if not vol_slot:
                    self.calls_dropped += 1
                    _logger.warning("payload D10 no volume slot; dropped")
                    return
                vol = self._resolve_volume(vol_slot)
                r = set_volume(base_url=base, volume=vol, timeout_s=to)
                self._last_volume = vol
                self.calls_made += 1
                _logger.info("payload D10 volume -> %d -> %s", vol, r)
            else:
                self.calls_dropped += 1
                _logger.warning(
                    "payload envelope intent_id=%r not in the p2 handled "
                    "set (D01/D02/D06/D07/D10/D17/D18 today); dropped",
                    intent_id)
        except LightsClientError as exc:
            self.errors.append(str(exc))
            _logger.error("payload http call failed: %s", exc)

    def _resolve_bright(self, level: str) -> int:
        """D17 level enum -> MSG_BRIGHT 0..30 (18-A S1.1). Absolute levels
        map directly; up/down step +/-7 from the last-set brightness,
        clamped to [1, 30]."""
        if level in _BRIGHT_LEVEL:
            return _BRIGHT_LEVEL[level]
        if level == "up":
            return min(_BRIGHT_CEIL, self._last_bright + _BRIGHT_STEP)
        if level == "down":
            return max(_BRIGHT_FLOOR, self._last_bright - _BRIGHT_STEP)
        # Unknown level (should not happen; parser is closed-set): keep
        # current rather than guess a value.
        return self._last_bright

    def _resolve_volume(self, vol_slot: dict) -> int:
        """D10 volume slot {'abs':N} or {'rel':D} -> 0..100 (18 S6.4).
        Relative applies to the last-set volume (no [99] readback on this
        unit); both are clamped to [0, 100]."""
        if "abs" in vol_slot:
            return max(_VOL_FLOOR, min(_VOL_CEIL, int(vol_slot["abs"])))
        delta = int(vol_slot.get("rel", 0))
        return max(_VOL_FLOOR, min(_VOL_CEIL, self._last_volume + delta))
