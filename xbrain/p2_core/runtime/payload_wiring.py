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

    def handle_envelope(self, payload_bytes: bytes) -> None:
        """Callback body; runs from a worker thread hand-off, NOT the
        Rust Zenoh thread (see main_wiring for the trampoline)."""
        try:
            env = json.loads(payload_bytes.decode("utf-8"))
        except Exception as exc:      # noqa: BLE001
            _logger.warning("cmd/payload parse fail: %s", exc)
            return
        intent_id = env.get("intent_id", "")
        with self._lock:
            self._dispatch(intent_id)

    def _dispatch(self, intent_id: str) -> None:
        """Route by intent id. Deliberately verbose so the log names
        WHICH intent triggered the HTTP call -- 'D06 lights on' beats
        'lights on' when three intents route here."""
        from xbrain.p4_agent.ai_client.lights_client import (
            LightsClientError, set_redblue,
        )
        try:
            if intent_id == "D06":
                # strobe_on == red/blue warning lamp on. Pattern 1.
                r = set_redblue(base_url=self._cfg.payload_base_url,
                                on=True,
                                timeout_s=self._cfg.http_timeout_s)
                self.calls_made += 1
                _logger.info("payload D06 lights redblue ON -> %s", r)
            elif intent_id == "D07":
                r = set_redblue(base_url=self._cfg.payload_base_url,
                                on=False,
                                timeout_s=self._cfg.http_timeout_s)
                self.calls_made += 1
                _logger.info("payload D07 lights redblue OFF -> %s", r)
            else:
                self.calls_dropped += 1
                _logger.warning(
                    "payload envelope intent_id=%r not in the p2 "
                    "handled set (D06/D07 today); dropped",
                    intent_id)
        except LightsClientError as exc:
            self.errors.append(str(exc))
            _logger.error("payload http call failed: %s", exc)
