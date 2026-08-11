"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: speaker_wiring.py
Brief: p2_core speaker/half-duplex wiring -- cmd/audio/speak -> GZH-2 + rt/audio/gate

Description:
p2_core's speaker domain owns the physical speaker path:

  cmd/audio/speak (GEN) <-- p4 (LLM reply) / p3 (task alarm)
    1. close half-duplex gate: rt/audio/gate = {open: false}
    2. HTTP POST /tts to payload-service :18080 (GZH-2)
    3. block for est_ms (returned by payload-service)
    4. open half-duplex gate: rt/audio/gate = {open: true}
    5. publish cmd/audio/speak/ack (with actual duration)

Why p2 (not p4) makes the HTTP call:
  * p2 owns the SPEAKER domain arbiter -- only one caller can hold
    the speaker at a time (14 S4). Centralising the HTTP call in the
    arbiter's callback makes preemption trivial: cancel the pending
    request when someone higher-priority wins the domain.
  * The half-duplex gate is p2's responsibility (RT-A1): publishing
    the gate frame and doing the HTTP call from the SAME thread
    guarantees the gate closes BEFORE audio starts playing, not
    after.

Preemption for MVP: the speaker arbiter is FIFO. Full preemption
lands with 14 S4's ARB-2/ARB-8; this file uses a simple lock so a
second cmd/audio/speak while TTS is playing gets rejected with
E_BUSY (the router can retry).
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

from xbrain.common.errors import E_BUSY, E_UNHEALTHY


_logger = logging.getLogger("xbrain.p2.speaker")


GATE_TOPIC = "rt/audio/gate"
SPEAK_TOPIC = "cmd/audio/speak"
SPEAK_ACK_TOPIC = "cmd/audio/speak/ack"


@dataclass
class SpeakerWiringConfig:
    """All fields required at construction (no defaults)."""
    payload_base_url: str          # http://127.0.0.1:18080
    tts_http_timeout_s: float
    est_ms_per_sentence: float     # fallback for U62


@dataclass
class GatePayload:
    open: bool         # True = mic allowed; False = mic muted (TTS playing)
    reason: str        # 'tts_playback' / 'idle' / 'estop'
    mono_ms: int

    def to_bytes(self) -> bytes:
        return json.dumps({
            "open": self.open,
            "reason": self.reason,
            "mono_ms": self.mono_ms,
        }).encode("utf-8")


class SpeakerBusy(Exception):
    """cmd/audio/speak arrived while another utterance is playing."""


class SpeakerHwError(Exception):
    """payload-service returned failure; speaker probably down."""


class SpeakerDomain:
    """Serialises TTS + gate publish through one lock. Callers
    invoke `handle_speak()` from any thread; the underlying HTTP call
    blocks the caller for est_ms."""

    def __init__(self, cfg: SpeakerWiringConfig, rt_session,
                 now_mono_ms_fn, mic_publisher=None) -> None:
        self._cfg = cfg
        self._rt = rt_session
        self._now = now_mono_ms_fn
        self._lock = threading.Lock()
        self._gate_pub = rt_session.declare_publisher(GATE_TOPIC)
        # 2026-08-11 V-HALFDUPLEX-1: optional handle to MicPublisherThread.
        # When set, handle_speak() calls .mute()/.unmute() around the
        # TTS playback window so the TTS audio played through the GZH-2
        # speaker never re-enters through the USB MIC. rt/audio/gate is
        # still published for external observers (HMI, event log), but
        # the actual gating is done at the p2 publisher source -- p4
        # doesn't need to filter anything.
        self._mic_pub = mic_publisher
        # Announce idle-open initially.
        self._publish_gate(open_=True, reason="idle")

    def _publish_gate(self, open_: bool, reason: str) -> None:
        payload = GatePayload(open=open_, reason=reason,
                                mono_ms=self._now())
        self._gate_pub.put(payload.to_bytes())

    def handle_speak(self, text: str) -> dict:
        """Blocking. Returns an ack dict with {ok, actual_ms, code}.

        Half-duplex order:
          1. mute MicPublisher BEFORE the TTS request lands on GZH-2,
             so the very first sample the speaker plays cannot enter
             the MIC pipeline.
          2. publish gate=closed for external observers.
          3. call TTS + sleep the estimated playback duration.
          4. publish gate=open + unmute (drains any queued frames
             captured during the mute window -- see unmute()).
        """
        if not self._lock.acquire(blocking=False):
            return {"ok": False, "code": E_BUSY,
                    "reason": "speaker busy"}
        muted_here = False
        try:
            if self._mic_pub is not None:
                self._mic_pub.mute()
                muted_here = True
            self._publish_gate(open_=False, reason="tts_playback")
            try:
                est_ms = self._invoke_tts(text)
            except SpeakerHwError as exc:
                self._publish_gate(open_=True, reason="idle")
                if muted_here:
                    self._mic_pub.unmute()
                    muted_here = False
                return {"ok": False, "code": E_UNHEALTHY,
                        "reason": str(exc)}
            # Playback is asynchronous on the device side; est_ms is the
            # TTS builder's estimate. Sleep the same window so the mute
            # is released only after the speaker has actually gone quiet.
            time.sleep(est_ms / 1000.0)
            self._publish_gate(open_=True, reason="idle")
            return {"ok": True, "actual_ms": est_ms, "code": "OK"}
        finally:
            if muted_here:
                # unmute drains the capture queue so residual DURING-tts
                # frames don't leak through after we unmute.
                self._mic_pub.unmute()
            self._lock.release()

    def _invoke_tts(self, text: str) -> float:
        """Call the payload-service TTS endpoint. Kept as a separate
        method so tests can monkeypatch it without touching HTTP."""
        # Import here so pytest doesn't drag `requests` at module
        # import time for pure-unit tests.
        from xbrain.p4_agent.ai_client.tts_client import (
            TtsClientError, speak,
        )
        try:
            return speak(
                base_url=self._cfg.payload_base_url,
                text=text,
                timeout_s=self._cfg.tts_http_timeout_s,
                est_ms_per_sentence=self._cfg.est_ms_per_sentence,
            )
        except TtsClientError as exc:
            raise SpeakerHwError(str(exc)) from exc

    def shutdown(self) -> None:
        """Signal mic-open + drop the publisher on process exit."""
        try:
            self._publish_gate(open_=True, reason="idle")
        finally:
            try:
                self._gate_pub.undeclare()
            except Exception:      # noqa: BLE001
                pass


def parse_speak_payload(raw: bytes) -> str:
    """Extract text from cmd/audio/speak. Envelope is JSON with
    {text: str} plus optional metadata."""
    d = json.loads(raw.decode("utf-8"))
    text = d.get("text", "")
    if not isinstance(text, str) or not text.strip():
        raise SpeakerHwError("cmd/audio/speak has no text field")
    return text
