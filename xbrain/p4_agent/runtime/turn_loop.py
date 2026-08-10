"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: turn_loop.py
Brief: p4 voice turn loop -- rt/audio/mic -> VAD -> ASR -> intent -> dispatch

Description:
Drives one voice turn end-to-end:

  rt/audio/mic frames (RT plane, published by p2 audio_io)
    -> VAD accumulator (feed_frame per 20 ms)
    -> On utterance close: transcribe via services/asr :8010
    -> Classify text via tier-1 keyword rules (fastpath) or LLM
    -> Dispatch via intent_dispatch to ONE of the 5 cmd/* keys
    -> Publish payload on the GEN plane

Threading:
  * Zenoh subscriber callback runs on the Rust thread pool. It
    MUST NOT do sync HTTP or long work (CLAUDE.md 4.2). The
    callback pushes each AudioFrame into a threadsafe queue.
  * A worker thread drains the queue: feeds VAD, on utterance
    close calls ASR + classifier + publish.

Simple intent classification for the MVP:
  * literal keyword lookup for a handful of demo intents; anything
    else -> intent id 'D_UNKNOWN' -> dispatched to cmd/audio/speak
    with an apology text
  * production tier-1 lives in xbrain/p4_agent/classifier/routes.py
    and can be swapped in when the intents.yaml table lands
"""

from __future__ import annotations

import json
import logging
import queue
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

from xbrain.p2_core.audio.audio_io import AudioFrame
from xbrain.p2_core.runtime.mic_capture import decode_frame
from xbrain.p4_agent.runtime.intent_dispatch import (
    DispatchResult, UnknownIntentDispatch, dispatch,
)
from xbrain.p4_agent.runtime.vad import (
    VadConfig, VadState_, feed_frame,
)


_logger = logging.getLogger("xbrain.p4.turn_loop")


MIC_TOPIC = "rt/audio/mic"


# Minimal demo keyword table. Production uses intents.yaml projection.
_DEMO_KEYWORDS = {
    "巡逻": "B01",         # patrol
    "开始巡逻": "B01",
    "回家": "B04",         # return_home
    "去充电": "B02",       # charge
    "充电": "B02",
    "停止": "B09",         # estop
    "急停": "B09",
    "站起来": "A05",       # stand up
    "趴下": "A06",         # sit down
    "你好": "D07",         # chitchat greeting
    "现在几点": "G24",     # time query
    "你在哪": "G01",       # pose query
    "打开警笛": "R05",     # siren on
    "关灯": "R09",         # lights off
}


@dataclass
class TurnLoopConfig:
    """All fields required at construction."""
    asr_base_url: str          # http://127.0.0.1:8010
    asr_http_timeout_s: float
    vad_cfg: VadConfig


class TurnLoopError(Exception):
    pass


def naive_classify(text: str) -> str:
    """MVP classifier. Real one lives in classifier/routes.py; the
    smoke-test only exercises a handful of intents."""
    text = (text or "").strip()
    if not text:
        return "D_UNKNOWN"
    # Longest-first so '开始巡逻' beats '巡逻'.
    for keyword in sorted(_DEMO_KEYWORDS, key=len, reverse=True):
        if keyword in text:
            return _DEMO_KEYWORDS[keyword]
    return "D_UNKNOWN"


def transcribe_utterance(cfg: TurnLoopConfig, pcm_samples: List[int]) -> str:
    """Blocking HTTP call to services/asr :8010."""
    from xbrain.p4_agent.ai_client.asr_client import (
        AsrClientError, transcribe,
    )
    import struct
    pcm_bytes = struct.pack(f"<{len(pcm_samples)}h", *pcm_samples)
    try:
        return transcribe(
            base_url=cfg.asr_base_url,
            pcm_16k_mono=pcm_bytes,
            timeout_s=cfg.asr_http_timeout_s)
    except AsrClientError as exc:
        raise TurnLoopError(f"ASR failed: {exc}") from exc


class TurnLoopWorker(threading.Thread):
    """Drains AudioFrame queue, runs VAD + ASR + dispatch per turn."""

    def __init__(self, cfg: TurnLoopConfig,
                 in_queue: queue.Queue,
                 publish_fn: Callable[[str, bytes], None],
                 stop_evt: threading.Event) -> None:
        super().__init__(name="p4.turn_loop", daemon=True)
        self._cfg = cfg
        self._q = in_queue
        self._publish = publish_fn
        self._stop_evt = stop_evt
        self._vad_state = VadState_()
        # Public counters for smoke-test assertions
        self.turns_dispatched = 0
        self.frames_received = 0
        self.frames_speech = 0
        self.last_intent_id: Optional[str] = None
        self.last_text: Optional[str] = None
        self.errors: List[str] = []

    def run(self) -> None:
        while not self._stop_evt.is_set():
            try:
                item = self._q.get(timeout=0.1)
            except queue.Empty:
                continue
            if isinstance(item, AudioFrame):
                self.frames_received += 1
                self._on_frame(item)
            else:
                _logger.warning("turn_loop: unknown queue item type")

    def _on_frame(self, frame: AudioFrame) -> None:
        closed = feed_frame(self._vad_state, frame.samples,
                              self._cfg.vad_cfg)
        if closed is None:
            return
        # Utterance ended. Run the ASR + classify + dispatch chain.
        try:
            text = transcribe_utterance(self._cfg, closed)
        except TurnLoopError as exc:
            self.errors.append(str(exc))
            _logger.warning("turn_loop asr error: %s", exc)
            return
        text = (text or "").strip()
        _logger.info("turn_loop asr='%s'", text)
        intent_id = naive_classify(text)
        try:
            result = dispatch(intent_id, text)
        except UnknownIntentDispatch as exc:
            self.errors.append(str(exc))
            _logger.warning("turn_loop dispatch fail: %s", exc)
            return
        payload_bytes = json.dumps(
            result.payload, ensure_ascii=False,
            separators=(",", ":")).encode("utf-8")
        self._publish(result.key, payload_bytes)
        self.turns_dispatched += 1
        self.last_intent_id = intent_id
        self.last_text = text
        _logger.info("turn_loop dispatched intent=%s key=%s",
                     intent_id, result.key)


def on_mic_frame_callback(worker_queue: queue.Queue):
    """Return a Zenoh subscriber callback. The callback DECODES the
    JSON envelope + pushes AudioFrame into the worker queue. It is
    intentionally short so the Rust thread returns quickly."""
    def _cb(sample) -> None:
        try:
            frame = decode_frame(bytes(sample.payload))
        except Exception:      # noqa: BLE001
            return
        try:
            worker_queue.put_nowait(frame)
        except queue.Full:
            try:
                worker_queue.get_nowait()
            except queue.Empty:
                pass
            worker_queue.put_nowait(frame)
    return _cb
