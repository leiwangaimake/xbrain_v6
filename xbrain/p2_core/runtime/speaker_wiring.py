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

from xbrain.common.errors import E_BUSY, E_SCHEMA, E_UNHEALTHY

_logger = logging.getLogger("xbrain.p2.speaker")


GATE_TOPIC = "rt/audio/gate"
SPEAK_TOPIC = "cmd/audio/speak"
SPEAK_ACK_TOPIC = "cmd/audio/speak/ack"

#: The domain-2 sources a cmd/audio/speak may name (14 S4.1 table): the three
#: TTS channels plus the two system voices. alarm_d / broadcast_b are NOT here:
#: they are mode-driven holders P2 takes itself, never via this key (11
#: S8.8.1 "P2 validates source by publisher" -- a speak request claiming the
#: alarm's 900 would outrank a broadcast).
SPEAK_SOURCES = frozenset({"tts_cloud", "tts_wecom", "tts_local",
                           "prompt_tone", "bit_announce"})
#: How often a playing TTS checks whether a higher source is waiting (11
#: S7A.3 P-1). 50 ms keeps a broadcast's takeover well inside the 100 ms
#: grace; it is a poll period, not a safety bound.
PREEMPT_POLL_S = 0.05


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
                 now_mono_ms_fn, mic_publisher=None, hub=None) -> None:
        self._cfg = cfg
        # Domain-2 arbitration (14 S4.1, 99 U99 C1). None = the pre-2026-10-28
        # behaviour (one non-blocking lock, first come first served) -- kept
        # only for the unit tests that build a SpeakerDomain without a hub.
        self._hub = hub
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
        # state/audio(11 S8.10)要的三个量: 在不在说 . 从何时起 . 最后一次
        # gate 的取值. 它们本来就在这个对象里(锁 + _publish_gate 的入参),
        # 只是没有对外的读法 -- 外部去摸 self._lock 会把内部同步机制变成
        # 公开接口, 那才是真正难改的耦合.
        self._speaking_since_mono: "float | None" = None
        self._gate_open = True
        self._gate_reason = "idle"
        self._speaking_source: Optional[str] = None
        # Announce idle-open initially.
        self._publish_gate(open_=True, reason="idle")

    def set_gate(self, open_: bool, reason: str) -> None:
        """外部驱动的门控(B 模式进出用). 与 handle_speak 内部那次走同一条路.

        *** 公开这一个入口, NO 不让调用方直接碰 _publish_gate.
        门的取值必须同时进 rt/audio/gate 与 state/audio(BIZ-P2-0 断言四),
        而"同时"的唯一保证是只有一处赋值.

        *** 关门要连[真的静音 MicPublisher]一起做, NO 不能只发门控报文.
        handle_speak 那条路是 mute() 之后才 _publish_gate 的; 只发报文的话,
        state/audio 会显示 gate_reason=broadcast_active 而 mic.open 仍是
        true, 麦克风照常上行 -- 门"关了"只存在于报文里.
        2026-09-04 终测接 B 模式关麦时先只写了 _publish_gate, 实测报文上
        gate_reason 已变而 mic.open 没变, 才发现漏了这一半.
        """
        if self._mic_pub is not None:
            if open_:
                self._mic_pub.unmute()
            else:
                self._mic_pub.mute()
        self._publish_gate(open_, reason)

    def _publish_gate(self, open_: bool, reason: str) -> None:
        # 记住最后一次的取值: state/audio 与 rt/audio/gate 必须一致
        # (BIZ-P2-0 断言四), 而一致的前提是两边读同一个来源.
        self._gate_open = open_
        self._gate_reason = reason
        payload = GatePayload(open=open_, reason=reason,
                                mono_ms=self._now())
        self._gate_pub.put(payload.to_bytes())

    def handle_speak(self, text: str,
                     source: Optional[str] = None) -> dict:
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
        if self._hub is not None:
            return self._speak_arbitrated(text, source)
        if not self._lock.acquire(blocking=False):
            return {"ok": False, "code": E_BUSY,
                    "reason": "speaker busy"}
        muted_here = False
        try:
            if self._mic_pub is not None:
                self._mic_pub.mute()
                muted_here = True
            self._speaking_since_mono = self._now() / 1000.0
            # 域2 持有者. None = 发布方没填 source(见 parse_speak_source).
            self._speaking_source = source
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

    def _speak_arbitrated(self, text: str, source: Optional[str]) -> dict:
        """handle_speak through the domain-2 arbiter (14 S4.1, 11 S7A).

        1. source must be one of SPEAK_SOURCES (11 S8.8.1: required; a missing
           one is refused, never guessed -- guessing tts_local would rank a
           cloud reply below a local one).
        2. request with atomic=True: idle -> granted; a lower holder -> the
           arbiter queues us and that holder yields; equal or higher holder
           (a broadcast, an alarm, another TTS) -> E_BUSY, as before.
        3. play exactly as before (mute, gate closed, TTS, sleep est_ms) but
           sleep in PREEMPT_POLL_S steps: when a higher source is waiting we
           ack_preempt and stop at once -- and we do NOT reopen the gate or
           unmute, because the preemptor (B / D) owns them from now on. The
           old code reopened the gate at the end of every TTS, so a broadcast
           entered during a TTS got its mic gate opened under it.
        4. release(done) at the normal end.
        """
        if source not in SPEAK_SOURCES:
            return {"ok": False, "code": E_SCHEMA,
                    "reason": "speak source %r is not a domain-2 TTS source %s"
                              % (source, sorted(SPEAK_SOURCES))}
        hub = self._hub
        req_id = hub.new_req_id("speaker", source)
        grant = hub.request("speaker", source, self._now(), req_id=req_id,
                            atomic=True)
        res = getattr(grant.result, "value", grant.result)
        if res == "denied":
            h = hub.holder("speaker")
            return {"ok": False, "code": grant.code or E_BUSY,
                    "reason": "speaker held by %s" % (h.source_id if h else "?")}
        if res == "queued" and not self._wait_for_grant(source, req_id):
            hub.cancel("speaker", source, req_id, self._now())
            return {"ok": False, "code": E_BUSY,
                    "reason": "speaker not handed over in time"}
        with self._lock:          # speaking window; audio_view reads it
            if self._mic_pub is not None:
                self._mic_pub.mute()
            self._speaking_since_mono = self._now() / 1000.0
            self._speaking_source = source
            self._publish_gate(open_=False, reason="tts_playback")
            try:
                est_ms = self._invoke_tts(text)
            except SpeakerHwError as exc:
                self._publish_gate(open_=True, reason="idle")
                if self._mic_pub is not None:
                    self._mic_pub.unmute()
                hub.release("speaker", source, req_id, "error", self._now())
                return {"ok": False, "code": E_UNHEALTHY, "reason": str(exc)}
            end = time.monotonic() + est_ms / 1000.0
            while time.monotonic() < end:
                if not hub.holds("speaker", source, req_id):
                    # revoked (lease / forced): the new holder owns gate + mic
                    return {"ok": False, "code": E_BUSY,
                            "reason": "speaker revoked mid-playback"}
                if hub.being_preempted("speaker", source, req_id):
                    hub.ack_preempt("speaker", source, req_id, self._now())
                    return {"ok": False, "code": E_BUSY,
                            "reason": "preempted by a higher source"}
                time.sleep(min(PREEMPT_POLL_S, max(0.0, end - time.monotonic())))
            self._publish_gate(open_=True, reason="idle")
            if self._mic_pub is not None:
                self._mic_pub.unmute()
            hub.release("speaker", source, req_id, "done", self._now())
            return {"ok": True, "actual_ms": est_ms, "code": "OK"}

    def _wait_for_grant(self, source: str, req_id: str) -> bool:
        """A queued request: wait until the holder yields or the arbiter's
        forced deadline promotes us (the P2 main loop ticks it). Bounded by
        the TTS HTTP timeout plus the arbiter's wait_atomic ceiling."""
        limit = time.monotonic() + self._cfg.tts_http_timeout_s + 3.5
        while time.monotonic() < limit:
            if self._hub.holds("speaker", source, req_id):
                return True
            time.sleep(PREEMPT_POLL_S)
        return False

    def _invoke_tts(self, text: str) -> float:
        """Call the payload-service TTS endpoint. Kept as a separate
        method so tests can monkeypatch it without touching HTTP."""
        # Import here so pytest doesn't drag `requests` at module
        # import time for pure-unit tests.
        from xbrain.p4_agent.ai_client.tts_client import (
            TtsClientError,
            speak,
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

    def audio_view(self) -> dict:
        """state/audio(11 S8.10)要的那几个量, 一次读出来.

        *** 一次性快照, NO 不是几个独立的 getter.
        speaker_state 与 gate_reason 必须自洽(BIZ-P2-0 断言四): 分成多次读的
        话, 中间可能夹进一次 handle_speak 的状态跃迁, 于是同一条 state/audio
        里"在说话"配着"gate 开着" -- 那正是该断言要抓的不一致.

        speaking 用 locked() 判: 锁被 handle_speak 持有的整个区间就是"在说",
        与 gate 的关闭区间同起同止(见 handle_speak 的半双工时序).
        """
        speaking = self._lock.locked()
        if self._hub is not None:
            # The arbiter is the holder of record (14 S4.1): a broadcast or an
            # alarm holds domain 2 without ever entering handle_speak.
            h = self._hub.holder("speaker")
            if h is not None and h.source_id in SPEAK_SOURCES and speaking:
                return {"speaking": True,
                        "since_mono": self._speaking_since_mono,
                        "holder": h.source_id,
                        "gate_open": self._gate_open,
                        "gate_reason": self._gate_reason}
        return {
            "speaking": speaking,
            "since_mono": self._speaking_since_mono if speaking else None,
            # S8.10 逐字 "none = 空闲". 说话中而 source 未知时给 None
            # (字段在但为空), 与 "none"(确实空闲)是两件事.
            "holder": (self._speaking_source if speaking else "none"),
            "gate_open": self._gate_open,
            "gate_reason": self._gate_reason,
        }

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


def parse_speak_source(raw: bytes) -> Optional[str]:
    """SpeakRequest.source(11 S8.8.1) -- 域2 的持有者标识.

    S8.8.1 里 source 是[必填], 取值须属 14 S4.1 注册表(alarm_d(900) /
    broadcast_b(800) / tts_cloud(600) / tts_wecom(500) / tts_local(400)).
    parse_speak_payload 至今只取 text, 把它丢了 -- 于是 state/audio 的
    speaker.holder 无源可依.

    *** 缺失时返回 None, NO 不兜一个 "tts_local".
    兜底会把"发布方没按契约填"变成"本地 TTS 在说话", 而这两件事在报文上
    完全一样. holder=None 让下游看得见"有人在说但不知是谁".
    *** 本函数[不]校验闭集. S8.8.1 逐字要求 P2 "按发布进程校验" source,
    那是一条[拒绝]路径(校验不过要回 ack 拒绝), 属 BIZ-P2-2 仲裁面; 在这里
    抛会把一条本可播出的话变成异常. 已登记为债: 见模块头注.
    """
    try:
        d = json.loads(raw.decode("utf-8"))
    except Exception:      # noqa: BLE001
        return None
    if not isinstance(d, dict):
        return None
    src = d.get("source")
    return src if isinstance(src, str) and src else None
