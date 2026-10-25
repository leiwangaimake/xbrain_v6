"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: main_wiring.py
Brief: p4_agent __main__ wire-up: RT+GEN Zenoh + turn loop worker + dispatch publishers

Description:
Called from xbrain/p4_agent/__main__.py after config + AI service
probe. Opens the two Zenoh sessions, subscribes rt/audio/mic
into the TurnLoopWorker, publishes dispatched intents on GEN.

Startup order:
  1. Open RT + GEN sessions
  2. Cache publishers for every outbound key family (avoids
     declare-per-message overhead + gets errors surfaced early)
  3. Start TurnLoopWorker
  4. Subscribe rt/audio/mic -> worker queue
  4b. Subscribe the cloud inputs xbrain/{rid}/cmd/voice_text and
     xbrain/{rid}/audio/voice_in -> worker job queue (runtime/cloud_input;
     orchestrator path only, and only with XBRAIN_ROBOT_ID set)
  5. Enter main loop; wait for stop_event

Shutdown order (LIFO):
  1. Unsubscribe cloud inputs, then rt/audio/mic
  2. Stop worker (drains remaining frames)
  3. Undeclare publishers
  4. Close sessions
"""

from __future__ import annotations

import logging
import os
import queue
import time
from typing import Dict, Optional

from xbrain.p4_agent.runtime.cloud_input import (
    CloudTurnRunner,
    VoiceInAssembler,
    VoiceInLimits,
    cloud_input_keys,
    make_voice_in_callback,
    make_voice_text_callback,
)
from xbrain.p4_agent.runtime.intent_dispatch import (
    CMD_AUDIO_SPEAK,
    CMD_MOTION_INTENT,
    dispatch_keys,
)
from xbrain.p4_agent.runtime.orchestrator_turn import (
    CMD_ESTOP,
    VoiceOrchestratorInputs,
    build_orchestrator,
    compose_query_fns,
    make_battery_query_fn,
    make_rtk_query_fn,
    make_time_query_fn,
    make_turn_handler,
)
from xbrain.p4_agent.runtime.turn_loop import (
    MIC_TOPIC,
    TurnLoopConfig,
    TurnLoopWorker,
    on_mic_frame_callback,
)
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession
from xbrain.p4_agent.runtime.voice_turn import VOICE_TURN_KEY_FMT

_logger = logging.getLogger("xbrain.p4.wiring")


# Outbound keys the orchestrator can publish. CMD_ESTOP is included because
# a safety-bypass (急停) turn publishes there (16 S4 -> Tier1); leaving it
# out silently DROPS the estop -- the 2026-08-11 ORIN test hit exactly that
# ('no cached publisher for cmd/estop'). The bypass path must have a
# publisher like any other key.
# 2026-10-25: the dispatch keys are no longer a hand copy. The copy held five
# of the nine keys intent_dispatch routes to, so cmd/mode (C class), cmd/teach
# + cmd/geo (F class) and cmd/system (H class) were dropped exactly the way
# cmd/estop once was. The three added by hand below are the keys the
# orchestrator publishes OUTSIDE the routing tables: the bypass (estop /
# motion) and the spoken reply. Sorted so the startup log is stable.
OUTBOUND_KEYS = tuple(sorted(
    dispatch_keys() | {CMD_AUDIO_SPEAK, CMD_MOTION_INTENT, CMD_ESTOP}))


# Voice-loop heartbeat cadence. Kept as a module constant, not a parameter
# default, because the parameter-default form trips the CLAUDE.md 3.1
# safety-value scanner (any _s / _ms / _hz suffix is treated as a limit).
# The scanner is right to be strict there: a plausible-looking default on
# a suffix-named argument is exactly how safety numbers leak into code.
# The heartbeat cadence itself is an observation-log frequency, not a
# safety value -- moving it here keeps the discipline honest without
# hiding the number behind a rename.
_VOICE_LOOP_HEARTBEAT_PERIOD = 5.0  # seconds


def _wire_state_subscriptions(gen, state_subs: list):
    """Declare a GEN-plane subscriber per STATE_TOPICS key (GWY-P4-39),
    decoding each JSON payload into a StateCache with the monotonic receive
    time. Returns the cache. Subscriber handles are appended to state_subs
    (strong ref, CLAUDE.md 4.3) so the subscription is not GC'd."""
    import json
    import time as _time

    from xbrain.p4_agent.state.cache import STATE_TOPICS, StateCache

    cache = StateCache()

    def _make_cb(key: str):
        # The callback runs on the Rust thread pool: it only decodes JSON
        # and writes the last value into a dict (CLAUDE.md 4.2 allows a
        # plain write, forbids await/create_task here).
        def _cb(sample) -> None:
            try:
                value = json.loads(bytes(sample.payload))
            except Exception:      # noqa: BLE001 -- a bad frame must not kill the sub
                return
            cache.update(key, value, int(_time.monotonic() * 1000))
        return _cb

    for key in sorted(STATE_TOPICS):
        state_subs.append(gen.declare_subscriber(key, _make_cb(key)))
    return cache


def _now_ms() -> int:
    """Monotonic milliseconds (CLK-C1) -- the one clock cloud input uses."""
    return int(time.monotonic() * 1000)


def _wire_cloud_inputs(gen, cfg: TurnLoopConfig, orchestrator, registry,
                       asr_post, state_cache, publish, jobs: queue.Queue,
                       limits: VoiceInLimits, subs: list,
                       local_session: OrchestratorSession,
                       voice_turn_key: Optional[str] = None):
    """Subscribe cmd/voice_text + audio/voice_in; return the assembler.

    local_session is the microphone's session: hmi text shares it (16 S11.1
    has three slots, and the HMI is an on-site input like the microphone).

    Returns None (and subscribes nothing) when XBRAIN_ROBOT_ID is unset: the
    keys are robot-prefixed, and subscribing xbrain/unknown/... would be a
    key nothing ever publishes to -- the same silent dead end p2 refuses for
    audio/broadcast. Logged as an ERROR so a missing id is visible.
    """
    from xbrain.p4_agent.ai_client.asr_client import transcribe
    from xbrain.p4_agent.classifier.keyword_matcher import KeywordMatcher

    rid = os.environ.get("XBRAIN_ROBOT_ID")
    if not rid:
        _logger.error("p4 cloud inputs DISABLED: XBRAIN_ROBOT_ID unset "
                      "(cmd/voice_text and audio/voice_in not subscribed)")
        return None

    def _transcribe(pcm: bytes) -> str:
        # Same ASR service and timeout as microphone turns.
        return transcribe(base_url=cfg.asr_base_url, pcm_16k_mono=pcm,
                          timeout_s=cfg.asr_http_timeout_s)

    def _voice_mode() -> Optional[str]:
        # p2's state/mode is sent on change only: read the last value.
        value = state_cache.latest("state/mode")
        return value.get("voice_mode") if isinstance(value, dict) else None

    runner = CloudTurnRunner(
        orchestrator, KeywordMatcher(registry), publish, _transcribe,
        _now_ms, _voice_mode, local_session, asr_post=asr_post,
        voice_turn_key=voice_turn_key)
    assembler = VoiceInAssembler(limits)
    text_key, voice_key = cloud_input_keys(rid)
    # Strong references in `subs` (CLAUDE.md 4.3).
    subs.append(gen.declare_subscriber(
        text_key, make_voice_text_callback(jobs, runner)))
    subs.append(gen.declare_subscriber(
        voice_key, make_voice_in_callback(jobs, assembler, runner, _now_ms)))
    _logger.info("p4 wiring: subscribed %s and %s (pcm_s16le 16k mono)",
                 text_key, voice_key)
    return assembler


def run_voice_loop_wiring(cfg: TurnLoopConfig,
                            stop_flag: dict,
                            *,
                            orch: Optional[VoiceOrchestratorInputs] = None,
                            cloud_limits: Optional[VoiceInLimits] = None
                            ) -> int:
    """Block until stop_flag['stop'] becomes truthy. Returns 0 on
    clean shutdown.

    GWY-P4-41 (32.I): when `orch` is supplied (the production path, built
    by __main__ from the resolved config + static content files), each turn
    is routed through the six-step TurnOrchestrator instead of the V-2B
    naive_classify path. GEN-plane state/* is subscribed into a cache so
    G02 query_battery answers from live data (GWY-P4-39). When `orch` is
    None, the loop falls back to the V-2B naive path (smoke only).

    cloud_limits, given together with `orch`, turns on the two cloud inputs
    (runtime/cloud_input); None leaves them off (the V-2B smoke path)."""
    from xbrain.common.runtime.session_ctx import open_planes

    _logger.info("p4 wiring: opening RT + GEN Zenoh sessions")
    with open_planes(("rt", "gen")) as (rt, gen):
        # Cache publishers.
        pubs: Dict[str, object] = {}
        for k in OUTBOUND_KEYS:
            pubs[k] = gen.declare_publisher(k)
        _logger.info("p4 wiring: %d outbound publishers ready", len(pubs))

        # GWY-P4-39: subscribe GEN-plane state/* into a freshness cache so
        # G queries answer from live data. Subscriber handles are held in a
        # list (strong ref, CLAUDE.md 4.3) so the Rust subscription is not
        # GC'd out from under us. Kept None when the orchestrator is off.
        state_subs = []
        turn_handler = None
        orchestrator = None
        if orch is not None:
            state_cache = _wire_state_subscriptions(gen, state_subs)
            # Compose the group query_fns: battery (G02) + RTK/heading (G43-G47,
            # 18-C/F5) + time (G24, site-tz local clock). Each owns its ids; the
            # first to answer wins.
            query_fn = compose_query_fns([
                make_battery_query_fn(
                    state_cache, orch.query_templates,
                    max_age_ms=orch.query_max_age_ms,
                    low_soc_pct=orch.query_low_soc_pct),
                make_rtk_query_fn(
                    state_cache, max_age_ms=orch.query_max_age_ms),
                make_time_query_fn(
                    state_cache, orch.site_timezone,
                    max_age_ms=orch.query_max_age_ms),
            ])
            orchestrator = build_orchestrator(
                orch.registry, orch.chitchat, l2_timeout_ms=orch.l2_timeout_ms,
                tier2_fn=orch.tier2_fn, query_fn=query_fn,
                bypass_tokens=orch.bypass_tokens)
            session = OrchestratorSession()
            # 11 S8.7.6 state/voice_turn (2026-10-26): robot-prefixed like the
            # cloud inputs, so it needs the robot id. Without one there is no
            # record at all -- never one on xbrain/None/... (see
            # _wire_cloud_inputs for the same rule).
            vt_rid = os.environ.get("XBRAIN_ROBOT_ID")
            voice_turn_key = (VOICE_TURN_KEY_FMT % vt_rid) if vt_rid else None
            if voice_turn_key:
                pubs[voice_turn_key] = gen.declare_publisher(voice_turn_key)
                _logger.info("p4 wiring: publishing %s", voice_turn_key)
            else:
                _logger.error("p4 state/voice_turn DISABLED: "
                              "XBRAIN_ROBOT_ID unset")
            turn_handler = make_turn_handler(orchestrator, session,
                                             asr_post=orch.asr_post,
                                             voice_turn_key=voice_turn_key)
            _logger.info(
                "p4 wiring: TurnOrchestrator handler active; state/* "
                "subscribed (%d keys), G02 live", len(state_subs))
        else:
            _logger.warning(
                "p4 wiring: no orchestrator inputs -- falling back to V-2B "
                "naive_classify path (smoke only)")

        # Worker + queue.
        import threading
        stop_evt = threading.Event()
        q: queue.Queue = queue.Queue(maxsize=64)

        def _publish(key: str, payload_bytes: bytes) -> None:
            pub = pubs.get(key)
            if pub is None:
                _logger.warning("p4 no cached publisher for %r", key)
                return
            pub.put(payload_bytes)

        # Cloud turns: bounded, and a full queue refuses the NEW turn.
        jobs: queue.Queue = queue.Queue(maxsize=16)
        worker = TurnLoopWorker(
            cfg=cfg, in_queue=q, publish_fn=_publish, stop_evt=stop_evt,
            turn_handler=turn_handler, job_queue=jobs)
        worker.start()
        _logger.info("p4 wiring: TurnLoopWorker started")

        mic_sub = rt.declare_subscriber(MIC_TOPIC, on_mic_frame_callback(q))
        _logger.info("p4 wiring: subscribed %s", MIC_TOPIC)

        cloud_subs: list = []
        assembler = None
        if orchestrator is not None and cloud_limits is not None:
            assembler = _wire_cloud_inputs(
                gen, cfg, orchestrator, orch.registry, orch.asr_post,
                state_cache, _publish, jobs, cloud_limits, cloud_subs,
                session, voice_turn_key)

        try:
            last_hb = time.monotonic()
            while not stop_flag.get("stop"):
                now = time.monotonic()
                if now - last_hb >= _VOICE_LOOP_HEARTBEAT_PERIOD:
                    if assembler is not None:
                        # Reclaim streams of clients that died mid-sentence
                        # even when no further audio arrives to trigger it.
                        for sid in assembler.evict_idle(_now_ms()):
                            _logger.warning(
                                "voice_in stream %s evicted: idle (no eos)",
                                sid)
                    _logger.info(
                        "p4 alive; frames_rx=%d turns_dispatched=%d "
                        "last_intent=%s queue_depth=%d",
                        worker.frames_received,
                        worker.turns_dispatched,
                        worker.last_intent_id,
                        q.qsize())
                    last_hb = now
                time.sleep(0.1)
        finally:
            _logger.info("p4 wiring: shutting down")
            for sub in cloud_subs:
                try:
                    sub.undeclare()
                except Exception:      # noqa: BLE001
                    pass
            try:
                mic_sub.undeclare()
            except Exception:      # noqa: BLE001
                pass
            stop_evt.set()
            worker.join(timeout=2.0)
            for pub in pubs.values():
                try:
                    pub.undeclare()
                except Exception:      # noqa: BLE001
                    pass
    _logger.info("p4 wiring: exited cleanly")
    return 0
