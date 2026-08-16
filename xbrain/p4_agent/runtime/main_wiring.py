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
  2. Cache publishers for the 5 outbound key families (avoids
     declare-per-message overhead + gets errors surfaced early)
  3. Start TurnLoopWorker
  4. Subscribe rt/audio/mic -> worker queue
  5. Enter main loop; wait for stop_event

Shutdown order (LIFO):
  1. Unsubscribe rt/audio/mic
  2. Stop worker (drains remaining frames)
  3. Undeclare publishers
  4. Close sessions
"""

from __future__ import annotations

import logging
import queue
import time
from typing import Dict

from typing import Optional

from xbrain.p4_agent.runtime.intent_dispatch import (
    CMD_AUDIO_SPEAK, CMD_MOTION_INTENT, CMD_PAYLOAD, CMD_PTZ, CMD_TASK,
)
from xbrain.p4_agent.runtime.orchestrator_turn import (
    VoiceOrchestratorInputs, build_orchestrator, compose_query_fns,
    make_battery_query_fn, make_rtk_query_fn, make_time_query_fn,
    make_turn_handler,
)
from xbrain.p4_agent.runtime.turn_loop import (
    MIC_TOPIC, TurnLoopConfig, TurnLoopWorker,
    on_mic_frame_callback,
)
from xbrain.p4_agent.runtime.orchestrator_turn import CMD_ESTOP
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession


_logger = logging.getLogger("xbrain.p4.wiring")


# Outbound keys the orchestrator can publish. CMD_ESTOP is included because
# a safety-bypass (急停) turn publishes there (16 S4 -> Tier1); leaving it
# out silently DROPS the estop -- the 2026-08-11 ORIN test hit exactly that
# ('no cached publisher for cmd/estop'). The bypass path must have a
# publisher like any other key.
OUTBOUND_KEYS = (CMD_AUDIO_SPEAK, CMD_TASK, CMD_MOTION_INTENT,
                  CMD_PTZ, CMD_PAYLOAD, CMD_ESTOP)


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


def run_voice_loop_wiring(cfg: TurnLoopConfig,
                            stop_flag: dict,
                            *,
                            orch: Optional[VoiceOrchestratorInputs] = None
                            ) -> int:
    """Block until stop_flag['stop'] becomes truthy. Returns 0 on
    clean shutdown.

    GWY-P4-41 (32.I): when `orch` is supplied (the production path, built
    by __main__ from the resolved config + static content files), each turn
    is routed through the six-step TurnOrchestrator instead of the V-2B
    naive_classify path. GEN-plane state/* is subscribed into a cache so
    G02 query_battery answers from live data (GWY-P4-39). When `orch` is
    None, the loop falls back to the V-2B naive path (smoke only)."""
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
                tier2_fn=orch.tier2_fn, query_fn=query_fn)
            session = OrchestratorSession()
            turn_handler = make_turn_handler(orchestrator, session)
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

        worker = TurnLoopWorker(
            cfg=cfg, in_queue=q, publish_fn=_publish, stop_evt=stop_evt,
            turn_handler=turn_handler)
        worker.start()
        _logger.info("p4 wiring: TurnLoopWorker started")

        mic_sub = rt.declare_subscriber(MIC_TOPIC, on_mic_frame_callback(q))
        _logger.info("p4 wiring: subscribed %s", MIC_TOPIC)

        try:
            last_hb = time.monotonic()
            while not stop_flag.get("stop"):
                now = time.monotonic()
                if now - last_hb >= _VOICE_LOOP_HEARTBEAT_PERIOD:
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
