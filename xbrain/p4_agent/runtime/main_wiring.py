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

from xbrain.p4_agent.runtime.intent_dispatch import (
    CMD_AUDIO_SPEAK, CMD_MOTION_INTENT, CMD_PAYLOAD, CMD_PTZ, CMD_TASK,
)
from xbrain.p4_agent.runtime.turn_loop import (
    MIC_TOPIC, TurnLoopConfig, TurnLoopWorker,
    on_mic_frame_callback,
)


_logger = logging.getLogger("xbrain.p4.wiring")


OUTBOUND_KEYS = (CMD_AUDIO_SPEAK, CMD_TASK, CMD_MOTION_INTENT,
                  CMD_PTZ, CMD_PAYLOAD)


def run_voice_loop_wiring(cfg: TurnLoopConfig,
                            stop_flag: dict,
                            heartbeat_period_s: float = 5.0) -> int:
    """Block until stop_flag['stop'] becomes truthy. Returns 0 on
    clean shutdown."""
    from xbrain.common.runtime.session_ctx import open_planes

    _logger.info("p4 wiring: opening RT + GEN Zenoh sessions")
    with open_planes(("rt", "gen")) as (rt, gen):
        # Cache publishers.
        pubs: Dict[str, object] = {}
        for k in OUTBOUND_KEYS:
            pubs[k] = gen.declare_publisher(k)
        _logger.info("p4 wiring: %d outbound publishers ready", len(pubs))

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
            cfg=cfg, in_queue=q, publish_fn=_publish, stop_evt=stop_evt)
        worker.start()
        _logger.info("p4 wiring: TurnLoopWorker started")

        mic_sub = rt.declare_subscriber(MIC_TOPIC, on_mic_frame_callback(q))
        _logger.info("p4 wiring: subscribed %s", MIC_TOPIC)

        try:
            last_hb = time.monotonic()
            while not stop_flag.get("stop"):
                now = time.monotonic()
                if now - last_hb >= heartbeat_period_s:
                    _logger.info(
                        "p4 alive; turns_dispatched=%d last_intent=%s "
                        "queue_depth=%d",
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
