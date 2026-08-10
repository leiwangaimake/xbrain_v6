"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: main_wiring.py
Brief: p3_task voice-loop MVP wiring -- subscribe cmd/task + log observed

Description:
Minimum-viable p3 for the voice-loop smoke test:

  * open GEN Zenoh session
  * subscribe cmd/task -- log each admitted task (intent_id + text)
  * publish state/task heartbeat frame every 5 s so p5 sees life

Task queue + task.db + docking SM live in xbrain/p3_task/{state,
schedule,charge,...} and stay untouched by this MVP -- wiring them
into a live queue is a later batch. The purpose of this file is
the loop: 'when p4 says patrol, p3 hears it AND signals ack'.
"""

from __future__ import annotations

import json
import logging
import time
from typing import Optional


_logger = logging.getLogger("xbrain.p3.wiring")


CMD_TASK_TOPIC = "cmd/task"
STATE_TASK_TOPIC = "state/task"


def _now_mono_ms() -> int:
    return int(time.monotonic() * 1000)


def run_voice_loop_wiring(stop_flag: dict,
                            heartbeat_period_s: float = 5.0) -> int:
    """Block until stop_flag['stop'] is truthy. Returns 0 on
    clean shutdown."""
    from xbrain.common.runtime.session_ctx import open_planes

    _logger.info("p3 wiring: opening GEN session")
    with open_planes(("gen",)) as gen:
        state_pub = gen.declare_publisher(STATE_TASK_TOPIC)
        # Announce init state.
        state_pub.put(json.dumps({
            "schema": "state_task_v1",
            "active_task": None,
            "mono_ms": _now_mono_ms(),
        }).encode("utf-8"))

        tasks_seen = 0
        last_admitted_intent: Optional[str] = None

        def _on_task(sample) -> None:
            nonlocal tasks_seen, last_admitted_intent
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                _logger.warning("p3 malformed cmd/task payload")
                return
            intent = d.get("intent_id", "?")
            text = d.get("text", "")
            last_admitted_intent = intent
            tasks_seen += 1
            _logger.info(
                "p3 admit cmd/task intent=%s text=%r (seen=%d)",
                intent, text, tasks_seen)
            # Reflect into state/task so p5 can see it.
            state_pub.put(json.dumps({
                "schema": "state_task_v1",
                "active_task": {
                    "intent_id": intent,
                    "text": text,
                    "admitted_mono_ms": _now_mono_ms(),
                },
            }).encode("utf-8"))

        task_sub = gen.declare_subscriber(CMD_TASK_TOPIC, _on_task)
        _logger.info("p3 wiring: subscribed %s", CMD_TASK_TOPIC)

        try:
            last_hb = time.monotonic()
            while not stop_flag.get("stop"):
                now = time.monotonic()
                if now - last_hb >= heartbeat_period_s:
                    _logger.info("p3 alive; tasks_seen=%d last_intent=%s",
                                 tasks_seen, last_admitted_intent)
                    last_hb = now
                time.sleep(0.1)
        finally:
            try:
                task_sub.undeclare()
            except Exception:      # noqa: BLE001
                pass
            try:
                state_pub.undeclare()
            except Exception:      # noqa: BLE001
                pass
    return 0
