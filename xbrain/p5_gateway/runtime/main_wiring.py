"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: main_wiring.py
Brief: p5_gateway voice-loop MVP wiring -- state/link publisher + event drain

Description:
Minimum-viable p5 for the voice-loop smoke test:

  * open GEN session
  * publish state/link (P5 is the UNIQUE publisher, 11 §7.1A)
    every 1 s -- lets HMI + Qt see 'gateway alive'
  * subscribe cmd/audio/speak/ack + state/task and log
  * subscribe event/{severity}/{category} and log

Full event pipeline (schema check + dedupe + record.db + cloud
uplink) lives in xbrain/p5_gateway/event/ and stays untouched by
this MVP. The purpose here is: 'gateway is alive AND observes the
downstream ACKs'.
"""

from __future__ import annotations

import json
import logging
import time
from typing import Optional


_logger = logging.getLogger("xbrain.p5.wiring")


STATE_LINK_TOPIC = "state/link"
CMD_AUDIO_SPEAK_ACK_TOPIC = "cmd/audio/speak/ack"
STATE_TASK_TOPIC = "state/task"


def _now_mono_ms() -> int:
    return int(time.monotonic() * 1000)


def run_voice_loop_wiring(stop_flag: dict,
                            heartbeat_period_s: float = 1.0) -> int:
    """Block until stop_flag['stop'] truthy. Returns 0 on clean shutdown."""
    from xbrain.common.runtime.session_ctx import open_planes

    _logger.info("p5 wiring: opening GEN session")
    with open_planes(("gen",)) as gen:
        link_pub = gen.declare_publisher(STATE_LINK_TOPIC)

        speak_acks_seen = 0
        state_task_updates = 0

        def _on_speak_ack(sample) -> None:
            nonlocal speak_acks_seen
            speak_acks_seen += 1
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                d = {}
            _logger.info("p5 obs speak/ack #%d: %s",
                         speak_acks_seen,
                         json.dumps(d, ensure_ascii=False))

        def _on_state_task(sample) -> None:
            nonlocal state_task_updates
            state_task_updates += 1
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                d = {}
            _logger.info("p5 obs state/task update #%d: %s",
                         state_task_updates,
                         json.dumps(d, ensure_ascii=False))

        ack_sub = gen.declare_subscriber(
            CMD_AUDIO_SPEAK_ACK_TOPIC, _on_speak_ack)
        task_sub = gen.declare_subscriber(
            STATE_TASK_TOPIC, _on_state_task)
        _logger.info("p5 wiring: subscribed speak/ack + state/task")

        try:
            last_hb = time.monotonic()
            while not stop_flag.get("stop"):
                now = time.monotonic()
                if now - last_hb >= heartbeat_period_s:
                    link_pub.put(json.dumps({
                        "schema": "state_link_v1",
                        "gateway_up": True,
                        "mono_ms": _now_mono_ms(),
                        "speak_acks": speak_acks_seen,
                        "task_updates": state_task_updates,
                    }).encode("utf-8"))
                    last_hb = now
                time.sleep(0.1)
        finally:
            try:
                ack_sub.undeclare()
            except Exception:      # noqa: BLE001
                pass
            try:
                task_sub.undeclare()
            except Exception:      # noqa: BLE001
                pass
            try:
                link_pub.undeclare()
            except Exception:      # noqa: BLE001
                pass
    return 0
