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


CMD_ESTOP_TOPIC = "cmd/estop"


def _start_hmi(gen, hmi_cfg: dict, hmi_state: dict):
    """Wire + start the HMI web server against what P5 can serve TODAY.

    Returns (server, thread) or (None, None) when HMI is not configured / cannot
    start -- an HMI failure must NEVER take down the voice loop, so every error
    here is logged and swallowed (the gateway must stay up so the operator can
    still see the voice side, 10 S3.3.7 W-1). What is wired now: state/task ->
    plan panel, state/link -> status/ESTOP arming, and the ESTOP button ->
    cmd/estop. What is NOT (fences/events/pose/mode) is recorded in NEXT.md and
    surfaces as available:false so the frontend greys those layers, never fakes.
    """
    from xbrain.p5_gateway.hmi.web_server import (
        HmiBindError, build_app, make_bound_sockets, start_in_thread,
    )

    bind = hmi_cfg.get("bind") if isinstance(hmi_cfg, dict) else None
    web = hmi_cfg.get("web") if isinstance(hmi_cfg, dict) else None
    if not bind or not web:
        _logger.warning("p5 HMI: no hmi.bind/hmi.web config; HMI not started")
        return None, None

    # ESTOP button -> W1 (17 S6.2). MVP sends the frame on cmd/estop; the
    # dedicated <=10 ms fast path (17 S6.4 / P-1) is a follow-up (NEXT.md).
    estop_pub = gen.declare_publisher(CMD_ESTOP_TOPIC)

    def _estop_sender() -> None:
        estop_pub.put(json.dumps({"type": "estop", "action": "stop"})
                      .encode("utf-8"))
        _logger.warning("p5 HMI ESTOP pressed -> cmd/estop published")

    class _Provider:
        """Reads P5's live shared state (updated by the sync callbacks) into the
        snapshot kwargs. Only the wired sources are non-None; the rest default to
        None so data_readers reports them unavailable (17 S6.10.4)."""

        def snapshot_inputs(self):
            # Copy references under the GIL; the callback replaces whole values,
            # never mutates in place, so a torn read is not possible here.
            return {
                "tasks": hmi_state.get("tasks"),   # from state/task (wired)
                "link": hmi_state.get("link"),     # from state/link (wired)
                # NOT wired yet -> None -> frontend greys (NEXT.md HMI-W1..W4):
                "fences": None, "routes": None, "waypoints": None,
                "enu_origin": None, "pose": None, "mode": None,
                "health": None, "events": None,
            }

        def fence_degraded(self):
            # Fence cache is not subscribed in the MVP wiring, so /api/fences
            # cannot answer authoritatively -> 503 E_DEGRADED (P5F-2), the honest
            # code, never a 200 empty set.
            return True

    try:
        socks = make_bound_sockets(bind)
        import os                                    # noqa: PLC0415
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        static_root = os.path.join(here, web.get("static_dir", "hmi/static"))
        app = build_app(web, _Provider(), _estop_sender, static_root)
        server, thread = start_in_thread(app, socks)
        _logger.info("p5 HMI: serving on %s (static %s)",
                     [e for e in bind if e], static_root)
        return server, thread
    except HmiBindError as exc:
        # A bind failure (e.g. all-null or a wildcard) must not crash the voice
        # loop; the HMI just does not come up and the reason is logged.
        _logger.error("p5 HMI: bind refused (%s); HMI not started", exc)
        return None, None
    except Exception as exc:      # noqa: BLE001
        _logger.error("p5 HMI: failed to start (%s: %s); voice loop continues",
                     type(exc).__name__, exc)
        return None, None


def run_voice_loop_wiring(stop_flag: dict,
                            heartbeat_period_s: float = 1.0,
                            hmi_cfg: Optional[dict] = None) -> int:
    """Block until stop_flag['stop'] truthy. Returns 0 on clean shutdown.

    hmi_cfg is the resolved `hmi` config subtree (bind + web) or None. When
    present the HMI web server starts in a background thread (17 S6.10); when
    absent or malformed the voice loop runs exactly as before -- the HMI is
    strictly additive and never a precondition for the voice side.
    """
    from xbrain.common.runtime.session_ctx import open_planes

    # Shared state the HMI provider reads. The sync callbacks below REPLACE whole
    # values (never mutate in place) so the web thread's reads are consistent
    # under the GIL without a lock.
    hmi_state: dict = {"tasks": None, "link": None}

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
            # Feed the HMI plan panel. state/task carries the current task view;
            # wrap it as a one-element list (the panel renders a list of plans),
            # or None when the payload had nothing usable so the panel stays
            # "no plan" rather than showing an empty card.
            hmi_state["tasks"] = [d] if d else None
            _logger.info("p5 obs state/task update #%d: %s",
                         state_task_updates,
                         json.dumps(d, ensure_ascii=False))

        ack_sub = gen.declare_subscriber(
            CMD_AUDIO_SPEAK_ACK_TOPIC, _on_speak_ack)
        task_sub = gen.declare_subscriber(
            STATE_TASK_TOPIC, _on_state_task)
        _logger.info("p5 wiring: subscribed speak/ack + state/task")

        # Start the HMI web server (best-effort; never blocks the voice loop).
        hmi_server, _hmi_thread = (None, None)
        if hmi_cfg:
            hmi_server, _hmi_thread = _start_hmi(gen, hmi_cfg, hmi_state)

        try:
            last_hb = time.monotonic()
            while not stop_flag.get("stop"):
                now = time.monotonic()
                if now - last_hb >= heartbeat_period_s:
                    link_payload = {
                        "schema": "state_link_v1",
                        "gateway_up": True,
                        # estop_path lets the HMI arm/grey its ESTOP button
                        # (NAV-64). MVP reports "ok" whenever the gateway loop is
                        # alive; the real end-to-end estop probe (17 S6.3) is a
                        # follow-up (NEXT.md HMI-W5).
                        "estop_path": "ok",
                        "mono_ms": _now_mono_ms(),
                        "speak_acks": speak_acks_seen,
                        "task_updates": state_task_updates,
                    }
                    hmi_state["link"] = link_payload   # feed HMI status/ESTOP
                    link_pub.put(json.dumps(link_payload).encode("utf-8"))
                    last_hb = now
                time.sleep(0.1)
        finally:
            # Stop the HMI first so it stops reading shared state, then tear down
            # the zenoh entities. should_exit is uvicorn's clean-stop flag.
            if hmi_server is not None:
                hmi_server.should_exit = True
            for entity in (ack_sub, task_sub, link_pub):
                try:
                    entity.undeclare()
                except Exception:      # noqa: BLE001
                    pass
    return 0
