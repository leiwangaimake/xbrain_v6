"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: main_wiring.py
Brief: p2_core __main__ wire-up: RT+GEN Zenoh sessions + mic + speaker + shutdown

Description:
Called from xbrain/p2_core/__main__.py after config load. Opens the
two Zenoh sessions, launches the MIC capture threads, wires the
speaker subscriber, blocks on a stop_event.

Startup order:
  1. Open RT session (for rt/audio/mic + rt/audio/gate)
  2. Open GEN session (for cmd/audio/speak subscribe)
  3. Instantiate SpeakerDomain (registers rt/audio/gate publisher)
  4. Start MIC capture + publisher threads
  5. Subscribe cmd/audio/speak
  6. Enter main loop; wait for stop_event

Shutdown order (LIFO):
  1. Stop MIC threads (arecord SIGTERM)
  2. Unsubscribe cmd/audio/speak
  3. SpeakerDomain.shutdown() publishes gate=open one last time
  4. Close GEN session
  5. Close RT session

For MVP the speaker + gate + mic are the only runtime wiring; the
seven arbiter domains (motion / speaker / asr / payload_light / ptz
/ gpu / dock) will grow on top of this scaffold in future batches.
"""

from __future__ import annotations

import json
import logging
import os
import time
from datetime import datetime, timezone
from typing import Optional

from xbrain.p2_core.runtime.mic_capture import (
    MicCaptureConfig, spawn_mic_pipeline,
)
from xbrain.p2_core.runtime.speaker_wiring import (
    SPEAK_TOPIC, SpeakerBusy, SpeakerDomain, SpeakerHwError,
    SpeakerWiringConfig, parse_speak_payload,
)


_logger = logging.getLogger("xbrain.p2.wiring")


def _now_mono_ms() -> int:
    return int(time.monotonic() * 1000)


def run_voice_loop_wiring(mic_cfg: MicCaptureConfig,
                            spk_cfg: SpeakerWiringConfig,
                            stop_flag: dict,
                            heartbeat_period_s: float = 5.0) -> int:
    """Block until stop_flag['stop'] becomes truthy.
    Returns 0 on clean shutdown."""
    from xbrain.common.runtime.session_ctx import open_planes

    _logger.info("p2 wiring: opening RT + GEN Zenoh sessions")
    with open_planes(("rt", "gen")) as (rt, gen):
        # MIC pipeline (RT plane) -- constructed FIRST so SpeakerDomain
        # can hold a reference to mic_pub_thread for the half-duplex
        # mute() / unmute() calls around TTS playback (V-HALFDUPLEX-1,
        # 2026-08-11). Without this, TTS audio played through the
        # GZH-2 speaker re-entered the USB MIC, ASR transcribed it,
        # and the same intent looped forever every ~2s.
        mic_thread, mic_pub_thread, mic_stop = spawn_mic_pipeline(
            mic_cfg, zenoh_session=rt)
        _logger.info("p2 wiring: MIC capture started (dev=%s topic=%s)",
                     mic_cfg.arecord_device, mic_cfg.zenoh_topic)

        # Speaker domain -- publishes rt/audio/gate on RT plane AND
        # holds the mic-publisher handle for half-duplex mute.
        speaker = SpeakerDomain(cfg=spk_cfg, rt_session=rt,
                                  now_mono_ms_fn=_now_mono_ms,
                                  mic_publisher=mic_pub_thread)

        # GEN plane subscribers. All handles are held in one list so the
        # Rust-side subscriptions are not GC'd out from under us (CLAUDE.md
        # 4.3: a dropped declare_subscriber handle silently unsubscribes).
        _gen_subs: list = []

        def _on_speak(sample) -> None:
            """Zenoh subscriber callback. RUNS ON RUST THREAD --
            must not do async / long blocking directly (CLAUDE.md
            4.2). We hand off via speaker.handle_speak which
            manages its own lock + blocking."""
            try:
                text = parse_speak_payload(bytes(sample.payload))
            except Exception as exc:      # noqa: BLE001
                _logger.warning("cmd/audio/speak parse fail: %s", exc)
                return
            # Best-effort handoff to a worker so the Rust callback
            # thread returns quickly.
            import threading
            threading.Thread(
                target=lambda: _speak_and_log(speaker, text),
                name="p2.speak_handler", daemon=True).start()

        _gen_subs.append(gen.declare_subscriber(SPEAK_TOPIC, _on_speak))
        _logger.info("p2 wiring: subscribed %s", SPEAK_TOPIC)

        # -- cmd/payload subscriber (2026-08-11 V-STROBE-1) ------------
        # p4 dispatch routes D01-D07/D11/D17/D18 (lamp/siren) here;
        # PayloadDomain translates the envelope into a payload-service
        # /lights HTTP call. GEN plane like speak. Same Rust-thread
        # handoff pattern to avoid blocking Zenoh.
        from xbrain.p2_core.runtime.payload_wiring import (
            CMD_PAYLOAD_TOPIC, PayloadDomain, PayloadWiringConfig,
        )
        payload_cfg = PayloadWiringConfig(
            payload_base_url=spk_cfg.payload_base_url,
            http_timeout_s=spk_cfg.tts_http_timeout_s)
        payload = PayloadDomain(cfg=payload_cfg)

        def _on_payload(sample) -> None:
            import threading as _t
            data = bytes(sample.payload)
            _t.Thread(
                target=lambda: payload.handle_envelope(data),
                name="p2.payload_handler", daemon=True).start()

        _gen_subs.append(gen.declare_subscriber(CMD_PAYLOAD_TOPIC, _on_payload))
        _logger.info("p2 wiring: subscribed %s", CMD_PAYLOAD_TOPIC)

        # -- cmd/ptz subscriber (2026-08-11 PTZ audit) ----------------
        # p4 dispatch routes the E-class PTZ intents here; PtzDomain drives
        # the 布控球 via ONVIF ContinuousMove/Stop. Before this cmd/ptz had
        # NO consumer. Camera credentials come from onvif_credentials.json;
        # a missing file just disables PTZ (audio/payload keep running).
        # Sub handle held in _gen_subs (strong ref, CLAUDE.md 4.3).
        from xbrain.p2_core.runtime.ptz_wiring import (
            CMD_PTZ_TOPIC, PtzDomain, load_onvif_config,
        )
        ptz_domain = None
        # CONFIG-SOURCE-OK(secrets): a credentials file, not a config value on the
        # L0-L6 axis; configs/secrets/ is never materialised to resolved/.
        onvif_secrets = "/opt/xbrain_v6/configs/secrets/onvif_credentials.json"
        onvif_cfg = load_onvif_config(onvif_secrets)
        if onvif_cfg is not None:
            ptz_domain = PtzDomain(onvif_cfg)

            def _on_ptz(sample) -> None:
                import threading as _t
                data = bytes(sample.payload)
                _t.Thread(
                    target=lambda: ptz_domain.handle_envelope(data),
                    name="p2.ptz_handler", daemon=True).start()

            _gen_subs.append(gen.declare_subscriber(CMD_PTZ_TOPIC, _on_ptz))
            _logger.info("p2 wiring: subscribed %s (PTZ active)", CMD_PTZ_TOPIC)
        else:
            _logger.warning("p2 wiring: PTZ disabled (no onvif credentials)")

        # Device liveness -> 11 S6.2 device_offline/online events (SW-12 producers).
        # p5 persists + backfills these. MIC is REAL today (arecord thread
        # alive/dead below); payload + ptz are REGISTERED but fed 'unknown' (None)
        # each tick until their clients expose a reachability check -- unknown
        # emits nothing (never a false online), so wiring them now is a safe seam
        # (their real detection is GATED-HW).
        _dev_evt_seq = [0]

        def _emit_device_event(ev: dict) -> None:
            # Publish on the RELATIVE event/{sev}/{cat} key (bus convention); p5's
            # event/** subscriber picks it up, derives channel, persists.
            key = "event/%s/%s" % (ev["sev"], ev["cat"])
            gen.put(key, json.dumps({
                "eid": ev["eid"], "title": ev["title"], "detail": ev["detail"],
                "src": ev["src"], "ts": ev["ts"]}).encode("utf-8"))

        def _dev_eid(dev: str, offline: bool) -> str:
            _dev_evt_seq[0] += 1
            return "dev-%s-%s-%d" % (dev, "off" if offline else "on",
                                     _dev_evt_seq[0])

        from xbrain.p2_core.runtime.device_health_bridge import DeviceHealthBridge
        device_bridge = DeviceHealthBridge(
            rid=os.environ.get("XBRAIN_ROBOT_ID", "unknown"),
            emit=_emit_device_event,
            now_iso=lambda: datetime.now(timezone.utc).isoformat(),
            eid_gen=_dev_eid)
        for _d in ("mic", "ptz", "payload_speaker", "payload_siren",
                   "payload_strobe", "payload_light"):
            device_bridge.register(_d)
        _logger.info("p2 wiring: device health bridge ON (mic real; "
                     "payload/ptz seamed, GATED-HW)")

        # Main loop: wait for stop.
        try:
            last_hb = time.monotonic()
            while not stop_flag.get("stop"):
                now = time.monotonic()
                if now - last_hb >= heartbeat_period_s:
                    # Rich heartbeat: capture/publisher alive flags,
                    # capture/publish counters, and the bug-net
                    # exception messages if either thread died. Without
                    # this a threading exception vanishes into
                    # threading._threading_default_excepthook and the
                    # log shows only mic_frames=0 with no cause.
                    cap_alive = mic_thread.is_alive()
                    pub_alive = mic_pub_thread.is_alive()
                    cap_exc = getattr(mic_thread, "last_exception", None)
                    pub_exc = getattr(mic_pub_thread, "last_exception", None)
                    pub_errs = list(getattr(mic_pub_thread, "errors", ()))
                    _logger.info(
                        "p2 alive; captured=%d published=%d muted=%d "
                        "cap_alive=%s pub_alive=%s errors=%s",
                        getattr(mic_thread, "frames_captured", 0),
                        mic_pub_thread.frames_published,
                        getattr(mic_pub_thread, "frames_muted", 0),
                        cap_alive, pub_alive,
                        pub_errs[-1] if pub_errs else "none")
                    if cap_exc:
                        _logger.error("mic capture thread crashed:\n%s",
                                      cap_exc)
                    if pub_exc:
                        _logger.error("mic publisher thread crashed:\n%s",
                                      pub_exc)
                    # SW-12 device liveness. MIC: alive iff both arecord threads
                    # live (a dropped USB MIC kills them -> device_offline after the
                    # debounce). payload: poll payload-service GET /status for the
                    # 8519/8529 GZH-2 socket state (audio -> speaker/siren, lights ->
                    # strobe/light); None -> unknown, no false offline. ptz: fed None
                    # for now -- an ONVIF ping would block this loop, so it needs a
                    # non-blocking probe thread (GATED-HW, seam ready).
                    device_bridge.observe("mic", bool(cap_alive and pub_alive))
                    device_bridge.observe("ptz", None)
                    _ps = payload.device_status()
                    _audio = _ps["audio"] if _ps else None
                    _lights = _ps["lights"] if _ps else None
                    device_bridge.observe("payload_speaker", _audio)
                    device_bridge.observe("payload_siren", _audio)
                    device_bridge.observe("payload_strobe", _lights)
                    device_bridge.observe("payload_light", _lights)
                    last_hb = now
                time.sleep(0.1)
        finally:
            _logger.info("p2 wiring: shutting down")
            try:
                mic_stop.set()
                mic_thread.stop()
                mic_thread.join(timeout=2.0)
                mic_pub_thread.join(timeout=2.0)
            except Exception:      # noqa: BLE001
                pass
            try:
                speak_sub.undeclare()
            except Exception:      # noqa: BLE001
                pass
            try:
                speaker.shutdown()
            except Exception:      # noqa: BLE001
                pass
    _logger.info("p2 wiring: exited cleanly")
    return 0


def _speak_and_log(speaker: SpeakerDomain, text: str) -> None:
    try:
        ack = speaker.handle_speak(text)
    except SpeakerBusy:
        _logger.info("p2 speaker busy; skipping request")
        return
    except SpeakerHwError as exc:
        _logger.warning("p2 speaker HW error: %s", exc)
        return
    _logger.info("p2 speaker ack: %s", ack)
