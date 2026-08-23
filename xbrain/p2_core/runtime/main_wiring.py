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
import queue
import logging
import os
import time
from datetime import datetime, timezone
from typing import Optional

from xbrain.p2_core.runtime.mic_capture import (
    MicCaptureConfig, spawn_mic_pipeline,
)
from xbrain.p2_core.health.aggregate import HealthAggregator, refresh_health
from xbrain.p2_core.health.factor import FactorConfig
from xbrain.p2_core.runtime.speaker_wiring import (
    SPEAK_TOPIC, SpeakerBusy, SpeakerDomain, SpeakerHwError,
    SpeakerWiringConfig, parse_speak_payload,
)


_logger = logging.getLogger("xbrain.p2.wiring")

# 11 S2.2: P2 is the publisher of health/summary; the five keys below are the
# sources it derives items from. All GEN-plane, all relative keys (bus
# convention -- the rid prefix is the session's, not the caller's).
HEALTH_SUMMARY_TOPIC = "health/summary"
HEALTH_PUBLISH_PERIOD_S = 1.0            # 11 S2.2 / 14 S2.3 P-2: 1 Hz stable
STATE_POSE_TOPIC = "state/pose"          # -> rtk + heading (11 S3.2 / S3.3)
STATE_CLOCK_TOPIC = "state/clock"        # -> clock (CLK-A2 mirror)
STATE_ROBOT_TOPIC = "state/robot"        # -> chassis (chassis_relay, CR-4)
STATE_POWER_TOPIC = "state/power"        # -> battery (chassis_relay, CR-5)
STATE_LINK_TOPIC = "state/link"          # -> network (11 S4.6)


def _factor_cfg() -> FactorConfig:
    """The 14 S8.2 step-1 factor table from the resolved p2_core config.

    Read through the config layer rather than defaulted here: these four values
    decide how much a degraded item slows the robot, and a default in code
    would keep the machine moving at a speed nobody configured. A missing key
    raises, which is what the startup assertions are for.
    """
    from xbrain.common.config.resolved import load_resolved

    cfg = load_resolved("p2_core")
    return FactorConfig(
        fatal_degraded=cfg.get("health.factors.fatal_degraded"),
        degraded_fail=cfg.get("health.factors.degraded_fail"),
        degraded_degraded=cfg.get("health.factors.degraded_degraded"),
        unknown=cfg.get("health.factors.unknown"))


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

        # 11 S2.2 / S5.1: P2 is the publisher of health/summary at 1 Hz. The
        # four state sources it derives items from are subscribed here; each
        # callback only stores the decoded body (RUST THREAD, CLAUDE.md 4.2)
        # and the loop below does the derivation and the publish.
        health_pub = gen.declare_publisher(HEALTH_SUMMARY_TOPIC)
        health_agg = HealthAggregator()
        # Read ONCE at startup, not per tick: the resolved snapshot does not
        # change while the process runs (a config change goes through the
        # freeze line and a restart), and re-reading it at 1 Hz would put a
        # file read plus a sha256 verification in the publish path.
        factor_cfg = _factor_cfg()
        state_cache: dict = {}

        def _make_state_sink(name: str):
            def _sink(sample) -> None:
                try:
                    body = json.loads(bytes(sample.payload).decode("utf-8"))
                except Exception:      # noqa: BLE001
                    return
                # p1 publishes state/pose and state/clock enveloped as
                # {..., data:{...}}; the bare form is accepted too so a stub
                # publisher (or a future producer that does not envelope)
                # works without a second code path.
                data = body.get("data") if isinstance(body, dict) else None
                state_cache[name] = data if isinstance(data, dict) else body
            return _sink

        for _topic, _name in ((STATE_POSE_TOPIC, "pose"),
                              (STATE_CLOCK_TOPIC, "clock"),
                              (STATE_ROBOT_TOPIC, "robot"),
                              (STATE_POWER_TOPIC, "power"),
                              (STATE_LINK_TOPIC, "link")):
            _gen_subs.append(gen.declare_subscriber(_topic,
                                                    _make_state_sink(_name)))
        _logger.info("p2 wiring: health sources subscribed, publishing %s",
                     HEALTH_SUMMARY_TOPIC)

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
            CMD_PTZ_TOPIC, PtzDomain, PtzLivenessProbe, load_onvif_config,
            make_onvif_reachability_check,
        )
        ptz_domain = None
        ptz_probe = None
        # CONFIG-SOURCE-OK(secrets): a credentials file, not a config value on the
        # L0-L6 axis; configs/secrets/ is never materialised to resolved/.
        onvif_secrets = "/opt/xbrain_v6/configs/secrets/onvif_credentials.json"
        onvif_cfg = load_onvif_config(onvif_secrets)
        if onvif_cfg is not None:
            ptz_domain = PtzDomain(onvif_cfg)
            # SW-12: non-blocking ONVIF reachability probe on its own thread +
            # session, so the heartbeat reads a cached verdict instead of blocking
            # on the round-trip. Feeds the device bridge for ptz device_offline.
            ptz_probe = PtzLivenessProbe(make_onvif_reachability_check(onvif_cfg))
            ptz_probe.start()

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

        # -- cmd/mode subscriber (2026-08-21) -------------------------
        # 11 S2.2.3 lists p2_core as THIS key's subscriber, and it had none:
        # every 18-class-C intent (enter/exit alarm, enter/exit broadcast,
        # patrol mode, standby, set behaviour) was routed by p4 to cmd/task,
        # where P3 skipped it for having no top-level action. Eight voice
        # commands that reached nobody, with no error on either side.
        #
        # Frames are QUEUED here and applied on the main loop below, NOT in
        # this callback: ModeStateMachine documents itself as "not thread-safe
        # by design (main-thread only)", and it holds the cmd_id idempotency
        # history -- two Rust threads racing it would replay or double-apply.
        # That is a different discipline from payload/ptz above, which hand off
        # to a worker thread because their domains are I/O and stateless.
        from xbrain.p2_core.runtime.mode_wiring import (
            CMD_MODE_ACK_TOPIC, CMD_MODE_TOPIC, STATE_MODE_TOPIC, ModeFace,
        )
        mode_queue: "queue.Queue" = queue.Queue(maxsize=64)
        mode_ack_pub = gen.declare_publisher(CMD_MODE_ACK_TOPIC)
        mode_state_pub = gen.declare_publisher(STATE_MODE_TOPIC)

        def _publish_mode_state(key: str, data: bytes) -> None:
            mode_state_pub.put(data)

        mode_face = ModeFace(publish=_publish_mode_state)

        def _on_mode(sample) -> None:
            # RUST THREAD: copy the bytes and hand off. Nothing else.
            try:
                mode_queue.put_nowait(bytes(sample.payload))
            except queue.Full:
                # Dropped LOUDLY. A silently dropped mode command is
                # indistinguishable to the sender from one that was applied,
                # and mode decides mic gating and broadcast.
                _logger.error("p2 cmd/mode queue full; frame dropped")

        _gen_subs.append(gen.declare_subscriber(CMD_MODE_TOPIC, _on_mode))
        _logger.info("p2 wiring: subscribed %s (mode face active)",
                     CMD_MODE_TOPIC)

        # Device liveness -> 11 S6.2 device_offline/online events (SW-12 producers).
        # p5 persists + backfills these. All three producers are REAL:
        #   mic          -- arecord capture/publish thread alive/dead (below)
        #   payload_*    -- PayloadDomain.device_status() polls the service /status
        #                   for the 8519/8529 socket link state
        #   ptz          -- PtzLivenessProbe (non-blocking ONVIF probe thread)
        # Each source can return None ('unknown this tick', e.g. hung endpoint),
        # and the bridge feeds nothing on None -- so an unknown never fabricates a
        # false online/offline. The debounce lives in the monitor (no cloud flood).
        _dev_evt_seq = [0]

        def _emit_device_event(ev: dict) -> None:
            # Publish on the RELATIVE event/{sev}/{cat} key (bus convention); p5's
            # event/** subscriber picks it up, derives channel, persists.
            key = "event/%s/%s" % (ev["sev"], ev["cat"])
            gen.put(key, json.dumps({
                "eid": ev["eid"], "title": ev["title"], "detail": ev["detail"],
                "src": ev["src"], "ts": ev["ts"]}).encode("utf-8"))

        # Boot-unique token: the seq resets to 0 each p2 start but record.db
        # persists, so without it the first offline of a device after a restart
        # regenerates an eid that already exists -> UNIQUE(eid) violation -> the
        # DAO degrades it to JSONL instead of persisting (found in the SW-12 audit).
        # os.urandom keeps the token independent of clock state (not a wall-clock).
        _dev_eid_boot = os.urandom(3).hex()

        def _dev_eid(dev: str, offline: bool) -> str:
            _dev_evt_seq[0] += 1
            return "dev-%s-%s-%s-%d" % (dev, "off" if offline else "on",
                                        _dev_eid_boot, _dev_evt_seq[0])

        from xbrain.p2_core.runtime.device_health_bridge import DeviceHealthBridge
        device_bridge = DeviceHealthBridge(
            rid=os.environ.get("XBRAIN_ROBOT_ID", "unknown"),
            emit=_emit_device_event,
            now_iso=lambda: datetime.now(timezone.utc).isoformat(),
            eid_gen=_dev_eid)
        # Per-device OFFLINE detail (11 S6.2 reason/socket evidence). audio sub-
        # devices (speaker/siren) sit on the 8519 socket, lights (strobe/light) on
        # 8529; mic reason aligns with AsrGate.reason device_fault (16 S8.9.2); ptz
        # reason is the ONVIF-unreachable verdict the probe produces.
        _dev_offline_detail = {
            "mic": {"reason": "device_fault"},
            "ptz": {"reason": "onvif_unreachable"},
            "payload_speaker": {"reason": "device_link_down", "socket": 8519},
            "payload_siren": {"reason": "device_link_down", "socket": 8519},
            "payload_strobe": {"reason": "device_link_down", "socket": 8529},
            "payload_light": {"reason": "device_link_down", "socket": 8529},
        }
        for _d, _od in _dev_offline_detail.items():
            device_bridge.register(_d, offline_detail=_od)
        _logger.info("p2 wiring: device health bridge ON "
                     "(mic + payload + ptz all real)")

        # Main loop: wait for stop.
        try:
            last_hb = time.monotonic()
            last_health = 0.0        # 0 -> publish on the very first pass
            while not stop_flag.get("stop"):
                now = time.monotonic()
                # Drain cmd/mode on the MAIN thread (see the subscriber above
                # on why it is queued). Fully drained rather than one per pass:
                # a burst of mode commands must not be spread over seconds.
                while True:
                    try:
                        _raw = mode_queue.get_nowait()
                    except queue.Empty:
                        break
                    ack = mode_face.handle_frame(
                        _raw, now_mono_ms=int(now * 1000))
                    mode_ack_pub.put(json.dumps(
                        ack, ensure_ascii=False).encode("utf-8"))
                # 11 S2.2: health/summary at 1 Hz. Derived every pass from the
                # live caches; an item whose source has not been heard from
                # stays UNKNOWN with a detail naming what is missing, never ok.
                if now - last_health >= HEALTH_PUBLISH_PERIOD_S:
                    try:
                        refresh_health(health_agg, state_cache, now_mono_s=now,
                                       device_states=device_bridge.states())
                        health_pub.put(json.dumps(
                            health_agg.build_summary(factor_cfg),
                            ensure_ascii=False).encode("utf-8"))
                    except Exception as exc:      # noqa: BLE001
                        _logger.error("p2 health publish failed: %s", exc)
                    last_health = now
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
                    # strobe/light); None -> unknown, no false offline. ptz: read the
                    # PtzLivenessProbe verdict (cached by its own thread, NON-blocking
                    # here -- an inline ONVIF ping would stall this heartbeat).
                    device_bridge.observe("mic", bool(cap_alive and pub_alive))
                    device_bridge.observe(
                        "ptz", ptz_probe.reachable if ptz_probe is not None else None)
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
                if ptz_probe is not None:
                    ptz_probe.stop()
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
