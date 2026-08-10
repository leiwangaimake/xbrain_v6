"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __main__.py
Brief: `python -m xbrain.p2_core` entry point (systemd ExecStart target)

Description:
Same skeleton pattern as xbrain/p4_agent/__main__.py:
  * load /run/xbrain/resolved/p2_core.yaml (refuses to start on the
    absent-snapshot / unresolved-value / closed-set-violation paths)
  * enter a heartbeat loop that logs "p2_core ready" every 30 s
  * install SIGTERM/SIGINT handlers so systemd stop is graceful

* What this does NOT do yet (documented for the reader who wonders):
  * instantiate the seven Arbiter domains (motion / speaker / asr /
    payload_light / ptz / gpu / dock) -- their construction needs the
    per-domain source specs from p2_core.yaml which are currently
    all null per CLAUDE.md 3.1; once the values land the wiring goes
    HERE, not into common/arbiter (that's library, not runtime)
  * subscribe cmd/motion/intent from Zenoh general plane
  * publish state/arb/{domain}, cmd/motion/factor, health/bit
  * run the mode state machine (14 S9 six states)
  * run BIT (14 S12 startup + periodic self-test)

Why the skeleton lands before the full runtime:
  * Systemd unit xbrain-p2-core.service was built in CFG-BT-3 with
    ExecStart=`python3 -m xbrain.p2_core`. Before this file existed,
    the unit would fail on start with 'No module named
    xbrain.p2_core.__main__' -- the deploy chain was compiled to a
    target that did not exist.
  * A skeleton that loads config + heartbeats lets integration tests
    verify the boot chain end-to-end (Stage 0 -> 0z -> 0c -> 3 ->
    p2_core UP), which unblocks all downstream work that depends on
    the boot chain being verified.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from typing import Optional


_logger = logging.getLogger("xbrain.p2_core")
_HEARTBEAT_SECONDS = 30.0


def _install_signal_handlers(stop_flag: dict) -> None:
    """SIGTERM/SIGINT -> flip stop_flag so main_loop exits cleanly.

    Uses a mutable dict so tests can inject their own flag and drive
    main_loop() to a clean exit without process signals."""
    def _handler(signum: int, frame) -> None:
        _logger.info("received signal %d, will exit after next tick", signum)
        stop_flag["stop"] = True
    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)


def main_loop(
    max_ticks: Optional[int] = None,
    tick_seconds: float = _HEARTBEAT_SECONDS,
    stop_flag: Optional[dict] = None,
) -> int:
    """Heartbeat loop; returns 0 on clean exit.

    Same shape as xbrain/p4_agent main_loop -- kept consistent so
    an operator reading the logs of p2/p4 sees the same format.

    Testable: pass max_ticks=1, tick_seconds=0.01 in unit tests."""
    if stop_flag is None:
        stop_flag = {"stop": False}
    tick = 0
    while not stop_flag.get("stop"):
        _logger.info("p2_core ready; tick=%d", tick)
        tick += 1
        if max_ticks is not None and tick >= max_ticks:
            break
        remaining = tick_seconds
        while remaining > 0 and not stop_flag.get("stop"):
            step = min(0.5, remaining)
            time.sleep(step)
            remaining -= step
    _logger.info("p2_core exit after %d ticks", tick)
    return 0


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(prog="xbrain.p2_core")
    ap.add_argument("--config", default=None,
                    help="path to resolved p2_core.yaml (default: "
                         "/run/xbrain/resolved/p2_core.yaml)")
    ap.add_argument("--dry-run", action="store_true",
                    help="load config then exit 0 without heartbeat")
    ap.add_argument("--heartbeat-seconds", type=float,
                    default=_HEARTBEAT_SECONDS,
                    help="seconds between heartbeat log lines")
    ap.add_argument("--voice-loop", action="store_true",
                    help="run voice-loop wiring (MIC capture + speaker + "
                         "half-duplex gate) instead of pure heartbeat")
    ap.add_argument("--payload-base-url", default="http://127.0.0.1:18080",
                    help="payload-service base URL (voice-loop only)")
    ap.add_argument("--arecord-device", default="hw:0,0",
                    help="ALSA device (voice-loop only)")
    ap.add_argument("--tts-http-timeout-s", type=float, default=5.0,
                    help="TTS HTTP timeout in seconds (voice-loop only)")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s")

    # p2_core does not (yet) have its own config loader with 16-S12.4-
    # style version-check discipline; use the generic resolved.py.
    try:
        from xbrain.common.config.resolved import load_resolved
    except ImportError as exc:
        _logger.error("cannot import config loader: %s", exc)
        return 2

    try:
        # load_resolved runs the boot_id gate + sha256 check + refuses
        # to open anything outside RESOLVED_ROOT (safety confinement).
        _cfg = load_resolved("p2_core")
    except FileNotFoundError:
        _logger.error(
            "resolved config for p2_core not found; run "
            "xbrain-config-freeze.service first")
        return 3
    except Exception as exc:
        _logger.error(
            "p2_core config invalid: %s: %s "
            "(run xbrain-config-freeze.service to regenerate)",
            type(exc).__name__, exc)
        return 4

    _logger.info("p2_core config OK")

    if args.dry_run:
        _logger.info("dry-run requested; exiting 0")
        return 0

    stop_flag: dict = {"stop": False}
    _install_signal_handlers(stop_flag)

    if args.voice_loop:
        # Voice-loop wiring path: real MIC + speaker + half-duplex gate.
        from xbrain.p2_core.runtime.main_wiring import run_voice_loop_wiring
        from xbrain.p2_core.runtime.mic_capture import (
            DEFAULT_MIC_TOPIC, MicCaptureConfig,
        )
        from xbrain.p2_core.runtime.speaker_wiring import SpeakerWiringConfig
        mic_cfg = MicCaptureConfig(
            arecord_device=args.arecord_device,
            zenoh_topic=DEFAULT_MIC_TOPIC,
            max_queue_frames=8)
        spk_cfg = SpeakerWiringConfig(
            payload_base_url=args.payload_base_url,
            tts_http_timeout_s=args.tts_http_timeout_s,
            est_ms_per_sentence=4000.0)
        return run_voice_loop_wiring(
            mic_cfg=mic_cfg, spk_cfg=spk_cfg, stop_flag=stop_flag)

    return main_loop(tick_seconds=args.heartbeat_seconds, stop_flag=stop_flag)


if __name__ == "__main__":
    sys.exit(main())
