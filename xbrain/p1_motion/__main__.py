"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __main__.py
Brief: `python -m xbrain.p1_motion` entry point (systemd ExecStart target)

Description:
Same skeleton pattern as xbrain/p2_core and xbrain/p4_agent. Makes
systemd unit xbrain-p1-motion.service runnable.

* What this does NOT do yet:
  * open two Zenoh sessions (RT plane + GEN plane -- p1 is one of the
    three cross-plane processes per 11 S1.1.6)
  * subscribe cmd/motion/factor (from p2_core), perception/targets
    (from perception), state/link (from p5_gateway)
  * publish cmd_vel at 20 Hz to chassis_relay (RT plane)
  * run the four-band speed gate (11 S9.6.2)
  * run RNS (reactive nav stack, internal module)
  * run the rotation permission gate (12 S6A RCG-1..4)

Why the skeleton lands before the full 20 Hz loop:
  * P1 is a SAFETY-critical process. Its full loop is 60+ Python
    modules per the design spec (12 covers ~300 pages just for P1),
    and building it under time pressure is exactly how the six ARB-*
    invariants get violated (14 S3.3.2 "P2 crashing must not stop P1
    arbitrating motion").
  * A skeleton that heartbeat's while safely emitting ZERO commands
    is the minimum-viable P1 for boot-chain verification, and cannot
    move the robot (which is the right default when the runtime
    logic is not yet written).
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from typing import Optional


_logger = logging.getLogger("xbrain.p1_motion")
_HEARTBEAT_SECONDS = 30.0


def _install_signal_handlers(stop_flag: dict) -> None:
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

    * CRITICAL: this loop MUST NOT publish any cmd_vel while the real
    runtime is stubbed. A stub that published even 'zero cmd_vel'
    would be misleading -- it looks like p1 is working, but there is
    no arbitrated factor coming in, no perception overlay, no speed
    gate, no rotation permission check. If anything downstream ever
    starts trusting stubbed p1 output, the robot moves under a
    fabricated authority. Publishing NOTHING keeps the chassis in
    timeout_lock, which is the safe state.

    Testable: pass max_ticks=1, tick_seconds=0.01 in unit tests."""
    if stop_flag is None:
        stop_flag = {"stop": False}
    tick = 0
    while not stop_flag.get("stop"):
        _logger.info("p1_motion ready (skeleton; no cmd_vel published); tick=%d",
                     tick)
        tick += 1
        if max_ticks is not None and tick >= max_ticks:
            break
        remaining = tick_seconds
        while remaining > 0 and not stop_flag.get("stop"):
            step = min(0.5, remaining)
            time.sleep(step)
            remaining -= step
    _logger.info("p1_motion exit after %d ticks", tick)
    return 0


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(prog="xbrain.p1_motion")
    ap.add_argument("--config", default=None,
                    help="path to resolved p1_motion.yaml (default: "
                         "/run/xbrain/resolved/p1_motion.yaml)")
    ap.add_argument("--dry-run", action="store_true",
                    help="load config then exit 0 without heartbeat")
    ap.add_argument("--heartbeat-seconds", type=float,
                    default=_HEARTBEAT_SECONDS,
                    help="seconds between heartbeat log lines")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s")

    try:
        from xbrain.common.config.resolved import load_resolved
    except ImportError as exc:
        _logger.error("cannot import config loader: %s", exc)
        return 2

    try:
        _cfg = load_resolved("p1_motion")
    except FileNotFoundError:
        _logger.error(
            "resolved config for p1_motion not found; run "
            "xbrain-config-freeze.service first")
        return 3
    except Exception as exc:
        _logger.error(
            "p1_motion config invalid: %s: %s "
            "(run xbrain-config-freeze.service to regenerate)",
            type(exc).__name__, exc)
        return 4

    _logger.info("p1_motion config OK")

    if args.dry_run:
        _logger.info("dry-run requested; exiting 0")
        return 0

    stop_flag: dict = {"stop": False}
    _install_signal_handlers(stop_flag)
    return main_loop(tick_seconds=args.heartbeat_seconds, stop_flag=stop_flag)


if __name__ == "__main__":
    sys.exit(main())
