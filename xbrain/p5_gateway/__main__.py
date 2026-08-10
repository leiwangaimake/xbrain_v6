"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __main__.py
Brief: `python -m xbrain.p5_gateway` entry point (systemd ExecStart target)

Description:
Same skeleton pattern as p1_motion / p2_core / p3_task / p4_agent.
Makes xbrain-p5-gateway.service runnable.

* ONE DIFFERENCE from the other P-processes: p5_gateway supports a
MINIMAL MODE (observation window W-1, INF-DP-8). When
xbrain-config-freeze.service failed to produce a snapshot, p5_gateway
must still start (with default L0 constants only) so HMI can display
the failure and the operator can see what broke. All other processes
refuse to start on missing config; p5_gateway specifically does not.

MVP status: the config-optional path lands here (no snapshot ->
minimal mode heartbeat), but the FULL minimal mode (HMI + boot_fail
replay + three-state BIT annotation) is INF-DP-8 proper.

* What this does NOT do yet:
  * FastAPI HMI backend on 127.0.0.1:8080 (not 0.0.0.0, per SEC-06)
  * event pipeline: sub cmd/event/*, dedup, write to record.db
  * state/link publisher (P5 is the UNIQUE publisher of state/link)
  * cloud uplink translator
  * boot_fail.jsonl replay on next successful boot
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from typing import Optional


_logger = logging.getLogger("xbrain.p5_gateway")
_HEARTBEAT_SECONDS = 30.0


def _install_signal_handlers(stop_flag: dict) -> None:
    def _handler(signum: int, frame) -> None:
        _logger.info("received signal %d, will exit after next tick", signum)
        stop_flag["stop"] = True
    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)


def main_loop(
    minimal_mode: bool = False,
    max_ticks: Optional[int] = None,
    tick_seconds: float = _HEARTBEAT_SECONDS,
    stop_flag: Optional[dict] = None,
) -> int:
    """Heartbeat loop. If minimal_mode=True, log line says so.

    Minimal mode differs from full mode in what's actually running (in
    full mode: HMI + event pipeline + state/link publisher; in minimal:
    only the "config-freeze failed" HMI annotation). The heartbeat
    log line distinguishes so an operator watching journal knows
    which state they're in."""
    if stop_flag is None:
        stop_flag = {"stop": False}
    label = "MINIMAL" if minimal_mode else "ready"
    tick = 0
    while not stop_flag.get("stop"):
        _logger.info("p5_gateway %s; tick=%d", label, tick)
        tick += 1
        if max_ticks is not None and tick >= max_ticks:
            break
        remaining = tick_seconds
        while remaining > 0 and not stop_flag.get("stop"):
            step = min(0.5, remaining)
            time.sleep(step)
            remaining -= step
    _logger.info("p5_gateway exit after %d ticks (minimal_mode=%s)",
                 tick, minimal_mode)
    return 0


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(prog="xbrain.p5_gateway")
    ap.add_argument("--config", default=None)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--force-minimal-mode", action="store_true",
                    help="skip config load; enter minimal mode "
                         "(observation window W-1) directly")
    ap.add_argument("--heartbeat-seconds", type=float,
                    default=_HEARTBEAT_SECONDS)
    ap.add_argument("--resolved-root", default=None,
                    help="dev-only: override /run/xbrain/resolved for local smoke")
    ap.add_argument("--voice-loop", action="store_true",
                    help="run voice-loop MVP wiring instead of heartbeat")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s")

    minimal_mode = args.force_minimal_mode

    if not minimal_mode:
        try:
            from xbrain.common.config.resolved import load_resolved
            _load_kwargs = {"root": args.resolved_root} if args.resolved_root else {}
            _cfg = load_resolved("p5_gateway", **_load_kwargs)
            _logger.info("p5_gateway config OK; entering full mode")
        except FileNotFoundError:
            # * p5_gateway SPECIFIC: config-freeze failed = enter
            # minimal mode, NOT exit. This is the whole point of the
            # observation window W-1 per 10 S3.3 -- HMI must still
            # come up so operator can see the failure. Other P-procs
            # exit here.
            _logger.warning(
                "resolved config for p5_gateway not found; "
                "entering MINIMAL MODE (observation window W-1)")
            minimal_mode = True
        except Exception as exc:
            _logger.warning(
                "p5_gateway config invalid (%s: %s); "
                "entering MINIMAL MODE",
                type(exc).__name__, exc)
            minimal_mode = True

    if args.dry_run:
        _logger.info("dry-run requested; exiting 0 (minimal_mode=%s)",
                     minimal_mode)
        return 0

    stop_flag: dict = {"stop": False}
    _install_signal_handlers(stop_flag)

    if args.voice_loop:
        from xbrain.p5_gateway.runtime.main_wiring import run_voice_loop_wiring
        return run_voice_loop_wiring(stop_flag=stop_flag)

    return main_loop(
        minimal_mode=minimal_mode,
        tick_seconds=args.heartbeat_seconds,
        stop_flag=stop_flag)


if __name__ == "__main__":
    sys.exit(main())
