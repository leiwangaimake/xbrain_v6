"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __main__.py
Brief: `python -m xbrain.p3_task` entry point (systemd ExecStart target)

Description:
Same skeleton pattern as p1_motion / p2_core / p4_agent. Makes
xbrain-p3-task.service runnable. Full runtime (task queue + docking
orchestration + fence writer + 15 S12 TC-1..TC-48 test scenarios)
lands with BIZ-P3-24 and dependencies.

★ What this does NOT do yet:
  * open aiosqlite connections to task.db / fence.db / geo.db
  * subscribe cmd/task/create, cmd/task/start, cmd/task/cancel
  * publish state/task, state/dock, event/task/*
  * run the docking state machine (15 S7 handover / seg1 / seg2)
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from typing import Optional


_logger = logging.getLogger("xbrain.p3_task")
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
    if stop_flag is None:
        stop_flag = {"stop": False}
    tick = 0
    while not stop_flag.get("stop"):
        _logger.info("p3_task ready; tick=%d", tick)
        tick += 1
        if max_ticks is not None and tick >= max_ticks:
            break
        remaining = tick_seconds
        while remaining > 0 and not stop_flag.get("stop"):
            step = min(0.5, remaining)
            time.sleep(step)
            remaining -= step
    _logger.info("p3_task exit after %d ticks", tick)
    return 0


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(prog="xbrain.p3_task")
    ap.add_argument("--config", default=None)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--heartbeat-seconds", type=float,
                    default=_HEARTBEAT_SECONDS)
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
        _cfg = load_resolved("p3_task")
    except FileNotFoundError:
        _logger.error(
            "resolved config for p3_task not found; run "
            "xbrain-config-freeze.service first")
        return 3
    except Exception as exc:
        _logger.error(
            "p3_task config invalid: %s: %s "
            "(run xbrain-config-freeze.service to regenerate)",
            type(exc).__name__, exc)
        return 4

    _logger.info("p3_task config OK")

    if args.dry_run:
        _logger.info("dry-run requested; exiting 0")
        return 0

    stop_flag: dict = {"stop": False}
    _install_signal_handlers(stop_flag)
    return main_loop(tick_seconds=args.heartbeat_seconds, stop_flag=stop_flag)


if __name__ == "__main__":
    sys.exit(main())
