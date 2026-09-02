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

* What this does NOT do yet:
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
    ap.add_argument("--resolved-root", default=None,
                    help="dev-only: override /run/xbrain/resolved for local smoke")
    ap.add_argument("--voice-loop", action="store_true",
                    help="run voice-loop MVP wiring instead of heartbeat")
    ap.add_argument("--task-db", default=None,
                    help="dev/voice-loop: task.db path "
                         "(default data/run/task.db)")
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
        _load_kwargs = {"root": args.resolved_root} if args.resolved_root else {}
        _cfg = load_resolved("p3_task", **_load_kwargs)
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

    if args.voice_loop:
        from xbrain.p3_task.runtime.main_wiring import (
            DEFAULT_TASK_DB, run_voice_loop_wiring,
        )
        # 11 S9A.2: FenceSet 的 enu_origin 是[必填]字段("本地 ENU 平面的锚点,
        # 全系统唯一, 来自 common.geo.enu_origin"). 在此之前 p3 加载了配置却
        # 只用来验证存在性, 没有任何值往下传, 于是 build_fence_set 的
        # enu_origin 形参一直取默认 None, cmd/fence 上该字段恒为 null.
        # 下游后果: HMI 拿不到投影锚点 -> 几何数据齐全却画不出来; 11 FV-7
        # (顶点距原点 <= 20km) 连判据都无从谈起.
        # NO 不在这里兜底成 0/0/0 -- 那是 CLAUDE.md 3.1 的 fail-silent 陷阱,
        # 一个假原点会把所有围栏画到几千公里外而不报任何错. 配置缺失时传 None,
        # 由既有的 FV-ORG 启动断言去拦.
        _geo = (_cfg.get("geo") or {}) if hasattr(_cfg, "get") else {}
        _enu = _geo.get("enu_origin") or None
        return run_voice_loop_wiring(
            stop_flag=stop_flag,
            task_db_path=args.task_db or DEFAULT_TASK_DB,
            enu_origin=_enu)

    return main_loop(tick_seconds=args.heartbeat_seconds, stop_flag=stop_flag)


if __name__ == "__main__":
    sys.exit(main())
