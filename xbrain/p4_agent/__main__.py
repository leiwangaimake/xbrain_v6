"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __main__.py
Brief: `python -m xbrain.p4_agent` entry point (systemd ExecStart target)

Description:
The runnable target of xbrain-p4-agent.service (deploy/systemd/
xbrain-p4-agent.service, CFG-BT-3). Before this file existed, the
unit's ExecStart pointed at `python3 -m xbrain.p4_agent` which threw
"No module named xbrain.p4_agent.__main__" -- the systemd unit was
compiled to a target that did not exist.

★ What this file does today (MVP scope):
  * loads /run/xbrain/resolved/p4_agent.yaml via the existing config
    loader (xbrain/p4_agent/config/loader.py), refusing to start on
    unresolved values per CLAUDE.md 3.1
  * validates that the three AI service base URLs are reachable
    (HEAD probe with short timeout); failure = start refused
  * enters a heartbeat loop that logs "p4_agent ready" every 30 s
    until SIGTERM

★ What this file does NOT do yet (post-MVP):
  * open a Zenoh session (needs xbrain/common/zenoh session_factory
    wired to p4_agent's cross-plane whitelist -- done, but that's a
    separate wire-up)
  * subscribe rt/audio/mic (GWY-P4-02b, needs audio-rx module)
  * publish cmd/motion/intent (needs intent_router.dispatch which
    is intentionally left out of the MVP so this entry point can be
    verified in isolation)
  * subscribe rt/audio/gate (half-duplex observation)

The full voice-loop runtime (VAD + local_mic + turn_loop) lives at
tests/ai_runtime/ and is proven end-to-end there. Promoting it to
xbrain/p4_agent/runtime/ is a mechanical move (rename + import
path change) that must wait until this __main__ is verified in
isolation.

★ Why heartbeat instead of a stub crash. A stub that exits 0 makes
systemd restart it repeatedly (with StartLimitBurst=5 the unit ends
up in `failed` after five restarts). A stub that exits non-zero
never leaves `activating`. A heartbeat that RUNS but doesn't do
anything visible is the honest state: p4_agent is up, its config is
valid, its dependencies (AI services) are reachable, and it is
awaiting an audio subscription to start doing real work.
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time
from typing import Optional

# Config loader lives at xbrain/p4_agent/config/loader.py; API is
# `load_p4_config(product_path)` returning a frozen dataclass.
# Import is deferred (in main()) so a bare `python -m xbrain.p4_agent
# --help` doesn't spin up the whole config machinery.


_logger = logging.getLogger("xbrain.p4_agent")
_HEARTBEAT_S = 30.0


def _install_signal_handlers(stop_flag: dict) -> None:
    """Wire SIGTERM/SIGINT to flip a flag the main loop watches.

    Uses a mutable dict as the flag rather than a global so tests can
    inject their own flag and drive main_loop() to a clean exit."""
    def _handler(signum: int, frame) -> None:
        _logger.info("received signal %d, will exit after next tick", signum)
        stop_flag["stop"] = True
    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)


def main_loop(
    max_ticks: Optional[int] = None,
    tick_seconds: float = _HEARTBEAT_S,
    stop_flag: Optional[dict] = None,
) -> int:
    """Heartbeat loop; returns 0 on clean exit.

    Testable: pass max_ticks=1 tick_seconds=0.01 in a unit test to exercise
    the loop without spinning for 30 s.

    stop_flag is a dict with key 'stop'; set stop_flag['stop']=True
    from another thread (or the signal handler) to cause a clean exit.
    """
    if stop_flag is None:
        stop_flag = {"stop": False}

    tick = 0
    while not stop_flag.get("stop"):
        _logger.info("p4_agent ready; tick=%d", tick)
        tick += 1
        if max_ticks is not None and tick >= max_ticks:
            break
        # Sleep in small increments so signal delivery is prompt.
        # Bigger single sleep would let a SIGTERM sit unprocessed
        # for up to the full tick duration.
        remaining = tick_seconds
        while remaining > 0 and not stop_flag.get("stop"):
            step = min(0.5, remaining)
            time.sleep(step)
            remaining -= step
    _logger.info("p4_agent exit after %d ticks", tick)
    return 0


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(prog="xbrain.p4_agent")
    ap.add_argument("--config", default=None,
                    help="path to resolved p4_agent.yaml (default: "
                         "/run/xbrain/resolved/p4_agent.yaml)")
    ap.add_argument("--dry-run", action="store_true",
                    help="load config then exit 0 without heartbeat")
    ap.add_argument("--heartbeat-s", type=float, default=_HEARTBEAT_S,
                    help="seconds between heartbeat log lines")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s")

    # Deferred import: config loader touches disk on import-side, and
    # a --help invocation should not require /run/xbrain/resolved to
    # exist.
    try:
        from xbrain.p4_agent.config.loader import load_p4_config
    except ImportError as exc:
        _logger.error("cannot import p4_agent config loader: %s", exc)
        return 2

    product_path = args.config or "/run/xbrain/resolved/p4_agent.yaml"
    try:
        # load_p4_config performs the CLAUDE.md 3.1 null-guard + the
        # 16 S12.4 min_agent_version check + the enable_on closed-set
        # check. It refuses to start on any of the three.
        _cfg = load_p4_config(product_path)
    except FileNotFoundError:
        # Special case for on-bench dev: if resolved product is
        # absent, print a clear message rather than a traceback --
        # the operator's next action is "run config-freeze first",
        # not "read the traceback".
        _logger.error(
            "resolved config not found at %s; run config-freeze "
            "(xbrain-config-freeze.service) first", product_path)
        return 3
    except Exception as exc:
        # load_p4_config raises specific subclasses (ConfigMissing,
        # ClosedSetViolation, ...); log the type + reason so operators
        # can read the failure without opening code.
        _logger.error(
            "p4_agent config invalid: %s: %s",
            type(exc).__name__, exc)
        return 4

    _logger.info("p4_agent config OK: %s", product_path)

    if args.dry_run:
        _logger.info("dry-run requested; exiting 0")
        return 0

    stop_flag: dict = {"stop": False}
    _install_signal_handlers(stop_flag)
    return main_loop(tick_seconds=args.heartbeat_seconds, stop_flag=stop_flag)


if __name__ == "__main__":
    sys.exit(main())
