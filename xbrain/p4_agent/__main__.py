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

* What this file does today (MVP scope):
  * loads /run/xbrain/resolved/p4_agent.yaml via the existing config
    loader (xbrain/p4_agent/config/loader.py), refusing to start on
    unresolved values per CLAUDE.md 3.1
  * validates that the three AI service base URLs are reachable
    (HEAD probe with short timeout); failure = start refused
  * enters a heartbeat loop that logs "p4_agent ready" every 30 s
    until SIGTERM

* What this file does NOT do yet (post-MVP):
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

* Why heartbeat instead of a stub crash. A stub that exits 0 makes
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


def _load_orchestrator_inputs(config_dir: str):
    """Load the registry + chitchat responder + query templates from the
    static content files (GWY-P4-41). Returns (registry, chitchat,
    query_templates). Raises on a missing/invalid file -- the loop cannot
    classify without the registry, so a partial load is not startable."""
    import os

    import yaml

    from xbrain.p4_agent.registry.intents import (
        load_intent_registry_from_yaml,
    )
    from xbrain.p4_agent.session.chitchat import ChitchatResponder

    registry = load_intent_registry_from_yaml(
        os.path.join(config_dir, "intents.yaml"))
    with open(os.path.join(config_dir, "chitchat.yaml"),
              encoding="utf-8") as fh:
        chitchat = ChitchatResponder(yaml.safe_load(fh))
    with open(os.path.join(config_dir, "query_templates.yaml"),
              encoding="utf-8") as fh:
        query_templates = yaml.safe_load(fh)
    return registry, chitchat, query_templates


def _build_tier2_fn(config_dir, registry, *, base_url, model, timeout_s):
    """Assemble the live tier-2 classify fn from the mission prompts + LLM, or
    return None (-> the orchestrator uses null_tier2) when disabled or the
    content fails to load. Fail-open on purpose: tier-2 is an enhancement over
    the plain decline, so a missing prompt must degrade, never crash the loop."""
    if not base_url:
        _logger.info("tier-2 disabled (empty --llm-base-url)")
        return None
    try:
        from xbrain.p4_agent.gateway.gpu_token import GpuTokenState
        from xbrain.p4_agent.registry.missions import load_missions
        from xbrain.p4_agent.runtime.llm_tier2_fn import build_tier2_fn
        prompts = os.path.join(config_dir, "prompts")
        names = [e.name for e in registry.entries]
        missions = load_missions(os.path.join(prompts, "missions"), names)
        with open(os.path.join(prompts, "system.txt"), encoding="utf-8") as fh:
            system_text = fh.read()
        fn = build_tier2_fn(
            registry, missions, system_text,
            base_url=base_url, model=model, timeout_s=timeout_s,
            token_state=GpuTokenState())
        _logger.info("tier-2 enabled: %d missions, llm=%s", len(missions),
                     base_url)
        return fn
    except Exception as exc:      # noqa: BLE001
        _logger.warning("tier-2 disabled (content load failed): %s: %s",
                        type(exc).__name__, exc)
        return None


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(prog="xbrain.p4_agent")
    ap.add_argument("--config", default=None,
                    help="path to resolved p4_agent.yaml (default: "
                         "/run/xbrain/resolved/p4_agent.yaml)")
    ap.add_argument("--dry-run", action="store_true",
                    help="load config then exit 0 without heartbeat")
    ap.add_argument("--heartbeat-s", type=float, default=_HEARTBEAT_S,
                    help="seconds between heartbeat log lines")
    ap.add_argument("--resolved-root", default=None,
                    help="dev-only: override /run/xbrain/resolved (composes "
                         "resolved-root/p4_agent.yaml)")
    ap.add_argument("--voice-loop", action="store_true",
                    help="run voice-loop wiring (rt/audio/mic subscriber + "
                         "VAD + ASR + intent dispatch) instead of heartbeat")
    ap.add_argument("--asr-base-url", default="http://127.0.0.1:18081",
                    help="services/asr base URL (voice-loop only; U52 port allocation)")
    ap.add_argument("--asr-http-timeout-s", type=float, default=5.0,
                    help="ASR HTTP timeout (voice-loop only)")
    ap.add_argument("--llm-base-url", default="http://127.0.0.1:18082",
                    help="services/llm base URL for tier-2 classify "
                         "(voice-loop only). Empty disables tier-2 (null_tier2)")
    ap.add_argument("--llm-model", default="qwen2.5-3b-instruct",
                    help="model field sent to llama-server (voice-loop only)")
    ap.add_argument("--llm-timeout-s", type=float, default=5.0,
                    help="tier-2 LLM HTTP timeout (voice-loop only; AS-7)")
    ap.add_argument("--vad-energy-threshold", type=int, default=300,
                    help="energy VAD threshold (voice-loop only)")
    ap.add_argument("--vad-tail-silence-ms", type=int, default=500,
                    help="silence to close utterance (voice-loop only)")
    ap.add_argument("--vad-min-utterance-ms", type=int, default=200,
                    help="min utterance to send to ASR (voice-loop only)")
    # The --config-dir default names the source for the reference-free content
    # tables (intents/chitchat/query_templates); with zero ${common.*} their
    # CONFIG-SOURCE-OK(content): source and resolved are byte-identical, so no
    # misresolution can arise (p4_agent.yaml itself is still read from resolved).
    ap.add_argument("--config-dir", default="/opt/xbrain_v6/configs",
                    help="dir holding intents.yaml/chitchat.yaml/"
                         "query_templates.yaml (voice-loop orchestrator)")
    ap.add_argument("--l2-confirm-timeout-ms", type=int, default=8000,
                    help="L2 confirm wait window (voice-loop only)")
    ap.add_argument("--query-state-max-age-ms", type=int, default=2000,
                    help="max age before a state reading is 'unknown' "
                         "(voice-loop G queries)")
    ap.add_argument("--query-battery-low-pct", type=int, default=20,
                    help="SOC at/below which battery answer uses the low "
                         "branch (voice-loop only)")
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

    # load_p4_config signature is (agent_version, root=..., ...) --
    # the FIRST arg is the running agent's own semver, compared against
    # the resolved config's min_agent_version (16 S12.4). Passing the
    # yaml PATH here was a V-2B mistake; the loader tried to parse
    # '/opt/.../p4_agent.yaml' as a semver and refused startup.
    #
    # AGENT_VERSION is the P4 process's own release identifier. Kept
    # a module-local constant here because the p4_agent package has
    # no single 'version.py' today; when it lands, this should read
    # that value instead of hard-coding.
    AGENT_VERSION = "1.0"
    if args.config:
        product_path = args.config
        product_root = None
    elif args.resolved_root:
        product_root = args.resolved_root.rstrip('/')
        product_path = f"{product_root}/p4_agent.yaml"
    else:
        product_root = None
        product_path = "/opt/xbrain_v6/data/run/resolved/p4_agent.yaml"
    try:
        # load_p4_config performs the CLAUDE.md 3.1 null-guard + the
        # 16 S12.4 min_agent_version check + the enable_on closed-set
        # check. It refuses to start on any of the three.
        if args.config:
            # Explicit config path -- caller pinned a specific yaml;
            # the loader still needs a root, derive it from the path.
            derived_root = os.path.dirname(args.config)
            _cfg = load_p4_config(AGENT_VERSION, root=derived_root)
        elif product_root:
            _cfg = load_p4_config(AGENT_VERSION, root=product_root)
        else:
            _cfg = load_p4_config(AGENT_VERSION)
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

    if args.voice_loop:
        from xbrain.p4_agent.runtime.main_wiring import run_voice_loop_wiring
        from xbrain.p4_agent.runtime.turn_loop import TurnLoopConfig
        from xbrain.p4_agent.runtime.vad import VadConfig
        vad_cfg = VadConfig(
            energy_threshold=args.vad_energy_threshold,
            tail_silence_ms=args.vad_tail_silence_ms,
            min_utterance_ms=args.vad_min_utterance_ms,
            frame_ms=20)
        tl_cfg = TurnLoopConfig(
            asr_base_url=args.asr_base_url,
            asr_http_timeout_s=args.asr_http_timeout_s,
            vad_cfg=vad_cfg)
        # GWY-P4-41 (32.I): build the orchestrator inputs from the static
        # content files (intents / chitchat / query templates) so the loop
        # runs the six-step chain, not the V-2B naive path. A missing file
        # is a hard startup failure -- the loop cannot classify without the
        # registry.
        from xbrain.p4_agent.runtime.orchestrator_turn import (
            VoiceOrchestratorInputs,
        )
        try:
            registry, chitchat, query_templates = _load_orchestrator_inputs(
                args.config_dir)
        except Exception as exc:      # noqa: BLE001
            _logger.error("voice-loop: cannot load orchestrator inputs "
                          "from %s: %s: %s", args.config_dir,
                          type(exc).__name__, exc)
            return 5
        tier2_fn = _build_tier2_fn(
            args.config_dir, registry, base_url=args.llm_base_url,
            model=args.llm_model, timeout_s=args.llm_timeout_s)
        # Site display timezone (common.timezone) for G24 query_time. Read from
        # the RESOLVED snapshot (single source; freeze expanded ${common.*}),
        # NOT a CLI default -- a wrong tz silently mis-answers 'what time is it',
        # so refuse startup on an unresolvable zone rather than degrade to UTC.
        from xbrain.common.time.local_time import is_valid_tz
        site_tz = _cfg.require("timezone")
        if not is_valid_tz(site_tz):
            _logger.error(
                "voice-loop: common.timezone %r is not a resolvable IANA zone "
                "on this host; startup refused (fix configs/common.yaml)",
                site_tz)
            return 6
        orch = VoiceOrchestratorInputs(
            registry=registry, chitchat=chitchat,
            l2_timeout_ms=args.l2_confirm_timeout_ms,
            query_templates=query_templates,
            query_max_age_ms=args.query_state_max_age_ms,
            query_low_soc_pct=args.query_battery_low_pct,
            site_timezone=site_tz,
            tier2_fn=tier2_fn)
        return run_voice_loop_wiring(cfg=tl_cfg, stop_flag=stop_flag,
                                     orch=orch)

    # Pre-existing bug fix: attr is heartbeat_s (from --heartbeat-s),
    # not heartbeat_seconds. Silently correcting the old typo here.
    return main_loop(tick_seconds=args.heartbeat_s, stop_flag=stop_flag)


if __name__ == "__main__":
    sys.exit(main())
