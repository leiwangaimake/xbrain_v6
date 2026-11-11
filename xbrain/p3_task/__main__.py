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
import math
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
            DEFAULT_TASK_DB,
            run_voice_loop_wiring,
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
        # 18 PS-6 (99 U109): the oldest pose_snap a keypoint capture accepts.
        # No fallback value here either: unset is passed as None and the
        # runtime refuses snapshot captures by name.
        _teach = (_cfg.get("teach") or {}) if hasattr(_cfg, "get") else {}
        # 99 U126: the arrive radius of a teach-recorded route (15 S2.4.3,
        # common.recording.arrive_radius_m). Missing / null / <= 0 refuses to
        # start with the key path (EX_CONFIG 78, CLAUDE.md 3.6 iron rule 2):
        # without it every recorded patrol would be refused at push time, a
        # failure that only shows when someone says "patrol".
        _route = (_cfg.get("route") or {}) if hasattr(_cfg, "get") else {}
        _radius = _route.get("recorded_arrive_radius_m")
        if (isinstance(_radius, bool) or not isinstance(_radius, (int, float))
                or _radius <= 0.0):
            _logger.error(
                "p3_task config: route.recorded_arrive_radius_m=%r must be a "
                "positive number (common.recording.arrive_radius_m, 15 S2.4.3)",
                _radius)
            return 78
        # 99 U127: a waypoint within route.membership_m of a route polyline
        # belongs to it and a goto follows that route. Same refusal shape as
        # the radius above: without it every goto would silently be a
        # straight line again.
        _member = _route.get("membership_m")
        if (isinstance(_member, bool) or not isinstance(_member, (int, float))
                or _member <= 0.0):
            _logger.error(
                "p3_task config: route.membership_m=%r must be a positive "
                "number (99 U127 route-aware goto)", _member)
            return 78
        # 99 U132: the recording sampling pair (common.recording.min_dist_m /
        # sample_hz via teach.*). Refused by key path rather than recorded at
        # a rate or spacing nobody configured.
        # 99 U162: + record_duration_s, the recording's only limit
        # (common.recording.session_timeout_s), refused the same way.
        for _tk in ("min_dist_m", "sample_hz", "record_duration_s"):
            _tv = _teach.get(_tk)
            if (isinstance(_tv, bool) or not isinstance(_tv, (int, float))
                    or _tv <= 0.0):
                _logger.error(
                    "p3_task config: teach.%s=%r must be a positive number "
                    "(99 U132 / U162 recording)", _tk, _tv)
                return 78
        # 99 U163: the time-left warnings (seconds before the 180 min limit,
        # e.g. [600, 60]). Required like the rest -- a list of finite positive
        # numbers, each below record_duration_s (one at or above it would
        # fire at the start).
        _tw = _teach.get("record_warn_before_s")
        if (not isinstance(_tw, list) or not _tw
                or any(isinstance(w, bool) or not isinstance(w, (int, float))
                       or not math.isfinite(w) or w <= 0.0
                       or w >= float(_teach["record_duration_s"]) for w in _tw)):
            _logger.error(
                "p3_task config: teach.record_warn_before_s=%r must be a non-empty "
                "list of positive seconds below teach.record_duration_s (99 U163)", _tw)
            return 78
        # 99 U142 / 11 S3.3.3: how long single / no_fix must last before the
        # running task is suspended and the alarm raised, and how long a usable
        # fix before recovery is reported. Refused by key path: without them a
        # lost fix would leave the task "running" and the robot silent.
        _rtk = (_cfg.get("rtk_loss") or {}) if hasattr(_cfg, "get") else {}
        for _rk in ("lost_sustain_s", "recover_sustain_s"):
            _rv = _rtk.get(_rk)
            if (isinstance(_rv, bool) or not isinstance(_rv, (int, float))
                    or _rv <= 0.0):
                _logger.error(
                    "p3_task config: rtk_loss.%s=%r must be a positive number "
                    "(99 U142 positioning loss, 11 S3.3.3)", _rk, _rv)
                return 78
        # 99 U146 / U157 low-battery return: the five values of the user's
        # rule. Refused by key path (EX_CONFIG 78): without them a low battery
        # would raise no alarm and drive nowhere, or the charging alarm would
        # have no end and the interrupted tasks would never resume.
        _lb = (_cfg.get("low_battery") or {}) if hasattr(_cfg, "get") else {}
        _lb_keys = ("trigger_soc_pct", "stop_soc_pct", "cloud_alarm_period_s",
                    "charge_full_soc_pct", "swap_resume_soc_pct",
                    "swap_absent_min_s", "cold_swap_min_rise_pct",
                    "plug_confirm_s", "unplug_confirm_s")
        for _lk in _lb_keys:
            _lv = _lb.get(_lk)
            # finite too: a .nan passed `<= 0` and silenced the alarm cadence
            # for good (2026-11-11 audit)
            if (isinstance(_lv, bool) or not isinstance(_lv, (int, float))
                    or not math.isfinite(_lv) or _lv <= 0.0):
                _logger.error(
                    "p3_task config: low_battery.%s=%r must be a positive number "
                    "(99 U146 / U157 low-battery return)", _lk, _lv)
                return 78
        from xbrain.p3_task.lifecycle.low_battery import (
            LowBatteryConfigError,
            check_thresholds,
        )
        try:
            check_thresholds(_lb["trigger_soc_pct"], _lb["stop_soc_pct"],
                             _lb["charge_full_soc_pct"], _lb["swap_resume_soc_pct"])
        except LowBatteryConfigError as exc:
            _logger.error("p3_task config: %s (99 U157)", exc)
            return 78
        return run_voice_loop_wiring(
            stop_flag=stop_flag,
            task_db_path=args.task_db or DEFAULT_TASK_DB,
            enu_origin=_enu,
            teach_snap_max_age_ms=_teach.get("snap_max_age_ms"),
            recorded_arrive_radius_m=float(_radius),
            route_membership_m=float(_member),
            teach_sample_hz=float(_teach["sample_hz"]),
            teach_min_dist_m=float(_teach["min_dist_m"]),
            teach_record_duration_s=float(_teach["record_duration_s"]),
            teach_record_warn_before_s=[float(w) for w in _teach["record_warn_before_s"]],
            rtk_lost_sustain_s=float(_rtk["lost_sustain_s"]),
            rtk_recover_sustain_s=float(_rtk["recover_sustain_s"]),
            low_battery={k: float(_lb[k]) for k in _lb_keys})

    return main_loop(tick_seconds=args.heartbeat_seconds, stop_flag=stop_flag)


if __name__ == "__main__":
    sys.exit(main())
