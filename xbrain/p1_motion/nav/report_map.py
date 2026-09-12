"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: report_map.py
Brief: RNS failure reason -> outward carrier fields (12 S4.2c.6, the single upstream is 20 S9.0.2)

Description:
The report channel is chosen by the mission's ORIGIN (12 S4.2c.1), never by
its category: origin=route reports on state/motion/path_progress with
state="failed" and fail_reason = the 20 S9.0.2 reason VERBATIM (no
re-mapping, 11 S3.5B v2.0); origin=relmove reports on
cmd/motion/relative_move/status with state="aborted" and the 11 v2.0 seven-
value abort_reason plus detail.item. This module is the relmove half of that
table and the event severity both origins share (12 S4.2c.6 "事件两 origin
都发").

The table (12 S4.2c.6, verbatim rows):
    max_deviation          -> deviation   (item none)            warn
    wall_closed_loop       -> obstacle    unreachable            warn
    wall_no_progress       -> obstacle    no_progress            warn
    wall_budget            -> obstacle    budget                 warn
    watchdog_no_progress   -> fence if the argmax limiter source is fence,
                              else obstacle; item = that source  warn
    blocked_by_dynamic     -> obstacle    dynamic:{track_id}     warn
    rtk_unreliable         -> input_lost  rtk                    fault
    no_path_in_domain      -> obstacle    unreachable:domain     warn
    extrinsic_uncalibrated -> input_lost  extrinsic              fault
    target_lost            -> (reserved #20-13, no carrier)      raise

Closed sets both ways (CLAUDE.md 3.5): an NavFailReason outside the table
raises ReportMapError rather than degrading to some "unknown" reason, and the
seven abort_reason values are the 11 S9.3.2A.6 TTS table's keys -- a value P4
cannot phrase must never reach the wire.

What it does NOT do: it does not build the status body (mission_host.py), does
not publish, and does not touch path_progress (fail_reason is the reason
string itself, no table needed).
"""
from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from xbrain.p1_motion.rns.types import (WATCHDOG_FENCE_SOURCE, NavFailReason,
                                        NavFailure)

#: 11 S9.3.2A.6 abort_reason closed set (v2.0: six + deviation).
ABORT_REASONS = ("soft_estop", "obstacle", "fence", "timeout", "preempted",
                 "input_lost", "deviation")
SEV_WARN = "warn"
SEV_FAULT = "fault"


class ReportMapError(ValueError):
    """A reason with no row in 12 S4.2c.6 (reserved or unknown)."""


#: reason -> (abort_reason, detail.item, severity) for the static rows.
_STATIC: Dict[NavFailReason, Tuple[str, Optional[str], str]] = {
    NavFailReason.MAX_DEVIATION: ("deviation", None, SEV_WARN),
    NavFailReason.WALL_CLOSED_LOOP: ("obstacle", "unreachable", SEV_WARN),
    NavFailReason.WALL_NO_PROGRESS: ("obstacle", "no_progress", SEV_WARN),
    NavFailReason.WALL_BUDGET: ("obstacle", "budget", SEV_WARN),
    NavFailReason.RTK_UNRELIABLE: ("input_lost", "rtk", SEV_FAULT),
    NavFailReason.NO_PATH_IN_DOMAIN: ("obstacle", "unreachable:domain", SEV_WARN),
    NavFailReason.EXTRINSIC_UNCALIBRATED: ("input_lost", "extrinsic", SEV_FAULT),
}


def map_failure(failure: NavFailure) -> Tuple[str, Optional[str], str]:
    """(abort_reason, detail_item, severity) for a relmove-origin failure.
    mutant: map watchdog_no_progress to obstacle regardless of the limiter
    source -> a fence-caused stall is voiced as an obstacle -> red."""
    reason = failure.reason
    if reason in _STATIC:
        return _STATIC[reason]
    detail: Dict[str, Any] = failure.detail or {}
    if reason is NavFailReason.WATCHDOG_NO_PROGRESS:
        limiter = detail.get("limiter")
        carrier = "fence" if limiter == WATCHDOG_FENCE_SOURCE else "obstacle"
        return (carrier, str(limiter) if limiter is not None else None, SEV_WARN)
    if reason is NavFailReason.BLOCKED_BY_DYNAMIC:
        return ("obstacle", "dynamic:%s" % detail.get("track_id"), SEV_WARN)
    raise ReportMapError("no 12 S4.2c.6 carrier for reason %r" % (reason,))


def event_severity(failure: NavFailure) -> str:
    """The event/{sev}/motion severity for any origin (same table column)."""
    return map_failure(failure)[2]
