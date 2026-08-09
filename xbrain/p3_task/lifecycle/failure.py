"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: failure.py
Brief: BIZ-P3-21 P3 failure matrix S11.2 (crash / db corrupt / disk full / net loss / L3 lost / P1 restart)

Description:
15 S11.2 six-row failure matrix. Every row prescribes ONE recovery
path so there is no choose-your-own-adventure when things go wrong.

  F-1  p3 process crash
       -> systemd restart (Restart=on-failure); on start, replay
          journal, requeue running tasks as 'starting'
  F-2  database corrupt (integrity_check fails)
       -> refuse to start, no auto-restart; wait for human
  F-3  disk full
       -> enter DegradedWriteMode; queue writes; emit health event
  F-4  network loss (Zenoh)
       -> keep local scheduling; every remote publish is buffered
          in memory until the peer reconnects (bounded)
  F-5  L3 lost (link to cloud/HMI)
       -> return_home task auto-injected if running task allows it
  F-6  P1 process restart
       -> re-push current route on RP-2 trigger
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FailureResponse:
    code: str        # F-1..F-6
    action: str      # short slug the caller executes
    detail: str


FAILURE_KINDS = frozenset({
    "process_crash", "db_corrupt", "disk_full",
    "network_lost", "l3_lost", "p1_restart",
})


class UnknownFailureKind(Exception):
    pass


def classify(kind: str) -> FailureResponse:
    """Map a failure kind to its prescribed response."""
    if kind == "process_crash":
        return FailureResponse(
            "F-1", "restart_and_requeue",
            "systemd restarts p3; replay journal, requeue running tasks")
    if kind == "db_corrupt":
        return FailureResponse(
            "F-2", "refuse_start_wait_human",
            "no auto-restart; human must inspect (15 S8)")
    if kind == "disk_full":
        return FailureResponse(
            "F-3", "enter_degraded_write",
            "buffer writes in memory (bounded); emit health event")
    if kind == "network_lost":
        return FailureResponse(
            "F-4", "buffer_remote_publish",
            "keep local scheduling; buffer remote publishes")
    if kind == "l3_lost":
        return FailureResponse(
            "F-5", "inject_return_home",
            "auto-inject return_home if compatible with running task")
    if kind == "p1_restart":
        return FailureResponse(
            "F-6", "repush_route",
            "re-push current task's route on RP-2 trigger")
    raise UnknownFailureKind(f"unknown failure kind {kind!r}")
