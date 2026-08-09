"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: skeleton.py
Brief: GWY-P5-25 nine-task asyncio skeleton + P-1/P-2 + G-1/G-2/G-3

Description:
17 S1 p5_gateway is single-process but has NINE cooperating asyncio
tasks. Naming them so misuse is a name error, not a runtime crash:

  T-1  event_ingress      Zenoh subs -> event pipeline
  T-2  event_pipeline     the 7-step pipeline (schema, dedup, ...)
  T-3  cloud_uplink       stream events to cloud
  T-4  hmi_ws             WebSocket server for HMI
  T-5  rest_api           REST readonly endpoints
  T-6  approval_loop      L3 approval queue
  T-7  telemetry          4-class telemetry aggregation
  T-8  delivery_ledger    3-stage handshake tracker
  T-9  watchdog           deadman / probe / uptime

P-1 P-2 are the two writers to record.db: T-2 (events) + T-8
(delivery). Nothing else writes; other tasks read via DAO.

G-1/G-2/G-3 are the three global guards enforced at startup:
  G-1  every task must register into TASK_REGISTRY at start
  G-2  T-9 monitors all others; task death = process exit
  G-3  no task may block > 100ms without releasing back to the loop
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


NINE_TASKS = (
    "event_ingress", "event_pipeline", "cloud_uplink", "hmi_ws",
    "rest_api", "approval_loop", "telemetry", "delivery_ledger",
    "watchdog",
)


P1_P2_WRITERS = frozenset({"event_pipeline", "delivery_ledger"})


class DuplicateTaskRegistration(Exception):
    pass


class IncompleteTaskRegistration(Exception):
    pass


@dataclass
class TaskRegistry:
    """G-1 gate. All nine tasks must call register() at startup.
    A missing entry raises IncompleteTaskRegistration."""
    tasks: dict

    @classmethod
    def empty(cls) -> "TaskRegistry":
        return cls(tasks={})

    def register(self, name: str, handle: object) -> None:
        if name not in NINE_TASKS:
            raise DuplicateTaskRegistration(
                f"unknown task name {name!r}")
        if name in self.tasks:
            raise DuplicateTaskRegistration(
                f"task {name!r} already registered")
        self.tasks[name] = handle

    def assert_complete(self) -> None:
        missing = set(NINE_TASKS) - set(self.tasks)
        if missing:
            raise IncompleteTaskRegistration(
                f"missing tasks: {sorted(missing)}")

    def can_write_record_db(self, name: str) -> bool:
        return name in P1_P2_WRITERS
