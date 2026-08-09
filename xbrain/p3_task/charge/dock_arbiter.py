"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: dock_arbiter.py
Brief: BIZ-P3-10 domain 7 dock arbiter (three keys + state/arb/dock + six ops)

Description:
14 defines seven arbitration domains. Domain 7 (dock) is the ONLY
domain P3 arbitrates itself; the other six live in p2_core.

The dock arbiter guards contention on ONE resource: the physical
charging dock. Two robots must never attempt contact on the same
dock simultaneously. The arbiter carries three keys per dock:

  cmd/dock/req      request to reserve a dock
  state/arb/dock    published grant / deny snapshot
  state/dock/pose   ongoing occupancy pose (published by charger)

The six operations (op = one of these):
  OP-1  reserve      hold the dock for me
  OP-2  release      done, others may take it
  OP-3  cede         voluntarily give up (e.g. lower priority arrived)
  OP-4  demand       preempt (only allowed when priority is higher)
  OP-5  refresh      keepalive
  OP-6  query        read the current holder

RULES:
* only one holder per dock at any time
* demand only succeeds when requester_priority > holder_priority
* reserve on an occupied dock returns DENIED
* release without holding is idempotent (no-op)
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class DockOp(str, Enum):
    RESERVE = "reserve"
    RELEASE = "release"
    CEDE    = "cede"
    DEMAND  = "demand"
    REFRESH = "refresh"
    QUERY   = "query"


class DockResult(str, Enum):
    GRANTED = "granted"
    DENIED  = "denied"
    NO_OP   = "no_op"
    PREEMPTED = "preempted"


@dataclass
class DockHolder:
    task_id: str
    priority: int


class DockArbiter:
    """One arbiter instance covers all docks in a site.
    All calls run on the p3 db thread; there is NO cross-thread
    lock because the arbiter has no other caller (15 §7 discipline)."""

    def __init__(self) -> None:
        self._holders: dict = {}   # dock_id -> DockHolder

    def apply(self, dock_id: str, task_id: str, priority: int,
                op: DockOp) -> DockResult:
        holder = self._holders.get(dock_id)
        if op == DockOp.QUERY:
            return DockResult.GRANTED if holder is None else DockResult.DENIED
        if op == DockOp.RESERVE:
            if holder is None:
                self._holders[dock_id] = DockHolder(task_id, priority)
                return DockResult.GRANTED
            if holder.task_id == task_id:
                return DockResult.GRANTED    # already ours, idempotent
            return DockResult.DENIED
        if op == DockOp.RELEASE:
            if holder is None or holder.task_id != task_id:
                return DockResult.NO_OP
            del self._holders[dock_id]
            return DockResult.GRANTED
        if op == DockOp.CEDE:
            if holder is not None and holder.task_id == task_id:
                del self._holders[dock_id]
                return DockResult.GRANTED
            return DockResult.NO_OP
        if op == DockOp.DEMAND:
            if holder is None:
                self._holders[dock_id] = DockHolder(task_id, priority)
                return DockResult.GRANTED
            if holder.task_id == task_id:
                return DockResult.GRANTED
            if priority > holder.priority:
                self._holders[dock_id] = DockHolder(task_id, priority)
                return DockResult.PREEMPTED
            return DockResult.DENIED
        if op == DockOp.REFRESH:
            if holder is not None and holder.task_id == task_id:
                return DockResult.GRANTED
            return DockResult.NO_OP
        raise ValueError(f"unknown op {op!r}")

    def current_holder(self, dock_id: str) -> DockHolder | None:
        return self._holders.get(dock_id)
