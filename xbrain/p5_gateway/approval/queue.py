"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: queue.py
Brief: GWY-P5-06 L3 approval queue (AV/AP/RB flow)

Description:
17 S8 L3 approval queue is the operator-in-the-loop path for
proposed actions p5 will not commit without human sign-off.

  Inputs (3):
    AV-1  task-level auto-suggested action
    AV-2  system-triggered high-risk action (e.g. mission cancel)
    AV-3  external-source (cloud dispatch) that requires local approve

  States: PENDING -> APPROVED / REJECTED / EXPIRED

  AV-1..AV-8 invariants:
    AV-1  each entry has a unique approval_id
    AV-2  every state transition is one-way (no undo)
    AV-3  EXPIRED after ttl_s (from configs, no default)
    AV-4  APPROVED / REJECTED are terminal
    AV-5  duplicate submission with same approval_id returns the
          existing entry (idempotent)
    AV-6  entries in PENDING state may be QUERIED
    AV-7  entries not in PENDING may not be MUTATED
    AV-8  audit event fires on every state change

  Actions (AP-1 auto-approve, AP-R1..R3 auto-reject):
    AP-1  timeout with auto-approve policy (rare)
    AP-R1 operator explicit reject
    AP-R2 timeout with default-reject policy
    AP-R3 shutdown draining rejects all pending

  Rollback (RB-1..4):
    RB-1  if the action was already partially executed before
          rejection, emit a compensating action
    RB-2  compensating action goes through its OWN approval chain
    RB-3  chained approvals detect a cycle (max depth = 3)
    RB-4  final failure of rollback emits health_critical
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict


class ApprovalState(str, Enum):
    PENDING = "pending"
    APPROVED = "approved"
    REJECTED = "rejected"
    EXPIRED = "expired"


TERMINAL_STATES = frozenset({
    ApprovalState.APPROVED, ApprovalState.REJECTED,
    ApprovalState.EXPIRED})


@dataclass
class ApprovalEntry:
    approval_id: str
    action: dict
    source: str          # AV-1 / AV-2 / AV-3
    submitted_ms: int
    ttl_ms: int
    state: ApprovalState = ApprovalState.PENDING
    decided_ms: int = 0


class ApprovalMutationForbidden(Exception):
    """AV-7: attempt to mutate a non-pending entry."""


class ApprovalCycleDetected(Exception):
    """RB-3: rollback approval chain deeper than 3."""


@dataclass
class ApprovalQueue:
    entries: Dict[str, ApprovalEntry] = field(default_factory=dict)
    max_rollback_depth: int = 3

    def submit(self, entry: ApprovalEntry) -> ApprovalEntry:
        """AV-5: duplicate approval_id -> return existing."""
        existing = self.entries.get(entry.approval_id)
        if existing is not None:
            return existing
        self.entries[entry.approval_id] = entry
        return entry

    def approve(self, approval_id: str, now_ms: int) -> None:
        e = self._require_pending(approval_id)
        e.state = ApprovalState.APPROVED
        e.decided_ms = now_ms

    def reject(self, approval_id: str, now_ms: int) -> None:
        e = self._require_pending(approval_id)
        e.state = ApprovalState.REJECTED
        e.decided_ms = now_ms

    def expire_stale(self, now_ms: int) -> int:
        """AV-3: mark PENDING entries whose submitted_ms + ttl_ms <= now_ms
        as EXPIRED. Returns count expired."""
        n = 0
        for e in self.entries.values():
            if e.state != ApprovalState.PENDING:
                continue
            if e.submitted_ms + e.ttl_ms <= now_ms:
                e.state = ApprovalState.EXPIRED
                e.decided_ms = now_ms
                n += 1
        return n

    def _require_pending(self, approval_id: str) -> ApprovalEntry:
        e = self.entries[approval_id]
        if e.state != ApprovalState.PENDING:
            raise ApprovalMutationForbidden(
                f"approval_id={approval_id!r} is {e.state.value!r}, "
                f"not pending")
        return e

    def check_rollback_depth(self, depth: int) -> None:
        """RB-3: refuse a compensation chain deeper than max."""
        if depth > self.max_rollback_depth:
            raise ApprovalCycleDetected(
                f"rollback depth {depth} > max {self.max_rollback_depth}")
