"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: audit_ring.py
Brief: MOT-RD-9 RNS monotonic clock + ring-buffer audit trail

Description:
20 §11 RNS realtime rules:
  * every time reference is monotonic ms (never wall-clock)
  * audit records go into a fixed-size ring buffer (no growth,
    no I/O in the hot path)

The ring buffer holds the last N decisions. Retrieving is only
done from the SLOW path (post-tick log flush); the tick body only
APPENDS.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Deque, List
from collections import deque


@dataclass(frozen=True)
class RnsAuditRecord:
    """One RNS decision record."""
    mono_ms: int
    action: str        # 'proposed' / 'shutdown' / 'skipped'
    reason: str
    detail: dict


class RnsAuditRing:
    """Fixed-capacity ring buffer. Append is O(1); no growth."""

    def __init__(self, capacity: int = 256) -> None:
        self._buf: Deque[RnsAuditRecord] = deque(maxlen=capacity)

    def append(self, rec: RnsAuditRecord) -> None:
        """Hot-path append; no I/O, no lock."""
        self._buf.append(rec)

    def snapshot(self) -> List[RnsAuditRecord]:
        """Slow-path retrieval; copies the buffer for the log flush."""
        return list(self._buf)

    def __len__(self) -> int:
        return len(self._buf)
