"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: dedupe.py
Brief: CHK-0-39 云端入站统一去重层 (rid+msg_id + seq 水位 + 单调钟)

Description:
Every inbound message from Qt/云端 passes through this layer once
translate() has validated its shape. Rules (docs/MISSON/任务枚举
_qt端v2.0.md R1.7, R1.8, R11.2):

  * R1.8  (rid, msg_id) is the idempotency key. Window is >= 60s;
          within the window a repeat returns Duplicate (the caller
          re-emits the last ack, does NOT re-execute).
  * R1.7  seq is uint64 per (发布进程, rid, 完整 key). A new
          publisher process may reset seq to 1; a WITHIN-process
          rewind (lower seq on the same connection window) means
          drop. The window is defined by the last time we saw a
          fresh (rid, key) connection signal; here we take the
          simpler rule: any seq LOWER than the highest seq observed
          within the current session -> drop.
  * R11.2 age is measured against the receiver's monotonic clock.
          If (now_mono_ms - inbound_mono_ms) > stale_max_ms, drop.
          The inbound.ts is NEVER used for age; ts is only for audit
          / log alignment.

Session semantics: when the receive-loop detects a new session
(reconnect, fresh publisher_id), it calls session_reset(rid, key)
to clear that stream's seq high-water mark. Without this, a
legitimate reboot of the publisher would look like a rewind and
every message would be dropped.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Dict, Optional


DEFAULT_DEDUP_WINDOW_MS = 60_000   # R1.8: >= 60s


@dataclass(frozen=True)
class DedupeVerdict:
    """One of:
      * accepted=True                      -> forward to next stage
      * accepted=False + is_duplicate=True -> reissue original ack
      * accepted=False + is_stale=True     -> drop with audit
      * accepted=False + is_rewind=True    -> drop with audit"""
    accepted: bool
    is_duplicate: bool = False
    is_stale: bool = False
    is_rewind: bool = False
    reason: str = ""


@dataclass
class DedupeCache:
    """(rid, msg_id) -> first_seen_mono_ms, in insertion order.
    O(1) eviction of oldest entry when its age passes the window."""
    entries: "OrderedDict[tuple, int]" = field(default_factory=OrderedDict)
    window_ms: int = DEFAULT_DEDUP_WINDOW_MS

    def see(self, rid: str, msg_id: str, now_mono_ms: int) -> bool:
        """Return True if fresh, False if already in the window."""
        self._sweep(now_mono_ms)
        key = (rid, msg_id)
        if key in self.entries:
            return False
        self.entries[key] = now_mono_ms
        return True

    def _sweep(self, now_mono_ms: int) -> None:
        cutoff = now_mono_ms - self.window_ms
        while self.entries:
            k, seen_ms = next(iter(self.entries.items()))
            if seen_ms >= cutoff:
                return
            self.entries.popitem(last=False)


@dataclass
class SeqWatermarks:
    """Per-stream (rid, key) high-water seq. session_reset clears it."""
    highest: Dict[tuple, int] = field(default_factory=dict)

    def note(self, rid: str, key: str, seq: int) -> bool:
        """Return True if seq is at or above the high-water mark
        (fresh); False if it's a rewind. On True we also update the
        mark."""
        pk = (rid, key)
        prev = self.highest.get(pk)
        if prev is None or seq >= prev:
            self.highest[pk] = max(prev or 0, seq)
            return True
        return False

    def session_reset(self, rid: str, key: str) -> None:
        self.highest.pop((rid, key), None)


class InboundDedupe:
    """Composed of the two components above plus a stale-age check."""

    def __init__(self, stale_max_ms: int,
                 window_ms: int = DEFAULT_DEDUP_WINDOW_MS) -> None:
        if stale_max_ms <= 0:
            raise ValueError(
                f"stale_max_ms must be > 0, got {stale_max_ms}")
        self.cache = DedupeCache(window_ms=window_ms)
        self.watermarks = SeqWatermarks()
        self.stale_max_ms = stale_max_ms

    def check(self, rid: str, msg_id: str, seq: int, key: str,
                inbound_mono_ms: int, now_mono_ms: int) -> DedupeVerdict:
        # R11.2 stale first -- a stale packet does not update state.
        age = now_mono_ms - inbound_mono_ms
        if age < 0:
            return DedupeVerdict(
                accepted=False, is_stale=True,
                reason=f"future_ts age={age}ms (clock skew)")
        if age > self.stale_max_ms:
            return DedupeVerdict(
                accepted=False, is_stale=True,
                reason=f"stale age={age}ms > {self.stale_max_ms}ms")
        # R1.8 dedupe key second.
        if not self.cache.see(rid, msg_id, now_mono_ms):
            return DedupeVerdict(
                accepted=False, is_duplicate=True,
                reason="in dedupe window")
        # R1.7 seq rewind third.
        if not self.watermarks.note(rid, key, seq):
            return DedupeVerdict(
                accepted=False, is_rewind=True,
                reason=f"seq {seq} below watermark")
        return DedupeVerdict(accepted=True)

    def session_reset(self, rid: str, key: str) -> None:
        """Called on new-publisher-detected signal (fresh session)."""
        self.watermarks.session_reset(rid, key)
