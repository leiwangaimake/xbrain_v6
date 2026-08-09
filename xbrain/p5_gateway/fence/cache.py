"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cache.py
Brief: GWY-P5-15 fence_cache (cmd/fence subscribe + P5F-1/-2/-3)

Description:
17 S15 fence cache: p5_gateway subscribes to cmd/fence (published
by p3_task) and maintains an in-memory cache used by REST + HMI +
authz. The cache is READ-ONLY on the p5 side; p5 does NOT mutate
fence data (p3 is the only writer).

P5F-1  cache invalidation on every cmd/fence message
P5F-2  serve stale-but-marked when the cmd/fence stream is silent
        for stale_after_ms; response is tagged 'stale=true' so the
        caller decides how to react
P5F-3  never fall back to reading p3's fence.db directly (crosses
        the process boundary that DAO layer enforces)
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class FenceCache:
    fences: tuple = ()
    updated_ms: int = 0

    def on_update(self, new_fences, now_ms: int) -> None:
        """P5F-1: overwrite cache on cmd/fence receive."""
        self.fences = tuple(new_fences)
        self.updated_ms = now_ms

    def snapshot(self, now_ms: int, stale_after_ms: int):
        """P5F-2: return (fences, is_stale). is_stale flag is set
        when the cache has not been refreshed for stale_after_ms."""
        is_stale = (now_ms - self.updated_ms) > stale_after_ms
        return self.fences, is_stale


class DirectDbAccessForbidden(Exception):
    """P5F-3: attempt to read fence.db directly from p5."""


def refuse_direct_db_read(operation: str) -> None:
    raise DirectDbAccessForbidden(
        f"p5 must not read p3's fence.db directly (op={operation!r})")
