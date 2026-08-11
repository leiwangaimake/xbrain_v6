"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cache.py
Brief: GWY-P4-39 (32.G) -- freshness-aware state/* cache for G queries

Description:
11 S7.16: P4 is a SUBSCRIBER of the GEN-plane state topics (it publishes
none of them). A G-class query ('battery?', 'where am I?') answers from
the LATEST value P4 has received, so this cache stores, per key, the last
value AND the monotonic time it arrived.

Why the timestamp is load-bearing: a stale reading is worse than admitting
ignorance. 16 S8.3 states it for speed ('state/pose age > 1 s -> do not
speak any speed number, say unknown'); the same rule generalises to every
G query (the QT shadow rule, 16 S8.2.1). So get_fresh() returns the value
ONLY if its age is within the caller's threshold, and None otherwise --
the query layer turns None into an 'unknown' answer, never into a
last-known value.

Clock discipline (CLK-C1): the cache stores and compares MONOTONIC
milliseconds supplied by the caller. A wall-clock step must never make a
stale reading look fresh (or a fresh one look stale); the cache never
reads a clock itself, so there is no CLOCK_REALTIME to read the wrong one
from.

Threading: update() is called from the Zenoh subscriber callback (Rust
thread pool). Assignment of a small dataclass into a dict is atomic under
CPython's GIL for a single key, and the query side only ever READS the
latest entry, so no lock is needed for this single-writer-per-key,
last-value-wins cache (CLAUDE.md 4.2 forbids await/create_task in the
callback, not a plain dict write).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, FrozenSet, Optional


# 11 S7.16 / WL-G3: P4 subscribes each state key EXPLICITLY (no wildcard
# subscription -- that is the 'universal bridge' anti-pattern W-2). These
# are the keys the query layer reads from; the wiring (GWY-P4-41) declares
# one subscriber per key into this cache.
STATE_TOPICS: FrozenSet[str] = frozenset({
    "state/pose",          # P1-1, 10 Hz -- position + heading + motion
    "state/power",         # energy: soc + range (16 S8.4 context table)
    "state/mode",          # p2_core -- current mode + health summary
    "state/fence",         # P1-14 -- fence runtime state
    "state/arbitration",   # p5 aggregate of the 7 arb domains
    "state/targets",       # perception, 5-10 Hz -- detected targets
})


@dataclass(frozen=True)
class _Entry:
    value: Any
    mono_ms: int


class StateCache:
    """Last-value-wins cache keyed by state topic, with per-key age."""

    __slots__ = ("_d",)

    def __init__(self) -> None:
        self._d: Dict[str, _Entry] = {}

    def update(self, key: str, value: Any, now_mono_ms: int) -> None:
        """Store the newest value for `key` with its receive time. Called
        from the subscriber callback; last write wins."""
        self._d[key] = _Entry(value=value, mono_ms=now_mono_ms)

    def get_fresh(self, key: str, now_mono_ms: int,
                  max_age_ms: int) -> Optional[Any]:
        """Return the value for `key` iff it arrived within max_age_ms of
        now; else None (missing OR stale). None is the signal the query
        layer turns into an 'unknown' answer (never a stale value)."""
        e = self._d.get(key)
        if e is None:
            return None
        if now_mono_ms - e.mono_ms > max_age_ms:
            return None
        return e.value

    def age_ms(self, key: str, now_mono_ms: int) -> Optional[int]:
        """Age of `key` in ms, or None if never received. For telemetry /
        debugging; the query path uses get_fresh directly."""
        e = self._d.get(key)
        if e is None:
            return None
        return now_mono_ms - e.mono_ms

    def has(self, key: str) -> bool:
        return key in self._d
