"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: threads.py
Brief: BIZ-P3-0 four-thread skeleton (main / rx / db / tx)

Description:
15 S2 p3_task process = FOUR threads:
  main: startup, config, systemd notify, watchdog
  rx  : Zenoh subscribers (15 topics) drain into an in-memory queue
  db  : the ONLY thread that touches aiosqlite handles (three files)
  tx  : Zenoh publishers (8 topics) drain from the tx queue

Rx callbacks run on the Zenoh Rust thread pool. They MUST NOT
touch aiosqlite -- they hand the parsed frame to the rx thread
via a threadsafe queue. The db thread pulls from the same queue
and does the write. This preserves 15 S9 "one connection per
file per thread" and prevents cross-thread reuse of aiosqlite.

MVP scope: named-role registry + safety guards for cross-thread
handle use. Real Zenoh wiring is BIZ-P3-19/25 territory.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from enum import Enum


class ThreadRole(str, Enum):
    MAIN = "main"
    RX = "rx"
    DB = "db"
    TX = "tx"


@dataclass(frozen=True)
class ThreadIdent:
    """Assigned thread role. Set at thread startup, never mutated."""
    role: ThreadRole
    tid: int


class ThreadRegistry:
    """One entry per role. Set-once; a second call to bind() the
    same role raises. Reading is lock-free (dict get is atomic)."""

    def __init__(self) -> None:
        self._map: dict = {}
        self._lock = threading.Lock()

    def bind(self, role: ThreadRole, tid: int) -> ThreadIdent:
        with self._lock:
            if role in self._map:
                raise RuntimeError(
                    f"role {role.value!r} already bound to tid "
                    f"{self._map[role].tid}")
            ident = ThreadIdent(role=role, tid=tid)
            self._map[role] = ident
            return ident

    def get(self, role: ThreadRole) -> ThreadIdent:
        try:
            return self._map[role]
        except KeyError:
            raise RuntimeError(f"role {role.value!r} not bound yet")

    def role_of(self, tid: int) -> ThreadRole:
        for role, ident in self._map.items():
            if ident.tid == tid:
                return role
        raise RuntimeError(f"tid {tid} has no bound role")


def require_thread(registry: ThreadRegistry, expected: ThreadRole) -> None:
    """Assert the CALLER is running on the expected role.
    Used at DAO entry points: only the db thread may call DAOs."""
    actual = registry.role_of(threading.get_ident())
    if actual != expected:
        raise RuntimeError(
            f"call must run on thread {expected.value!r}, "
            f"but caller is on {actual.value!r}")
