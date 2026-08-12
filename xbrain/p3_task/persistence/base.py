"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: base.py
Brief: BIZ-P3-1 aiosqlite persistence base + FS-a..FS-g rules

Description:
15 S9 persistence layer contract. The p3_task process opens exactly
THREE connections:
  * one for task.db (writer + reader)
  * one for fence.db (writer + reader)
  * one for geo.db  (writer + reader)
Each connection lives on the dedicated db thread; no other thread may
issue a DB call directly. Every writing transaction MUST use
PRAGMA journal_mode=WAL and synchronous=FULL and MUST open with
BEGIN IMMEDIATE (FS-a..FS-g).

15 S9 forbids the sync sqlite3 module in this process (blocks the
asyncio loop). CLAUDE.md 4.1 restates this as a hard lint rule.
Callers must pass an aiosqlite Connection; a real sqlite3.Connection
raises PersistenceMisuse at construction time.

MVP scope: base connection wrapper only. Table DDL lives in
BIZ-P3-3/4/5; DAO code in BIZ-P3-6.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


REQUIRED_PRAGMAS: dict = {
    "journal_mode": "WAL",
    "synchronous": "FULL",
    "foreign_keys": "ON",
    "busy_timeout": "5000",
}


class PersistenceMisuse(Exception):
    """Caller violated 15 S9 (sync driver, missing PRAGMA, etc)."""


@dataclass(frozen=True)
class DbHandle:
    """One writer+reader connection scoped to one physical database file.

    The aiosqlite driver is enforced structurally: the object exposes
    a coroutine 'execute' plus '__aenter__/__aexit__'. A vanilla
    sqlite3 Connection lacks those and fails validation."""

    name: str
    path: str
    conn: object

    def __post_init__(self) -> None:
        if not hasattr(self.conn, "__aenter__") or not hasattr(self.conn, "execute"):
            raise PersistenceMisuse(
                f"DbHandle {self.name}: connection is not an aiosqlite "
                f"async connection (see 15 S9)")


def format_pragma_statements(pragmas: dict = REQUIRED_PRAGMAS) -> Iterable[str]:
    """Yield 'PRAGMA k=v;' lines in stable order.
    Emitted at connection open; not run inside a BEGIN IMMEDIATE tx."""
    for k, v in pragmas.items():
        yield f"PRAGMA {k}={v};"


def assert_all_required_pragmas(pragmas: dict) -> None:
    """FS-a: startup must have set every required PRAGMA. Missing any
    is a fail-safe: refuse to serve traffic (raise, not warn)."""
    for k, want in REQUIRED_PRAGMAS.items():
        got = pragmas.get(k)
        if got is None:
            raise PersistenceMisuse(f"required pragma {k!r} was never set")
        if str(got).upper() != str(want).upper():
            raise PersistenceMisuse(
                f"pragma {k}: got {got!r}, need {want!r}")


async def open_configured(path: str, ddl_statements=()):
    """Open ONE aiosqlite connection to `path`, apply the REQUIRED_PRAGMAs, and
    run `ddl_statements` (CREATE ... IF NOT EXISTS), returning the connection.

    aiosqlite is imported HERE, inside persistence/, on purpose: CLAUDE.md 4.1
    forbids importing sqlite3/aiosqlite anywhere else -- business/wiring code
    opens no connection of its own, it calls this and then goes through a DAO.
    The parent directory is created if missing (the voice-loop data/run dir)."""
    import os

    import aiosqlite

    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    conn = await aiosqlite.connect(path)
    for stmt in format_pragma_statements():
        await conn.execute(stmt)
    for stmt in ddl_statements:
        await conn.execute(stmt)
    await conn.commit()
    return conn
