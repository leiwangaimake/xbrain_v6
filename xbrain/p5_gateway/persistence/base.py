"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: base.py
Brief: record.db connection + PRAGMA discipline (15 S9.1: two-writer + one reader)

Description:
15 S9.1 freezes the connection-level PRAGMAs for all four DBs: WAL, synchronous
NORMAL, foreign_keys OFF, busy_timeout 5000. But alarm/fault events (FS-d) and
approval_state (FS-c) MUST commit with synchronous=FULL (11 PWR-3): under NORMAL,
a commit reaches the WAL but is not fsync'd, so a power loss right after can lose
the alarm -- exactly the event that must never be lost. Because synchronous is a
CONNECTION-level pragma, "this one FULL, that one NORMAL" on a single connection
is impossible without a silent-revert failure mode (15 S9.1 S-2). So record.db is
opened as THREE connections (S-2, "两写一读"):
  * writer NORMAL  -- the default: info/warn events, cursor updates (high volume)
  * writer FULL    -- alarm/fault events (FS-d), approval_state (FS-c)
  * reader         -- query_only=1: HMI reads + backfill scans, cannot corrupt the
                      single-writer invariant even by accident

aiosqlite only (15 S9.1 S-1): the stdlib sqlite3 driver blocks the calling thread
on every execute()/commit(), and one 50-200 ms WAL fsync would freeze the whole
asyncio loop (perception callbacks, estop). CLAUDE.md 4.1 restates this as a lint
rule; here it is ALSO enforced structurally -- a real sqlite3.Connection lacks the
async dunders and fails validation at construction.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


# Connection-level defaults for the two writers + the reader (15 S9.1). The FULL
# writer overrides synchronous only; everything else is identical, so the two
# writers agree on WAL/FK/busy and differ ONLY in durability (S-2).
BASE_PRAGMAS: dict = {
    "journal_mode": "WAL",
    "synchronous": "NORMAL",   # default writer; FULL writer overrides this key
    "foreign_keys": "OFF",     # app-layer referential integrity (15 S9.1 DBF-2)
    "busy_timeout": "5000",    # 5 s before OperationalError (writers serialise)
}

# FS-d / FS-c: alarm/fault events + approval_state commit FULL (11 PWR-3). Only
# the synchronous key changes -- do NOT diverge on WAL/FK/busy or the two writers
# stop being the same schema on the same file.
FULL_WRITER_SYNCHRONOUS = "FULL"


class PersistenceMisuse(Exception):
    """Caller violated 15 S9.1 (sync driver, missing PRAGMA, write on reader)."""


@dataclass(frozen=True)
class RecordConn:
    """One aiosqlite connection to record.db, tagged with its role. The role is
    not decoration: the DAO routes alarm/fault inserts to role='writer_full' and
    refuses any write on role='reader', so a durability or single-writer mistake
    is a raise, not a silent NORMAL-commit of an alarm.

    aiosqlite is enforced structurally (same as p3_task/persistence): the object
    must expose an async 'execute' plus __aenter__/__aexit__. A vanilla
    sqlite3.Connection lacks those and fails here, not deep inside a transaction.
    """

    role: str          # 'writer_normal' | 'writer_full' | 'reader'
    path: str
    conn: object

    _ROLES = frozenset({"writer_normal", "writer_full", "reader"})

    def __post_init__(self) -> None:
        if self.role not in RecordConn._ROLES:
            raise PersistenceMisuse(
                f"RecordConn role must be one of {sorted(RecordConn._ROLES)}, "
                f"got {self.role!r}")
        if not hasattr(self.conn, "__aenter__") or not hasattr(self.conn, "execute"):
            raise PersistenceMisuse(
                f"RecordConn[{self.role}]: connection is not an aiosqlite async "
                f"connection (15 S9.1 S-1 forbids sync sqlite3)")

    @property
    def is_writer(self) -> bool:
        return self.role in ("writer_normal", "writer_full")


def pragma_statements(full: bool = False, reader: bool = False) -> Iterable[str]:
    """Yield the 'PRAGMA k=v;' lines for one connection, in stable order.
    full=True swaps synchronous to FULL (FS-c/FS-d); reader=True appends
    query_only=1 so the read connection cannot write even by mistake."""
    for k, v in BASE_PRAGMAS.items():
        if k == "synchronous" and full:
            v = FULL_WRITER_SYNCHRONOUS
        yield f"PRAGMA {k}={v};"
    if reader:
        yield "PRAGMA query_only=1;"


def assert_base_pragmas(pragmas: dict) -> None:
    """FS-a-style fail-safe: at startup every base pragma must actually be set.
    A missing pragma is refused (raise), never warned -- a record.db opened with
    default SQLite settings (rollback journal, FK ON) would violate the frozen
    15 S9.1 contract silently."""
    for k, want in BASE_PRAGMAS.items():
        got = pragmas.get(k)
        if got is None:
            raise PersistenceMisuse(f"required pragma {k!r} was never set")
        if str(got).upper() != str(want).upper():
            raise PersistenceMisuse(f"pragma {k}: got {got!r}, need {want!r}")


async def _open_one(path: str, role: str, ddl_statements=()):
    """Open ONE aiosqlite connection with the pragmas for `role` and apply DDL.
    aiosqlite is imported HERE, inside persistence/, on purpose (CLAUDE.md 4.1
    forbids the import elsewhere): wiring code never opens its own connection, it
    calls this and then goes through the DAO."""
    import os

    import aiosqlite

    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    # isolation_level=None -> sqlite3 does NO implicit transaction management, so
    # record_dao's explicit BEGIN IMMEDIATE / COMMIT / ROLLBACK are the sole
    # authority. Without this, sqlite3's default deferred auto-begin collides with
    # a raw "BEGIN IMMEDIATE" ("cannot start a transaction within a transaction"),
    # and we would silently get a DEFERRED lock -- losing the upfront write lock
    # that keeps ch_seq allocation race-free (SEQ-2) across the two writers.
    conn = await aiosqlite.connect(path, isolation_level=None)
    full = role == "writer_full"
    reader = role == "reader"
    for stmt in pragma_statements(full=full, reader=reader):
        await conn.execute(stmt)
    # DDL runs only on a writer; a reader is query_only and cannot CREATE.
    if not reader:
        for stmt in ddl_statements:
            await conn.execute(stmt)
        await conn.commit()
    return RecordConn(role=role, path=path, conn=conn)


async def open_record_writer(path: str, full: bool = False, ddl_statements=()):
    """Open the NORMAL (default) or FULL writer to record.db. The FULL writer is
    for FS-d (alarm/fault events) and FS-c (approval_state) only."""
    return await _open_one(
        path, "writer_full" if full else "writer_normal", ddl_statements)


async def open_record_reader(path: str):
    """Open the query_only reader (HMI reads + backfill scans)."""
    return await _open_one(path, "reader")
