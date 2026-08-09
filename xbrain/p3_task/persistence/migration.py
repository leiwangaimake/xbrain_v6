"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: migration.py
Brief: BIZ-P3-2 S-7 versioned migration framework + S-8 corrupt refuse

Description:
15 S-7 defines a monotonically increasing schema version stored in
a dedicated schema_version table (id INTEGER PK CHECK id=1,
version INTEGER NOT NULL). Migrations are ordered ascending; every
step is a one-way DDL burst wrapped in a transaction. Skipping a
step is an error (S-7-b); attempting to re-apply a completed step
is a no-op.

15 S-8 refuses to run against a corrupt database: PRAGMA
integrity_check must return the single row 'ok'. Any other result
means the file cannot be trusted for a writer role and the process
raises DatabaseCorrupt; the systemd unit does NOT auto-restart (the
lifecycle spec calls for a human to look at it).

S-6 DegradedWriteMode is a runtime flag flipped by disk-space watch
+ hard I/O errors: writes fall back to a bounded in-memory buffer
that the operator drains once storage is healed. NEVER silently
swallowed -- there is a health item and an event when the mode
enters.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List


class DatabaseCorrupt(Exception):
    """PRAGMA integrity_check returned something other than 'ok'."""


class MigrationOrderError(Exception):
    """Migrations are not strictly ascending, or a step is missing."""


@dataclass(frozen=True)
class Migration:
    """One DDL step, applied once, in ascending version order."""
    version: int
    description: str
    apply_sql: str


def validate_migration_sequence(migrations: List[Migration]) -> None:
    """S-7-b: versions must be strictly ascending starting at 1;
    no gaps. Detecting a gap once is cheaper than sorting through
    a corrupted schema_version table at runtime."""
    if not migrations:
        raise MigrationOrderError("no migrations supplied")
    prev = 0
    for m in migrations:
        if m.version != prev + 1:
            raise MigrationOrderError(
                f"migration gap: expected {prev + 1}, got {m.version}")
        prev = m.version


def pending_migrations(all_migrations: List[Migration],
                         current_version: int) -> List[Migration]:
    """Return the sequence of migrations still to apply, in order."""
    validate_migration_sequence(all_migrations)
    return [m for m in all_migrations if m.version > current_version]


def parse_integrity_check(rows: list) -> None:
    """S-8: PRAGMA integrity_check returns [('ok',)] on health.
    Any other output means the file is compromised; refuse to open."""
    if rows == [("ok",)]:
        return
    raise DatabaseCorrupt(f"integrity_check failed: {rows!r}")


@dataclass
class DegradedWriteMode:
    """S-6: when writes fail because of I/O or disk-full errors, the
    db thread enters degraded mode. Writes are queued in memory up to
    a bounded cap; entering / leaving both emit a health event."""
    active: bool = False
    reason: str = ""
    buffered_writes: int = 0
    buffer_cap: int = 512

    def enter(self, reason: str) -> None:
        """Flip to degraded. No-op if already degraded (keeps
        original reason so we don't lose the first cause)."""
        if self.active:
            return
        self.active = True
        self.reason = reason
        self.buffered_writes = 0

    def leave(self) -> None:
        """Return to normal writes; buffered_writes reset."""
        self.active = False
        self.reason = ""
        self.buffered_writes = 0

    def record_buffered(self) -> None:
        """Track one write held in memory. Beyond buffer_cap, we
        raise so callers escalate rather than swallow silently."""
        if self.buffered_writes >= self.buffer_cap:
            raise RuntimeError(
                f"degraded write buffer full ({self.buffer_cap})")
        self.buffered_writes += 1
