"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_a.py
Brief: BIZ-P3-0/1/2 skeleton + persistence base + migration tests

Description:
Batch A covers the four-thread role registry, aiosqlite connection
validation, PRAGMA gate, migration ordering, integrity-check parsing,
and degraded-write buffering. Every negative test names the CLAUDE.md
or 15 S# clause it enforces so a spec drift is easy to trace.
"""

import threading

import pytest

from xbrain.p3_task.persistence.base import (
    DbHandle, PersistenceMisuse, REQUIRED_PRAGMAS,
    assert_all_required_pragmas, format_pragma_statements,
)
from xbrain.p3_task.persistence.migration import (
    DatabaseCorrupt, DegradedWriteMode, Migration, MigrationOrderError,
    parse_integrity_check, pending_migrations, validate_migration_sequence,
)
from xbrain.p3_task.persistence.threads import (
    ThreadRegistry, ThreadRole, require_thread,
)


pytestmark = pytest.mark.no_device


# --- BIZ-P3-0 thread registry ---

def test_bind_role_records_tid():
    reg = ThreadRegistry()
    ident = reg.bind(ThreadRole.DB, tid=42)
    assert ident.role == ThreadRole.DB and ident.tid == 42


def test_rebind_same_role_rejected():
    """15 S2: each role bound exactly once (role is a set-once
    contract, not a mutable variable)."""
    reg = ThreadRegistry()
    reg.bind(ThreadRole.DB, tid=42)
    with pytest.raises(RuntimeError, match="already bound"):
        reg.bind(ThreadRole.DB, tid=99)


def test_require_thread_rejects_wrong_role():
    """DAO called from rx thread must raise (15 S9)."""
    reg = ThreadRegistry()
    reg.bind(ThreadRole.RX, tid=threading.get_ident())
    with pytest.raises(RuntimeError, match="must run on"):
        require_thread(reg, ThreadRole.DB)


def test_require_thread_allows_correct_role():
    reg = ThreadRegistry()
    reg.bind(ThreadRole.DB, tid=threading.get_ident())
    require_thread(reg, ThreadRole.DB)   # no raise


# --- BIZ-P3-1 aiosqlite base ---

def test_dbhandle_rejects_sync_sqlite3():
    """Any non-aiosqlite connection object is refused at
    construction (defense against CLAUDE.md 4.1 sqlite3 import)."""
    class FakeSync:
        def execute(self, *a, **kw): pass
    with pytest.raises(PersistenceMisuse, match="aiosqlite"):
        DbHandle(name="task", path=":memory:", conn=FakeSync())


def test_dbhandle_accepts_aiosqlite_shape():
    """A stub with __aenter__ + execute passes structural check."""
    class FakeAsync:
        async def __aenter__(self): return self
        async def __aexit__(self, *a): pass
        async def execute(self, *a, **kw): pass
    DbHandle(name="task", path=":memory:", conn=FakeAsync())


def test_pragma_statements_include_required_keys():
    lines = list(format_pragma_statements())
    for k in REQUIRED_PRAGMAS:
        assert any(k in ln for ln in lines)


def test_assert_all_required_pragmas_raises_on_missing():
    with pytest.raises(PersistenceMisuse, match="never set"):
        assert_all_required_pragmas({"journal_mode": "WAL"})


def test_assert_all_required_pragmas_raises_on_wrong_value():
    bad = dict(REQUIRED_PRAGMAS)
    bad["synchronous"] = "OFF"     # not FULL
    with pytest.raises(PersistenceMisuse, match="synchronous"):
        assert_all_required_pragmas(bad)


def test_assert_all_required_pragmas_accepts_full_set():
    assert_all_required_pragmas(dict(REQUIRED_PRAGMAS))


# --- BIZ-P3-2 migration ---

def test_validate_rejects_empty_migrations():
    with pytest.raises(MigrationOrderError, match="no migrations"):
        validate_migration_sequence([])


def test_validate_rejects_gap():
    ms = [Migration(1, "init", ""), Migration(3, "skip", "")]
    with pytest.raises(MigrationOrderError, match="gap"):
        validate_migration_sequence(ms)


def test_validate_accepts_dense_ascending():
    ms = [Migration(1, "a", ""), Migration(2, "b", ""), Migration(3, "c", "")]
    validate_migration_sequence(ms)   # no raise


def test_pending_returns_only_higher_versions():
    ms = [Migration(i, str(i), "") for i in range(1, 6)]
    got = pending_migrations(ms, current_version=3)
    assert [m.version for m in got] == [4, 5]


def test_pending_returns_all_when_fresh_db():
    ms = [Migration(1, "a", ""), Migration(2, "b", "")]
    got = pending_migrations(ms, current_version=0)
    assert [m.version for m in got] == [1, 2]


# --- BIZ-P3-2 integrity check ---

def test_integrity_check_ok_row_passes():
    parse_integrity_check([("ok",)])


def test_integrity_check_corrupt_raises():
    """S-8: any row other than ('ok',) means refuse to open."""
    with pytest.raises(DatabaseCorrupt):
        parse_integrity_check([("*** in database main ***",)])


def test_integrity_check_empty_raises():
    with pytest.raises(DatabaseCorrupt):
        parse_integrity_check([])


# --- BIZ-P3-2 degraded write ---

def test_degraded_enter_and_leave():
    d = DegradedWriteMode()
    d.enter("disk_full")
    assert d.active and d.reason == "disk_full"
    d.leave()
    assert not d.active and d.reason == ""


def test_degraded_enter_is_idempotent():
    """Second enter must not overwrite the first reason (the first
    is the true cause; latter causes are downstream)."""
    d = DegradedWriteMode()
    d.enter("disk_full")
    d.enter("io_error")
    assert d.reason == "disk_full"


def test_degraded_buffer_cap_raises():
    """S-6: at buffer cap, escalate (raise) rather than swallow."""
    d = DegradedWriteMode(buffer_cap=2)
    d.enter("disk_full")
    d.record_buffered()
    d.record_buffered()
    with pytest.raises(RuntimeError, match="buffer full"):
        d.record_buffered()
