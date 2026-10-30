"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_event_stats.py
Brief: state/event/stats counts record.db rows per sev for the site-local day and the last 7 days, names the latest alarm / fault, and is never a guessed zero
Description:
User ruling 2026-10-30 (99 U103): p5 publishes an event-statistics state so p4
can answer "报警了几次" / "最近都发生了什么" / "上次警报". These cases build a
real record.db (the production DDL) in a temp dir and read it back:
  * the day boundary is local midnight of the site zone (Asia/Shanghai
    00:00 = 16:00 UTC the day before), not UTC midnight;
  * last_7d is today plus the six days before; older rows are out;
  * every sev is reported, 0 when absent; last_alarm / last_fault are the
    newest row of that sev;
  * a missing / unreadable database answers None (do not publish), and the
    reader never writes (mode=ro).
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import sqlite3  # the temp record.db
from datetime import datetime, timezone  # stamps

import pytest  # fixtures, marks

from xbrain.p5_gateway.event.stats import EventStats, day_start_utc  # under test
from xbrain.p5_gateway.persistence.schema_record import DDL_EVENTS  # production schema

pytestmark = pytest.mark.no_device  # pure in-process

TZ = "Asia/Shanghai"  # the site zone
NOW = datetime(2026, 10, 30, 11, 38, 46, tzinfo=timezone.utc).timestamp()  # 19:38:46 CST


def _stamp(y, mo, d, h, mi=0):  # a detected_at string (UTC), as p5 writes it
    return "%04d-%02d-%02d %02d:%02d:00" % (y, mo, d, h, mi)  # STAMP_FMT


def _db(tmp_path, rows):  # a record.db with the given (sev, cat, detected_at) rows
    path = str(tmp_path / "record.db")  # temp file
    c = sqlite3.connect(path)  # the writer, like p5's EventSubsystem
    c.execute("PRAGMA journal_mode=WAL")  # production runs WAL
    c.execute(DDL_EVENTS)  # the real table
    for i, (sev, cat, at) in enumerate(rows):  # insert in order: id grows
        c.execute("INSERT INTO events (eid, rid, channel, ch_seq, sev, cat, title, detail, src, ts,"
                  " detected_at, created_at, need_ack) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
                  ("e%d" % i, "gj-001", "normal", i, sev, cat, "t", "{}", "test", 0.0, at, at, 0))  # one row
    c.commit()  # visible to readers
    c.close()  # released
    return path  # the db


def test_day_boundary_is_local_midnight():
    """MUTATION: drop .astimezone(tz) in day_start_utc -> UTC midnight -> fails."""
    assert day_start_utc(NOW, TZ) == "2026-10-29 16:00:00"  # 2026-10-30 00:00 CST
    assert day_start_utc(NOW, TZ, 6) == "2026-10-23 16:00:00"  # six days earlier
    assert day_start_utc(NOW, "UTC") == "2026-10-30 00:00:00"  # the zone matters


def test_counts_today_and_last_7d(tmp_path):
    """MUTATION: count last_7d from day_start_utc(..., 7) -> the 8th day leaks in -> fails."""
    rows = [("alarm", "comm", _stamp(2026, 10, 30, 8, 32)),   # today 16:32 CST
            ("warn", "perception", _stamp(2026, 10, 29, 16, 0)),  # today 00:00 CST exactly
            ("alarm", "intrusion", _stamp(2026, 10, 29, 15, 59)),  # yesterday 23:59 CST
            ("fault", "bit", _stamp(2026, 10, 23, 16, 0)),   # 7th day back, 00:00 -> in
            ("fault", "bit", _stamp(2026, 10, 23, 15, 59)),  # 8th day -> out
            ("info", "task", _stamp(2026, 10, 30, 11, 0))]   # today
    got = EventStats(_db(tmp_path, rows), TZ).read(NOW)  # one read
    assert got["day"] == "2026-10-30" and got["tz"] == TZ and got["total"] == 6  # header
    assert got["today"] == {"info": 1, "warn": 1, "alarm": 1, "fault": 0}  # since local midnight
    assert got["last_7d"] == {"info": 1, "warn": 1, "alarm": 2, "fault": 1}  # 7 local days


def test_last_alarm_and_fault_are_the_newest_rows(tmp_path):
    """MUTATION: ORDER BY id ASC -> the oldest alarm is reported -> fails."""
    rows = [("alarm", "intrusion", _stamp(2026, 10, 20, 1)), ("alarm", "comm", _stamp(2026, 10, 30, 8, 32)),
            ("fault", "bit", _stamp(2026, 10, 28, 4, 29))]  # two alarms, one fault
    got = EventStats(_db(tmp_path, rows), TZ).read(NOW)
    at = datetime(2026, 10, 30, 8, 32, tzinfo=timezone.utc).timestamp()  # the newest alarm
    assert got["last_alarm"] == {"cat": "comm", "at": at}  # newest, UTC epoch
    assert got["last_fault"]["cat"] == "bit"


def test_empty_table_reports_zeros_and_no_last(tmp_path):
    """Zeros here are a reading of an empty table, not a default."""
    got = EventStats(_db(tmp_path, []), TZ).read(NOW)
    assert got["today"] == {"info": 0, "warn": 0, "alarm": 0, "fault": 0}  # every sev present
    assert got["last_alarm"] is None and got["last_fault"] is None and got["total"] == 0


def test_missing_database_is_none_not_zero(tmp_path):
    """MUTATION: return the zero dict on sqlite errors -> "没有报警" when the db is gone -> fails."""
    assert EventStats(str(tmp_path / "absent.db"), TZ).read(NOW) is None  # mode=ro never creates it
    assert not (tmp_path / "absent.db").exists()  # and nothing was created


def test_reader_is_read_only(tmp_path):
    """MUTATION: open without mode=ro -> the write below succeeds -> fails."""
    st = EventStats(_db(tmp_path, []), TZ)  # reader
    st.read(NOW)  # opens the connection
    with pytest.raises(sqlite3.OperationalError):  # attempt to write through it
        st._connect().execute("DELETE FROM events")  # must be refused
    st.close()  # released


def test_recovers_after_an_error(tmp_path):
    """A failed read drops the handle; the next read reopens and works."""
    path = _db(tmp_path, [("alarm", "comm", _stamp(2026, 10, 30, 8))])  # one alarm today
    st = EventStats(path, TZ)  # reader
    st._conn = sqlite3.connect(":memory:")  # a handle without the table -> sqlite error
    assert st.read(NOW) is None  # error -> no publish
    assert st.read(NOW)["today"]["alarm"] == 1  # reopened on the real file
