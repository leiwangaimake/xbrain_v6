"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: schema.py
Brief: GWY-P5-02 record.db events + event_cursor + SEQ-1..3 + JSONL degrade

Description:
17 S4 events table and cursor:

  events           append-only log; ONE ROW per event.
  event_cursor     one row per downstream consumer: cloud, hmi;
                   tracks the last delivered event_seq for replay.

  SEQ-1  event_seq is a monotonic increasing INTEGER (autoincrement)
  SEQ-2  event_seq is dense (no gaps) within a single p5 process
         lifetime; a gap after restart means the crash lost the
         in-flight batch (documented behaviour, cursor rewinds)
  SEQ-3  each downstream's cursor advances by <= 1 per delivery
         success; concurrent readers must serialise their cursor
         update

JSONL degrade: when DB writes cannot complete (disk full etc.),
events are appended to a rolling JSONL file so nothing is lost.
On recovery the JSONL file is replayed into the DB.
"""

from __future__ import annotations


DDL_EVENTS = """
CREATE TABLE IF NOT EXISTS events (
  event_seq     INTEGER PRIMARY KEY AUTOINCREMENT,
  event_id      TEXT NOT NULL,
  source        TEXT NOT NULL,
  category      TEXT NOT NULL,
  level         TEXT NOT NULL CHECK (level IN ('info','warn','error')),
  payload_json  TEXT NOT NULL,
  received_ms   INTEGER NOT NULL,
  UNIQUE (source, event_id)
);
""".strip()


DDL_EVENT_CURSOR = """
CREATE TABLE IF NOT EXISTS event_cursor (
  consumer      TEXT PRIMARY KEY,
  last_seq      INTEGER NOT NULL,
  updated_ms    INTEGER NOT NULL
);
""".strip()


ALL_EVENT_STATEMENTS = (DDL_EVENTS, DDL_EVENT_CURSOR)


CONSUMERS = frozenset({"cloud", "hmi"})


class UnknownConsumer(Exception):
    pass


class SeqOrderViolation(Exception):
    pass


def validate_consumer(consumer: str) -> None:
    if consumer not in CONSUMERS:
        raise UnknownConsumer(consumer)


def advance_cursor(current: int, next_seq: int) -> int:
    """SEQ-3: cursor advances by exactly 1 (or stays put).
    A jump of >1 means a delivery batch skipped seqs -- refuse."""
    if next_seq < current:
        raise SeqOrderViolation(
            f"cursor rewind: {current} -> {next_seq}")
    if next_seq - current > 1:
        raise SeqOrderViolation(
            f"cursor gap: {current} -> {next_seq}")
    return next_seq
