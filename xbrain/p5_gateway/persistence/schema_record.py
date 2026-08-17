"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: schema_record.py
Brief: record.db events + event_cursor DDL (the 17 S3.4 authoritative schema)

Description:
This is the ONE authoritative definition of the record.db events table (17 S3.4
says so verbatim: "本表是 events 的唯一权威定义"). It replaces the earlier
placeholder (event_seq / source / level in {info,warn,error} / per-consumer
cursor) that never matched the contract: real events carry a channel (normal|
alarm) with a per-channel monotonic ch_seq, a 4-value sev (info|warn|alarm|
fault), the 11 S6.2 category, a location snapshot, dedup + delivery bookkeeping.

Why the dual "partial index" instead of one (delivered, sev, seq) index
(17 S3.4): the two backfill cursors (alarm / normal, U18) must not block each
other, and a delivered row must LEAVE the index so its size tracks the backlog,
not history. idx_evt_bf_alarm / idx_evt_bf_normal each hold only their channel's
delivered=0 rows -- two physically separate B-trees, one cursor each.

The three ch_seq rules this schema enforces (17 S3.4 SEQ-1..3), and why:
  SEQ-1  ch_seq is the P5-assigned in-DB sequence, NOT the S3.0 envelope seq
         (which is per-key and resets on restart -- using it as the backfill
         cursor would replay all history after any restart).
  SEQ-2  ch_seq must be dense (no gaps) within a channel; allocation + insert
         happen in ONE BEGIN IMMEDIATE tx (record_dao), so the cloud can use a
         missing number to detect loss.
  SEQ-3  ch_seq does not reset across restart: startup sets next_ch_seq =
         MAX(ch_seq)+1 from the table; event_cursor MUST survive archive/rebuild.

This module holds ONLY the DDL + the pure cursor rule. No connection, no I/O --
those are base.py / record_dao.py, so the schema is testable against a bare
in-memory connection (CLAUDE.md 7.2).
"""

from __future__ import annotations


# The events table -- 17 S3.4 verbatim (SQL comments kept in English per 2.1).
# foreign_keys is OFF at the connection (15 S9.1 DBF-2): every FK-like column
# below is a SOFT reference, so a dangling trace_id/task_id never blocks a write.
DDL_EVENTS = """
CREATE TABLE IF NOT EXISTS events (
    id             INTEGER PRIMARY KEY AUTOINCREMENT,
    eid            TEXT    NOT NULL UNIQUE,          -- = 11 S6.1 Event.eid
    rid            TEXT    NOT NULL,                 -- self-carried: record.db archives standalone

    -- (1) dual channel (U18 / U18a) -- hardcoded per category in 11 S6.2
    channel        TEXT    NOT NULL
                   CHECK (channel IN ('normal','alarm')),
    ch_seq         INTEGER NOT NULL,                 -- per-channel, dense, never resets

    -- (2) classification
    sev            TEXT    NOT NULL
                   CHECK (sev IN ('info','warn','alarm','fault')),
    cat            TEXT    NOT NULL,                 -- 11 S6.2 closed set (23 values)
    title          TEXT    NOT NULL,
    detail         TEXT    NOT NULL,                 -- JSON, per-category detail shape
    src            TEXT    NOT NULL,                 -- producing process (envelope src)

    -- (3) associations (FK OFF, all soft refs -- 15 S9.1 DBF-2)
    trace_id       TEXT,
    task_id        TEXT,
    episode_id     INTEGER,                          -- paired-event key (11 S9A.9 E-2)

    -- (4) time (15 S9.1 DBF-3): ts is display-only, NEVER an ordering key
    ts             REAL    NOT NULL,
    ts_sync        INTEGER NOT NULL DEFAULT 0,       -- 0 = ts untrustworthy (11 S1.5.5)
    detected_at    TEXT    NOT NULL,                 -- local human-readable evidence time
    created_at     TEXT    NOT NULL,                 -- UTC write time

    -- (5) on-scene snapshot: the event carries "where it happened" itself
    location_lat   REAL, location_lon REAL, location_alt REAL,
    pose_x         REAL, pose_y REAL,
    heading_deg    REAL,
    heading_src    TEXT,                             -- dual_antenna|cog|none (U34)
    fix_type       TEXT,

    -- (6) dedup (EVT-14)
    dedup_key      TEXT,
    dedup_window_s INTEGER,
    dedup_count    INTEGER NOT NULL DEFAULT 1,
    last_ts        REAL,

    -- (7) delivery (EVT-13): need_ack computed at insert (S3.3 union), frozen
    need_ack       INTEGER NOT NULL,
    delivered      INTEGER NOT NULL DEFAULT 0,       -- 0 pending | 1 delivered | -1 given up
    deliver_tries  INTEGER NOT NULL DEFAULT 0,
    delivered_at   TEXT,
    backfill_batch TEXT,                             -- which batch pushed it, for reconciliation

    -- (8) media references only (EVT-15) -- delivery STATE lives in the delivery table
    media_json     TEXT
);
""".strip()


# Two partial indexes = the two backfill cursors, physically separate (17 S3.4).
# A row leaves its index the instant delivered != 0, so index size ~ backlog.
DDL_EVENT_INDEXES = (
    "CREATE INDEX IF NOT EXISTS idx_evt_bf_alarm  ON events(ch_seq) "
    "WHERE channel = 'alarm'  AND delivered = 0;",
    "CREATE INDEX IF NOT EXISTS idx_evt_bf_normal ON events(ch_seq) "
    "WHERE channel = 'normal' AND delivered = 0;",
    "CREATE UNIQUE INDEX IF NOT EXISTS idx_evt_chseq ON events(channel, ch_seq);",
    "CREATE INDEX IF NOT EXISTS idx_evt_dedup ON events(dedup_key, last_ts) "
    "WHERE dedup_key IS NOT NULL;",
    "CREATE INDEX IF NOT EXISTS idx_evt_trace ON events(trace_id) "
    "WHERE trace_id IS NOT NULL;",
    "CREATE INDEX IF NOT EXISTS idx_evt_time ON events(detected_at DESC);",
)


# The cursor table: the ONLY cross-restart ordering anchor (17 S3.4 SEQ-3). It
# must survive archive/rebuild or the cloud will discard new events as old ones.
DDL_EVENT_CURSOR = """
CREATE TABLE IF NOT EXISTS event_cursor (
    channel          TEXT PRIMARY KEY
                     CHECK (channel IN ('normal','alarm')),
    next_ch_seq      INTEGER NOT NULL DEFAULT 1,     -- next number to hand out
    confirmed_upto   INTEGER NOT NULL DEFAULT 0,     -- cloud-confirmed contiguous high-water
    last_backfill_at TEXT
);
""".strip()

# Seed the two cursor rows (idempotent -- IF NOT EXISTS above lets this re-run).
DDL_EVENT_CURSOR_SEED = (
    "INSERT OR IGNORE INTO event_cursor(channel) VALUES ('normal'), ('alarm');"
)


# Applied in order at connection open. Table -> indexes -> cursor -> seed.
ALL_RECORD_STATEMENTS = (
    (DDL_EVENTS,)
    + DDL_EVENT_INDEXES
    + (DDL_EVENT_CURSOR, DDL_EVENT_CURSOR_SEED)
)


# The two channels, mirroring the CHECK constraints above. A value outside this
# set must raise, never be silently coerced (11 S13.6 closed-set discipline).
CHANNELS = frozenset({"normal", "alarm"})
# The four severities (11 S6.1). Kept here so the DAO can validate before insert
# rather than relying on the CHECK to raise a less specific error mid-transaction.
SEVERITIES = frozenset({"info", "warn", "alarm", "fault"})


class SeqOrderViolation(Exception):
    """A cursor was asked to move backwards (17 S3.4 SEQ-3). Rewinding
    confirmed_upto would make the cloud re-accept and then discard events it
    already has, so this is refused, not warned."""


def need_ack(channel: str, sev: str) -> bool:
    """17 S3.3 (2): need ack iff channel is the alarm lane OR sev is alarm/fault
    -- the UNION, not just sev. The union closes the left-bottom hole: a
    fence.recovered is sev=info but rides channel=alarm (11 S9A.9 E-1), and if it
    were best-effort it could be dropped, leaving the cloud stuck in alarm state
    forever. Computed once at insert and frozen into the column (S3.3), so
    backfill never re-judges it.
    """
    if channel not in CHANNELS:
        raise ValueError(f"channel not in {CHANNELS}: {channel!r}")
    if sev not in SEVERITIES:
        raise ValueError(f"sev not in {SEVERITIES}: {sev!r}")
    return channel == "alarm" or sev in ("alarm", "fault")


def advance_confirmed(current: int, acked_upto: int) -> int:
    """SEQ-3: confirmed_upto is a monotonic high-water mark. The cloud acks
    "I have everything up to N"; we move confirmed_upto to N. An ack BELOW the
    current mark is a rewind -- refused, because acting on it would re-queue rows
    the cloud already confirmed. An equal ack is a no-op (idempotent delivery)."""
    if acked_upto < current:
        raise SeqOrderViolation(
            f"confirmed_upto rewind: {current} -> {acked_upto}")
    return acked_upto
