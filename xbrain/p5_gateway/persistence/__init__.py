"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: xbrain.p5_gateway.persistence -- record.db (the 4th DB, 15 S9.0) layer

Description:
The problem this package solves: p5_gateway is the SOLE writer of record.db
(15 S9.10), the durable home for the event log + delivery ledger. Before this
package existed, the running p5 dropped every event into a 50-entry in-memory
ring and nothing survived a restart or a network outage -- the store-and-forward
that the cloud backfill (17 S3.5) is built on had no store.

What lives here (mirrors p3_task/persistence, which owns task/fence/geo.db):
  base.py           connection + PRAGMA discipline (15 S9.1): the two-writer
                    (NORMAL default + FULL for alarm/fault, FS-d) + one reader
                    (query_only) model, aiosqlite only (S-1, no sync sqlite3).
  schema_record.py  the events + event_cursor DDL -- the 17 S3.4 authoritative
                    schema (channel/ch_seq dual-cursor, NOT the old per-consumer
                    placeholder), plus the ch_seq / confirmed_upto cursor rules.
  record_dao.py     the DAO: insert (ch_seq allocation + dedup merge + need_ack),
                    backfill query, mark_delivered, cursor advance, JSONL degrade.

Boundary: this package NEVER decides event semantics (severity, channel, dedup
policy) -- those come from the producer + 11 S6.2 and are handed in. It only
persists and serves what it is given, so a bug here cannot fabricate an event or
silently reclassify one.
"""
