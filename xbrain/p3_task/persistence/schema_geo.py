"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: schema_geo.py
Brief: BIZ-P3-3 geo.db + fence.db DDL + 5/16 quota triggers + geo_object

Description:
15 S9 splits geographical data into TWO physical files:
  * geo.db   waypoints / routes / assoc / docks
  * fence.db fences + zone tags
This split lets fences reload independently of route edits and lets
fence-only reads open a read-only handle without carrying the whole
geo tree.

Quotas (15 S9):
  * waypoints per route <= 16
  * docks total          <= 5
Both enforced with sqlite triggers so a rogue writer or a bulk
import script cannot exceed them silently.

Every geo_object row (waypoints / routes / docks / fences) carries
rev (monotonic per object) + content_hash + tombstone. The rev
column is the anchor for SN-5 dedupe and for outbound push
ordering; see BIZ-P3-27 for the invariant work.
"""

from __future__ import annotations


DDL_WAYPOINTS = """
CREATE TABLE IF NOT EXISTS waypoints (
  waypoint_id   TEXT PRIMARY KEY,
  name          TEXT,
  x_m           REAL NOT NULL,
  y_m           REAL NOT NULL,
  heading_rad   REAL,
  rev           INTEGER NOT NULL DEFAULT 1,
  content_hash  TEXT NOT NULL,
  tombstone     INTEGER NOT NULL DEFAULT 0 CHECK (tombstone IN (0,1)),
  updated_ms    INTEGER NOT NULL
);
""".strip()
# name: the keypoint's display name (11 S7.8.2, "东门岗亭"), for the HMI keypoint
# label + state/geo/objects broadcast (11 S7.10A). Nullable here because this
# runtime schema was the minimal early cut (15 S9.3 has it NOT NULL); F06
# record_waypoint populates it, and geo objects with no name render as an
# unlabelled point (GO-4), never a fabricated one.


DDL_ROUTES = """
CREATE TABLE IF NOT EXISTS routes (
  route_id      TEXT PRIMARY KEY,
  name          TEXT NOT NULL,
  rev           INTEGER NOT NULL DEFAULT 1,
  content_hash  TEXT NOT NULL,
  tombstone     INTEGER NOT NULL DEFAULT 0 CHECK (tombstone IN (0,1)),
  updated_ms    INTEGER NOT NULL
);
""".strip()


DDL_ROUTE_WAYPOINT_ASSOC = """
CREATE TABLE IF NOT EXISTS route_waypoint_assoc (
  route_id      TEXT NOT NULL REFERENCES routes(route_id),
  seq           INTEGER NOT NULL,
  waypoint_id   TEXT NOT NULL REFERENCES waypoints(waypoint_id),
  PRIMARY KEY (route_id, seq)
);
""".strip()


DDL_DOCKS = """
CREATE TABLE IF NOT EXISTS docks (
  dock_id       TEXT PRIMARY KEY,
  name          TEXT,
  x_m           REAL NOT NULL,
  y_m           REAL NOT NULL,
  heading_rad   REAL NOT NULL,
  rev           INTEGER NOT NULL DEFAULT 1,
  content_hash  TEXT NOT NULL,
  tombstone     INTEGER NOT NULL DEFAULT 0 CHECK (tombstone IN (0,1)),
  updated_ms    INTEGER NOT NULL
);
""".strip()


TRIGGER_WAYPOINT_QUOTA = """
CREATE TRIGGER IF NOT EXISTS trg_waypoint_quota
BEFORE INSERT ON route_waypoint_assoc
BEGIN
  SELECT RAISE(ABORT, 'waypoint quota per route exceeded (max 16)')
   WHERE (SELECT COUNT(*) FROM route_waypoint_assoc
           WHERE route_id = NEW.route_id) >= 16;
END;
""".strip()


TRIGGER_DOCK_QUOTA = """
CREATE TRIGGER IF NOT EXISTS trg_dock_quota
BEFORE INSERT ON docks
BEGIN
  SELECT RAISE(ABORT, 'dock quota exceeded (max 5)')
   WHERE (SELECT COUNT(*) FROM docks WHERE tombstone = 0) >= 5;
END;
""".strip()


DDL_FENCES = """
CREATE TABLE IF NOT EXISTS fences (
  fence_id      TEXT PRIMARY KEY,
  kind          TEXT NOT NULL CHECK (kind IN ('polygon','circle','zone')),
  geom_json     TEXT NOT NULL,
  zone_label    TEXT,
  rev           INTEGER NOT NULL DEFAULT 1,
  content_hash  TEXT NOT NULL,
  tombstone     INTEGER NOT NULL DEFAULT 0 CHECK (tombstone IN (0,1)),
  updated_ms    INTEGER NOT NULL
);
""".strip()


GEO_DB_STATEMENTS = (
    DDL_WAYPOINTS,
    DDL_ROUTES,
    DDL_ROUTE_WAYPOINT_ASSOC,
    DDL_DOCKS,
    TRIGGER_WAYPOINT_QUOTA,
    TRIGGER_DOCK_QUOTA,
)


FENCE_DB_STATEMENTS = (
    DDL_FENCES,
)


DDL_COMMANDS = """
CREATE TABLE IF NOT EXISTS commands (
  cmd_seq       INTEGER PRIMARY KEY AUTOINCREMENT,
  category      TEXT NOT NULL,
  scope         TEXT NOT NULL,
  payload_json  TEXT NOT NULL,
  origin        TEXT NOT NULL,
  received_ms   INTEGER NOT NULL,
  applied_ms    INTEGER,
  result_code   TEXT
);
""".strip()

RECORD_DB_STATEMENTS = (DDL_COMMANDS,)
