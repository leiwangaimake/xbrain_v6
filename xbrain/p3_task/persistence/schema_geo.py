"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: schema_geo.py
Brief: BIZ-P3-3 geo.db + fence.db DDL + dock quota trigger + geo_object

Description:
15 S9 splits geographical data into TWO physical files:
  * geo.db   waypoints / routes / assoc / docks
  * fence.db fences + zone tags
This split lets fences reload independently of route edits and lets
fence-only reads open a read-only handle without carrying the whole
geo tree.

Quotas (15 S9):
  * docks total <= 5  (sqlite trigger, so a bulk import cannot exceed it)
The old "waypoints per route <= 16" trigger is retired: route geometry is now
INLINE (path_points), capped in commit_route (11 S7.8.3 <= 5000), not by an
assoc-row trigger.

Every geo_object row (waypoints / routes / docks / fences) carries
rev (monotonic per object) + content_hash + tombstone. The rev
column is the anchor for SN-5 dedupe and for outbound push
ordering; see BIZ-P3-27 for the invariant work.

ALIGNMENT (v1.5 / 2026-08-20, PLAN A EXECUTED -- see 15 S9.3.0):
waypoints / routes / route_waypoint_assoc are now the 15 S9.3 model:
  * WGS84 (rtk_lat/lon) instead of ENU x_m/y_m.
  * route geometry INLINE on routes (path_points JSON mode B, or waypoint_ids
    JSON mode A, XOR) -- NOT an ordered route_waypoint_assoc(seq) vertex list.
  * waypoints are NAMED keypoints only (route vertices no longer live here).
  * route_waypoint_assoc restored to its 15 S9.3 PROXIMITY meaning
    (min_distance_m / nearest_idx), UNPOPULATED until F06 voice nav wires it.
  * sync columns rev / content_hash / tombstone / updated_ms ADDED on top
    (15 S9.3 DDL omits them; state/geo/objects needs them).
STILL interim (NOT yet 15 S9.3): docks stay ENU (charging-subsystem ripple,
out of scope). fences (kind vs role) go with the fence runtime batch, NOT here.
"""

from __future__ import annotations


DDL_WAYPOINTS = """
CREATE TABLE IF NOT EXISTS waypoints (
  id             INTEGER PRIMARY KEY AUTOINCREMENT,
  geo_id         TEXT NOT NULL UNIQUE,          -- 'w-'+slug, immutable (11 S7.8.1)
  name           TEXT NOT NULL UNIQUE,          -- named keypoints ONLY; route vertices are INLINE on routes
  type           TEXT NOT NULL,                 -- poi | dock | home | gate | ...
  rtk_lat        REAL NOT NULL,                 -- WGS84
  rtk_lon        REAL NOT NULL,
  rtk_alt        REAL,
  yaw_deg        REAL,                          -- heading at capture (true north, CW+)
  arrival_radius REAL NOT NULL DEFAULT 1.0,     -- per-point, not a global constant
  description    TEXT,
  rev            INTEGER NOT NULL DEFAULT 1,
  content_hash   TEXT NOT NULL,
  tombstone      INTEGER NOT NULL DEFAULT 0 CHECK (tombstone IN (0,1)),
  updated_ms     INTEGER NOT NULL,
  CHECK (geo_id GLOB 'w-*')
);
""".strip()
# v1.5 (2026-08-20 PLAN A): raised to the 15 S9.3 model -- WGS84 (rtk_lat/lon),
# NAMED keypoints only (route geometry moved INLINE onto routes), name NOT NULL
# UNIQUE (voice "go to <name>" needs a unique target). Sync columns rev /
# content_hash / tombstone / updated_ms are ADDED on top of the 15 S9.3 DDL
# (which omits them) because state/geo/objects needs catalog_rev + a tombstone
# filter. id + geo_id UNIQUE matches 15 S9.3; GeoObjectDAO looks the row up by
# the UNIQUE geo_id, so a separate AUTOINCREMENT id costs the DAO nothing.


DDL_ROUTES = """
CREATE TABLE IF NOT EXISTS routes (
  id            INTEGER PRIMARY KEY AUTOINCREMENT,
  geo_id        TEXT NOT NULL UNIQUE,           -- 'r-'+slug, immutable (11 S7.8.1)
  name          TEXT NOT NULL UNIQUE,
  waypoint_ids  TEXT,                           -- JSON [geo_id,...]  mode A: named anchors (config import)
  path_points   TEXT,                           -- JSON [[lat,lon],...] mode B: voice recording
  max_speed     REAL,
  loop_mode     TEXT NOT NULL DEFAULT 'oneway', -- oneway | pingpong | closed (NAV-22)
  direction     TEXT NOT NULL DEFAULT 'forward',-- forward | reverse (first entry direction)
  total_len_m   REAL,                           -- computed at commit (11 S7.8.3)
  description   TEXT,
  rev           INTEGER NOT NULL DEFAULT 1,
  content_hash  TEXT NOT NULL,
  tombstone     INTEGER NOT NULL DEFAULT 0 CHECK (tombstone IN (0,1)),
  updated_ms    INTEGER NOT NULL,
  CHECK ((waypoint_ids IS NULL) <> (path_points IS NULL)),   -- XOR: exactly one geometry
  CHECK (loop_mode IN ('oneway','pingpong','closed')),
  CHECK (direction IN ('forward','reverse')),
  CHECK (total_len_m IS NULL OR total_len_m > 0.0),
  CHECK (geo_id GLOB 'r-*')
);
""".strip()
# v1.5: route geometry is now INLINE (path_points OR waypoint_ids, XOR), NOT the
# old route_waypoint_assoc(seq) vertex list. This is the model flip of 15 S9.3.


# 15 S9.3 semantics: NOT the route geometry (that is INLINE on routes now), but
# the keypoint<->route PROXIMITY relation -- a keypoint within min_distance_m of a
# route, for "go to the Nth point" / "which route is this keypoint on" voice nav.
# Left UNPOPULATED by commit_route for now (a derived relation, wired with F06
# voice nav); the table exists so the read side never has to ALTER it in later.
DDL_ROUTE_WAYPOINT_ASSOC = """
CREATE TABLE IF NOT EXISTS route_waypoint_assoc (
  id             INTEGER PRIMARY KEY AUTOINCREMENT,
  route_id       TEXT NOT NULL REFERENCES routes(geo_id),
  waypoint_id    TEXT NOT NULL REFERENCES waypoints(geo_id),
  min_distance_m REAL NOT NULL,                 -- keypoint to route min distance; anchor mode = 0
  nearest_idx    INTEGER NOT NULL,              -- index of the nearest path_points vertex
  UNIQUE (route_id, waypoint_id)
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
# NOTE (v1.5): docks are DELIBERATELY still the ENU interim schema. The full 15
# S9.3 dock model (WGS84 + handover point) ripples into the charging subsystem
# (dock_select / dock_arbiter), out of scope for the geo/HMI migration. docks are
# not rendered on the HMI map, so this ENU-vs-WGS84 mix is invisible there.


# The old "16 waypoints per route" trigger is GONE: in the 15 S9.3 model route
# geometry is INLINE (path_points, capped in commit_route, 11 S7.8.3 <= 5000),
# and route_waypoint_assoc is now proximity (no natural per-route cap).
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
