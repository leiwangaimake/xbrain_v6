"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: schema_geo.py
Brief: BIZ-P3-3 geo.db + fence.db DDL + dock quota trigger + geo_object

Description:
15 S9 splits geographical data into TWO physical files:
  * geo.db   waypoints / routes / assoc / docks
  * fence.db fences (role: allow/forbid/speed_limit/warning, 11 S9A.2)
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
fences now carry a ROLE (allow/forbid/speed_limit/warning, 11 S9A.2) with the
S9A.1A count triggers (<= 5 active, at most 1 allow); vertices are WGS84.
STILL interim (NOT yet 15 S9.3): docks stay ENU (charging-subsystem ripple,
out of scope for this migration).
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


# v1.5 (2026-08-20 PLAN A / fence runtime): fences carry a ROLE (11 S9A.2), NOT
# the old kind='zone' overload. role = allow | forbid | speed_limit | warning;
# kind is the GEOMETRY type (polygon | circle). Vertices in geom_json are WGS84
# (11 S9A.2 line: "WGS84 十进制度 float64"), matching the geo WGS84 migration.
# warning fences never hard-enforce (S9A.2: role=warning hard_enforce always 0).
DDL_FENCES = """
CREATE TABLE IF NOT EXISTS fences (
  fence_id      TEXT PRIMARY KEY,                -- 'f-'+slug (11 S7.8.1)
  name          TEXT,                            -- display name (11 S9A.2 polygons[].name); nullable
  role          TEXT NOT NULL,                   -- allow | forbid | speed_limit | warning
  kind          TEXT NOT NULL DEFAULT 'polygon', -- polygon | circle (geometry type)
  geom_json     TEXT NOT NULL,                   -- WGS84 vertices (11 S9A.2)
  hard_enforce  INTEGER NOT NULL DEFAULT 1 CHECK (hard_enforce IN (0,1)),
  soft_margin_m REAL,
  rev           INTEGER NOT NULL DEFAULT 1,
  content_hash  TEXT NOT NULL,
  tombstone     INTEGER NOT NULL DEFAULT 0 CHECK (tombstone IN (0,1)),
  updated_ms    INTEGER NOT NULL,
  CHECK (role IN ('allow','forbid','speed_limit','warning')),
  CHECK (kind IN ('polygon','circle')),
  CHECK (role <> 'warning' OR hard_enforce = 0),   -- 11 S9A.2: warning never hard-enforces
  CHECK (fence_id GLOB 'f-*')
);
""".strip()


# 11 S9A.1A operating rules enforced at the DB so a bulk import cannot violate them:
#   total active <= 5 (S9A.1)  +  at most ONE active allow (S9A.1A FS-5A upper half).
# The "at least one allow" half (FS-5A lower) is a SET invariant, not a single-row
# one, so it is checked at FenceSet build/broadcast time (validate_active_fence_set
# in fence/geom.py) -- a per-row INSERT trigger cannot assert existence.
TRIGGER_FENCE_TOTAL_QUOTA = """
CREATE TRIGGER IF NOT EXISTS trg_fence_total_quota
BEFORE INSERT ON fences
BEGIN
  SELECT RAISE(ABORT, 'fence quota exceeded (max 5 active, 11 S9A.1)')
   WHERE (SELECT COUNT(*) FROM fences WHERE tombstone = 0) >= 5;
END;
""".strip()

TRIGGER_FENCE_SINGLE_ALLOW = """
CREATE TRIGGER IF NOT EXISTS trg_fence_single_allow
BEFORE INSERT ON fences
WHEN NEW.role = 'allow'
BEGIN
  SELECT RAISE(ABORT, 'duplicate allow fence (11 S9A.1A: exactly 1)')
   WHERE (SELECT COUNT(*) FROM fences WHERE tombstone = 0 AND role = 'allow') >= 1;
END;
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
    TRIGGER_FENCE_TOTAL_QUOTA,
    TRIGGER_FENCE_SINGLE_ALLOW,
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
