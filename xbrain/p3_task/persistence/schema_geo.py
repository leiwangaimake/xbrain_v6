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
docks are now the FULL 15 S9.3 model too (WGS84 + handover point + num/on_route),
so the whole geo tree is WGS84 (charging is unwired, so no integration ripple).
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
  num            INTEGER,                       -- 1..99 voice "3 hao", unique per type (11 S7.8.2)
  alias_json     TEXT NOT NULL DEFAULT '[]',    -- JSON [str] ASR L3 synonyms (11 S7.8.2)
  state          TEXT NOT NULL DEFAULT 'active',-- draft|active|disabled|deleted (11 S7.8.2)
  created_by     TEXT NOT NULL DEFAULT 'factory',
  updated_by     TEXT NOT NULL DEFAULT 'factory',
  rev            INTEGER NOT NULL DEFAULT 1,
  content_hash   TEXT NOT NULL,
  tombstone      INTEGER NOT NULL DEFAULT 0 CHECK (tombstone IN (0,1)),
  updated_ms     INTEGER NOT NULL,
  CHECK (state IN ('draft','active','disabled','deleted')),
  CHECK (num IS NULL OR (num >= 1 AND num <= 99)),
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
  num           INTEGER,                        -- 1..99 voice "3 hao lu jing" (11 S7.8.2)
  alias_json    TEXT NOT NULL DEFAULT '[]',     -- JSON [str] ASR L3 synonyms
  state         TEXT NOT NULL DEFAULT 'active', -- draft|active|disabled|deleted
  created_by    TEXT NOT NULL DEFAULT 'factory',
  updated_by    TEXT NOT NULL DEFAULT 'factory',
  rev           INTEGER NOT NULL DEFAULT 1,
  content_hash  TEXT NOT NULL,
  tombstone     INTEGER NOT NULL DEFAULT 0 CHECK (tombstone IN (0,1)),
  updated_ms    INTEGER NOT NULL,
  CHECK (state IN ('draft','active','disabled','deleted')),
  CHECK (num IS NULL OR (num >= 1 AND num <= 99)),
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
  id                   INTEGER PRIMARY KEY AUTOINCREMENT,
  geo_id               TEXT NOT NULL UNIQUE,       -- 'd-'+slug, immutable (11 S7.8.1)
  name                 TEXT NOT NULL UNIQUE,       -- display / TTS name
  num                  INTEGER,                    -- 1..99, voice "2 hao zhuang"
  rtk_lat              REAL NOT NULL,              -- dock body (WGS84)
  rtk_lon              REAL NOT NULL,
  rtk_alt              REAL,
  dock_heading_rad     REAL NOT NULL,              -- dock body heading = approach dir
  handover_lat         REAL NOT NULL,              -- handover point (stage-1 nav target, CHG-35/36)
  handover_lon         REAL NOT NULL,
  handover_heading_rad REAL NOT NULL,
  handover_tol_m       REAL NOT NULL DEFAULT 0.3,  -- Q19 eng default, recalibrate later
  handover_tol_rad     REAL NOT NULL DEFAULT 0.09,
  on_route_json        TEXT NOT NULL DEFAULT '[]', -- JSON ["r-...",...] dock-select (CHG-02)
  enabled              INTEGER NOT NULL DEFAULT 1 CHECK (enabled IN (0,1)),  -- disable != delete (G-3)
  occupied_by          TEXT,                       -- fleet reserve (CHG-42); single robot = NULL
  description          TEXT,
  alias_json           TEXT NOT NULL DEFAULT '[]', -- JSON [str] ASR L3 synonyms (num already above)
  state                TEXT NOT NULL DEFAULT 'active',
  created_by           TEXT NOT NULL DEFAULT 'factory',
  updated_by           TEXT NOT NULL DEFAULT 'factory',
  rev                  INTEGER NOT NULL DEFAULT 1,  -- sync (added on top of 15 S9.3)
  content_hash         TEXT NOT NULL,
  tombstone            INTEGER NOT NULL DEFAULT 0 CHECK (tombstone IN (0,1)),
  updated_ms           INTEGER NOT NULL,
  CHECK (state IN ('draft','active','disabled','deleted')),
  CHECK (num IS NULL OR (num >= 1 AND num <= 99)),
  CHECK (handover_tol_m > 0.0 AND handover_tol_rad > 0.0),
  CHECK (geo_id GLOB 'd-*')
);
""".strip()
# v1.5 (2026-08-20): docks raised to the FULL 15 S9.3 model (WGS84 rtk_lat/lon +
# handover point + num/on_route/enabled/occupied_by), completing the geo WGS84
# migration. Safe because the charging subsystem (dock_select / dock_arbiter) is
# NOT wired into the runtime yet -- dock_select is a pure function on an ENU Dock
# dataclass, so its DB->Dock loader (a future charging batch) will project
# handover_lat/lon -> ENU there, not here. docks are keyed by geo_id like
# waypoints/routes; the sync columns are added on top of the 15 S9.3 DDL.


# The old "16 waypoints per route" trigger is GONE: in the 15 S9.3 model route
# geometry is INLINE (path_points, capped in commit_route, 11 S7.8.3 <= 5000),
# and route_waypoint_assoc is now proximity (no natural per-route cap).
# Quota triggers count LIVE rows, and "live" now means state='active' AND
# tombstone=0, not tombstone alone. A draft object is one the operator recorded
# but has not put into service (11 S7.8.2), and charging it against the quota
# would mean a recorded-but-unused dock blocks a real one.
#
# Every trigger below is DROPped before it is created (see the *_STATEMENTS
# tuples). CREATE TRIGGER IF NOT EXISTS on an EXISTING database keeps the OLD
# body silently -- the definition changed here would simply never take effect on
# any robot that has already run, and the symptom would be a quota enforced by
# yesterday's rule with nothing in any log to say so.
TRIGGER_DOCK_QUOTA = """
CREATE TRIGGER trg_dock_quota
BEFORE INSERT ON docks
WHEN NEW.state = 'active'
BEGIN
  SELECT RAISE(ABORT, 'dock quota exceeded (max 5)')
   WHERE (SELECT COUNT(*) FROM docks
           WHERE tombstone = 0 AND state = 'active') >= 5;
END;
""".strip()

# The UPDATE half. Without it, set_state can walk a sixth dock into service one
# state flip at a time -- the INSERT trigger only ever sees the row arrive.
TRIGGER_DOCK_QUOTA_UPD = """
CREATE TRIGGER trg_dock_quota_upd
BEFORE UPDATE OF state ON docks
WHEN NEW.state = 'active' AND OLD.state <> 'active'
BEGIN
  SELECT RAISE(ABORT, 'dock quota exceeded (max 5)')
   WHERE (SELECT COUNT(*) FROM docks
           WHERE tombstone = 0 AND state = 'active') >= 5;
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
  num           INTEGER,                        -- 1..99 voice "2 hao wei lan" (F15)
  alias_json    TEXT NOT NULL DEFAULT '[]',
  state         TEXT NOT NULL DEFAULT 'active', -- draft|active|disabled|deleted
  created_by    TEXT NOT NULL DEFAULT 'factory',
  updated_by    TEXT NOT NULL DEFAULT 'factory',
  rev           INTEGER NOT NULL DEFAULT 1,
  content_hash  TEXT NOT NULL,
  tombstone     INTEGER NOT NULL DEFAULT 0 CHECK (tombstone IN (0,1)),
  updated_ms    INTEGER NOT NULL,
  CHECK (state IN ('draft','active','disabled','deleted')),
  CHECK (num IS NULL OR (num >= 1 AND num <= 99)),
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
CREATE TRIGGER trg_fence_total_quota
BEFORE INSERT ON fences
WHEN NEW.state = 'active'
BEGIN
  SELECT RAISE(ABORT, 'fence quota exceeded (max 5 active, 11 S9A.1)')
   WHERE (SELECT COUNT(*) FROM fences
           WHERE tombstone = 0 AND state = 'active') >= 5;
END;
""".strip()

TRIGGER_FENCE_SINGLE_ALLOW = """
CREATE TRIGGER trg_fence_single_allow
BEFORE INSERT ON fences
WHEN NEW.role = 'allow' AND NEW.state = 'active'
BEGIN
  SELECT RAISE(ABORT, 'duplicate allow fence (11 S9A.1A: exactly 1)')
   WHERE (SELECT COUNT(*) FROM fences
           WHERE tombstone = 0 AND state = 'active' AND role = 'allow') >= 1;
END;
""".strip()

# The UPDATE halves. F15 (set_state fence -> active) is precisely the path that
# turns a stored draft into a live fence, so without these two the S9A.1A
# invariants hold at INSERT and are then walked past one activation at a time --
# and the second allow fence is the dangerous one: two keep-in polygons make the
# permitted area their union, quietly widening it.
TRIGGER_FENCE_TOTAL_QUOTA_UPD = """
CREATE TRIGGER trg_fence_total_quota_upd
BEFORE UPDATE OF state ON fences
WHEN NEW.state = 'active' AND OLD.state <> 'active'
BEGIN
  SELECT RAISE(ABORT, 'fence quota exceeded (max 5 active, 11 S9A.1)')
   WHERE (SELECT COUNT(*) FROM fences
           WHERE tombstone = 0 AND state = 'active') >= 5;
END;
""".strip()

TRIGGER_FENCE_SINGLE_ALLOW_UPD = """
CREATE TRIGGER trg_fence_single_allow_upd
BEFORE UPDATE OF state ON fences
WHEN NEW.role = 'allow' AND NEW.state = 'active' AND OLD.state <> 'active'
BEGIN
  SELECT RAISE(ABORT, 'duplicate allow fence (11 S9A.1A: exactly 1)')
   WHERE (SELECT COUNT(*) FROM fences
           WHERE tombstone = 0 AND state = 'active' AND role = 'allow') >= 1;
END;
""".strip()


# 11 S7.9.2 step 5: idempotency is keyed on cmd_id. A redelivered command (Zenoh
# does not guarantee exactly-once, C-5) must return the FIRST execution's result
# and must not bump rev a second time. The log lives in BOTH databases, not in one
# shared place, so the log row is written inside the SAME transaction as the
# mutation it records: a fence upsert writes fence.db only, and a log row in
# geo.db would be a second commit that can be lost independently -- after which
# the command looks unseen and gets applied twice.
DDL_GEO_CMD_LOG = """
CREATE TABLE IF NOT EXISTS geo_cmd_log (
  cmd_id      TEXT PRIMARY KEY,                -- 11 S7.9.1 sender-minted id
  action      TEXT NOT NULL,
  geo_id      TEXT,                            -- NULL for set-wide actions
  result      TEXT NOT NULL,                   -- accepted | rejected
  code        TEXT NOT NULL,                   -- OK or a closed-set E_*
  detail_json TEXT,                            -- the ack detail, replayed verbatim
  applied_ms  INTEGER NOT NULL
);
""".strip()


GEO_DB_STATEMENTS = (
    DDL_WAYPOINTS,
    DDL_ROUTES,
    DDL_ROUTE_WAYPOINT_ASSOC,
    DDL_DOCKS,
    DDL_GEO_CMD_LOG,
    # DROP before CREATE: see the comment above TRIGGER_DOCK_QUOTA. These run on
    # every open, so a trigger body edited here reaches an existing robot's db.
    "DROP TRIGGER IF EXISTS trg_dock_quota;",
    "DROP TRIGGER IF EXISTS trg_dock_quota_upd;",
    TRIGGER_DOCK_QUOTA,
    TRIGGER_DOCK_QUOTA_UPD,
)


FENCE_DB_STATEMENTS = (
    DDL_FENCES,
    DDL_GEO_CMD_LOG,
    "DROP TRIGGER IF EXISTS trg_fence_total_quota;",
    "DROP TRIGGER IF EXISTS trg_fence_single_allow;",
    "DROP TRIGGER IF EXISTS trg_fence_total_quota_upd;",
    "DROP TRIGGER IF EXISTS trg_fence_single_allow_upd;",
    TRIGGER_FENCE_TOTAL_QUOTA,
    TRIGGER_FENCE_SINGLE_ALLOW,
    TRIGGER_FENCE_TOTAL_QUOTA_UPD,
    TRIGGER_FENCE_SINGLE_ALLOW_UPD,
)


# ---------------------------------------------------------------------------
# Additive column migration.
#
# CREATE TABLE IF NOT EXISTS does nothing to a table that already exists, so the
# columns added above would be missing on every robot whose geo.db/fence.db was
# created by an earlier build -- and the failure would be an OperationalError on
# the first upsert, on the robot, not here. This closes that gap the only way
# SQLite offers: read PRAGMA table_info and ADD COLUMN what is absent.
#
# Why not the versioned Migration framework in migration.py: that framework keys
# off a schema_version row these two files have never carried, so every existing
# geo.db would present as version 0 and re-run the whole ladder against tables
# that are already at the top of it. An idempotent column-presence check needs no
# version at all, and it converges from ANY prior shape, including the ones no
# recorded version ever described.
#
# Every default here matches the CREATE TABLE default above, so an old database
# and a fresh one end up identical -- test_schema_migration asserts exactly that
# by comparing PRAGMA table_info of both, because a divergence between the DDL
# and this table is silent and permanent otherwise.
#
# state defaults to 'active' for EXISTING rows on purpose. Those rows predate the
# lifecycle column and are in service right now; defaulting them to 'draft' would
# switch off every fence and dock on the robot at the next process start, which
# is a fail-DANGEROUS migration. New objects get their state from the applier,
# where a newly recorded fence IS a draft until F15 activates it.
# The two column definitions that carry a CHECK. Written once and referenced
# below so the ADD COLUMN form cannot drift from the CREATE TABLE form: a
# migrated database WITHOUT the constraint would accept an off-set state, and
# PRAGMA table_info -- what the migration test compares -- does not report CHECK
# constraints, so that particular divergence is invisible to the test that
# exists to catch divergence.
_STATE_COLDEF = ("TEXT NOT NULL DEFAULT 'active' "
                 "CHECK (state IN ('draft','active','disabled','deleted'))")
_NUM_COLDEF = "INTEGER CHECK (num IS NULL OR (num >= 1 AND num <= 99))"

ADDED_COLUMNS = {
    "waypoints": (
        ("num", _NUM_COLDEF),
        ("alias_json", "TEXT NOT NULL DEFAULT '[]'"),
        ("state", _STATE_COLDEF),
        ("created_by", "TEXT NOT NULL DEFAULT 'factory'"),
        ("updated_by", "TEXT NOT NULL DEFAULT 'factory'"),
    ),
    "routes": (
        ("num", _NUM_COLDEF),
        ("alias_json", "TEXT NOT NULL DEFAULT '[]'"),
        ("state", _STATE_COLDEF),
        ("created_by", "TEXT NOT NULL DEFAULT 'factory'"),
        ("updated_by", "TEXT NOT NULL DEFAULT 'factory'"),
    ),
    "docks": (
        ("alias_json", "TEXT NOT NULL DEFAULT '[]'"),
        ("state", _STATE_COLDEF),
        ("created_by", "TEXT NOT NULL DEFAULT 'factory'"),
        ("updated_by", "TEXT NOT NULL DEFAULT 'factory'"),
    ),
    "fences": (
        ("num", _NUM_COLDEF),
        ("alias_json", "TEXT NOT NULL DEFAULT '[]'"),
        ("state", _STATE_COLDEF),
        ("created_by", "TEXT NOT NULL DEFAULT 'factory'"),
        ("updated_by", "TEXT NOT NULL DEFAULT 'factory'"),
    ),
}


async def ensure_added_columns(conn) -> int:
    """ADD COLUMN whatever ADDED_COLUMNS lists and this database lacks.

    Idempotent and order-free: it asks the database what it has rather than what
    version it claims to be. Returns the number of columns added, so the caller
    can log a migration having happened instead of it being invisible.

    Tables the connection does not own are skipped (geo.db has no fences table
    and vice versa) -- PRAGMA table_info on an absent table returns no rows,
    which is exactly the "nothing to do" answer.
    """
    added = 0
    for table, columns in ADDED_COLUMNS.items():
        cur = await conn.execute(f"PRAGMA table_info({table})")
        rows = await cur.fetchall()
        if not rows:
            continue                        # not a table of this database
        have = {r[1] for r in rows}
        for name, coldef in columns:
            if name in have:
                continue
            await conn.execute(
                f"ALTER TABLE {table} ADD COLUMN {name} {coldef}")
            added += 1
    if added:
        await conn.commit()
    return added


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
