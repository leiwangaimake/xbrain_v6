"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: schema_task.py
Brief: BIZ-P3-4 task.db DDL (tasks / patrol_progress / memory / snapshot / pending_push)

Description:
15 S9 five tables of task.db. Table names and column shapes come
straight from 15 S9; the CHECK constraints and the five indexes on
'tasks' come from the same section. All strings are ASCII to satisfy
CLAUDE.md S2.2 for source. Test file uses the DDL against an
in-memory aiosqlite handle and asserts that violating CHECK rows are
rejected.

Kept as constants (not templated code) so DDL diffs are visible in
git history and reviewable one table at a time.
"""

from __future__ import annotations

import time

from xbrain.common.enums import SUSPEND_KIND, TASK_STATE
from xbrain.p3_task.state.machine import SUSPEND_REASONS, TERMINAL_STATES


def iso_from_wall_ms(wall_ms: int) -> str:
    """Render a wall-clock epoch-ms into the created_at / updated_at text the
    15 S9 tables store.

    It lives beside the DDL because the DDL is what fixes the format: the
    15 S9.5 columns default to strftime('%Y-%m-%dT%H:%M:%fZ','now'), so a
    caller that hand-rolls a different spelling produces rows that sort
    against the defaults incorrectly -- and sorting is how
    idx_patrol_route_time answers "the latest run of this route".

    WALL-CLOCK-OK(record): these two columns are an audit timeline an operator
    reads, not an age or a timeout. Every interval decision in P3 (PP-1's
    progress_flush_s, the 3 s path_progress staleness) is computed on
    CLOCK_MONOTONIC and never touches this (CLK-C1).
    """
    whole, frac_ms = divmod(int(wall_ms), 1000)
    return "%s.%03dZ" % (time.strftime("%Y-%m-%dT%H:%M:%S",
                                       time.gmtime(whole)), frac_ms)

# Terminal states, for the duration_sec CHECK (duration is written only at a
# terminal). Imported from the machine so the DDL and the graph agree on which
# states are terminal.
_TERMINAL_FOR_DDL = TERMINAL_STATES


# 12-value task state closed set. NOT re-listed here -- taken from the single
# frozen source (common/enums/sets.yaml via common.enums). A local literal is
# what let this CHECK once carry a DIFFERENT 12 values than 11 S4.4, so the
# 'state' column silently accepted queued/completed/aborted the cloud could not
# read. sorted() gives a deterministic IN-clause for reviewable DDL diffs.
TASK_STATES = tuple(sorted(TASK_STATE.values))

# suspend_kind closed set (passive, yielding). suspend_reason PRODUCER set is
# the CR-6 subset owned by state.machine (11 S4.4 minus energy_unreachable) --
# imported, not re-derived, so the DDL CHECK and the machine can never disagree
# on which reason is legal to persist.
SUSPEND_KINDS = tuple(sorted(SUSPEND_KIND.values))
SUSPEND_REASON_VALUES = tuple(sorted(SUSPEND_REASONS))


TASK_TYPES = (
    "patrol",
    "goto",
    "charge",
    "return_home",
    "standby",
    "teach",
    "follow",
)

# source closed set (15 S9.5): where the task came from, and the axis the
# scheduler priority table ranks on (cloud > wecom > local > auto). 'charge' is
# P3's own return_home source (S4.2.1). 'auto' has no producer yet (reserved).
TASK_SOURCES = ("cloud", "wecom", "local", "auto", "charge")

# resume_policy closed set (15 S7.5 / CHG-32-33): resolved at admission and
# frozen on the row (never re-read from config afterwards).
RESUME_POLICIES = ("continue", "restart", "abort", "manual")


def _in_clause(items) -> str:
    return "(" + ",".join(f"'{v}'" for v in items) + ")"


DDL_TASKS = f"""
CREATE TABLE IF NOT EXISTS tasks (
  task_id       TEXT PRIMARY KEY,
  parent_task_id TEXT,
  task_type     TEXT NOT NULL CHECK (task_type IN {_in_clause(TASK_TYPES)}),
  state         TEXT NOT NULL CHECK (state IN {_in_clause(TASK_STATES)}),
  priority      INTEGER NOT NULL CHECK (priority BETWEEN 0 AND 100),
  submit_seq    INTEGER NOT NULL,
  suspend_kind  TEXT,
  suspend_reason TEXT,
  interrupt_reason TEXT,     -- last interrupt cause; NOT cleared on resume (audit)
  mission_json  TEXT NOT NULL,
  -- *** 实现是 >= 0, 而 15 S9.5 的 DDL 写的是 CHECK (total_steps >= 1).
  -- 这处放宽此前没有任何记录, 2026-09-02 补登记.
  --
  -- 放宽的原因: task_row_from_command 建行时写死 total_steps=0, 注释是
  -- "expanded later by the route layer" -- 那一层是路径展开(NEXT.md EX-1:
  -- mission_json + 航点名 -> geo.db 航点 -> total_steps + V-3/V-6 校验),
  -- 至今未建. 若保持 >= 1, 每一条 goto/patrol 在入库那一刻就被 DDL 拒掉.
  --
  -- 这不是"判据挡路就改判据"的豁免, 是一条[有期限的]欠账:
  --   EX-1 落地后, total_steps 由路径展开填出真值, 本行应改回 >= 1,
  --   并同批把存量 0 值的行迁移或清理(2026-09-02 时 144 行里有 142 行是 0).
  -- 在那之前, >= 0 与 15 S9.5 的差异必须留在这里可见 -- 一处没有记录的
  -- 放宽, 与"当初就是这么设计的"在代码上完全不可区分(CLAUDE.md 3.2 形态2).
  --
  -- NO 不把 total_steps 默认成 1 来"满足"约束: 那是编一个步数, 而下游的
  -- current_step BETWEEN 0 AND total_steps 会据此放行一个不存在的进度.
  total_steps   INTEGER NOT NULL CHECK (total_steps >= 0),
  current_step  INTEGER NOT NULL DEFAULT 0
                 CHECK (current_step BETWEEN 0 AND total_steps),
  step_status_json TEXT NOT NULL DEFAULT '[]',
  result_json   TEXT,
  error_context_json TEXT,
  source        TEXT NOT NULL CHECK (source IN {_in_clause(TASK_SOURCES)}),
  -- Raw command text the task was created from: the voice ASR transcript (post
  -- normalisation) or the typed text, for whichever channel (local / cloud /
  -- wecom). Party-A REQUIRES this stored for post-incident traceability
  -- (17 S6.8.4 field 3 / 15 S9.5A.4): given an incident event, follow trace_id
  -- to the task and read what was actually commanded. NULL for system-minted
  -- tasks (return_home / charge) that no human or cloud command produced. It is
  -- a FIRST-CLASS column, NOT a mission_json field, so it stays queryable and
  -- survives any change to the mission_json shape.
  command_text  TEXT,
  resume_policy TEXT NOT NULL CHECK (resume_policy IN {_in_clause(RESUME_POLICIES)}),
  resume_count  INTEGER NOT NULL DEFAULT 0 CHECK (resume_count >= 0),
  route_geo_id  TEXT,        -- immutable geo_id of the referenced route (tombstone)
  user_id       TEXT,
  trace_id      TEXT NOT NULL,   -- ties cmd -> task -> event across the stack
  ttl_seconds   INTEGER,
  scheduled_at  TEXT,        -- ISO wall time a timed task becomes due
  -- Monotonic anchors (CLK-C1): internal ordering / age. created_ms drives the
  -- ix_tasks_created index; NOT a wall clock, so it never steps at RTK/NTP sync.
  created_ms    INTEGER NOT NULL,
  updated_ms    INTEGER NOT NULL,
  -- Wall-clock audit columns (15 S9.5): DISPLAY / AUDIT ONLY, filled at the
  -- matching transition. Never used to compute a duration (a wall diff steps
  -- seconds at every cold-boot RTK/NTP sync) -- that is started_mono below.
  created_at    TEXT,
  started_at    TEXT,
  paused_at     TEXT,
  finished_at   TEXT,
  cancelled_at  TEXT,
  -- Authoritative duration (15 S9.5, R12.4): started_mono is the monotonic read
  -- at entry to running; duration_sec = now_mono - started_mono at the terminal.
  -- If the terminal's boot != started_boot the task crossed a restart and
  -- duration_sec MUST be NULL (never a wall diff) -- enforced by the writer.
  started_mono  REAL,
  started_boot  TEXT,
  duration_sec  REAL,
  -- suspend_kind / suspend_reason are non-null IFF state == 'suspended'
  -- (11 S4.4 / 15 S9.5). A bare closed-set CHECK is not enough: it admits a
  -- suspend field on a running row (fail-silent).
  CHECK ((state = 'suspended') = (suspend_kind IS NOT NULL)),
  CHECK ((state = 'suspended') = (suspend_reason IS NOT NULL)),
  CHECK (suspend_kind IS NULL OR suspend_kind IN {_in_clause(SUSPEND_KINDS)}),
  -- CR-6 (15 S9.5): suspend_reason is a DELIBERATE proper subset of the
  -- 11 S4.4 closed set -- it omits 'energy_unreachable', which has no producer
  -- (SUSPEND_REASONS is that subset, owned by state.machine).
  CHECK (suspend_reason IS NULL OR
         suspend_reason IN {_in_clause(SUSPEND_REASON_VALUES)}),
  -- interrupt_reason shares the suspend_reason value set (15 S9.5): only the
  -- suspend closed set, but NOT paired with the suspended state (it survives a
  -- resume). A missing CHECK here is the documented hole where a typo persists.
  CHECK (interrupt_reason IS NULL OR
         interrupt_reason IN {_in_clause(SUSPEND_REASON_VALUES)}),
  -- CR-8 (15 S9.5): the two closed-set CHECKs above each pass the combo
  -- kind='passive' + reason='preempted' -- a fail-silent row that never enters
  -- the yielding auto-resume scan (idx_tasks_yielding). Enforce the pairing:
  -- kind is 'yielding' exactly when reason is preempted/mode_takeover.
  CHECK (suspend_kind IS NULL OR suspend_reason IS NULL OR
         (suspend_kind = 'yielding')
           = (suspend_reason IN ('preempted','mode_takeover'))),
  -- duration_sec is non-null only at a terminal state (15 S9.5).
  CHECK (duration_sec IS NULL OR state IN {_in_clause(sorted(_TERMINAL_FOR_DDL))})
);
""".strip()


DDL_TASKS_INDEXES = (
    "CREATE INDEX IF NOT EXISTS ix_tasks_state ON tasks(state);",
    "CREATE INDEX IF NOT EXISTS ix_tasks_priority_seq "
    "ON tasks(priority DESC, submit_seq ASC);",
    "CREATE INDEX IF NOT EXISTS ix_tasks_type_state ON tasks(task_type, state);",
    "CREATE INDEX IF NOT EXISTS ix_tasks_updated ON tasks(updated_ms);",
    "CREATE INDEX IF NOT EXISTS ix_tasks_created ON tasks(created_ms);",
    # Partial index for the yielding auto-resume scan (15 S3.2 / S6.3): when a
    # yielded-to task reaches a terminal state, the scheduler batch-selects the
    # tasks waiting to auto-resume. The predicate MUST be lowercase and match
    # the CHECK vocabulary or it never fires (the fail-silent index trap).
    "CREATE INDEX IF NOT EXISTS ix_tasks_yielding ON tasks(suspend_reason) "
    "WHERE suspend_kind = 'yielding';",
    # Scheduled-wakeup partial index (15 S9.5): the delayed-wakeup loop polls
    # timed tasks by scheduled_at cheaply. The predicate MUST be lowercase
    # 'scheduled' and match the CHECK vocabulary, or it never fires and a timed
    # task reads as 'due at its time but never started' with no error (the
    # fail-silent index trap 15 S3.2 names as the one real runtime risk).
    "CREATE INDEX IF NOT EXISTS ix_tasks_scheduled ON tasks(scheduled_at) "
    "WHERE state = 'scheduled';",
)


# 15 S9.5 patrol_progress, rebuilt 2026-09-28 (docs/NEXT.md EX-6).
#
# *** What was here before and why it had to go. The previous shape was four
# columns -- task_id / waypoint_ix / progress / updated_ms -- and none of the
# four names appears in 15 S9.5 or in the PathProgress wire message (11 S3.5B).
# 15 S9.5 states the rule in as many words: "字段名与 PathProgress 报文逐字一致,
# 禁止在此处另起别名 (CFG-40)". The old `progress REAL 0..1` in particular was
# a DERIVED percentage, which PP-3 forbids storing at all (a derived column is
# a second truth), and it could not express the breakpoint U07a is defined on:
# the breakpoint is the PAIR (waypoint_index, seg_done_m) and seg_done_m had no
# column. So every downstream mechanism that reads this table -- U07a resume,
# S7.3A remap (needs route_rev / route_total_m / dist_done_m), the pingpong
# direction rule of S9.5B, the GC-3 lap clamp (needs loop_index / loop_total)
# -- had no input. Rebuilding is not a schema preference; those features cannot
# be written against the old shape.
#
# The 20 columns and both indexes below are transcribed from 15 S9.5. Two
# CHECKs are worth reading before "simplifying" them:
#
#   * loop_index is bounded ONLY when loop_total != 0. loops = 0 means an
#     unbounded standing patrol (11 S7.8.3 LM-4), and the naive
#     `loop_index <= loop_total` breaks on the SECOND lap of such a task.
#   * waypoint_index starts at -1, not 0: it means "the last waypoint passed",
#     and before the first one is reached there is none. A 0-based floor would
#     make a task that has not reached its first point indistinguishable from
#     one that has passed point 0.
#
# NOT here: `persisted` and any monotonic timestamp. `persisted` is a RUNTIME
# derived bit of TaskState (path_progress older than 3 s -> false, 11 S3.5B)
# and every interval/age decision is computed in memory off CLOCK_MONOTONIC
# (DBF-3 / CLK-C1). Persisting a monotonic value would make it meaningless
# across the next boot while still looking like a timestamp.
DDL_PATROL_PROGRESS = """
CREATE TABLE IF NOT EXISTS patrol_progress (
  id              INTEGER PRIMARY KEY AUTOINCREMENT,
  task_id         TEXT    NOT NULL,
  route_name      TEXT    NOT NULL,
  route_geo_id    TEXT,
  route_rev       INTEGER NOT NULL,
  direction       TEXT    NOT NULL,
  loop_mode       TEXT    NOT NULL DEFAULT 'oneway',
  waypoint_index  INTEGER NOT NULL DEFAULT -1,
  waypoint_total  INTEGER NOT NULL,
  seg_done_m      REAL    NOT NULL DEFAULT 0.0,
  dist_done_m     REAL    NOT NULL DEFAULT 0.0,
  odom_dist_m     REAL    NOT NULL DEFAULT 0.0,
  route_total_m   REAL    NOT NULL,
  loop_index      INTEGER NOT NULL DEFAULT 0,
  loop_total      INTEGER NOT NULL DEFAULT 1,
  skipped_m       REAL    NOT NULL DEFAULT 0.0,
  dir_sign        INTEGER NOT NULL DEFAULT 1,
  status          TEXT    NOT NULL DEFAULT 'active',
  created_at      TEXT    NOT NULL,
  updated_at      TEXT    NOT NULL,
  CHECK (direction IN ('forward','reverse')),
  CHECK (loop_mode IN ('oneway','pingpong','closed')),
  CHECK (dir_sign IN (-1, 1)),
  CHECK (status IN ('active','completed','aborted')),
  CHECK (waypoint_index >= -1 AND waypoint_index < waypoint_total),
  CHECK (loop_index >= 0 AND (loop_total = 0 OR loop_index <= loop_total))
);
""".strip()


DDL_PATROL_PROGRESS_INDEXES = (
    # 15 S9.5: at most one active row per task. This is the index that makes
    # "which run is the current one" answerable without a timestamp race -- a
    # second active row for the same task is rejected by the database rather
    # than resolved by whichever query happens to ORDER BY first.
    "CREATE UNIQUE INDEX IF NOT EXISTS idx_patrol_active "
    "ON patrol_progress(task_id) WHERE status = 'active';",
    "CREATE INDEX IF NOT EXISTS idx_patrol_route_time "
    "ON patrol_progress(route_name, created_at DESC);",
)


#: The column whose presence distinguishes the 15 S9.5 shape from the
#: pre-2026-09-28 four-column one. Any contract column would do; route_rev is
#: named because it is the one S7.3A remap cannot run without.
_PATROL_SHAPE_MARKER = "route_rev"


async def ensure_patrol_progress_shape(conn) -> bool:
    """Replace a pre-2026-09-28 four-column `patrol_progress` with the 15 S9.5
    shape. Returns True if it reshaped. MUST run BEFORE the DDL burst.

    *** Why this exists at all: every statement in ALL_DDL_STATEMENTS is
    CREATE ... IF NOT EXISTS, so on a database that already carries the legacy
    table the new DDL is a silent no-op. The process then starts normally and
    fails on the FIRST progress write with "no such column: route_rev" -- at
    the moment a patrol reaches its first waypoint, on the robot, which is the
    worst possible place to discover a schema mismatch.

    *** Why a DROP is acceptable here and would not be in general: the legacy
    table had no writer. The only caller of PatrolProgressDAO.upsert in the
    whole tree was one unit test, EX-3 (the path_progress subscriber that would
    have populated it) did not exist, and the live data/run/task.db on the ORIN
    held zero rows on 2026-09-28. A non-empty legacy table therefore means
    something happened that this reasoning did not cover, so it RAISES instead
    of dropping: losing a breakpoint silently is exactly the failure U07a
    exists to prevent.
    """
    cur = await conn.execute("PRAGMA table_info(patrol_progress)")
    cols = [row[1] for row in await cur.fetchall()]
    if not cols:
        return False                       # no table yet; the DDL will build it
    if _PATROL_SHAPE_MARKER in cols:
        return False                       # already the contract shape
    cur = await conn.execute("SELECT COUNT(*) FROM patrol_progress")
    row = await cur.fetchone()
    n_rows = int(row[0]) if row else 0
    if n_rows:
        raise PatrolProgressShapeConflict(
            "patrol_progress carries the pre-2026-09-28 four-column shape AND "
            "%d row(s); refusing to drop it. Columns found: %s. Export the "
            "rows, then delete the table by hand." % (n_rows, cols))
    await conn.execute("DROP TABLE patrol_progress")
    await conn.commit()
    return True


class PatrolProgressShapeConflict(Exception):
    """A legacy patrol_progress table that still holds rows (see above)."""


DDL_MEMORY = """
CREATE TABLE IF NOT EXISTS memory (
  key   TEXT PRIMARY KEY,
  value BLOB NOT NULL,
  updated_ms INTEGER NOT NULL
);
""".strip()


# 15 S9.3A task_route_snapshot, rebuilt 2026-09-28 (docs/NEXT.md EX-2).
#
# *** What was here before. Five columns, task_id/seq/x_m/y_m/heading_rad --
# one row per point, in ENU metres, with a heading. 15 S9.3A defines one row
# per TASK carrying WGS84 points as JSON. The old shape could not hold what
# S7.3A reads (route_id, rev, loop_mode, total_len_m = the remap's T0, and the
# precomputed arclen array), and metres are the wrong frame: cmd/motion/route
# is frame "wgs84" (11 S3.5A) and P1 does the projection itself
# (nav/route_intake.py), so storing metres here would mean projecting twice
# against two different site origins.
#
# 11 S7.12.1 R1: "运行中的任务跑的是快照, 不是活对象". This row IS the thing
# pushed to P1 (11 S3.5A reason 3), which is why SN-1 orders the write before
# the push: reversed, a power cut leaves P1 holding geometry P3 has no
# snapshot of, and the remap then uses the wrong T0.
#
# *** point_count floor is 1, NOT the 2 that 15 S9.3A's CHECK says
# (CLAUDE.md IRON RULE 1 -- 15 S9.3A has been corrected to match).
# 11 S3.5A is the contract single source of truth (CLAUDE.md 1) and its v2.0
# block (2026-09-08, #20-1) reads "v2.0 起最少 1 点 -- 单点 = goto 退化折线
# (20 RNS-N-1)". xbrain/p1_motion/nav/route_intake.py already implements the
# 1-point floor and its tests are green, so a `>= 2` here would refuse to
# snapshot exactly the case P1 accepts: a one-waypoint goto.
DDL_TASK_ROUTE_SNAPSHOT = """
CREATE TABLE IF NOT EXISTS task_route_snapshot (
  task_id      TEXT    PRIMARY KEY REFERENCES tasks(task_id),
  route_id     TEXT    NOT NULL,
  rev          INTEGER NOT NULL,
  loop_mode    TEXT    NOT NULL,
  point_count  INTEGER NOT NULL,
  total_len_m  REAL    NOT NULL,
  points_json  TEXT    NOT NULL,
  arclen_json  TEXT    NOT NULL,
  created_at   TEXT    NOT NULL,
  CHECK (loop_mode IN ('oneway','pingpong','closed')),
  CHECK (point_count >= 1 AND point_count <= 5000),
  CHECK (total_len_m > 0.0)
);
""".strip()

DDL_TASK_ROUTE_SNAPSHOT_INDEXES = (
    "CREATE INDEX IF NOT EXISTS idx_snapshot_route "
    "ON task_route_snapshot(route_id, rev);",
)

#: Marks the 15 S9.3A shape apart from the pre-2026-09-28 per-point one.
_SNAPSHOT_SHAPE_MARKER = "points_json"


async def ensure_task_route_snapshot_shape(conn) -> bool:
    """Replace a pre-2026-09-28 per-point task_route_snapshot with the
    15 S9.3A shape. Returns True if it reshaped. MUST run BEFORE the DDL
    burst, for the same reason as ensure_patrol_progress_shape.

    SN-3 says a suspended task's snapshot must never be cleaned up, so the
    non-empty case RAISES here too: a dropped snapshot means 11 S7.12.3 step 1
    ("快照丢失则无法重映射") and the task can only go to needs_review. The
    live data/run/task.db on the ORIN held zero rows on 2026-09-28 -- the old
    SnapshotDAO.replace had no caller outside one unit test.
    """
    cur = await conn.execute("PRAGMA table_info(task_route_snapshot)")
    cols = [row[1] for row in await cur.fetchall()]
    if not cols:
        return False
    if _SNAPSHOT_SHAPE_MARKER in cols:
        return False
    cur = await conn.execute("SELECT COUNT(*) FROM task_route_snapshot")
    row = await cur.fetchone()
    n_rows = int(row[0]) if row else 0
    if n_rows:
        raise TaskRouteSnapshotShapeConflict(
            "task_route_snapshot carries the pre-2026-09-28 per-point shape "
            "AND %d row(s); refusing to drop it -- SN-3 forbids losing a "
            "suspended task's snapshot. Columns found: %s." % (n_rows, cols))
    await conn.execute("DROP TABLE task_route_snapshot")
    await conn.commit()
    return True


class TaskRouteSnapshotShapeConflict(Exception):
    """A legacy task_route_snapshot that still holds rows (see above)."""


DDL_GEO_PENDING_PUSH = """
CREATE TABLE IF NOT EXISTS geo_pending_push (
  push_id    INTEGER PRIMARY KEY AUTOINCREMENT,
  kind       TEXT NOT NULL,
  object_id  TEXT NOT NULL,
  rev        INTEGER NOT NULL,
  enqueued_ms INTEGER NOT NULL
);
""".strip()


# 11 S2.3: "the receiver de-duplicates on cmd_id". cmd/task needs its own log
# because the idempotency key is NOT the task_id -- 11 S7.2 (as corrected
# 2026-08-20) lets a sender omit task.task_id and have P3 allocate one, so a
# redelivered submit has nothing else to be recognised by. Written in the SAME
# transaction as the insert: split across two commits, a crash between them
# leaves the task recorded and the command unseen, and the retry mints a second
# task the operator never asked for.
DDL_TASK_CMD_LOG = """
CREATE TABLE IF NOT EXISTS task_cmd_log (
  cmd_id      TEXT PRIMARY KEY,
  action      TEXT NOT NULL,
  task_id     TEXT,                            -- the id acted on / allocated
  result      TEXT NOT NULL,                   -- accepted | rejected | duplicate
  code        TEXT NOT NULL,
  detail_json TEXT,
  applied_ms  INTEGER NOT NULL
);
""".strip()


ALL_DDL_STATEMENTS = (
    DDL_TASK_CMD_LOG,
    DDL_TASKS,
    *DDL_TASKS_INDEXES,
    DDL_PATROL_PROGRESS,
    *DDL_PATROL_PROGRESS_INDEXES,
    DDL_MEMORY,
    DDL_TASK_ROUTE_SNAPSHOT,
    *DDL_TASK_ROUTE_SNAPSHOT_INDEXES,
    DDL_GEO_PENDING_PUSH,
)
