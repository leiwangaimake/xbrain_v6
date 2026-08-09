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
CLAUDE.md §2.2 for source. Test file uses the DDL against an
in-memory aiosqlite handle and asserts that violating CHECK rows are
rejected.

Kept as constants (not templated code) so DDL diffs are visible in
git history and reviewable one table at a time.
"""

from __future__ import annotations


# 12-value task state closed set (11 §4.4 lowercase enum). Duplicated
# here as a CHECK regex because SQLite lacks native enums; the
# canonical closed set stays in common/closed_sets.
TASK_STATES = (
    "pending",
    "queued",
    "starting",
    "running",
    "suspended",
    "resuming",
    "completing",
    "completed",
    "cancelling",
    "cancelled",
    "failed",
    "aborted",
)


TASK_TYPES = (
    "patrol",
    "goto",
    "charge",
    "return_home",
    "standby",
    "teach",
    "follow",
)


def _in_clause(items) -> str:
    return "(" + ",".join(f"'{v}'" for v in items) + ")"


DDL_TASKS = f"""
CREATE TABLE IF NOT EXISTS tasks (
  task_id       TEXT PRIMARY KEY,
  task_type     TEXT NOT NULL CHECK (task_type IN {_in_clause(TASK_TYPES)}),
  state         TEXT NOT NULL CHECK (state IN {_in_clause(TASK_STATES)}),
  priority      INTEGER NOT NULL CHECK (priority BETWEEN 0 AND 100),
  submit_seq    INTEGER NOT NULL,
  suspend_kind  TEXT,
  suspend_reason TEXT,
  mission_json  TEXT NOT NULL,
  total_steps   INTEGER NOT NULL CHECK (total_steps >= 0),
  current_step  INTEGER NOT NULL DEFAULT 0
                 CHECK (current_step BETWEEN 0 AND total_steps),
  step_status_json TEXT NOT NULL DEFAULT '[]',
  created_ms    INTEGER NOT NULL,
  updated_ms    INTEGER NOT NULL
);
""".strip()


DDL_TASKS_INDEXES = (
    "CREATE INDEX IF NOT EXISTS ix_tasks_state ON tasks(state);",
    "CREATE INDEX IF NOT EXISTS ix_tasks_priority_seq "
    "ON tasks(priority DESC, submit_seq ASC);",
    "CREATE INDEX IF NOT EXISTS ix_tasks_type_state ON tasks(task_type, state);",
    "CREATE INDEX IF NOT EXISTS ix_tasks_updated ON tasks(updated_ms);",
    "CREATE INDEX IF NOT EXISTS ix_tasks_created ON tasks(created_ms);",
)


DDL_PATROL_PROGRESS = """
CREATE TABLE IF NOT EXISTS patrol_progress (
  task_id    TEXT PRIMARY KEY REFERENCES tasks(task_id),
  waypoint_ix INTEGER NOT NULL,
  progress   REAL NOT NULL CHECK (progress BETWEEN 0.0 AND 1.0),
  updated_ms INTEGER NOT NULL
);
""".strip()


DDL_MEMORY = """
CREATE TABLE IF NOT EXISTS memory (
  key   TEXT PRIMARY KEY,
  value BLOB NOT NULL,
  updated_ms INTEGER NOT NULL
);
""".strip()


DDL_TASK_ROUTE_SNAPSHOT = """
CREATE TABLE IF NOT EXISTS task_route_snapshot (
  task_id    TEXT NOT NULL REFERENCES tasks(task_id),
  seq        INTEGER NOT NULL,
  x_m        REAL NOT NULL,
  y_m        REAL NOT NULL,
  heading_rad REAL,
  PRIMARY KEY (task_id, seq)
);
""".strip()


DDL_GEO_PENDING_PUSH = """
CREATE TABLE IF NOT EXISTS geo_pending_push (
  push_id    INTEGER PRIMARY KEY AUTOINCREMENT,
  kind       TEXT NOT NULL,
  object_id  TEXT NOT NULL,
  rev        INTEGER NOT NULL,
  enqueued_ms INTEGER NOT NULL
);
""".strip()


ALL_DDL_STATEMENTS = (
    DDL_TASKS,
    *DDL_TASKS_INDEXES,
    DDL_PATROL_PROGRESS,
    DDL_MEMORY,
    DDL_TASK_ROUTE_SNAPSHOT,
    DDL_GEO_PENDING_PUSH,
)
