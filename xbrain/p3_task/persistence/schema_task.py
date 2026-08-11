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

from xbrain.common.enums import SUSPEND_KIND, TASK_STATE
from xbrain.p3_task.state.machine import SUSPEND_REASONS


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
  updated_ms    INTEGER NOT NULL,
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
  -- CR-8 (15 S9.5): the two closed-set CHECKs above each pass the combo
  -- kind='passive' + reason='preempted' -- a fail-silent row that never enters
  -- the yielding auto-resume scan (idx_tasks_yielding). Enforce the pairing:
  -- kind is 'yielding' exactly when reason is preempted/mode_takeover.
  CHECK (suspend_kind IS NULL OR suspend_reason IS NULL OR
         (suspend_kind = 'yielding')
           = (suspend_reason IN ('preempted','mode_takeover')))
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
    # NOTE: the scheduled-wakeup partial index (idx_tasks_scheduled ON
    # tasks(state, scheduled_at) WHERE state = 'scheduled', 15 S9.5) lands in
    # PB2 together with the scheduled_at column it indexes -- building it now,
    # before that column exists, is not possible. 'scheduled' is already a legal
    # state after this batch, so no row is lost meanwhile.
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
