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
from xbrain.p3_task.state.machine import SUSPEND_REASONS, TERMINAL_STATES

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
    DDL_MEMORY,
    DDL_TASK_ROUTE_SNAPSHOT,
    DDL_GEO_PENDING_PUSH,
)
