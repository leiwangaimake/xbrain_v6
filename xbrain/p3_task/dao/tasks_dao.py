"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: tasks_dao.py
Brief: BIZ-P3-6 TasksDAO (task row CRUD + priority-FIFO scan)

Description:
15 S9 DAO discipline: exactly ONE DAO class per table; DAO holds
NO business logic (state-machine gates live in state/, scheduling
in schedule/); DAO only reads and writes rows atomically.

TasksDAO exposes:
  insert(task)             append a new row (submit_seq monotonic)
  update_state(task_id, state)  no-op if state unchanged (idempotent)
  fetch_by_id(task_id)     one row or None
  list_by_priority()       priority DESC, submit_seq ASC
The priority-FIFO scan powers BIZ-P3-9 scheduling loop.

All statements are parameterised (never f-string joined) to close
one SQL-injection path even though every writer is local.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TaskRow:
    """One row of the `tasks` table (15 S9.5). The Python side uses '' for an
    absent TEXT column and None for an absent number; insert() maps '' -> NULL
    so the DB canonical (NULL = absent) holds and the pairing CHECKs behave.

    Required (no default): the identity + the 15 S9.5 NOT NULL columns
    (source / trace_id / resume_policy). Everything a lifecycle transition
    fills later (timestamps, result, duration) defaults to absent."""
    # -- identity + required (15 S9.5 NOT NULL) --
    task_id: str
    task_type: str
    state: str
    priority: int
    submit_seq: int
    mission_json: str
    total_steps: int
    current_step: int
    step_status_json: str
    created_ms: int
    updated_ms: int
    source: str                 # cloud|wecom|local|auto|charge (priority axis)
    trace_id: str               # cmd -> task -> event correlation
    resume_policy: str          # continue|restart|abort|manual (frozen at admit)
    # -- optional (absent until set) --
    suspend_kind: str = ""
    suspend_reason: str = ""
    interrupt_reason: str = ""   # last interrupt cause; kept across resume
    parent_task_id: str = ""
    # Raw command text the task was created from (15 S9.5A.4): '' -> NULL for
    # system-minted tasks (return_home/charge) with no human/cloud command.
    command_text: str = ""
    result_json: str = ""
    error_context_json: str = ""
    resume_count: int = 0
    route_geo_id: str = ""
    user_id: str = ""
    ttl_seconds: "int | None" = None
    scheduled_at: str = ""       # ISO wall time a timed task becomes due
    # wall-clock audit (display only); filled at the matching transition
    created_at: str = ""
    started_at: str = ""
    paused_at: str = ""
    finished_at: str = ""
    cancelled_at: str = ""
    # monotonic duration (authoritative; NULL if the task crossed a restart)
    started_mono: "float | None" = None
    started_boot: str = ""
    duration_sec: "float | None" = None


# Column order shared by INSERT and SELECT (one list, so the two can never
# drift out of alignment). Matches the DDL column order in schema_task.
_COLUMNS = (
    "task_id", "parent_task_id", "task_type", "state", "priority",
    "submit_seq", "suspend_kind", "suspend_reason", "interrupt_reason",
    "mission_json", "total_steps", "current_step", "step_status_json",
    "result_json", "error_context_json", "source", "command_text",
    "resume_policy",
    "resume_count", "route_geo_id", "user_id", "trace_id", "ttl_seconds",
    "scheduled_at", "created_ms", "updated_ms", "created_at", "started_at",
    "paused_at", "finished_at", "cancelled_at", "started_mono", "started_boot",
    "duration_sec",
)

# TEXT columns whose '' Python value must persist as NULL (absent). Numbers
# already use None for absent, so they are not listed here.
_NULLABLE_TEXT = frozenset({
    "parent_task_id", "suspend_kind", "suspend_reason", "interrupt_reason",
    "result_json", "error_context_json", "route_geo_id", "user_id",
    "scheduled_at", "created_at", "started_at", "paused_at", "finished_at",
    "cancelled_at", "started_boot", "command_text",
})


def _row_values(row: "TaskRow") -> tuple:
    """Row -> value tuple in _COLUMNS order, coercing '' -> NULL for the
    nullable TEXT columns (see TaskRow docstring)."""
    out = []
    for c in _COLUMNS:
        v = getattr(row, c)
        if c in _NULLABLE_TEXT and v == "":
            v = None
        out.append(v)
    return tuple(out)


class TasksDAO:
    """Owns rows of the 'tasks' table. Runs on the db thread only."""

    def __init__(self, conn) -> None:
        self._conn = conn

    async def insert(self, row: TaskRow) -> None:
        # Values in _COLUMNS order, with '' -> NULL for nullable TEXT (so the
        # suspend/interrupt closed-set + pairing CHECKs see NULL, not '', for
        # absent -- '' IS NOT NULL is TRUE and would fail those CHECKs).
        placeholders = ", ".join("?" for _ in _COLUMNS)
        await self._conn.execute(
            f"INSERT INTO tasks ({', '.join(_COLUMNS)}) VALUES ({placeholders})",
            _row_values(row))

    async def update_state(self, task_id: str, state: str,
                            updated_ms: int) -> int:
        cur = await self._conn.execute(
            "UPDATE tasks SET state=?, updated_ms=? WHERE task_id=?",
            (state, updated_ms, task_id))
        return cur.rowcount

    async def dispatch_task(self, task_id: str, updated_ms: int,
                            started_at: str, started_mono: float) -> int:
        """ready -> running, 同时落 started_at / started_mono.

        *** 与 update_state 分开, 因为派发要多写两列.
        15 S9.5 有 started_at, 而 duration_sec 的 DDL CHECK 依赖终态时间戳 --
        running 却没有开始时间的话, 任务时长永远算不出来. 2026-09-02 实测:
        库里那条 running 的 started_at 是 NULL.

        started_at 是墙钟(审计/上报用, 与 created_at 同口径), started_mono 是
        单调钟(算时长用, CLK-C1: 一切时长判定用单调钟). 两个都写 -- 墙钟给人
        看, 单调钟给机器算, 墙钟跳变时后者不受影响.
        """
        cur = await self._conn.execute(
            "UPDATE tasks SET state='running', updated_ms=?, "
            "started_at=?, started_mono=? WHERE task_id=?",
            (updated_ms, started_at, started_mono, task_id))
        return cur.rowcount

    async def preempt_task(self, task_id: str, reason: str,
                           updated_ms: int) -> int:
        """running -> suspended(yielding), 15 S6.3 抢占.

        *** kind 恒为 yielding, NO 不接受调用方指定.
        CR-8 配对(11 S4.4): kind == 'yielding' IFF reason in
        {preempted, mode_takeover}. 让调用方传 kind 就等于给了它写出
        yielding+low_battery 这种组合的机会, 而那会被 DDL CHECK 拒掉 --
        在这里定死, 违规不可能发生.

        与 suspend_task(estop 那条, kind=passive)分开而不是合成一个带参数的:
        两条路径的 kind 恒定且不同, 合并后第一个 bug 就是把 estop 写成
        yielding -- 那会让急停挂起的任务在让位对象终态后[自动恢复运行]
        (15 S3.2: yielding 的恢复条件是"让位对象一进终态就自动回 ready"),
        而急停要求的是等人.
        """
        cur = await self._conn.execute(
            "UPDATE tasks SET state='suspended', suspend_kind='yielding', "
            "suspend_reason=?, updated_ms=? WHERE task_id=?",
            (reason, updated_ms, task_id))
        return cur.rowcount

    async def fetch_by_id(self, task_id: str):
        cur = await self._conn.execute(
            f"SELECT {', '.join(_COLUMNS)} FROM tasks WHERE task_id=?",
            (task_id,))
        row = await cur.fetchone()
        if row is None:
            return None
        # NULL -> '' for TEXT so the Python side never sees None on a str field;
        # numbers keep None. Build kwargs by column name (order-independent).
        kw = {}
        for c, v in zip(_COLUMNS, row):
            if c in _NULLABLE_TEXT and v is None:
                v = ""
            kw[c] = v
        return TaskRow(**kw)

    async def list_by_priority(self, limit: int = 512):
        """调度器的取数口: 只取[未终结]的任务, 按 15 S6.1 的序.

        *** 必须按状态过滤, NO 不能取全表前 N.
        原实现不带 WHERE, 于是终态任务(cancelled/done/failed/...)也按优先级
        占名额. 2026-09-02 实测: 库里 145 行, LIMIT 32 取到的 32 行里 pending
        数为 0 -- 107 条 pending [永远进不了调度视野], 不是排队等而是看不见.
        而且会持续恶化: 终态行不会消失, 按优先级占着前 32 名, 最终把视野塞满,
        任务系统在积累一定数量后静默停止工作.

        15 S3.2 定义"活跃" = status IN (ready, running, suspended); 调度还要
        看 pending(phase 1 要把它验成 ready)与 scheduled/blocked(到点/解除后
        回 ready). 所以这里的集合是[活跃 + 未终结的等待态], 显式枚举而不是
        NOT IN <终态集> -- 沿用 15 S3.2 的理由: 将来新增状态不会被静默归入.

        limit 提到 512: 32 是个没有依据的小数, 而 15 S9.5 的队列没有深度上限.
        512 仍是护栏(防一次读进十万行), 但已远超任何现场的活跃任务数; 真正
        的保护是上面的状态过滤 -- 终态行再多也不占名额.
        """
        cur = await self._conn.execute(
            "SELECT task_id, priority, submit_seq, state FROM tasks "
            "WHERE state IN ('pending','scheduled','blocked',"
            "                'ready','running','suspended') "
            "ORDER BY priority DESC, submit_seq ASC LIMIT ?", (limit,))
        return await cur.fetchall()

    async def suspend_task(self, task_id: str, kind: str, reason: str,
                           updated_ms: int) -> int:
        """Suspend a task, writing its suspend_kind/reason together.

        The tasks DDL pairs suspend_kind/suspend_reason with the suspended
        state (both non-null IFF suspended, schema CHECK). So the estop suspend
        (ES-2) cannot go through update_state -- that writes only `state`, and
        a suspended row with NULL kind/reason violates the CHECK. This method
        writes all three atomically. The pause path (task_apply._write_state)
        has its own; this one is the estop path (kind=passive, reason=
        estop_soft), kept separate so the two reasons never get crossed.

        The resume that later clears them is task_apply._write_state's else
        branch (suspend_kind=NULL on any non-suspended target), so an
        estop-suspended task resumes the same as a pause-suspended one.
        """
        cur = await self._conn.execute(
            "UPDATE tasks SET state='suspended', suspend_kind=?, "
            "suspend_reason=?, updated_ms=? WHERE task_id=?",
            (kind, reason, updated_ms, task_id))
        return cur.rowcount
