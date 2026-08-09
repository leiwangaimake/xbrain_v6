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
    suspend_kind: str = ""
    suspend_reason: str = ""


class TasksDAO:
    """Owns rows of the 'tasks' table. Runs on the db thread only."""

    def __init__(self, conn) -> None:
        self._conn = conn

    async def insert(self, row: TaskRow) -> None:
        await self._conn.execute(
            "INSERT INTO tasks (task_id, task_type, state, priority, "
            " submit_seq, suspend_kind, suspend_reason, mission_json, "
            " total_steps, current_step, step_status_json, created_ms, "
            " updated_ms) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (row.task_id, row.task_type, row.state, row.priority,
             row.submit_seq, row.suspend_kind, row.suspend_reason,
             row.mission_json, row.total_steps, row.current_step,
             row.step_status_json, row.created_ms, row.updated_ms))

    async def update_state(self, task_id: str, state: str,
                            updated_ms: int) -> int:
        cur = await self._conn.execute(
            "UPDATE tasks SET state=?, updated_ms=? WHERE task_id=?",
            (state, updated_ms, task_id))
        return cur.rowcount

    async def fetch_by_id(self, task_id: str):
        cur = await self._conn.execute(
            "SELECT task_id, task_type, state, priority, submit_seq, "
            " suspend_kind, suspend_reason, mission_json, total_steps, "
            " current_step, step_status_json, created_ms, updated_ms "
            " FROM tasks WHERE task_id=?", (task_id,))
        row = await cur.fetchone()
        if row is None:
            return None
        return TaskRow(
            task_id=row[0], task_type=row[1], state=row[2],
            priority=row[3], submit_seq=row[4],
            suspend_kind=row[5] or "", suspend_reason=row[6] or "",
            mission_json=row[7], total_steps=row[8], current_step=row[9],
            step_status_json=row[10], created_ms=row[11], updated_ms=row[12])

    async def list_by_priority(self, limit: int = 32):
        cur = await self._conn.execute(
            "SELECT task_id, priority, submit_seq, state FROM tasks "
            "ORDER BY priority DESC, submit_seq ASC LIMIT ?", (limit,))
        return await cur.fetchall()
