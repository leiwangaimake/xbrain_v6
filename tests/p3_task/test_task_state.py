"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_state.py
Brief: 11 S4.4 TaskState -- route_id / started_ts / bucketing / the 1 Hz floor

Description:
The defect: P3 published {schema, active_task:{task_id, state, mono_ms}} on
state/task instead of the contracted 11 S4.4 TaskState, so route_id and
started_ts -- both mandatory on the cloud task item (v2.0 S3.2) -- had no path
off the robot, and `queue` / `suspended` did not exist at all.

The SQL half runs against a real in-memory sqlite (CLAUDE.md 7.2) rather than a
fake cursor: the bucketing depends on the WHERE clause agreeing with the state
closed set, and a fake that returns the rows the test chose would confirm the
buckets while proving nothing about which rows the query actually selects.
"""

from __future__ import annotations

import json
import inspect

import aiosqlite
import pytest

from xbrain.common.enums import TASK_STATE
from xbrain.p3_task.runtime import main_wiring
from xbrain.p3_task.state.machine import TERMINAL_STATES
from xbrain.p3_task.state.task_state import (
    NON_TERMINAL_STATES, build_task_state, current_item, queue_item,
    read_task_state, suspended_item, wall_iso_to_epoch,
)


# INF-TS-1: 纯单测, 不碰设备(无 zenohd / 无底盘 / 无 ORIN 专属硬件).
pytestmark = pytest.mark.no_device


def _row(**kw):
    """A tasks row with every column task_state reads, overridable per case."""
    base = {"task_id": "t-1", "task_type": "goto", "state": "running",
            "priority": 80, "source": "cloud", "route_geo_id": "r-charge",
            "resume_policy": "continue", "started_at": "2026-09-02T09:07:27Z",
            "suspend_kind": None, "suspend_reason": None, "paused_at": None,
            "submit_seq": 7}
    base.update(kw)
    return base


# ------------------------------------------------------- the two the cloud needs

def test_route_id_comes_from_the_route_geo_id_column():
    """v2.0 S3.2 marks route_id mandatory ("实际加载路径 ID"); task.db calls the
    column route_geo_id and the placeholder broadcast carried neither.
    MUTATION: drop the route_id key, or read row["route_id"] -> red."""
    item = current_item(_row(route_geo_id="r-oil_area"))
    assert item["route_id"] == "r-oil_area"


def test_started_ts_is_the_epoch_of_the_stored_iso_string():
    """v2.0 S3.2 gives started_ts as a number; task.db stores started_at as a UTC
    ISO string. MUTATION: pass started_at through unconverted -> red."""
    item = current_item(_row(started_at="2026-09-02T09:07:27Z"))
    assert isinstance(item["started_ts"], float)
    # 2026-09-02T09:07:27Z, checked by round-tripping rather than by a literal
    # (a hand-computed epoch in a test is one more thing that can be wrong).
    from datetime import datetime, timezone
    expect = datetime(2026, 9, 2, 9, 7, 27, tzinfo=timezone.utc).timestamp()
    assert item["started_ts"] == expect


def test_a_task_that_never_started_has_a_null_started_ts():
    """started_at is NULL until dispatch; 0.0 would read as 1970 on the operator's
    screen. MUTATION: default to 0.0 -> red."""
    assert current_item(_row(started_at=None))["started_ts"] is None


def test_a_malformed_timestamp_yields_none_not_an_exception():
    """One bad row must not take the broadcast down for every other task.
    MUTATION: let fromisoformat raise -> red."""
    assert wall_iso_to_epoch("not-a-time") is None
    assert wall_iso_to_epoch(None) is None
    assert wall_iso_to_epoch("") is None


# ------------------------------------------------------------------ no invention

def test_progress_is_null_while_the_route_is_not_expanded():
    """11 S4.4 progress is 12-of-14 mandatory and anchored on
    task_route_snapshot, which is empty until the executor lands (EX-4). A
    fabricated block would put pct=0.0 on screen, indistinguishable from "just
    started" -- v2.0 S3.2 forbids exactly that.
    MUTATION: emit a zero-filled progress dict -> red."""
    assert current_item(_row())["progress"] is None
    assert suspended_item(_row(state="suspended"))["progress"] is None


def test_the_contract_field_name_for_the_task_type_is_type():
    """11 S4.4 current.type; task.db column is task_type.
    MUTATION: emit task_type instead -> red."""
    item = current_item(_row(task_type="patrol"))
    assert item["type"] == "patrol"
    assert "task_type" not in item


# --------------------------------------------------------------------- bucketing

def test_the_running_task_becomes_current_and_is_not_also_queued():
    """MUTATION: append the running row to queue as well -> red."""
    st = build_task_state([_row(task_id="t-run", state="running")])
    assert st["current"]["task_id"] == "t-run"
    assert st["queue"] == []
    assert st["suspended"] == []


def test_suspended_rows_go_to_the_suspended_list_with_their_kind_and_reason():
    """suspend_kind/suspend_reason are the v0.3 mandatory pair; the HMI shows
    "为什么停了" from them. MUTATION: bucket suspended into queue -> red."""
    st = build_task_state([_row(task_id="t-s", state="suspended",
                                suspend_kind="passive",
                                suspend_reason="operator_pause",
                                paused_at="2026-09-02T08:00:00Z")])
    assert st["queue"] == []
    item = st["suspended"][0]
    assert item["suspend_kind"] == "passive"
    assert item["suspend_reason"] == "operator_pause"
    assert isinstance(item["suspended_ts"], float)


def test_a_suspended_task_keeps_its_type_route_and_start_time():
    """v2.0 S3.2 makes task_type / route_id / started_ts mandatory on EVERY item,
    and a suspended task is one that RAN -- it loaded a route and it started, so
    both are real values, not unknowns. The S4.4 suspended example lists only the
    suspend-specific fields; carrying these three as well takes nothing away and
    is what keeps the paused list from showing typeless, routeless entries -- the
    list an operator looks at precisely when something stopped.
    MUTATION: drop any of the three from suspended_item -> red."""
    item = suspended_item(_row(state="suspended", task_type="goto",
                               route_geo_id="r-charge",
                               started_at="2026-09-02T09:07:27Z",
                               suspend_kind="yielding",
                               suspend_reason="preempted"))
    assert item["type"] == "goto"
    assert item["route_id"] == "r-charge"
    assert isinstance(item["started_ts"], float)


def test_a_queued_task_reports_no_route_because_it_has_not_loaded_one():
    """v2.0 defines route_id as the route ACTUALLY LOADED. A queued task has not
    started, so it has loaded nothing -- naming its intended route here would
    tell the operator it is under way. S4.4's queue entry has four fields for
    this reason.
    MUTATION: add route_id to queue_item -> red."""
    item = queue_item(_row(state="ready", route_geo_id="r-charge"))
    assert "route_id" not in item
    assert sorted(item) == ["priority", "state", "task_id", "type"]


def test_every_other_non_terminal_state_lands_in_the_queue():
    """blocked/pending/ready/scheduled all wait, so all four are queue.
    MUTATION: only bucket 'ready' -> red for the other three."""
    rows = [_row(task_id="t-%s" % s, state=s)
            for s in ("blocked", "pending", "ready", "scheduled")]
    st = build_task_state(rows)
    assert sorted(t["task_id"] for t in st["queue"]) == [
        "t-blocked", "t-pending", "t-ready", "t-scheduled"]
    assert st["current"] is None


def test_a_terminal_row_is_dropped_rather_than_queued():
    """Showing a finished task as pending is worse than not showing it.
    MUTATION: else-branch everything into queue -> red."""
    st = build_task_state([_row(task_id="t-done", state="done")])
    assert st["queue"] == [] and st["current"] is None and st["suspended"] == []


def test_the_non_terminal_set_is_derived_from_the_state_closed_set():
    """A thirteenth state must not fall out of every bucket. Derived, so adding
    one to TASK_STATE puts it in queue automatically.
    MUTATION: hardcode the five names -> red once TASK_STATE grows, and red now
    if the literal disagrees."""
    assert NON_TERMINAL_STATES == frozenset(TASK_STATE) - frozenset(TERMINAL_STATES)
    assert "running" in NON_TERMINAL_STATES
    assert not (NON_TERMINAL_STATES & frozenset(TERMINAL_STATES))


# ----------------------------------------------------------------- the real SQL

#: *** 直接用生产 DDL, NO 不在测试里手抄一份.
#: 本行原先是一份手抄的 12 列副本, 上面还标着注释 "the real SQL" -- 它与
#: 真 schema 已经漂移: 真表有 mission_json / total_steps / current_step /
#: result_json 等十几列. 2026-09-04 给 TaskState 加 mission_json 时才撞上
#: (夹具建的表里没有那一列, 测试报 no such column).
#: 这正是 CLAUDE.md 3.7 那条"人抄的清单会过期": 抄的时候是对的, 之后真
#: schema 每加一列, 这份副本就旧一分, 而没有任何判据会红.
from xbrain.p3_task.persistence.schema_task import DDL_TASKS as _DDL


#: 用例不关心但真表要求非空的列, 按类型给中性值.
#: *** 由 PRAGMA table_info [现算], NO 不在这里手列一份.
#: 手列过一版, 补完 mission_json 撞 total_steps, 补完又撞 created_ms --
#: 每加一列就再漂一次(CLAUDE.md 3.7). 现算之后, 生产 schema 再加多少个
#: NOT NULL 列, 这里都不用动.
_NEUTRAL = {"INTEGER": 0, "REAL": 0.0, "TEXT": ""}

#: 有闭集约束(CHECK)的列不能给空串, 单独指定. 值取自生产上真会出现的那个.
_SEEDED = {"mission_json": "{}", "resume_policy": "continue",
           "source": "cloud", "task_type": "goto", "state": "ready"}


async def _seed(conn, rows):
    await conn.execute(_DDL)
    cur = await conn.execute("PRAGMA table_info(tasks)")
    cols = await cur.fetchall()
    required = [(c[1], (c[2] or "TEXT").upper())
                for c in cols if c[3] and c[4] is None]
    for r in rows:
        for name, typ in required:
            if name in r:
                continue
            r.setdefault(name, _SEEDED.get(
                name, _NEUTRAL.get(typ.split("(")[0], "")))
        cols_sql = ", ".join(r)
        marks = ", ".join("?" for _ in r)
        await conn.execute("INSERT INTO tasks (%s) VALUES (%s)"
                           % (cols_sql, marks), tuple(r.values()))
    await conn.commit()


@pytest.mark.asyncio
async def test_terminal_rows_do_not_occupy_the_read_limit():
    """The WHERE clause must do the filtering, NOT the bucketing downstream.

    Bucketing drops terminal rows anyway, so seeding a few and checking they are
    absent passes with or without the WHERE -- the downstream net hides the hole
    (3.2). The limit is what makes the difference observable, and it is the
    failure that actually happened once: list_by_priority had no state filter,
    terminal rows took the top of the ordering, and 107 pending tasks became
    invisible to the scheduler rather than merely queued.

    Here three terminal rows outrank the live one. With the WHERE, the live task
    is one of three rows read and shows up. Without it, the terminal rows consume
    the limit and the queue comes back empty.
    MUTATION: drop the WHERE from read_task_state -> red."""
    async with aiosqlite.connect(":memory:") as conn:
        await _seed(conn, [
            _row(task_id="t-done", state="done", priority=99, submit_seq=1),
            _row(task_id="t-cancelled", state="cancelled", priority=98,
                 submit_seq=2),
            _row(task_id="t-failed", state="failed", priority=97, submit_seq=3),
            _row(task_id="t-rdy", state="ready", priority=40, submit_seq=4),
        ])
        st = await read_task_state(conn, limit=3)
    assert [t["task_id"] for t in st["queue"]] == ["t-rdy"], (
        "a live task fell outside the read limit because terminal rows were "
        "selected alongside it")


@pytest.mark.asyncio
async def test_the_queue_is_ordered_the_way_the_scheduler_orders_it():
    """Head of queue must be the task that would actually run next (15 S6.1:
    priority DESC, submit_seq ASC). MUTATION: ORDER BY submit_seq only -> red."""
    async with aiosqlite.connect(":memory:") as conn:
        await _seed(conn, [
            _row(task_id="t-low", state="ready", priority=40, submit_seq=1),
            _row(task_id="t-high", state="ready", priority=90, submit_seq=2),
            _row(task_id="t-mid", state="ready", priority=80, submit_seq=3),
        ])
        st = await read_task_state(conn)
    assert [t["task_id"] for t in st["queue"]] == ["t-high", "t-mid", "t-low"]


@pytest.mark.asyncio
async def test_the_broadcast_carries_the_three_contract_lists_even_when_idle():
    """A consumer must not have to special-case "no tasks yet".
    MUTATION: return {} when there are no rows -> red."""
    async with aiosqlite.connect(":memory:") as conn:
        await _seed(conn, [])
        st = await read_task_state(conn)
    assert st["current"] is None
    assert st["queue"] == [] and st["suspended"] == []


# -------------------------------------------------------------------- the wiring

def test_the_transition_callback_publishes_the_whole_state_not_a_delta():
    """One task starting changes the queue for all the others, so a per-task
    delta cannot express the transition. Asserted on p3's real source: the
    callback is built by a factory the tests can reach, but what it publishes
    is a closure over the loop's connection.
    MUTATION: put a single-task dict back -> red."""
    src = inspect.getsource(main_wiring._make_publish)
    assert "publish_state" in src
    assert "active_task" not in src, (
        "the placeholder single-task shape is back in the transition path")


def test_state_task_has_a_1_hz_floor_as_well_as_the_event():
    """11 S2.2.2 is "event + 1 Hz". Without the floor an idle system and a dead
    P3 look identical on the wire.
    MUTATION: delete the periodic block -> red."""
    src = inspect.getsource(main_wiring._amain)
    # The GUARD, not just the names: a neutered condition (if False:) leaves
    # every one of those identifiers in the source and passes a substring check.
    assert "now - last_task_state >= TASK_STATE_PERIOD_S" in src, (
        "the 1 Hz floor for state/task is gone or its guard was neutered")
    assert "last_task_state = now" in src, (
        "the period timer is never reset -- the floor becomes a busy publish")
    assert "await _publish_task_state()" in src


def test_both_halves_publish_through_one_builder():
    """Two builders of the same broadcast is how the placeholder shape survived
    -- nothing compared them. MUTATION: inline a second json.dumps for the
    periodic path -> red."""
    src = inspect.getsource(main_wiring._amain)
    assert src.count("read_task_state(") == 1, (
        "state/task is being built in more than one place")


# --- waypoint_total (2026-09-04 终测) ---------------------------------

def test_waypoint_total_comes_from_the_frozen_mission_not_the_route_table():
    """*** 航点数来自 mission_json, 与 EX-1 路径展开无关.

    S4.4 对 waypoint_total 的措辞是"取自快照, 不是当前 route 表" --
    GOTO_KEYPOINT 的航点随命令而来并冻结在 mission_json 里, 那就是这条
    任务自己的快照, 拿得到.

    实测背景: 终测里一条带 1 个航点的任务, 甲方界面显示 "0/0" --
    p5 从 progress.waypoint_total 取 total_count, 而 progress 整块是 None,
    于是落到 `or 0`. 编一个 0 与编一个 100 是同一类错(3.1).

    MUTATION: _partial_progress 改回恒返回 None -> 这里红.
    """
    from xbrain.p3_task.state.task_state import current_item
    row = {"task_id": "t1", "task_type": "goto", "state": "running",
           "priority": 50, "source": "cloud", "route_geo_id": "r-charge",
           "resume_policy": "continue", "started_at": None,
           "mission_json": json.dumps({"params": {"waypoints": [
               {"id": "w-a"}, {"id": "w-b"}, {"id": "w-c"}]}})}
    item = current_item(row)
    assert item["progress"]["waypoint_total"] == 3


def test_route_rev_stays_absent_because_its_only_legal_source_is_the_snapshot():
    """*** 与上一条配对: 能填的填, 不能填的一个都不许填.

    S4.4 说 route_rev 是"任务开始时[锁定]的路径版本", 唯一合法来源是
    task_route_snapshot(EX-1 未建). 去读 routes.rev 正是该条明令禁止的:
    路径中途被人改过, 报出来的版本就与机器人正在走的不是同一份.

    MUTATION: 在 _partial_progress 里补一个 route_rev -> 这里红.
    """
    from xbrain.p3_task.state.task_state import current_item
    row = {"task_id": "t1", "task_type": "goto", "state": "running",
           "priority": 50, "source": "cloud", "route_geo_id": "r-charge",
           "resume_policy": "continue", "started_at": None,
           "mission_json": json.dumps({"params": {"waypoints": [{"id": "w-a"}]}})}
    prog = current_item(row)["progress"]
    assert "route_rev" not in prog
    for k in ("waypoint_index", "seg_done_m", "route_total_m",
              "loop_index", "loop_total"):
        assert k not in prog, k


def test_a_task_with_no_waypoints_reports_no_progress_at_all():
    """没有航点表述的任务(充电/返航)整块 progress 缺席, NO 不报 0 个航点.

    0 的意思是"零个航点", 与"不知道"是两件事; v2.0 的 total_count 只作
    数量显示, 一个假的 0 会让操作员以为任务是空的.
    """
    from xbrain.p3_task.state.task_state import current_item
    row = {"task_id": "t1", "task_type": "return_home", "state": "running",
           "priority": 95, "source": "charge", "route_geo_id": None,
           "resume_policy": "continue", "started_at": None,
           "mission_json": "{}"}
    assert current_item(row)["progress"] is None
