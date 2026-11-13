"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u190_cloud_patrol_check.py
Brief: 99 U190 -- a cloud PATROL_ROUTE naming an unknown path is refused at submit, like GOTO_KEYPOINT

Description:
User ruling 2026-11-13 ("加上检查"): the cloud patrol gets GOTO_KEYPOINT's
submit-time path check (99 U164) -- E_NOT_FOUND (v2.0 1003) with detail
{reason: route_not_found, field: recorded_path_id}, nothing stored. Before
it, the task was accepted and failed later at the scheduler's PC-8. Each
case names the mutant it kills; the voice channel keeps its behaviour (the
check is the cloud contract's, v2.0 S3.1).
"""
from __future__ import annotations  # postponed annotations

import aiosqlite  # in-memory task db
import pytest  # markers

from xbrain.common.errors import E_NOT_FOUND  # expected code
from xbrain.p3_task.dao.tasks_dao import TasksDAO  # the DAO the context needs
from xbrain.p3_task.ingest.goto_precheck import cloud_goto_refusal  # under test
from xbrain.p3_task.ingest.task_apply import (  # submit path
    TaskContext,  # what the wiring builds
    handle_task_payload,  # the cmd/task entry
)
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # task DDL

pytestmark = pytest.mark.no_device  # pure logic + in-memory sqlite


async def _route_exists(gid):  # geo.db stand-in: only r-a is a live path
    return gid == "r-a"  # every other id is unknown


async def _no_keypoint(_wid):  # a patrol must never ask for a waypoint
    raise AssertionError("a patrol has no waypoints to look up")


def _patrol(route_id, task_id="t-cloud-1"):  # the task body the gateway builds (U190)
    return {"task_id": task_id, "type": "patrol", "route_id": route_id,
            "params": {"slots": {"loops": 1}}}


@pytest.mark.asyncio
async def test_the_refusal_names_the_path_and_needs_no_waypoints():
    """MUTATION: the patrol early return removed -> the stray waypoint list
    is looked up -> _no_keypoint raises -> red; the path check removed ->
    None for the unknown path -> red."""
    r = await cloud_goto_refusal(_patrol("r-x"), fence_set=None, keypoint=_no_keypoint,
                                 route_exists=_route_exists)  # unknown path
    assert r[0] == E_NOT_FOUND  # 1003 at the gateway
    assert r[2] == {"reason": "route_not_found", "field": "recorded_path_id",
                    "recorded_path_id": "r-x"}  # the v2.0 S3.1 locator
    ok = dict(_patrol("r-a"), params={"slots": {}, "waypoints": [{"id": "w-9"}]})  # stray list
    assert await cloud_goto_refusal(ok, fence_set=None, keypoint=_no_keypoint,
                                    route_exists=_route_exists) is None  # path only


async def _ctx():  # a fresh task db and the context with the cloud check wired
    conn = await aiosqlite.connect(":memory:")  # in-memory (CLAUDE.md 7.2)
    for stmt in ALL_DDL_STATEMENTS:  # schema
        await conn.execute(stmt)  # one DDL
    await conn.commit()  # ready

    async def precheck(task):  # what main_wiring's _cloud_goto_precheck does
        return await cloud_goto_refusal(task, fence_set=None, keypoint=_no_keypoint,
                                        route_exists=_route_exists)
    return conn, TaskContext(conn, TasksDAO(conn), cloud_goto_precheck=precheck)


async def _rows(conn):  # stored tasks
    cur = await conn.execute("SELECT COUNT(*) FROM tasks")  # all rows
    return (await cur.fetchone())[0]  # count


@pytest.mark.asyncio
async def test_a_cloud_patrol_on_an_unknown_path_is_refused_and_not_stored():
    """MUTATION: task_apply still limited to type goto -> accepted, one row -> red."""
    conn, ctx = await _ctx()  # wired context
    try:
        ack = await handle_task_payload({"cmd_id": "c-1", "action": "submit", "source": "cloud",
                                         "task": _patrol("r-x")}, ctx,
                                        now_mono_ms=1, date_str="20261113")  # unknown path
        assert ack["result"] == "rejected" and ack["code"] == E_NOT_FOUND  # in the ack
        assert ack["detail"]["reason"] == "route_not_found"  # named
        assert await _rows(conn) == 0  # nothing stored
        good = await handle_task_payload({"cmd_id": "c-2", "action": "submit", "source": "cloud",
                                          "task": _patrol("r-a", "t-cloud-2")}, ctx,
                                         now_mono_ms=2, date_str="20261113")  # known path
        assert good["result"] == "accepted" and await _rows(conn) == 1  # stored
    finally:  # always
        await conn.close()  # cleanup


@pytest.mark.asyncio
async def test_the_voice_channel_is_not_checked_at_submit():
    """The check is the cloud contract's; voice keeps PC-8 at the scheduler.
    MUTATION: the source condition dropped -> rejected -> red."""
    conn, ctx = await _ctx()  # wired context
    try:
        ack = await handle_task_payload({"cmd_id": "c-3", "action": "submit", "source": "voice",
                                         "task": _patrol("r-x", "t-voice-1")}, ctx,
                                        now_mono_ms=3, date_str="20261113")  # unknown path
        assert ack["result"] == "accepted"  # unchanged behaviour
    finally:  # always
        await conn.close()  # cleanup
