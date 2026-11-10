"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_goto_precheck.py
Brief: 11 S12.1.2 W2 -- P3 refuses an HMI goto at submit: positioning failed / fence / keypoint

Description:
The W2 table gives P3 three refusals for an HMI goto; until 2026-11-10 none
existed and every HMI goto was acknowledged "accepted". These cases pin the
rules (ingest/goto_precheck) and their place in the submit path (task_apply):
refused in the ACK and nothing written, HMI-originated gotos only.

Each assertion names the mutant that turns it red (CLAUDE.md 3.3).
"""

from __future__ import annotations

import aiosqlite
import pytest

from xbrain.common.errors import E_DEGRADED, E_NOT_FOUND, E_OUT_OF_FENCE
from xbrain.p3_task.dao.tasks_dao import TasksDAO
from xbrain.p3_task.ingest.goto_precheck import fence_refusal, hmi_goto_refusal
from xbrain.p3_task.ingest.task_apply import TaskContext, handle_task_payload
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS

pytestmark = pytest.mark.no_device

_FIXED = {"lat": 31.1585, "lon": 121.3913, "fix_type": "rtk_fixed"}


def _square(lat0, lon0, half, role, poly_id="p"):
    """An axis-aligned square of `half` degrees around (lat0, lon0)."""
    return {"poly_id": poly_id, "role": role, "vertices": [
        {"lat": lat0 - half, "lon": lon0 - half}, {"lat": lat0 - half, "lon": lon0 + half},
        {"lat": lat0 + half, "lon": lon0 + half}, {"lat": lat0 + half, "lon": lon0 - half}]}


async def _no_keypoint(_wid):
    return None


async def _kp(wid):
    return (31.1585, 121.3913) if wid == "w-1" else None


def _goto(**params):
    return {"type": "goto", "params": params}


# ------------------------------------------------------------- the rules --

@pytest.mark.asyncio
@pytest.mark.parametrize("pose", [None, {"fix_type": "single"},
                                  {"fix_type": "no_fix"}, {"lat": 1.0}])
async def test_failed_positioning_is_degraded(pose):
    # 99 U142: only single and no fix fail; no pose and no fix_type fail too.
    # RED MUTANT: drop the check -> None; or test `fix != "rtk_fixed"` ->
    # see the float case below.
    ref = await hmi_goto_refusal(_goto(lat=31.1585, lon=121.3913), pose=pose,
                                 fence_set=None, keypoint=_no_keypoint)
    assert ref is not None and ref[0] == E_DEGRADED
    assert ref[2]["reason"] == "positioning_failed"


@pytest.mark.asyncio
@pytest.mark.parametrize("fix", ["rtk_float", "dgps", "rtk_fixed"])
async def test_float_and_dgps_take_tasks(fix):
    # U142 verbatim: float and dgps take tasks like fixed (slower).
    # RED MUTANT: refuse anything but rtk_fixed -> these go red.
    ref = await hmi_goto_refusal(_goto(lat=31.1585, lon=121.3913),
                                 pose={"fix_type": fix}, fence_set=None,
                                 keypoint=_no_keypoint)
    assert ref is None


@pytest.mark.asyncio
async def test_missing_keypoint_is_not_found_and_a_live_one_passes():
    # RED MUTANT: skip the lookup -> the missing one is accepted.
    gone = await hmi_goto_refusal(_goto(waypoint_id="w-9"), pose=_FIXED,
                                  fence_set=None, keypoint=_kp)
    ok = await hmi_goto_refusal(_goto(waypoint_id="w-1"), pose=_FIXED,
                                fence_set=None, keypoint=_kp)
    assert gone[0] == E_NOT_FOUND and ok is None


def test_target_outside_allow_is_refused_inside_is_not():
    fs = {"polygons": [_square(31.1585, 121.3913, 0.001, "allow")]}
    # RED MUTANT: invert the allow test -> both answers flip.
    assert fence_refusal(31.1585, 121.3913, fs) is None
    out = fence_refusal(31.1700, 121.3913, fs)
    assert out[0] == E_OUT_OF_FENCE and out[2]["subject"] == "target"
    assert out[2]["role"] == "allow"


def test_target_inside_forbid_is_refused():
    fs = {"polygons": [_square(31.1585, 121.3913, 0.01, "allow"),
                       _square(31.1585, 121.3913, 0.0005, "forbid", "fb-1")]}
    # RED MUTANT: ignore forbid polygons -> None.
    ref = fence_refusal(31.1585, 121.3913, fs)
    assert ref[0] == E_OUT_OF_FENCE and ref[2]["poly_id"] == "fb-1"


def test_warning_and_speed_limit_constrain_nothing_and_no_fence_means_no_refusal():
    # 11 S9A.10A: warning / speed_limit are not motion constraints.
    # RED MUTANT: treat every role as forbid -> refused.
    fs = {"polygons": [_square(31.1585, 121.3913, 0.001, "warning"),
                       _square(31.1585, 121.3913, 0.001, "speed_limit")]}
    assert fence_refusal(31.1585, 121.3913, fs) is None
    assert fence_refusal(31.1585, 121.3913, None) is None


# ---------------------------------------------------- the submit path --

async def _ctx(precheck):
    conn = await aiosqlite.connect(":memory:")
    for stmt in ALL_DDL_STATEMENTS:
        await conn.execute(stmt)
    await conn.commit()
    return conn, TaskContext(conn, TasksDAO(conn), goto_precheck=precheck)


async def _count(conn):
    cur = await conn.execute("SELECT COUNT(*) FROM tasks")
    return (await cur.fetchone())[0]


def _submit(source, **params):
    return {"cmd_id": "h-x", "action": "submit", "source": source,
            "task": {"type": "goto", "params": params}}


@pytest.mark.asyncio
async def test_an_hmi_goto_refused_by_the_precheck_is_rejected_and_not_stored():
    # RED MUTANT: no goto_precheck call in _submit -> accepted, one row.
    async def refuse(_task):
        return (E_DEGRADED, "positioning failed", {"reason": "positioning_failed"})
    conn, ctx = await _ctx(refuse)
    try:
        ack = await handle_task_payload(_submit("hmi", lat=31.1585, lon=121.3913,
                                                arrive_radius_m=1.0),
                                        ctx, now_mono_ms=1, date_str="20261110")
        assert ack["result"] == "rejected" and ack["code"] == E_DEGRADED
        assert ack["detail"]["reason"] == "positioning_failed"
        assert await _count(conn) == 0
    finally:
        await conn.close()


@pytest.mark.asyncio
async def test_other_channels_are_not_prechecked():
    # The W2 table is the HMI's; a voice goto keeps its old behaviour.
    # RED MUTANT: drop the `source == "hmi"` condition -> the voice goto is
    # refused here too.
    calls = []

    async def refuse(task):
        calls.append(task)
        return (E_DEGRADED, "positioning failed", {"reason": "positioning_failed"})
    conn, ctx = await _ctx(refuse)
    try:
        ack = await handle_task_payload(_submit("voice", waypoint_id="w-1"),
                                        ctx, now_mono_ms=1, date_str="20261110")
        assert ack["result"] == "accepted" and calls == []
        assert await _count(conn) == 1
    finally:
        await conn.close()


@pytest.mark.asyncio
async def test_an_hmi_goto_that_passes_is_stored_with_its_command_text():
    async def allow(_task):
        return None
    conn, ctx = await _ctx(allow)
    try:
        ack = await handle_task_payload(
            _submit("hmi", waypoint_id="w-1", text="HMI 点击前往 关键点 1号测试点"),
            ctx, now_mono_ms=1, date_str="20261110")
        assert ack["result"] == "accepted" and ack["detail"]["task_id"]
        cur = await conn.execute("SELECT command_text FROM tasks")
        assert (await cur.fetchone())[0] == "HMI 点击前往 关键点 1号测试点"
    finally:
        await conn.close()
