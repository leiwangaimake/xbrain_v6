"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_teach_runtime.py
Brief: cmd/teach live session against real task.db / geo.db / fence.db (batch 4b)

Description:
The recording session end to end: start -> drive (pose samples) -> mark -> undo
-> finish -> save, with the geometry landing in geo.db / fence.db through the
existing commit writers. Uses in-memory databases and an injected monotonic
clock, so the 1 Hz sampling gate is exercised without sleeping.

The cases that carry the batch:

  * arming refuses while a state source is MISSING, and names it. On this build
    nothing publishes health/summary, state/robot or state/power, so this is
    also the honest current answer for "can the robot record yet" -- the test
    exists so that answer cannot quietly become "yes" by defaulting a gate.
  * a saved fence is a DRAFT. Saving is not activating (S12A.7 constraint 1).
  * points are buffered in task.db's memory table, not appended to geo.db, and
    the buffer is cleared on finish and on discard (S12A.6.1 cleanup contract).
  * a mismatched session_id is refused (S12A.4).
"""
from __future__ import annotations

import json

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.common.errors import (
    E_BUSY, E_NAME_CONFLICT, E_TEACH_GEOMETRY, E_TEACH_QUALITY, E_TEACH_STATE,
)
from xbrain.p3_task.persistence.schema_geo import (
    FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS,
)
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p3_task.teach.runtime import TeachRuntime

pytestmark = pytest.mark.no_device

_LAT, _LON = 34.6970, 135.5050
_DEG_PER_M = 1.0 / 111320.0


async def _open(statements):
    conn = await aiosqlite.connect(":memory:")
    for stmt in statements:
        await conn.execute(stmt)
    await conn.commit()
    return conn


@pytest_asyncio.fixture
async def rt():
    task = await _open(ALL_DDL_STATEMENTS)
    geo = await _open(GEO_DB_STATEMENTS)
    fence = await _open(FENCE_DB_STATEMENTS)
    runtime = TeachRuntime(task, geo, fence, boot_id="b0")
    # Healthy caches by default; the missing-source case clears them.
    runtime.update_pose({"lat": _LAT, "lon": _LON, "fix_type": "rtk_fixed",
                         "heading_rad": 0.0, "heading_valid": True,
                         "heading_level": 1, "speed_mps": 0.0}, 100.0)
    runtime.update_health({"allow_motion": True})
    runtime.update_robot({"hes": False, "estop_path": "up"})
    runtime.update_power({"soc_pct": 80.0})
    runtime.update_teleop({"sources": [{"device": "gamepad",
                                       "stale": False, "alive": True}]})
    yield runtime
    for c in (task, geo, fence):
        await c.close()


def _cmd(action, **over):
    base = {"cmd_id": f"c-{action}", "action": action,
            "issuer": {"src": "p4_agent", "channel": "local_voice"}}
    base.update(over)
    return base


async def _start(rt, kind="route", t=100.0, cmd_id="c-start"):
    # Refresh the pose at t: state/pose runs at 10 Hz on the robot, and arming
    # reads it through the same staleness gate sampling does -- a fixture that
    # let it age would be testing the freshness check, not the start path.
    rt.update_pose({"lat": _LAT, "lon": _LON, "fix_type": "rtk_fixed",
                    "heading_rad": 0.0, "heading_valid": True,
                    "heading_level": 1, "speed_mps": 0.0}, t)
    ack = await rt.handle({"cmd_id": cmd_id, "action": "start",
                           "issuer": {"src": "p4_agent",
                                      "channel": "local_voice"},
                           "start": {"kind": kind}},
                          now_mono_s=t, now_ms=1000)
    assert ack["result"] == "accepted", ack
    return ack["detail"]["session_id"]


async def _drive(rt, session_id, metres, t0, count):
    """Feed `count` pose samples a second apart, `metres` further each time."""
    for i in range(count):
        t = t0 + i
        rt.update_pose({"lat": _LAT + metres * (i + 1) * _DEG_PER_M,
                        "lon": _LON, "fix_type": "rtk_fixed",
                        "heading_rad": 0.0, "heading_valid": True,
                        "heading_level": 1, "speed_mps": 1.0}, t)
        await rt.offer_pose(t, 1000 + i)


async def _buffer_rows(rt):
    cur = await rt._task_conn.execute(
        "SELECT key FROM memory WHERE key LIKE 'teach:%'")
    return await cur.fetchall()


# ------------------------------------------------------------- arming -------

@pytest.mark.asyncio
async def test_start_refuses_and_names_a_missing_state_source(rt):
    """*** A gate it cannot evaluate is a refusal, and it says which source.

    MUTATION: treat an absent cache as healthy (allow_motion defaulting to True,
    say) and recording arms on a stack where nothing reports health, e-stop
    path or battery -- exactly the three that make the recording state safe.
    """
    rt.update_health.__self__._health = None      # simulate never received
    ack = await rt.handle(_cmd("start", start={"kind": "route"}),
                          now_mono_s=100.0, now_ms=1)
    assert ack["result"] == "rejected" and ack["code"] == E_TEACH_QUALITY
    assert ack["detail"]["reason"] == "state_unavailable"
    assert "health/summary" in ack["detail"]["missing"]


@pytest.mark.asyncio
async def test_start_refuses_while_a_patrol_runs(rt):
    """S12A.3 check 2: a recording and a driving task cannot share the robot."""
    ack = await rt.handle(_cmd("start", start={"kind": "route"}),
                          now_mono_s=100.0, now_ms=1,
                          running_task_types=("patrol",))
    assert ack["result"] == "rejected" and ack["code"] == E_BUSY


@pytest.mark.asyncio
async def test_start_refuses_without_a_nonvoice_estop(rt):
    """Check 7 again, this time through the runtime: no live gamepad source AND
    a down e-stop path. MUTATION: OR the two criteria the wrong way round (AND)
    or drop the check -- recording arms with no way to stop the robot."""
    rt.update_teleop({"sources": []})
    rt.update_robot({"hes": False, "estop_path": "down"})
    ack = await rt.handle(_cmd("start", start={"kind": "route"}),
                          now_mono_s=100.0, now_ms=1)
    assert ack["result"] == "rejected"
    assert ack["detail"]["reason"] == "no_nonvoice_estop"


# ------------------------------------------------------------ recording -----

@pytest.mark.asyncio
async def test_record_drive_finish_save_writes_a_route(rt):
    """The whole F01 -> F02 -> F03 path."""
    sid = await _start(rt)
    await _drive(rt, sid, metres=2.0, t0=101.0, count=5)
    finish = await rt.handle(_cmd("finish", session_id=sid),
                             now_mono_s=110.0, now_ms=2000)
    assert finish["detail"]["state"] == "finalizing"
    assert finish["detail"]["point_count"] == 5
    assert finish["detail"]["validation"]["ok"] is True
    save = await rt.handle(
        _cmd("save", session_id=sid, save={"name": "east gate route"}),
        now_mono_s=111.0, now_ms=2100)
    assert save["result"] == "accepted", save
    applied = save["detail"]["applied"]
    assert applied["name"] == "east gate route" and applied["point_count"] == 5
    assert applied["total_len_m"] > 5.0
    cur = await rt._geo_conn.execute(
        "SELECT geo_id, name, path_points FROM routes")
    row = await cur.fetchone()
    assert row[1] == "east gate route"
    assert len(json.loads(row[2])) == 5


@pytest.mark.asyncio
async def test_points_are_buffered_in_task_db_and_cleared_on_finish(rt):
    """S12A.6.1: the buffer lives in task.db so a half-recorded route never
    appears in geo.db. MUTATION: append straight to `routes` and a scheduler can
    dispatch a task onto a path the operator is still driving."""
    sid = await _start(rt)
    await _drive(rt, sid, metres=2.0, t0=101.0, count=3)
    assert len(await _buffer_rows(rt)) == 3
    cur = await rt._geo_conn.execute("SELECT COUNT(*) FROM routes")
    assert (await cur.fetchone())[0] == 0, "geo.db must be untouched until save"
    await rt.handle(_cmd("finish", session_id=sid), now_mono_s=110.0,
                    now_ms=2000)
    await rt.handle(_cmd("save", session_id=sid, save={"name": "r1"}),
                    now_mono_s=111.0, now_ms=2100)
    assert await _buffer_rows(rt) == []


@pytest.mark.asyncio
async def test_discard_clears_the_buffer_and_writes_nothing(rt):
    sid = await _start(rt)
    await _drive(rt, sid, metres=2.0, t0=101.0, count=3)
    ack = await rt.handle(_cmd("discard", session_id=sid, reason="not_needed"),
                          now_mono_s=110.0, now_ms=2000)
    assert ack["detail"]["discarded_points"] == 3
    assert await _buffer_rows(rt) == []
    cur = await rt._geo_conn.execute("SELECT COUNT(*) FROM routes")
    assert (await cur.fetchone())[0] == 0


@pytest.mark.asyncio
async def test_pause_stops_sampling_and_resume_restarts_it(rt):
    sid = await _start(rt)
    await _drive(rt, sid, metres=2.0, t0=101.0, count=2)
    await rt.handle(_cmd("pause", session_id=sid), now_mono_s=103.0, now_ms=1)
    await _drive(rt, sid, metres=2.0, t0=104.0, count=3)
    state = rt.teach_state_payload(107.0)
    assert state["stats"]["point_count"] == 2, "paused must not sample"
    await rt.handle(_cmd("resume", session_id=sid), now_mono_s=108.0, now_ms=1)
    await _drive(rt, sid, metres=2.0, t0=109.0, count=2)
    assert rt.teach_state_payload(111.0)["stats"]["point_count"] == 4


@pytest.mark.asyncio
async def test_mark_and_undo(rt):
    sid = await _start(rt)
    await _drive(rt, sid, metres=2.0, t0=101.0, count=2)
    mark = await rt.handle(_cmd("mark", session_id=sid), now_mono_s=103.2,
                           now_ms=1)
    assert mark["detail"]["manual_count"] == 1
    undo = await rt.handle(_cmd("undo", session_id=sid, undo={"count": 2}),
                           now_mono_s=104.0, now_ms=1)
    assert undo["detail"]["removed"] == 2
    assert undo["detail"]["point_count"] == 1
    # The persisted buffer follows the undo -- otherwise a crash after an undo
    # would restore the points the operator just removed.
    assert len(await _buffer_rows(rt)) == 1


@pytest.mark.asyncio
async def test_stale_pose_is_refused_where_dedup_cannot_help(rt):
    """A frozen pose must not be recorded as a fresh position.

    The timed sampling path is covered twice over -- a stale pose is also an
    unchanged one, so the 0.5 m dedup drops it regardless. The case that
    isolates the age check is MARK: F05 bypasses both the interval and the
    distance gate by design, so staleness is the only thing left between a dead
    pose stream and a vertex placed at a position the robot left minutes ago.

    MUTATION: delete the age check in _pose_fresh -- the timed-sampling
    assertion below stays green (dedup covers it) and THIS one goes red, which
    is why the mark case is here rather than a second sampling case.
    """
    sid = await _start(rt)
    await _drive(rt, sid, metres=2.0, t0=101.0, count=2)
    # Pose stops updating; time moves on.
    assert await rt.offer_pose(120.0, 1) is False
    assert rt.teach_state_payload(120.0)["stats"]["point_count"] == 2
    mark = await rt.handle(_cmd("mark", session_id=sid), now_mono_s=120.0,
                           now_ms=1)
    assert mark["result"] == "rejected" and mark["code"] == E_TEACH_QUALITY
    assert rt.teach_state_payload(120.0)["stats"]["point_count"] == 2


@pytest.mark.asyncio
async def test_wrong_session_id_is_refused(rt):
    """S12A.4: a stale command must not land in the current session."""
    sid = await _start(rt)
    ack = await rt.handle(_cmd("finish", session_id="ts-other"),
                          now_mono_s=110.0, now_ms=1)
    assert ack["result"] == "rejected" and ack["code"] == E_TEACH_STATE
    assert ack["detail"]["open_session"] == sid


@pytest.mark.asyncio
async def test_duplicate_cmd_id_replays(rt):
    sid = await _start(rt)
    await _drive(rt, sid, metres=2.0, t0=101.0, count=2)
    first = await rt.handle(_cmd("mark", session_id=sid), now_mono_s=103.2,
                            now_ms=1)
    again = await rt.handle(_cmd("mark", session_id=sid), now_mono_s=103.4,
                            now_ms=1)
    assert first["result"] == "accepted" and again["result"] == "duplicate"
    assert rt.teach_state_payload(104.0)["stats"]["manual_count"] == 1


@pytest.mark.asyncio
async def test_second_session_is_refused_while_one_is_open(rt):
    await _start(rt)
    ack = await rt.handle(
        {"cmd_id": "c-2", "action": "start",
         "issuer": {"src": "p4_agent", "channel": "local_voice"},
         "start": {"kind": "route"}}, now_mono_s=120.0, now_ms=1)
    assert ack["result"] == "rejected"


# --------------------------------------------------------------- fence ------

@pytest.mark.asyncio
async def test_saved_fence_is_a_draft_not_active(rt):
    """*** S12A.7 constraint 1 and 11 S7.9.5's separate L2 cell for activation.
    MUTATION: commit the fence as active -- saying "save fence" then changes
    where the robot may go, with no second confirmation anywhere."""
    sid = await _start(rt, kind="fence")
    # A square about 20 m on a side, driven as 4 corners.
    import math
    d = 20.0 * _DEG_PER_M
    dlon = d / math.cos(math.radians(_LAT))
    corners = [(_LAT, _LON), (_LAT + d, _LON), (_LAT + d, _LON + dlon),
               (_LAT, _LON + dlon)]
    for i, (la, lo) in enumerate(corners):
        rt.update_pose({"lat": la, "lon": lo, "fix_type": "rtk_fixed",
                        "heading_rad": 0.0, "heading_valid": True,
                        "heading_level": 1, "speed_mps": 1.0}, 101.0 + i)
        await rt.offer_pose(101.0 + i, 1000 + i)
    await rt.handle(_cmd("finish", session_id=sid), now_mono_s=110.0,
                    now_ms=2000)
    save = await rt.handle(
        _cmd("save", session_id=sid, save={"name": "north zone"}),
        now_mono_s=111.0, now_ms=2100)
    assert save["result"] == "accepted", save
    assert save["detail"]["activated"] is False
    cur = await rt._fence_conn.execute(
        "SELECT role, state FROM fences WHERE name='north zone'")
    assert await cur.fetchone() == ("forbid", "draft")


@pytest.mark.asyncio
async def test_too_few_points_blocks_the_save(rt):
    """S12A.7 too_few_points is a BLOCKING issue. MUTATION: treat every issue as
    advisory and a one-point 'route' is committed, then dispatched."""
    sid = await _start(rt)
    await _drive(rt, sid, metres=2.0, t0=101.0, count=1)
    await rt.handle(_cmd("finish", session_id=sid), now_mono_s=110.0, now_ms=1)
    save = await rt.handle(_cmd("save", session_id=sid, save={"name": "r"}),
                           now_mono_s=111.0, now_ms=1)
    assert save["result"] == "rejected" and save["code"] == E_TEACH_GEOMETRY
    cur = await rt._geo_conn.execute("SELECT COUNT(*) FROM routes")
    assert (await cur.fetchone())[0] == 0


@pytest.mark.asyncio
async def test_duplicate_name_is_refused_without_overwrite(rt):
    sid = await _start(rt)
    await _drive(rt, sid, metres=2.0, t0=101.0, count=3)
    await rt.handle(_cmd("finish", session_id=sid), now_mono_s=110.0, now_ms=1)
    await rt.handle(_cmd("save", session_id=sid, save={"name": "same"}),
                    now_mono_s=111.0, now_ms=1)
    sid2 = await _start(rt, t=200.0, cmd_id="c-start2")
    await _drive(rt, sid2, metres=2.0, t0=201.0, count=3)
    await rt.handle(_cmd("finish", session_id=sid2, cmd_id="c-f2"),
                    now_mono_s=210.0, now_ms=1)
    clash = await rt.handle(
        {"cmd_id": "c-s2", "action": "save", "session_id": sid2,
         "issuer": {"src": "p4_agent", "channel": "local_voice"},
         "save": {"name": "same"}}, now_mono_s=211.0, now_ms=1)
    assert clash["result"] == "rejected" and clash["code"] == E_NAME_CONFLICT


# ------------------------------------------------------------ mark_once -----

@pytest.mark.asyncio
async def test_mark_once_waypoint_writes_a_keypoint(rt):
    """F06, the 'record this spot as the east gate' path."""
    ack = await rt.handle(
        _cmd("mark_once", mark_once={"kind": "waypoint", "name": "east gate",
                                     "capture_heading": True}),
        now_mono_s=100.0, now_ms=5000)
    assert ack["result"] == "accepted", ack
    cur = await rt._geo_conn.execute(
        "SELECT geo_id, name, rtk_lat, yaw_deg FROM waypoints")
    row = await cur.fetchone()
    assert row[0].startswith("w-") and row[1] == "east gate"
    assert abs(row[2] - _LAT) < 1e-9 and row[3] == 0.0


@pytest.mark.asyncio
async def test_mark_once_refuses_below_rtk_fixed(rt):
    """S12A.8: a keypoint recorded at float quality is a keypoint the robot
    will drive to and miss. MUTATION: accept any fix and F06 stores a
    decimetre-wrong gate position that looks exactly like a good one."""
    rt.update_pose({"lat": _LAT, "lon": _LON, "fix_type": "rtk_float",
                    "speed_mps": 0.0}, 100.0)
    ack = await rt.handle(
        _cmd("mark_once", mark_once={"kind": "waypoint", "name": "gate"}),
        now_mono_s=100.0, now_ms=1)
    assert ack["result"] == "rejected" and ack["code"] == E_TEACH_QUALITY


@pytest.mark.asyncio
async def test_mark_once_dock_is_refused_as_unwired_not_faked(rt):
    """F10 needs a handover-point model the charging subsystem does not have in
    this build. It says so rather than storing a dock whose handover pose was
    invented. MUTATION: write the dock anyway with a guessed handover and the
    robot drives at a charging contact from the wrong side."""
    ack = await rt.handle(
        _cmd("mark_once", mark_once={"kind": "dock", "name": "dock 1",
                                     "capture_heading": True}),
        now_mono_s=101.5, now_ms=1)
    assert ack["result"] == "rejected"
    assert "not wired" in ack["detail"]["reason"]
    cur = await rt._geo_conn.execute("SELECT COUNT(*) FROM docks")
    assert (await cur.fetchone())[0] == 0


@pytest.mark.asyncio
async def test_dock_capture_requires_stillness_before_anything_else(rt):
    """S12A.8: |vx| < 0.1 m/s sustained for a second. MUTATION: skip the
    stillness tracker and a dock is captured while the robot is still rolling,
    which is precisely what the requirement exists to prevent."""
    rt.update_pose({"lat": _LAT, "lon": _LON, "fix_type": "rtk_fixed",
                    "heading_rad": 0.0, "heading_valid": True,
                    "heading_level": 1, "speed_mps": 0.9}, 100.0)
    ack = await rt.handle(
        _cmd("mark_once", mark_once={"kind": "dock", "name": "dock 1",
                                     "capture_heading": True}),
        now_mono_s=100.2, now_ms=1)
    assert ack["result"] == "rejected" and ack["code"] == E_TEACH_QUALITY
    assert "stationary" in ack["detail"]["reason"]


# ------------------------------------------------------------ teachstate ----

@pytest.mark.asyncio
async def test_idle_teach_state_carries_only_the_state(rt):
    """S12A.5: an idle payload full of null stats reads like a session that
    lost its data."""
    assert rt.teach_state_payload(100.0)["session"] == {"state": "idle"}


@pytest.mark.asyncio
async def test_teach_state_reports_progress(rt):
    sid = await _start(rt)
    await _drive(rt, sid, metres=2.0, t0=101.0, count=3)
    st = rt.teach_state_payload(104.0)
    assert st["session"]["session_id"] == sid
    assert st["session"]["state"] == "recording"
    assert st["stats"]["point_count"] == 3
    assert st["stats"]["length_m"] > 3.0
    assert st["stats"]["rns_intervened"] is False
    assert st["control"]["driver"] == "gamepad"
    assert st["session"]["elapsed_s"] == 4.0


@pytest.mark.asyncio
async def test_session_expires_at_its_deadline(rt):
    """The deadline is monotonic (S12A.5 B5): a wall-clock step at RTK first
    lock -- which happens on every cold boot, often mid-recording -- must not
    end the session early or extend it."""
    sid = await _start(rt)
    await _drive(rt, sid, metres=2.0, t0=101.0, count=2)
    assert rt.expire(500.0) is None            # well inside 1800 s
    assert rt.expire(100.0 + 1800.0) == "max_duration"
    assert rt.teach_state_payload(2000.0)["session"]["state"] == "finalizing"
