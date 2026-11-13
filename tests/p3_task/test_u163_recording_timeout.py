"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u163_recording_timeout.py
Brief: 99 U163 -- recording time-left warnings, auto-finish reports, paused expiry, no route point cap

Description:
User rulings 2026-11-11: the 180 min recording limit is the only limit; at the
limit the recording is finished and saved automatically AND said out loud and
reported to the cloud; navigation (PTP and route) carries no point, length or
lap limit. Pins, against the real TeachRuntime and the real P3 loop:
  * time-left warnings (teach.record_warn_before_s) fire once each, largest
    first, while recording or paused, never at or past the deadline;
  * a paused session expires too (it used to wait for a resume);
  * an auto-save whose write fails keeps the session (finalizing, buffer
    intact) and says why, instead of escaping to the loop's generic handler;
  * the loop raises event/{sev}/teach for the warnings and the auto-finish,
    marked first so P4 speaks them;
  * the task.db snapshot table loses its 5000-point CHECK on an existing
    database, rows kept; a cmd/geo route of any size is accepted.
Each case names the mutant it kills.
"""
from __future__ import annotations

import asyncio
import json

import aiosqlite
import pytest
import pytest_asyncio

from tests.p3_task.test_teach_named import _LAT, _LON, _cmd, _drive, _open, _pose
from xbrain.common.errors import E_NAME_CONFLICT
from xbrain.p3_task.persistence.schema_geo import FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS
from xbrain.p3_task.persistence.schema_task import (
    ALL_DDL_STATEMENTS,
    ensure_task_route_snapshot_uncapped,
)
from xbrain.p3_task.teach.runtime import TeachRuntime

pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def rt():
    """A runtime with healthy caches, a 1000 s limit and warnings at 600 / 60 s."""
    task, geo, fence = (await _open(ALL_DDL_STATEMENTS), await _open(GEO_DB_STATEMENTS),
                        await _open(FENCE_DB_STATEMENTS))
    runtime = TeachRuntime(task, geo, fence, boot_id="b0", sample_hz=1.0, min_dist_m=0.5,
                           snap_max_age_ms=8000, record_duration_s=1000.0,
                           record_warn_before_s=[60.0, 600.0])  # given unordered on purpose
    _pose(runtime, _LAT, _LON, 100.0)
    runtime.update_health({"allow_motion": True})
    runtime.update_robot({"hes": False, "estop_path": "up"})
    runtime.update_power({"soc_pct": 80.0})
    runtime.update_teleop({"sources": [{"device": "gamepad", "stale": False, "alive": True}]})
    yield runtime
    for c in (task, geo, fence):
        await c.close()


async def _start(rt, name, t=100.0):
    _pose(rt, _LAT, _LON, t)
    ack = await rt.handle(_cmd("c-start-" + name, "start", start={"kind": "route", "name": name}),
                          now_mono_s=t, now_ms=1000)
    assert ack["result"] == "accepted", ack
    return ack


@pytest.mark.asyncio
async def test_time_left_warnings_fire_once_each_largest_first(rt):
    """MUTATION: no once-latch -> 600 fires every pass -> red; ascending order
    -> 60 before 600 on a joint pass -> red; fire at/after the deadline -> red."""
    await _start(rt, "1号路径")
    assert rt.due_warnings(100.0 + 300.0) == []  # 700 s left: nothing yet
    assert [w for w, _ in rt.due_warnings(100.0 + 400.0)] == [600.0]  # 600 s left
    assert rt.due_warnings(100.0 + 401.0) == []  # once
    assert [w for w, _ in rt.due_warnings(100.0 + 940.0)] == [60.0]  # 60 s left
    assert rt.due_warnings(100.0 + 1000.0) == []  # at the deadline: the auto-finish speaks


@pytest.mark.asyncio
async def test_no_warning_at_or_past_the_deadline(rt):
    """A loop that stalls past the deadline gives no warning at all: the
    auto-finish says it. MUTATION: drop the remaining > 0 guard -> 600 and 60
    fire at the deadline -> red."""
    await _start(rt, "5号路径")
    assert rt.due_warnings(100.0 + 1000.0) == []
    assert rt.due_warnings(100.0 + 1500.0) == []


@pytest.mark.asyncio
async def test_both_warnings_due_at_once_come_largest_first(rt):
    """A pass that lands inside both windows (a long stall) gives both, 10 min first."""
    await _start(rt, "2号路径")
    assert [w for w, _ in rt.due_warnings(100.0 + 950.0)] == [600.0, 60.0]


@pytest.mark.asyncio
async def test_a_paused_session_expires_and_is_saved(rt):
    """Until 2026-11-11 only `recording` expired: a recording paused past the
    limit never ended. MUTATION: expire on recording only -> None -> red."""
    sid = (await _start(rt, "3号路径"))["detail"]["session_id"]
    await _drive(rt, 2.0, 3)
    ack = await rt.handle(_cmd("c-p", "pause", session_id=sid), now_mono_s=200.0, now_ms=2000)
    assert ack["result"] == "accepted", ack
    assert rt.due_warnings(100.0 + 450.0)  # warnings run while paused too
    assert rt.expire(100.0 + 1000.0) == "max_duration"
    detail = await rt.save_named(3000)
    assert detail["saved"] is True


@pytest.mark.asyncio
async def test_an_auto_save_that_cannot_write_keeps_the_session(rt):
    """The name was taken while recording: the write fails. The session stays in
    finalizing with its points so the operator can save it under another name;
    the reason is returned, never raised. MUTATION: let the exception escape
    -> raises -> red; discard on failure -> state closed -> red."""
    sid = (await _start(rt, "4号路径"))["detail"]["session_id"]
    await _drive(rt, 2.0, 3)
    await rt._geo_conn.execute(  # someone else took the name meanwhile
        "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction, total_len_m,"
        " rev, content_hash, updated_ms) VALUES ('r-other', '4号路径', ?, 'oneway',"
        " 'forward', 10.0, 1, 'h', 0)", (json.dumps([[_LAT, _LON], [_LAT + 1e-4, _LON]]),))
    await rt._geo_conn.commit()
    assert rt.expire(100.0 + 1000.0) == "max_duration"
    detail = await rt.save_named(3000)
    assert detail["saved"] is False and detail["retained"] is True
    assert detail["code"] == E_NAME_CONFLICT and detail["state"] == "finalizing"
    ack = await rt.handle(_cmd("c-save", "save", session_id=sid, save={"name": "4号路径B"}),
                          now_mono_s=1200.0, now_ms=4000)
    assert ack["result"] == "accepted", ack  # the points were kept


@pytest.mark.asyncio
async def test_the_snapshot_table_loses_its_point_cap_rows_kept(tmp_path):
    """An existing task.db carries CHECK(point_count <= 5000); SQLite cannot drop
    a CHECK in place, so the table is rebuilt, rows copied. MUTATION: skip the
    rebuild -> the 6000-point insert fails -> red; drop the rows -> red."""
    db = str(tmp_path / "task.db")
    old = [s.replace("CHECK (point_count >= 1),", "CHECK (point_count >= 1 AND point_count <= 5000),")
           for s in ALL_DDL_STATEMENTS]
    async with aiosqlite.connect(db) as c:
        for stmt in old:
            await c.execute(stmt)
        await c.execute("INSERT INTO task_route_snapshot VALUES ('t-1','r-1',1,'oneway',2,10.0,"
                        "'[]','[]','2026-11-11')")
        await c.commit()
        assert await ensure_task_route_snapshot_uncapped(c) is True
        assert await ensure_task_route_snapshot_uncapped(c) is False  # idempotent
        await c.execute("INSERT INTO task_route_snapshot VALUES ('t-2','r-2',1,'oneway',6000,10.0,"
                        "'[]','[]','2026-11-11')")
        cur = await c.execute("SELECT task_id FROM task_route_snapshot ORDER BY task_id")
        assert [r[0] for r in await cur.fetchall()] == ["t-1", "t-2"]


def test_a_cmd_geo_route_of_any_size_is_accepted():
    """~~11 S7.8.3 <= 5000 vertices (E_GEO_TOO_LARGE)~~ removed. MUTATION: put
    the cap back -> GeoCommandError -> red."""
    from xbrain.p3_task.ingest.geo_object import _route
    pts = [{"lat": _LAT + i * 1e-5, "lon": _LON} for i in range(6000)]
    parsed = _route({"name": "长路径", "geom": {"points": pts}})
    assert len(json.loads(parsed.columns["path_points"])) == 6000


@pytest.mark.asyncio
async def test_the_loop_reports_warnings_and_the_auto_save(tmp_path, monkeypatch):
    """The real P3 loop with a 3 s limit and warnings at 2 s / 1 s: two
    teach.time_warning events, then teach.expired with result saved, all on
    event/*/teach (11 S12A.12) and marked first. MUTATION: drop the warning emit -> no
    warnings -> red; drop the auto-finish emit -> no saved event -> red."""
    from tests.p3_task.test_recorded_route_radius import _fake_planes
    from xbrain.p3_task.runtime import main_wiring as mw

    subs, pubs = {}, {}
    _fake_planes(monkeypatch, subs, pubs)

    def _sample(body):
        return type("S", (), {"payload": json.dumps(body).encode("utf-8")})()

    def _pose_body(i):
        return {"lat": _LAT + i * 1.5e-5, "lon": _LON, "fix_type": "rtk_fixed",
                "heading_rad": 0.0, "heading_valid": True, "heading_level": 1,
                "speed_mps": 1.0}

    stop = {"stop": False}
    task = asyncio.create_task(mw._amain(
        stop, 99.0, str(tmp_path / "task.db"), geo_db_path=str(tmp_path / "geo.db"),
        fence_db_path=str(tmp_path / "fence.db"), recorded_arrive_radius_m=1.0,
        teach_sample_hz=4.0, teach_min_dist_m=0.5, teach_record_duration_s=3.0,
        teach_record_warn_before_s=[2.0, 1.0]))
    try:
        for _ in range(300):
            if mw.CMD_TEACH_TOPIC in subs:
                break
            await asyncio.sleep(0.01)
        for key, body in ((mw.STATE_POSE_TOPIC, _pose_body(0)),
                          (mw.HEALTH_SUMMARY_TOPIC, {"allow_motion": True}),
                          (mw.STATE_ROBOT_TOPIC, {"hes": False, "estop_path": "up"}),
                          (mw.STATE_POWER_TOPIC, {"soc_pct": 80.0}),
                          (mw.STATE_TELEOP_TOPIC, {"sources": [{"device": "gamepad",
                                                                "stale": False, "alive": True}]})):
            subs[key](_sample({"data": body}))
        await asyncio.sleep(0.1)
        subs[mw.CMD_TEACH_TOPIC](_sample(_cmd("c-live", "start",
                                              start={"kind": "route", "name": "超时路径"})))
        for i in range(1, 22):  # walk north for ~4.2 s, past the 3 s limit
            subs[mw.STATE_POSE_TOPIC](_sample({"data": _pose_body(i)}))
            await asyncio.sleep(0.2)
        events = [json.loads(p) for k, ps in pubs.items() if k.endswith("/teach")
                  and k.startswith("event/") for p in ps]
        kinds = [e["detail"]["kind"] for e in events]
        assert kinds[:2] == ["teach.time_warning", "teach.time_warning"], kinds
        assert kinds[2] == "teach.expired" and events[2]["detail"]["result"] == "saved", kinds
        assert [e["sev"] for e in events[:3]] == ["info", "info", "warn"]
        assert all(e["detail"]["first"] is True and e["cat"] == "teach" for e in events)
        saved = events[2]["detail"]
        assert saved["name"] == "超时路径" and saved["saved"] is True and saved["geo_id"]
        assert saved["session_kind"] == "route"
        assert saved["channel"] == "local_voice"
    finally:
        stop["stop"] = True
        await asyncio.wait_for(task, timeout=10.0)


@pytest.mark.parametrize("warn,rc", [
    ([600, 60], 0), (None, 78), ([], 78), ([600, 0], 78), ([10800], 78),
    ([float("nan")], 78), ([True], 78), ("600", 78)])
def test_main_refuses_a_bad_warning_list(monkeypatch, warn, rc):
    """teach.record_warn_before_s is required like every other key: missing,
    empty, non-positive, not below the limit (it would fire at the start),
    NaN or bool -> EX_CONFIG 78; a good list reaches the wiring as floats.
    MUTATION: drop the check -> the bad rows start -> red."""
    import xbrain.common.config.resolved as resolved  # where main loads from
    import xbrain.p3_task.runtime.main_wiring as mw  # the wiring main calls
    from tests.p3_task.test_low_battery import _FULL_LB
    from xbrain.p3_task import __main__ as p3main  # unit under test
    cfg = {"geo": {"enu_origin": {"lat": 31.0, "lon": 121.0, "alt": 0.0}},  # minimal resolved config
           "teach": {"snap_max_age_ms": 8000, "min_dist_m": 0.5, "sample_hz": 1.0,
                     "record_duration_s": 10800, "record_warn_before_s": warn},
           "route": {"recorded_arrive_radius_m": 1.0, "membership_m": 5.0},
           "fence": {"speed_limit_default_mps": 0.6, "speed_limit_min_mps": 0.1, "speed_limit_max_mps": 2.0,
                     "min_area_m2": 1.0},  # 99 U185 / U191
           "rtk_loss": {"lost_sustain_s": 1.0, "recover_sustain_s": 2.0},
           "low_battery": dict(_FULL_LB)}
    monkeypatch.setattr(resolved, "load_resolved", lambda *_a, **_k: cfg)  # no file
    seen = {}  # what the wiring received
    monkeypatch.setattr(mw, "run_voice_loop_wiring", lambda **kw: seen.update(kw) or 0)
    assert p3main.main(["--voice-loop"]) == rc  # the exit code
    if rc == 0:
        assert seen["teach_record_warn_before_s"] == [600.0, 60.0]
