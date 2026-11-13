"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u191_fence_geometry.py
Brief: 99 U191 -- a fence ring written through cmd/geo is checked like a recorded one (FV-1/2/5)

Description:
Measured 2026-11-13 on the robot: cmd/geo accepted a figure-of-eight, a
zero-area, a 1 cm and a duplicated-vertex fence, and the figure-of-eight could
be switched on and reached P1 -- while update_cloud S6.4A told the cloud such
a fence is refused. User ruling the same day: refuse them, with a reason code
each; the minimum area is 1 m^2, in p3_task.yaml fence.min_area_m2, for every
role and for recordings too. Each case names the mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import math  # metres -> degrees

import aiosqlite  # in-memory dbs
import pytest  # markers, parametrize

from xbrain.common.errors import E_FENCE_INVALID  # expected code
from xbrain.p3_task.ingest.geo_apply import (  # cmd/geo path
    GeoContext,  # geo_apply context
    handle_geo_payload,  # the cmd/geo entry
)
from xbrain.p3_task.persistence.schema_geo import (  # DDL
    FENCE_DB_STATEMENTS,  # fence.db
    GEO_DB_STATEMENTS,  # geo.db
)
from xbrain.p3_task.teach.validate import ring_geometry_refusal  # under test

pytestmark = pytest.mark.no_device  # pure logic + in-memory sqlite

_LAT, _LON = 31.0, 121.0  # a site
_D = 1.0 / 111320.0  # one metre of latitude
_E = _D / math.cos(math.radians(_LAT))  # one metre of longitude


def _ring(*xy):  # [(lat, lon)] from metres east / north of the site
    return [(_LAT + y * _D, _LON + x * _E) for x, y in xy]  # one vertex each


_SQUARE_2M = _ring((0, 0), (2, 0), (2, 2), (0, 2))  # 4 m^2: legal
_BOWTIE = _ring((0, 0), (10, 10), (0, 10), (10, 0))  # edges 0 and 2 cross
_LINE = _ring((0, 0), (0, 5), (0, 10))  # three points on a line: area 0
_TINY = _ring((0, 0), (0.5, 0), (0.5, 0.5), (0, 0.5))  # 0.25 m^2
_DUP = _ring((0, 0), (10, 0), (10, 0.01), (0, 10))  # vertices 1 and 2 are 1 cm apart


@pytest.mark.parametrize("ring, reason", [
    (_SQUARE_2M[:2], "too_few_vertices"),  # FV-1
    (_DUP, "duplicate_vertex"),  # FV-2 duplicate
    (_BOWTIE, "self_intersect"),  # FV-2 crossing
    (_LINE, "area_too_small"),  # FV-5 zero area
    (_TINY, "area_too_small"),  # FV-5 below 1 m^2
])
def test_each_rule_has_its_reason(ring, reason):
    """MUTATION: any one rule removed -> that case passes or is named by a
    later rule -> red; the duplicate threshold dropped -> _DUP is self_intersect
    or accepted -> red."""
    got = ring_geometry_refusal(ring, 1.0)  # the user's 1 m^2
    assert got is not None and got[0] == reason  # the root cause named


def test_a_legal_ring_passes_and_the_details_carry_the_numbers():
    """A 2 m square is fine at 1 m^2. MUTATION: area compared in degrees ->
    every ring is "too small" -> red; the edge pair or the area omitted -> red."""
    assert ring_geometry_refusal(_SQUARE_2M, 1.0) is None  # legal
    assert ring_geometry_refusal(_BOWTIE, 1.0)[1] == {"edges": [0, 2]}  # which edges
    small = ring_geometry_refusal(_TINY, 1.0)[1]  # the area detail
    assert small["min_area_m2"] == 1.0 and 0.2 < small["area_m2"] < 0.3  # square metres
    assert ring_geometry_refusal(_TINY, None) is None  # not configured: area rule off only
    assert ring_geometry_refusal(_BOWTIE, None)[0] == "self_intersect"  # the others still run


async def _open(statements):  # in-memory db with a schema
    conn = await aiosqlite.connect(":memory:")  # fresh
    for stmt in statements:  # schema
        await conn.execute(stmt)  # one DDL
    await conn.commit()  # ready
    return conn  # caller closes


def _upsert(ring, n):  # one cloud cmd/geo fence upsert (forbid)
    return {"cmd_id": "c-%d" % n, "action": "upsert", "origin": "cloud", "type": "fence",
            "geo_id": "f-t%d" % n, "base_rev": 0,
            "obj": {"name": "t%d禁止区域" % n,
                    "geom": {"role": "forbid", "outer": [{"lat": a, "lon": b} for a, b in ring]}}}


@pytest.mark.asyncio
@pytest.mark.parametrize("ring, reason", [
    (_BOWTIE, "self_intersect"), (_LINE, "area_too_small"),
    (_TINY, "area_too_small"), (_DUP, "duplicate_vertex"), (_SQUARE_2M[:2], "too_few_vertices"),
])
async def test_cmd_geo_refuses_a_bad_ring_and_stores_nothing(ring, reason):
    """The measured hole. MUTATION: the geo_write check removed -> accepted,
    one row -> red; refused with E_GEO_INVALID instead of FV's E_FENCE_INVALID
    -> red."""
    geo, fence = await _open(GEO_DB_STATEMENTS), await _open(FENCE_DB_STATEMENTS)  # dbs
    try:  # dbs closed below
        ctx = GeoContext(geo_conn=geo, fence_conn=fence, min_area_m2=1.0)  # configured
        ack = await handle_geo_payload(_upsert(ring, 1), ctx, now_ms=1)  # the write
        assert ack["result"] == "rejected" and ack["code"] == E_FENCE_INVALID  # FV-1/2/5
        assert ack["detail"]["reason"] == reason  # named
        cur = await fence.execute("SELECT COUNT(*) FROM fences")  # nothing stored
        assert (await cur.fetchone())[0] == 0  # none
        ok = await handle_geo_payload(  # a legal one right after
            _upsert(_SQUARE_2M, 2), ctx, now_ms=2)
        assert ok["result"] == "accepted"  # the check does not refuse everything
    finally:  # always
        await geo.close()  # cleanup
        await fence.close()  # cleanup


def test_p3_refuses_to_start_without_the_minimum_area(monkeypatch):
    """fence.min_area_m2 is required and reaches the wiring. MUTATION: the
    __main__ check removed -> starts with None -> the area rule silently off
    -> red; not passed to the wiring -> red."""
    import xbrain.common.config.resolved as resolved  # where main imports it from
    import xbrain.p3_task.runtime.main_wiring as mw  # the wiring main calls
    from xbrain.p3_task import __main__ as p3main  # unit under test

    def cfg(area):  # the minimal resolved p3 config with one area value
        return {  # every other section as shipped (test_recorded_route_radius)"geo": {"enu_origin": {"lat": 31.0, "lon": 121.0, "alt": 0.0}},
                "teach": {"snap_max_age_ms": 8000, "min_dist_m": 0.5, "sample_hz": 1.0,  # U132 / U162 / U163
                          "record_duration_s": 10800, "record_warn_before_s": [600, 60]},
                "route": {"recorded_arrive_radius_m": 1.0, "membership_m": 5.0},  # U126 / U127
                "rtk_loss": {"lost_sustain_s": 1.0, "recover_sustain_s": 2.0},  # U142
                "fence": dict({"speed_limit_default_mps": 0.6, "speed_limit_min_mps": 0.1,
                               "speed_limit_max_mps": 2.0},
                              **({} if area is None else {"min_area_m2": area})),
                "low_battery": {"trigger_soc_pct": 20.0, "stop_soc_pct": 5.0,  # U146 / U157
                                "cloud_alarm_period_s": 2.0, "charge_full_soc_pct": 100.0,
                                "swap_resume_soc_pct": 80.0, "swap_absent_min_s": 5.0,
                                "cold_swap_min_rise_pct": 10.0, "plug_confirm_s": 5.0,
                                "unplug_confirm_s": 5.0}}
    seen = {}  # what the wiring received
    monkeypatch.setattr(mw, "run_voice_loop_wiring",  # do not run the loop
                        lambda **kw: seen.update(kw) or 0)  # just record the kwargs
    for bad in (None, 0.0, -1.0, True):  # missing / not positive / not a number
        bad_cfg = cfg(bad)  # this round's config
        monkeypatch.setattr(resolved, "load_resolved",
                            lambda *_a, _c=bad_cfg, **_k: _c)  # bound per round
        assert p3main.main(["--voice-loop"]) == 78  # EX_CONFIG
    monkeypatch.setattr(resolved, "load_resolved", lambda *_a, **_k: cfg(1.0))  # shipped value
    assert p3main.main(["--voice-loop"]) == 0 and seen["fence_min_area_m2"] == 1.0


@pytest.mark.asyncio
async def test_a_recording_uses_the_configured_minimum_area():
    """TeachRuntime passes fence.min_area_m2 to the recording check (was the
    code constant 100 m^2). MUTATION: runtime passes nothing -> the area rule
    is off -> the fence saves -> red."""
    from tests.p3_task.test_u184_fence_layout import (  # the U184 recording helpers
        _LAT as _SITE_LAT,  # the U184 site, where _PIT_SIDE lies
    )
    from tests.p3_task.test_u184_fence_layout import (
        _LON as _SITE_LON,
    )
    from tests.p3_task.test_u184_fence_layout import (
        _PIT_SIDE,  # a ring of a few hundred m^2
        _pose,  # one rtk_fixed pose sample
        _record,  # start, drive, finish = save
    )
    from tests.p3_task.test_u184_fence_layout import (
        _open as _open_db,  # in-memory db with a schema
    )
    from xbrain.common.errors import E_TEACH_GEOMETRY  # recording geometry refusal
    from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # task DDL
    from xbrain.p3_task.teach.runtime import TeachRuntime  # under test
    dbs = (await _open_db(ALL_DDL_STATEMENTS), await _open_db(GEO_DB_STATEMENTS),
           await _open_db(FENCE_DB_STATEMENTS))  # task, geo, fence
    try:
        rt = TeachRuntime(*dbs, boot_id="b0", sample_hz=1.0, min_dist_m=0.5,  # U132 pair
                          snap_max_age_ms=8000, record_duration_s=10800,  # configured values
                          fence_speed_limit={"default": 0.6, "min": 0.1, "max": 2.0},  # U185
                          fence_min_area_m2=1.0e6)  # a floor no real ring reaches
        _pose(rt, _SITE_LAT, _SITE_LON, 100.0)  # a fresh fix at that site
        rt.update_health({"allow_motion": True})  # arming
        rt.update_robot({"hes": False, "estop_path": "up"})  # arming
        rt.update_power({"soc_pct": 80.0})  # arming
        rt.update_teleop({"sources": [{"device": "gamepad", "stale": False, "alive": True}]})
        fin = await _record(rt, "fence", "北门禁止区域围栏", _PIT_SIDE)  # record and save
        assert (fin["result"], fin["code"]) == ("rejected", E_TEACH_GEOMETRY)  # not saved
        assert any(i["code"] == "area_too_small" for i in fin["detail"]["issues"])  # why
    finally:  # always
        for c in dbs:  # cleanup
            await c.close()  # close
