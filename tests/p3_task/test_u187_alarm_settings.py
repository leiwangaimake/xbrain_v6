"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u187_alarm_settings.py
Brief: 99 U187 -- the warning zones' alarm settings stored by P3 (window, targets, siren)

Description:
The user's ruling of 2026-11-13 (99 U187): the time window and the suspicious
targets of the autonomous alarm are sent by the cloud, persisted on the robot,
overridable by a newer value and deletable; not sent = the default (all day,
people in every warning zone). P3 stores them in fence.db fence_settings via
cmd/geo set_fence_defaults and broadcasts the effective view (alarm_view) to P2
on cmd/fence. Each case names the mutant it kills:
  * the default view when nothing was sent;
  * the rule types map to the target classes (人 / 车 / 动物), a disabled rule
    watches nothing, an empty region list means every warning zone;
  * a partial update leaves absent keys alone, null deletes one key only;
  * one bad key refuses the whole command and nothing is stored;
  * an exclusion rule is refused, never stored as an inclusion;
  * an alarm-only change is not in force for P1 (P5 does not wait for a rev).
"""
from __future__ import annotations  # postponed annotations

import aiosqlite  # in-memory db
import pytest  # marker, parametrize
import pytest_asyncio  # async fixture

from xbrain.common.errors import E_CHANNEL_DENIED, E_GEO_INVALID  # expected codes
from xbrain.p3_task.fence.settings import alarm_view, read_settings  # under test
from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload  # cmd/geo path
from xbrain.p3_task.persistence.schema_geo import FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS  # DDL

pytestmark = pytest.mark.no_device  # pure logic + in-memory sqlite

_SPEED = {"default": 0.6, "min": 0.1, "max": 2.0}  # p3_task.yaml fence.speed_limit_*_mps
_N = [0]  # cmd_id counter: P3 replays a seen cmd_id as a duplicate


async def _open(statements):  # in-memory db with a schema
    conn = await aiosqlite.connect(":memory:")  # fresh
    for stmt in statements:  # schema
        await conn.execute(stmt)  # one DDL
    await conn.commit()  # ready
    return conn  # caller closes


@pytest_asyncio.fixture  # the cmd/geo context as p3 builds it
async def ctx():
    geo, fence = await _open(GEO_DB_STATEMENTS), await _open(FENCE_DB_STATEMENTS)  # dbs
    yield GeoContext(geo_conn=geo, fence_conn=fence, task_conn=None, speed_limit=_SPEED)  # U185
    await geo.close()  # cleanup
    await fence.close()  # cleanup


async def _set(ctx, origin="cloud", **obj):  # one set_fence_defaults command
    _N[0] += 1  # unique cmd_id
    return await handle_geo_payload({"cmd_id": "a-%d" % _N[0], "action": "set_fence_defaults",
                                     "origin": origin, "obj": obj}, ctx, now_ms=_N[0])


_WINDOW = {"start": "22:00", "end": "05:00"}  # crosses midnight: legal (user ruling)


@pytest.mark.asyncio  # async case
async def test_nothing_sent_is_all_day_and_people_everywhere(ctx):
    """The user's default. MUTATION: default rules empty -> P2 watches
    nothing on a fresh robot -> red; source not "default" -> the HMI cannot
    tell a default from a cloud value -> red."""
    view = await alarm_view(ctx.fence_conn)  # nothing stored
    assert view["window"] is None  # all day
    assert view["rules"] == [{"targets": ["person"], "region_ids": None}]  # people, every zone
    assert view["rules_source"] == "default"  # labelled
    assert view["siren_level"] is None and view["cooldown_sec"] is None  # P2's own defaults


@pytest.mark.asyncio  # async case
async def test_rules_map_to_target_classes(ctx):
    """人 / 车 / 动物 from the rule types; a disabled rule is dropped; []
    region ids = every warning zone. MUTATION: RULE_TARGET swapped (vehicle ->
    person) -> red; disabled rules kept -> red; [] kept as [] (= no zone) -> red."""
    rules = [{"type": "person_in_region", "region_ids": ["f-a"]},  # people in one zone
             {"type": "vehicle_in_region", "region_ids": []},  # cars anywhere
             {"type": "animal_in_region", "enabled": False, "region_ids": ["f-b"]}]  # switched off
    ack = await _set(ctx, alarm_window=_WINDOW, alarm_rules=rules)  # the cloud sends both
    assert ack["result"] == "accepted"  # stored
    view = await alarm_view(ctx.fence_conn)  # what P2 receives
    assert view["window"] == _WINDOW and view["rules_source"] == "cloud"  # as sent
    assert view["rules"] == [{"targets": ["person"], "region_ids": ["f-a"]},
                             {"targets": ["vehicle"], "region_ids": None}]  # the animal rule is off
    animal = await _set(ctx, alarm_rules=[{"type": "animal_in_region"}])  # the new full list
    assert animal["result"] == "accepted"  # overrides the old list
    assert (await alarm_view(ctx.fence_conn))["rules"] == [
        {"targets": ["animal"], "region_ids": None}]  # replaced, not merged


@pytest.mark.asyncio  # async case
async def test_partial_update_and_delete_one_key(ctx):
    """Absent = unchanged, null = deleted (back to the default). MUTATION:
    every ALARM_KEY written (absent as None) -> the window disappears when
    only the siren changes -> red; null ignored -> the window stays -> red."""
    await _set(ctx, alarm_window=_WINDOW, siren_level=70)  # first configuration
    await _set(ctx, siren_level=40)  # only the siren
    stored = await read_settings(ctx.fence_conn)  # rows
    assert stored == {"alarm_window": _WINDOW, "siren_level": 40}  # window kept
    gone = await _set(ctx, alarm_window=None)  # the cloud deletes the window
    assert gone["detail"]["alarm"] == {"alarm_window": None}  # said back
    assert await read_settings(ctx.fence_conn) == {"siren_level": 40}  # only that key went
    assert (await alarm_view(ctx.fence_conn))["window"] is None  # all day again


@pytest.mark.asyncio  # async case
@pytest.mark.parametrize("bad", [
    {"alarm_window": {"start": "25:00", "end": "05:00"}},  # not a time of day
    {"alarm_window": "22:00-05:00"},  # not the object form
    {"alarm_rules": [{"type": "bird_in_region"}]},  # birds are not reported (user ruling)
    {"alarm_rules": [{"type": "person_in_region", "alarm_role": "exclude"}]},  # not built
    {"siren_level": 101},  # out of range
    {"alarm_level": True},  # a bool is not 1
    {"cooldown_sec": 0.1},  # below the bound
])
async def test_a_bad_value_refuses_the_whole_command(ctx, bad):
    """Checked before anything is written. MUTATION: the validator skipped
    -> garbage stored -> red; writes before the check -> duration_sec stored
    although the command was refused -> red."""
    ack = await _set(ctx, **dict({"duration_sec": 5}, **bad))  # one good key, one bad
    assert ack["result"] == "rejected" and ack["code"] == E_GEO_INVALID  # refused
    assert ack["detail"]["reason"] == "alarm_setting_invalid"  # names the cause
    assert await read_settings(ctx.fence_conn) == {}  # nothing stored, not even the good key


@pytest.mark.asyncio  # async case
async def test_only_the_cloud_sets_alarm_settings(ctx):
    """set_fence_defaults is cloud-only (99 U185), the alarm keys included.
    MUTATION: voice allowed -> stored -> red."""
    ack = await _set(ctx, origin="voice", alarm_window=_WINDOW)  # a voice attempt
    assert ack["result"] == "rejected" and ack["code"] == E_CHANNEL_DENIED  # refused
    assert await read_settings(ctx.fence_conn) == {}  # nothing stored


@pytest.mark.asyncio  # async case
async def test_an_alarm_only_change_is_not_in_force_for_p1(ctx):
    """P1 enforces polygons and limits, not the alarm settings, so P5 must
    not wait for a state/fence rev that never comes. MUTATION: in_force True
    for alarm keys -> P5 times the configuration out after 6 s -> red."""
    ack = await _set(ctx, alarm_rules=[{"type": "person_in_region"}])  # alarm only
    assert ack["result"] == "accepted" and ack["detail"]["in_force"] is False  # done at once
    assert "speed_limit_default_mps" not in ack["detail"]  # the speed default untouched
