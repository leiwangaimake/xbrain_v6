"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_low_battery.py
Brief: 99 U146/U157 low-battery return -- episode phases, held tasks, dock / home choice, the injected task, 5 % stop, cloud alarms, dock-on-route

Description:
The user's low-battery rule (2026-11-05 ruling 3, restated 2026-11-06):
below 20 % one voice alarm and a cloud alarm every 2 s; on the charger the
alarm goes on worded "charging" until 100 %, a swap ends it at 80 % (99 U154
/ U157), and the tasks the episode interrupted or held resume only then;
drive (normal speed) to
the nearest 充电桩 / 充电站 point
that lies on a route, else to 返航终点站; say unreachable when
remain_mile_km cannot cover it; at 5 % stop and lie down; at boot report a
missing home point; a dock off every route is refused when it is recorded.
Each case names the mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import asyncio  # the live loop
import json  # path_points

import aiosqlite  # geo.db
import pytest  # marks, parametrize
import pytest_asyncio  # async fixture

from xbrain.common.errors import E_GEO_INVALID, E_LOW_BATTERY  # refusal codes
from xbrain.p3_task.lifecycle import low_battery as lb  # under test
from xbrain.p3_task.persistence.schema_geo import GEO_DB_STATEMENTS  # geo.db DDL
from xbrain.p3_task.runtime.route_push_runtime import load_route_geoms  # routes as P3 reads them
from tests.p3_task.live_loop import (  # the real loop over fake planes
    goto_row, route_ops, start_p3, stop_p3, task_state, wait_for)

pytestmark = pytest.mark.no_device  # pure logic plus an in-process loop

ROUTE = [[35.0, 135.0], [35.002, 135.0]]  # ~222 m north from the robot (35.0, 135.0)
ROUTE_SQL = ("INSERT INTO routes (geo_id, name, path_points, loop_mode, direction, total_len_m,"  # one mode B route
             " rev, content_hash, updated_ms) VALUES ('r-a', '1号路径', ?, 'oneway', 'forward',"
             " 222.0, 1, 'h', 0)", (json.dumps(ROUTE),))  # a mode B route


def _wp(gid, name, wtype, lat, lon):  # a waypoint row
    """(sql, params) of one waypoint row."""
    return ("INSERT INTO waypoints (geo_id, name, type, rtk_lat, rtk_lon, arrival_radius, rev,"  # columns as P3 stores them
            " content_hash, updated_ms) VALUES (?, ?, ?, ?, ?, 1.0, 1, 'h', 0)",
            (gid, name, wtype, lat, lon))  # as P3 stores it


DOCK_ON = _wp("w-d1", "1号充电站", "dock", 35.0015, 135.00002)  # ~2 m off the route
DOCK_OFF = _wp("w-d2", "2号充电站", "dock", 35.0, 135.01)  # ~900 m off any route, but closer straight
HOME = _wp("w-home", "返航终点站", "home", 35.0005, 135.0)  # on the route


def _power(soc, charging=False, remain_km=5.0, charge="idle"):  # a chassis power frame
    """A state/power body as quadruped publishes it (11 S4.2, 13 BAT-1)."""
    return {"soc_pct": soc, "remain_mile_km": remain_km, "charge": charge,  # 11 S4.2 fields
            "list": [{"index": 0, "level_pct": soc, "charging": charging, "present": True}]}


# -- the episode ----------------------------------------------------------------------
# trigger, stop, period, charge_full, swap_resume, swap_absent_min_s, cold_swap_min_rise_pct,
# plug_confirm_s, unplug_confirm_s (U157). The charging debounce is 0.5 s here (5 s shipped)
# so the unit cases stay short; a charging change takes two frames 0.5 s apart.
RULE = (20.0, 5.0, 2.0, 100.0, 80.0, 5.0, 10.0, 0.5, 0.5)


def _mon(prime=True):  # the shipped thresholds, a short debounce
    """A monitor; primed = the debounced charging flag already known False, as
    in steady running. A restart test uses prime=False: right after a start the
    flag is unknown (ChargingDebouncer.stable is None)."""
    m = lb.LowBatteryMonitor(*RULE)
    if prime:
        m.charging.update(False, -10.0)
        m.charging.update(False, -9.0)
    return m


def _plug(m, body, t):  # a charging change confirmed: two frames 0.5 s apart
    """Feed body at t and t + 0.5; the events of the second frame."""
    m.update(body, t)
    return m.update(body, t + 0.5)


def test_low_starts_once_and_alarms_every_period():  # trigger and cadence
    """MUTATION: no cadence -> an alarm every frame -> red; '<=' 20 -> 20.0
    starts an episode -> red."""
    m = _mon()
    assert m.update(_power(20.0), 0.0) == []  # 20 % is not below 20
    assert m.update(_power(19.0), 1.0) == ["low", "alarm"]  # starts, first alarm
    assert m.update(_power(19.0), 2.0) == []  # 1 s later: not due
    assert m.update(_power(19.0), 3.0) == ["alarm"]  # 2 s after the last
    assert m.update(_power(25.0), 4.0) == []  # soc up without charging: still on


@pytest.mark.parametrize("body", [_power(19.0, charging=True), _power(19.0, charge="charging")])
def test_charging_turns_the_alarm_into_charging_not_an_end(body):  # either charging signal
    """99 U125: either the battery flag or the six-state field starts the
    charging phase. 99 U157: charging no longer ENDS the episode -- the alarm
    goes on (worded by phase) at once and every period after. MUTATION: read
    one of the two signals only -> red for the other; end the episode on
    charging (the U146 rule) -> `low` false -> red."""
    m = _mon()
    m.update(_power(19.0), 0.0)  # an episode
    assert m.update(body, 1.0) == []  # one charging frame: not confirmed yet (CD-1)
    assert m.update(body, 1.5) == ["charging", "alarm"]  # confirmed: phase change, alarm at once
    assert m.low and m.phase == lb.PHASE_CHARGING  # still an episode
    assert m.update(body, 2.0) == []  # not due
    assert m.update(body, 3.5) == ["alarm"]  # the charging alarm keeps its cadence


@pytest.mark.parametrize("soc,ends", [(99.0, False), (100.0, True)])
def test_the_charging_alarm_ends_only_at_charge_full(soc, ends):  # U157 charge_full_soc_pct
    """MUTATION: '>' instead of '>=' -> 100 does not end -> red; end on any
    charging soc -> 99 ends -> red."""
    m = _mon()
    m.update(_power(19.0), 0.0)
    _plug(m, _power(30.0, charging=True), 1.0)  # charging phase
    got = m.update(_power(soc, charging=True), 4.0)
    assert (got == ["charge_full"]) is ends and m.low is not ends


def test_unplugged_before_full_holds_or_drives_back():  # U157 ruling 2
    """Unplugged at or above the trigger -> interrupted (still alarmed);
    drained below it -> low again (a new return task); full while unplugged
    ends it. MUTATION: end on unplug -> red; skip the interrupted -> low
    re-entry -> no second "low" -> red."""
    m = _mon()
    m.update(_power(19.0), 0.0)
    _plug(m, _power(40.0, charging=True), 1.0)  # charging
    assert m.update(_power(60.0), 2.0) == []  # one unplugged frame: not confirmed (CD-1)
    assert m.update(_power(60.0), 2.5) == ["interrupted", "alarm"]  # pulled at 60 %
    assert m.phase == lb.PHASE_INTERRUPTED and m.low
    assert m.update(_power(19.0), 5.0)[:1] == ["low"]  # drained: back to low
    _plug(m, _power(19.0, charging=True), 6.0)  # replugged
    assert _plug(m, _power(15.0), 7.0)[:1] == ["low"]  # pulled again while low: low
    _plug(m, _power(50.0, charging=True), 8.0)
    assert m.update(_power(100.0), 10.0) == ["charge_full"]  # unplugged at full: ends


def test_the_five_percent_latch_rearms_after_charging():  # stop once per low phase
    """The 5 % pause lifts on the charger (U146) and a later low phase may stop
    again. MUTATION: keep `stopped` across the charging phase -> no second
    stop -> red."""
    m = _mon()
    assert "stop" in m.update(_power(4.0), 0.0)
    _plug(m, _power(4.0, charging=True), 1.0)
    assert not m.stopped  # the dispatch pause is lifted on the charger
    assert _plug(m, _power(4.0), 2.0)[:2] == ["low", "stop"]  # unplugged at 4 %


# -- the battery swap (99 U154 / U157) --------------------------------------------------
def _packs(soc, *packs):  # a power frame with explicit slots
    """A state/power body whose `list` has one entry per (present, level) slot;
    soc is soc_pct as quadruped computes it (min over present slots, 13 V-68)."""
    return {"soc_pct": soc, "remain_mile_km": 5.0, "charge": "idle",  # 11 S4.2 fields
            "list": [{"index": i, "level_pct": lvl, "charging": False, "present": pres}
                     for i, (pres, lvl) in enumerate(packs)]}


def test_a_swap_closes_the_episode_once_the_level_is_back():  # pulled, refilled, enough
    """User ruling 2026-11-10: a battery swap is a lawful end. MUTATION: no
    swap rule -> still low -> red; close while the slot is still empty -> red."""
    m = _mon()
    assert m.update(_packs(4.0, (True, 4.0)), 0.0) == ["low", "stop", "alarm"]  # low and lying down
    assert m.update(_packs(None, (False, 0.0)), 1.0) == []  # pack out: no soc, nothing closes
    assert m.update(_packs(90.0, (True, 90.0)), 7.0) == ["swapped"]  # a full pack in after 6 s
    assert not m.low and not m.stopped  # the dispatch pause is lifted with it


def test_soc_back_without_a_swap_does_not_close():  # a wobble is not a swap
    """A small rise is a wobble, not a swap. (Until 2026-11-12 this case fed
    19 -> 60 % and expected the episode to stay open; 99 U171 makes such a
    rise a swap, see the next test.) MUTATION: drop the rise threshold ->
    19 -> 28 closes -> red."""
    m = _mon()
    m.update(_packs(19.0, (True, 19.0)), 0.0)  # an episode
    got = m.update(_packs(28.0, (True, 28.0)), 1.0)  # +9, below cold_swap_min_rise_pct 10
    assert not {"swapped", "swapped_partial"} & set(got) and m.low  # never pulled out


# -- 99 U171 (user ruling 2026-11-12): a large rise without charging is a swap ------
def test_a_hot_swap_seen_only_as_a_jump_closes_the_episode():
    """Measured 2026-11-11 21:40-21:41: 14 -> 100 % in one step, no long enough
    present:false, and the episode alarmed until 22:51. MUTATION: no rise
    rule -> still low -> red."""
    m = _mon()
    assert m.update(_packs(14.0, (True, 14.0)), 0.0) == ["low", "alarm"]
    assert m.update(_packs(100.0, (True, 100.0)), 1.0) == ["swapped"]  # >= swap_resume 80
    assert not m.low and not m.last_cold_swap


def test_a_rise_is_measured_from_the_lowest_soc_of_the_episode():
    """19 -> 12 -> 23: +4 from the start but +11 from the low -> a swap.
    MUTATION: measure from the episode start instead of the minimum -> red."""
    m = _mon()
    m.update(_power(19.0), 0.0)  # episode starts at 19
    assert m.update(_power(12.0), 1.0) == []  # drained further
    assert m.update(_power(23.0), 2.0) == ["swapped_partial"]  # 20 <= 23 < 80


def test_what_a_charger_adds_is_not_a_swap():
    """Charged 15 -> 40 then unplugged: interrupted, not swapped. MUTATION:
    keep the base at the episode low while charging -> 40 - 15 reads as a
    swap -> red."""
    m = _mon()
    m.update(_power(15.0), 0.0)  # low
    assert "charging" in _plug(m, _power(16.0, charging=True), 1.0)
    m.update(_power(40.0, charging=True), 2.0)  # charged up on the dock
    got = _plug(m, _power(40.0), 3.0)  # unplugged at 40
    assert got[0] == "interrupted" and m.low  # the U157 interrupted phase, alarm on
    # the first frame IN the interrupted phase is where a stale base (15) bites
    got = m.update(_power(40.0), 4.0)
    assert not {"swapped", "swapped_partial"} & set(got) and m.low


@pytest.mark.parametrize("soc,want", [(19.0, None), (20.0, "swapped_partial"),
                                      (79.0, "swapped_partial"), (80.0, "swapped")])
def test_a_swap_resumes_at_swap_resume_and_parks_below(soc, want):  # U154 floor, U157 split
    """soc_pct is the minimum over present packs: swapping one of two low
    packs leaves the other low. Below the trigger nothing ends -- and the
    refill is used up, so a later charge to 20 % is NOT a swap (audit
    2026-11-11: the old latch ended the charging alarm at 20 %). From the
    trigger to swap_resume the alarm ends but the tasks wait for a person; at
    swap_resume it ends and they resume. MUTATION: '>' for '>=' at either
    boundary -> 20 or 80 lands in the wrong bucket -> red; one threshold for
    both -> 79 resumes -> red; keep the refill pending -> the 20 % charge
    below ends it -> red."""
    m = _mon()
    m.update(_packs(15.0, (True, 15.0), (True, 15.0)), 0.0)  # two low packs
    m.update(_packs(15.0, (False, 0.0), (True, 15.0)), 1.0)  # slot 0 pulled
    got = m.update(_packs(soc, (True, 95.0), (True, soc)), 8.0)  # slot 0 refilled 7 s later
    if want is None:
        assert not {"swapped", "swapped_partial"} & set(got) and m.low  # still low
        _plug(m, dict(_packs(soc, (True, 95.0), (True, soc)), charge="charging"), 9.0)
        got = m.update(dict(_packs(20.0, (True, 95.0), (True, 20.0)), charge="charging"), 12.0)
        assert got == ["alarm"] and m.phase == lb.PHASE_CHARGING  # charging to 20 is no swap
    else:
        assert got == [want] and not m.low  # ended, in the right bucket


def test_a_presence_glitch_is_not_a_swap():  # swap_absent_min_s
    """Audit 2026-11-11, reproduced on the robot: one frame of present:false
    then charging to 20 % ended the episode (two packs), and on a single-pack
    robot charging at 85 % one glitch frame ended it as a full swap. An
    absence shorter than swap_absent_min_s is forgotten. MUTATION: drop the
    duration check -> either case ends -> red; '>' for '>=' -> the exact 5 s
    swap below is missed -> red."""
    m = _mon()  # two packs
    m.update(_packs(15.0, (True, 15.0), (True, 15.0)), 0.0)
    m.update(_packs(15.0, (False, 0.0), (True, 15.0)), 1.0)  # glitch
    m.update(_packs(15.0, (True, 15.0), (True, 15.0)), 1.5)  # back after 0.5 s
    _plug(m, dict(_packs(15.0, (True, 15.0), (True, 15.0)), charge="charging"), 2.0)
    got = m.update(dict(_packs(20.0, (True, 20.0), (True, 20.0)), charge="charging"), 5.0)
    assert got == ["alarm"] and m.phase == lb.PHASE_CHARGING
    m2 = _mon()  # single pack, slot 0 always empty
    m2.update(_packs(15.0, (False, 0.0), (True, 15.0)), 0.0)
    m2.update(dict(_packs(85.0, (False, 0.0), (True, 85.0)), charge="charging"), 1.0)
    m2.update(dict(_packs(None, (False, 0.0), (False, 0.0)), charge="charging"), 2.0)  # glitch
    got = m2.update(dict(_packs(85.0, (False, 0.0), (True, 85.0)), charge="charging"), 2.5)
    assert got == [] and m2.phase == lb.PHASE_CHARGING  # still charging toward 100
    m3 = _mon()  # exactly swap_absent_min_s counts
    m3.update(_packs(15.0, (True, 15.0)), 0.0)
    m3.update(_packs(None, (False, 0.0)), 1.0)
    assert m3.update(_packs(90.0, (True, 90.0)), 6.0) == ["swapped"]


def test_a_slot_empty_when_the_episode_starts_counts():  # single_battery, the second slot filled
    """A pack put into a slot that was already empty is a swap too (once it has
    been empty for swap_absent_min_s, counted from the episode start).
    MUTATION: no presence tracking on the starting frame -> never closes -> red."""
    m = _mon()
    m.update(_packs(18.0, (False, 0.0), (True, 18.0)), 0.0)  # starts with slot 0 empty
    assert m.update(_packs(85.0, (True, 95.0), (True, 85.0)), 6.0) == ["swapped"]  # slot 0 filled


def test_a_swap_during_charging_ends_it_too():  # any episode phase
    """MUTATION: judge swaps in the low phase only -> red."""
    m = _mon()
    m.update(_packs(18.0, (True, 18.0)), 0.0)
    m.update(dict(_packs(30.0, (True, 30.0)), charge="charging"), 1.0)  # charging
    m.update(dict(_packs(None, (False, 0.0)), charge="charging"), 2.0)  # pulled
    assert m.update(_packs(90.0, (True, 90.0)), 9.0) == ["swapped"]


def test_swap_history_belongs_to_one_episode():  # reset on close
    """A slot still empty when an episode ends must not carry its empty-since
    time into the next one. Episode 1 ends at full charge with slot 0 out;
    episode 2 starts and slot 0 is filled 0.5 s later -- a glitch, not a swap.
    MUTATION: drop the presence reset in _end_episode -> slot 0 counts as empty
    since episode 1 (9 s) -> swapped_partial -> red. (The earlier version of
    this case ended episode 1 with a judged swap, which empties the history by
    itself, so the mutant stayed green -- 2026-11-11 audit.)"""
    m = _mon()
    m.update(_packs(15.0, (True, 15.0), (True, 15.0)), 0.0)  # episode 1
    _plug(m, dict(_packs(30.0, (True, 30.0), (True, 30.0)), charge="charging"), 1.0)
    m.update(dict(_packs(30.0, (False, 0.0), (True, 30.0)), charge="charging"), 2.0)  # slot 0 out
    assert m.update(dict(_packs(100.0, (False, 0.0), (True, 100.0)), charge="charging"),
                    3.0) == ["charge_full"]  # ends with slot 0 still out
    assert "low" in _plug(m, _packs(19.0, (False, 0.0), (True, 19.0)), 10.0)  # episode 2
    got = m.update(_packs(25.0, (True, 95.0), (True, 25.0)), 11.0)  # filled 0.5 s later
    assert not {"swapped", "swapped_partial"} & set(got) and m.low


def test_stop_once_at_five_percent():  # the 5 % latch
    """MUTATION: '<' 5 -> 5.0 does not stop -> red; no once-latch -> red."""
    m = _mon()
    m.update(_power(19.0), 0.0)  # episode
    assert "stop" in m.update(_power(5.0), 3.0)  # the last 5 %
    assert "stop" not in m.update(_power(4.0), 6.0)  # once


_OK = (20.0, 5.0, 2.0, 100.0, 80.0, 5.0, 10.0, 5.0, 5.0)


def _bad(i, v):  # the shipped tuple with one value replaced
    return _OK[:i] + (v,) + _OK[i + 1:]


@pytest.mark.parametrize("args", [
    _bad(0, None), _bad(1, 0.0), _bad(1, 20.0), _bad(2, 0),
    _bad(3, None), _bad(4, None),
    _bad(3, 101.0),   # full could never be reached
    _bad(3, 70.0),    # swap_resume above full
    _bad(4, 20.0),    # swap_resume not above the trigger
    _bad(5, None),    # swap_absent_min_s missing
    _bad(6, 0.0),     # cold_swap_min_rise_pct not positive
    _bad(2, float("nan")),  # NaN period: the alarm never repeats
    _bad(5, float("inf")),
    _bad(7, None), _bad(8, 0.0)])  # the CD-1 debounce pair
def test_thresholds_are_required_and_ordered(args):  # config refusals
    """CLAUDE.md 3.1 + U157 order stop < trigger < swap_resume <= full <= 100,
    every value finite and positive (audit 2026-11-11: a NaN period passed).
    MUTATION: drop any one comparison or the finite check -> its row -> red."""
    with pytest.raises(lb.LowBatteryConfigError):  # each bad tuple
        lb.LowBatteryMonitor(*args)  # refused


def test_equal_swap_resume_and_full_is_allowed():  # the '<=' ends of the order
    """MUTATION: '<' for '<=' -> a legal 100/100 config refused -> red."""
    lb.LowBatteryMonitor(*_bad(4, 100.0))


def test_a_nan_soc_is_no_reading():
    """MUTATION: let NaN through _num -> NaN compares False everywhere and the
    frame silently judges nothing (or worse) -> the episode below never starts
    on the NaN frame but must start on the real one."""
    m = _mon()
    assert m.update(_power(float("nan")), 0.0) == []
    assert m.update(_power(19.0), 1.0) == ["low", "alarm"]


def test_snapshot_and_rollback_undo_a_frame():
    """The wiring rolls a frame back when its handler failed, so the next
    frame raises the same events again. MUTATION: rollback skips a field ->
    the second "low" never comes -> red."""
    m = _mon()
    m.update(_packs(15.0, (False, 0.0), (True, 15.0)), 0.0)  # low, slot 0 empty
    m.update(dict(_packs(15.0, (False, 0.0), (True, 15.0)), charge="charging"), 1.0)  # raw true
    snap = m.snapshot()
    assert _plug(m, dict(_packs(15.0, (False, 0.0), (True, 15.0)), charge="charging"),
                 1.2) == ["charging", "alarm"]
    m.rollback(snap)
    assert m.snapshot() == snap  # every field, not just the phase (2026-11-11 audit)
    assert m.update(dict(_packs(15.0, (False, 0.0), (True, 15.0)), charge="charging"),
                    1.7) == ["charging", "alarm"]  # raised again
    # a rolled-back END must bring the restore facts back too (the cold-swap
    # baseline is cleared when an episode ends). MUTATION: rollback skips
    # restored_soc -> the snapshots differ -> red.
    m2 = _mon(prime=False)
    m2.restore(lb.PHASE_LOW, 8.0)
    m2.update(_power(69.0), 0.0)  # flag unknown: not judged
    snap2 = m2.snapshot()
    assert m2.update(_power(69.0), 0.6) == ["swapped_partial"]  # the cold swap ends it
    m2.rollback(snap2)
    assert m2.snapshot() == snap2 and m2.low


# -- restart protection, second layer + cold swap (U157, 2026-11-11) --------------------
@pytest.mark.parametrize("body,want", [
    (None, False), (_power(50.0), False), (_power(50.0, charging=True), True),
    (_power(99.0, charge="charging"), True), (_power(100.0, charging=True), False),
    ({"soc_pct": None, "charge": "charging", "list": []}, True)])
def test_charging_below_full_reads_the_chassis(body, want):
    """MUTATION: '<=' for '<' -> 100 still blocks -> red; unknown soc while
    charging treated as full -> red; ignore the six-state field -> red."""
    assert lb.charging_below_full(body, 100.0) is want


def test_the_guard_also_reads_the_debounced_flag():
    """CD-1 for the guard: a one-frame drop of the raw bit while charging is
    confirmed must not lift it. MUTATION: ignore stable_charging -> False -> red."""
    assert lb.charging_below_full(_power(50.0), 100.0, True) is True
    assert lb.charging_below_full(_power(50.0), 100.0, None) is False


def test_the_charging_debouncer():
    """15 S8.5 CD-1. Unknown until confirmed; a flicker shorter than the confirm
    time changes nothing either way. MUTATION: judge on one frame -> the
    flickers flip it -> red; start at False instead of unknown -> red."""
    d = lb.ChargingDebouncer(5.0, 5.0)
    assert d.update(True, 0.0) is None  # nothing known yet
    assert d.update(True, 4.9) is None
    assert d.update(True, 5.0) is True  # held 5 s
    assert d.update(False, 6.0) is True  # a drop...
    assert d.update(True, 7.0) is True  # ...shorter than 5 s: still charging
    assert d.update(False, 8.0) is True
    assert d.update(False, 13.0) is False  # held false 5 s
    assert d.update(True, 14.0) is False  # a blip of charging: not yet


@pytest.mark.parametrize("last,soc,want,cold", [(8.0, 69.0, ["swapped_partial"], True),
                                                (8.0, 85.0, ["swapped"], True),
                                                (8.0, 15.0, None, False),
                                                (19.0, 20.0, None, False),   # wobble
                                                (12.0, 21.9, None, False),   # 9.9 < 10
                                                (12.0, 22.0, ["swapped_partial"], True),
                                                (None, 69.0, None, False)])  # no soc on record
def test_a_cold_swap_is_judged_on_the_first_reading_after_a_restore(last, soc, want, cold):
    """Found live 2026-11-11: 8 % at power-off, a 69 % pack at boot, and the
    restored low phase kept alarming. Restored low + first reading >= the
    trigger, not charging, and >= cold_swap_min_rise_pct above the record ->
    a swap, same 80 % split; a 19 -> 20 wobble across the restart is not.
    MUTATION: no cold rule -> red; drop the rise check -> the wobble ends ->
    red; '>' for '>=' on the rise -> 22.0 stays -> red; judge on any later
    frame -> the 69 % frame below closes the wobble case -> red."""
    m = _mon(prime=False)  # right after a start: the charging flag is unknown
    m.restore(lb.PHASE_LOW, last)
    assert m.update(_power(soc), 0.0) == ["alarm"]  # flag unknown: not judged yet
    got = m.update(_power(soc), 0.6)  # flag confirmed not charging: judged now
    if want is None:
        assert m.low and not {"swapped", "swapped_partial"} & set(got)
        # 99 U171: a LATER large rise now ends it too -- as a running swap,
        # never as a cold one (the cold rule judges the first reading only).
        later = m.update(_power(69.0), 3.0)
        if last is None:  # no soc on record: nothing to measure from yet
            assert later == ["alarm"]
        else:
            assert later == ["swapped_partial"] and m.last_cold_swap is False
    else:
        assert got == want and not m.low and m.last_cold_swap is cold


def test_no_cold_swap_while_charging_or_outside_the_low_phase():
    """Charging at boot is the charging phase, and a restored charging phase
    unplugged above the trigger is interrupted -- neither is a cold swap.
    MUTATION: drop the `not charging` / phase condition -> red."""
    m = _mon(prime=False)
    m.restore(lb.PHASE_LOW, 8.0)
    assert _plug(m, _power(69.0, charging=True), 0.0) == ["charging", "alarm"]
    m2 = _mon(prime=False)
    m2.restore(lb.PHASE_CHARGING, 8.0)
    assert m2.update(_power(69.0), 0.0) == ["alarm"]  # unknown flag: still charging
    assert m2.update(_power(69.0), 0.6) == ["interrupted", "alarm"]


def test_a_hot_swap_is_not_reported_cold():
    """MUTATION: report every swap as cold -> red."""
    m = _mon()
    m.update(_packs(18.0, (True, 18.0)), 0.0)
    m.update(_packs(None, (False, 0.0)), 1.0)
    assert m.update(_packs(90.0, (True, 90.0)), 8.0) == ["swapped"] and not m.last_cold_swap


@pytest.mark.asyncio
async def test_charging_holds_a_preempted_task_without_any_episode(tmp_path, monkeypatch):
    """The second layer on its own: no memory row, no episode, a preempted
    task and a chassis charging at 50 % -> it stays suspended; at 100 % it
    resumes. MUTATION: drop resume_blocked from the wiring -> it resumes at
    50 % -> red."""
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch,
        rows=[goto_row("t-run", "suspended", 1, suspend_kind="yielding",
                       suspend_reason="preempted")],
        geo_sql=[ROUTE_SQL, DOCK_ON, HOME], route_membership_m=5.0, low_battery=LB)
    try:
        await _feed(h, _power(50.0, charging=True), 2.0)
        assert (await task_state(task_db, "t-run"))[0] == "suspended"
        await _feed(h, _power(100.0, charging=True), 1.0)
        assert await _poll_state(task_db, "t-run", "running", n=300)
    finally:
        await stop_p3(stop, loop)


@pytest.mark.asyncio
async def test_a_cold_swap_ends_a_restored_episode_end_to_end(tmp_path, monkeypatch):
    """Restored low phase, first reading 69 % single pack not charging ->
    cleared (battery_swap, cold_swap, auto_resume false), the preempted task
    parked for a person, the memory row idle. MUTATION: no cold rule -> no
    cleared -> red."""
    from xbrain.p3_task.dao.simple_daos import MemoryDAO
    from xbrain.p3_task.dao.tasks_dao import TasksDAO
    from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
    task_db = str(tmp_path / "task.db")
    async with aiosqlite.connect(task_db) as t:  # what the previous P3 left behind
        for stmt in ALL_DDL_STATEMENTS:
            await t.execute(stmt)
        await TasksDAO(t).insert(goto_row("t-run", "suspended", 1,
                                          suspend_kind="yielding", suspend_reason="preempted"))
        await MemoryDAO(t).put(lb.EPISODE_MEMORY_KEY,
                               lb.encode_episode(lb.PHASE_LOW, {"t-run"}, None, 8.0), 0)
        await t.commit()
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)
    try:
        await _feed(h, _packs(69.0, (False, 0.0), (True, 69.0)), 2.0)
        cleared = [e for e in h.frames("event/info/charging")
                   if e["detail"]["type"] == "low_battery_cleared"]
        assert len(cleared) == 1
        d = cleared[0]["detail"]
        assert (d["cause"], d["cold_swap"], d["auto_resume"], d["parked"]) == (
            "battery_swap", True, False, ["t-run"])
        assert (await task_state(task_db, "t-run")) == ("suspended", "passive", "low_battery")
        async with aiosqlite.connect(task_db) as t:
            raw = await MemoryDAO(t).get(lb.EPISODE_MEMORY_KEY)
        assert lb.decode_episode(raw) is None  # the record is idle
    finally:
        await stop_p3(stop, loop)


# -- __main__ (U157 keys) ---------------------------------------------------------------
_FULL_LB = {"trigger_soc_pct": 20.0, "stop_soc_pct": 5.0, "cloud_alarm_period_s": 2.0,
            "charge_full_soc_pct": 100.0, "swap_resume_soc_pct": 80.0,
            "swap_absent_min_s": 5.0, "cold_swap_min_rise_pct": 10.0,
            "plug_confirm_s": 5.0, "unplug_confirm_s": 5.0}


@pytest.mark.parametrize("patch,rc", [
    ({}, 0), ({"charge_full_soc_pct": None}, 78), ({"swap_resume_soc_pct": None}, 78),
    ({"charge_full_soc_pct": 99.0, "swap_resume_soc_pct": 99.5}, 78),  # swap_resume above full
    ({"charge_full_soc_pct": 99.0}, 0),  # a chassis that tops out at 99 %
    ({"swap_absent_min_s": None}, 78), ({"cold_swap_min_rise_pct": None}, 78),
    ({"cloud_alarm_period_s": float("nan")}, 78),  # NaN passed `<= 0` until 2026-11-11
    ({"plug_confirm_s": None}, 78), ({"unplug_confirm_s": 0.0}, 78)])
def test_main_refuses_without_the_u157_keys(monkeypatch, patch, rc):
    """EX_CONFIG 78 by key path, and the five values reach the wiring.
    MUTATION: drop either new key from the __main__ check, or the order check
    -> its row starts -> red; forget to pass a key -> KeyError / wrong dict
    -> red."""
    import xbrain.common.config.resolved as resolved  # where main loads from
    import xbrain.p3_task.runtime.main_wiring as mw  # the wiring main calls
    from xbrain.p3_task import __main__ as p3main  # unit under test
    cfg = {"geo": {"enu_origin": {"lat": 31.0, "lon": 121.0, "alt": 0.0}},  # minimal resolved config
           "teach": {"snap_max_age_ms": 8000, "min_dist_m": 0.5, "sample_hz": 1.0,
                     "record_duration_s": 10800, "record_warn_before_s": [600, 60]},
           "route": {"recorded_arrive_radius_m": 1.0, "membership_m": 5.0},
           "rtk_loss": {"lost_sustain_s": 1.0, "recover_sustain_s": 2.0},
           "low_battery": dict(_FULL_LB, **patch)}
    monkeypatch.setattr(resolved, "load_resolved", lambda *_a, **_k: cfg)  # no file
    seen = {}  # what the wiring received
    monkeypatch.setattr(mw, "run_voice_loop_wiring", lambda **kw: seen.update(kw) or 0)
    assert p3main.main(["--voice-loop"]) == rc  # the exit code
    if rc == 0:
        assert seen["low_battery"] == dict(_FULL_LB, **patch)  # all five, as configured


# -- the persisted episode (U157) -------------------------------------------------------
def test_episode_record_round_trips_and_garbage_holds_nothing():
    """MUTATION: decode an idle / unreadable row as an episode -> red."""
    raw = lb.encode_episode(lb.PHASE_CHARGING, {"t-2", "t-1"}, None, 37)
    assert lb.decode_episode(raw) == (lb.PHASE_CHARGING, {"t-1", "t-2"}, None, 37.0)
    old = b'{"phase":"low","held":["t-1"],"task_id":null}'  # written before soc was stored
    assert lb.decode_episode(old) == (lb.PHASE_LOW, {"t-1"}, None, None)
    assert lb.decode_episode(lb.encode_episode(lb.PHASE_IDLE, set(), None)) is None
    assert lb.decode_episode(b"{not json") is None and lb.decode_episode(None) is None
    assert lb.decode_episode(b'{"phase":"bogus","held":[]}') is None


def test_restore_rejects_an_unknown_phase_and_restarts_the_cadence():
    """MUTATION: restore without resetting the cadence -> no alarm on the first
    frame after a restart -> red."""
    m = _mon()
    with pytest.raises(ValueError):
        m.restore("nap")
    assert m.update(_power(19.0), 0.0) == ["low", "alarm"]  # a cadence is running
    m.restore(lb.PHASE_LOW, 19.0)  # (production restores a fresh monitor; this pins the reset)
    assert m.update(_power(19.0), 0.1) == ["alarm"]  # alarm at once, not 2 s after 0.0


# -- the target ------------------------------------------------------------------------
@pytest_asyncio.fixture
async def geo():  # catalogue fixture
    """An in-memory geo.db with the route."""
    conn = await aiosqlite.connect(":memory:")  # CLAUDE.md 7.2
    for stmt in GEO_DB_STATEMENTS:  # real schema
        await conn.execute(stmt)  # real schema
    await conn.execute(*ROUTE_SQL)  # the route
    await conn.commit()  # no open transaction left for the appliers' BEGIN IMMEDIATE
    yield conn  # to the case
    await conn.close()  # cleanup


async def _seed(conn, *rows):  # seed helper
    """Insert waypoint rows and commit."""
    for sql, params in rows:  # each row
        await conn.execute(sql, params)  # one row
    await conn.commit()  # visible


@pytest.mark.asyncio
@pytest.mark.parametrize("rows,want", [
    ((DOCK_ON, DOCK_OFF, HOME), ("w-d1", "dock")),  # the on-route dock, not the closer off-route one
    ((DOCK_OFF, HOME), ("w-home", "home")),  # off-route docks never qualify -> home
    ((DOCK_OFF,), None),  # nothing qualifies
])
async def test_pick_target(geo, rows, want):  # dock vs home choice
    """MUTATION: drop the on-route filter -> w-d2 / no home fallback -> red."""
    await _seed(geo, *rows)  # the catalogue
    routes = await load_route_geoms(geo)  # as P3 reads them
    got = await lb.pick_target(geo, (35.0, 135.0), routes, 5.0)  # from the robot
    assert (None if got is None else (got.geo_id, got.kind)) == want  # chosen or none


@pytest.mark.asyncio
async def test_unreachable_compares_remaining_metres(geo):  # remaining range check
    """MUTATION: treat a missing remain_mile_km as 0 -> True -> red."""
    await _seed(geo, DOCK_ON)  # one dock, ~167 m away along the route
    routes = await load_route_geoms(geo)  # routes as P3 reads them
    tgt = await lb.pick_target(geo, (35.0, 135.0), routes, 5.0)  # the dock
    assert lb.unreachable(_power(19.0, remain_km=0.1), (35.0, 135.0), tgt, routes, 5.0) is True  # 100 m left, ~167 m to go
    assert lb.unreachable(_power(19.0, remain_km=1.0), (35.0, 135.0), tgt, routes, 5.0) is False  # 1 km left: enough
    assert lb.unreachable({"soc_pct": 19.0}, (35.0, 135.0), tgt, routes, 5.0) is None  # not judged


@pytest.mark.asyncio
@pytest.mark.parametrize("lat,lon,refused", [(35.0015, 135.00002, False), (35.0, 135.01, True)])
async def test_a_dock_off_every_route_is_refused(geo, lat, lon, refused):  # upsert path
    """99 U146 dock-on-route, cmd/geo upsert path. MUTATION: skip the check
    -> the off-route dock is stored -> red."""
    from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload  # the applier
    ctx = GeoContext(geo_conn=geo, fence_conn=geo, route_membership_m=5.0)  # as wired
    ack = await handle_geo_payload({  # an HMI create
        "cmd_id": "g-1", "action": "upsert", "type": "waypoint", "geo_id": "w-new",
        "origin": "hmi", "base_rev": 0,
        "obj": {"name": "3号充电站", "geom": {"lat": lat, "lon": lon}}}, ctx, now_ms=1)
    assert (ack["result"] == "rejected") is refused  # off-route refused
    if refused:  # the refusal names why
        assert ack["code"] == E_GEO_INVALID and ack["detail"]["reason"] == "dock_off_route"


@pytest.mark.asyncio
@pytest.mark.parametrize("lat,lon,refused", [(35.0015, 135.00002, False), (35.0, 135.01, True)])
async def test_renaming_into_a_dock_off_every_route_is_refused(geo, lat, lon, refused):  # rename path
    """99 U146, rename path: a poi renamed "X充电站" becomes a dock, so it must
    be on a route. MUTATION: skip the rename check -> stored -> red."""
    from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload  # the applier
    await _seed(geo, _wp("w-p", "旧点", "poi", lat, lon))  # a plain point
    ctx = GeoContext(geo_conn=geo, fence_conn=geo, route_membership_m=5.0)  # as wired
    ack = await handle_geo_payload({  # a voice rename
        "cmd_id": "g-2", "action": "rename", "type": "waypoint", "geo_id": "w-p",
        "origin": "voice", "base_rev": 1, "obj": {"name": "4号充电站"}}, ctx, now_ms=2)
    assert (ack["result"] == "rejected") is refused, ack  # off-route refused
    if refused:  # the refusal names why
        assert ack["code"] == E_GEO_INVALID and ack["detail"]["reason"] == "dock_off_route"


# -- through the real P3 loop ------------------------------------------------------------
LB = {"trigger_soc_pct": 20.0, "stop_soc_pct": 5.0, "cloud_alarm_period_s": 2.0,  # the rule
      "charge_full_soc_pct": 100.0, "swap_resume_soc_pct": 80.0,  # U157
      "swap_absent_min_s": 5.0, "cold_swap_min_rise_pct": 10.0,  # U157, 2026-11-11
      "plug_confirm_s": 1.0, "unplug_confirm_s": 1.0}  # CD-1, short for the loop cases


async def _feed(h, body, seconds):  # power frames at 2 Hz
    """state/power at 2 Hz for `seconds` (the chassis sends 1 Hz; faster is fine)."""
    from xbrain.p3_task.runtime.main_wiring import STATE_POWER_TOPIC  # CR-5 key
    for _ in range(int(seconds * 2)):  # frame count
        h.send(STATE_POWER_TOPIC, {"data": body})  # as the relay forwards it
        await asyncio.sleep(0.5)


async def _poll_state(task_db, tid, want, n=100):  # the loop writes, poll it
    """True once task `tid` reaches state `want` (committed)."""
    for _ in range(n):
        if (await task_state(task_db, tid))[0] == want:
            return True
        await asyncio.sleep(0.02)  # give the loop a turn
    return False


@pytest.mark.asyncio
async def test_the_episode_end_to_end(tmp_path, monkeypatch):  # the whole episode
    """Low -> charge task to the on-route dock preempts t-run, alarms every 2 s
    with only the first marked; 5 % -> running task suspended, P1 cleared,
    prone sent, submits refused; charging -> the low-battery task is
    cancelled, the alarm goes on worded battery_charging and t-run STAYS
    suspended (99 U157 -- until 2026-11-10 it resumed here); 100 % -> cleared
    with cause charge_full and t-run resumes by itself.
    MUTATION: no task / no cadence / no prone / no refusal / task left
    suspended -> red; end the episode on charging -> t-run running while
    charging -> red; never end at full -> no cleared / t-run never resumes ->
    red. (Dropping only the scheduler's held filter does NOT redden this one:
    while charging the chassis guard holds t-run too. The held filter is
    pinned by test_a_queued_task_waits_for_the_episode_and_a_human_resume_does_not,
    which runs with the chassis not charging -- corrected 2026-11-11 audit.)"""
    monkeypatch.setenv("XBRAIN_ROBOT_ID", "gj-test")  # the prone envelope needs a rid
    from xbrain.p3_task.runtime.main_wiring import CMD_TASK_TOPIC  # where P4 sends
    h, stop, loop, task_db = await start_p3(  # loop with the rule on
        tmp_path, monkeypatch, rows=[goto_row("t-run", "ready", 1)],
        geo_sql=[ROUTE_SQL, DOCK_ON, DOCK_OFF, HOME],
        route_membership_m=5.0, low_battery=LB)  # the rule on
    try:
        assert await wait_for(lambda: route_ops(h))  # t-run running
        await _feed(h, _power(18.0), 5.0)  # low for 5 s
        alarms = h.frames("event/alarm/charging")  # cloud reports
        lows = [a for a in alarms if a["detail"]["type"] == "low_battery"]  # the low reports
        assert 2 <= len(lows) <= 4  # every 2 s, not every frame
        assert [a["detail"]["first"] for a in lows][:2] == [True, False]  # one voice only
        assert lows[0]["detail"]["target_name"] == "1号充电站"  # the on-route dock
        tid = lows[0]["detail"]["task_id"]  # the injected task
        assert (await task_state(task_db, tid))[0] in ("ready", "running")  # injected
        assert (await task_state(task_db, "t-run"))[:3] == ("suspended", "yielding", "preempted")
        await _feed(h, _power(4.0), 2.0)  # the last 5 %
        assert await wait_for(lambda: h.frames("cmd/chassis/ctrl"))  # prone sent
        ctrl = h.frames("cmd/chassis/ctrl")[0]  # the envelope
        assert ctrl["data"]["action"] == "prone" and ctrl["src"] == "p3_task"  # enveloped
        assert any(a["detail"]["type"] == "low_battery_stop" for a in h.frames("event/alarm/charging"))
        h.send(CMD_TASK_TOPIC, {"cmd_id": "c-x", "action": "submit", "source": "voice",  # a submit while lying down
                                "task": {"type": "goto", "priority": 5,
                                         "params": {"waypoint_id": "w-gate"}}})
        assert await wait_for(lambda: h.frames("cmd/task/ack"))  # answered
        assert h.frames("cmd/task/ack")[-1]["code"] == E_LOW_BATTERY  # refused
        await _feed(h, _power(40.0, charging=True), 3.0)  # docked and charging
        assert await _poll_state(task_db, tid, "cancelled")  # the return task is closed
        charging = [a for a in h.frames("event/alarm/charging")
                    if a["detail"]["type"] == "battery_charging"]
        assert len(charging) >= 1 and not any(a["detail"]["first"] for a in charging)  # never voiced
        assert not [e for e in h.frames("event/info/charging")
                    if e["detail"]["type"] == "low_battery_cleared"]  # not ended by charging
        assert (await task_state(task_db, "t-run"))[0] == "suspended"  # held on the charger
        await _feed(h, _power(100.0, charging=True), 1.0)  # full
        cleared = [e for e in h.frames("event/info/charging")
                   if e["detail"]["type"] == "low_battery_cleared"]
        assert len(cleared) == 1 and cleared[0]["detail"]["cause"] == "charge_full"
        assert cleared[0]["detail"]["auto_resume"] is True and "t-run" in cleared[0]["detail"]["held"]
        assert await _poll_state(task_db, "t-run", "running", n=300)  # resumed by itself
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_a_queued_task_waits_for_the_episode_and_a_human_resume_does_not(
        tmp_path, monkeypatch):  # U157 ruling 3 + the manual release
    """A task queued before the episode does not dispatch when the return task
    arrives at the dock (it waits for the end); a held task a person resumes
    leaves the hold and runs. MUTATION: hold preempted tasks only -> t-q
    dispatches on arrival -> red; ignore the resume release -> t-run stays
    suspended -> red."""
    from xbrain.p3_task.runtime.main_wiring import CMD_TASK_TOPIC  # where P4 sends
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, rows=[goto_row("t-run", "ready", 1),
                                     goto_row("t-q", "pending", 2)],
        geo_sql=[ROUTE_SQL, DOCK_ON, HOME], route_membership_m=5.0, low_battery=LB)
    try:
        assert await wait_for(lambda: route_ops(h))  # t-run running
        await _feed(h, _power(18.0), 2.0)  # low: t-run preempted
        tid = h.frames("event/alarm/charging")[0]["detail"]["task_id"]
        async with aiosqlite.connect(task_db) as t:  # the return task arrives
            await t.execute("UPDATE tasks SET state='done' WHERE task_id=?", (tid,))
            await t.commit()
        await _feed(h, _power(18.0), 2.0)  # a few ticks with nothing running
        assert (await task_state(task_db, "t-q"))[0] in ("pending", "ready")  # held
        assert (await task_state(task_db, "t-run"))[0] == "suspended"  # held
        h.send(CMD_TASK_TOPIC, {"cmd_id": "c-r", "action": "resume", "source": "hmi",
                                "task_id": "t-run"})  # a person resumes it
        assert await _poll_state(task_db, "t-run", "running", n=300)  # their call
        assert (await task_state(task_db, "t-q"))[0] in ("pending", "ready")  # t-q still held
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_a_battery_swap_ends_the_episode_end_to_end(tmp_path, monkeypatch):  # the swap path
    """99 U154 / U157: lying down at 4 %, the pack is pulled and a 95 % one put
    in -> low_battery_cleared with cause battery_swap and auto_resume, the
    low-battery task cancelled, and submits no longer refused with
    E_LOW_BATTERY. MUTATION: "swapped" not wired -> no clear / task left /
    still refused -> red."""
    monkeypatch.setenv("XBRAIN_ROBOT_ID", "gj-test")  # the prone envelope needs a rid
    from xbrain.p3_task.runtime.main_wiring import CMD_TASK_TOPIC  # where P4 sends
    h, stop, loop, task_db = await start_p3(  # loop with the rule on
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)  # the rule on
    try:
        await _feed(h, _packs(4.0, (True, 4.0)), 2.0)  # low and the last 5 % at once
        assert await wait_for(lambda: h.frames("cmd/chassis/ctrl"))  # prone sent
        tid = h.frames("event/alarm/charging")[0]["detail"]["task_id"]  # the injected task
        await _feed(h, _packs(None, (False, 0.0)), 6.0)  # pulled out, longer than swap_absent_min_s
        assert not h.frames("event/info/charging")  # an empty slot closes nothing
        await _feed(h, _packs(95.0, (True, 95.0)), 1.0)  # a full pack put in
        cleared = [e for e in h.frames("event/info/charging")
                   if e["detail"]["type"] == "low_battery_cleared"]  # the close report
        assert len(cleared) == 1 and cleared[0]["detail"]["cause"] == "battery_swap"  # once, by swap
        assert cleared[0]["detail"]["auto_resume"] is True
        assert await _poll_state(task_db, tid, "cancelled")  # the task is closed too
        h.send(CMD_TASK_TOPIC, {"cmd_id": "c-y", "action": "submit", "source": "voice",  # a submit after the swap
                                "task": {"type": "goto", "priority": 5,
                                         "params": {"waypoint_id": "w-home"}}})
        assert await wait_for(lambda: [a for a in h.frames("cmd/task/ack") if a["cmd_id"] == "c-y"])
        ack = [a for a in h.frames("cmd/task/ack") if a["cmd_id"] == "c-y"][-1]  # its answer
        assert ack["code"] != E_LOW_BATTERY  # the 5 % refusal is lifted
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_a_partial_swap_parks_the_interrupted_task_for_a_person(
        tmp_path, monkeypatch):  # U157 ruling 1
    """Swapped to 60 %: the alarm ends (auto_resume false) and the preempted
    task is re-tagged passive / low_battery -- it never resumes by itself.
    MUTATION: treat 60 % as a full swap, or skip the re-tag -> t-run running
    -> red."""
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, rows=[goto_row("t-run", "ready", 1)],
        geo_sql=[ROUTE_SQL, DOCK_ON, HOME], route_membership_m=5.0, low_battery=LB)
    try:
        assert await wait_for(lambda: route_ops(h))  # t-run running
        await _feed(h, _packs(18.0, (True, 18.0)), 2.0)  # low: t-run preempted
        await _feed(h, _packs(None, (False, 0.0)), 6.0)  # pulled, longer than swap_absent_min_s
        await _feed(h, _packs(60.0, (True, 60.0)), 2.0)  # a 60 % pack in
        cleared = [e for e in h.frames("event/info/charging")
                   if e["detail"]["type"] == "low_battery_cleared"]
        assert len(cleared) == 1 and cleared[0]["detail"]["auto_resume"] is False
        assert cleared[0]["detail"]["parked"] == ["t-run"]
        await _feed(h, _packs(60.0, (True, 60.0)), 1.0)  # ticks with nothing running
        assert (await task_state(task_db, "t-run")) == ("suspended", "passive", "low_battery")
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_a_restart_mid_charge_keeps_holding(tmp_path, monkeypatch):  # the memory row
    """P3 restarted while charging: the episode comes back from task.db memory
    (the alarm is worded battery_charging again), the preempted task stays
    suspended, and full charge resumes it. MUTATION: skip the restore -> no
    battery_charging alarm -> red. (t-run staying suspended here is also held
    by the chassis guard; the restored held set alone is pinned by
    test_a_restart_in_the_low_phase_keeps_holding -- 2026-11-11 audit.)"""
    from xbrain.p3_task.dao.simple_daos import MemoryDAO
    from xbrain.p3_task.dao.tasks_dao import TasksDAO
    from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
    task_db = str(tmp_path / "task.db")
    async with aiosqlite.connect(task_db) as t:  # what the previous P3 left behind
        for stmt in ALL_DDL_STATEMENTS:
            await t.execute(stmt)
        await TasksDAO(t).insert(goto_row("t-run", "suspended", 1,
                                          suspend_kind="yielding", suspend_reason="preempted"))
        await MemoryDAO(t).put(lb.EPISODE_MEMORY_KEY,
                               lb.encode_episode(lb.PHASE_CHARGING, {"t-run"}, None), 0)
        await t.commit()
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)
    try:
        await _feed(h, _power(50.0, charging=True), 2.0)  # still charging after the restart
        assert (await task_state(task_db, "t-run"))[0] == "suspended"  # held
        assert any(a["detail"]["type"] == "battery_charging"
                   for a in h.frames("event/alarm/charging"))  # the phase came back
        await _feed(h, _power(100.0, charging=True), 1.0)  # full
        assert await _poll_state(task_db, "t-run", "running", n=300)  # resumed
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_boot_reports_a_missing_home(tmp_path, monkeypatch):  # the boot report
    """MUTATION: drop the boot check -> no report -> red."""
    h, stop, loop, _db = await start_p3(tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_OFF],  # no home, one off-route dock
                                        route_membership_m=5.0, low_battery=LB)
    try:
        assert await wait_for(lambda: h.frames("event/alarm/charging"))  # reported at boot
        d = h.frames("event/alarm/charging")[0]["detail"]  # its detail
        assert d["type"] == "return_target_missing"  # the type
        assert d["missing"] == ["返航终点站", "充电桩/充电站"] and d["docks_off_route"] == ["2号充电站"]
    finally:
        await stop_p3(stop, loop)  # cleanup


# -- 2026-11-11 audit: restart, retry and F-5 cases through the real loop -------------------
async def _seed_task_db(tmp_path, rows, record=None):
    """task.db as a previous P3 left it: rows plus an optional episode record."""
    from xbrain.p3_task.dao.simple_daos import MemoryDAO
    from xbrain.p3_task.dao.tasks_dao import TasksDAO
    from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
    task_db = str(tmp_path / "task.db")
    async with aiosqlite.connect(task_db) as t:
        for stmt in ALL_DDL_STATEMENTS:
            await t.execute(stmt)
        for row in rows:
            await TasksDAO(t).insert(row)
        if record is not None:
            await MemoryDAO(t).put(lb.EPISODE_MEMORY_KEY, record, 0)
        await t.commit()
    return task_db


async def _live_states(task_db):
    """{task_id: state} of every row (committed)."""
    async with aiosqlite.connect(task_db) as t:
        cur = await t.execute("SELECT task_id, state FROM tasks")
        return dict(await cur.fetchall())


@pytest.mark.asyncio
async def test_a_restart_in_the_low_phase_keeps_holding(tmp_path, monkeypatch):
    """Restored low phase, the return task already arrived (done), chassis NOT
    charging: the preempted task stays suspended -- only the restored held set
    can hold it (no chassis guard here), and no new return task is sent (the
    robot is at the dock). MUTATION: skip restoring `held` -> t-run resumes ->
    red; reissue a done return task -> a second lb- row -> red."""
    rows = [goto_row("t-run", "suspended", 1, suspend_kind="yielding",
                     suspend_reason="preempted"),
            goto_row("lb-old", "done", 2, priority=95)]
    await _seed_task_db(tmp_path, rows,
                        lb.encode_episode(lb.PHASE_LOW, {"t-run"}, "lb-old", 15.0))
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)
    try:
        await _feed(h, _power(15.0), 3.0)
        st = await _live_states(task_db)
        assert st["t-run"] == "suspended"
        assert [t for t in st if t.startswith("lb-") and t != "lb-old"] == []
    finally:
        await stop_p3(stop, loop)


@pytest.mark.asyncio
async def test_a_restart_while_returning_reissues_the_return_task(tmp_path, monkeypatch):
    """User ruling 2026-11-11: the return task was running at the power cut, so
    boot recovery suspends it (passive, unclean_shutdown) and it would wait for
    a person while the alarm says "returning". P3 cancels it and sends a fresh
    one. MUTATION: drop the reissue -> no new lb- row, lb-old suspended -> red."""
    rows = [goto_row("lb-old", "running", 1, priority=95)]
    await _seed_task_db(tmp_path, rows,
                        lb.encode_episode(lb.PHASE_LOW, set(), "lb-old", 15.0))
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)
    try:
        await _feed(h, _power(15.0), 2.0)
        st = await _live_states(task_db)
        assert st["lb-old"] == "cancelled"
        fresh = [t for t in st if t.startswith("lb-") and t != "lb-old"]
        assert len(fresh) == 1 and st[fresh[0]] in ("pending", "ready", "running")
    finally:
        await stop_p3(stop, loop)


@pytest.mark.asyncio
async def test_a_failed_low_handler_is_retried(tmp_path, monkeypatch):
    """Audit 2026-11-11: if the "low" handler raised (here: pick_target fails
    once), the monitor stayed in the low phase with no return task and an
    empty held set, and nothing retried it. Now the frame is rolled back and
    the next frame does it. MUTATION: drop the rollback -> no lb- task, t-run
    not preempted -> red."""
    calls = {"n": 0}
    real = lb.pick_target

    async def flaky(*a, **k):
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("geo.db busy")
        return await real(*a, **k)
    monkeypatch.setattr(lb, "pick_target", flaky)
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, rows=[goto_row("t-run", "ready", 1)],
        geo_sql=[ROUTE_SQL, DOCK_ON, HOME], route_membership_m=5.0, low_battery=LB)
    try:
        assert await wait_for(lambda: route_ops(h))  # t-run running
        await _feed(h, _power(18.0), 3.0)
        st = await _live_states(task_db)
        assert calls["n"] >= 2
        assert [t for t in st if t.startswith("lb-")], st  # injected on the retry
        assert st["t-run"] == "suspended"  # preempted by it
    finally:
        await stop_p3(stop, loop)


@pytest.mark.asyncio
async def test_a_link_loss_return_waits_for_the_episode(tmp_path, monkeypatch):
    """User ruling 2026-11-11: an F-5 return_home injected during a low-battery
    episode is held like a queued task, so it cannot drive the robot off the
    dock after the return task arrives. MUTATION: do not hold it -> the rh-
    task runs once the return task is done -> red."""
    from xbrain.p3_task.runtime.main_wiring import STATE_LINK_TOPIC
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)
    try:
        await _feed(h, _power(18.0), 2.0)  # episode: return task injected
        tid = h.frames("event/alarm/charging")[0]["detail"]["task_id"]
        h.send(STATE_LINK_TOPIC, {"level": 3, "gw_start_mono": 100.0, "link_epoch": 2,
                                  "disconnected_s": 1801.0})  # cloud down past rtb_s
        await _feed(h, _power(18.0), 2.0)
        rh = [t for t in await _live_states(task_db) if t.startswith("rh-")]
        assert rh, "F-5 did not inject"
        async with aiosqlite.connect(task_db) as t:  # the return task arrives
            await t.execute("UPDATE tasks SET state='done' WHERE task_id=?", (tid,))
            await t.commit()
        await _feed(h, _power(18.0), 3.0)  # ticks with nothing running
        assert (await _live_states(task_db))[rh[0]] in ("pending", "ready")  # held
    finally:
        await stop_p3(stop, loop)


@pytest.mark.asyncio
async def test_a_retry_after_the_insert_does_not_send_a_second_return_task(
        tmp_path, monkeypatch):
    """The "low" handler failing AFTER the return task was inserted (here: the
    held snapshot fails once) is rolled back: the task that frame inserted is
    cancelled (no return task may drive without an episode behind it) and the
    retry inserts the one live return task. The episode record then follows
    the whole-percent soc. MUTATION: skip the orphan cancel -> two live lb-
    rows -> red; drop the soc re-persist -> the record keeps 18 -> red."""
    calls = {"n": 0}
    real = lb.live_task_ids

    async def flaky(dao):
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("task.db busy")
        return await real(dao)
    monkeypatch.setattr(lb, "live_task_ids", flaky)
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)
    try:
        await _feed(h, _power(18.0), 3.0)
        assert calls["n"] >= 2
        st = await _live_states(task_db)
        live = [t for t in st if t.startswith("lb-") and st[t] != "cancelled"]
        assert len(live) == 1, st
        await _feed(h, _power(16.0), 1.5)  # no phase change: only the soc moved
        from xbrain.p3_task.dao.simple_daos import MemoryDAO
        async with aiosqlite.connect(task_db) as t:
            rec = lb.decode_episode(await MemoryDAO(t).get(lb.EPISODE_MEMORY_KEY))
        assert rec is not None and rec[3] == 16.0  # re-persisted on the whole-percent change
    finally:
        await stop_p3(stop, loop)


# -- 2026-11-11 third audit -----------------------------------------------------------------
def _return_row(task_id, state, seq, waypoint_id):  # an injected return task as P3 writes it
    from xbrain.p3_task.dao.tasks_dao import TaskRow
    return TaskRow(task_id=task_id, task_type="charge", state=state, priority=95, submit_seq=seq,
                   mission_json=json.dumps({"source": "auto", "reason": "low_battery",
                                            "params": {"waypoint_id": waypoint_id}}),
                   total_steps=1, current_step=0, step_status_json="[]", created_ms=0,
                   updated_ms=0, source="charge", trace_id="lowbatt-" + task_id,
                   resume_policy="continue")


@pytest.mark.asyncio
async def test_the_low_battery_return_preempts_a_link_loss_return(tmp_path, monkeypatch):
    """User ruling 2026-11-11: a link-loss return_home running when the battery
    drops below 20 % is preempted by the return task (both 95) and waits in the
    held set. MUTATION: no urgent_task_id from the wiring -> rh- keeps running
    -> red."""
    from xbrain.p3_task.runtime.main_wiring import STATE_LINK_TOPIC
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)
    try:
        await _feed(h, _power(60.0), 1.5)
        h.send(STATE_LINK_TOPIC, {"level": 3, "gw_start_mono": 100.0, "link_epoch": 2,
                                  "disconnected_s": 1801.0})
        await _feed(h, _power(60.0), 1.5)
        rh = [t for t, st in (await _live_states(task_db)).items() if t.startswith("rh-")]
        assert rh and (await _live_states(task_db))[rh[0]] == "running"
        await _feed(h, _power(18.0), 2.5)
        st = await _live_states(task_db)
        assert st[rh[0]] == "suspended"
        assert [t for t in st if t.startswith("lb-") and st[t] == "running"]
    finally:
        await stop_p3(stop, loop)


@pytest.mark.asyncio
async def test_a_reissue_waits_for_the_first_power_frame(tmp_path, monkeypatch):
    """Audit 2026-11-11: the boot reissue was dispatched before any power frame
    was judged. Now nothing is sent until a frame is in; at <= 5 % none is sent
    (the stop latched). MUTATION: inject at boot -> an lb- row before any frame
    -> red; ignore `stopped` -> a row at 4 % -> red."""
    rows = [goto_row("lb-old", "running", 1, priority=95)]
    await _seed_task_db(tmp_path, rows, lb.encode_episode(lb.PHASE_LOW, set(), "lb-old", 15.0))
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)
    try:
        await asyncio.sleep(1.5)  # no power frame yet
        st = await _live_states(task_db)
        assert st["lb-old"] == "cancelled" and [t for t in st if t.startswith("lb-")] == ["lb-old"]
        await _feed(h, _power(4.0), 2.0)  # the last 5 %
        st = await _live_states(task_db)
        assert [t for t in st if t.startswith("lb-")] == ["lb-old"]  # stopped: none sent
    finally:
        await stop_p3(stop, loop)


@pytest.mark.asyncio
async def test_a_restored_alarm_names_the_return_tasks_own_target(tmp_path, monkeypatch):
    """Audit 2026-11-11: after a restart the alarm named a pose-less pick (the
    oldest dock). It now names the live return task's own waypoint. MUTATION:
    pick again without a pose -> the older far dock is named -> red."""
    far = _wp("w-d0", "0号充电站", "dock", 35.0019, 135.00002)  # older id, on the route, farther
    rows = [_return_row("lb-old", "pending", 1, "w-d1")]
    await _seed_task_db(tmp_path, rows, lb.encode_episode(lb.PHASE_LOW, set(), "lb-old", 15.0))
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, far, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)
    try:
        await _feed(h, _power(15.0), 2.0)
        lows = [a for a in h.frames("event/alarm/charging") if a["detail"]["type"] == "low_battery"]
        assert lows and lows[0]["detail"]["target_name"] == "1号充电站"
    finally:
        await stop_p3(stop, loop)


@pytest.mark.asyncio
async def test_a_persist_without_a_reading_keeps_the_recorded_soc(tmp_path, monkeypatch):
    """Audit 2026-11-11: a persist before the first power frame wrote soc None
    and erased the cold-swap baseline. MUTATION: write None when no reading ->
    the record's soc is None -> red."""
    from xbrain.p3_task.dao.simple_daos import MemoryDAO
    from xbrain.p3_task.runtime.main_wiring import CMD_TASK_TOPIC
    rows = [goto_row("t-q", "pending", 1), _return_row("lb-old", "pending", 2, "w-d1")]
    await _seed_task_db(tmp_path, rows, lb.encode_episode(lb.PHASE_LOW, {"t-q"}, "lb-old", 15.0))
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)
    try:
        h.send(CMD_TASK_TOPIC, {"cmd_id": "c-c", "action": "cancel", "source": "hmi",
                                "task_id": "t-q"})  # releases t-q -> a persist, no frame yet
        assert await _poll_state(task_db, "t-q", "cancelled")
        await asyncio.sleep(0.5)
        async with aiosqlite.connect(task_db) as t:
            rec = lb.decode_episode(await MemoryDAO(t).get(lb.EPISODE_MEMORY_KEY))
        assert rec is not None and rec[1] == set() and rec[3] == 15.0
    finally:
        await stop_p3(stop, loop)


@pytest.mark.asyncio
async def test_an_episode_starting_at_five_percent_preempts_not_parks(tmp_path, monkeypatch):
    """Audit 2026-11-11: an episode that began at <= 5 % in one frame parked the
    running task passive / low_battery, so it never resumed although the end
    said auto_resume. It is now suspended yielding / preempted. MUTATION: park
    every running task passive -> red."""
    monkeypatch.setenv("XBRAIN_ROBOT_ID", "gj-test")  # the prone envelope needs a rid
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, rows=[goto_row("t-run", "ready", 1)],
        geo_sql=[ROUTE_SQL, DOCK_ON, HOME], route_membership_m=5.0, low_battery=LB)
    try:
        assert await wait_for(lambda: route_ops(h))  # t-run running
        await _feed(h, _power(4.0), 2.0)  # low and stop in one frame
        assert (await task_state(task_db, "t-run")) == ("suspended", "yielding", "preempted")
    finally:
        await stop_p3(stop, loop)

