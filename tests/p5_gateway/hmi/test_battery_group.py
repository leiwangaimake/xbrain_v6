"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_battery_group.py
Brief: HMI toolbar battery gauges (99 U160) -- state/power.list[] -> snapshot.battery

Description:
The toolbar shows two battery slots, A = index 0 and B = index 1 (user ruling
2026-11-11), from state/power.list[] (11 S4.2). Three rules carry the weight
and are pinned here:

  * an EMPTY slot is "absent" with no numbers. The chassis reports it as
    level 0, 0 V, -273 C (measured on the robot 2026-11-11); passing those
    through would paint a red "0%" and an impossible temperature;
  * no state/power, a stale one, or a missing slot is "unknown", never a
    zero -- the gauges grey out instead of freezing old numbers;
  * the slot -> label map follows `index`, not list position.

The colour thresholds live in the frontend (test_battery_view_js.py).
"""

from __future__ import annotations

from xbrain.p5_gateway.hmi import data_readers
from xbrain.p5_gateway.runtime import main_wiring

#: The real state/power body read from the robot on 2026-11-11 (slot 1 empty).
LIVE = {"soc_pct": 35, "present_count": 1, "list": [
    {"index": 0, "level_pct": 35, "voltage_v": 70.43, "temp_c": 33.2,
     "charging": False, "present": True, "serial": ""},
    {"index": 1, "level_pct": 0, "voltage_v": 0, "temp_c": -273,
     "charging": False, "present": False, "serial": ""}]}


def _packs(power):
    return {p["label"]: p for p in data_readers.battery_group(power)["packs"]}


def test_live_body_maps_slot0_to_a_and_slot1_to_b():
    packs = _packs(LIVE)
    assert packs["A"]["index"] == 0 and packs["A"]["state"] == "present"
    assert packs["A"]["level_pct"] == 35 and packs["A"]["temp_c"] == 33.2
    assert packs["B"]["index"] == 1


def test_empty_slot_carries_no_fake_numbers():
    # RED MUTANT: ignore `present` and copy the row -> B shows 0 % and -273 C.
    b = _packs(LIVE)["B"]
    assert b["state"] == "absent"
    assert b["level_pct"] is None and b["temp_c"] is None and b["voltage_v"] is None


def test_label_follows_index_not_list_order():
    # RED MUTANT: label by list position -> swapping the rows swaps A and B.
    swapped = {"list": list(reversed(LIVE["list"]))}
    packs = _packs(swapped)
    assert packs["A"]["state"] == "present" and packs["B"]["state"] == "absent"


def test_no_power_is_unknown_not_zero():
    # RED MUTANT: return available True with level 0 -> red "0%" on screen
    # while the chassis is simply not reporting.
    g = data_readers.battery_group(None)
    assert g["available"] is False
    assert all(p["state"] == "unknown" and p["level_pct"] is None for p in g["packs"])


def test_missing_slot_or_bad_present_is_unknown():
    g = _packs({"list": [{"index": 0, "present": "yes", "level_pct": 80}]})
    assert g["A"]["state"] == "unknown"
    assert g["B"]["state"] == "unknown"


def test_non_numeric_level_on_a_present_pack_is_none():
    g = _packs({"list": [{"index": 0, "present": True, "level_pct": "n/a",
                          "temp_c": True}]})
    assert g["A"]["state"] == "present"
    assert g["A"]["level_pct"] is None and g["A"]["temp_c"] is None


def test_snapshot_carries_the_battery_group():
    assert data_readers.build_snapshot(power=LIVE)["battery"]["available"] is True
    assert data_readers.build_snapshot()["battery"]["available"] is False


def test_stale_state_power_is_dropped():
    # RED MUTANT: return the cached power whatever its age -> a dead
    # chassis_relay leaves the last percentages on screen as if live.
    st = {"power": LIVE, "power_updated_ms": 1000}
    assert main_wiring._power_if_fresh(st, 1000 + main_wiring.POWER_STALE_AFTER_MS) is LIVE
    assert main_wiring._power_if_fresh(st, 1001 + main_wiring.POWER_STALE_AFTER_MS) is None
    assert main_wiring._power_if_fresh({"power": None}, 0) is None
