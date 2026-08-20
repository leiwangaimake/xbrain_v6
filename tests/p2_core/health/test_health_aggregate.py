"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_health_aggregate.py
Brief: HealthSummary assembly + per-source derivations (11 S5.1, batch 8)

Description:
The health/summary P2 publishes at 1 Hz, and the derivations that fill it.

The case that carries this batch is test_unwired_sources_stay_unknown: six of
the nineteen items have no producer in this build, and the summary must say so
rather than report them ok. A health summary that reads `ok` on a machine whose
monitors are all silent is worse than no summary -- it is a confident wrong
answer, and P3 admits recordings on the strength of it.

The derivations are tested one source at a time because that is where a wrong
reading becomes a wrong permission: state_from_pose deciding rtk is ok on a
float fix would let a route be recorded at decimetre error.
"""
from __future__ import annotations

import pytest

from xbrain.p2_core.health.aggregate import (
    HealthAggregator, refresh_health, state_from_clock, state_from_link,
    state_from_pose, state_from_power, state_from_robot,
)
from xbrain.p2_core.health.factor import FactorConfig
from xbrain.p2_core.health.items import ITEMS, HealthState

pytestmark = pytest.mark.no_device

_CFG = FactorConfig(fatal_degraded=0.3, degraded_fail=0.5,
                    degraded_degraded=0.7, unknown=0.5)

_GOOD_POSE = {"lat": 34.697, "lon": 135.505, "fix_type": "rtk_fixed",
              "heading_valid": True, "heading_level": 1,
              "heading_source": "dual_antenna"}


def _all_healthy_cache():
    """Every source present and good -- including the two chassis_relay keys a
    stub would have to supply on a machine with no chassis."""
    return {"pose": _GOOD_POSE, "clock": {"sync": True, "source": "rtk"},
            "robot": {"hes": False, "chassis_link": True, "estop_path": "up"},
            "power": {"soc_pct": 82.0}, "link": {"level": "L0"}}


# ------------------------------------------------------------- assembly ----

def test_summary_shape_matches_the_contract():
    agg = HealthAggregator()
    summary = agg.build_summary(_CFG)
    assert summary["overall"] in ("ok", "degraded", "fatal")
    assert set(summary["items"]) == set(ITEMS)
    for name, body in summary["items"].items():
        assert body["state"] in ("ok", "warn", "degraded", "fail", "unknown")
        assert body["level"] in ("fatal", "degraded", "warn")
        assert body["kind"] in ("device", "cap")


def test_unwired_sources_stay_unknown_and_say_what_is_missing():
    """*** The batch's load-bearing case.

    MUTATION: initialise the entries as OK instead of UNKNOWN (the tempting
    "nothing has reported a problem" reading) -- allow_motion then becomes true
    on a machine with no camera, no chassis and no battery monitor, and P3
    admits a recording on that basis.
    """
    agg = HealthAggregator()
    refresh_health(agg, {}, now_mono_s=100.0)
    summary = agg.build_summary(_CFG)
    for name in ("cam_rgbd", "lidar", "compute", "gpu", "storage"):
        assert summary["items"][name]["state"] == "unknown", name
        # And each names the missing thing, so a refused recording can say why.
        assert summary["items"][name]["detail"]
    assert summary["allow_motion"] is False
    assert summary["overall"] != "ok"


def test_since_mono_moves_only_on_a_state_change():
    """11 S5.1 since_mono drives the 14 S8.4 hysteresis and the HMI's
    "degraded for 4m12s". MUTATION: stamp it every observation -- a source
    publishing at 1 Hz makes a four-minute degradation read as one second old,
    and recover_hold_s never elapses."""
    agg = HealthAggregator()
    agg.observe("mic", HealthState.FAIL, now_mono_s=100.0)
    agg.observe("mic", HealthState.FAIL, now_mono_s=140.0)
    assert agg.entries["mic"].since_mono == 100.0
    agg.observe("mic", HealthState.OK, now_mono_s=150.0)
    assert agg.entries["mic"].since_mono == 150.0


def test_observe_rejects_an_unknown_item():
    """A producer and the S5.1A table disagreeing must surface. MUTATION:
    setdefault the entry instead -- an item nobody reviewed joins the summary
    and the aggregate silently starts weighting it."""
    with pytest.raises(KeyError):
        HealthAggregator().observe("teleporter", HealthState.OK,
                                   now_mono_s=1.0)


def test_overall_is_not_ok_while_anything_is_unknown():
    """MUTATION: roll unknown up as ok -- the summary reads `ok` on a machine
    whose monitors are all silent, which is the one answer it must never give."""
    agg = HealthAggregator()
    refresh_health(agg, _all_healthy_cache(), now_mono_s=1.0)
    # cam_rgbd et al are still unknown (no perception process).
    assert agg.build_summary(_CFG)["overall"] == "degraded"


# ---------------------------------------------------------- derivations ----

def test_rtk_and_heading_from_pose():
    rtk, heading = state_from_pose(_GOOD_POSE)
    assert rtk[0] == HealthState.OK and heading[0] == HealthState.OK
    # A float fix is a real degradation, not a failure: U34 keeps a teleop
    # escape on it. MUTATION: map rtk_float to OK and a route gets recorded at
    # decimetre error and then driven as if it were surveyed.
    float_fix, _h = state_from_pose({**_GOOD_POSE, "fix_type": "rtk_float"})
    assert float_fix[0] == HealthState.DEGRADED
    # COG heading is degraded, not ok (H-1 / U34).
    _r, cog = state_from_pose({**_GOOD_POSE, "heading_level": 2,
                               "heading_source": "cog"})
    assert cog[0] == HealthState.DEGRADED
    _r, invalid = state_from_pose({**_GOOD_POSE, "heading_valid": False})
    assert invalid[0] == HealthState.FAIL


def test_absent_pose_is_fail_not_unknown():
    """P1 publishes pose at 10 Hz whenever it runs, so its absence is a
    determinate answer -- the positioning chain is down. MUTATION: return
    UNKNOWN and a dead P1 looks like an unwired one."""
    rtk, heading = state_from_pose(None)
    assert rtk[0] == HealthState.FAIL and heading[0] == HealthState.FAIL


def test_absent_chassis_is_unknown_not_fail():
    """The asymmetry with pose is deliberate: chassis_relay is a C++ process
    that does not exist in this build, so silence means "not wired", not "the
    chassis dropped". MUTATION: return FAIL and the summary asserts a chassis
    failure it has no evidence for -- and being a fatal item, it would report
    overall=fatal on every machine."""
    assert state_from_robot(None)[0] == HealthState.UNKNOWN
    assert state_from_robot({"hes": False, "chassis_link": True})[0] == \
        HealthState.OK
    assert state_from_robot({"hes": True})[0] == HealthState.FAIL


def test_clock_absent_is_fail_per_clk_a3():
    assert state_from_clock(None)[0] == HealthState.FAIL
    assert state_from_clock({"sync": False})[0] == HealthState.FAIL
    assert state_from_clock({"sync": True, "source": "rtk"})[0] == \
        HealthState.OK


def test_battery_without_a_calibrated_threshold_is_not_failed():
    """*** critical_soc_pct is U-BIT-1, awaiting the operator, and battery is
    one of the five items that forbid motion.

    MUTATION: pick a plausible default (say 15%) -- too low and the robot
    strands itself in the field, too high and it refuses to work, and either
    way nobody chose the number. Without it the SoC is reported and the state
    stays degraded: visible, and not acted upon.
    """
    state, detail = state_from_power({"soc_pct": 8.0})
    assert state == HealthState.DEGRADED and "8.0" in detail
    # With a threshold supplied, it does act.
    assert state_from_power({"soc_pct": 8.0}, critical_soc_pct=15.0)[0] == \
        HealthState.FAIL
    assert state_from_power({"soc_pct": 80.0}, critical_soc_pct=15.0)[0] == \
        HealthState.OK
    assert state_from_power(None)[0] == HealthState.UNKNOWN


def test_network_degrades_without_blocking():
    """TSK-20 / U36: losing the cloud does not stop the current task."""
    assert state_from_link({"level": "L0"})[0] == HealthState.OK
    assert state_from_link({"level": "L3"})[0] == HealthState.DEGRADED
    assert state_from_link(None)[0] == HealthState.UNKNOWN


# ------------------------------------------------------------- refresh -----

def test_refresh_maps_device_liveness_but_not_unsampled_devices():
    """A device the bridge has never sampled must not become ok. MUTATION: read
    DeviceLivenessMonitor.reported_up directly (it starts True by design, so
    no spurious online event fires at boot) -- every unplumbed device then
    reports ok."""
    agg = HealthAggregator()
    refresh_health(agg, {}, now_mono_s=1.0,
                   device_states={"mic": True, "ptz": False,
                                  "payload_light": None})
    states = agg.states()
    assert states["mic"] == HealthState.OK
    assert states["ptz"] == HealthState.FAIL
    assert states["payload_light"] == HealthState.UNKNOWN


def test_a_source_that_stops_publishing_ages_into_a_worse_state():
    """The derivations run EVERY tick, not on change. MUTATION: only refresh on
    change and the last good reading freezes forever -- a dead P1 leaves rtk
    reading ok indefinitely."""
    agg = HealthAggregator()
    cache = _all_healthy_cache()
    refresh_health(agg, cache, now_mono_s=1.0)
    assert agg.states()["rtk"] == HealthState.OK
    cache.pop("pose")
    refresh_health(agg, cache, now_mono_s=2.0)
    assert agg.states()["rtk"] == HealthState.FAIL


def test_with_every_source_healthy_motion_is_still_refused_without_a_camera():
    """The honest end state of this build, asserted so it cannot drift silently.

    Everything a stub can supply is good -- pose, clock, chassis, power, link --
    and allow_motion is still false, because cam_rgbd has no producer and
    14 S8.3 admits no profile without it. When perception lands, this case is
    the one that has to be updated, deliberately.
    """
    agg = HealthAggregator()
    refresh_health(agg, _all_healthy_cache(), now_mono_s=1.0)
    summary = agg.build_summary(_CFG)
    assert summary["allow_motion"] is False
    assert summary["max_profile"] == "none"
    assert summary["items"]["cam_rgbd"]["state"] == "unknown"
    # But the four wired items ARE ok, so the refusal names one missing thing
    # rather than everything.
    for wired in ("rtk", "heading", "clock", "chassis", "battery", "network"):
        assert summary["items"][wired]["state"] != "unknown", wired
