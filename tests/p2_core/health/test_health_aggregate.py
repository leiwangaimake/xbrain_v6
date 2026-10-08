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
    HealthAggregator,
    refresh_health,
    state_from_clock,
    state_from_link,
    state_from_perception,
    state_from_pose,
    state_from_power,
    state_from_robot,
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
    for _name, body in summary["items"].items():
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
    """The asymmetry with pose is deliberate: silence on state/robot does not
    say WHICH link is missing -- the chassis may be offline (quadruped then has
    nothing to publish on rt/chassis/state) or chassis_relay may not be
    forwarding. MUTATION: return FAIL and the summary asserts a chassis failure
    it has no evidence for -- and being a fatal item, it would report
    overall=fatal on every machine.

    *** 2026-09-27: the docstring used to justify UNKNOWN with "chassis_relay
    is a C++ process that does not exist in this build". The relay went up on
    2026-09-26, so that reason is gone while the VERDICT still holds for the
    other reason above. The detail string moved with it; asserted below so the
    text and the justification cannot drift apart again."""
    st, why = state_from_robot(None)
    assert st == HealthState.UNKNOWN
    # The operator reads this string while deciding where to look. It must not
    # name a process that is running -- that is a wrong-direction diagnosis,
    # not merely stale prose.
    assert "chassis_relay not wired" not in why
    assert "chassis offline or relay not forwarding" in why
    assert state_from_robot({"hes": True})[0] == HealthState.FAIL

    # conn, by the six contract wire names (11 S4.1). The first draft read a
    # field called chassis_link as a boolean; RobotState never carried it, so
    # the link-down branch could never fire -- the 2026-09-26 audit's
    # always-green find. Every branch below went red once against the old
    # body, which is the point of listing them one by one.
    assert state_from_robot({"conn": "connected"})[0] == HealthState.OK
    for bad in ("disconnected", "lost", "incompatible"):
        st, why = state_from_robot({"conn": bad})
        assert st == HealthState.FAIL and bad in why
    assert state_from_robot({"conn": "degraded"})[0] == HealthState.DEGRADED
    # A value outside the closed set must throw, never slide through as
    # "linked" (CLAUDE.md 3.5: no silent pass-through).
    import pytest as _pytest

    from xbrain.common.enums import ClosedSetViolation as _CSV
    with _pytest.raises(_CSV):
        state_from_robot({"conn": "ok"})   # quadruped's INTERNAL name, not wire

    # faults[]: the third leg of 11 S5.1C's criterion. fatal fails, degraded
    # degrades, warn alone stays OK; E_SAFETY_LINK_LOST fails regardless of
    # level because 11 S13.15 routes the estop-probe watchdog through it and
    # names the consequence ("P2 降级 hold").
    ok_w = {"conn": "connected", "faults": [{"code": "chs:0x1", "level": "warn"}]}
    assert state_from_robot(ok_w)[0] == HealthState.OK
    deg = {"conn": "connected",
           "faults": [{"code": "chs:0x2", "level": "degraded"}]}
    assert state_from_robot(deg)[0] == HealthState.DEGRADED
    fat = {"conn": "connected", "faults": [{"code": "chs:0x3", "level": "fatal"}]}
    assert state_from_robot(fat)[0] == HealthState.FAIL
    sll = {"conn": "connected",
           "faults": [{"code": "E_SAFETY_LINK_LOST", "level": "warn"}]}
    st, why = state_from_robot(sll)
    assert st == HealthState.FAIL and "safety link" in why


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
    no_power_st, no_power_why = state_from_power(None)
    assert no_power_st == HealthState.UNKNOWN
    # Same 2026-09-27 correction as the chassis item: the detail must not send
    # the operator hunting for a chassis_relay that is running (up since
    # 2026-09-26). state/power is CR-5's forward of rt/chassis/power, which
    # quadruped only produces once the chassis answers.
    assert "chassis_relay not wired" not in no_power_why
    assert "chassis offline or relay not forwarding" in no_power_why


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


# --- cam_rgbd from state/perception (R-2 option b, 14 S8.1) ------------------
#
# 14 S8.1's verdict for this item is "< 15 Hz 降级, 超时失效", so the five
# cases below are the five answers the derivation can give. They are written
# as one case per answer rather than one parametrised case, because the REASON
# each answer is correct differs and a shared body would have nowhere to say
# so -- and the reasons are the part that rots.

def test_refresh_threads_perception_rx_mono_into_age():
    """refresh_health is the seam between state_cache / rx_mono and
    state_from_perception, and the seam has the only arithmetic in the path:
    rx_age_s = now_mono_s - rx_mono["perception"]. The five tests above drive
    state_from_perception directly and never exercise that subtraction, so a
    bug there -- wrong key, sign flipped, rx_mono dropped -- would pass them
    all. This one feeds refresh_health the way the loop does.

    now_mono_s and rx_mono share ONE clock (both the P2 loop's time.monotonic),
    which is the point of recording rx_mono locally instead of reading the
    envelope's mono: CLK-C4 says a monotonic reading only means something in
    its own boot domain, and here both readings are p2's.

    mutant: in refresh_health, compute _perc_age from the envelope's mono
    instead of now - rx_mono -> the age is a cross-domain nonsense number and
    this test's fresh case stops being reliably OK.
    """
    cache = dict(_all_healthy_cache())
    cache["perception"] = {"fps_depth": 29.4}
    # Fresh: arrival == now, well inside max_age_s, rate above floor -> OK.
    agg = HealthAggregator()
    refresh_health(agg, cache, now_mono_s=100.0,
                   rx_mono={"perception": 100.0},
                   perception_min_fps_depth=15.0, perception_max_age_s=3.0)
    assert agg.states()["cam_rgbd"] is HealthState.OK
    # Stale: arrival 10 s ago, past the 3 s budget -> FAIL, even though the
    # cached body still reads 29.4 (a last-value cache never clears itself).
    agg = HealthAggregator()
    refresh_health(agg, cache, now_mono_s=110.0,
                   rx_mono={"perception": 100.0},
                   perception_min_fps_depth=15.0, perception_max_age_s=3.0)
    assert agg.states()["cam_rgbd"] is HealthState.FAIL
    # No rx_mono for perception -> age unknown -> cannot time out, falls to the
    # rate verdict. With a healthy rate that is OK; the point is it does not
    # crash on the missing key.
    agg = HealthAggregator()
    refresh_health(agg, cache, now_mono_s=110.0, rx_mono={},
                   perception_min_fps_depth=15.0, perception_max_age_s=3.0)
    assert agg.states()["cam_rgbd"] is HealthState.OK


def test_cam_rgbd_unknown_without_the_mirror():
    """No state/perception at all -> UNKNOWN, not FAIL.

    UNKNOWN and FAIL are different claims: silence on this key does not
    distinguish "p1 is not mirroring" from "the camera died", and calling it
    FAIL would forbid motion for a reason that is not evidence. Same asymmetry
    state_from_robot documents for state/robot.

    mutant: return FAIL on absence -> red.
    """
    assert state_from_perception(None)[0] is HealthState.UNKNOWN
    # A body without the field is the same situation, not a parse error: a
    # future producer that drops fps_depth must not read as healthy.
    assert state_from_perception({"fps_infer": 20.0})[0] is HealthState.UNKNOWN


def test_cam_rgbd_degraded_while_thresholds_unset():
    """Thresholds absent -> DEGRADED with the rate in the detail.

    CLAUDE.md 3.1 forbids a default on this item (it forbids motion), so the
    derivation must have an answer for "configured nothing yet". DEGRADED is
    that answer: the operator sees the rate, and nothing acts on a number
    nobody calibrated. OK would hide it; FAIL would strand the robot on a
    missing config key.

    mutant: default min_fps_depth to 15.0 in the signature -> red.
    """
    st, detail = state_from_perception({"fps_depth": 29.4})
    assert st is HealthState.DEGRADED
    assert "29.4" in detail and "not set" in detail


def test_cam_rgbd_stale_beats_a_healthy_rate():
    """Age is checked BEFORE the rate, and that order is the point.

    A stale body's fps_depth describes the past. Reading the rate first would
    report a healthy camera from a heartbeat that stopped arriving minutes
    ago -- which is exactly the failure mirror_status_to_gen's None branch is
    built to avoid on the publishing side, seen from the consuming side.

    mutant: move the staleness check below the rate check -> red (a stale body
    carrying 29.4 would answer OK).
    """
    st, detail = state_from_perception(
        {"fps_depth": 29.4}, rx_age_s=9.0, min_fps_depth=15.0, max_age_s=3.0)
    assert st is HealthState.FAIL
    assert "stale" in detail


def test_cam_rgbd_degraded_below_the_rate_floor():
    """< 15 Hz -> DEGRADED, verbatim from 14 S8.1.

    DEGRADED rather than FAIL: a low frame rate is a camera that still sees,
    and the item's own level (DEVICE/FATAL in items.py) decides what a FAIL
    would cost. 14 S8.1 says 降级 for this half and 失效 only for the timeout.

    mutant: return FAIL here -> red.
    """
    st, detail = state_from_perception(
        {"fps_depth": 9.0}, rx_age_s=0.5, min_fps_depth=15.0, max_age_s=3.0)
    assert st is HealthState.DEGRADED
    assert "9.0" in detail


def test_cam_rgbd_ok_when_fresh_and_fast_enough():
    """The positive case, without which the four above all pass on a
    derivation that never answers OK (CLAUDE.md 3.3 -- negative-only
    assertions are passed by an empty shell).

    mutant: make the comparison fps <= min_fps_depth and pass fps == floor
    -> this case still passes, so the boundary gets its own assertion below.
    """
    st, detail = state_from_perception(
        {"fps_depth": 29.4}, rx_age_s=0.5, min_fps_depth=15.0, max_age_s=3.0)
    assert st is HealthState.OK
    assert "29.4" in detail
    # Boundary: exactly at the floor is NOT below it. 14 S8.1 writes "< 15 Hz
    # 降级", so 15.0 itself is still acceptable -- an off-by-one here would
    # degrade a camera running exactly to spec.
    assert state_from_perception(
        {"fps_depth": 15.0}, rx_age_s=0.5, min_fps_depth=15.0,
        max_age_s=3.0)[0] is HealthState.OK
