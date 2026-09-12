"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_health_factor_publish.py
Brief: cmd/motion/factor body (11 S3.6) -- shape, wire closed set, reason rules, P1 round trip, wiring

Description:
Five things about P2's grant to P1 that must not drift:
  * the body has exactly the 11 S3.6 fields with values from compute_factor;
  * max_profile on the wire is always one of P1's two accepted values --
    compute_factor's internal "none" must never leave P2 (P1 rejects the whole
    message and keeps its previous value, so a FATAL failure would only stop
    the robot through the 10 s dead-ladder; this test found that);
  * `reason` names the dominating item as <item>_<state> (blocking FAIL item,
    else the camera when no profile is admissible, else the min-factor item,
    else none);
  * every body the publisher can produce parses through P1's own
    HealthFactorSlot parser -- the two processes share the contract, not a
    copy of it;
  * main_wiring declares the publisher and puts the body in the same 1 Hz
    block as health/summary (text check, as test_p2_estop_wiring does).
"""
from __future__ import annotations

import pathlib

import pytest

from xbrain.p1_motion.nav.health_factor import MAX_PROFILES, parse_health_factor
from xbrain.p2_core.health.factor import (DETAIL_REF, FactorConfig,
                                          build_health_factor, compute_factor,
                                          dominant_reason)
from xbrain.p2_core.health.items import ITEMS, HealthState

pytestmark = pytest.mark.no_device

CFG = FactorConfig(fatal_degraded=0.3, degraded_fail=0.5, degraded_degraded=0.7, unknown=0.5)
ALL_OK = {i: HealthState.OK for i in ITEMS}


def test_body_has_the_11_s36_fields_and_p1_parses_it():
    body = build_health_factor(ALL_OK, CFG)
    assert set(body) == {"speed_factor", "max_profile", "allow_motion", "reason", "detail_ref"}
    assert body == {"speed_factor": 1.0, "max_profile": "patrol", "allow_motion": True,
                    "reason": "none", "detail_ref": DETAIL_REF}
    assert parse_health_factor(body) == (1.0, True, "patrol")


def test_reason_names_the_blocking_item():
    """mutant: always 'none' -> red."""
    body = build_health_factor(dict(ALL_OK, chassis=HealthState.FAIL), CFG)
    assert body["reason"] == "chassis_fail"
    assert parse_health_factor(body) == (0.0, False, "obstacle_avoid")


def test_camera_unknown_blocks_and_is_the_reason():
    """The production case today (no camera-health source, 14 S8.1 / NEXT):
    cam_rgbd unknown -> no admissible profile -> allow_motion false, and the
    body says so instead of pointing at a healthy item."""
    body = build_health_factor(dict(ALL_OK, cam_rgbd=HealthState.UNKNOWN), CFG)
    assert body["allow_motion"] is False and body["reason"] == "cam_rgbd_unknown"
    assert parse_health_factor(body)[1] is False


def test_blocked_body_never_carries_none_on_the_wire():
    """mutant: pass compute_factor's max_profile through -> P1 raises -> red.
    The internal value IS 'none' (14 S8.3) -- the wire must translate it."""
    st = dict(ALL_OK, cam_rgbd=HealthState.FAIL)
    assert compute_factor(st, CFG).max_profile == "none"
    body = build_health_factor(st, CFG)
    assert body["max_profile"] == "obstacle_avoid" and body["max_profile"] in MAX_PROFILES
    assert parse_health_factor(body) == (0.0, False, "obstacle_avoid")


def test_reason_is_the_min_factor_item_when_slowed():
    """mutant: pick the max-factor item instead of the min -> red."""
    st = dict(ALL_OK, lidar=HealthState.DEGRADED, battery=HealthState.UNKNOWN)
    out = compute_factor(st, CFG)
    assert out.allow_motion and out.speed_factor == pytest.approx(0.5)     # battery unknown 0.5 < lidar 0.7
    assert dominant_reason(st, CFG, out) == "battery_unknown"
    assert build_health_factor(dict(ALL_OK, lidar=HealthState.DEGRADED), CFG)["reason"] == "lidar_degraded"


def test_every_single_item_state_round_trips_through_p1():
    """Every body P2 can produce from one item off nominal (and the all-unknown
    start-up aggregate) must parse in P1 and agree with compute_factor. This is
    the class of defect the wire 'none' was: each side correct on its own."""
    cases = [{}] + [dict(ALL_OK, **{item: state}) for item in ITEMS for state in HealthState]
    for st in cases:
        out = compute_factor(st, CFG)
        body = build_health_factor(st, CFG)
        sf, am, mp = parse_health_factor(body)          # raises on any off-set value
        assert (sf, am) == (pytest.approx(round(out.speed_factor, 3)), out.allow_motion)
        assert mp == (out.max_profile if am else "obstacle_avoid")
        assert body["detail_ref"] == DETAIL_REF and isinstance(body["reason"], str)
        assert (body["reason"] == "none") == (am and sf == 1.0)


def test_main_wiring_publishes_the_factor_in_the_health_block():
    """mutant: drop factor_pub.put -> red."""
    src = (pathlib.Path(__file__).resolve().parents[3] / "xbrain" / "p2_core" / "runtime"
           / "main_wiring.py").read_text(encoding="utf-8")
    assert 'CMD_MOTION_FACTOR_TOPIC = "cmd/motion/factor"' in src
    assert "factor_pub = gen.declare_publisher(CMD_MOTION_FACTOR_TOPIC)" in src
    assert src.count("factor_pub.put(") == 1
    health_block = src.index("if now - last_health >= HEALTH_PUBLISH_PERIOD_S:")
    assert src.index("factor_pub.put(") > health_block
    assert "build_health_factor(health_agg.states(), factor_cfg)" in src
