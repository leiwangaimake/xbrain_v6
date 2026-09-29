"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_nav_cfg.py
Brief: build_nav_config -- every loop parameter refuses null / missing and names the key

Description:
The one property that matters: a null anywhere (max_wz_radps is null in
production until V-01) must raise naming the key, never become a NavConfig
with None inside. A complete tree builds; each leaf is then nulled in turn
and must be refused by name.
"""
from __future__ import annotations

import copy

import pytest

from xbrain.p1_motion.runtime.nav_cfg import NavConfigError, build_nav_config

pytestmark = pytest.mark.no_device

P1 = {
    "geo": {"enu_origin": {"lat": 31.2304, "lon": 121.4737, "alt": 4.0}},
    "nav": {"max_vx_mps": 2.0, "max_wz_radps": 1.2, "holonomic": True, "v_nom_mps": 1.5,
            "v_obstacle_avoid_mps": 0.5},
    "relative_move": {"max_distance_m": 20.0, "max_yaw_rad": 6.2832,
                      "pure_rotation_eps_m": 0.02, "default_timeout_s": 20.0,
                      "abort_on_obstacle": True},
    "timeouts_ms": {"gnss": 200, "health_degrade": 3000, "health_dead": 10000},
    # 12 S7 clip constants (fence/clip.py): shared truth refs, resolved at freeze.
    "fence": {"brake_k": 1.5, "brake_a_mps2": 2.5, "t_lat_s": 0.4, "soft_margin_min_m": 2.0,
              "predict_dt_s": 0.45, "margin_by_fix": {"rtk_fixed": 0.3, "rtk_float": 1.0},
              "projection_iters": 3},
    "speed_gate": {"hysteresis": {"speed_up_hold_s": 3.0, "d_up_margin_m": 0.5}},
    # 12 S12 rotation_clearance (step 6b). The three zeros are the STRICTEST
    # settings the section gives, not uncalibrated placeholders: any occupied
    # cell refuses, unknown cells get no tolerance, and inner radius 0 means
    # the whole disc is judged.
    "rotation_clearance": {"data_source": "lidar_grid", "margin_rot_m": 1.0,
                           "r_self_mask_m": 0.0, "rot_occ_max": 0,
                           "rot_unknown_max_cells": 0,
                           "rot_unknown_ratio_max": 0.5,
                           "grid_age_max_ms": 200, "recheck_ticks": 3,
                           "wz_eps_radps": 0.05, "k_rot": 0.5,
                           "r_robot_fallback_m": 0.60, "ped_speed_mps": 1.5,
                           "allow_visual_override": False},
}
# No inflation section, exactly like configs/rns.yaml: r_robot has no key
# anywhere, so RCG-1 has nothing to evaluate and the permit refuses. That is
# 12 S6A.3.3's specified behaviour for an uncalibrated body, and the test below
# pins that nav_cfg reports None rather than borrowing r_eff_m for it.
RNS = {"rns": {"route": {"search_window": 30}, "geometry": {"r_eff_m": 0.5}}}


def test_complete_tree_builds():
    c = build_nav_config(P1, RNS)
    assert c.max_wz_radps == 1.2 and c.holonomic is True and c.v_nom_mps == 1.5
    assert c.relmove.default_timeout_s == 20.0 and c.health_dead_ms == 10000
    assert c.frame.origin == (31.2304, 121.4737) and c.rns["rns"]["route"]["search_window"] == 30
    # the fence constants: patrol tier as v_profile_max, obstacle_avoid tier as
    # the degraded teleop cap (11 S3.2.1 / U54), the table by fix type.
    assert c.fence.brake_k == 1.5 and c.fence.v_profile_max_mps == 1.5
    assert c.fence.teleop_cap_degraded_mps == 0.5 and c.fence.margin_by_fix["rtk_float"] == 1.0
    assert c.speed_up_hold_ms == 3000 and c.d_up_margin_m == 0.5     # 12 S6.7 T_up in ms
    # 12 S12 rotation_clearance -> RotationLimits, and the two values that are
    # deliberately NOT read from a file.
    assert c.rot_limits.margin_rot_m == 1.0 and c.rot_limits.k_rot == 0.5
    # None because no loaded tree carries the key. 12 S12 landing plan (2)
    # rules this state: the LIMIT branch degrades to a veto plus one fault,
    # rather than clamping to a guessed rate.
    assert c.rot_limits.wz_blind_radps is None
    # None because configs/rns.yaml has no inflation section. r_eff_m is NOT
    # borrowed for it -- 12 S6A.4.1 iron rule (1) forbids r_eff reaching
    # r_check, and doing so here would let an unmeasured body pass RCG-1.
    assert c.r_robot_m is None


def test_r_robot_is_read_only_from_the_rns_inflation_key():
    """12 S12: r_robot's single definition is the RNS inflation section.

    When that key does land, nav_cfg must pick it up from there -- not from a
    P1-private copy (the freeze line's assertion B would flag one as a
    suspected duplicate) and not from geometry.r_eff_m next door.
    """
    rns = {"rns": {"route": {"search_window": 30},
                   "geometry": {"r_eff_m": 0.5},
                   "inflation": {"r_robot_m": 0.482}}}
    assert build_nav_config(P1, rns).r_robot_m == pytest.approx(0.482)


@pytest.mark.parametrize("key", ["rot_occ_max", "k_rot", "allow_visual_override"])
def test_missing_rotation_key_names_itself(key):
    """A missing rotation_clearance leaf refuses by name (CLAUDE.md 3.1).

    Parametrised over one of each kind -- a count, a coefficient and a flag --
    because a walker that special-cases falsy values would let 0 and False
    through as "missing" or as "present" depending on which mistake it made.
    """
    import copy as _copy
    tree = _copy.deepcopy(P1)
    del tree["rotation_clearance"][key]
    with pytest.raises(NavConfigError, match=key):
        build_nav_config(tree, RNS)


@pytest.mark.parametrize("dotted", [
    "geo.enu_origin.lat", "nav.max_vx_mps", "nav.max_wz_radps", "nav.holonomic",
    "nav.v_nom_mps", "relative_move.max_distance_m", "relative_move.abort_on_obstacle",
    "timeouts_ms.gnss", "timeouts_ms.health_dead",
    "fence.brake_k", "fence.t_lat_s", "fence.margin_by_fix.rtk_fixed", "fence.projection_iters",
    "speed_gate.hysteresis.speed_up_hold_s", "speed_gate.hysteresis.d_up_margin_m",
])
def test_null_leaf_refused_by_name(dotted):
    """mutant: treat a missing leaf as None instead of raising -> red."""
    t = copy.deepcopy(P1)
    cur = t
    parts = dotted.split(".")
    for p in parts[:-1]:
        cur = cur[p]
    cur[parts[-1]] = None
    with pytest.raises(NavConfigError) as ei:
        build_nav_config(t, RNS)
    assert dotted.split(".")[-1] in str(ei.value)
    # the null wording is the operator's cue ("fill it"), distinct from the
    # off-type wording; a walker that returned None would lose it.
    assert "null" in str(ei.value) or "enu_origin" in dotted
    del cur[parts[-1]]
    with pytest.raises(NavConfigError) as ei:
        build_nav_config(t, RNS)
    assert "missing" in str(ei.value) or "enu_origin" in dotted


@pytest.mark.parametrize("dotted, bad", [
    ("nav.max_wz_radps", 0.0), ("nav.max_wz_radps", True), ("nav.holonomic", 1),
    ("timeouts_ms.gnss", 1.5), ("relative_move.max_distance_m", -1.0),
])
def test_off_type_refused(dotted, bad):
    t = copy.deepcopy(P1)
    a, b = dotted.split(".")
    t[a][b] = bad
    with pytest.raises(NavConfigError):
        build_nav_config(t, RNS)


def test_missing_rns_section_refused():
    with pytest.raises(NavConfigError):
        build_nav_config(P1, {"route": {}})
    with pytest.raises(NavConfigError):
        build_nav_config(P1, {"rns": {"route": {}, "geometry": {"r_eff_m": None}}})
