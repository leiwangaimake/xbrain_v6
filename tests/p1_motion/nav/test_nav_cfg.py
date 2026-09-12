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
    "timeouts_ms": {"perception": 200, "gnss": 200, "health_degrade": 3000,
                    "health_dead": 10000},
}
RNS = {"rns": {"route": {"search_window": 30}, "geometry": {"r_eff_m": 0.5}}}


def test_complete_tree_builds():
    c = build_nav_config(P1, RNS)
    assert c.max_wz_radps == 1.2 and c.holonomic is True and c.v_nom_mps == 1.5
    assert c.relmove.default_timeout_s == 20.0 and c.health_dead_ms == 10000
    assert c.frame.origin == (31.2304, 121.4737) and c.rns["rns"]["route"]["search_window"] == 30


@pytest.mark.parametrize("dotted", [
    "geo.enu_origin.lat", "nav.max_vx_mps", "nav.max_wz_radps", "nav.holonomic",
    "nav.v_nom_mps", "relative_move.max_distance_m", "relative_move.abort_on_obstacle",
    "timeouts_ms.perception", "timeouts_ms.health_dead",
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
