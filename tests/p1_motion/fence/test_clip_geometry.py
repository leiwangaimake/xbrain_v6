"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_clip_geometry.py
Brief: fence/clip.py primitives -- signed edge distance + normal, brake inverse, half-space projection, compile

Description:
The pieces evaluate() is built from, each against a hand-computable case:
the FE-2 analytic distance (inside / outside / vertex / on-boundary, allow
and forbid), the 11 S9A.6 (3)/(4) numbers (the U54 table values 2.40 /
0.38 / 0.90 and the corridor row 1.14 m -> 1.19 m/s), the 12 S7.2 /
11 S9A.8 projection incl. the concave (degenerate) corner, and the
ENU compile of a HeldFenceSet (warning dropped, forbid keep_in False,
soft_margin inheritance, the wire soft_margin validation).
"""
from __future__ import annotations

import math

import pytest

from xbrain.common.fence.geom import fence_set_crc32
from xbrain.p1_motion.fence.clip import (CompiledPolygon, FenceClipError, FenceConstants,
                                         boundary_hit, compile_fence, d_stop_m,
                                         margin_soft_eff_m, project_halfspaces,
                                         v_fence_mps)
from xbrain.p1_motion.fence.fence_set import (FenceSetError, HeldFenceSet, HeldPolygon,
                                              compile_fence_set)
from xbrain.p1_motion.path.local_frame import LocalFrame

pytestmark = pytest.mark.no_device

CFG = FenceConstants(brake_k=1.5, brake_a_mps2=2.5, t_lat_s=0.4, soft_margin_min_m=2.0,
                     predict_dt_s=0.45,
                     margin_by_fix={"rtk_fixed": 0.3, "rtk_float": 1.0, "dgps": 1.5, "single": 3.0},
                     projection_iters=3, v_profile_max_mps=2.0, teleop_cap_degraded_mps=0.5)
SQ = CompiledPolygon("p-outer", "allow", "outer", True, True,
                     ((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)), None)
FB = CompiledPolygon("p-fuel", "forbid", "fuel", True, False,
                     ((20.0, 0.0), (30.0, 0.0), (30.0, 10.0), (20.0, 10.0)), None)


def _close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


# --- signed distance ---------------------------------------------------------

def test_inside_allow_is_positive_with_normal_toward_the_edge():
    hit = boundary_hit(5.0, 3.0, SQ)
    assert hit.d_nom_m == pytest.approx(3.0)
    assert _close(hit.normal, (0.0, -1.0))


def test_outside_allow_is_negative_and_breached_normal_points_further_out():
    """mutant: same normal rule for d < 0 as for d > 0 -> the retreat is
    removed and the escape kept -> red."""
    hit = boundary_hit(12.0, 5.0, SQ)
    assert hit.d_nom_m == pytest.approx(-2.0)
    assert _close(hit.normal, (1.0, 0.0))


def test_vertex_is_the_nearest_feature_outside_a_corner():
    hit = boundary_hit(-1.0, -1.0, SQ)
    assert hit.d_nom_m == pytest.approx(-math.sqrt(2.0))
    assert _close(hit.normal, (-1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0)))


def test_on_the_boundary_uses_the_edge_normal_oriented_outward():
    hit = boundary_hit(10.0, 5.0, SQ)
    assert abs(hit.d_nom_m) < 1e-9
    assert _close(hit.normal, (1.0, 0.0))


def test_forbid_polygon_is_positive_outside_and_negative_inside():
    out = boundary_hit(15.0, 5.0, FB)
    assert out.d_nom_m == pytest.approx(5.0) and _close(out.normal, (1.0, 0.0))
    inn = boundary_hit(25.0, 3.0, FB)
    assert inn.d_nom_m == pytest.approx(-3.0) and _close(inn.normal, (0.0, 1.0))


# --- physics -----------------------------------------------------------------

def test_d_stop_matches_the_11_s9a6_u54_table():
    assert d_stop_m(2.0, CFG) == pytest.approx(2.40)
    assert d_stop_m(1.0, CFG) == pytest.approx(0.90)
    assert d_stop_m(0.5, CFG) == pytest.approx(0.375)
    assert margin_soft_eff_m(CFG) == pytest.approx(2.40)          # max(2.0, 2.40)
    assert margin_soft_eff_m(CFG, 3.0) == pytest.approx(3.0)      # declared floor widens


def test_v_fence_is_the_brake_inverse():
    """mutant: drop the '- k t' term -> ~1 m/s allowed at the boundary -> red."""
    for v in (0.2, 0.5, 1.0, 2.0):
        assert v_fence_mps(d_stop_m(v, CFG), CFG) == pytest.approx(v)
    assert v_fence_mps(0.0, CFG) == 0.0 and v_fence_mps(-1.0, CFG) == 0.0
    assert v_fence_mps(1.14, CFG) == pytest.approx(1.19, abs=0.005)   # corridor row W=4 m


def test_constants_refuse_nulls_and_missing_fix_rows():
    with pytest.raises(FenceClipError):
        FenceConstants(brake_k=1.5, brake_a_mps2=None, t_lat_s=0.4, soft_margin_min_m=2.0,
                       predict_dt_s=0.45, margin_by_fix={"rtk_fixed": 0.3, "rtk_float": 1.0},
                       projection_iters=3, v_profile_max_mps=2.0, teleop_cap_degraded_mps=0.5)
    with pytest.raises(FenceClipError):
        FenceConstants(brake_k=1.5, brake_a_mps2=2.5, t_lat_s=0.4, soft_margin_min_m=2.0,
                       predict_dt_s=0.45, margin_by_fix={"rtk_fixed": 0.3},
                       projection_iters=3, v_profile_max_mps=2.0, teleop_cap_degraded_mps=0.5)
    with pytest.raises(FenceClipError):
        FenceConstants(brake_k=1.5, brake_a_mps2=2.5, t_lat_s=0.4, soft_margin_min_m=2.0,
                       predict_dt_s=0.45, margin_by_fix={"rtk_fixed": 0.3, "rtk_float": 1.0},
                       projection_iters=0, v_profile_max_mps=2.0, teleop_cap_degraded_mps=0.5)


# --- projection --------------------------------------------------------------

def test_zero_cap_removes_only_the_outward_component():
    assert _close(project_halfspaces((1.0, 0.0), [((1.0, 0.0), 0.0)], 3)[0], (0.0, 0.0))
    assert _close(project_halfspaces((1.0, 1.0), [((1.0, 0.0), 0.0)], 3)[0], (0.0, 1.0))
    v, deg = project_halfspaces((-1.0, 0.5), [((1.0, 0.0), 0.0)], 3)
    assert _close(v, (-1.0, 0.5)) and not deg


def test_soft_cap_limits_the_approach_component():
    assert _close(project_halfspaces((2.0, 1.0), [((1.0, 0.0), 0.5)], 3)[0], (0.5, 1.0))


def test_two_orthogonal_constraints_converge_without_degeneracy():
    v, deg = project_halfspaces((1.0, 1.0), [((1.0, 0.0), 0.0), ((0.0, 1.0), 0.0)], 3)
    assert _close(v, (0.0, 0.0)) and not deg


def test_concave_corner_is_degenerate_and_zero():
    """mutant: skip the residual check -> a dead end leaks outward speed -> red."""
    n2 = (-math.cos(math.radians(30.0)), math.sin(math.radians(30.0)))
    v, deg = project_halfspaces((0.0, 1.0), [((1.0, 0.0), 0.0), (n2, 0.0)], 3)
    assert deg and v == (0.0, 0.0)


# --- compile -----------------------------------------------------------------

def _held(**kw):
    outer = HeldPolygon("p-outer", "allow", "outer", True,
                        ((30.999, 120.999), (31.001, 120.999), (31.001, 121.001), (30.999, 121.001)),
                        kw.get("outer_soft"))
    fuel = HeldPolygon("p-fuel", "forbid", "fuel", True,
                       ((31.0002, 121.0002), (31.0004, 121.0002), (31.0004, 121.0004)), 3.0)
    warn = HeldPolygon("z-gate", "warning", "gate", False,
                       ((31.0, 121.0), (31.0001, 121.0), (31.0001, 121.0001)), None)
    return HeldFenceSet("fs", 3, "00000000", (outer, fuel, warn), kw.get("set_soft"))


def test_compile_drops_warning_and_projects_to_enu():
    cf = compile_fence(_held(set_soft=2.5), LocalFrame(31.0, 121.0))
    assert cf.rev == 3 and [p.poly_id for p in cf.polygons] == ["p-outer", "p-fuel"]
    outer, fuel = cf.polygons
    assert outer.keep_in and not fuel.keep_in
    assert outer.soft_margin_min_m == 2.5 and fuel.soft_margin_min_m == 3.0   # inheritance / override
    x, y = outer.xy[1]                                # (lat 31.001, lon 120.999)
    assert x < 0.0 and y > 0.0 and abs(y - 111.32) < 0.5   # 0.001 deg north = 111.32 m


def test_wire_soft_margin_is_carried_and_validated():
    verts = [{"lat": 30.999, "lon": 120.999}, {"lat": 31.001, "lon": 120.999},
             {"lat": 31.001, "lon": 121.001}]
    polys = [{"poly_id": "p-outer", "role": "allow", "winding": "ccw", "hard_enforce": True,
              "vertices": verts, "soft_margin_min_m": 2.5}]
    wire = {"fence_set_id": "fs", "rev": 1, "polygons": polys, "soft_margin_min_m": 2.0,
            "crc32": fence_set_crc32("fs", 1, polys)}
    held = compile_fence_set(wire)
    assert held.soft_margin_min_m == 2.0 and held.polygons[0].soft_margin_min_m == 2.5
    bad = dict(wire, soft_margin_min_m=-1.0)
    with pytest.raises(FenceSetError):
        compile_fence_set(bad)
