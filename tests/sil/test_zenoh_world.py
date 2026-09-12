"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_zenoh_world.py
Brief: the SIL's Zenoh bodies must parse through p1's own intakes (the contract between stub and p1)

Description:
zenoh_world.py is what the E2E robot "sees"; if its bodies drift from what
p1 parses, the E2E fails silently as "no perception" or "route rejected".
So every body is pushed through the SAME code p1 runs: three_keys parsers
for the perception keys, RouteAssembler for the route (WGS84 round trip
within a centimetre), translate_relative_move for the goto displacement,
assemble_pose for the gnss pair, parse_health_factor for the grant.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts" / "sil"))

from sil_world import SilWorld  # noqa: E402
from zenoh_world import (clear_body, factor_body, gnss_bodies,  # noqa: E402
                         perception_bodies, relmove_body, route_body,
                         world_to_body)
from xbrain.p1_motion.nav.health_factor import parse_health_factor  # noqa: E402
from xbrain.p1_motion.nav.relmove_intake import RelMoveLimits, translate_relative_move  # noqa: E402
from xbrain.p1_motion.nav.route_intake import RouteAssembler, RouteClear, RouteSet  # noqa: E402
from xbrain.p1_motion.path.gnss_pose import assemble_pose  # noqa: E402
from xbrain.p1_motion.path.local_frame import LocalFrame  # noqa: E402
from xbrain.p1_motion.perception_src.three_keys import (parse_objects,  # noqa: E402
                                                        parse_profile, parse_status)

pytestmark = pytest.mark.no_device

FRAME = LocalFrame(31.2304, 121.4737)


def _world():
    w = SilWorld()
    w.add_obstacle("person", 3.0, 0.5)
    w.add_obstacle("wall", 5.0, -2.0, 5.0, 2.0, 0.5)
    w.rx, w.ry, w.ryaw = 0.0, 0.0, 0.0
    return w


def test_perception_bodies_parse_through_three_keys():
    snap = _world().synth_snapshot(5000)
    prof, objs, stat = perception_bodies(snap)
    # exactly the wire path: JSON out, JSON in (tuples must already be lists)
    prof, objs, stat = (json.loads(json.dumps(b)) for b in (prof, objs, stat))
    assert isinstance(perception_bodies(snap)[0]["d_free"], list)
    p = parse_profile(prof)
    assert p.n_bins == 181 and p.t_capture_mono_ms == 5000
    o = parse_objects(objs)
    assert len(o.objects) == 1 and o.objects[0].class_name == "person"
    s = parse_status(stat)
    assert s.extrinsic_calibrated is True


def test_route_round_trips_through_the_p1_assembler():
    """mutant: swap lat/lon in route_body -> metres off by kilometres -> red."""
    pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (-5.0, 12.5)]
    body = route_body(FRAME, pts, cmd_id="rg-1", route_rev=3, arrive_radius_m=0.8)
    rs = RouteAssembler(FRAME).accept(body)
    assert isinstance(rs, RouteSet) and rs.route_rev == 3 and rs.waypoint_total == 4
    for (x, y), (bx, by) in zip(pts, rs.points_xy):
        assert abs(x - bx) < 0.01 and abs(y - by) < 0.01
    assert rs.endpoint_arrive_radius_m == 0.8
    assert isinstance(RouteAssembler(FRAME).accept(clear_body("rg-2", "r-sil", 3)), RouteClear)


def test_goto_as_relative_move_lands_on_the_clicked_point():
    pose, yaw, goal = (2.0, -1.0), 0.7, (6.5, 3.2)
    dx, dy = world_to_body(pose[0], pose[1], yaw, goal[0], goal[1])
    g = translate_relative_move(relmove_body("rm-1", dx, dy), pose_xy=pose, yaw_rad=yaw,
                                heading_valid=True, holonomic=True,
                                limits=RelMoveLimits(20.0, 6.2832, 0.02, 20.0, True))
    assert g.endpoint_xy == pytest.approx(goal, abs=1e-9)
    assert g.abort_on_obstacle is False and g.timeout_s == 120.0


def test_gnss_pair_gives_a_trusted_pose():
    fix, heading = gnss_bodies(12.0, -3.0, 1.2, 0.5, FRAME, 100.0, 90.0)
    pose = assemble_pose(heading, fix)
    assert pose["i_fix"] == 1.0 and pose["heading_valid"] is True
    assert pose["heading_rad"] == 1.2 and pose["i_heading"] == 1.0
    x, y = FRAME.to_xy(pose["lat"], pose["lon"])
    assert abs(x - 12.0) < 1e-6 and abs(y + 3.0) < 1e-6


def test_grant_body_parses():
    assert parse_health_factor(factor_body()) == (1.0, True, "patrol")
