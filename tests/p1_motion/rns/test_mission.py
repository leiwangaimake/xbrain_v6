"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_mission.py
Brief: goto/path unification + endpoint-only arrival (P1.3 -- 20 S2.1/S2.4)

Description:
Guards the Mission layer: goto and path run the SAME follow code (A-RT-1), and
arrival is judged ONLY at the endpoint (A-RT-2 / S2.4). The A-RT-2 test is the
important one: it drives a dense 0.5 m path and asserts the index climbs point by
point without an intermediate "arrival" jumping it forward. Each test names its
mutant (CLAUDE.md 3.3).
"""

from __future__ import annotations

import math

import pytest

from xbrain.p1_motion.rns.route import Mission
from xbrain.p1_motion.rns.types import MissionKind, Origin


def _dense_path(n=21, spacing=0.5):
    # a straight 0.5 m path along +x (the recorded-path density, 11 S12A.6)
    return [(i * spacing, 0.0) for i in range(n)]


def _mk(points, kind=MissionKind.PATH, origin=Origin.ROUTE,
        arrival_radius_m=1.0, max_deviation_m=10.0, search_window=8):
    return Mission(kind, origin, points, search_window=search_window,
                   arrival_radius_m=arrival_radius_m,
                   max_deviation_m=max_deviation_m)


def test_goto_is_single_segment_polyline():
    # RNS-N-1: goto = one point. Same Mission, no separate code path.
    m = _mk([(5.0, 0.0)], kind=MissionKind.GOTO, origin=Origin.RELMOVE)
    fs = m.advance((0.0, 0.0), lookahead_m=2.0)
    assert fs.dist_to_endpoint_m == pytest.approx(5.0)
    assert fs.arrived is False
    fs2 = m.advance((4.5, 0.0), lookahead_m=2.0)  # within arrival_radius 1.0
    assert fs2.arrived is True


def test_path_and_goto_use_same_advance():
    # A-RT-1: both kinds go through Mission.advance -- there is no goto-only or
    # path-only method. This test would need a second method to break; that a
    # single advance() serves both IS the unification.
    goto = _mk([(3.0, 0.0)], kind=MissionKind.GOTO)
    path = _mk(_dense_path())
    # same call shape, same return type
    assert type(goto.advance((0.0, 0.0), 2.0)) is type(path.advance((0.0, 0.0), 2.0))


def test_intermediate_points_never_trigger_arrival():
    # A-RT-2: driving a dense path, arrived must stay False until the ENDPOINT.
    # mutant: judge arrival against the current segment's far vertex (not the
    # endpoint) -> arrived flips True at the first 0.5 m point -> reddens.
    m = _mk(_dense_path(n=21))  # endpoint at x=10.0, arrival_radius 1.0
    # intermediate points are those > arrival_radius from the endpoint: x <= 8.5
    # (dist 1.5). None of them may report arrival, even though each sits exactly
    # on a recorded 0.5 m vertex.
    for x in [i * 0.5 for i in range(0, 18)]:  # 0.0 .. 8.5
        fs = m.advance((x, 0.0), lookahead_m=1.0)
        assert fs.arrived is False, "arrived at intermediate point x=%s" % x
    # only inside the endpoint radius does arrival fire
    fs = m.advance((9.5, 0.0), lookahead_m=1.0)  # dist to endpoint 0.5 < 1.0
    assert fs.arrived is True


def test_dense_path_index_climbs_point_by_point():
    # the A-RT-2 mutant's SYMPTOM (20 S13): an intermediate arrival would jump
    # the index by >1. Here index advances smoothly, one segment at a time.
    m = _mk(_dense_path(n=41))  # 0.0 .. 20.0
    last = 0
    for x in [i * 0.5 for i in range(0, 40)]:
        m.advance((x, 0.0), lookahead_m=1.0)
        jump = m.tracker.min_index - last
        assert jump <= 1, "index jumped %d at x=%s (path shape broken)" % (jump, x)
        last = m.tracker.min_index


def test_mission_carries_origin_and_kind():
    # 12 S4.2c.1: origin decides the report channel; source.py reads it.
    m = _mk(_dense_path(), origin=Origin.RELMOVE, kind=MissionKind.GOTO)
    assert m.origin is Origin.RELMOVE
    assert m.kind is MissionKind.GOTO


def test_endpoint_is_last_point():
    m = _mk(_dense_path(n=21))
    assert m.endpoint == (10.0, 0.0)


def test_deviation_exceeded_flag():
    # e > max_deviation_m sets the flag (P1.5 turns it into failure).
    m = _mk(_dense_path(), max_deviation_m=2.0)
    fs = m.advance((5.0, 5.0), lookahead_m=1.0)  # 5 m off the y=0 path
    assert fs.deviation_exceeded is True
    fs2 = m.advance((5.0, 1.0), lookahead_m=1.0)  # 1 m off, under limit
    assert fs2.deviation_exceeded is False
