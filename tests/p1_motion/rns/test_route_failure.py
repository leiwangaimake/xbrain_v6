"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_route_failure.py
Brief: Deviation failure (P1.5) + route_rev cross-check (P1.8)

Description:
Guards the two route-layer failure paths: deviation over max (A-DEV-1, 20 S2.7)
becomes a MAX_DEVIATION NavFailure with e/s detail, and a route_rev mismatch
(11 S7.12.1) refuses rather than following stale geometry. Each names its mutant.
"""

from __future__ import annotations

import pytest

from xbrain.p1_motion.rns.route import (
    Mission, RouteRevMismatch, check_route_rev,
)
from xbrain.p1_motion.rns.types import MissionKind, NavFailReason, Origin


def _path(n=21, spacing=0.5):
    return [(i * spacing, 0.0) for i in range(n)]


def _mk(max_dev=2.0):
    return Mission(MissionKind.PATH, Origin.ROUTE, _path(),
                   search_window=8, arrival_radius_m=1.0, max_deviation_m=max_dev)


def test_deviation_within_limit_no_failure():
    m = _mk(max_dev=2.0)
    fs = m.advance((5.0, 1.0), lookahead_m=1.0)  # 1 m off, under 2 m
    assert m.deviation_failure(fs) is None


def test_deviation_over_limit_yields_failure():
    # A-DEV-1: e > max_deviation -> MAX_DEVIATION failure with e/s detail.
    # mutant: drop the deviation_exceeded gate -> never fails -> the machine
    # drifts forever "in progress" (20 S13 trap). reddens here.
    m = _mk(max_dev=2.0)
    # (2.0, 5.0): within the search window (segments 0..8 span x=0..8), the foot
    # is (2.0, 0.0) so e = 5.0 exactly.
    fs = m.advance((2.0, 5.0), lookahead_m=1.0)
    f = m.deviation_failure(fs)
    assert f is not None
    assert f.reason is NavFailReason.MAX_DEVIATION
    assert f.detail["e_m"] == pytest.approx(5.0, abs=1e-6)
    assert "s_arc_m" in f.detail
    assert f.detail["max_deviation_m"] == 2.0


def test_route_rev_match_passes():
    check_route_rev(command_route_rev=7, loaded_route_rev=7)  # no raise


def test_route_rev_mismatch_refuses():
    # 11 S7.12.1: a command aimed at rev 8 must NOT be executed against loaded
    # rev 7. mutant: skip the check (follow loaded anyway) -> wrong geometry.
    with pytest.raises(RouteRevMismatch) as ei:
        check_route_rev(command_route_rev=8, loaded_route_rev=7)
    assert "8" in str(ei.value) and "7" in str(ei.value)
