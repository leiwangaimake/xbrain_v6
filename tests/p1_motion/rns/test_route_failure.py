"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_route_failure.py
Brief: Deviation failure (P1.5) + route_rev cross-check (P1.8)

Description:
Guards the route_rev cross-check (11 S7.12.1): a mismatch refuses rather than
following stale geometry. (The deviation-failure tests that lived here were
removed with the mechanism itself -- 20 S2.7 v1.20 user ruling.)
"""

from __future__ import annotations

import pytest

from xbrain.p1_motion.rns.route import RouteRevMismatch, check_route_rev


def test_route_rev_match_passes():
    check_route_rev(command_route_rev=7, loaded_route_rev=7)  # no raise


def test_route_rev_mismatch_refuses():
    # 11 S7.12.1: a command aimed at rev 8 must NOT be executed against loaded
    # rev 7. mutant: skip the check (follow loaded anyway) -> wrong geometry.
    with pytest.raises(RouteRevMismatch) as ei:
        check_route_rev(command_route_rev=8, loaded_route_rev=7)
    assert "8" in str(ei.value) and "7" in str(ei.value)
