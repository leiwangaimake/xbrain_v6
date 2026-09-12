"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_report_map.py
Brief: 12 S4.2c.6 reason -> abort_reason / item / severity table, both closed sets

Description:
Every static row is pinned by value, the two dynamic rows (watchdog fence vs
obstacle by argmax source; blocked_by_dynamic carrying the track id) by
construction, and the reserved target_lost row must RAISE rather than
produce a reason P4 has no phrase for.
"""
from __future__ import annotations

import pytest

from xbrain.p1_motion.nav.report_map import (ABORT_REASONS, ReportMapError,
                                             event_severity, map_failure)
from xbrain.p1_motion.rns.types import NavFailReason, NavFailure

pytestmark = pytest.mark.no_device


@pytest.mark.parametrize("reason, expect", [
    (NavFailReason.MAX_DEVIATION, ("deviation", None, "warn")),
    (NavFailReason.WALL_CLOSED_LOOP, ("obstacle", "unreachable", "warn")),
    (NavFailReason.WALL_NO_PROGRESS, ("obstacle", "no_progress", "warn")),
    (NavFailReason.WALL_BUDGET, ("obstacle", "budget", "warn")),
    (NavFailReason.RTK_UNRELIABLE, ("input_lost", "rtk", "fault")),
    (NavFailReason.NO_PATH_IN_DOMAIN, ("obstacle", "unreachable:domain", "warn")),
    (NavFailReason.EXTRINSIC_UNCALIBRATED, ("input_lost", "extrinsic", "fault")),
])
def test_static_rows(reason, expect):
    got = map_failure(NavFailure(reason, {}))
    assert got == expect
    assert got[0] in ABORT_REASONS


def test_watchdog_carrier_follows_the_argmax_source():
    """mutant: always obstacle -> the fence row red."""
    assert map_failure(NavFailure(NavFailReason.WATCHDOG_NO_PROGRESS,
                                  {"limiter": "fence"})) == ("fence", "fence", "warn")
    assert map_failure(NavFailure(NavFailReason.WATCHDOG_NO_PROGRESS,
                                  {"limiter": "mission"})) == ("obstacle", "mission", "warn")


def test_blocked_by_dynamic_carries_track_id():
    assert map_failure(NavFailure(NavFailReason.BLOCKED_BY_DYNAMIC,
                                  {"track_id": 7, "class_name": "person"})) == (
        "obstacle", "dynamic:7", "warn")


def test_reserved_reason_raises():
    with pytest.raises(ReportMapError):
        map_failure(NavFailure(NavFailReason.TARGET_LOST, {}))


def test_event_severity_is_the_table_column():
    assert event_severity(NavFailure(NavFailReason.RTK_UNRELIABLE, {})) == "fault"
    assert event_severity(NavFailure(NavFailReason.MAX_DEVIATION, {})) == "warn"
