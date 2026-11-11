"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_charging_channel.py
Brief: The low-battery reports ride the alarm backfill channel, the CHG stage events stay normal (99 U157)

Description:
User ruling 2026-11-11: every report of a low-battery episode -- the opening
alarms, the charging / interrupted alarms, the close and the boot report --
rides the alarm cursor, so a reconnect replays them first and the opening and
closing reports share one cursor (11 S9A.9 E-1: a pair split across cursors
can leave the cloud stuck in alarm). Until then the whole charging category
rode normal: 11 said "channel = alarm" in its event/*/charging row but its
S6.2 table said "charging -> normal", and channel_map followed the table
(record.db on the robot held 15135 charging events, all normal, including
sev=alarm low-battery reports). Each case names the mutant it kills.
"""
from __future__ import annotations

import pytest

from xbrain.p5_gateway.event.channel_map import derive_channel

pytestmark = pytest.mark.no_device

LOW_BATTERY_TYPES = ["low_battery", "low_battery_unreachable", "low_battery_no_target",
                     "low_battery_stop", "battery_charging", "charge_interrupted",
                     "low_battery_cleared", "return_target_missing"]


@pytest.mark.parametrize("kind", LOW_BATTERY_TYPES)
def test_every_low_battery_report_rides_alarm(kind):
    """MUTATION: drop any one override -> that type falls back to normal -> red."""
    assert derive_channel("charging", {"type": kind}) == "alarm"


def test_the_open_and_the_close_share_one_cursor():
    """E-1 pairing. MUTATION: override the alarms but not low_battery_cleared
    -> the close rides normal while the alarm rides alarm -> red."""
    assert (derive_channel("charging", {"type": "low_battery"})
            == derive_channel("charging", {"type": "low_battery_cleared"}))


@pytest.mark.parametrize("detail", [None, {}, {"stage": "charge_complete"},
                                    {"type": "something_else"}])
def test_other_charging_events_stay_normal(detail):
    """The CHG stage events keep the S6.2 default. MUTATION: flip the whole
    category to alarm -> red."""
    assert derive_channel("charging", detail) == "normal"
