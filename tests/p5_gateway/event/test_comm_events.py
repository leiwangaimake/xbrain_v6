"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_comm_events.py
Brief: 11 S4.6.8 comm-event mapping -- level transition -> kind/sev

Description:
Pins the 11 S4.6.8 table: each cloud-link level transition maps to the right comm
event kind + severity, no event when nothing transitioned, and cloud_up carries the
outage duration + link_epoch. Mutations paired per 3.3.
"""

import pytest

from xbrain.p5_gateway.event.comm_events import comm_event_for_level


pytestmark = pytest.mark.no_device


def test_first_observation_is_no_event():
    assert comm_event_for_level(None, 2, 0.0, 1) is None


def test_no_transition_is_no_event():
    assert comm_event_for_level(1, 1, 3.0, 1) is None


def test_l0_to_l1_degraded():
    kind, sev, detail = comm_event_for_level(0, 1, 0.0, 1)
    assert kind == "cloud_degraded" and sev == "warn"
    assert detail["kind"] == "cloud_degraded" and detail["level"] == 1


def test_l1_to_l2_down():
    kind, sev, _ = comm_event_for_level(1, 2, 0.0, 1)
    assert kind == "cloud_down" and sev == "warn"


def test_l2_to_l3_lost_is_alarm():
    kind, sev, _ = comm_event_for_level(2, 3, 0.0, 1)
    # MUTATION: cloud_lost at warn would ride best-effort and never force ack/backfill
    # priority for "we have lost the cloud".
    assert kind == "cloud_lost" and sev == "alarm"


def test_return_to_l0_is_cloud_up_with_outage():
    kind, sev, detail = comm_event_for_level(3, 0, 1803.4, 7)
    assert kind == "cloud_up" and sev == "info"
    # S4.6.8: cloud_up MUST carry the total outage duration + link_epoch, taken from
    # the tick BEFORE the reset (disconnected_s is 0 the instant we return to L0).
    assert detail["disconnected_s"] == 1803.4 and detail["link_epoch"] == 7


def test_jump_reaches_level_event():
    # A sparse-evaluate jump 0->2 fires for the level reached (cloud_down), not two.
    kind, sev, _ = comm_event_for_level(0, 2, 0.0, 1)
    assert kind == "cloud_down"
