"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_link_detector.py
Brief: LinkReconnectDetector -- cloud-link down->up edge for the backfill trigger

Description:
Pins the ONE thing the detector must get right: report a down->up edge EXACTLY once
per reconnect, so the wiring fires exactly one backfill. Synthetic monotonic clock,
no Zenoh. Mutations paired per 3.3: a repeat edge while up = a backfill every tick.
"""

import pytest

from xbrain.p5_gateway.reconnect.link_detector import LinkReconnectDetector


pytestmark = pytest.mark.no_device


def test_starts_down_and_no_rx_is_never_an_edge():
    d = LinkReconnectDetector(timeout_s=10.0)
    assert d.is_up is False
    assert d.poll(100.0) is False       # never heard from cloud -> no edge
    assert d.poll(200.0) is False


def test_first_contact_is_an_edge():
    d = LinkReconnectDetector(timeout_s=10.0)
    d.note_cloud_rx(100.0)
    assert d.poll(100.5) is True        # down->up -> trigger backfill
    assert d.is_up is True


def test_no_repeat_edge_while_up():
    d = LinkReconnectDetector(timeout_s=10.0)
    d.note_cloud_rx(100.0)
    assert d.poll(100.5) is True
    d.note_cloud_rx(101.0)
    # MUTATION: if poll returned True while staying up, the wiring would fire a
    # backfill every heartbeat -> a replay storm on a healthy link.
    assert d.poll(101.5) is False
    assert d.poll(102.0) is False


def test_silence_drops_to_down_without_an_edge():
    d = LinkReconnectDetector(timeout_s=10.0)
    d.note_cloud_rx(100.0)
    assert d.poll(100.5) is True
    # now - last_rx = 20 > 10 -> down. Going DOWN is not a backfill edge.
    assert d.poll(120.0) is False
    assert d.is_up is False


def test_reconnect_after_silence_is_an_edge():
    d = LinkReconnectDetector(timeout_s=10.0)
    d.note_cloud_rx(100.0)
    d.poll(100.5)                       # up
    d.poll(120.0)                       # silence -> down
    d.note_cloud_rx(121.0)              # cloud back
    assert d.poll(121.2) is True        # reconnect -> one backfill
    assert d.poll(121.5) is False       # and not again while it stays up


def test_bad_timeout_rejected():
    with pytest.raises(ValueError):
        LinkReconnectDetector(timeout_s=0)
