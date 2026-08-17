"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_device_events.py
Brief: device_offline/online build + liveness debounce (batch 6)

Description:
build_device_event maps device id -> category and sets the sev/detail the 11 S6.2
addition specifies, and a built event -- run through the real pipeline -- derives
channel=alarm + need_ack (proving it is contract-valid and rides the alarm backfill
cursor). DeviceLivenessMonitor fires once per confirmed transition and swallows
flaps. Mutations paired per 3.3.
"""

import pytest

from xbrain.p5_gateway.event.device_events import (
    DEVICE_CATEGORY, DeviceLivenessMonitor, UnknownDevice, build_device_event,
)
from xbrain.p5_gateway.event.channel_map import derive_channel


pytestmark = pytest.mark.no_device


def _build(device_id, offline):
    return build_device_event(
        device_id, offline, rid="m20s-001", eid=f"{device_id}-x",
        detected_at="2026-08-17 10:00:00", created_at="2026-08-17T02:00:00Z",
        ts=100.0, src="payload_service")


# --- build_device_event ---

def test_payload_device_maps_to_payload_category():
    ev = _build("payload_light", offline=True)
    assert ev["cat"] == "payload" and ev["sev"] == "warn"
    assert ev["detail"] == {"type": "device_offline", "device": "payload_light"}
    # channel NOT set here -- the pipeline derives it (S3.3).
    assert "channel" not in ev


def test_mic_maps_to_voice_ptz_to_ptz():
    assert _build("mic", True)["cat"] == "voice"
    assert _build("ptz", True)["cat"] == "ptz"


def test_online_is_info_and_device_online():
    ev = _build("ptz", offline=False)
    assert ev["sev"] == "info"
    assert ev["detail"]["type"] == "device_online"


def test_unknown_device_raises():
    with pytest.raises(UnknownDevice):
        _build("gnss_chassis", True)


def test_built_event_derives_alarm_channel():
    # The whole reason these ride channel=alarm: a built device event, fed the
    # S6.2 map, lands on the alarm cursor (offline AND online, paired, E-1).
    for offline in (True, False):
        ev = _build("payload_siren", offline)
        assert derive_channel(ev["cat"], ev["detail"]) == "alarm"


def test_every_device_category_is_a_valid_event_category():
    # Meta: every device rolls up to a real 11 S6.2 category.
    from xbrain.common.enums import EVENT_CATEGORY
    assert set(DEVICE_CATEGORY.values()) <= set(EVENT_CATEGORY.values)


# --- DeviceLivenessMonitor debounce ---

def _mon(**kw):
    fired = []
    m = DeviceLivenessMonitor(
        "payload_speaker",
        emit=lambda dev, offline: fired.append((dev, offline)),
        **kw)
    return m, fired


def test_healthy_from_boot_emits_nothing():
    m, fired = _mon()
    for _ in range(10):
        assert m.observe(True) is None
    assert fired == []


def test_offline_fires_once_after_threshold():
    m, fired = _mon(down_threshold=3)
    assert m.observe(False) is None      # streak 1
    assert m.observe(False) is None      # streak 2
    assert m.observe(False) is True      # streak 3 -> offline
    assert m.observe(False) is None      # already offline, no repeat
    assert fired == [("payload_speaker", True)]


def test_flap_below_threshold_does_not_fire():
    m, fired = _mon(down_threshold=3)
    m.observe(False)
    m.observe(False)
    m.observe(True)      # recovered before the 3rd down -> streak resets
    # MUTATION: down_threshold=1 (no debounce) -> the first down would have fired.
    assert fired == []


def test_online_fires_after_recovery():
    m, fired = _mon(down_threshold=2, up_threshold=2)
    m.observe(False); m.observe(False)   # offline
    assert m.observe(True) is None       # up streak 1
    assert m.observe(True) is False      # up streak 2 -> online
    assert fired == [("payload_speaker", True), ("payload_speaker", False)]


def test_bad_thresholds_rejected():
    with pytest.raises(ValueError):
        DeviceLivenessMonitor("ptz", emit=lambda a, b: None, down_threshold=0)
