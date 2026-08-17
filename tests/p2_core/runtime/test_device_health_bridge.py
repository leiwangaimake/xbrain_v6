"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_device_health_bridge.py
Brief: DeviceHealthBridge -- liveness -> device_offline/online event emit

Description:
Feeds the bridge liveness samples and checks it emits the correct 11 S6.2 event on
a confirmed transition, stays silent on 'unknown' (None) and on a flap below the
debounce, and recovers with an online event. Mutations paired per 3.3.
"""

import pytest

from xbrain.p2_core.runtime.device_health_bridge import DeviceHealthBridge


pytestmark = pytest.mark.no_device


def _bridge(**kw):
    emitted = []
    b = DeviceHealthBridge(
        rid="dev", emit=emitted.append,
        now_iso=lambda: "2026-08-17T02:00:00Z",
        eid_gen=lambda dev, off: f"{dev}-{'off' if off else 'on'}",
        **kw)
    return b, emitted


def test_healthy_emits_nothing():
    b, emitted = _bridge()
    b.register("mic")
    for _ in range(10):
        b.observe("mic", True)
    assert emitted == []


def test_offline_emits_device_offline_event():
    b, emitted = _bridge(down_threshold=3)
    b.register("mic")
    b.observe("mic", False)
    b.observe("mic", False)
    b.observe("mic", False)   # threshold -> offline
    assert len(emitted) == 1
    ev = emitted[0]
    assert ev["cat"] == "voice" and ev["sev"] == "warn"
    assert ev["detail"] == {"type": "device_offline", "device": "mic"}
    assert ev["eid"] == "mic-off" and ev["src"] == "p2_core"


def test_unknown_none_feeds_nothing():
    b, emitted = _bridge(down_threshold=1)
    b.register("ptz")
    # MUTATION: if None advanced the debounce, this would emit an offline.
    for _ in range(5):
        b.observe("ptz", None)
    assert emitted == []


def test_flap_below_threshold_silent():
    b, emitted = _bridge(down_threshold=3)
    b.register("payload_light")
    b.observe("payload_light", False)
    b.observe("payload_light", False)
    b.observe("payload_light", True)   # recovered before threshold
    assert emitted == []


def test_recovery_emits_online():
    b, emitted = _bridge(down_threshold=2, up_threshold=2)
    b.register("mic")
    b.observe("mic", False); b.observe("mic", False)   # offline
    b.observe("mic", True); b.observe("mic", True)     # online
    assert [e["detail"]["type"] for e in emitted] == \
        ["device_offline", "device_online"]
    assert emitted[1]["cat"] == "voice" and emitted[1]["sev"] == "info"


def test_observe_unregistered_device_is_noop():
    b, emitted = _bridge(down_threshold=1)
    # never registered -> observe does nothing (no monitor)
    b.observe("mic", False)
    assert emitted == []


def test_register_idempotent():
    b, emitted = _bridge(down_threshold=1)
    b.register("ptz")
    b.register("ptz")   # second register must not reset the monitor
    b.observe("ptz", False)
    assert len(emitted) == 1
