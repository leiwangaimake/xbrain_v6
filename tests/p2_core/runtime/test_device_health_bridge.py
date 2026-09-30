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


#: The wall reading the fixture's injected clock hands back. A fixed, clearly
#: non-zero value so an assertion on ts pins the CLOCK rather than "roughly now"
#: -- 0.0 is what this used to be, and 0.0 is exactly the value p5 silently
#: replaces (see test_the_event_ts_is_the_injected_wall_clock_not_zero).
FIXED_WALL_S = 1787000000.0


def _bridge(**kw):
    emitted = []
    b = DeviceHealthBridge(
        rid="dev", emit=emitted.append,
        now_iso=lambda: "2026-08-17T02:00:00Z",
        eid_gen=lambda dev, off: f"{dev}-{'off' if off else 'on'}",
        now_wall_s=lambda: FIXED_WALL_S,
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
    b.observe("mic", False)
    b.observe("mic", False)   # offline
    b.observe("mic", True)
    b.observe("mic", True)     # online
    assert [e["detail"]["type"] for e in emitted] == \
        ["device_offline", "device_online"]
    assert emitted[1]["cat"] == "voice" and emitted[1]["sev"] == "info"


def test_offline_detail_attached_on_offline_only():
    b, emitted = _bridge(down_threshold=1, up_threshold=1)
    b.register("ptz", offline_detail={"reason": "onvif_unreachable"})
    b.observe("ptz", False)   # offline
    b.observe("ptz", True)    # online
    # 11 S6.2: reason is offline evidence -> on the offline event.
    assert emitted[0]["detail"] == {
        "type": "device_offline", "device": "ptz", "reason": "onvif_unreachable"}
    # MUTATION: if offline_detail leaked onto the online event, a failure reason
    # would ride a recovery. Online must stay {type, device}.
    assert emitted[1]["detail"] == {"type": "device_online", "device": "ptz"}


def test_payload_offline_detail_carries_socket():
    b, emitted = _bridge(down_threshold=1)
    b.register("payload_strobe",
               offline_detail={"reason": "device_link_down", "socket": 8529})
    b.observe("payload_strobe", False)
    assert emitted[0]["detail"]["socket"] == 8529
    assert emitted[0]["detail"]["reason"] == "device_link_down"


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


def test_the_event_ts_is_the_injected_wall_clock_not_zero():
    """11 S6.1 Event.ts must say when the device went down, not when p5 saw it.

    Until 2026-09-30 this bridge passed ts=0.0. Nothing went red because p5's
    _normalise_event writes `d.get("ts") or data.get("ts") or now.timestamp()`
    and 0.0 is FALSY, so record.db quietly received p5's RECEIVE time. Two
    things were wrong with leaning on that: the event time became "when the
    gateway happened to be listening" (a restarted or backlogged p5 stamps its
    own clock, and an offline/online pair can land out of order), and the whole
    path depended on a value being accidentally falsy -- tightening that `or`
    into an `is None` test, which is the correct fix on the p5 side, turns every
    one of these into a real 1970 timestamp.

    Asserts the clock, not "roughly now": a >0 check would also pass for
    time.time() read inline, and the point here is that ts and detected_at come
    from ONE injected source so they can never disagree.

    MUTATION: pass ts=0.0 in _on_transition -> red. Read time.time() inline
    instead of self._now_wall_s() -> red (value is not FIXED_WALL_S).
    """
    b, emitted = _bridge(down_threshold=1)
    b.register("mic")
    b.observe("mic", False)
    b.observe("mic", True)
    b.observe("mic", True)
    assert len(emitted) == 2, "expected the offline/online pair"
    for ev in emitted:
        assert ev["ts"] == FIXED_WALL_S, (
            "event %r carries ts=%r -- a constant/zero ts is what made p5 "
            "substitute its own receive time" % (ev["eid"], ev["ts"]))
        # Same clock as the record fields, which is the property injection buys.
        assert ev["detected_at"] == "2026-08-17T02:00:00Z"


def test_this_bridge_publishes_no_dedup_key_so_it_needs_no_window():
    """Why there is no dedup_window_s here, recorded so nobody adds a guessed one.

    11 S6.2's device_offline / device_online rows specify detail.type,
    detail.device, detail.socket, reason, sev and channel -- and NO dedup_key
    and no window. That is not an omission to paper over: p5's
    record_dao._attempt_insert only calls _try_merge `if ev.get("dedup_key")`,
    so with no key nothing can be merged and a window would never be read.
    The flood protection is DeviceLivenessMonitor's debounce, which is counted
    in samples per state rather than seconds -- the same argument 12 S6A.8
    PUB-5 makes for not inventing a heartbeat period (CLAUDE.md S3.7: do not
    write down a number nobody measured).

    MUTATION: add a dedup_key here without a window -> red. That combination is
    the one that silently swallows events, because _try_merge folds onto the
    latest un-abandoned row unconditionally when neither side has a window.
    """
    b, emitted = _bridge(down_threshold=1)
    b.register("ptz")
    b.observe("ptz", False)
    ev = emitted[0]
    assert "dedup_key" not in ev, (
        "a dedup_key appeared; 11 S6.2 gives this event neither a key nor a "
        "window, and a key WITHOUT a window is silently merged by p5")
    assert "dedup_window_s" not in ev
