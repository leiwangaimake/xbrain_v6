"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_voice_loop_smoke.py
Brief: V-3B voice loop smoke test -- p4 dispatch -> p3 observe -> p5 observe end-to-end

Description:
Validates the wiring produced by V-2A/B/C at a functional level.
Uses zenoh PEER mode (no external zenohd) so pytest runs on any
dev machine. The full runtime with routers + AI services + real
MIC + GZH-2 + chassis is validated by scripts/dev/start_voice_loop.sh
on the ORIN (not by this pytest).

Scenario:
  1. Open ONE gen-plane peer session
  2. Subscribe cmd/task + state/task + state/link on it
  3. Simulate p4 by publishing a cmd/task payload (as if intent
     dispatcher decided B01 patrol)
  4. Assert the payload appears + shape matches

Not tested here (needs live infra, exercised by shell smoke):
  * MIC -> arecord -> rt/audio/mic (needs USB MIC + alsa)
  * ASR HTTP call (needs services/asr on :18081)
  * TTS HTTP call (needs services/payload + GZH-2)
  * P1 -> chassis_stub TCP (integration only; test_v2c covered
    the ChassisClient logic in isolation)
"""

from __future__ import annotations

import json
import threading
import time

import pytest


pytestmark = pytest.mark.no_device


def _make_peer_config():
    """A zenoh peer-mode config: no scouting, no connect, so tests
    are hermetic (no interference with a live router)."""
    import zenoh
    cfg = zenoh.Config()
    cfg.insert_json5("mode", '"peer"')
    cfg.insert_json5("scouting/multicast/enabled", "false")
    cfg.insert_json5("scouting/gossip/enabled", "false")
    return cfg


def _pick_port() -> int:
    """OS-assigned free TCP port for this fixture instance so
    concurrent test runs don't collide."""
    import socket
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def peer_pair():
    """Single peer session used as both publisher + subscriber
    (zenoh allows same-session self-observation in peer mode).
    Yields (pub_sess, sub_sess) as the same object -- easier +
    faster than a two-node bring-up, sufficient for a wire-shape
    smoke test."""
    import zenoh

    cfg = _make_peer_config()
    sess = zenoh.open(cfg)
    time.sleep(0.1)

    yield sess, sess

    try:
        sess.close()
    except Exception:      # noqa: BLE001
        pass


def test_cmd_task_publish_and_observe(peer_pair):
    """The core wiring assertion: a payload published on
    cmd/task under the shape p4 produces gets observed with all
    the expected fields."""
    pub_sess, sub_sess = peer_pair

    observed = []
    evt = threading.Event()

    def _on_task(sample):
        observed.append(bytes(sample.payload))
        evt.set()

    sub = sub_sess.declare_subscriber("cmd/task", _on_task)

    # Emit a payload matching intent_dispatch.build_payload shape.
    from xbrain.p4_agent.runtime.intent_dispatch import dispatch

    result = dispatch("B01", "巡逻")
    payload_bytes = json.dumps(result.payload,
                                 ensure_ascii=False,
                                 separators=(",", ":")).encode("utf-8")
    pub = pub_sess.declare_publisher("cmd/task")
    pub.put(payload_bytes)

    assert evt.wait(timeout=3.0), \
        "cmd/task subscriber did not observe within 3 s (peer setup?)"

    # Shape assertion: the received payload matches what p3 expects.
    parsed = json.loads(observed[0].decode("utf-8"))
    assert parsed["intent_id"] == "B01"
    assert parsed["text"] == "巡逻"
    assert "mono_ms" in parsed
    assert parsed["schema"] == "p4_intent_v1"

    try:
        pub.undeclare()
    except Exception:      # noqa: BLE001
        pass
    try:
        sub.undeclare()
    except Exception:      # noqa: BLE001
        pass


def test_state_link_publish_and_observe(peer_pair):
    """P5's state/link heartbeat is what HMI / Qt uses to detect
    'gateway alive'. Verify a published frame reaches a subscriber."""
    pub_sess, sub_sess = peer_pair

    observed = []
    evt = threading.Event()

    def _on_link(sample):
        observed.append(bytes(sample.payload))
        evt.set()

    sub = sub_sess.declare_subscriber("state/link", _on_link)
    pub = pub_sess.declare_publisher("state/link")

    pub.put(json.dumps({
        "schema": "state_link_v1",
        "gateway_up": True,
        "mono_ms": 42,
    }).encode())

    assert evt.wait(timeout=3.0)
    parsed = json.loads(observed[0].decode())
    assert parsed["schema"] == "state_link_v1"
    assert parsed["gateway_up"] is True

    try:
        pub.undeclare()
    except Exception:      # noqa: BLE001
        pass
    try:
        sub.undeclare()
    except Exception:      # noqa: BLE001
        pass


def test_p4_publish_pipeline_shape_matches_p3_subscription():
    """No live sessions -- pure static check: what p4 publishes
    for a task intent is exactly what p3's _on_task expects to
    receive. Ensures schema alignment without any zenoh calls."""
    from xbrain.p3_task.runtime.main_wiring import CMD_TASK_TOPIC
    from xbrain.p4_agent.runtime.intent_dispatch import (
        CMD_TASK, dispatch,
    )

    # p4 dispatches B01 to CMD_TASK.
    result = dispatch("B01", "巡逻")
    assert result.key == CMD_TASK == CMD_TASK_TOPIC

    payload = result.payload
    # p3's _on_task extracts intent_id + text
    assert "intent_id" in payload and "text" in payload


def test_p4_dispatch_covers_five_key_families():
    """Sanity: p4 can dispatch to all 5 outbound key families."""
    from xbrain.p4_agent.runtime.intent_dispatch import (
        CMD_AUDIO_SPEAK, CMD_MOTION_INTENT, CMD_PAYLOAD, CMD_PTZ,
        CMD_TASK, dispatch,
    )
    cases = [
        ("D07", CMD_AUDIO_SPEAK),
        ("B01", CMD_TASK),
        ("A05", CMD_MOTION_INTENT),
        ("E01", CMD_PTZ),
        ("R05", CMD_PAYLOAD),
    ]
    for intent_id, expected_key in cases:
        assert dispatch(intent_id, "x").key == expected_key


def test_audio_frame_wire_roundtrip_via_peer(peer_pair):
    """Verify the AudioFrame JSON codec + rt/audio/mic wiring:
    p2 encodes, p4 decodes."""
    pub_sess, sub_sess = peer_pair

    from xbrain.p2_core.audio.audio_io import (
        ASR_RATE_HZ, ASR_SAMPLES_PER_FRAME, AudioFrame,
    )
    from xbrain.p2_core.runtime.mic_capture import (
        decode_frame, encode_frame,
    )

    src = AudioFrame(
        rate_hz=ASR_RATE_HZ, channels=1, sample_width=2,
        frame_ms=20,
        samples=list(range(ASR_SAMPLES_PER_FRAME)))

    observed = []
    evt = threading.Event()

    def _on_mic(sample):
        observed.append(bytes(sample.payload))
        evt.set()

    sub = sub_sess.declare_subscriber("rt/audio/mic", _on_mic)
    pub = pub_sess.declare_publisher("rt/audio/mic")
    pub.put(encode_frame(src))

    assert evt.wait(timeout=3.0)
    decoded = decode_frame(observed[0])
    assert decoded.rate_hz == src.rate_hz
    assert decoded.samples == src.samples

    try:
        pub.undeclare()
    except Exception:      # noqa: BLE001
        pass
    try:
        sub.undeclare()
    except Exception:      # noqa: BLE001
        pass
