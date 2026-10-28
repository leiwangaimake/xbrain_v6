"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_broadcast_mode_fields.py
Brief: p5 projects p2's broadcast fields to the cloud -- state/mode source/exit_reason, state/audio last_frame_age_ms

Description:
Gap B1 / B4 of the 2026-10-28 broadcast review (99 U97): the cloud's
state/mode.exit_reason and state/audio.last_frame_age_ms were constant null --
p5 read hmi_state keys that nothing wrote. p2 now publishes them (state/mode
source / entered_ts / exit_reason; state/audio speaker.last_frame_age_ms) and
p5's _on_state_mode stores them under mode_source / mode_entered_ts /
mode_exit_reason. These cases pin the projection end of that path.

Kept in its own file (not test_cloud_projector.py) so it does not interleave
with another session's uncommitted edits there.
"""

from __future__ import annotations  # postponed annotations

import pytest  # marks

from xbrain.p5_gateway.runtime.cloud_state import CloudProjector  # the cloud projection

pytestmark = pytest.mark.no_device  # pure in-process


class _Clock:  # fake monotonic clock
    """Fake monotonic clock in seconds (the projector reads seconds)."""

    def __init__(self, t: float) -> None:  # fixed time in seconds
        self.t = t  # seconds

    def __call__(self) -> float:  # the projector calls it
        return self.t  # never moves


class _Bridge:  # placeholder bridge
    """The projector only needs an object to hold; these cases call the
    projection methods directly and never publish."""


def _proj(t: float = 1000.0) -> CloudProjector:  # projector at time t
    return CloudProjector(_Bridge(), now_mono=_Clock(t))  # injected clock


def test_mode_projects_source_entered_ts_and_exit_reason():
    """MUTATION: keep the old constant source="system" or drop exit_reason ->
    this fails (the pre-fix projection)."""
    out = _proj()._mode({"mode": "normal", "stream_id": None,  # after a broadcast
                         "mode_source": "cloud",  # written by _on_state_mode
                         "mode_entered_ts": 1785732030.2,  # p2's entered_ts
                         "mode_exit_reason": "timeout"})  # p2's exit_reason
    assert out["source"] == "cloud"  # not the old constant
    assert out["entered_ts"] == 1785732030.2  # passed through
    assert out["exit_reason"] == "timeout"  # no longer always null


def test_mode_without_p2_fields_still_projects_as_system():
    """An older p2 (or nothing received yet): source falls back to system,
    exactly the former constant -- never a guess."""
    out = _proj()._mode({"mode": "broadcast", "stream_id": "audio-gj001-0001"})  # old p2
    assert out["source"] == "system"  # the former constant
    assert out["exit_reason"] is None  # unknown stays null


def _audio_state(age_ms, updated_ms):  # hmi_state during a broadcast
    """state/audio as p2 publishes it during a broadcast."""
    return {"audio": {"speaker": {"holder": "broadcast_b",  # speaking
                                  "last_frame_age_ms": age_ms},  # p2's value
                      "mic": {"open": False, "gate_reason": "broadcast_active"},  # gated
                      "devices": {"mic": "ok"}},  # healthy mic
            "audio_updated_ms": updated_ms, "stream_id": "audio-gj001-0001"}  # arrival


def test_last_frame_age_adds_the_time_the_message_waited_here():
    """p2 said 40 ms at publish; the message has sat in p5 for 600 ms ->
    640 ms. MUTATION: pass p2's value through unchanged -> 40 -> fails (the
    value would freeze between p2's 1 Hz publishes and hide a dead stream)."""
    proj = _proj(t=1000.6)                          # now = 1 000 600 ms
    out = proj._audio(_audio_state(40, 1_000_000.0))  # arrived 600 ms ago
    assert out["last_frame_age_ms"] == 640  # 40 + 600


def test_last_frame_age_is_null_when_p2_gives_none():
    """No frame yet -> null, never an invented 0."""
    proj = _proj(t=1000.0)  # now = arrival time
    out = proj._audio(_audio_state(None, 1_000_000.0))  # no frame yet
    assert out["last_frame_age_ms"] is None  # never an invented 0


# -- AUDIO_CONTROL ack echo on rejection (v2.0 S2.5) ------------------------
# Minimal copies of the cloud-bridge test doubles (tests/p5_gateway/
# test_cloud_bridge.py), kept local so this file stands alone.

import json  # noqa: E402

RID = "gj-001"  # the robot id on the cloud keys


class _Pub:  # publisher double
    """Records every put on the declared key."""

    def __init__(self, key, log):  # key + shared log
        self._key, self._log = key, log  # remember both

    def put(self, payload):  # zenoh put
        self._log.append((self._key, payload))  # recorded


class _Session:  # session double
    """Captures subscriptions and publishes like a zenoh session would."""

    def __init__(self):  # empty
        self.subs, self.puts = {}, []  # callbacks and puts

    def declare_subscriber(self, key, cb):  # zenoh subscribe
        self.subs[key] = cb  # so the test can feed it
        return object()  # a handle the bridge keeps

    def declare_publisher(self, key):  # zenoh publisher
        return _Pub(key, self.puts)  # shares the log


class _Sample:  # zenoh sample double
    def __init__(self, key, body):  # key + JSON body
        self.key_expr = key  # as zenoh names it
        self.payload = json.dumps(body, ensure_ascii=False).encode("utf-8")  # bytes


def _puts(session, key):  # decoded puts on one key
    return [json.loads(p.decode("utf-8")) for k, p in session.puts if k == key]  # in order


def test_a_rejected_exit_still_echoes_the_stream_id():
    """p2 refuses an exit whose stream_id is not the live session (E_NOT_FOUND,
    detail.stream_id). The cloud ack must still carry detail.stream_id: v2.0
    S2.5 applies the echo rule to accepted, duplicate AND rejected. The echo
    rides on translate_ack copying the internal detail wholesale (the
    AUDIO_CONTROL block in cloud_wiring only lifts applied.stream_id) -- a
    mutant that read applied only in that block SURVIVED (equivalent, so it
    was not added); the load-bearing mutant is translate_ack dropping detail
    -> no stream_id -> this fails. It also pins the gateway naming the
    channel (source = cloud) on the cmd/mode it sends to p2."""
    from xbrain.p5_gateway.runtime.cloud_wiring import CloudBridge  # the bridge

    session = _Session()  # fresh
    CloudBridge(session, RID).wire()  # subscribes cmd/task etc.
    key = "xbrain/%s/cmd/task" % RID  # Qt's command key
    session.subs[key](_Sample(key, {  # Qt sends
        "v": 1, "rid": RID, "ts": 1.0, "seq": 1, "src": "qt_hmi",  # S3.0 envelope
        "data": {"msg_id": "a-1", "task_id": "ta-1",  # v2.0 ids
                 "task_type": "AUDIO_CONTROL",  # v2.0 S2.5
                 "payload": {"mode": "pc_to_dog", "action": "exit_broadcast",  # exit
                             "stream_id": "audio-gj001-0007"}}}))  # a stale id
    mode_cmd = _puts(session, "cmd/mode")[0]  # what went to p2
    # The gateway names the channel so p2 can fill state/mode.source.
    assert mode_cmd["source"] == "cloud"  # channel for state/mode.source
    session.subs["cmd/mode/ack"](_Sample("cmd/mode/ack", {  # p2 answers
        "schema": "mode_ack_v1", "cmd_id": mode_cmd["cmd_id"],  # same command
        "result": "rejected", "code": "E_NOT_FOUND",  # refused
        "detail": {"reason": "stream_id_mismatch",  # why
                   "stream_id": "audio-gj001-0007"}}))  # echo from p2
    acks = _puts(session, "xbrain/%s/cmd/task/ack" % RID)  # what Qt receives
    assert acks and acks[0]["data"]["accepted"] is False  # a rejection
    assert acks[0]["data"]["detail"]["stream_id"] == "audio-gj001-0007"  # still echoed
