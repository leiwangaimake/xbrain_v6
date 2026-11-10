"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_hmi_estop_ws_deadman.py
Brief: HMI W1 estop over the WebSocket (17 S6.4) + WS close releases cmd/teleop (17 S6.2.1)

Description:
Two A-class HMI fixes from the 2026-11-10 audit, asserted end to end.

W1 estop on the socket (11 S12.1.1 / S12.1.4 gate G0). Before, an estop frame on
the WebSocket was refused (E_SCHEMA without a req_id, E_NOT_IMPLEMENTED with
one) and never forwarded; the button only had a REST POST. Now the frame is
forwarded before parsing, rate limiting or anything else, and always answered
accepted/OK -- except when the publish itself raised, which answers
error/E_INTERNAL so the operator is told to use the hardware stop. A frame that
is not JSON no longer ends the reader (the old receive_json() did, silently, so
every later click on that tab went unanswered).

WS close -> release (17 S6.2.1 WD-1/WD-2/WD-4/WD-6). deadman.py existed but was
called from nowhere, and its payload {deadman, vx, vy, wz} carried neither
`device` nor `source`, so P1 would have dropped it anyway. The release is
asserted here against P1's real TeleopTracker, not against a copy of its shape.

Each assertion names the mutant that turns it red (CLAUDE.md 3.3).
"""

from __future__ import annotations

import json
import os

import pytest

from xbrain.common.errors import E_INTERNAL, E_SCHEMA

pytestmark = pytest.mark.no_device

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
_STATIC = os.path.join(_REPO, "xbrain", "p5_gateway", "hmi", "static")
_MIN_WEB = {"push_hz": 20, "map": {}, "font": {}, "layout": {},
            "fence": {}, "route": {}, "waypoint": {}}


class _Provider:
    """Records forwarded uplinks; never answers them (no ack needed here)."""

    def __init__(self):
        self.sent = []

    def snapshot_inputs(self):
        return {"fences": None, "routes": None, "waypoints": None,
                "enu_origin": None, "pose": None, "tasks": None, "mode": None,
                "link": None, "health": None, "events": None}

    def fence_degraded(self):
        return True

    def send_uplink(self, key, payload):
        self.sent.append((key, payload))

    def take_uplink_ack(self, req_id):
        return None


class _Sender:
    """Stands in for main_wiring's _estop_sender: counts calls, returns a cmd_id."""

    def __init__(self, fail=False):
        self.calls = 0
        self.fail = fail

    def __call__(self):
        self.calls += 1
        if self.fail:
            raise RuntimeError("zenoh put failed")
        return "h-estop-test-%d" % self.calls


def _client(sender, on_closed=None):
    from fastapi.testclient import TestClient

    from xbrain.p5_gateway.hmi.web_server import build_app
    return TestClient(build_app(_MIN_WEB, _Provider(), sender, _STATIC,
                                on_session_closed=on_closed))


def _recv_ack(ws, tries=60):
    for _ in range(tries):
        frame = ws.receive_json()
        if frame.get("kind") == "ack":
            return frame
    raise AssertionError("no ack in %d frames" % tries)


# ------------------------------------------------------------ W1 on the WS --

def test_ws_estop_is_forwarded_and_accepted_with_the_cmd_id():
    # RED MUTANT: drop the G0 branch -> the frame reaches the builder map and is
    # answered E_NOT_IMPLEMENTED with sender.calls == 0 (the pre-fix behaviour).
    sender = _Sender()
    with _client(sender).websocket_connect("/ws") as ws:
        ws.send_json({"type": "estop", "action": "stop", "req_id": "e-1"})
        ack = _recv_ack(ws)
    assert sender.calls == 1
    assert ack["req_type"] == "estop" and ack["req_id"] == "e-1"
    assert ack["result"] == "accepted" and ack["code"] == "OK"
    assert ack["detail"]["cmd_id"] == "h-estop-test-1"


def test_ws_estop_without_req_id_or_action_still_stops():
    # 11 S12.1.1 W1: all three fields may be omitted; never E_SCHEMA.
    # RED MUTANT: route estop through parse_envelope (which needs req_id) ->
    # E_SCHEMA and no publish.
    sender = _Sender()
    with _client(sender).websocket_connect("/ws") as ws:
        ws.send_json({"type": "estop"})
        ack = _recv_ack(ws)
    assert sender.calls == 1
    assert ack["result"] == "accepted" and ack["req_id"] is None


def test_truncated_estop_frame_is_forwarded_as_stop():
    # S7.1.2: a truncated JSON is forwarded as stop -- the failure direction is
    # "rather stop by mistake". RED MUTANT: decide estop only on a parsed dict
    # -> this frame is answered E_SCHEMA and nothing is published.
    sender = _Sender()
    with _client(sender).websocket_connect("/ws") as ws:
        ws.send_text('{"type": "estop", "act')
        ack = _recv_ack(ws)
    assert sender.calls == 1
    assert ack["req_type"] == "estop" and ack["result"] == "accepted"


def test_failed_publish_is_reported_not_accepted():
    # The one case that must NOT say accepted: the stop did not go out.
    # RED MUTANT: answer accepted regardless -> the button shows nothing while
    # the robot keeps moving.
    sender = _Sender(fail=True)
    with _client(sender).websocket_connect("/ws") as ws:
        ws.send_json({"type": "estop", "req_id": "e-2"})
        ack = _recv_ack(ws)
    assert ack["result"] == "error" and ack["code"] == E_INTERNAL


def test_estop_bypasses_an_exhausted_rate_limit(monkeypatch):
    # HW-4: estop bypasses the bucket. The bucket is made EMPTY AND NON-REFILLING
    # (capacity 0, rate 0) rather than drained by a burst of frames: a drained
    # bucket refills one token per 100 ms, and the first version of this test
    # let a mutant through because a token had come back by the time the stop
    # arrived. A task frame first proves the bucket really refuses.
    # RED MUTANT: run the bucket before G0 -> the estop is refused too.
    from xbrain.p5_gateway.hmi import web_server
    monkeypatch.setattr(web_server, "UPLINK_BURST", 0)
    monkeypatch.setattr(web_server, "UPLINK_RATE_PER_S", 0.0)
    sender = _Sender()
    with _client(sender).websocket_connect("/ws") as ws:
        ws.send_json({"type": "task", "req_id": "t-1",
                      "action": "pause", "task_id": "t-x"})
        busy = _recv_ack(ws)
        ws.send_json({"type": "estop", "req_id": "e-3"})
        ack = _recv_ack(ws)
    assert busy["req_type"] == "task" and busy["result"] == "rejected"
    assert sender.calls == 1
    assert ack["req_type"] == "estop" and ack["result"] == "accepted"


def test_garbage_frame_is_answered_and_the_socket_stays_usable():
    # 11 S12.1.5: never close the socket over a bad frame (it carries the
    # e-stop). RED MUTANT: restore `return` on a decode error -> the second
    # frame below is never answered and _recv_ack raises.
    sender = _Sender()
    with _client(sender).websocket_connect("/ws") as ws:
        ws.send_text("not json at all")
        bad = _recv_ack(ws)
        ws.send_json({"type": "estop", "req_id": "e-4"})
        good = _recv_ack(ws)
    assert bad["result"] == "rejected" and bad["code"] == E_SCHEMA
    assert bad["req_id"] is None
    assert good["req_id"] == "e-4" and sender.calls == 1


def test_rest_fallback_returns_the_cmd_id():
    sender = _Sender()
    resp = _client(sender).post("/api/estop")
    assert resp.status_code == 200
    assert resp.json()["cmd_id"] == "h-estop-test-1" and sender.calls == 1


# ------------------------------------------------- WS close -> release ------

def test_every_ws_close_calls_the_session_closed_hook_once():
    # WD-1/WD-2: every close, unconditionally -- this tab never drove anything.
    # RED MUTANT: remove the hook call from the finally -> closed == [].
    closed = []
    with _client(_Sender(), closed.append).websocket_connect("/ws") as ws:
        ws.receive_json()                        # the connect keyframe
    assert closed == ["testclient"]


def test_a_failing_hook_does_not_break_the_server():
    # The hook raising must stay inside the guard; a later connection works.
    # RED MUTANT: drop the try/except around the hook -> the exception escapes
    # the WS coroutine and the TestClient re-raises it on exit.
    def boom(_ip):
        raise RuntimeError("publish failed")
    client = _client(_Sender(), boom)
    with client.websocket_connect("/ws") as ws:
        ws.receive_json()
    with client.websocket_connect("/ws") as ws:
        assert ws.receive_json()["kind"] == "state_snapshot"


def test_release_payload_actually_stops_an_active_hmi_teleop_in_p1():
    # The release is judged by P1's real tracker and P1's own device rule
    # (main_wiring _on_teleop: `device`, else source=hmi -> keyboard_hmi).
    # RED MUTANT: the old payload {deadman, vx, vy, wz} -> no device and no
    # source, P1 returns before observe(), the HMI source stays active.
    from xbrain.p1_motion.teleop.state import TeleopTracker
    from xbrain.p5_gateway.hmi.deadman import build_deadman_payload

    tracker = TeleopTracker()
    tracker.observe("keyboard_hmi", now_mono_ms=1000, deadman=True,
                    axes={"vx": 0.5, "vy": 0.0, "wz": 0.0})
    assert tracker.arbitrate(1000) == "keyboard_hmi"

    d = json.loads(json.dumps(build_deadman_payload("10.20.3.7")))
    device = d.get("device")
    if device is None and d.get("source") == "hmi":
        device = "keyboard_hmi"
    assert device is not None, "P1 would ignore this frame"
    tracker.observe(device, now_mono_ms=1050, deadman=bool(d.get("deadman")),
                    axes=d.get("axes") if isinstance(d.get("axes"), dict) else None)
    assert tracker.arbitrate(1050) is None
    assert d["client_id"] == "hmi-10.20.3.7"


def test_start_hmi_wires_release_and_session_closed_event(monkeypatch):
    # The wiring end: the hook build_app receives publishes the release on
    # cmd/teleop and records hmi.session_closed; hmi_state carries the release
    # for the shutdown path (WD-6).
    # RED MUTANT: forget to pass on_session_closed to build_app -> captured
    # hook is None; skip emit_event in the hook -> events == [].
    from xbrain.p5_gateway.hmi import web_server
    from xbrain.p5_gateway.runtime.main_wiring import _start_hmi

    captured = {}
    monkeypatch.setattr(web_server, "make_bound_sockets", lambda bind: [])
    monkeypatch.setattr(web_server, "build_app",
                        lambda *a, **kw: captured.update(kw) or "app")
    monkeypatch.setattr(web_server, "start_in_thread",
                        lambda app, socks: ("srv", "thr"))

    order = []

    class _Pub:
        def __init__(self, key):
            self.key = key

        def put(self, data):
            order.append((self.key, json.loads(data)))

    class _Gen:
        def declare_publisher(self, key):
            return _Pub(key)

    events = []
    hmi_state = {}
    cfg = {"bind": ["127.0.0.1:1"], "web": {"static_dir": "hmi/static"}}
    assert _start_hmi(_Gen(), cfg, hmi_state,
                      emit_event=lambda k, b: events.append((k, b))) == ("srv", "thr")
    hook = captured.get("on_session_closed")
    assert hook is not None
    hook("10.20.3.7")
    teleop = [p for k, p in order if k == "cmd/teleop"]
    assert teleop and teleop[0]["deadman"] is False
    assert teleop[0]["device"] == "keyboard_hmi"
    assert len(events) == 1
    key, body = events[0]
    assert key == "event/info/system"
    assert body["code"] == "HMI_SESSION_CLOSED"
    assert body["detail"]["type"] == "hmi.session_closed"
    assert body["detail"]["client_ip"] == "10.20.3.7"
    assert body["detail"]["session_id"] == hmi_state["hmi_session_id"]
    # WD-6: the shutdown path finds the same release.
    before = len(order)
    hmi_state["hmi_release_teleop"]("p5-shutdown")
    assert order[before][0] == "cmd/teleop"
