"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cloud_ptz.py
Brief: Cloud xbrain/{rid}/cmd/ptz -- contract v0.2 validation, mapping, renewal, R-6, ack translation, bridge wiring

Description:
The gateway half of the Qt PTZ contract (docs/MISSON/PTZ控制契约_qt端v0.2草案.md,
99 U98). Two layers are pinned:
  * the pure router (inbound/ptz_router.py): every contract S3 rule that is
    decided at the gateway, the S7 mapping onto the in-robot PtzCommand, the
    per-task renewal / change rules, the R-6 rate limit, the S5 translation of
    p2's ack, the S2 ack time limits, and "one reject event per task";
  * the real CloudBridge wiring with a fake Zenoh session: a cloud frame
    reaches the in-robot cmd/ptz key, p2's ack comes back on the cloud ack key
    in the v2.0 shape, a repeated msg_id is answered duplicate, and a frame
    with no p2 answer is answered 2001 by tick (never silence).
Each case names the mutant that turns it red (CLAUDE.md 3.3). No device.
"""
from __future__ import annotations  # postponed annotations

import json  # fake Zenoh payloads are JSON

import pytest  # test runner

from xbrain.common import errors  # closed E_* set
from xbrain.p5_gateway.inbound.ptz_router import CloudPtzRouter  # unit under test

pytestmark = pytest.mark.no_device  # runs without any device

RID = "gj-001"  # the robot id every key carries


class Clock:  # fake clock
    """Fake monotonic seconds."""

    def __init__(self):  # starts at 1000 s
        self.t = 1000.0  # arbitrary start

    def __call__(self):  # now_mono() callable
        return self.t  # now


def router():  # router factory
    """A router on a fake clock; r.clock moves time."""
    clock = Clock()  # time moves only when a case says so
    r = CloudPtzRouter(now_mono=clock, cmd_prefix="c-")  # bridge prefix
    r.clock = clock  # handle for the cases
    return r  # ready


def data(action, msg="m-1", task="t-1", **payload):  # data object factory
    """A contract S3.1 data object."""
    payload["action"] = action  # every payload names its action
    return {"msg_id": msg, "task_id": task, "task_type": "PTZ_CONTROL",  # contract S3.1 fields
            "payload": payload}  # no envelope: the bridge strips it


def move(direction="up", msg="m-1", task="t-1", speed=1.0, hold_ms=800):  # move factory
    """A contract S3.2 move."""
    return data("move", msg, task, direction=direction, speed=speed, hold_ms=hold_ms)  # contract S3.2 fields


def code(out):  # answer triple
    """(error_code, detail.code, detail.reason) of an immediate answer."""
    d = out.ack["detail"]  # contract S5 detail
    return out.ack["error_code"], d["code"], d.get("reason")  # the triple the cases check


# -- S7 mapping ------------------------------------------------------------------

def test_move_diagonal_maps_to_one_jog():  # S7 mapping
    """up_left -> jog pan -s / tilt +s / zoom 0 (11 S7.4.4 signs), source
    cloud, cmd_id c-<msg_id>. MUTATION: swap the pan sign -> red."""
    r = router()  # fresh router
    out = r.handle(move("up_left", speed=0.8))  # one press
    assert out.ack is None and out.pending_cmd_id == "c-m-1"  # waits for p2
    cmd = out.forward[0]  # the in-robot command
    assert cmd == {"cmd_id": "c-m-1", "action": "jog", "source": "cloud", "hold_ms": 800,  # exact in-robot shape
                   "vel": {"pan": -0.8, "tilt": 0.8, "zoom": 0.0}}  # contract S7 row 1


def test_zoom_and_zoom_step_map_to_jog_and_nudge():  # S7 mapping
    """zoom in -> jog zoom +; zoom_step out -> nudge zoom - with pulse_ms."""
    r = router()  # fresh router
    z = r.handle(data("zoom", direction="in", speed=0.5, hold_ms=800)).forward[0]  # zoom in
    assert z["action"] == "jog" and z["vel"] == {"pan": 0.0, "tilt": 0.0, "zoom": 0.5}  # jog on the zoom axis
    s = r.handle(data("zoom_step", "m-2", "t-2", direction="out", speed=0.6,  # zoom step out
                      pulse_ms=1000)).forward[0]  # one step
    assert s["action"] == "nudge" and s["vel"]["zoom"] == -0.6 and s["pulse_ms"] == 1000  # nudge with its pulse


def test_refocus_and_stop_map_through():  # S7 mapping
    """refocus -> refocus; stop with axes -> stop with the same axes."""
    r = router()  # fresh router
    assert r.handle(data("refocus")).forward[0]["action"] == "refocus"  # refocus passes through
    st = r.handle(data("stop", "m-2", axes=["pan", "tilt"])).forward[0]  # partial stop
    assert st["action"] == "stop" and st["axes"] == ["pan", "tilt"]  # partial stop keeps its axes


# -- per-task renewal / change ----------------------------------------------------

def test_renewal_forwards_a_jog_with_its_own_cmd_id():  # R-3
    """Same task, same params -> forwarded again (p2 sees a renewal), each
    with the renewal's own cmd_id (contract R-3)."""
    r = router()  # fresh router
    r.handle(move(msg="m-1"))  # press
    out = r.handle(move(msg="m-2"))  # renewal 250 ms later
    assert out.forward[0]["cmd_id"] == "c-m-2" and out.ack is None  # own id, waits for p2


def test_change_within_a_task_is_refused_and_stops():  # S3.2
    """Contract S3.2: direction changed inside one task -> 1003
    params_changed_within_task AND a stop is forwarded. MUTATION: forward
    the new direction -> a jog instead of a stop -> red."""
    r = router()  # fresh router
    r.handle(move("up", msg="m-1"))  # press up
    out = r.handle(move("left", msg="m-2"))  # same task, new direction
    assert code(out) == (1003, "E_INVALID_FIELD", "params_changed_within_task")  # contract reason
    assert [c["action"] for c in out.forward] == ["stop"]  # head stopped


def test_new_task_after_stop_starts_cleanly():  # task lifecycle
    """stop ends the task; the next press with a new task_id is a start."""
    r = router()  # fresh router
    r.handle(move("up", msg="m-1", task="t-1"))  # press
    r.handle(data("stop", "m-2", task="t-1"))  # release
    out = r.handle(move("left", msg="m-3", task="t-2"))  # next press
    assert out.forward[0]["action"] == "jog" and out.ack is None  # a start, not a refusal


# -- S3.7 gateway rejections ---------------------------------------------------------

@pytest.mark.parametrize("payload,expected", [  # one case per contract row
    ({"action": "zoom", "direction": "in", "speed": 1.0, "hold_ms": 800, "zoom_level": 10},  # absolute zoom
     (1003, "E_INVALID_FIELD", "no_absolute_zoom")),  # 1003 no_absolute_zoom
    ({"action": "move", "direction": "up", "speed": 1.0, "hold_ms": 800, "pan_deg": 30},  # angle
     (1003, "E_INVALID_FIELD", "no_absolute_positioning")),  # 1003 no_absolute_positioning
    ({"action": "zoom", "direction": "in", "speed": 1.0, "hold_ms": 800, "camera": "cam_ptz_ir"},  # IR zoom
     (1006, "E_TASK_UNSUPPORTED", "ir_fixed_focal")),  # 1006 ir_fixed_focal
    ({"action": "refocus", "camera": "cam_ptz_ir"},  # IR refocus
     (1006, "E_TASK_UNSUPPORTED", "ir_fixed_focal")),  # 1006 ir_fixed_focal
    ({"action": "move", "direction": "up", "speed": 1.0, "hold_ms": 800, "camera": "cam_ptz_vis"},  # move with a camera
     (1003, "E_INVALID_FIELD", "shared_head")),  # 1003 shared_head
    ({"action": "refocus", "bitrate": 16384},  # camera configuration
     (1006, "E_TASK_UNSUPPORTED", "camera_config_robot_managed")),  # 1006 camera_config_robot_managed
    ({"action": "goto_preset"}, (1006, "E_TASK_UNSUPPORTED", "action_not_open")),  # not opened
    ({"action": "fly"}, (1003, "E_INVALID_FIELD", None)),  # unknown action
    ({"action": "move", "direction": "north", "speed": 1.0, "hold_ms": 800},  # unknown direction
     (1003, "E_INVALID_FIELD", None)),  # 1003
    ({"action": "move", "direction": "up", "speed": 0, "hold_ms": 800},  # non-positive speed
     (1003, "E_INVALID_FIELD", None)),  # 1003
    ({"action": "move", "direction": "up", "speed": 1.0},  # missing hold_ms
     (1002, "E_REQUIRED_FIELD", None)),  # 1002
])  # end of table
def test_gateway_rejections_follow_the_contract(payload, expected):  # S3.7 / S5
    """Contract S3.7 / S5, decided at the gateway; nothing is forwarded.
    MUTATION: let absolute zoom reach p2 -> it would come back as 2002 -> red."""
    r = router()  # fresh router
    out = r.handle({"msg_id": "m-1", "task_id": "t-1", "task_type": "PTZ_CONTROL",  # one frame
                    "payload": payload})  # one bad frame
    assert code(out) == expected and out.forward == []  # refused, nothing sent


def test_clamps_are_reported_in_the_ack():  # clamps
    """speed 1.5 -> 1.0 and hold 5000 -> 2000, both named in detail.clamped."""
    r = router()  # fresh router
    out = r.handle(move(speed=1.5, hold_ms=5000))  # out of range values
    assert out.forward[0]["hold_ms"] == 2000 and out.forward[0]["vel"]["tilt"] == 1.0  # clamped values forwarded
    fields, *_ = r.translate({"cmd_id": "c-m-1", "result": "accepted", "code": "OK",  # p2 accepts
                              "detail": {"renewed": False}})  # p2 accepts
    assert fields["detail"]["clamped"] == ["hold_ms", "speed"]  # both clamps reported


def test_bad_stop_is_refused_but_still_stops_everything():  # S3.6
    """Contract S3.6: unknown axis -> 1003, and a FULL stop goes out anyway.
    MUTATION: refuse without forwarding -> no stop -> red."""
    r = router()  # fresh router
    out = r.handle(data("stop", axes=["roll"]))  # bad axis name
    assert code(out)[0] == 1003  # refused
    assert out.forward[0]["action"] == "stop" and "axes" not in out.forward[0]  # full stop, no axes


# -- R-6 rate limit ----------------------------------------------------------------------

def test_over_the_rate_a_renewal_is_coalesced_but_stop_goes_out():  # R-6
    """8 forwarded in one second; the 9th renewal is answered accepted +
    coalesced without forwarding; a stop is still forwarded.
    MUTATION: forward everything -> 9 forwards -> red."""
    r = router()  # fresh router
    for i in range(8):  # eight renewals of one press, same second
        assert r.handle(move(msg="m-%d" % i)).forward  # all forwarded
    out = r.handle(move(msg="m-8"))  # the ninth
    assert out.forward == [] and out.ack["detail"]["coalesced"] is True  # merged, not forwarded
    assert r.handle(data("stop", "m-9")).forward[0]["action"] == "stop"  # never coalesced


def test_rate_window_slides():  # R-6 window
    """After a second the window has room again."""
    r = router()  # fresh router
    for i in range(8):  # fill the window
        r.handle(move(msg="m-%d" % i))  # forwarded
    r.clock.t += 1.1  # a second later
    assert r.handle(move(msg="m-9")).forward  # forwarded again


# -- S5 translation of p2's ack ----------------------------------------------------------

def test_accepted_ack_carries_contract_fields():  # S5 accepted
    """renewed / hold_ms / clamped / coalesced as contract S5 lists them."""
    r = router()  # fresh router
    r.handle(move(msg="m-1"))  # press
    fields, ref, task, ev = r.translate({"cmd_id": "c-m-1", "result": "accepted", "code": "OK",  # p2 accepts a renewal
                                         "detail": {"renewed": True, "coalesced": True}})  # with a merge on its side
    assert (ref, task, ev) == ("m-1", "t-1", False)  # routed back to the frame
    assert fields["detail"] == {"action": "move", "renewed": True, "hold_ms": 800,  # contract S5 detail
                                "clamped": [], "coalesced": True}  # exact field set


def test_busy_maps_to_2002_with_holder():  # S5 busy
    """E_BUSY -> 2002 E_BUSY with holder and holder_priority (11 S7A.0)."""
    r = router()  # fresh router
    r.handle(move())  # press
    fields, *_ = r.translate({"cmd_id": "c-m-1", "result": "rejected", "code": errors.E_BUSY,  # p2 refuses
                              "detail": {"holder": "voice_local", "holder_priority": 700}})  # held by voice
    assert fields["error_code"] == 2002 and fields["detail"]["holder"] == "voice_local"  # 2002 with holder


def test_refocus_failure_maps_to_2001_not_ready():  # S5 refocus failure
    """E_UNHEALTHY refocus_failed -> 2001 E_NOT_READY keeping reason and
    focus_mode_restored. MUTATION: pass the internal code through -> red."""
    r = router()  # fresh router
    r.handle(data("refocus"))  # refocus
    fields, *_ = r.translate({"cmd_id": "c-m-1", "result": "rejected", "code": errors.E_UNHEALTHY,  # p2 refuses
                              "detail": {"item": "ptz", "reason": "refocus_failed",  # refocus details
                                         "focus_mode_restored": False}})  # auto not restored
    d = fields["detail"]  # contract S5 row
    assert fields["error_code"] == 2001 and d["code"] == "E_NOT_READY"  # 2001 E_NOT_READY
    assert d["reason"] == "refocus_failed" and d["focus_mode_restored"] is False  # details passed through


def test_not_implemented_maps_to_1006():  # S5 not driven
    """E_NOT_IMPLEMENTED -> 1006 E_TASK_UNSUPPORTED."""
    r = router()  # fresh router
    r.handle(move())  # press
    fields, *_ = r.translate({"cmd_id": "c-m-1", "result": "rejected",  # p2 refuses
                              "code": errors.E_NOT_IMPLEMENTED, "detail": {}})  # not driven
    assert (fields["error_code"], fields["detail"]["code"]) == (1006, "E_TASK_UNSUPPORTED")  # 1006 E_TASK_UNSUPPORTED


# -- S2 ack time limits, reject-event dedup -----------------------------------------------

def test_no_p2_answer_expires_at_500ms_refocus_at_2s():  # S2 time limits
    """move answers 2001 after 0.5 s; refocus only after 2 s.
    MUTATION: one timeout for all -> refocus expires early -> red."""
    r = router()  # fresh router
    r.handle(move(msg="m-1"))  # waits up to 0.5 s
    r.handle(data("refocus", "m-2", "t-2"))  # waits up to 2 s
    r.clock.t += 0.6  # past 0.5 s
    first = r.expired()  # what timed out
    assert [m for _f, m, _t in first] == ["m-1"] and first[0][0]["error_code"] == 2001  # only the move expired
    r.clock.t += 1.5  # past 2 s
    assert [m for _f, m, _t in r.expired()] == ["m-2"]  # then the refocus


def test_reject_event_once_per_task():  # S5 reject dedup
    """Two rejections in one press -> one audit event (contract S5)."""
    r = router()  # fresh router
    bad = {"action": "move", "direction": "north", "speed": 1.0, "hold_ms": 800}  # a direction outside the set
    first = r.handle({"msg_id": "m-1", "task_id": "t-1", "task_type": "PTZ_CONTROL",  # first refusal
                      "payload": bad})  # first refusal
    second = r.handle({"msg_id": "m-2", "task_id": "t-1", "task_type": "PTZ_CONTROL",  # same press again
                       "payload": bad})  # same task again
    assert first.reject_event is True and second.reject_event is False  # one event


# -- the real bridge with a fake Zenoh session ---------------------------------------------

class _Sample:  # zenoh sample stand-in
    """What a Zenoh subscriber callback receives."""

    def __init__(self, key, body):  # key + JSON bytes
        self.key_expr = key  # absolute or relative key
        self.payload = json.dumps(body).encode("utf-8")  # bytes like zenoh


class _Pub:  # publisher stand-in
    """Records puts on one declared key."""

    def __init__(self, key, log):  # records on put
        self.key, self.log = key, log  # key + shared log

    def put(self, payload):  # same signature as zenoh
        self.log.append((self.key, payload))  # recorded in order


class _Session:  # session stand-in
    """Records declarations and puts."""

    def __init__(self):  # subs + puts
        self.subs, self.puts = {}, []  # key -> callback, (key, bytes)

    def declare_subscriber(self, key, cb):  # kept for feeding
        self.subs[key] = cb  # kept for feeding
        return object()  # a handle the bridge must hold

    def declare_publisher(self, key):  # records on put
        return _Pub(key, self.puts)  # records on put

    def put(self, key, payload):  # session-level put
        self.puts.append((key, payload))  # session-level put


def _bridge():  # real bridge factory
    """A wired CloudBridge on a fake clock."""
    from xbrain.p5_gateway.runtime.cloud_wiring import CloudBridge  # the bridge under test
    s, clock = _Session(), {"t": 1000.0}  # session + clock
    b = CloudBridge(s, RID, now_mono=lambda: clock["t"])  # real bridge
    b.wire()  # declares everything
    return b, s, clock  # bridge, session, clock


def _frame(msg_id, payload, task_id="t-1"):  # full Qt envelope
    """A full v2.0 envelope from Qt."""
    return {"v": 1, "rid": RID, "ts": 1785733000.0, "seq": 1, "src": "qt_hmi",  # v2.0 S1.1 envelope
            "data": {"msg_id": msg_id, "task_id": task_id, "task_type": "PTZ_CONTROL",  # data object
                     "payload": payload}}  # the action


def _out(s, key):  # decoded puts
    """Decoded puts on one key."""
    return [json.loads(p) for k, p in s.puts if k == key]  # in put order


def test_bridge_forwards_and_translates_p2_ack():  # end to end
    """Cloud frame -> in-robot cmd/ptz; p2 ack -> v2.0 ack on the cloud key.
    MUTATION: subscribe the in-robot ack on the cloud key -> no ack -> red."""
    b, s, _c = _bridge()  # real wiring
    s.subs["xbrain/%s/cmd/ptz" % RID](_Sample("xbrain/%s/cmd/ptz" % RID, _frame(  # Qt sends a press
        "m-1", {"action": "move", "direction": "right", "speed": 1.0, "hold_ms": 800})))  # move right
    cmd = _out(s, "cmd/ptz")[-1]  # what p2 receives
    assert cmd["action"] == "jog" and cmd["vel"]["pan"] == 1.0  # p2 receives a jog
    s.subs["cmd/ptz/ack"](_Sample("cmd/ptz/ack", {"schema": "ptz_ack_v1", "cmd_id": cmd["cmd_id"],  # p2 answers
                                                    "result": "accepted", "code": "OK",  # accepted
                                                    "detail": {"renewed": False}}))  # not a renewal
    ack = _out(s, "xbrain/%s/cmd/ptz/ack" % RID)[-1]["data"]  # what Qt receives
    assert ack["ref_msg_id"] == "m-1" and ack["task_type"] == "PTZ_CONTROL"  # routed back to the frame
    assert ack["result"] == "accepted" and ack["detail"]["action"] == "move"  # translated for Qt


def test_bridge_answers_duplicate_msg_id():  # v2.0 S1.2
    """The same msg_id twice -> duplicate (v2.0 S1.2), forwarded once."""
    b, s, _c = _bridge()  # real wiring
    f = _frame("m-1", {"action": "refocus"})  # one frame
    for _ in range(2):  # sent twice
        s.subs["xbrain/%s/cmd/ptz" % RID](_Sample("xbrain/%s/cmd/ptz" % RID, f))  # fed twice
    assert len(_out(s, "cmd/ptz")) == 1  # forwarded once
    assert _out(s, "xbrain/%s/cmd/ptz/ack" % RID)[-1]["data"]["result"] == "duplicate"  # second answered duplicate


def test_bridge_answers_2001_when_p2_is_silent():  # S2 silence
    """No p2 ack in 0.5 s -> tick answers 2001 E_NOT_READY (never silence).
    MUTATION: drop the PTZ part of tick -> no ack -> red."""
    b, s, clock = _bridge()  # real wiring
    s.subs["xbrain/%s/cmd/ptz" % RID](_Sample("xbrain/%s/cmd/ptz" % RID, _frame(  # Qt sends a zoom
        "m-1", {"action": "zoom", "direction": "in", "speed": 1.0, "hold_ms": 800})))  # zoom in
    clock["t"] += 0.6  # past the move/zoom ack limit
    b.tick()  # the gateway's periodic step
    ack = _out(s, "xbrain/%s/cmd/ptz/ack" % RID)[-1]["data"]  # what Qt receives
    assert ack["error_code"] == 2001 and ack["detail"]["code"] == "E_NOT_READY"  # 2001 E_NOT_READY from tick
