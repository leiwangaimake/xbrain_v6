"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_uplink_w2_w7.py
Brief: HMI uplink W2 goto + W7 task -> cmd/task TaskCommand (11 S12.1.1)

Description:
The two classes that let the browser act on tasks: W2 turns a tap on the map
into a goto task, W7 pauses / resumes / cancels one. Both land on cmd/task as
an 11 S7.2 TaskCommand, so what is asserted here is the CONTRACT frame, not a
P5-private shape -- P3 has exactly one receiver and these must fit it.

Each criterion carries a mutation that must turn red (CLAUDE.md 3.3). The
mutations worth naming, because each is a plausible implementation:

  * mapping a retired speed_profile (cruise / transit) onto patrol instead of
    refusing it -- S13.6 (3) forbids reading an off-set value as the nearest
    one, and it would hide a stale frontend from both sides;
  * accepting W7 without task_id and letting it mean "the current task", which
    S12.1.1 and S7.2 both forbid in the same words;
  * taking origin / source from the frame instead of stamping "hmi", which
    under U23 (no auth) hands the browser the whole permission boundary.

Boundaries: pure frame building. No socket, no Zenoh, no clock -- whether P3
then ACCEPTS the task (fence pre-check, task state) is P3's, and deliberately
not re-checked here.
"""
from __future__ import annotations

import pytest

from xbrain.common.errors import E_CONFIRM_REQUIRED, E_SCHEMA
from xbrain.p3_task.ingest.task_command import parse_task_command
from xbrain.p5_gateway.hmi import uplink

pytestmark = pytest.mark.no_device


def _goto(**kw):
    frame = {"type": "goto", "req_id": "r1"}
    frame.update(kw)
    return uplink.build_goto_command(frame)


def _task(**kw):
    frame = {"type": "task", "req_id": "r2"}
    frame.update(kw)
    return uplink.build_task_command(frame)


def _built(result):
    """The payload of a command that was expected to build, or fail loudly."""
    assert isinstance(result, uplink.UplinkCommand), (
        "expected a command, got refusal %r" % (getattr(result, "reason", result),))
    return result.payload


# -- W2 goto -------------------------------------------------------------

def test_goto_by_waypoint_builds_a_goto_task():
    """*** S12.1.1 W2: the HMI's goto MUST become a task.

    Not a BehaviorCommand: that publisher set is p2_core/p3_task only (S2.2.3),
    and going direct would skip P3's fence pre-check and the U07a breakpoint
    ledger -- motion with no fence validation and no record of the interrupt.

    MUTATION: publish cmd/motion/behavior instead and the key here is wrong.
    """
    cmd = _built(_goto(waypoint_id="w-east"))
    assert cmd["action"] == "submit"
    assert cmd["task"]["type"] == "goto"
    assert cmd["task"]["params"]["waypoint_id"] == "w-east"
    # P3 allocates the id (S7.2 corrected 2026-08-20).
    assert "task_id" not in cmd["task"]
    # And it parses as a real TaskCommand -- P3 has one receiver, not a
    # P5-specific one.
    parsed = parse_task_command(cmd)
    assert parsed.action == "submit" and parsed.task["type"] == "goto"


def test_goto_by_coordinates_builds_a_goto_task():
    cmd = _built(_goto(lat=35.012345, lon=135.098765))
    assert cmd["task"]["params"]["lat"] == pytest.approx(35.012345)
    assert cmd["task"]["params"]["lon"] == pytest.approx(135.098765)


def test_waypoint_id_wins_when_both_forms_are_sent():
    """S12.1.1 W2 states the precedence outright: waypoint_id wins.

    Refusing the ambiguity would be wrong -- a frontend sending the tapped
    point alongside the snapped waypoint is being helpful.

    MUTATION: preferring lat/lon puts a raw coordinate on a frame that named a
    waypoint, and P3 would drive to the tap instead of the surveyed point.
    """
    params = _built(_goto(waypoint_id="w-east", lat=1.0, lon=2.0))["task"]["params"]
    assert params["waypoint_id"] == "w-east"
    assert "lat" not in params and "lon" not in params


def test_goto_with_no_target_is_refused():
    """P5's gate G4: neither form present -> E_SCHEMA, retry never."""
    ref = _goto()
    assert isinstance(ref, uplink.UplinkRefusal) and ref.code == E_SCHEMA


@pytest.mark.parametrize("lat,lon", [
    (95.0, 0.0),            # latitude past the pole
    (0.0, 200.0),           # longitude past the antimeridian
    ("35.0", 135.0),        # a string that LOOKS like a coordinate
    (None, 135.0),          # half a pair
])
def test_out_of_range_or_non_numeric_coordinates_are_refused(lat, lon):
    """G4 rejects what the frame alone shows to be wrong. A string is included
    deliberately: float("35.0") would succeed, so a coerce-then-check
    implementation passes it -- and then P3 receives a coordinate whose type
    depends on which client sent it."""
    ref = _goto(lat=lat, lon=lon)
    assert isinstance(ref, uplink.UplinkRefusal) and ref.code == E_SCHEMA


def test_boolean_latitude_is_refused():
    """*** bool is an int in Python: `lat: true` passes isinstance(x, int) and
    would travel on as latitude 1.0 -- a real place ~110 km off the equator,
    which P3 has no way to recognise as a bug.

    MUTATION: drop the isinstance(value, bool) exclusion and this builds.
    """
    ref = _goto(lat=True, lon=True)
    assert isinstance(ref, uplink.UplinkRefusal) and ref.code == E_SCHEMA


@pytest.mark.parametrize("profile", ["cruise", "transit"])
def test_retired_speed_profiles_are_refused_not_downgraded(profile):
    """*** U33 DELETED cruise and transit. S13.6 (3) forbids interpreting an
    off-set value as the nearest one, and the W2 table says so in place:
    "must not be downgraded to patrol".

    MUTATION: map them to patrol. The frame builds, the robot moves, and a
    frontend still on the old vocabulary looks correct to everyone -- which is
    exactly how it survives to the next release.
    """
    ref = _goto(waypoint_id="w-east", speed_profile=profile)
    assert isinstance(ref, uplink.UplinkRefusal)
    assert ref.code == E_SCHEMA
    assert ref.detail["speed_profile"] == profile


def test_valid_speed_profile_passes_through():
    params = _built(_goto(waypoint_id="w-east",
                          speed_profile="obstacle_avoid"))["task"]["params"]
    assert params["speed_profile"] == "obstacle_avoid"


def test_absent_speed_profile_is_not_defaulted():
    """The field is optional (S12.1.1 W2). Absent means absent -- P5 does not
    pick one, because the profile is a motion decision and P3/P1 own it.

    MUTATION: defaulting to "patrol" here would make P5 the quiet author of a
    speed choice nobody in the chain asked it for.
    """
    assert "speed_profile" not in _built(_goto(waypoint_id="w-east"))["task"]["params"]


# -- W7 task -------------------------------------------------------------

@pytest.mark.parametrize("action", ["pause", "resume"])
def test_l0_actions_need_no_confirm(action):
    """pause/resume are L0 (S12.1.1 W7): "hold on a second" is the most
    frequent field intervention, and a dialog costs the seconds it exists to
    save."""
    cmd = _built(_task(action=action, task_id="t-20260730-004"))
    assert cmd["action"] == action
    assert cmd["task_id"] == "t-20260730-004"
    assert parse_task_command(cmd).task_id == "t-20260730-004"


def test_cancel_without_confirm_is_refused():
    """cancel is L2 (18 B07). MUTATION: dropping the level check makes a
    single click end a running task."""
    ref = _task(action="cancel", task_id="t-1")
    assert isinstance(ref, uplink.UplinkRefusal)
    assert ref.code == E_CONFIRM_REQUIRED


def test_cancel_with_confirm_builds():
    cmd = _built(_task(action="cancel", task_id="t-1",
                       confirm={"level": "L2"}))
    assert cmd["action"] == "cancel"


def test_clear_queue_needs_confirm_but_no_task_id():
    """clear_queue is the one action with no task_id -- it is a set operation.
    Still L2, because it acts on tasks the operator cannot all see at once."""
    assert isinstance(_task(action="clear_queue"), uplink.UplinkRefusal)
    cmd = _built(_task(action="clear_queue", confirm={"level": "L2"}))
    assert "task_id" not in cmd


@pytest.mark.parametrize("action", ["pause", "resume", "cancel"])
def test_task_id_is_required(action):
    """*** S12.1.1 W7 and S7.2 both forbid "omit = the current task".

    The queue is live: between the operator reading "A is running" off the panel
    and this frame arriving, A may have ended and B started. The shorthand would
    pause B, and nothing in the record would show that it happened.

    MUTATION: fall back to the current task and this builds a frame that acts on
    whatever happens to be running.
    """
    ref = _task(action=action, confirm={"level": "L2"})
    assert isinstance(ref, uplink.UplinkRefusal) and ref.code == E_SCHEMA


def test_action_outside_the_closed_set_is_refused():
    """S12.1.1 W7's four actions are a closed set. `start` is the tempting one
    to add -- the HMI has no way to start a task except W2 goto -- and adding
    it here rather than to the table is what F-8 forbids."""
    ref = _task(action="start", task_id="t-1")
    assert isinstance(ref, uplink.UplinkRefusal) and ref.code == E_SCHEMA


# -- W3 exit_broadcast ---------------------------------------------------

def test_exit_broadcast_needs_nothing_but_a_req_id():
    """*** W3 carries no preconditions and no confirm, deliberately.

    S12.1.1: it is subject to no task state, no mode state and no L2 confirm --
    it does one thing, leave B mode. The situation it exists for is one where
    the local mic is closed by the half-duplex gate and a cloud operator saying
    "stop broadcasting" would trigger the self-trigger loop, so it is the ONLY
    non-voice exit from B mode. A confirm dialog here would be actively harmful.

    MUTATION: require confirm.level like W7 cancel does -> this goes red, and
    on the robot the operator is left with a broadcasting robot and no way out.
    """
    built = uplink.build_exit_broadcast_command({"type": "exit_broadcast",
                                                 "req_id": "r3"})
    assert isinstance(built, uplink.UplinkCommand)
    # It becomes a ModeCommand on cmd/mode -- P2's ModeFace maps the action to
    # IDLE. Not a payload/audio command: leaving B mode is a MODE change.
    assert built.key == "cmd/mode"
    assert built.payload["action"] == "exit_broadcast"
    assert built.payload["source"] == "hmi"
    assert built.payload["cmd_id"] == "h-r3"


def test_exit_broadcast_is_accepted_by_p2s_real_mode_face():
    """*** The pair, not just the builder: P5 builds it, P2 must apply it.

    MUTATION: emit action "stop_broadcast" (a plausible spelling that is NOT in
    S7.3's six-value closed set) and P2 refuses with E_SCHEMA.
    """
    import json as _json
    from xbrain.p2_core.mode.state_machine import ModeState, ModeStateMachine
    from xbrain.p2_core.runtime.mode_wiring import ModeFace
    built = uplink.build_exit_broadcast_command({"type": "exit_broadcast",
                                                 "req_id": "r3"})
    face = ModeFace(ModeStateMachine(ModeState.BROADCAST))
    ack = face.handle_frame(_json.dumps(built.payload).encode("utf-8"),
                            now_mono_ms=1)
    assert ack["result"] == "accepted"
    assert face.state is ModeState.IDLE


# -- both: the fields P5 stamps and never reads -------------------------

@pytest.mark.parametrize("build,extra", [
    (uplink.build_task_command, {"action": "pause", "task_id": "t-1"}),
    (uplink.build_goto_command, {"waypoint_id": "w-east"}),
])
def test_source_is_stamped_hmi_even_when_the_frame_claims_cloud(build, extra):
    """*** CH-2. The browser is UNAUTHENTICATED (U23), so it can put anything
    in this field -- and `source` is what 15 S4.2 maps onto the task priority
    the scheduler orders by. A frame claiming "cloud" would submit at priority
    80 instead of 40, outranking real cloud work.

    The frame here CLAIMS cloud on purpose: with no source field at all, a
    pass-through implementation and a stamping one give the same answer, and
    the mutation would not turn red. That is the trap this file's W4 sibling
    fell into once already.

    MUTATION: source = msg.get("source", "hmi").
    """
    frame = {"type": "x", "req_id": "r9", "source": "cloud", "origin": "cloud",
             "confirm": {"level": "L2"}}
    frame.update(extra)
    assert _built(build(frame))["source"] == "hmi"


@pytest.mark.parametrize("build,extra", [
    (uplink.build_task_command, {"action": "pause", "task_id": "t-1"}),
    (uplink.build_goto_command, {"waypoint_id": "w-east"}),
])
def test_cmd_id_is_the_prefixed_req_id(build, extra):
    """S12.1.1: cmd_id is "h-" + req_id. That prefix is how P5 recognises the
    ack as its own on a key that also carries voice and cloud answers -- see
    main_wiring's _on_uplink_ack, which drops anything without it."""
    frame = {"type": "x", "req_id": "abc"}
    frame.update(extra)
    assert _built(build(frame))["cmd_id"] == "h-abc"
