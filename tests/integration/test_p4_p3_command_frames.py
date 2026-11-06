"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_p4_p3_command_frames.py
Brief: the frame p4_agent PUBLISHES is the frame p3_task PARSES (11 S7.2/S7.9/S12A)

Description:
p4_agent builds a contract command, and p3_task parses one. Both halves had
tests, both were green, and the bytes between them did not match: the builder's
output travelled NESTED inside p4's own p4_intent_v1 envelope
({schema, intent_id, text, mono_ms, teach_command:{...}}), while the parser
reads cmd_id and action at the TOP level. P3 answered E_SCHEMA to every
F-class voice command, which is a rejection nobody was listening for.

Neither side's unit tests could see this: p4's asserted on the builder's return
value, p3's fed it a hand-written frame. This file asserts on the pair, and it
is the only place that does.

*** It deliberately calls decision_to_publishes -- the function that produces
what actually goes on the wire -- and NOT the builder. Asserting on the builder
here would reproduce the original blind spot exactly.

Boundaries: no Zenoh, no db. What is checked is the SHAPE crossing the process
boundary; whether P3 then accepts the command on its merits (channel matrix,
health gates) is tests/p3_task's business.
"""
from __future__ import annotations

import pytest
import yaml

from xbrain.p2_core.runtime.mode_wiring import ModeFace
from xbrain.p2_core.runtime.motion_intent_wiring import (
    MotionLimits,
    parse_intent_envelope,
    to_relative_move,
)
from xbrain.p2_core.runtime.motion_intent_wiring import (
    evaluate as motion_evaluate,
)
from xbrain.p3_task.ingest.geo_command import parse_geo_command
from xbrain.p3_task.ingest.task_command import parse_task_command
from xbrain.p3_task.teach.command import parse_teach_command
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.orchestrator_turn import decision_to_publishes
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession,
    Tier2Classification,
    TurnOrchestrator,
)
from xbrain.p4_agent.session.chitchat import ChitchatResponder

pytestmark = pytest.mark.no_device

# The ClockStatus PAYLOAD shape, not the envelope: 11 S3.11 gives `sync` and
# P1-13 mirrors it verbatim. Both call sites below said `ts_sync` -- the
# ENVELOPE field (S3.0), a different layer -- which reads as absent, and
# absent is G-4's fail-safe side, so every frame was refused with
# E_UNHEALTHY{item:clock} before reaching the gate each test is about. The
# implementation and tests/p2_core/test_motion_intent_gates.py were corrected
# on 2026-09-26; these two call sites were missed, and only one of them went
# red for it (the other asserts a G-3 refusal, and G-3 runs BEFORE G-4, so it
# kept passing for the wrong reason -- CLAUDE.md 9.1A).
_CLOCK_OK = {"sync": True, "source": "rtk"}

_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"

#: A live catalogue, so the F-class geo intents resolve a spoken name instead of
#: refusing the turn before a frame is ever built.
_ITEMS = [{"geo_id": "r-east", "type": "route", "name": "东门路线",
           "state": "active", "num": 1}]


def _orch(tier2=None):
    return TurnOrchestrator(
        load_intent_registry(
            yaml.safe_load(open(_INTENTS, encoding="utf-8"))),
        chitchat=ChitchatResponder(
            yaml.safe_load(open(_CHITCHAT, encoding="utf-8"))),
        tier2_fn=tier2 or (lambda *a, **k: None),
        state_fn=lambda: {"state/geo/manifest": {"items": _ITEMS}})


#: A running task, as P3 broadcasts it: the 11 S4.4 TaskState shape. B-class
#: control resolves its id from `current` (11 S9.5A), not the old `active_task`.
_RUNNING_TASK = {"state/task": {"schema": "task_state_v1",
                                "current": {"task_id": "t-20260823-004",
                                            "state": "running"},
                                "queue": [], "suspended": []}}


def _frames(text: str, tier2=None):
    """{key: payload} for one utterance, minus the spoken acknowledgment.

    One turn, also for an L2 command: since 99 U147 a voice / text command is
    one question, one answer, so a destructive geo command goes out on the
    turn that asked for it.
    """
    orch, session = _orch(tier2), OrchestratorSession()
    # state/task is part of the same snapshot state_fn returns, so B-class
    # control can resolve a task_id in these frames too.
    orch._state_fn = lambda: {**{"state/geo/manifest": {"items": _ITEMS}},
                              **_RUNNING_TASK}
    decision = orch.handle_turn(text, session, now_mono_ms=1)
    assert decision.kind == "dispatch", (
        "%r did not dispatch (kind=%s) -- the fixture, not the frame, is wrong"
        % (text, decision.kind))
    return {k: p for k, p in decision_to_publishes(decision)
            if k != "cmd/audio/speak"}


#: A patrol the way it reaches cmd/task since 2026-10-25: with a route the
#: catalogue resolves (11 S7.2 patrol needs route_id; p4 refuses a patrol
#: without one, and a bare "开始巡逻" names none). A keyword-matched turn
#: never carries a route slot, so the route comes from the tier-2 leg: an
#: utterance that matches no keyword, and an LLM stub that names 东门路线.
_PATROL_TEXT = "帮我处理一下那个事情"


def _patrol_t2(*_a, **_k):
    """Tier-2 stub: classify as patrol_route on the catalogued 东门路线."""
    return Tier2Classification(name="patrol_route", slots={"route": "东门路线"})


def test_voice_task_frame_parses_as_a_task_command():
    """*** cmd/task carries a TaskCommand, not an envelope wrapping one.

    MUTATION: publish dr.payload unchanged (the pre-2026-08-20 behaviour) and
    parse_task_command raises 'task command missing cmd_id' here.
    """
    payload = _frames(_PATROL_TEXT, tier2=_patrol_t2)["cmd/task"]
    cmd = parse_task_command(payload)
    assert cmd.action == "submit" and cmd.cmd_id
    assert cmd.task and cmd.task["type"] == "patrol"
    # P3 allocates the id (S7.2 corrected 2026-08-20): voice cannot mint a
    # legal t-YYYYMMDD-NNN because the per-day sequence is P3's.
    assert cmd.task_id is None
    # The envelope's provenance survives the unwrap -- this is what makes
    # dropping the envelope safe rather than lossy.
    assert cmd.task["params"]["intent"] == "patrol_route"
    assert cmd.task["params"]["text"] == _PATROL_TEXT
    # The catalogue resolved the spoken route: the geometry P3 dispatches on.
    assert cmd.task["route_id"] == "r-east"


def test_voice_teach_frame_parses_as_a_teach_command():
    """*** The F-class path that was broken end to end. Same mutation, same
    failure: nested, this frame has no top-level cmd_id."""
    payload = _frames("开始录制1号路径")["cmd/teach"]  # 99 U133: a start names the route
    cmd = parse_teach_command(payload)
    assert cmd.action == "start" and cmd.cmd_id


def test_voice_mode_frame_is_applied_by_p2s_real_receiver():
    """*** 18 C class -> cmd/mode, checked against P2's ACTUAL ModeFace.

    Before 2026-08-21 both halves were missing at once: p4 routed the whole C
    class to cmd/task (its prefix table's comment described a different intent
    set entirely), and p2_core never subscribed cmd/mode. Eight voice commands
    that reached nobody, with no error on either side.

    This asserts on the pair rather than on the builder: the frame goes through
    decision_to_publishes and is then fed to the real receiver, which must both
    accept it AND actually change mode.

    MUTATION: leave "C": CMD_TASK in the prefix map (no per-id override) and the
    key here is cmd/task, so the lookup fails. Or drop CMD_MODE from
    _CONTRACT_FRAME_SLOT and the frame arrives nested -> E_SCHEMA.
    """
    import json as _json
    payload = _frames("开始喊话")["cmd/mode"]
    face = ModeFace()
    ack = face.handle_frame(
        _json.dumps(payload).encode("utf-8"), now_mono_ms=1)
    assert ack["result"] == "accepted", ack
    # Accepted is not enough -- the mode must actually have moved. An ack that
    # says accepted while the machine sits in idle is the exact failure SP-C3
    # warns about.
    assert face.state.value == "broadcast"
    # 11 S7.3 spells the microphone "voice_local". Until 2026-10-26 P4 sent
    # "voice" -- not in that closed set -- and this line asserted it; the
    # builder now refuses anything outside S7.3 (runtime/turn_labels).
    assert payload["source"] == "voice_local"


def test_voice_motion_frame_passes_p2s_real_gates():
    """*** 18 A class -> cmd/motion/intent, checked against P2's ACTUAL gates.

    Both halves were broken at once: P4's routing pointed at this key but it
    sent its own p4_intent_v1 envelope (no data wrapper, no intent/slots/
    auth_level/turn_id), and p2_core never subscribed the key at all. So
    "go forward three metres" reached nobody, silently, from both ends.

    MUTATION: drop CMD_MOTION_INTENT from _CONTRACT_FRAME_SLOT and the frame
    arrives as the bare envelope -> parse_intent_envelope raises here.
    """
    tier2 = lambda *a, **k: Tier2Classification(          # noqa: E731
        name="move_forward", slots={"distance_m": 3.0})
    # The number must be IN the utterance since 2026-10-26 (8af46dd): an LLM
    # distance with no source in what was said is dropped as invented. "三号"
    # carries the 3 without a unit, so only the tier-2 slot can deliver it.
    payload = _frames("帮我处理一下那个三号的事情",
                      tier2=tier2)["cmd/motion/intent"]
    # G-1: the envelope is validated and NOT exempted.
    cmd = parse_intent_envelope(payload)
    # *** auth_level must be the contract's "L1", not the registry's L1a/L1b.
    # Passing the registry value straight through would be refused by G-2 as a
    # P4 defect -- it looks like a string detail and rejects the whole command.
    assert cmd["auth_level"] == "L1"
    assert cmd["intent"] == "move_forward"
    assert cmd["slots"] == {"distance_m": 3.0}
    assert cmd["channel"] == "mic_local"
    assert cmd["turn_id"]
    verdict = motion_evaluate(
        cmd, limits=MotionLimits(max_distance_m=20.0, max_angle_deg=720.0),
        clock=_CLOCK_OK)
    assert verdict.passed, verdict
    body = to_relative_move(cmd, rm_cmd_id="rm-1", params={})
    assert body["dx_m"] == 3.0


def test_p4_never_clamps_an_over_range_distance():
    """*** MI-1 with teeth: an over-range value must cross P4 UNCHANGED and be
    refused by P2's G-3.

    A "helpful" clamp in the builder (min(value, 20.0)) is the tempting bug:
    the frame then passes G-3, the robot walks 20 m, and the operator who said
    25 is never told the request was cut. It also breaks the audit premise --
    state/voice_turn would say 25 while the command says 20, and MI-1 exists
    precisely so those two can be compared byte for byte.

    MUTATION: clamp in to_motion_intent -> the G-3 refusal below turns into a
    pass and this test goes red.
    """
    tier2 = lambda *a, **k: Tier2Classification(          # noqa: E731
        name="move_forward", slots={"distance_m": 25.0})
    # "二十五号": 25 said (grounded, see the case above), no unit.
    cmd = parse_intent_envelope(
        _frames("帮我处理一下那个二十五号的事情",
                tier2=tier2)["cmd/motion/intent"])
    assert cmd["slots"]["distance_m"] == 25.0        # 原样, 没被 P4 削
    verdict = motion_evaluate(
        cmd, limits=MotionLimits(max_distance_m=20.0, max_angle_deg=720.0),
        clock=_CLOCK_OK)
    assert not verdict.passed and verdict.gate == "G-3"
    assert verdict.detail["limit"] == 20.0


def test_voice_pause_now_parses_as_a_contract_task_command():
    """*** 2026-08-23: voice pause/resume/cancel DO reach P3 now.

    Until this batch they could not: S7.2 requires task_id and forbids
    "omit = the current task", so P4 sent a bare p4_intent_v1 envelope that P3
    skipped. The fix is to resolve the task_id on the SENDER side from
    state/task -- S7.2 forbids the RECEIVER guessing, not the sender resolving,
    and naming the task is what lets P3 answer E_TASK_STATE instead of pausing
    whatever happens to be running by then.

    MUTATION: drop the resolution and emit the envelope again -> P3's parser
    refuses this frame for having no cmd_id.
    """
    payload = _frames("暂缓任务")["cmd/task"]
    cmd = parse_task_command(payload)
    assert cmd.action == "pause"
    assert cmd.task_id == "t-20260823-004"
    assert cmd.cmd_id


def test_h_class_lands_on_cmd_system_in_contract_shape():
    """*** 18 H class -> cmd/system (11 S7.15), not cmd/task -- in two halves.

    2026-11-05 rewrite. Until then this case expected a reboot turn to DISPATCH
    a cmd/system frame. Since the 2026-10-30 field test (R007-R016) every H
    intent but H03 is refused out loud ("...功能还没有接通执行端") because no
    cmd/system subscriber exists: turn_orchestrator._UNWIRED_SYSTEM. So:

    (1) the whole turn: a reboot sends NOTHING and says why. MUTATION: drop
        H07 from _UNWIRED_SYSTEM -> a frame is sent to a key nobody consumes
        and the operator hears "收到" for a reboot that never happens.
    (2) the frame shape that will be sent once an executor is wired, asserted
        on the builder itself so it does not rot meanwhile: origin is the
        CHANNEL (voice must never claim cloud, U23), and only the action's own
        params are copied (S7.15.1: a reboot must not ship force_step, a
        WALL-CLOCK STEP flag). The hostile slots below keep both mutations
        ("pass origin through", "copy every slot") red.
    """
    from xbrain.p4_agent.runtime.system_request import to_system_command

    tier2 = lambda *a, **k: Tier2Classification(          # noqa: E731
        name="reboot", slots={"origin": "cloud", "delay_s": 5,
                              "scope": "deep", "force_step": True})
    orch, session = _orch(tier2), OrchestratorSession()   # half (1)
    orch._state_fn = lambda: {**{"state/geo/manifest": {"items": _ITEMS}},
                              **_RUNNING_TASK}
    decision = orch.handle_turn("帮我处理一下那个事情", session, now_mono_ms=1)
    assert decision.intent_id == "H07" and decision.kind != "dispatch"
    assert "没有接通执行端" in (decision.reply_text or "")
    assert not [k for k, _ in decision_to_publishes(decision)
                if k == "cmd/system"]
    payload = to_system_command(                          # half (2)
        "reboot", slots={"origin": "cloud", "delay_s": 5, "scope": "deep",
                         "force_step": True},
        cmd_id="c-1", source="voice")
    assert payload["action"] == "reboot"
    assert payload["v"] == 1 and payload["cmd_id"] == "c-1"
    assert payload["origin"] == "voice"
    assert payload["delay_s"] == 5
    assert "scope" not in payload and "force_step" not in payload


def test_h04_reload_config_is_not_on_cmd_system():
    """*** 18 says verbatim that reload_config does NOT go on cmd/system --
    it needs a ConfigCommand on cmd/config (S7.6), a different message body.

    MUTATION: fold H04 into the cmd/system override list and it would ship a
    SystemCommand with an action that is not in S7.15.2's seven-value set.
    """
    from xbrain.p4_agent.runtime.intent_dispatch import choose_key
    assert choose_key("H04") != "cmd/system"


def test_skip_waypoint_is_deleted_and_sends_no_task_frame():
    """*** B10 skip_waypoint was deleted on 2026-11-05 (99 U129, user ruling
    "这个功能不支持, 不需要实现, 删除").

    It never had a contract action (S7.2's closed set is submit / cancel /
    pause / resume / clear_queue -- mapping it onto cancel would end the patrol
    the operator wants to continue), was answered "not connected" from 99 U111,
    and is now gone from the registry. The phrase must still put NOTHING on
    cmd/task. MUTATION: map the phrase to cancel_task -> a task frame -> red.
    """
    orch, session = _orch(None), OrchestratorSession()
    d = orch.handle_turn("跳过这个点", session, now_mono_ms=1)
    assert d.intent_name != "skip_waypoint"   # not an intent any more
    keys = [k for k, _ in decision_to_publishes(d)]
    assert "cmd/task" not in keys   # no task frame


def test_the_wire_check_is_on_the_published_frame_not_the_builder():
    """*** Guards this file's own premise (CLAUDE.md 3.2 form 1).

    If _frames returned the orchestrator's internal payload instead of what
    decision_to_publishes emits, all three tests above would pass while the
    wire stayed broken -- that is precisely how the bug survived. So: assert
    the two DIFFER for a contract key.
    """
    decision = _orch(_patrol_t2).handle_turn(_PATROL_TEXT,
                                             OrchestratorSession(),
                                             now_mono_ms=1)
    internal = decision.dispatch_result.payload
    published = _frames(_PATROL_TEXT, tier2=_patrol_t2)["cmd/task"]
    assert internal is not published
    assert "task_command" in internal and "task_command" not in published


def test_l2_geo_delete_carries_the_command():
    """*** The third contract key: an L2 delete crosses with its object.

    F11 delete_route is L2. Its confirm path once called dispatch() directly,
    skipping slot fill and the GeoCommand builder, and published an envelope
    naming no object and no action. Since 99 U147 the L2 command runs on its
    own turn through the same _dispatch_entry as any other; this pins that the
    frame still names what is deleted.

    MUTATION: dispatch the bare envelope (skip the GeoCommand builder) and
    parse_geo_command raises here.
    """
    # The target slot comes from tier-2: the fastpath keyword matcher fills no
    # geo slots, so a keyword-matched delete names no object at all. Stubbed
    # here rather than run against a live LLM -- what is under test is the frame
    # crossing to P3, not the classifier.
    tier2 = lambda *a, **k: Tier2Classification(          # noqa: E731
        name="delete_route", slots={"route": "东门路线"})
    # A DIRECTED phrase that matches no keyword, so it reaches layer-6 and the
    # stub above decides the intent. Wording it like a real delete would let the
    # keyword matcher pick the intent and the stub would never run.
    payload = _frames("帮我处理一下那个事情", tier2=tier2)["cmd/geo"]
    cmd = parse_geo_command(payload)
    assert cmd.action == "delete" and cmd.cmd_id
    # The command names the object the operator asked about -- not just
    # 'a delete'.
    assert cmd.geo_id == "r-east"
    # origin is the CHANNEL (S7.9.1); voice must not be able to claim cloud.
    assert cmd.origin == "voice"
