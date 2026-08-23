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

from xbrain.p3_task.ingest.geo_command import parse_geo_command
from xbrain.p3_task.ingest.task_command import parse_task_command
from xbrain.p2_core.runtime.mode_wiring import ModeFace
from xbrain.p2_core.runtime.motion_intent_wiring import (
    MotionLimits, evaluate as motion_evaluate, parse_intent_envelope,
    to_relative_move,
)
from xbrain.p3_task.teach.command import parse_teach_command
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.orchestrator_turn import decision_to_publishes
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession, Tier2Classification, TurnOrchestrator,
)
from xbrain.p4_agent.session.chitchat import ChitchatResponder

pytestmark = pytest.mark.no_device

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
        tier2_fn=tier2 or (lambda *a, **k: None), l2_timeout_ms=5000,
        geo_state_fn=lambda: {"state/geo/manifest": {"items": _ITEMS}})


def _frames(text: str, confirm: bool = False, tier2=None):
    """{key: payload} for one utterance, minus the spoken acknowledgment.

    confirm=True runs the L2 two-turn flow (utterance, then a spoken yes),
    which is the ONLY way an operator reaches a destructive geo command.
    """
    orch, session = _orch(tier2), OrchestratorSession()
    decision = orch.handle_turn(text, session, now_mono_ms=1)
    if confirm:
        assert decision.kind == "await_confirm", (
            "%r was expected to open an L2 confirm, got %s"
            % (text, decision.kind))
        decision = orch.handle_turn("是", session, now_mono_ms=2)
    assert decision.kind == "dispatch", (
        "%r did not dispatch (kind=%s) -- the fixture, not the frame, is wrong"
        % (text, decision.kind))
    return {k: p for k, p in decision_to_publishes(decision)
            if k != "cmd/audio/speak"}


def test_voice_task_frame_parses_as_a_task_command():
    """*** cmd/task carries a TaskCommand, not an envelope wrapping one.

    MUTATION: publish dr.payload unchanged (the pre-2026-08-20 behaviour) and
    parse_task_command raises 'task command missing cmd_id' here.
    """
    payload = _frames("开始巡逻")["cmd/task"]
    cmd = parse_task_command(payload)
    assert cmd.action == "submit" and cmd.cmd_id
    assert cmd.task and cmd.task["type"] == "patrol"
    # P3 allocates the id (S7.2 corrected 2026-08-20): voice cannot mint a
    # legal t-YYYYMMDD-NNN because the per-day sequence is P3's.
    assert cmd.task_id is None
    # The envelope's provenance survives the unwrap -- this is what makes
    # dropping the envelope safe rather than lossy.
    assert cmd.task["params"]["intent"] == "patrol_route"
    assert cmd.task["params"]["text"] == "开始巡逻"


def test_voice_teach_frame_parses_as_a_teach_command():
    """*** The F-class path that was broken end to end. Same mutation, same
    failure: nested, this frame has no top-level cmd_id."""
    payload = _frames("开始录制路径")["cmd/teach"]
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
    assert payload["source"] == "voice"


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
    payload = _frames("帮我处理一下那个事情", tier2=tier2)["cmd/motion/intent"]
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
        clock={"ts_sync": True})
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
    cmd = parse_intent_envelope(
        _frames("帮我处理一下那个事情", tier2=tier2)["cmd/motion/intent"])
    assert cmd["slots"]["distance_m"] == 25.0        # 原样, 没被 P4 削
    verdict = motion_evaluate(
        cmd, limits=MotionLimits(max_distance_m=20.0, max_angle_deg=720.0),
        clock={"ts_sync": True})
    assert not verdict.passed and verdict.gate == "G-3"
    assert verdict.detail["limit"] == 20.0


def test_a_control_intent_stays_an_envelope():
    """*** The unwrap must NOT fire on frames with no built command.

    Voice pause/cancel cannot be expressed as an S7.2 TaskCommand (it requires
    task_id, and S7.2 forbids 'omit = the current task'), so those frames stay
    p4_intent_v1 and P3 routes them by the ABSENCE of a top-level `action`.

    MUTATION: unwrapping unconditionally, or making P3 reject anything without
    an action, turns every voice pause into an E_SCHEMA ack.
    """
    payload = _frames("暂停任务")["cmd/task"]
    assert payload["schema"] == "p4_intent_v1"
    assert "action" not in payload


def test_the_wire_check_is_on_the_published_frame_not_the_builder():
    """*** Guards this file's own premise (CLAUDE.md 3.2 form 1).

    If _frames returned the orchestrator's internal payload instead of what
    decision_to_publishes emits, all three tests above would pass while the
    wire stayed broken -- that is precisely how the bug survived. So: assert
    the two DIFFER for a contract key.
    """
    decision = _orch().handle_turn("开始巡逻", OrchestratorSession(),
                                   now_mono_ms=1)
    internal = decision.dispatch_result.payload
    published = _frames("开始巡逻")["cmd/task"]
    assert internal is not published
    assert "task_command" in internal and "task_command" not in published


def test_confirmed_geo_delete_carries_the_command():
    """*** The third contract key, AND the second half of the same bug.

    F11 delete_route is L2, so it goes through the confirm flow -- and the
    confirm path used to call dispatch() directly, skipping slot fill and the
    GeoCommand builder entirely. The operator was asked 'confirm this deletion',
    said yes, and P4 published an envelope naming no object and no action.

    That is the worst possible place for it: every L2 intent is destructive by
    definition, so the commands built most carefully were the ones dispatched
    empty.

    MUTATION: restore the direct dispatch(entry.id, held_text) in
    _resolve_pending_confirm and parse_geo_command raises here.
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
    payload = _frames("帮我处理一下那个事情", confirm=True,
                      tier2=tier2)["cmd/geo"]
    cmd = parse_geo_command(payload)
    assert cmd.action == "delete" and cmd.cmd_id
    # The confirmed command names the object the operator was asked about --
    # not just 'a delete'.
    assert cmd.geo_id == "r-east"
    # origin is the CHANNEL (S7.9.1); voice must not be able to claim cloud.
    assert cmd.origin == "voice"
