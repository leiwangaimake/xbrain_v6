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
