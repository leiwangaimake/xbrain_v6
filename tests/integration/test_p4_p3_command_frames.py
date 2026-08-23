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
        state_fn=lambda: {"state/geo/manifest": {"items": _ITEMS}})


#: A running task, as P3 broadcasts it. B-class control resolves its id here.
_RUNNING_TASK = {"state/task": {"schema": "state_task_v1",
                                "active_task": {"task_id": "t-20260823-004",
                                                "state": "running"}}}


def _frames(text: str, confirm: bool = False, tier2=None):
    """{key: payload} for one utterance, minus the spoken acknowledgment.

    confirm=True runs the L2 two-turn flow (utterance, then a spoken yes),
    which is the ONLY way an operator reaches a destructive geo command.
    """
    orch, session = _orch(tier2), OrchestratorSession()
    # state/task is part of the same snapshot state_fn returns, so B-class
    # control can resolve a task_id in these frames too.
    orch._state_fn = lambda: {**{"state/geo/manifest": {"items": _ITEMS}},
                              **_RUNNING_TASK}
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
    payload = _frames("暂停任务")["cmd/task"]
    cmd = parse_task_command(payload)
    assert cmd.action == "pause"
    assert cmd.task_id == "t-20260823-004"
    assert cmd.cmd_id


def test_h_class_lands_on_cmd_system_in_contract_shape():
    """*** 18 H class -> cmd/system (11 S7.15), not cmd/task.

    The prefix table had "H": CMD_TASK -- the same mistake as the C class --
    so eight system commands went to P3, which skipped them for having no
    top-level action.

    Routed correctly they are still NOT consumed: no cmd/system subscriber
    exists yet (S2.2.3 splits it across three by action, SYS-1). Asserted here
    anyway, because "right key, right shape, waiting for a subscriber" and
    "wrong key, actively dropped" look identical to the operator and totally
    different during bring-up.
    """
    # *** The stub slots CLAIM cloud and carry params reboot does not use.
    # Without them this test is vacuous: with no origin field in the frame,
    # "pass the frame's origin through" and "hard-code voice" give the same
    # answer, and the mutation stays green. Same for the param filter -- with
    # no extra slots, "copy only this action's params" and "copy everything"
    # are indistinguishable. Both mutations were green until this fixture
    # carried the hostile values.
    tier2 = lambda *a, **k: Tier2Classification(          # noqa: E731
        name="reboot", slots={"origin": "cloud", "delay_s": 5,
                              "scope": "deep", "force_step": True})
    payload = _frames("帮我处理一下那个事情", confirm=True,
                      tier2=tier2)["cmd/system"]
    assert payload["action"] == "reboot"
    assert payload["v"] == 1 and payload["cmd_id"]
    # origin is one of only two authorisation boundaries in the whole system
    # (U23: the HMI is unauthenticated, so the channel IS the permission).
    # Voice must never be able to claim cloud.
    assert payload["origin"] == "voice"
    # reboot uses delay_s and nothing else. S7.15.1: params an action does not
    # use are simply not filled -- copying the whole slot bag would ship a
    # force_step (a WALL-CLOCK STEP flag, L2) on a reboot command.
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


def test_skip_waypoint_still_has_no_contract_action():
    """*** B10 skip_waypoint is deliberately NOT mapped.

    18's effect column says "P3 path advance", and S7.2's action closed set is
    submit / cancel / pause / resume / clear_queue -- there is no skip. Mapping
    it onto cancel would end the whole patrol the operator wants to continue.

    So it stays a p4_intent_v1 envelope that P3 skips, which is a visible gap
    rather than a wrong action. MUTATION: map it to cancel and this test goes
    red -- as it should, because that mapping is the dangerous one.
    """
    payload = _frames("跳过这个点")["cmd/task"]
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
