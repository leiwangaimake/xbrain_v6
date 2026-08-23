"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_orchestrator.py
Brief: GWY-P4-38 (32.F) -- turn orchestrator: bypass / L2 confirm / fastpath

Description:
Tests the six-step turn orchestrator against the real registry. Each
criterion carries a mutation that must turn red per CLAUDE.md 3.3:
estop bypasses classify, L2 waits for confirm, fastpath never touches the
LLM.
"""
from __future__ import annotations

import pytest
import yaml

from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.session.chitchat import ChitchatResponder
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession, Tier2Classification, TurnOrchestrator,
    refine_ptz_intent,
)

pytestmark = pytest.mark.no_device

_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"


def _reg():
    return load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))


def _chitchat():
    return ChitchatResponder(yaml.safe_load(open(_CHITCHAT, encoding="utf-8")))


class _RecordingTier2:
    """Stub tier-2. Records calls; returns a preset classified name."""

    def __init__(self, ret=None):
        self.calls = []
        self.ret = ret

    def __call__(self, text, session, now_mono_ms):
        self.calls.append(text)
        return self.ret


def _orch(tier2=None):
    return TurnOrchestrator(
        _reg(), chitchat=_chitchat(),
        tier2_fn=tier2 or _RecordingTier2(), l2_timeout_ms=5000)


# -- criterion 1: estop bypasses classify entirely -----------------------

def test_estop_bypasses_classify():
    t2 = _RecordingTier2()
    orch = _orch(t2)
    d = orch.handle_turn("急停", OrchestratorSession(), now_mono_ms=1000)
    assert d.kind == "bypass"
    assert d.bypass_action == "estop"
    assert d.route == "bypass"
    # It never reached the classifier or the LLM.
    assert t2.calls == []


def test_estop_bypass_is_not_a_classified_intent():
    """MUTATION A guard: if the orchestrator ran the classify chain FIRST,
    '急停' would fall through to overheard/unknown (bypass keywords are
    excluded from the layer-2 index) -- NOT a bypass. Getting kind==bypass
    proves the safety match runs before classification."""
    orch = _orch()
    d = orch.handle_turn("现在马上急停", OrchestratorSession(), now_mono_ms=1)
    assert d.kind == "bypass" and d.bypass_action == "estop"


def test_recording_suppresses_voice_estop():
    orch = _orch()
    s = OrchestratorSession()
    s.recording.in_recording = True
    d = orch.handle_turn("急停", s, now_mono_ms=1000)
    assert d.kind == "bypass_suppressed"
    assert d.tts_text                       # advises the handle estop


# -- criterion 2: auth=L2 waits for confirm ------------------------------

def test_l2_intent_awaits_confirm_not_dispatched():
    """B07 cancel_task is auth=L2. Heard once, it must open a confirm and
    NOT dispatch. MUTATION B: dispatch immediately => kind would be
    'dispatch' here."""
    orch = _orch()
    s = OrchestratorSession()
    d = orch.handle_turn("不巡了", s, now_mono_ms=1000)
    assert d.kind == "await_confirm"
    assert d.intent_id == "B07"
    assert d.auth == "L2"
    assert d.dispatch_result is None        # nothing dispatched
    assert s.pending_confirm is not None    # confirm is open


#: A live state snapshot with one running task, as P3 broadcasts it
#: (state_task_v1 + active_task). B-class control resolves its task_id here.
_RUNNING_TASK = {"state/task": {"schema": "state_task_v1",
                                "active_task": {"task_id": "t-20260823-004",
                                                "state": "running"}}}


def test_l2_confirm_then_dispatch():
    """*** B07 cancel: L2 confirm, then a TaskCommand naming a REAL task_id.

    The task_id is resolved on the SENDER side from state/task. S7.2 forbids
    "omit = the current task" because it forbids the RECEIVER guessing; naming
    the task in the frame is what makes P3 answer E_TASK_STATE instead of
    quietly cancelling whatever happens to be running now.

    MUTATION: send the frame without task_id (or with the id left None) and P3
    is back to guessing -- parse_task_command refuses it outright.
    """
    orch = TurnOrchestrator(
        _reg(), chitchat=_chitchat(), tier2_fn=_RecordingTier2(),
        l2_timeout_ms=5000, state_fn=lambda: _RUNNING_TASK)
    s = OrchestratorSession()
    orch.handle_turn("不巡了", s, now_mono_ms=1000)          # opens confirm
    d = orch.handle_turn("确认", s, now_mono_ms=1500)        # I01 -> dispatch
    assert d.kind == "dispatch"
    assert d.intent_id == "B07"
    assert d.dispatch_result is not None
    assert s.pending_confirm is None
    tc = d.dispatch_result.payload["task_command"]
    assert tc["action"] == "cancel"
    assert tc["task_id"] == "t-20260823-004"


def test_task_control_with_no_active_task_is_spoken_not_sent():
    """*** Nothing running -> say so; NEVER send a task_id-less control frame.

    A frame with no task_id hands the "which one?" decision to P3, which is
    exactly what S7.2 forbids.

    MUTATION: fall back to an empty task_id and dispatch anyway -- this turns
    into a dispatch and the guard is gone.
    """
    orch = TurnOrchestrator(
        _reg(), chitchat=_chitchat(), tier2_fn=_RecordingTier2(),
        l2_timeout_ms=5000, state_fn=lambda: {})
    s = OrchestratorSession()
    # 用"暂停任务"而不是 18 里的"先停一下": 后者含"停", 会先被安全旁路
    # (16 S4, 旁路先于分类)截走成 estop, 根本到不了 B05. 旁路优先是设计如此,
    # 这里只是绕开它去测 B05 本身.
    d = orch.handle_turn("暂停任务", s, now_mono_ms=1000)      # B05, L0
    assert d.kind == "reply"
    assert d.dispatch_result is None
    assert "没有正在执行的任务" in d.reply_text


def test_pause_resolves_the_running_task():
    """B05 pause is L0 -- no confirm, straight to a TaskCommand."""
    orch = TurnOrchestrator(
        _reg(), chitchat=_chitchat(), tier2_fn=_RecordingTier2(),
        l2_timeout_ms=5000, state_fn=lambda: _RUNNING_TASK)
    d = orch.handle_turn("暂停任务", OrchestratorSession(), now_mono_ms=1)
    assert d.kind == "dispatch"
    tc = d.dispatch_result.payload["task_command"]
    assert tc["action"] == "pause" and tc["task_id"] == "t-20260823-004"


def test_l2_deny_cancels():
    orch = _orch()
    s = OrchestratorSession()
    orch.handle_turn("不巡了", s, now_mono_ms=1000)
    d = orch.handle_turn("算了", s, now_mono_ms=1500)        # I02 deny
    assert d.kind == "confirm_denied"
    assert s.pending_confirm is None


def test_l2_confirm_times_out():
    orch = _orch()
    s = OrchestratorSession()
    orch.handle_turn("不巡了", s, now_mono_ms=1000)
    d = orch.handle_turn("确认", s, now_mono_ms=1000 + 6000)  # past 5000 ms
    assert d.kind == "confirm_timeout"
    assert s.pending_confirm is None


# -- criterion 3: fastpath never touches the LLM -------------------------

def test_fastpath_no_llm_no_prompt():
    """A04 hold is fastpath L0. MUTATION C: a fastpath intent that called
    the LLM would set llm_used/prompt_assembled or invoke tier2."""
    t2 = _RecordingTier2()
    orch = _orch(t2)
    d = orch.handle_turn("原地待命", OrchestratorSession(), now_mono_ms=1000)
    assert d.kind == "dispatch"
    assert d.intent_id == "A04"
    assert d.llm_used is False
    assert d.prompt_assembled is False
    assert t2.calls == []                   # tier-2 never called
    assert d.envelope is not None           # envelope built (EV-1..7)
    assert d.dispatch_result.key            # dispatched to a cmd/* key


# -- overheard silent (16 S5.2.1) ----------------------------------------

def test_overheard_is_silent():
    t2 = _RecordingTier2()
    orch = _orch(t2)
    d = orch.handle_turn("队友说的悄悄话", OrchestratorSession(), now_mono_ms=1)
    assert d.kind == "overheard"
    assert t2.calls == []                   # overheard never reaches the LLM


# -- chitchat reply is a preset, not an echo -----------------------------

def test_greeting_returns_preset_reply_not_echo():
    orch = _orch()
    d = orch.handle_turn("你好", OrchestratorSession(), now_mono_ms=1)
    assert d.kind == "reply"
    assert d.intent_id == "J01"
    assert d.reply_text in _chitchat()._p["greeting"]["default"]
    assert d.reply_text != "你好机器人你好"     # not an echo of the utterance


# -- unknown -> tier-2 (LLM used) ----------------------------------------

def test_unknown_directed_goes_to_tier2():
    t2 = _RecordingTier2(ret=None)          # gate/LLM denies
    orch = _orch(t2)
    d = orch.handle_turn("帮我看看那边的情况怎么样", OrchestratorSession(),
                         now_mono_ms=1)
    assert len(t2.calls) == 1               # tier-2 WAS called
    assert d.llm_used is True
    assert d.kind == "denied"               # tier2 returned None


def test_tier2_slots_propagate_to_dispatch():
    """The LLM-extracted slots reach the dispatch payload (the free-text/number
    slots the fastpath cannot fill). MUTATION: dropping the llm_slots merge in
    _dispatch_entry loses distance_m here. Also asserts tier-2 is called ONCE
    (no wasteful second call when the classified intent is route=='llm')."""
    t2 = _RecordingTier2(ret=Tier2Classification(
        name="move_forward", slots={"distance_m": 3}))
    orch = _orch(t2)
    d = orch.handle_turn("帮我处理一下那个事情", OrchestratorSession(),
                         now_mono_ms=1)
    assert d.kind == "dispatch" and d.intent_name == "move_forward"
    assert d.dispatch_result.payload.get("distance_m") == 3
    assert len(t2.calls) == 1


def test_tier2_classifies_out_of_scope_to_preset():
    t2 = _RecordingTier2(ret=Tier2Classification(name="out_of_scope", slots={}))
    orch = _orch(t2)
    # A DIRECTED phrase (imperative '帮我') that matches no keyword reaches
    # tier-2; the stub classifies it out_of_scope.
    d = orch.handle_turn("帮我看看今天股市怎么样", OrchestratorSession(),
                         now_mono_ms=1)
    assert d.kind == "reply"
    assert d.llm_used is True
    assert d.reply_text                     # preset out_of_scope reply


# -- CL-2: estop_path=down upgrades L1b -> L2 ----------------------------

def test_cl2_l1b_upgrades_to_l2_when_estop_down():
    """A11 turn_around is L1b. With estop_path=down it upgrades to L2 and
    must await confirm. MUTATION: no CL-2 upgrade => dispatches directly."""
    orch = _orch()
    s = OrchestratorSession()
    s.estop_path = "down"
    d = orch.handle_turn("转身", s, now_mono_ms=1000)
    assert d.kind == "await_confirm"
    assert d.auth == "L2"


def test_l1b_dispatches_normally_when_estop_up():
    orch = _orch()
    s = OrchestratorSession()               # estop_path defaults 'up'
    d = orch.handle_turn("转身", s, now_mono_ms=1000)
    assert d.kind == "dispatch"
    assert d.intent_id == "A11"


# -- PTZ post-classify refinement (18-B): degree -> E10, scan -> E07 -----

@pytest.mark.parametrize("text", [
    "云台左转30度", "云台向左转90度", "云台右转180度",
    "云台向上转45度", "云台左转九十度",
])
def test_ptz_degree_move_reclassified_to_e10(text):
    """A degree-bearing PTZ move is E10 (unsupported on the PELCO-D head),
    NOT the E01 jog its keyword ('云台左转') matches. MUTATION: drop the
    degree guard -> refine returns E01 and the head silently jogs a fixed
    pulse, so 30/90/180 all turn the same amount (18-B R-6 violation)."""
    assert refine_ptz_intent("E01", text) == "E10"


@pytest.mark.parametrize("text", [
    "云台向左环视一周", "平台向左环视一周", "镜头向右环视一周", "云台向左扫",
])
def test_ptz_scan_phrased_as_move_reclassified_to_e07(text):
    """A move that also says 环视/一圈/一周/扫 is a scan (E07). MUTATION: drop
    the scan override -> '平台向左环视一周' stays E01 (its 平台向左 keyword ties
    环视一周 and wins as the earlier registry entry) and does one small move
    instead of a full turn."""
    assert refine_ptz_intent("E01", text) == "E07"


def test_ptz_plain_move_is_unchanged():
    """A bare direction with no degree and no scan cue stays E01. MUTATION:
    an over-broad guard (e.g. any '度'/'周' substring) would divert these."""
    for text in ("云台向左", "云台左转", "平台向右", "向下看"):
        assert refine_ptz_intent("E01", text) == "E01"


def test_degree_move_end_to_end_is_denied_with_reason(monkeypatch):
    """Through handle_turn: a degree move is denied (E10) with a spoken reason,
    never dispatched to cmd/ptz. MUTATION: without the guard it dispatches E01
    and kind=='dispatch' with no denial."""
    orch = _orch()
    d = orch.handle_turn("云台左转90度", OrchestratorSession(), now_mono_ms=1000)
    assert d.kind == "denied"
    assert d.intent_id == "E10"
    assert d.tts_text and "角度" in d.tts_text     # explains the limitation


def test_scan_move_end_to_end_dispatches_e07():
    """Through handle_turn: '平台向左环视一周' dispatches E07 (a full scan), not
    a small E01 move. This is the ASR-alias case the operator hit."""
    orch = _orch()
    d = orch.handle_turn("平台向左环视一周", OrchestratorSession(),
                         now_mono_ms=1000)
    assert d.kind == "dispatch"
    assert d.intent_id == "E07"


# -- PTZ-prefix reclaim: a 云台 command a chassis keyword stole -----------

@pytest.mark.parametrize("text,expect", [
    ("云台旋转速度慢一点", "E09"),   # A13 '慢一点' stole it -> reclaim E09
    ("平台旋转速度快一点", "E09"),   # 平台 alias reclaimed too
    ("云台慢一点", "E09"),
])
def test_ptz_prefix_reclaims_from_chassis(text, expect):
    """A 云台-prefixed command wrongly keyword-matched to a chassis intent
    (A13 set_speed_profile) is reclaimed to PTZ. MUTATION: drop the reclaim ->
    '云台旋转速度慢一点' stays A13 and changes the CHASSIS speed instead of the
    camera's."""
    assert refine_ptz_intent("A13", text) == expect


@pytest.mark.parametrize("text", ["慢一点", "快一点", "全速"])
def test_bare_speed_stays_chassis(text):
    """Without a PTZ subject the same speed word is the chassis (A13) and must
    NOT be reclaimed. MUTATION: reclaim without the subject gate -> a bare
    '慢一点' becomes a PTZ speed and the chassis can no longer be slowed."""
    assert refine_ptz_intent("A13", text) == "A13"


def test_reclaim_only_when_resolve_ptz_finds_an_action():
    """A PTZ subject with no recognisable PTZ action is left as-is (not forced
    into a move). MUTATION: reclaim unconditionally -> a non-action utterance
    that merely names 云台 gets turned into a spurious PTZ command."""
    # '云台' present but no direction/zoom/scan/speed -> resolve_ptz is None.
    assert refine_ptz_intent("J01", "云台你好呀") == "J01"


def test_prefix_reclaim_end_to_end():
    """Through handle_turn: '云台旋转速度慢一点' dispatches the PTZ speed intent
    E09, not the chassis A13."""
    orch = _orch()
    d = orch.handle_turn("云台旋转速度慢一点", OrchestratorSession(),
                         now_mono_ms=1000)
    assert d.intent_id == "E09"


# -- PB4: task-create dispatch carries a cmd/task request ------------------

def test_task_create_carries_a_contract_task_command():
    """*** A task-create intent now carries the 11 S7.2 TaskCommand.

    It used to carry a PRIVATE `task_request` whose top-level keys had an empty
    intersection with S7.2's, and P3 understood only that -- so the HMI and the
    cloud, both listed publishers of cmd/task (S2.2), could not submit at all.

    MUTATION: go back to emitting task_request and P3's contract path stops
    seeing voice tasks, which is the two-sources-of-truth split this migration
    closed.
    """
    orch = _orch()
    d = orch.handle_turn("开始巡逻", OrchestratorSession(), now_mono_ms=1000)
    assert d.kind == "dispatch"
    tc = d.dispatch_result.payload.get("task_command")
    assert tc is not None
    assert tc["action"] == "submit" and tc["source"] == "voice"
    assert tc["task"]["type"] == "patrol"
    # task_id is NOT minted here: the form is t-YYYYMMDD-NNN and the per-day
    # sequence is P3's alone (S7.2, corrected 2026-08-20).
    assert "task_id" not in tc["task"]
    # 18 provenance survives the move, inside task.params -> mission_json.
    assert tc["task"]["params"]["intent"] == "patrol_route"
    assert tc["task"]["params"]["id"] == "B02"
    # cmd_id is the idempotency key and must be present (S2.3).
    assert tc["cmd_id"]


def test_device_intent_has_no_task_request():
    """A payload/PTZ command is NOT a task: its payload must not carry a
    task_request. MUTATION: enriching every dispatch (not just task-creates)
    would mint a task for a light command."""
    orch = _orch()
    d = orch.handle_turn("开灯", OrchestratorSession(), now_mono_ms=1000)
    assert d.kind == "dispatch"
    assert "task_command" not in d.dispatch_result.payload


def test_orchestrator_source_flows_into_request():
    """The orchestrator's source ('text' for a text channel) is what the
    command records -- P3 maps it onto the five-value tasks.source with an
    explicit table (15 S4.2). MUTATION: hard-coding 'voice' would fail here."""
    orch = TurnOrchestrator(
        _reg(), chitchat=_chitchat(), tier2_fn=_RecordingTier2(),
        l2_timeout_ms=5000, source="text")
    d = orch.handle_turn("开始巡逻", OrchestratorSession(), now_mono_ms=1000)
    assert d.dispatch_result.payload["task_command"]["source"] == "text"


# -- route name -> route_id (batch 15) -----------------------------------

def _manifest(items):
    """A state map as state_fn returns it: keyed by zenoh key (11 S7.10)."""
    return lambda: {"state/geo/manifest": {"items": items}}


_EAST = {"geo_id": "r-east", "type": "route", "name": "东门路线",
         "state": "active", "num": 1}


#: A DIRECTED utterance that matches no keyword, so it reaches layer-6 tier-2.
#: It has to: patrol_route is `fastpath_then_llm`, and a keyword-matched turn
#: never carries a route slot at all -- the slot only exists when the LLM
#: classified the utterance. Using a phrase containing 巡逻 would silently take
#: the fastpath and test nothing.
_UNMATCHED = "帮我处理一下那个事情"


def _route_orch(spoken="东门路线", state_fn=None):
    """An orchestrator whose tier-2 classifies to patrol_route with a spoken
    route slot -- the shape the LLM returns for 'go patrol the east route'."""
    return TurnOrchestrator(
        _reg(), chitchat=_chitchat(),
        tier2_fn=_RecordingTier2(ret=Tier2Classification(
            name="patrol_route", slots={"route": spoken})),
        l2_timeout_ms=5000, state_fn=state_fn)


def test_spoken_route_name_becomes_route_id():
    """*** The spoken name resolves to a geo_id, which lands in
    task.route_id -> tasks.route_geo_id.

    That column was NULL on every task ever recorded, which is why geo_refs had
    to match on the NAME to answer 'is this route in use' before a delete.

    MUTATION: passing route_id=None (or resolving against the manifest but
    discarding the result) leaves route_id absent and this fails.
    """
    orch = _route_orch(state_fn=_manifest([_EAST]))
    d = orch.handle_turn(_UNMATCHED, OrchestratorSession(), now_mono_ms=1)
    assert d.kind == "dispatch"
    tc = d.dispatch_result.payload["task_command"]
    assert tc["task"]["route_id"] == "r-east"
    # The spoken name survives too: an audit reads what was SAID, not only
    # what it resolved to.
    assert tc["task"]["params"]["slots"]["route"] == "东门路线"


def test_route_that_matches_nothing_is_refused_at_the_turn():
    """*** A named route absent from the catalogue is refused while the
    operator is still standing there, instead of becoming a queued task with no
    route attached that fails minutes later.

    MUTATION: swallowing GeoRequestError and continuing with route_id=None
    makes this a dispatch, and the operator hears the task was accepted.
    """
    orch = _route_orch("西门路线", state_fn=_manifest([_EAST]))
    d = orch.handle_turn(_UNMATCHED, OrchestratorSession(), now_mono_ms=1)
    assert d.kind == "reply"
    assert d.reply_text and d.reply_text == d.tts_text
    # No task frame was built: a refused turn must not also queue the task.
    assert d.dispatch_result is None


def test_absent_catalogue_still_submits_the_task():
    """*** THE REGRESSION GUARD for this migration.

    state_fn has no production call site: nothing in p4_agent subscribes to
    state/geo/manifest yet. If 'no catalogue' were treated as 'route not found',
    this batch would switch OFF the voice task path that already runs on the
    robot -- every patrol command would answer 'no such route', and the cause
    would be an unwired subscription rather than anything the operator said.

    MUTATION: refusing when manifest is None turns this into a reply. That
    mutation is exactly the code I wrote first, which is why this test exists.
    """
    orch = _route_orch(state_fn=None)          # production wiring today
    d = orch.handle_turn(_UNMATCHED, OrchestratorSession(), now_mono_ms=1)
    assert d.kind == "dispatch"
    tc = d.dispatch_result.payload["task_command"]
    assert tc["action"] == "submit"
    # No route_id -- and that is the honest outcome, not a resolved one. P3
    # falls back to matching the spoken name (see p3 task_row.py).
    assert "route_id" not in tc["task"]
    assert tc["task"]["params"]["slots"]["route"] == "东门路线"
