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
    OrchestratorSession, TurnOrchestrator,
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


def test_l2_confirm_then_dispatch():
    orch = _orch()
    s = OrchestratorSession()
    orch.handle_turn("不巡了", s, now_mono_ms=1000)          # opens confirm
    d = orch.handle_turn("确认", s, now_mono_ms=1500)        # I01 -> dispatch
    assert d.kind == "dispatch"
    assert d.intent_id == "B07"
    assert d.dispatch_result is not None
    assert s.pending_confirm is None


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


def test_tier2_classifies_out_of_scope_to_preset():
    t2 = _RecordingTier2(ret="out_of_scope")
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
