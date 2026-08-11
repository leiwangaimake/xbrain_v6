"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_text_channel.py
Brief: GWY-P4-42 (32.J) -- text command -> orchestrator (skips ASR) + channel gate

Description:
Tests the text-command channel: a cmd/voice_text message runs the same
orchestrator path as voice, minus ASR, and the H03f channel gate denies a
force-step time-sync over HMI. Each criterion carries a mutation that must
turn red per CLAUDE.md 3.3.
"""
from __future__ import annotations

import json

import pytest
import yaml

from xbrain.common.errors import E_CHANNEL_DENIED
from xbrain.p4_agent.classifier.keyword_matcher import KeywordMatcher
from xbrain.p4_agent.registry.channel_permission import channel_admission
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator
from xbrain.p4_agent.runtime.text_channel import (
    handle_text_command,
)
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession
from xbrain.p4_agent.session.chitchat import ChitchatResponder
from xbrain.p5_gateway.text_input import build_voice_text_msg

pytestmark = pytest.mark.no_device

_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"


def _reg():
    return load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))


def _setup():
    reg = _reg()
    cc = ChitchatResponder(yaml.safe_load(open(_CHITCHAT, encoding="utf-8")))
    orch = build_orchestrator(reg, cc, l2_timeout_ms=8000)
    return reg, orch, KeywordMatcher(reg), OrchestratorSession()


# -- criterion 2: H03f channel gate (unit) -------------------------------

def test_h03f_denied_on_hmi():
    allowed, code = channel_admission("H03", {"force_step": True}, "hmi")
    assert allowed is False
    assert code == E_CHANNEL_DENIED


def test_h03f_allowed_on_cloud():
    allowed, code = channel_admission("H03", {"force_step": True}, "cloud")
    assert allowed is True


def test_h03_non_force_not_subject_to_h03f_rule():
    # Without force_step the H03f override does not apply (this gate returns
    # allowed; the general origin gate is is_channel_allowed, wired later).
    allowed, _ = channel_admission("H03", {"force_step": False}, "hmi")
    assert allowed is True


def test_unknown_channel_raises():
    from xbrain.p4_agent.registry.channel_permission import (
        ChannelAdmissionError,
    )
    with pytest.raises(ChannelAdmissionError):
        channel_admission("H03", {}, "carrier_pigeon")


# -- criterion 2: gate enforced in the text handler ----------------------

def test_text_h03f_over_hmi_is_channel_denied():
    """MUTATION B guard: a force-step time-sync typed on HMI must be
    E_CHANNEL_DENIED, not dispatched."""
    reg, orch, matcher, sess = _setup()
    msg = build_voice_text_msg("hmi", "校时", "c-1", 1.0,
                               slots={"force_step": True})
    res = handle_text_command(msg, orch, sess, matcher, now_mono_ms=1000)
    assert res.denied is True
    assert res.code == E_CHANNEL_DENIED
    assert res.decision is None


def test_text_h03f_over_cloud_is_allowed():
    reg, orch, matcher, sess = _setup()
    msg = build_voice_text_msg("cloud", "校时", "c-2", 1.0,
                               slots={"force_step": True})
    res = handle_text_command(msg, orch, sess, matcher, now_mono_ms=1000)
    assert res.denied is False


# -- criterion 1: text runs the orchestrator, skipping ASR ---------------

def test_text_module_does_not_import_asr():
    """MUTATION A guard: the text path must NOT re-run ASR -- the input is
    already text. An ASR import/call in the module would be the bug."""
    import xbrain.p4_agent.runtime.text_channel as mod
    src = open(mod.__file__, encoding="utf-8").read()
    # No ASR client import and no transcribe CALL (the word may appear in
    # prose explaining why; a call/import is the actual bug).
    assert "import" not in src or "asr_client" not in src.replace(
        "ASR", "")
    assert "ai_client.asr_client" not in src
    assert ".transcribe(" not in src
    assert "transcribe(" not in src


def test_text_query_runs_same_orchestrator_path_as_voice():
    """A text 'query battery' produces the SAME classification/dispatch as
    a voice turn -- via orchestrator.handle_turn, no ASR."""
    reg, orch, matcher, sess = _setup()
    msg = build_voice_text_msg("hmi", "电量还有多少", "c-3", 1.0)
    res = handle_text_command(msg, orch, sess, matcher, now_mono_ms=1000)
    assert res.denied is False
    assert res.decision is not None
    assert res.decision.intent_id == "G02"          # same as voice classify


def test_text_action_dispatches_like_voice():
    reg, orch, matcher, sess = _setup()
    msg = build_voice_text_msg("hmi", "原地待命", "c-4", 1.0)
    res = handle_text_command(msg, orch, sess, matcher, now_mono_ms=1000)
    assert res.decision.intent_id == "A04"
    assert res.decision.kind == "dispatch"


def test_require_tts_reply_false_suppresses_speak():
    """require_tts_reply=false: a typed query answers without seizing the
    speaker (11 S8.7.5) -- no cmd/audio/speak publish."""
    reg, orch, matcher, sess = _setup()
    # A greeting normally speaks a preset; with tts off it must not publish
    # to cmd/audio/speak.
    msg = build_voice_text_msg("hmi", "你好", "c-5", 1.0,
                               require_tts_reply=False)
    res = handle_text_command(msg, orch, sess, matcher, now_mono_ms=1000)
    keys = [k for k, _ in res.publishes]
    assert "cmd/audio/speak" not in keys


# -- P5 message builder ---------------------------------------------------

def test_build_voice_text_msg_shape():
    msg = build_voice_text_msg("cloud", "停止喊话", "c-6", 12.5)
    assert msg["channel"] == "cloud"
    assert msg["text"] == "停止喊话"
    assert msg["require_tts_reply"] is True


def test_build_voice_text_rejects_bad_channel():
    from xbrain.p5_gateway.text_input import TextInputError
    with pytest.raises(TextInputError):
        build_voice_text_msg("sms", "x", "c-7", 1.0)
