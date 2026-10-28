"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_speak_source.py
Brief: every cmd/audio/speak from p4 names its domain-2 source by channel (11 S8.8.1, 99 U99 C1)

Description:
P2 arbitrates the speaker since 2026-10-28 (99 U99 C1) and refuses a speak
request without a TTS source (11 S8.8.1 "source required"). Until then p4
never sent one, so P2 could not rank a cloud reply above a local one, nor keep
a TTS out of a broadcast. These cases pin the p4 half:
  * turn_labels.tts_source maps the four turn channels and refuses others;
  * decision_to_publishes stamps the channel's source on every speak it emits
    (one place, so no speaking branch can forget it);
  * the text channel's own denial reply carries it too.
Driven through the production orchestrator over the production content files,
like test_voice_turn.py. Mutants named per case.
"""

from __future__ import annotations  # postponed annotations

import json  # decode the denial bytes

import pytest  # parametrize, raises
import yaml  # production content files

from xbrain.p4_agent.registry.intents import load_intent_registry  # 18 registry
from xbrain.p4_agent.runtime import turn_labels  # under test
from xbrain.p4_agent.runtime.intent_dispatch import CMD_AUDIO_SPEAK  # the speak key
from xbrain.p4_agent.runtime.orchestrator_turn import (
    build_orchestrator,
    decision_to_publishes,
)
from xbrain.p4_agent.runtime.text_channel import _speak_bytes  # denial reply
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # fresh dialog
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # chitchat presets

pytestmark = pytest.mark.no_device  # pure in-process

_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"  # 18 registry, production
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"  # chitchat presets


def _orch():  # production orchestrator
    """The production orchestrator (no LLM needed for a keyword reply)."""
    with open(_INTENTS, encoding="utf-8") as fh:
        reg = load_intent_registry(yaml.safe_load(fh))
    with open(_CHITCHAT, encoding="utf-8") as fh:
        cc = ChitchatResponder(yaml.safe_load(fh))  # greeting replies
    return build_orchestrator(reg, cc, l2_timeout_ms=8000)


@pytest.mark.parametrize("channel, source", [
    ("mic_local", "tts_local"), ("cloud", "tts_cloud"),
    ("wecom", "tts_wecom"), ("hmi", "tts_local")])
def test_tts_source_by_channel(channel, source):  # the mapping table
    """14 S4.1 register: cloud 600 > wecom 500 > local 400; the HMI is the
    on-site operator. MUTATION: map cloud to tts_local -> a row fails."""
    assert turn_labels.tts_source(channel) == source  # one row


def test_tts_source_refuses_an_unknown_channel():  # closed set
    """CLAUDE.md 3.5: never guess a source for an unknown channel."""
    with pytest.raises(ValueError):
        turn_labels.tts_source("radio")


@pytest.mark.parametrize("channel, modality, source", [
    ("mic_local", "voice", "tts_local"), ("cloud", "text", "tts_cloud")])
def test_every_speak_of_a_turn_names_the_channel_source(channel, modality, source):  # end to end
    """A chitchat greeting answers with a spoken reply on cmd/audio/speak.
    MUTATION: drop the stamping in decision_to_publishes -> no "source" key
    -> P2 would refuse the reply -> this fails."""
    d = _orch().handle_turn("你好", OrchestratorSession(), 1000,
                            channel=channel, modality=modality)  # the turn's entry
    speaks = [p for k, p in decision_to_publishes(d) if k == CMD_AUDIO_SPEAK]
    assert speaks, "the greeting produced no spoken reply"  # precondition
    assert all(p["source"] == source for p in speaks)  # every speak stamped


def test_the_text_channel_denial_reply_names_its_source():  # the other builder
    """H03f denial spoken back on the cloud text channel."""
    key, raw = _speak_bytes("该指令不允许从此通道下达", "cloud")
    assert key == CMD_AUDIO_SPEAK and json.loads(raw)["source"] == "tts_cloud"  # stamped
