"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_motion_channel.py
Brief: a cmd/motion/intent frame names the channel the turn really came in on (11 MO-2)

Description:
cmd/motion/intent carries `channel` (11 S9.3.2A.3: mic_local | cloud | wecom |
hmi), and P2 fills relative_move.source from it (MO-2 table). Until
2026-10-26 the value came from the orchestrator's constructor constant
source="voice" -- one orchestrator serves the microphone, cloud text, cloud
voice and the HMI, so every motion frame said mic_local and P2 could not tell
a cloud command from a spoken one. The lookup also defaulted any unknown value
to mic_local without a word.

What is pinned (motion phrase "掉头" = turn_around, the one MotionIntent
primitive that needs no slot; nothing reaches a robot -- frames are only
returned):
  * each entry labels its frame: mic -> mic_local, cloud text -> cloud,
    cloud voice on wecom -> wecom, HMI text -> hmi;
  * the frame and the turn's state/voice_turn record agree on channel;
  * (not pinned, on purpose: "the channel does not leak into the next
    turn". Every turn sets its channel before anything reads it, so a leak
    cannot be observed and such a case would be green whatever the code
    did -- CLAUDE.md 3.2 form 1. The orchestrator notes it as equivalent.)
  * an out-of-set channel raises (orchestrator and frame builder), never
    falls back to mic_local.

Mutants this file kills (run by hand, see the commit).

Boundaries: the label on the frame. What P2 does with it (relative_move
.source) is P2's and tested there.
"""
from __future__ import annotations

import json  # frames are JSON bytes

import pytest  # raises, parametrize, marker
import yaml  # the real content files, as p4 loads them

from xbrain.p4_agent.classifier.keyword_matcher import KeywordMatcher
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.cloud_input import (
    CloudTurnRunner,  # the cloud text / voice entries
    CompletedUtterance,  # a finished cloud sentence
)
from xbrain.p4_agent.runtime.motion_intent_request import to_motion_intent
from xbrain.p4_agent.runtime.orchestrator_turn import (
    build_orchestrator,  # as p4's wiring builds it
    make_turn_handler,  # the microphone entry
)
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession
from xbrain.p4_agent.session.chitchat import ChitchatResponder

# No robot, no Zenoh, no ASR / LLM service.
pytestmark = pytest.mark.no_device  # runs anywhere

#: The content files p4's __main__ loads (absolute: no home-dir copies).
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"  # 18 registry
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"  # chitchat presets
#: Record key for the cases that also read state/voice_turn.
_KEY = "xbrain/t-1/state/voice_turn"  # any robot id works
#: turn_around: a motion primitive with no slot, so the frame always goes out.
_MOVE = "掉头"  # no slot needed


def _parts():
    """(orchestrator, registry) over the production content files."""
    with open(_INTENTS, encoding="utf-8") as fh:  # production content
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    with open(_CHITCHAT, encoding="utf-8") as fh:  # production content
        cc = ChitchatResponder(yaml.safe_load(fh))  # chitchat presets
    return build_orchestrator(reg, cc, l2_timeout_ms=8000), reg  # tier-2 left null


def _frame_channel(pairs):
    """data.channel of the one cmd/motion/intent frame among the publishes."""
    frames = [json.loads(d) for k, d in pairs if k == "cmd/motion/intent"]
    assert len(frames) == 1, [k for k, _d in pairs]  # exactly one went out
    return frames[0]["data"]["channel"]  # the MO-2 field


def _record_channel(pairs):
    """channel of the turn's state/voice_turn record."""
    recs = [json.loads(d) for k, d in pairs if k == _KEY]
    assert len(recs) == 1  # one record per turn
    return recs[0]["channel"]  # the record's field


def _runner(orch, reg, published, transcript=""):
    """A cloud runner sharing `orch`; frames collected into `published`."""
    return CloudTurnRunner(orch, KeywordMatcher(reg),  # real matcher
                           lambda k, d: published.append((k, d)),
                           lambda pcm: transcript,  # the fake ASR answer
                           lambda: 1000, lambda: None,  # clock, no B mode
                           OrchestratorSession(), voice_turn_key=_KEY)


# -- each entry labels its own frame --------------------------------------

def test_the_microphone_labels_its_frame_mic_local():
    """The mic path: mic_local, and the record agrees."""
    orch, _reg = _parts()  # one orchestrator
    pairs = make_turn_handler(orch, OrchestratorSession(),
                              voice_turn_key=_KEY)(_MOVE)
    assert _frame_channel(pairs) == "mic_local"  # the frame label
    assert _record_channel(pairs) == "mic_local"  # same label both keys


@pytest.mark.parametrize("channel", ["cloud", "wecom", "hmi"])
def test_typed_text_labels_its_frame_with_its_channel(channel):
    """Cloud / WeChat / HMI text: the frame says where it came from.

    Before 2026-10-26 all three said mic_local.
    """
    orch, reg = _parts()  # one orchestrator
    published = []  # frames in order
    _runner(orch, reg, published).run_text(
        {"channel": channel, "text": _MOVE, "require_tts_reply": False})
    assert _frame_channel(published) == channel  # not mic_local
    assert _record_channel(published) == channel  # MI-1: keys agree


def test_cloud_voice_on_wecom_labels_its_frame_wecom():
    """A WeChat voice sentence: wecom on the frame, not mic_local."""
    orch, reg = _parts()  # one orchestrator
    published = []  # frames in order
    runner = _runner(orch, reg, published, transcript=_MOVE)
    runner.run_voice(CompletedUtterance("s-1", "wecom", b"\x01\x00" * 3200))
    assert _frame_channel(published) == "wecom"  # not mic_local


# -- out-of-set values raise -------------------------------------------------

def test_an_unknown_channel_raises_in_the_orchestrator():
    """11 S13.6: refused at the door, never defaulted to mic_local.

    A NON-motion phrase on purpose: with "掉头" the frame builder would raise
    too, and this case would stay green with the orchestrator's own check
    deleted -- it would be testing the builder twice.
    """
    orch, _reg = _parts()  # one orchestrator
    with pytest.raises(ValueError):  # refused at the door
        orch.handle_turn("电量还有多少", OrchestratorSession(), 1000,
                         channel="radio")  # not an MO-2 value


def test_an_unknown_channel_raises_in_the_frame_builder():
    """The builder checks too: the old lookup fell back to mic_local."""
    with pytest.raises(ValueError):  # refused by the builder
        to_motion_intent("turn_around", slots={}, cmd_id="c", turn_id="t",
                         channel="voice")  # the old constructor constant
