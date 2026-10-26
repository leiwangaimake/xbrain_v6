"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_voice_turn.py
Brief: every p4 turn ends with a state/voice_turn record that tells the truth about it (11 S8.7.6)

Description:
runtime/voice_turn builds the per-turn audit record p4 had never published.
The record is the only way a remote operator reads the robot's answer (the
GZH-2 module speaks it on the robot), and the only place "what ASR heard"
and "what the model decided" sit side by side. These cases pin that it says
what actually happened, not a plausible rendering of it.

What is pinned, each against the real orchestrator over configs/:
  * a motion intent's record carries the turn_id, intent and slots of the
    cmd/motion/intent frame that went out, byte for byte (11 MI-1);
  * the record is published AFTER the action frames, so a consumer never
    sees an audit record for an action that has not been sent yet;
  * an overheard turn is recorded -- verdict unknown, reply_kind none,
    layer overheard, nothing spoken (user ruling 2026-10-26);
  * asr_raw and asr_text are both present and differ when 16 S3 rewrote
    the text, with the rewrite listed in corrections[];
  * require_tts_reply=false: no speak frame, reply_kind none, and the
    answer text is still in the record (11 S8.7.5);
  * a turn refused at the channel gate is recorded as rejected with its
    code; hmi text is filed under the local session;
  * no robot id -> no record at all, never one on a placeholder key;
  * out-of-set values raise, and every decision kind the orchestrator
    source can produce has a verdict (a new kind turns this file red).

Mutants this file kills (run by hand, see the commit).

Boundaries: the record's content. Its transport (the cached publisher in
main_wiring) is the running-stack check; p5 storing corrections is p5's.
"""
from __future__ import annotations

import json  # records go out as JSON bytes
import re  # scans the orchestrator source for decision kinds

import pytest  # raises, parametrize, marker
import yaml  # the real content files, as p4 loads them

from xbrain.common.errors import E_CHANNEL_DENIED  # the H03f refusal code
from xbrain.p4_agent.classifier.keyword_matcher import KeywordMatcher
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime import turn_orchestrator as orch_mod
from xbrain.p4_agent.runtime.cloud_input import CloudTurnRunner  # the cloud path
from xbrain.p4_agent.runtime.orchestrator_turn import (
    build_orchestrator,  # as p4 builds it
    make_turn_handler,  # the mic path
)
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession,  # a fresh dialog per case
    TurnDecision,  # for the closed-set cases
)
from xbrain.p4_agent.runtime.voice_turn import (
    VERDICT_OF_KIND,  # kind -> verdict
    VoiceTurnError,  # raised on closed-set faults
    build_voice_turn,  # under test
    voice_turn_pair,  # the no-key rule
)
from xbrain.p4_agent.session.chitchat import ChitchatResponder

# No robot, no Zenoh, no ASR / LLM service.
pytestmark = pytest.mark.no_device  # runs anywhere

#: The content files p4's __main__ loads (absolute: no home-dir copies).
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"  # 18 registry
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"  # chitchat presets

#: A record key for the cases; any robot id works, the shape is what counts.
_KEY = "xbrain/t-1/state/voice_turn"  # any robot id works


def _parts():
    """(orchestrator, registry) over the production content files."""
    with open(_INTENTS, encoding="utf-8") as fh:  # production content
        reg = load_intent_registry(yaml.safe_load(fh))
    with open(_CHITCHAT, encoding="utf-8") as fh:
        cc = ChitchatResponder(yaml.safe_load(fh))
    return build_orchestrator(reg, cc, l2_timeout_ms=8000), reg


def _mic(raw, post=None, key=_KEY):
    """One microphone turn through make_turn_handler; its (key, bytes) list."""
    orch, _reg = _parts()  # fresh per call
    handler = make_turn_handler(orch, OrchestratorSession(), asr_post=post,
                                voice_turn_key=key)
    return handler(raw)  # returns frames, sends nothing


def _record(pairs):
    """The decoded state/voice_turn record among a turn's publishes."""
    found = [json.loads(d) for k, d in pairs if k == _KEY]  # only the record key
    assert len(found) == 1, "expected exactly one record, got %d" % len(found)
    return found[0]  # the decoded record


class _Post:
    """A 16 S3 post-processor with a fixed answer and correction list."""

    def __init__(self, text, rows):
        self._text, self._rows = text, rows  # what apply() returns

    def apply(self, raw):
        """Same (text, corrections) shape as AsrPostProcessor.apply."""
        return self._text, list(self._rows)  # a copy, like a real call


def _runner(published):
    """A cloud runner over the real orchestrator; frames go to `published`."""
    orch, reg = _parts()  # fresh per call
    return CloudTurnRunner(orch, KeywordMatcher(reg),  # real matcher
                           lambda k, d: published.append((k, d)),
                           lambda pcm: "", lambda: 1000, lambda: None,
                           OrchestratorSession(), voice_turn_key=_KEY)


# -- the stitch with the action frame -----------------------------------

def test_a_motion_record_reuses_the_frame_s_turn_id_intent_and_slots():
    """11 MI-1: the two keys must compare byte for byte.

    "掉头" (turn_around) sends a cmd/motion/intent frame; its data block and
    the record must agree on turn_id, intent and slots exactly. (Not
    "原地待命": A04 hold goes out as a plain p4_intent_v1 frame with no
    turn_id -- the stitch only exists for the MotionIntent primitives.)
    Nothing reaches the robot: the handler only returns the frames.
    """
    pairs = _mic("掉头")  # turn_around, no slots
    frame = [json.loads(d) for k, d in pairs if k == "cmd/motion/intent"]
    assert len(frame) == 1, pairs  # the action went out
    data = frame[0]["data"]  # the frame's audit block
    rec = _record(pairs)  # the record of that turn
    assert rec["turn_id"] == data["turn_id"]  # the audit stitch
    assert rec["intent"] == data["intent"]  # same intent name
    # Byte-identical, not merely equal-looking (MI-1 wording).
    assert json.dumps(rec["slots"]) == json.dumps(data["slots"])
    assert rec["verdict"] == "accepted" and rec["session"] == "local"


def test_the_record_is_published_after_the_action_frames():
    """A consumer must never see the record of an action not yet sent."""
    keys = [k for k, _d in _mic("掉头")]  # publish order
    assert keys[-1] == _KEY, keys  # last, after every action frame


# -- overheard, ASR columns, corrections --------------------------------

def test_an_overheard_turn_is_recorded_but_nothing_is_said():
    """User ruling 2026-10-26: record it, verdict unknown, reply none.

    The only publish of the turn is the record itself -- 16 S5.2.1's
    silence (no speech, no action) holds.
    """
    pairs = _mic("今天天气怎么样")  # no keyword, no address
    assert [k for k, _d in pairs] == [_KEY]  # nothing else went out
    rec = _record(pairs)  # its record
    assert (rec["verdict"], rec["reply_kind"], rec["layer"]) == (
        "unknown", "none", "overheard")
    assert rec["reply_text"] is None  # no answer was made


def test_asr_raw_and_asr_text_both_reach_the_record_with_corrections():
    """What ASR heard, what 16 S3 made of it, and the rewrite in between."""
    post = _Post("原地待命", [("L2", "代", "待", 1)])  # L2 sound-alike fix
    rec = _record(_mic("原地代命", post=post))  # misheard as spoken
    assert (rec["asr_raw"], rec["asr_text"]) == ("原地代命", "原地待命")
    # L2 carries a pinyin distance, not a similarity: score stays null.
    assert rec["corrections"] == [{"layer": "L2", "from": "代", "to": "待",
                                   "score": None, "distance": 1}]


# -- reply_kind and the text-only answer ---------------------------------

def test_a_muted_reply_is_still_readable_in_the_record():
    """require_tts_reply=false: not spoken, yet the answer is in the record.

    11 S8.7.5 says such an answer goes ONLY to state/voice_turn; with the
    record missing it would go nowhere.
    """
    published = []  # frames in order
    _runner(published).run_text({"channel": "cloud", "text": "电量还有多少",
                                 "require_tts_reply": False})
    assert "cmd/audio/speak" not in [k for k, _d in published]  # muted: not spoken
    rec = _record(published)  # the record
    assert rec["reply_kind"] == "none" and rec["reply_text"]  # text kept


def test_a_spoken_reply_is_marked_stream():
    """The same question, spoken: reply_kind stream (device TTS, ruling)."""
    published = []  # frames in order
    _runner(published).run_text({"channel": "cloud", "text": "电量还有多少",
                                 "require_tts_reply": True})
    assert _record(published)["reply_kind"] == "stream"  # device TTS ruling


# -- channel gate and sessions --------------------------------------------

def test_a_channel_refusal_is_recorded_with_its_code():
    """H03 force-step on hmi is refused before classification; recorded."""
    published = []  # frames in order
    _runner(published).run_text({"channel": "hmi", "text": "校时",
                                 "slots": {"force_step": True},  # the H03f trigger
                                 "require_tts_reply": False})
    rec = _record(published)  # the record
    assert (rec["verdict"], rec["reject_code"]) == ("rejected",
                                                    E_CHANNEL_DENIED)
    assert rec["session"] == "local" and rec["channel"] == "hmi"


# -- no key, closed sets, coverage of kinds -------------------------------

def test_without_a_key_no_record_is_published():
    """No robot id at startup: the turn publishes no record at all."""
    pairs = _mic("今天天气怎么样", key=None)  # no key: no record
    assert pairs == []  # overheard + no key: nothing whatsoever
    assert voice_turn_pair(None, {"x": 1}) == ()  # the helper itself


@pytest.mark.parametrize("kwargs", [
    {"channel": "radio"},  # not one of the four channels
    {"decision": TurnDecision(kind="something_new")},  # unmapped kind
    {"decision": TurnDecision(kind="reply", level="L9")},  # not a level
    {"decision": TurnDecision(kind="bypass", bypass_action="jump")},
])
def test_out_of_set_values_raise(kwargs):
    """11 S13.6: refused, never passed through or guessed."""
    base = {"channel": "cloud", "asr_raw": "a", "asr_text": "a",  # a valid base record
            "decision": TurnDecision(kind="reply"), "published_keys": [],
            "latency_ms": {}}  # required argument
    base.update(kwargs)  # the one field under test
    with pytest.raises(VoiceTurnError):
        build_voice_turn(**base)  # must raise


def test_every_decision_kind_in_the_orchestrator_has_a_verdict():
    """A new branch in the orchestrator must not land as a crash or a guess.

    Reads the orchestrator source for every kind="..." it can return; each
    must be in VERDICT_OF_KIND. Adding a kind there without a verdict here
    turns this red -- the moment to decide what it means for the record.
    """
    src = open(orch_mod.__file__, encoding="utf-8").read()  # the orchestrator source
    kinds = set(re.findall(r'kind="([a-z_]+)"', src))  # every kind it can return
    assert kinds, "no kinds found -- the scan is broken, not the code"
    assert sorted(kinds - set(VERDICT_OF_KIND)) == []  # none unmapped


def test_a_microphone_turn_that_fails_inside_is_recorded_not_silent():
    """2026-10-26: an exception inside the turn used to escape make_turn_handler
    (worker "job fail", nothing published). Now one rejected / E_INTERNAL record.

    MUTATION: drop the except branch around handle_turn -> this test errors.
    """
    orch, _reg = _parts()  # the production orchestrator
    def _boom(*_a, **_k):
        """Any internal failure, e.g. a query reading a missing field."""
        raise KeyError("boom")
    orch.handle_turn = _boom  # every turn fails inside
    # The battery ask is the turn that crashed on the real robot (G02 read a
    # missing state/power field); any utterance would do once handle_turn raises.
    pairs = make_turn_handler(orch, OrchestratorSession(), voice_turn_key=_KEY)("现在电量多少")
    rec = _record(pairs)  # exactly one record, nothing else
    assert (rec["verdict"], rec["reject_code"], rec["channel"]) == (
        "rejected", "E_INTERNAL", "mic_local")
    assert [k for k, _ in pairs] == [_KEY]  # no action frame went out
