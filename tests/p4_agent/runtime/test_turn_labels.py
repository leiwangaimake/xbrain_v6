"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_turn_labels.py
Brief: every command a turn builds says where the turn came from, in its own contract's spelling

Description:
Until 2026-10-26 P4 labelled every command with one constructor constant,
source="voice", although one orchestrator serves the microphone, cloud text,
cloud voice and the HMI. Effects, per key: a cloud task queued as on-site
(P3 priority 40 instead of 80); cmd/mode carrying "voice", outside 11 S7.3's
closed set; cmd/system judged by the S7.15.4 permission matrix as local voice
whatever its entry; cmd/geo origin hard-wired to "voice"; cmd/teach
issuer.channel local_voice for everything. runtime/turn_labels now maps the
TURN's channel + modality onto each contract, and every builder re-checks
its closed set.

What is pinned:
  * the mapping table, per contract and per channel (cloud voice vs cloud
    text where S12A.4 distinguishes them);
  * impossible or unknown turns raise (mic_local + text, hmi + voice, typos);
  * each builder refuses a value outside its contract's set -- including the
    old constant "voice" for cmd/mode;
  * end to end through the orchestrator: the same phrase sent on different
    entries produces frames labelled with each entry.

Mutants this file kills (run by hand, see the commit).

Boundaries: labels. What P3 / P2 do with them is tested on their side.
"""
from __future__ import annotations

import pytest  # raises, parametrize, marker
import yaml  # the real content files, as p4 loads them

from xbrain.p4_agent.classifier.keyword_matcher import KeywordMatcher
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime import turn_labels as tl  # the mapping under test
from xbrain.p4_agent.runtime.cloud_input import (
    CloudTurnRunner,  # the cloud voice entry
    CompletedUtterance,  # a finished cloud sentence
)
from xbrain.p4_agent.runtime.geo_request import to_geo_command  # S7.9.5 builder
from xbrain.p4_agent.runtime.mode_request import to_mode_command  # S7.3 builder
from xbrain.p4_agent.runtime.orchestrator_turn import decision_to_publishes
from xbrain.p4_agent.runtime.system_request import to_system_command  # S7.15 builder
from xbrain.p4_agent.runtime.task_control_request import (
    to_task_control_command,  # B-class control, cmd/task
)
from xbrain.p4_agent.runtime.teach_request import to_teach_command  # S12A.4 builder
from xbrain.p4_agent.runtime.text_channel import handle_text_command  # the text entry
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession,  # a fresh dialog per turn
    Tier2Classification,  # what the scripted LLM returns
    TurnOrchestrator,  # constructed directly to inject state
)
from xbrain.p4_agent.session.chitchat import ChitchatResponder

# No robot, no Zenoh, no ASR / LLM service.
pytestmark = pytest.mark.no_device  # runs anywhere

#: The content files p4's __main__ loads (absolute: no home-dir copies).
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"  # 18 registry
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"  # chitchat presets

#: One stored route (11 S7.10 manifest) for the geo delete case.
_MANIFEST = {"state/geo/manifest": {"items": [
    {"geo_id": "r-east", "type": "route", "name": "东门路线",
     "state": "active", "num": 1}]}}  # one active route
#: A directed phrase matching no keyword, so the scripted tier-2 decides.
_UNMATCHED = "帮我处理一下那个事情"  # reaches layer-6 tier-2

#: A running task (11 S4.4 shape) so B-class control can resolve its id.
_STATE = {"state/task": {"schema": "task_state_v1",
                         "current": {"task_id": "t-1", "state": "running"},
                         "queue": [], "suspended": []}}  # nothing queued


# -- the mapping table ---------------------------------------------------

@pytest.mark.parametrize("channel,task,mode,orig", [
    ("mic_local", "voice", "voice_local", "voice"),  # S7.3 says voice_local
    ("cloud", "cloud", "cloud", "cloud"),  # P3 priority 80, not 40
    ("wecom", "wecom", "wecom", "wecom"),  # its own row in every table
    ("hmi", "hmi", "hmi", "hmi"),  # P3 maps hmi -> local
])
def test_each_contract_gets_its_own_spelling(channel, task, mode, orig):
    """cmd/task source, cmd/mode source, cmd/system + cmd/geo origin."""
    assert tl.task_source(channel) == task  # cmd/task source
    assert tl.mode_source(channel) == mode  # cmd/mode source
    assert tl.origin(channel) == orig  # cmd/system + cmd/geo origin


@pytest.mark.parametrize("channel,modality,issuer", [
    ("mic_local", "voice", "local_voice"),  # the microphone
    ("cloud", "voice", "cloud_voice"),  # S12A.4 tells cloud voice ...
    ("cloud", "text", "cloud_text"),  # ... from cloud text
    ("wecom", "voice", "wecom"),  # spoken WeChat
    ("wecom", "text", "wecom"),  # typed WeChat
    ("hmi", "text", "hmi"),  # was cloud_text under the old map
])
def test_teach_issuer_channel_records_the_modality(channel, modality, issuer):
    """cmd/teach issuer.channel is the one label that also says how."""
    assert tl.teach_issuer_channel(channel, modality) == issuer


@pytest.mark.parametrize("channel,modality", [
    ("mic_local", "text"),  # the microphone only hears speech
    ("hmi", "voice"),  # the HMI only sends text
    ("radio", "voice"),  # not an entry
    ("cloud", "video"),  # not a modality
])
def test_impossible_turns_raise(channel, modality):
    """Refused, never repaired into the nearest valid pair."""
    with pytest.raises(ValueError):  # refused, not repaired
        tl.check_turn(channel, modality)  # the guard


# -- the builders re-check their own closed set ------------------------------

@pytest.mark.parametrize("build", [
    lambda: to_mode_command("enter_broadcast", slots={}, cmd_id="c",  # the old constant
                            source="voice"),  # the old, off-set constant
    lambda: to_system_command("reboot", slots={}, cmd_id="c",
                              source="text"),  # not an S7.15 origin
    lambda: to_task_control_command("pause_task", state=_STATE, cmd_id="c",
                                    source="text"),  # not a task source
    lambda: to_teach_command("record_route_start", slots={}, cmd_id="c",
                             issuer_channel="text"),  # not S12A.4
    lambda: to_geo_command("delete_route", slots={}, cmd_id="c",
                           manifest=[], origin="radio"),  # not S7.9.5
])
def test_builders_refuse_values_outside_their_contract(build):
    """ValueError: a caller bug, not a turn to answer with "say again"."""
    with pytest.raises(ValueError):  # a caller bug
        build()  # must raise


# -- end to end through the orchestrator -------------------------------------

def _orch(tier2=None):
    """The production registry, a running task and one stored route.

    tier2 scripts the LLM for the cases that need a layer-6 classification
    (system and geo intents carry slots only tier-2 fills).
    """
    with open(_INTENTS, encoding="utf-8") as fh:  # production content
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    with open(_CHITCHAT, encoding="utf-8") as fh:  # production content
        cc = ChitchatResponder(yaml.safe_load(fh))  # chitchat presets
    return TurnOrchestrator(reg, chitchat=cc,  # the real orchestrator
                            tier2_fn=tier2 or (lambda *a, **k: None),
                            l2_timeout_ms=5000,  # confirm window
                            state_fn=lambda: {**_STATE, **_MANIFEST})


def _frame(text, key, channel, modality):
    """The frame on `key` for one turn of `text` on the given entry."""
    d = _orch().handle_turn(text, OrchestratorSession(), 1000,  # one turn
                            channel=channel, modality=modality)  # on the given entry
    frames = dict(decision_to_publishes(d))  # key -> payload
    assert key in frames, (text, d.kind, list(frames))  # the frame went out
    return frames[key]  # its payload


@pytest.mark.parametrize("channel,modality,source", [
    ("mic_local", "voice", "voice_local"),  # S7.3 voice_local
    ("cloud", "text", "cloud"),  # typed cloud
    ("wecom", "voice", "wecom"),  # spoken WeChat
    ("hmi", "text", "hmi"),  # HMI text
])
def test_a_mode_command_is_labelled_with_its_entry(channel, modality, source):
    """"开始喊话" (C class) on each entry -> cmd/mode source per S7.3."""
    assert _frame("开始喊话", "cmd/mode", channel, modality)["source"] == source


def test_a_task_control_from_the_cloud_says_cloud():
    """"暂缓任务" typed on the cloud -> cmd/task source cloud, not voice."""
    assert _frame("暂缓任务", "cmd/task", "cloud", "text")["source"] == "cloud"


@pytest.mark.parametrize("channel,modality,issuer", [
    ("mic_local", "voice", "local_voice"),  # the microphone
    ("cloud", "voice", "cloud_voice"),  # spoken cloud
    ("cloud", "text", "cloud_text"),  # typed cloud
])
def test_a_recording_command_names_its_issuer(channel, modality, issuer):
    """"开始录制路径" -> cmd/teach issuer.channel per S12A.4."""
    frame = _frame("开始录制路径", "cmd/teach", channel, modality)  # F01 on each entry
    assert frame["issuer"]["channel"] == issuer  # S12A.4 value


def _confirmed(tier2, channel, modality, key):
    """Two turns on one entry -- the request, then "是" -- and the frame.

    System and destructive geo commands are L2: they only go out after the
    operator confirms, and the confirming turn comes in on the same entry.
    """
    orch, session = _orch(tier2), OrchestratorSession()  # one dialog
    first = orch.handle_turn(_UNMATCHED, session, 1, channel=channel,
                             modality=modality)  # same entry both turns
    assert first.kind == "await_confirm", first.kind  # L2 asked first
    d = orch.handle_turn("是", session, 2, channel=channel, modality=modality)
    return dict(decision_to_publishes(d))[key]  # the confirmed frame


def test_a_system_command_from_the_cloud_carries_origin_cloud():
    """S7.15.4 judges permissions by origin; a cloud reboot must say cloud.

    The scripted slots CLAIM origin "hmi": the frame must ignore any
    self-declared origin and carry the turn's own (origin is an
    authorisation boundary, never taken from slots).
    """
    tier2 = lambda *a, **k: Tier2Classification(  # noqa: E731
        name="reboot", slots={"origin": "hmi", "delay_s": 5})  # a hostile claim
    frame = _confirmed(tier2, "cloud", "text", "cmd/system")  # cloud entry
    assert frame["origin"] == "cloud"  # the turn's, not the slot's


def test_a_geo_command_from_wecom_carries_origin_wecom():
    """S7.9.5's write matrix reads origin; WeChat must not pass as voice.

    (Until 2026-10-26 the expression was `"voice" if ... else "voice"`.)
    """
    tier2 = lambda *a, **k: Tier2Classification(  # noqa: E731
        name="delete_route", slots={"route": "东门路线"})  # the stored route
    frame = _confirmed(tier2, "wecom", "voice", "cmd/geo")  # WeChat voice
    assert frame["origin"] == "wecom"  # not voice


def test_typed_cloud_text_reaches_the_frame_as_cloud_text():
    """Through the text ENTRY, not the orchestrator directly.

    handle_text_command must pass modality "text" on: dropped, the
    orchestrator's default ("voice") would label a typed recording command
    cloud_voice. Only a case that goes through the entry can see that.
    """
    orch = _orch()  # no tier-2 needed
    res = handle_text_command(  # the real text entry
        {"channel": "cloud", "text": "开始录制路径",
         "require_tts_reply": False},  # muted reply
        orch, OrchestratorSession(), KeywordMatcher(orch._registry), 1000)
    frame = dict(decision_to_publishes(res.decision))["cmd/teach"]
    assert frame["issuer"]["channel"] == "cloud_text"  # typed, from the cloud


def test_spoken_cloud_audio_reaches_the_frame_as_cloud_voice():
    """Through the cloud VOICE entry: modality "voice" must survive it.

    The twin of the case above -- if the runner said "text", a spoken
    recording command would be audited as typed (cloud_text).
    """
    orch, published = _orch(), []  # frames collected
    runner = CloudTurnRunner(orch, KeywordMatcher(orch._registry),  # the real runner
                             lambda k, d: published.append((k, d)),  # collects frames
                             lambda pcm: "开始录制路径",  # the fake ASR
                             lambda: 1000, lambda: None,  # clock, no B mode
                             OrchestratorSession())  # local slot
    res = runner.run_voice(CompletedUtterance("s-1", "cloud",
                                              b"\x01\x00" * 3200))  # 200 ms
    frame = dict(decision_to_publishes(res.decision))["cmd/teach"]
    assert frame["issuer"]["channel"] == "cloud_voice"  # spoken, from the cloud
