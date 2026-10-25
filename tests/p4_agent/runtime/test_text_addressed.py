"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_text_addressed.py
Brief: typed cmd/voice_text is directed by construction; cloud audio still meets the overheard gate

Description:
16 S5.2.1 layer 5 drops, silently, an utterance that matched no intent and
carries no address cue (wake word / robot name / second-person imperative).
Its reason is the microphone: it hears soldiers talking to each other. A
command somebody TYPED and sent to this robot over cmd/voice_text has no
bystanders, yet until 2026-10-25 the text channel ran the same gate, so
"今天天气怎么样" typed from the cloud was dropped without a word -- the
operator could not tell a typo from a dead link.

What these cases pin:
  * layer 5 with addressed=True sends an unmatched sentence on to layer 6
    (the LLM) instead of silence, and broken text ("嗯") is STILL overheard
    -- addressed settles who the text is for, not whether it means anything;
  * the default (every microphone path) is unchanged: the same sentence is
    overheard;
  * end to end, handle_text_command applies addressed=True to typed text and
    lets cloud AUDIO (the other caller) pass addressed=False;
  * the raw ASR text reaches the safety bypass through handle_text_command,
    so a stop word that post-processing lost still stops (16 S4 constraint 1).

Mutants this file kills (run by hand, see the commit):
  * addressed ignored in classify_after_bypass      -> layer case
  * broken-text check waived with addressed          -> broken case
  * addressed not forwarded by classify_text         -> end-to-end case
  * addressed not forwarded by handle_turn           -> end-to-end case
  * handle_text_command drops addressed / raw_text   -> end-to-end / bypass

Boundaries: layer 5 and the text entry only. What the LLM answers is tier-2
(not wired here: build_orchestrator falls back to the null tier-2), and the
channel admission gate is test_text_channel.py.
"""
from __future__ import annotations

import pytest
import yaml

from xbrain.p4_agent.classifier.keyword_matcher import KeywordMatcher
from xbrain.p4_agent.classifier.priority_chain import (
    ChainLayer,
    classify_after_bypass,
)
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator
from xbrain.p4_agent.runtime.text_channel import handle_text_command
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession
from xbrain.p4_agent.session.chitchat import ChitchatResponder

# Table reads and an in-process orchestrator; no robot, no Zenoh, no ASR or
# LLM service -- the cases must run on any machine with the repo, so a red
# result always means the code, never a service that happened to be down.
pytestmark = pytest.mark.no_device

#: The two content files p4's __main__ loads for the orchestrator. Absolute,
#: like every other p4 test: the suite is run from the repo root on .1.26,
#: and a relative path would silently pick up a stale home-directory copy.
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"
#: Chitchat presets; needed only because build_orchestrator requires them.
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"

#: No keyword of any intent, no device-family word for layer 4, and no
#: address cue: through the full pipeline it misses layers 2-4, so layer 5
#: alone decides it. (Probed on .1.26 2026-10-25; "把这里记为东门岗亭",
#: test_wake_name's control, is NOT usable here -- it hits F06 at layer 2.)
_BODY = "今天天气怎么样"


def _layer(text, **kw):
    """Layer 5/6 decision for a sentence no earlier layer matched.

    kw carries addressed (or nothing, for the microphone default), so the
    three layer cases differ in exactly the argument under test.
    """
    # All earlier layers missed: only layer 5's verdict is under test.
    return classify_after_bypass(text=text, long_phrase_match=None,
                                 session_state_match=None,
                                 large_class_match=None, **kw)


def _orch():
    """The production orchestrator over the real registry and chitchat.

    Built fresh per case so no pending confirm or chitchat state leaks from
    one case into the next; the session is passed in separately anyway.
    """
    # The registry p4 loads in production; a keyword added to it later that
    # happens to match _BODY would turn the end-to-end case red, which is the
    # signal to pick another sentence, not to loosen the assertion.
    with open(_INTENTS, encoding="utf-8") as fh:
        reg = load_intent_registry(yaml.safe_load(fh))
    with open(_CHITCHAT, encoding="utf-8") as fh:
        cc = ChitchatResponder(yaml.safe_load(fh))
    # Same constructor main_wiring uses; tier-2 left at its null default.
    return build_orchestrator(reg, cc, l2_timeout_ms=8000), KeywordMatcher(reg)


def _msg(text):
    """A cloud cmd/voice_text body (11 S8.7.5), reply muted.

    require_tts_reply=False keeps the cases about the decision, not about
    which speak frames were built; the field is not what is under test.
    """
    # Only the fields handle_text_command reads; cmd_id / issued_ts are the
    # gateway's bookkeeping and play no part in the decision.
    return {"channel": "cloud", "text": text, "require_tts_reply": False}


def test_an_addressed_sentence_goes_to_the_llm_not_silence():
    """addressed=True: layer 6 (unknown -> LLM).

    The sentence has no wake word and no imperative marker, so before the
    change this exact call (minus the flag) was the overheard control.
    """
    # Typed-text form of the call: the channel vouches for the address.
    r = _layer(_BODY, addressed=True)
    # Layer 6 and the LLM is woken: heard, not dropped.
    assert r.layer == ChainLayer.UNKNOWN.value and r.fires_llm, r


def test_broken_text_stays_overheard_even_when_addressed():
    """One character is semantically broken whoever it was sent to.

    Without this case, waiving layer 5 entirely for typed text would pass
    the case above -- and a stray keystroke would wake the LLM.
    """
    # Addressed, but under the two-character floor of is_semantically_broken.
    r = _layer("嗯", addressed=True)
    # Still silent: "addressed" never overrides "means nothing".
    assert r.layer == ChainLayer.OVERHEARD.value and not r.fires_llm, r


def test_the_microphone_default_is_unchanged():
    """The control: without addressed the same sentence is overheard.

    Guards the other direction: a change that made layer 5 lenient for
    everyone would pass the addressed case and fail here.
    """
    # The microphone form of the call: no flag, the default False.
    r = _layer(_BODY)
    # Silent, exactly as before 2026-10-25.
    assert r.layer == ChainLayer.OVERHEARD.value, r


def test_typed_text_is_not_dropped_but_cloud_audio_still_can_be():
    """End to end through handle_text_command, both callers.

    Typed text (default addressed) must not come back "overheard"; the same
    words arriving as cloud audio (addressed=False) must.

    Both halves run through every forwarding hop (text_channel ->
    handle_turn -> classify_text -> classify_after_bypass), so a hop that
    drops the flag turns one half red: the typed half if it is lost, the
    heard half if it is forced on.
    """
    # One orchestrator for both halves: the flag, not the instance, is what
    # differs, so the two verdicts can only differ because of it.
    orch, matcher = _orch()
    # Typed: directed by construction -> anything but silence.
    typed = handle_text_command(_msg(_BODY), orch, OrchestratorSession(),
                                matcher, now_mono_ms=1000)
    # Layer 6 is where it must land: the null tier-2 then declines it, but
    # it was heard and answered rather than dropped.
    assert typed.decision.layer == "unknown", typed.decision
    # Heard through a far-end microphone: the gate applies as for local mic.
    # Fresh session again: the typed turn must not leave a pending confirm
    # that this turn would be read as the answer to.
    heard = handle_text_command(_msg(_BODY), orch, OrchestratorSession(),
                                matcher, now_mono_ms=1000, addressed=False)
    assert heard.decision.kind == "overheard", heard.decision


def test_raw_asr_text_reaches_the_safety_bypass():
    """A stop word in the RAW text stops, whatever post-processing made of it.

    This is the cloud-audio caller's contract with handle_text_command: it
    hands over the corrected text AND what ASR actually returned, and the
    bypass must match the latter first (16 S4 constraint 1).

    The corrected text carries no stop word at all; only raw_text does. If
    handle_text_command stopped forwarding raw_text, the bypass would see
    the corrected text alone and this turn would not stop.
    """
    orch, matcher = _orch()
    # Corrected text = the harmless sentence; raw text = an estop word.
    res = handle_text_command(_msg(_BODY), orch, OrchestratorSession(),
                              matcher, now_mono_ms=1000,
                              raw_text="急停", addressed=False)
    # The bypass fired (Tier1 estop), ahead of every classifier layer.
    assert res.decision.kind == "bypass", res.decision
