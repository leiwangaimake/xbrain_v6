"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_motion_numbers.py
Brief: "前进三米" moves 3 m -- the fast leg reads the number, the LLM leg may not invent one (16 S6.6.4)

Description:
A05-A10 are fastpath_then_llm (16 S6.6.4). Until 2026-10-26 the fast leg read
no numbers and the LLM leg was never entered, so every "前进三米" / "左转九十度"
ended in the clarify question and no motion frame was ever built. Now:

  * the fast leg reads distance_m / angle_deg from the text (slots/
    motion_slots), and the frame carries exactly that value -- no LLM call;
  * with no number on the fast leg, the LLM leg runs once; a number it
    returns must appear in the utterance, else it is dropped and the turn
    asks (the 3B model invents distances for vague commands, and an invented
    number here would be real motion);
  * with the LLM down, the turn still asks rather than refusing;
  * the grounding applies on the layer-6 path too (no keyword, LLM picked the
    motion intent itself).

Nothing reaches a robot: the handler only returns frames.

Mutants this file kills (run by hand, see the commit).

Boundaries: P4's slot. Limits (G-3, 20 m) and the actual motion are P2's.
"""
from __future__ import annotations

import json  # frames are JSON bytes

import pytest  # raises, parametrize, marker
import yaml  # the real content files, as p4 loads them

from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.orchestrator_turn import (
    build_orchestrator,  # as p4's wiring builds it
    make_turn_handler,  # the microphone entry
)
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession,  # a fresh dialog per case
    Tier2Classification,  # what a tier-2 call returns
)
from xbrain.p4_agent.session.chitchat import ChitchatResponder
from xbrain.p4_agent.slots.motion_slots import (
    ground_llm_slots,  # the guard on LLM numbers
    parse_number,  # Chinese / arabic numerals
)

# No robot, no Zenoh, no LLM service (tier-2 is a scripted fake).
pytestmark = pytest.mark.no_device  # runs anywhere

#: The content files p4's __main__ loads (absolute: no home-dir copies).
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"  # 18 registry
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"  # chitchat presets


class _LLM:
    """A tier-2 fake: counts calls, answers with a scripted classification.

    answer None = the LLM is unavailable / declined (tier2_fn returns None).
    """

    def __init__(self, answer):
        self.answer, self.calls = answer, 0  # scripted reply, call count

    def __call__(self, _text, _session, _now_ms):
        """The tier2_fn signature the orchestrator calls."""
        self.calls += 1  # one more call
        return self.answer  # the scripted classification


def _turn(text, llm=None):
    """One mic turn; (motion frame slots or None, spoken text, llm fake)."""
    with open(_INTENTS, encoding="utf-8") as fh:  # production content
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    with open(_CHITCHAT, encoding="utf-8") as fh:  # production content
        cc = ChitchatResponder(yaml.safe_load(fh))  # chitchat presets
    llm = llm or _LLM(None)  # default: no LLM available
    orch = build_orchestrator(reg, cc, l2_timeout_ms=8000, tier2_fn=llm)
    pairs = make_turn_handler(orch, OrchestratorSession())(text)  # frames, nothing sent
    frames = [json.loads(d) for k, d in pairs if k == "cmd/motion/intent"]
    spoken = [json.loads(d)["text"] for k, d in pairs
              if k == "cmd/audio/speak"]  # the spoken reply
    slots = frames[0]["data"]["slots"] if frames else None  # None: no motion
    return slots, (spoken[0] if spoken else ""), llm  # what the case checks


# -- the fast leg reads the number ----------------------------------------

@pytest.mark.parametrize("text,slots", [
    ("前进三米", {"distance_m": 3.0}),          # Chinese numeral + 米
    ("前进1.5米", {"distance_m": 1.5}),         # arabic decimal
    ("后退半米", {"distance_m": 0.5}),          # 16 S6.7 M1: 半米 = 0.5
    ("前进一米半", {"distance_m": 1.5}),        # trailing 半 = +0.5
    ("后退五十厘米", {"distance_m": 0.5}),      # S8.0.4: cm / 100
    ("前进两米", {"distance_m": 2.0}),          # 两 as a numeral
    ("左移十二米", {"distance_m": 12.0}),       # 十二, no limit applied here
    ("左转九十度", {"angle_deg": 90.0}),        # degrees
    ("右转45度", {"angle_deg": 45.0}),          # arabic degrees
    ("左转一百八十度", {"angle_deg": 180.0}),   # 一百八十
])
def test_the_fast_leg_reads_the_number_and_moves(text, slots):
    """The frame carries the spoken value, and the LLM is never called."""
    got, _spoken, llm = _turn(text)  # one mic turn
    assert got == slots  # exactly the spoken value
    assert llm.calls == 0  # the fast leg finished on its own


# -- the LLM leg, and its guard -------------------------------------------

def test_no_number_runs_the_llm_once_and_its_invention_is_dropped():
    """"前进" with an LLM that invents 1 m: asked, not moved.

    This is the defect that made opening the LLM leg dangerous: the model
    fills a distance nobody said.
    """
    invent = _LLM(Tier2Classification("move_forward", {"distance_m": 1}))  # invents 1 m
    slots, spoken, llm = _turn("前进", invent)  # no number said
    assert llm.calls == 1  # the LLM leg ran, once
    assert slots is None  # no motion frame
    assert "多远" in spoken  # the MI-2 clarify


def test_a_number_the_llm_found_in_the_text_is_used():
    """"前进三" (no unit): the fast leg needs a unit, the LLM's 3 is said."""
    grounded = _LLM(Tier2Classification("move_forward", {"distance_m": 3}))  # returns 3
    slots, _spoken, llm = _turn("前进三", grounded)  # 三 said, no unit
    assert (slots, llm.calls) == ({"distance_m": 3}, 1)  # moved 3, LLM once


def test_with_the_llm_down_the_turn_still_asks():
    """tier-2 unavailable: the clarify question, not a refusal."""
    slots, spoken, llm = _turn("左转", _LLM(None))  # LLM unavailable
    assert llm.calls == 1 and slots is None  # asked once, no motion
    assert "多少度" in spoken  # asked, so the operator can answer


def test_the_layer6_path_is_guarded_too():
    """No keyword at all; the LLM picks move_forward and invents 1 m.

    The guard sits where LLM slots are merged, so it covers this path as
    well as the fastpath_then_llm leg.
    """
    invent = _LLM(Tier2Classification("move_forward", {"distance_m": 1}))  # invents 1 m
    slots, _spoken, _llm = _turn("往那边挪一挪吧机器人", invent)  # no keyword, a particle 一
    assert slots is None  # nothing moved on an invented number


# -- the parser and the guard, directly -----------------------------------

@pytest.mark.parametrize("token,value", [
    ("三", 3.0), ("十", 10.0), ("十二", 12.0), ("二十", 20.0),  # positional Chinese
    ("一百八十", 180.0), ("一点五", 1.5), ("2.5", 2.5),  # decimals
    ("一二", None), ("十百", None), ("点点", None), ("一点", None),  # malformed: refused
])
def test_numerals_parse_or_refuse(token, value):
    """Malformed numerals give None, never a guess."""
    assert parse_number(token) == value  # exact or None


@pytest.mark.parametrize("value,text,kept", [
    (3, "前进三", True),          # said as 三
    (0.5, "后退五十厘米", True),  # 50 / 100
    (1.5, "前进一米半", True),    # 1 + 半
    (1, "前进", False),           # nothing said: invented
    (4, "前进三", False),         # a different number than said
    (0, "前进零", False),         # positive only
    (True, "前进一", False),      # a bool is not a number
    (1, "往那边挪一挪", False),   # 一 as a particle, not a quantity
    (1, "转一下", False),         # "a bit", a count
    (2, "往前走两步", False),     # two STEPS, not two metres
    (0.5, "走一半", False),       # 半 without 米 is not 0.5 m
    (1, "前进一米吧", True),      # a bare 一 with a unit after it counts
])
def test_the_grounding_rule(value, text, kept):
    """Kept only when the value is in the utterance (as said / cm / +half)."""
    out = ground_llm_slots("move_forward", {"distance_m": value}, text)  # one LLM slot
    assert ("distance_m" in out) is kept  # kept or dropped as expected
