"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_wake_name.py
Brief: the robot's own name 海卫 marks an utterance as addressed to it (16 S5.2 layer 5)

Description:
What went wrong (2026-10-25 dry run on .1.26). The robot introduces itself as
海卫 (18 J02 party-A identity: name 海卫, full name 海卫-风息), yet an
utterance that started with that name but matched no keyword -- "海卫把这里记为
东门岗亭" -- was judged NOT addressed to the robot by layer 5 and dropped in
silence. 16 S5.2.1 counts "称呼机器人" (addressing the robot) as directed
speech; the wake-word table simply did not contain the robot's name.

What these cases pin:
  * the name alone makes an otherwise unmatched utterance directed, so it
    reaches the tier-2 LLM leg (layer 6) instead of being dropped;
  * the full name 海卫风息 works too (it contains the short name);
  * the same utterance WITHOUT the name is still overheard -- the name is
    what changed the outcome, not some other cue in the sentence.

Mutant this file kills (run by hand, see the batch commit): 海卫 removed from
the wake-word table -> the two directed cases.

Boundaries: layer 5 only (priority_chain.classify_after_bypass with no
earlier-layer match). What the LLM then does with the sentence is the tier-2
path's business (tests/p4_agent/runtime).
"""
from __future__ import annotations

import pytest

from xbrain.p4_agent.classifier.priority_chain import (
    ChainLayer,
    classify_after_bypass,
)

# Pure unit cases: string in, layer decision out.
pytestmark = pytest.mark.no_device

# The 2026-10-25 sentence with its address stripped: no keyword of any
# intent, no imperative marker ("把这里" is not one of 你去/帮我/请你/去把/把它).
_BODY = "把这里记为东门岗亭"


def _layer(text):
    """Layer 5/6 decision for an utterance no earlier layer matched.

    All earlier-layer matches are None: these cases are only about what
    layer 5 does with an unmatched sentence.
    """
    return classify_after_bypass(text=text, long_phrase_match=None,
                                 session_state_match=None,
                                 large_class_match=None)


@pytest.mark.parametrize("name", ["海卫", "海卫风息"])
def test_the_robot_s_name_makes_the_utterance_directed(name):
    """Addressed by name -> layer 6 (to the LLM), not layer 5 overheard."""
    # Name first, the way a person addresses someone: "海卫, ..." (ASR drops
    # the comma, so the name runs straight into the body).
    r = _layer(name + _BODY)
    # Layer 6 (unknown -> LLM), not layer 5 (overheard -> silent).
    assert r.layer == ChainLayer.UNKNOWN.value and r.fires_llm, r


def test_without_the_name_the_same_words_are_still_overheard():
    """The control: the body alone carries no address cue at all.

    Without this case, a layer that judged EVERYTHING directed would pass the
    two cases above -- and every bystander's sentence would reach the LLM.
    """
    # Same words, no name: nothing addresses the robot.
    r = _layer(_BODY)
    # Silent, and the LLM is not woken for it.
    assert r.layer == ChainLayer.OVERHEARD.value and not r.fires_llm, r
