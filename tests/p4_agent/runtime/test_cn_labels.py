"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cn_labels.py
Brief: the spoken ack names the action that was dispatched, not another intent's

Description:
turn_orchestrator._CN_LABELS gives the short spoken ack ("好的,<label>") per
18 id. Until 2026-10-26 A05 / A06 carried "站起来" / "趴下" -- labels left from
an older id plan -- while the registry says A05 = move_forward and A06 =
move_backward. Once A05-A10 started sending motion frames (8af46dd), "前进三米"
moved the robot forward while it said "stand up".

What is pinned:
  * no label is a keyword of a DIFFERENT intent -- the shape of that defect,
    checked for every label, so a future renumbering cannot reintroduce it;
  * end to end, "前进三米" / "后退半米" / "左转九十度" are acknowledged with
    their own action, and with no number in the ack (11 CMD-30: TTS restates
    only measured values, which come later in P2's ack).

Mutants this file kills (run by hand, see the commit).
"""
from __future__ import annotations

import json  # frames are JSON bytes

import pytest  # parametrize and the marker
import yaml  # the real registry

from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime import turn_orchestrator as orch_mod  # the label table
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


def _registry():
    """The production registry."""
    with open(_INTENTS, encoding="utf-8") as fh:  # production content
        return load_intent_registry(yaml.safe_load(fh))  # 18 registry


def test_no_label_is_another_intent_s_keyword():
    """A label that is some OTHER intent's keyword names the wrong action.

    "站起来" is a keyword of stand (A03); hung on A05 it made a forward move
    say "stand up". Checked for every label in the table.
    """
    reg = _registry()  # every intent
    wrong = []  # (id, label, other id)
    for intent_id, label in orch_mod._CN_LABELS.items():  # every label
        for e in reg.entries:  # every intent's keywords
            if e.id != intent_id and label in e.keywords:  # another intent owns it
                wrong.append((intent_id, label, e.id))  # label -> other id
    assert wrong == []  # none found


@pytest.mark.parametrize("text,ack", [
    ("前进三米", "好的,前进"),      # A05, was "好的,站起来"
    ("后退半米", "好的,后退"),      # A06, was "好的,趴下"
    ("左转九十度", "好的,左转"),    # A09, was the generic "收到"
])
def test_a_motion_command_is_acknowledged_with_its_own_action(text, ack):
    """The spoken ack names the action, and carries no requested number."""
    with open(_CHITCHAT, encoding="utf-8") as fh:  # production content
        cc = ChitchatResponder(yaml.safe_load(fh))  # chitchat presets
    orch = build_orchestrator(_registry(), cc, l2_timeout_ms=8000)  # no tier-2 needed
    pairs = make_turn_handler(orch, OrchestratorSession())(text)  # frames, nothing sent
    spoken = [json.loads(d)["text"] for k, d in pairs
              if k == "cmd/audio/speak"]  # the spoken ack
    assert spoken == [ack]  # exactly this, nothing else said
