"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_bypass_posture.py
Brief: "趴下" / "站起来" leave P4 as an 11 S9.3.4 cmd/chassis/ctrl command (MI-5)

Description:
11 MI-5 sends the two non-estop voice bypasses (A02 prone / A03 stand) to
cmd/chassis/ctrl (S9.3.4), never to cmd/motion/intent. Until 2026-10-26 P4
published them on cmd/motion/intent as a p4_intent_v1 frame: P2's motion
gate refused it (no S3.0 envelope) and p1's legacy forwarder sent it to the
chassis as an APDU the chassis does not know. The voice acknowledged, the
robot did not move (commx live test 2026-10-26).

What is checked here, on the frames the turn handler returns (nothing is
sent to any robot):

  * the posture frame is on cmd/chassis/ctrl with the S9.3.4 body
    {v, cmd_id, action, origin}, and NOTHING goes to cmd/motion/intent;
  * cmd_id is fresh per turn (quadruped's Q-3 idempotency treats a repeated
    cmd_id as a duplicate and would ignore the second "站起来");
  * origin follows the turn's entry: the microphone is voice, a cloud turn
    is cloud (S9.3.4's closed set hmi | cloud | voice | wecom);
  * an action outside stand / prone raises (11 S13.6), so enable -- which
    needs an L2 confirm_token -- can never leave through the bypass;
  * the estop bypass is unchanged (still cmd/estop).

Boundaries: P4's frame only. chassis_relay CR-11 re-envelopes it onto
rt/chassis/ctrl and quadruped executes it; both have their own tests.
"""
from __future__ import annotations

import json  # frames are JSON bytes

import pytest  # raises, parametrize, marker
import yaml  # the real content files, as p4 loads them

from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.orchestrator_turn import (
    build_orchestrator,  # as p4's wiring builds it
    decision_to_publishes,  # the branch under test
    make_turn_handler,  # the microphone entry
)
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession,  # a fresh dialog per case
    TurnDecision,  # for the out-of-set case
)
from xbrain.p4_agent.session.chitchat import ChitchatResponder

# No robot, no Zenoh, no LLM service.
pytestmark = pytest.mark.no_device  # runs anywhere

#: The content files p4's __main__ loads (absolute: no home-dir copies).
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"  # 18 registry
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"  # chitchat presets


def _orch():
    """The production orchestrator with the real registry and presets."""
    with open(_INTENTS, encoding="utf-8") as fh:  # production content
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    with open(_CHITCHAT, encoding="utf-8") as fh:  # production content
        cc = ChitchatResponder(yaml.safe_load(fh))  # chitchat presets
    return build_orchestrator(reg, cc, l2_timeout_ms=8000)  # no LLM needed


def _mic(text):
    """One microphone turn; {key: [payload, ...]} of what it would publish."""
    pairs = make_turn_handler(_orch(), OrchestratorSession())(text)  # frames only
    out = {}  # key -> decoded payloads
    for key, data in pairs:
        out.setdefault(key, []).append(json.loads(data))  # decode each frame
    return out


@pytest.mark.parametrize("text,action", [
    ("趴下", "prone"),     # A02
    ("站起来", "stand"),   # A03
])
def test_posture_goes_to_chassis_ctrl_with_the_s934_body(text, action):
    """cmd/chassis/ctrl, body {v, cmd_id, action, origin}, nothing else."""
    out = _mic(text)  # one microphone turn
    assert "cmd/motion/intent" not in out  # MI-5: never the motion key
    (frame,) = out["cmd/chassis/ctrl"]  # exactly one posture frame
    assert set(frame) == {"v", "cmd_id", "action", "origin"}  # S9.3.4 fields
    assert frame["v"] == 1  # S3.0 envelope version
    assert frame["action"] == action  # the spoken posture
    assert frame["origin"] == "voice"  # the microphone
    assert frame["cmd_id"].startswith("cc-")  # non-empty, this file's prefix


def test_each_turn_gets_its_own_cmd_id():
    """Two "站起来" turns: two cmd_ids, so Q-3 does not drop the second."""
    a = _mic("站起来")["cmd/chassis/ctrl"][0]["cmd_id"]  # first turn
    b = _mic("站起来")["cmd/chassis/ctrl"][0]["cmd_id"]  # second turn
    assert a != b  # fresh per turn


def test_a_cloud_turn_says_cloud():
    """The same words typed in the cloud: origin cloud, not voice."""
    orch = _orch()  # production orchestrator
    d = orch.handle_turn("站起来", OrchestratorSession(), 0,
                         channel="cloud", modality="text", addressed=True)
    ((key, frame),) = decision_to_publishes(d)  # one frame
    assert (key, frame["origin"]) == ("cmd/chassis/ctrl", "cloud")


def test_an_action_outside_stand_prone_raises():
    """enable (needs a confirm_token) or anything else never leaves here."""
    for action in ("enable", "jump"):  # legal-on-key and garbage
        with pytest.raises(ValueError):
            decision_to_publishes(TurnDecision(kind="bypass",
                                               bypass_action=action))


def test_the_estop_bypass_is_unchanged():
    """"急停" still goes to cmd/estop and nothing goes to chassis/ctrl."""
    out = _mic("急停")  # the A01 estop
    assert out["cmd/estop"][0]["action"] == "stop"  # S7.1 single value
    assert "cmd/chassis/ctrl" not in out  # posture key untouched
