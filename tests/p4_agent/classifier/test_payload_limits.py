"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_payload_limits.py
Brief: Impossible light requests are refused with the reason and dispatch nothing; valid light commands are untouched

Description:
Field test 2026-10-29: "把警灯调亮" / "红蓝暗一点" changed the SEARCHLIGHT
(P239 / P240); rate / searchlight-strobe / out-of-range pattern requests got
"没太听清" (P236-P247). These cases run the production orchestrator on the
production configs:
  * every field phrase answers its chitchat.yaml payload_limits reply and
    publishes no command (in particular no cmd/payload D17);
  * valid phrases ("红蓝模式", "用第三种模式", "把灯调亮", "警灯亮着吗") are not
    caught by the guard.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import pytest  # parametrize, marks
import yaml  # production configs

from xbrain.p4_agent.classifier.payload_limits import cn_number, limit_hit  # under test
from xbrain.p4_agent.registry.intents import load_intent_registry  # 18 registry
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # turn path
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # dialog state
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets

pytestmark = pytest.mark.no_device  # pure in-process

with open("/opt/xbrain_v6/configs/chitchat.yaml", encoding="utf-8") as _fh:  # production content file
    CC = yaml.safe_load(_fh)  # production presets


def _orch():  # production orchestrator
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:  # production content file
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    return build_orchestrator(reg, ChitchatResponder(CC))  # real presets


FIELD = [("把警灯调亮", "redblue_no_dimming"), ("红蓝暗一点", "redblue_no_dimming"),
         ("警示灯亮度调小", "redblue_no_dimming"), ("让警灯闪快点", "strobe_fixed_rate"),
         ("闪慢点", "strobe_fixed_rate"), ("调一下爆闪频率", "strobe_fixed_rate"),
         ("让探照灯闪", "searchlight_no_strobe"), ("探照灯爆闪", "searchlight_no_strobe"),
         ("照明灯闪起来", "searchlight_no_strobe"), ("用第零种模式", "pattern_out_of_range"),
         ("用第十七种模式", "pattern_out_of_range")]


@pytest.mark.parametrize("text, key", FIELD)
def test_field_phrases_are_refused_with_the_reason(text, key):  # one query answer: test field phrases are refused with the reason
    """MUTATION: drop the guard call in the orchestrator -> P239 dispatches
    D17 to the searchlight again -> this fails."""
    d = _orch().handle_turn(text, OrchestratorSession(), 1000, channel="cloud", modality="text")  # local read of the state field
    pubs = decision_to_publishes(d)  # what would go out
    spoken = [p["text"] for k, p in pubs if k.endswith("audio/speak")]  # TTS
    assert spoken == [CC["payload_limits"][key]]  # the reason, once
    assert [k for k, _p in pubs if not k.endswith("audio/speak")] == []  # no command at all


@pytest.mark.parametrize("text", ["红蓝模式", "切换到红蓝爆闪模式", "用第三种模式", "把灯调亮",
                                  "警灯亮着吗", "换一种闪法", "云台快点转", "开警示灯"])
def test_valid_phrases_pass_the_guard(text):  # one query answer: test valid phrases pass the guard
    """MUTATION: drop the device-word requirement -> "把灯调亮" is refused -> fails."""
    assert limit_hit(text) is None  # not ours to refuse


@pytest.mark.parametrize("s, n", [("17", 17), ("十七", 17), ("零", 0), ("三", 3), ("二十", 20), ("十", 10), ("abc", None)])
def test_cn_number(s, n):  # one query answer: test cn number
    """Pattern numbers come spoken; 16 must pass, 0 and 17 must not."""
    assert cn_number(s) == n  # the property this case pins


def test_the_range_edges():  # one query answer: test the range edges
    """MUTATION: PATTERN_MAX 17 -> "第十七种" passes -> this fails; 16 passes."""
    assert limit_hit("用第十六种模式") is None  # the last valid pattern
    assert limit_hit("用第一种模式") is None  # the first valid pattern
    assert limit_hit("用第十七种模式") == "pattern_out_of_range"  # the property this case pins
