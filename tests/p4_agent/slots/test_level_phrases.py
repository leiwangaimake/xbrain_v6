"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_level_phrases.py
Brief: Field phrases for volume / brightness / strobe pattern / PTZ amount reach their intent with the right value; elliptical phrases follow the session

Description:
Field test 2026-10-29 (P141-P156, P185, P187): "音量调到七十", "全功率照明",
"中等", "一般亮", "别那么亮", "最暗", "用第三种", "换一种", "换个样式" got
"没太听清" or were overheard; "云台往上微调" turned a normal step. The slot
parsers already knew most of these words -- the intent keyword lists did not,
so the turn never reached them. These cases run the production orchestrator:
  * each phrase dispatches its intent with the expected value;
  * spoken and Arabic volume numbers set the volume (clamped 0..100);
  * an elliptical phrase continues the volume / brightness adjustment of this
    session, or asks which one -- never "没太听清".
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import pytest  # parametrize, marks
import yaml  # production configs

from xbrain.p4_agent.registry.intents import load_intent_registry  # 18 registry
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # turn path
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # dialog state
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets
from xbrain.p4_agent.slots.payload_slots import parse_volume  # numeric volume

pytestmark = pytest.mark.no_device  # pure in-process

with open("/opt/xbrain_v6/configs/chitchat.yaml", encoding="utf-8") as _fh:  # production content file
    CC = yaml.safe_load(_fh)  # production presets


def _orch():  # production orchestrator
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:  # production content file
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    return build_orchestrator(reg, ChitchatResponder(CC))  # real presets


def _cmd(o, s, text):  # one cloud turn -> (decision, first non-speak payload or None)
    d = o.handle_turn(text, s, 1000, channel="cloud", modality="text")  # the turn
    cmds = [p for k, p in decision_to_publishes(d) if not k.endswith("audio/speak")]  # commands
    return d, (cmds[0] if cmds else None)  # result to the caller


@pytest.mark.parametrize("text, intent, field, value", [
    ("音量调到七十", "D10", "volume", {"abs": 70}), ("音量调到70", "D10", "volume", {"abs": 70}),
    ("全功率照明", "D17", "level", "max"), ("一般亮", "D17", "level", "mid"),
    ("别那么亮", "D17", "level", "low"), ("最暗", "D17", "level", "min"),
    ("灯开到最大", "D17", "level", "max"), ("用第三种", "D18", "mode", 3),
    ("换一种", "D18", "mode", None), ("换个样式", "D18", "mode", None),
    ("云台往上微调", "E01", "amount", "small")])
def test_field_phrase_reaches_its_intent_with_the_value(text, intent, field, value):  # one query answer: test field phrase reaches its intent with the value
    """MUTATION: drop the new D17 keywords -> "全功率照明" is overheard -> fails."""
    d, cmd = _cmd(_orch(), OrchestratorSession(), text)
    assert d.intent_id == intent and cmd is not None  # dispatched to the right intent
    assert cmd.get(field) == value  # with the right value


@pytest.mark.parametrize("text, n", [("音量调到七十", 70), ("声音调成一百", 100), ("音量设为5", 5),
                                     ("音量调到150", 100), ("音量调到零", 0)])
def test_volume_numbers(text, n):  # one query answer: test volume numbers
    """MUTATION: drop the numeric rule in parse_volume -> None -> fails."""
    assert parse_volume(text) == {"abs": n}  # the property this case pins


def test_elliptical_phrase_follows_the_last_adjustment():  # one query answer: test elliptical phrase follows the last adjustment
    """"中等" after a brightness change is brightness; after a volume change,
    volume. MUTATION: drop the session-context rule -> overheard -> fails."""
    o, s = _orch(), OrchestratorSession()
    _cmd(o, s, "亮一点")  # brightness first
    d, cmd = _cmd(o, s, "中等")
    assert d.intent_id == "D17" and cmd["level"] == "mid"  # the property this case pins
    _cmd(o, s, "大点声")  # then volume
    d, cmd = _cmd(o, s, "开到最大")
    assert d.intent_id == "D10" and cmd["volume"] == {"abs": 100}  # the property this case pins


def test_elliptical_phrase_without_context_asks():  # one query answer: test elliptical phrase without context asks
    """No earlier adjustment: ask which one, never "没太听清", nothing sent."""
    d, cmd = _cmd(_orch(), OrchestratorSession(), "调到最低")
    assert cmd is None and d.tts_text == CC["session"]["which_device"]  # the property this case pins
