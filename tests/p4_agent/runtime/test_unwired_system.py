"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_unwired_system.py
Brief: H-class system intents with no executor say so and send nothing -- no "收到", no confirm prompt for a no-op

Description:
Field test 2026-10-29 (R007-R016): "校时" answered "收到"; "重新加载配置" and
the six reboot phrases asked "确认吗", and after the operator confirmed nothing
happened. Nothing in the repo consumes cmd/system (intent_dispatch: "ROUTED but
not yet CONSUMED"), and H04's cmd/task fallback is dropped by P3. Until an
executor lands these must not pretend (CLAUDE.md 3.2). Production orchestrator,
production configs:
  * each H phrase is denied with its chitchat.yaml unwired_system reply;
  * nothing is published (no cmd/system, no cmd/task) and no confirm opens.
MUTATION: remove the _UNWIRED_SYSTEM branch -> "重启" opens a confirm ->
these cases fail.
"""

from __future__ import annotations  # postponed annotations

import pytest  # parametrize, marks
import yaml  # production configs

from xbrain.p4_agent.registry.intents import load_intent_registry  # 18 registry
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # turn path
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # dialog state
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets

pytestmark = pytest.mark.no_device  # pure in-process

with open("/opt/xbrain_v6/configs/chitchat.yaml", encoding="utf-8") as _fh:  # production content file
    CC = yaml.safe_load(_fh)  # production presets
_UW = CC["unwired_system"]  # template + names


def _orch():  # production orchestrator
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:  # production content file
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    return build_orchestrator(reg, ChitchatResponder(CC), l2_timeout_ms=8000)  # real presets


@pytest.mark.parametrize("text, hid", [("校时", "H03"), ("重新加载配置", "H04"), ("刷新规则", "H04"),
                                       ("重启系统", "H07"), ("重启一下", "H07"), ("重启设备", "H07"),
                                       ("做个自检", "H01"), ("关机", "H08")])
def test_unwired_system_intent_says_so_and_sends_nothing(text, hid):  # one query answer: test unwired system intent says so and sends nothing
    """No "收到", no confirm, nothing on the wire."""
    s = OrchestratorSession()  # fresh dialog
    d = _orch().handle_turn(text, s, 1000, channel="cloud", modality="text")  # local read of the state field
    pubs = decision_to_publishes(d)  # what would go out
    spoken = [p["text"] for k, p in pubs if k.endswith("audio/speak")]  # local read of the state field
    assert d.intent_id == hid and d.kind == "denied"  # refused, honestly
    assert spoken == [_UW["template"].replace("{what}", _UW["names"][hid])]  # names the function
    assert [k for k, _p in pubs if not k.endswith("audio/speak")] == []  # nothing sent
    assert s.pending_confirm is None  # no confirm for a no-op
