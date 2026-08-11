"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_orchestrator_wiring.py
Brief: GWY-P4-41 (32.I) -- voice loop routes turns through the orchestrator

Description:
Verifies main_wiring uses the six-step TurnOrchestrator (not the V-2B
naive_classify path) and that the wired turn handler maps each path to the
right cmd/* publish. The ORIN real-machine smoke (speak each of estop /
query / action / chitchat and read the log) is manual and logged, not
asserted here (criterion 2). Mutation A guard: main_wiring reverting to
naive_classify is caught by the grep test.
"""
from __future__ import annotations

import json

import pytest
import yaml

from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime import main_wiring
from xbrain.p4_agent.runtime.orchestrator_turn import (
    build_orchestrator, make_battery_query_fn, make_turn_handler,
)
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession
from xbrain.p4_agent.session.chitchat import ChitchatResponder
from xbrain.p4_agent.state.cache import StateCache

pytestmark = pytest.mark.no_device

_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"
_QUERY_TPL = "/opt/xbrain_v6/configs/query_templates.yaml"


def _reg():
    return load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))


def _chitchat():
    return ChitchatResponder(yaml.safe_load(open(_CHITCHAT, encoding="utf-8")))


def _templates():
    return yaml.safe_load(open(_QUERY_TPL, encoding="utf-8"))


def _texts(pairs):
    """Decode the (key, payload_bytes) pairs to (key, text/intent) tuples."""
    out = []
    for key, data in pairs:
        d = json.loads(data)
        out.append((key, d.get("text") or d.get("intent_id") or d.get("action")))
    return out


# -- criterion 1 (mutation A): main_wiring uses the orchestrator ----------

def test_main_wiring_uses_orchestrator_not_naive_classify():
    """MUTATION A guard: main_wiring must build the TurnOrchestrator and
    must NOT call naive_classify. A revert to the V-2B path would either
    drop the orchestrator import or call naive_classify here."""
    src = open(main_wiring.__file__, encoding="utf-8").read()
    assert "build_orchestrator" in src
    assert "make_turn_handler" in src
    # naive_classify must not be invoked from the wiring (it lives in
    # turn_loop as the fallback only).
    assert "naive_classify(" not in src


# -- each path maps to the right publish (fresh session per turn) ---------

def _handler(query_fn=None):
    orch = build_orchestrator(_reg(), _chitchat(), l2_timeout_ms=8000,
                              query_fn=query_fn)
    return orch, make_turn_handler(orch, OrchestratorSession())


def test_estop_path_publishes_cmd_estop():
    _, h = _handler()
    assert _texts(h("急停")) == [("cmd/estop", "estop")]


def test_action_path_publishes_cmd_motion():
    _, h = _handler()
    pairs = _texts(h("原地待命"))
    assert pairs[0][0] == "cmd/motion/intent"


def test_chitchat_path_speaks_preset():
    _, h = _handler()
    pairs = _texts(h("你好"))
    assert pairs[0][0] == "cmd/audio/speak"
    assert pairs[0][1] in _chitchat()._p["greeting"]["default"]


def test_overheard_path_is_silent():
    _, h = _handler()
    assert h("队友说的悄悄话") == []


def test_query_path_answers_from_live_state():
    """A G02 query answers from the live state cache via the wired
    query_fn (GWY-P4-39 + 32.I)."""
    cache = StateCache()
    cache.update("state/power", {"soc": 66, "range_km": 4}, now_mono_ms=0)
    # A large max_age so the reading is fresh regardless of the monotonic
    # 'now' inside the handler.
    qfn = make_battery_query_fn(cache, _templates(),
                                max_age_ms=10**12, low_soc_pct=20)
    _, h = _handler(query_fn=qfn)
    pairs = _texts(h("电量还有多少"))
    assert pairs[0][0] == "cmd/audio/speak"
    assert "66" in pairs[0][1]              # live soc, not a stub


def test_query_path_stale_state_answers_unknown():
    cache = StateCache()
    cache.update("state/power", {"soc": 66, "range_km": 4}, now_mono_ms=0)
    # max_age 1 ms: the monotonic 'now' in the handler is far past 0 -> stale.
    qfn = make_battery_query_fn(cache, _templates(),
                                max_age_ms=1, low_soc_pct=20)
    _, h = _handler(query_fn=qfn)
    pairs = _texts(h("电量还有多少"))
    assert pairs[0][0] == "cmd/audio/speak"
    assert "读不到" in pairs[0][1]           # stale -> unknown, not 66
    assert "66" not in pairs[0][1]
