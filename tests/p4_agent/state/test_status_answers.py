"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_status_answers.py
Brief: G-class answers from live state (health / faults / task / mode / geo / uptime) and the never-echo rule

Description:
Field test 2026-10-29 (R001-R006, P131, P235): status questions answered only
"收到", after the robot had read the user's own words back. Cause: a G id
without a query_fn was dispatched to cmd/audio/speak, i.e. the utterance was
spoken. These cases pin the fix with payloads shaped exactly like the ones
captured on the running robot (2026-10-30):
  * health items are judged by `state`, never by `level` (the criticality
    class) -- a healthy fatal-class chassis is NOT a fault;
  * a stale source gives the honest "读不到" reply, not a last-known value;
  * a G id with no data path answers its NO_DATA text, any other G id the
    generic one -- the utterance is never spoken back;
  * "报警了几次" / "系统时间" now reach G11 / G24.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import pytest  # parametrize, marks
import yaml  # production configs

from xbrain.p4_agent.registry.intents import load_intent_registry  # 18 registry
from xbrain.p4_agent.runtime.orchestrator_turn import (  # the production turn path
    build_orchestrator, decision_to_publishes, make_status_query_fn)
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # fresh dialog
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # chitchat presets
from xbrain.p4_agent.state import status_answers as sa  # under test
from xbrain.p4_agent.state.cache import STATE_TOPICS, StateCache  # the live cache

pytestmark = pytest.mark.no_device  # pure in-process

NOW = 100_000  # fake monotonic ms
with open("/opt/xbrain_v6/configs/query_templates.yaml", encoding="utf-8") as _fh:  # production content file
    TPL = yaml.safe_load(_fh)  # production reply templates (16 S8.2)
AGE = 3_000  # max_age_ms

# Shapes captured from the robot (health/summary is enveloped, state/task flat).
HEALTH = {"v": 1, "src": "p2_core", "data": {
    "schema": "health_summary_v1", "overall": "fatal", "speed_factor": 0.3, "allow_motion": True,  # id -> spoken label
    "items": {"chassis": {"state": "ok", "level": "fatal"},
              "heading": {"state": "fail", "level": "fatal"},
              "rtk": {"state": "degraded", "level": "fatal"},
              "lidar": {"state": "unknown", "level": "degraded"},
              "mic": {"state": "ok", "level": "degraded"}}}}
TASK = {"schema": "task_state_v1",
        "current": {"type": "return_home", "state": "running", "progress": None},
        "queue": [{"type": "return_home", "state": "ready"}, {"type": "charge", "state": "ready"}]}


def _with(pairs):  # cache from (key, value) pairs
    c = StateCache()  # local read of the state field
    for k, v in pairs:  # walk every entry
        c.update(k, v, NOW)  # received at NOW
    return c  # result to the caller


def test_faults_are_judged_by_state_not_level():  # one query answer: test faults are judged by state not level
    """MUTATION: read item['level'] as the status -> the healthy fatal-class
    chassis is reported as a fault -> this fails."""
    a = sa.faults_answer(_with([("health/summary", HEALTH)]), TPL, NOW, max_age_ms=AGE)  # local read of the state field
    assert a.known and "航向" in a.text and "RTK 定位" in a.text  # the real fail + degraded
    assert "底盘" not in a.text and "麦克风" not in a.text  # ok items are not faults
    assert "另有1项状态未知" in a.text  # unknown counted, not hidden


def test_health_names_motion_speed_and_problems():  # one query answer: test health names motion speed and problems
    """G16: what the operator needs -- may it move, how fast, what is wrong."""
    a = sa.health_answer(_with([("health/summary", HEALTH)]), TPL, NOW, max_age_ms=AGE)  # local read of the state field
    assert a.text.startswith("系统允许运动,限速30%")  # from allow_motion / speed_factor
    assert "航向故障" in a.text and "RTK 定位降级" in a.text  # named problems


def test_a_stale_source_is_unreadable_not_last_known():  # one query answer: test a stale source is unreadable not last known
    """MUTATION: read the cache without the freshness bound -> the old health
    is spoken as current -> this fails."""
    c = StateCache()  # local read of the state field
    c.update("health/summary", HEALTH, NOW - 10_000)  # 10 s old, bound 3 s
    a = sa.faults_answer(c, TPL, NOW, max_age_ms=AGE)  # local read of the state field
    assert not a.known and "读不到" in a.text  # honest


def test_task_answer_reads_current_and_queue():  # one query answer: test task answer reads current and queue
    """G06 from the flat state/task (last value, it is sent on change)."""
    c = StateCache()  # local read of the state field
    c.update("state/task", TASK, NOW - 600_000)  # 10 min old is still the current task
    a = sa.task_answer(c, TPL, NOW, max_age_ms=AGE)  # local read of the state field
    assert a.text == "当前任务是返航,执行中,另有2个任务排队"  # current + queue


def test_mode_and_uptime_and_geo():  # one query answer: test mode and uptime and geo
    """G05 / G09 / G21 from live sources."""
    c = _with([("state/mode", {"voice_mode": "alarm", "motion_behavior": "hold"}),  # local read of the state field
               ("state/geo/objects", {"routes": [], "waypoints": [{"name": "东门"}], "docks": []})])
    assert sa.mode_answer(c, TPL, NOW, max_age_ms=AGE).text == "当前是报警驱离模式,运动已暂停"  # the property this case pins
    assert sa.uptime_answer(c, TPL, NOW, max_age_ms=AGE).text.startswith("已经开机")  # the property this case pins
    assert sa.STATUS_QUERY_ANSWERS["G21"](c, TPL, NOW, max_age_ms=AGE).text == "现在还没有设置巡逻路径"  # the property this case pins
    assert sa.STATUS_QUERY_ANSWERS["G22"](c, TPL, NOW, max_age_ms=AGE).text == "共有1个位置点:东门"  # the property this case pins


def test_status_topics_are_subscribed():  # one query answer: test status topics are subscribed
    """MUTATION: drop health/summary from STATE_TOPICS -> G15/G16 always
    unreadable on the robot -> this fails."""
    for key in ("health/summary", "state/task", "state/geo/objects", "state/arb/ptz"):  # walk every entry
        assert key in STATE_TOPICS  # p4 subscribes it


def _orch(cache):  # production orchestrator with the status query_fn
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:  # production content file
        reg = load_intent_registry(yaml.safe_load(fh))  # value used below
    with open("/opt/xbrain_v6/configs/chitchat.yaml", encoding="utf-8") as fh:  # production content file
        cc = ChitchatResponder(yaml.safe_load(fh))  # value used below
    return build_orchestrator(reg, cc, l2_timeout_ms=8000,  # result to the caller
                              query_fn=make_status_query_fn(cache, TPL, max_age_ms=10 ** 12))


@pytest.mark.parametrize("text", ["有故障吗", "系统状态怎么样", "有几个人", "载荷都开着吗",
                                  "照明什么状态", "音量多大", "报警了几次", "当前任务"])
def test_a_g_query_never_speaks_the_utterance(text):  # one query answer: test a g query never speaks the utterance
    """The on-robot defect itself. MUTATION: dispatch a G id whose query_fn
    returns None (the old fall-through) -> the utterance is spoken -> fails."""
    c = _with([("health/summary", HEALTH), ("state/task", TASK)])  # local read of the state field
    d = _orch(c).handle_turn(text, OrchestratorSession(), 1000, channel="cloud", modality="text")  # local read of the state field
    spoken = [p["text"] for k, p in decision_to_publishes(d) if k.endswith("audio/speak")]  # local read of the state field
    assert spoken and all(s != text for s in spoken)  # never the user's words
    assert spoken != ["收到"]  # and never a bare ack
    assert d.intent_id and d.intent_id.startswith("G")  # it was a query


def test_unwired_g_id_gets_the_generic_honest_reply(monkeypatch):  # one query answer: test unwired g id gets the generic honest reply
    """A G id nobody owns. MUTATION: make_status_query_fn returns None for it
    (the old fall-through) -> the G id dispatches and speaks the utterance ->
    this fails."""
    tpl = dict(TPL)  # copy, then drop the specific no-data texts
    # A marker text, distinct from turn_orchestrator's last-resort constant, so
    # the yaml path and the constant path cannot pass for each other.
    tpl["query_no_data"] = {"generic": "这项查询暂时答不了(模板)"}
    reg_path = "/opt/xbrain_v6/configs/intents.yaml"  # value used below
    with open(reg_path, encoding="utf-8") as fh:  # production content file
        reg = load_intent_registry(yaml.safe_load(fh))  # value used below
    with open("/opt/xbrain_v6/configs/chitchat.yaml", encoding="utf-8") as fh:  # production content file
        cc = ChitchatResponder(yaml.safe_load(fh))  # value used below
    o = build_orchestrator(reg, cc, l2_timeout_ms=8000,  # value used below
                           query_fn=make_status_query_fn(_with([]), tpl, max_age_ms=10 ** 12))
    d = o.handle_turn("音量多大", OrchestratorSession(), 1000, channel="cloud", modality="text")  # local read of the state field
    spoken = [p["text"] for k, p in decision_to_publishes(d) if k.endswith("audio/speak")]  # local read of the state field
    assert spoken == ["这项查询暂时答不了(模板)"]  # from the templates, once


def test_ptz_pose_is_always_unknown():  # one query answer: test ptz pose is always unknown
    """QT-5: no position readback -> the designed unknown answer, never angles."""
    a = sa.ptz_pose_answer(_with([]), TPL, NOW, max_age_ms=AGE)  # local read of the state field
    assert a.text == TPL["query_ptz_pose"]["unknown"]  # the property this case pins


@pytest.mark.parametrize("text, gid", [("报警了几次", "G11"), ("系统时间", "G24")])
def test_new_query_phrases_route(text, gid):  # one query answer: test new query phrases route
    """Field phrases that used to fall to overheard."""
    d = _orch(_with([])).handle_turn(text, OrchestratorSession(), 1000, channel="cloud", modality="text")  # local read of the state field
    assert d.intent_id == gid  # the property this case pins
