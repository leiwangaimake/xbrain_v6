"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_event_answers.py
Brief: G10 / G11 / G13 answer from p5's state/event/stats -- counts per severity, the latest alarm with its time and category -- and say "暂时读不到" when it is stale or malformed
Description:
Field test 2026-10-29 / 10-30: "报警了几次", "最近都发生了什么", "上次警报"
answered "还没有接到事件库". User ruling (99 U103): p5 publishes the
record.db summary (11 S4.7), p4 answers from it. Production templates, and
the production orchestrator on the production configs:
  * G10 quiet day / notable day with the latest-alarm clause;
  * G11 speaks both windows (today, last 7 days);
  * G13 today vs an earlier day vs no alarm at all;
  * stale, missing, or a partial / boolean count -> unreadable, never zero;
  * an unknown category is spoken as "其他";
  * "报警了几次" through the turn path answers and dispatches nothing.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

from datetime import datetime, timezone  # event stamps

import pytest  # parametrize, marks
import yaml  # production configs

from xbrain.p4_agent.registry.intents import load_intent_registry  # 18 registry
from xbrain.p4_agent.runtime.orchestrator_turn import (  # the production turn path
    build_orchestrator, decision_to_publishes, make_status_query_fn)
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # fresh dialog
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # chitchat presets
from xbrain.p4_agent.state.cache import STATE_TOPICS, StateCache  # the live cache
from xbrain.p4_agent.state.status_answers import status_answer  # under test

pytestmark = pytest.mark.no_device  # pure in-process

with open("/opt/xbrain_v6/configs/query_templates.yaml", encoding="utf-8") as _fh:  # production templates
    TPL = yaml.safe_load(_fh)  # query templates
with open("/opt/xbrain_v6/configs/chitchat.yaml", encoding="utf-8") as _fh:  # production presets
    CC = yaml.safe_load(_fh)  # chitchat presets
EV = TPL["query_events"]  # the event block

TODAY_1632 = datetime(2026, 10, 30, 8, 32, tzinfo=timezone.utc).timestamp()  # 16:32 CST
OCT28_2016 = datetime(2026, 10, 28, 12, 16, tzinfo=timezone.utc).timestamp()  # 10-28 20:16 CST


def _counts(info=0, warn=0, alarm=0, fault=0):  # one count block
    return {"info": info, "warn": warn, "alarm": alarm, "fault": fault}  # per sev


def _stats(today=None, week=None, last_alarm=None, **extra):  # a p5 data object
    d = {"tz": "Asia/Shanghai", "day": "2026-10-30", "total": 0,  # header
         "today": today or _counts(), "last_7d": week or _counts(),  # windows
         "last_alarm": last_alarm, "last_fault": None}  # newest rows
    d.update(extra)  # overrides
    return d  # the data object


def _ask(gid, data, at=1000, now=1500):  # one answer from a cache holding `data`
    c = StateCache()  # empty
    if data is not None:  # the p5 publication, enveloped
        c.update("state/event/stats", {"v": 1, "src": "p5_gateway", "data": data}, at)  # 11 S3.0
    return status_answer(gid, c, TPL, now, max_age_ms=2000).text  # the reply


def test_quiet_day():
    """An info-only day answers the quiet branch with the total."""
    assert _ask("G10", _stats(today=_counts(info=12))) == EV["quiet"].format(total=12)


def test_notable_day_names_counts_and_the_latest_alarm():
    """MUTATION: drop the {last} clause -> no time / category -> fails."""
    got = _ask("G10", _stats(today=_counts(info=43, warn=41, alarm=1),  # the live 10-30 day
                             last_alarm={"cat": "comm", "at": TODAY_1632}))
    assert got == "今天记录了85条事件:报警1次,故障0次,警告41次;最近一次报警在今天16时32分,类别是通信"


def test_warnings_alone_are_not_quiet():
    """MUTATION: quiet test on alarm + fault only -> 41 warnings answer "没有警告" -> fails."""
    got = _ask("G10", _stats(today=_counts(info=1, warn=41)))
    assert got.startswith("今天记录了42条事件") and got.endswith("警告41次")  # no alarm clause


def test_period_speaks_both_windows():
    """MUTATION: read last_7d for both -> today says 6 -> fails."""
    got = _ask("G11", _stats(today=_counts(alarm=1), week=_counts(alarm=6, fault=8)))
    assert got == "今天报警1次,故障0次;近7天报警6次,故障8次"


def test_last_alarm_today_and_earlier_day_and_none():
    """MUTATION: always when_today -> an alarm two days ago says "今天" -> fails."""
    assert _ask("G13", _stats(last_alarm={"cat": "comm", "at": TODAY_1632})) == "最近一次报警在今天16时32分,类别是通信"
    assert _ask("G13", _stats(last_alarm={"cat": "intrusion", "at": OCT28_2016})) == \
        "最近一次报警在10月28日20时16分,类别是入侵"
    assert _ask("G13", _stats()) == EV["no_alarm"]  # the table has none


def test_unknown_category_is_other():
    assert _ask("G13", _stats(last_alarm={"cat": "brand_new", "at": TODAY_1632})).endswith("类别是其他")


@pytest.mark.parametrize("gid, what", [("G10", "事件记录"), ("G11", "报警次数"), ("G13", "报警记录")])
def test_stale_or_missing_is_unreadable(gid, what):
    """MUTATION: read cache.latest instead of get_fresh -> a 10 s old count is spoken -> fails."""
    want = TPL["query_unreadable"]["stale"].format(what=what)  # the honest reply
    assert _ask(gid, _stats(today=_counts(alarm=3)), at=1000, now=11000) == want  # 10 s old
    assert _ask(gid, None) == want  # never received


@pytest.mark.parametrize("bad", [{"info": 0, "warn": 0, "alarm": 1},  # a sev missing
                                 {"info": 0, "warn": 0, "alarm": True, "fault": 0},  # bool is not a count
                                 None])  # no block
def test_partial_counts_are_not_a_reading(bad):
    """MUTATION: drop the per-sev type check -> True speaks as 1 / a missing sev raises -> fails."""
    want = TPL["query_unreadable"]["stale"].format(what="报警次数")
    d = _stats()  # a whole object ...
    d["today"] = bad  # ... with one broken block (None must stay None, not the default)
    assert _ask("G11", d) == want  # never a made-up zero


def test_state_event_stats_is_subscribed():
    """Without the subscription every answer above is "暂时读不到" in production."""
    assert "state/event/stats" in STATE_TOPICS


@pytest.mark.parametrize("text, gid", [("报警了几次", "G11"), ("最近都发生了什么", "G10"), ("上次警报", "G13")])
def test_field_phrases_answer_and_dispatch_nothing(text, gid):
    """The 10-30 field phrases, through the production turn path."""
    c = StateCache()  # live cache
    c.update("state/event/stats", {"data": _stats(today=_counts(alarm=1), week=_counts(alarm=6),
                                                  last_alarm={"cat": "comm", "at": TODAY_1632})}, 1000)  # fresh
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:  # production registry
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    o = build_orchestrator(reg, ChitchatResponder(CC),  # real presets
                           query_fn=make_status_query_fn(c, TPL, max_age_ms=10 ** 12))  # never stale here
    d = o.handle_turn(text, OrchestratorSession(), 1000, channel="cloud", modality="text")  # the turn
    pubs = decision_to_publishes(d)  # what would go out
    spoken = [p["text"] for k, p in pubs if k.endswith("audio/speak")]  # TTS
    assert d.intent_id == gid and [k for k, _p in pubs if not k.endswith("audio/speak")] == []  # nothing sent
    assert len(spoken) == 1 and "报警" in spoken[0] and "暂时" not in spoken[0]  # a real answer
