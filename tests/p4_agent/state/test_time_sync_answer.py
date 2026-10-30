"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_time_sync_answer.py
Brief: H03 "校时" answers the clock sync status, the chronyd fact and the time to the second; forced sync says not wired; nothing is sent
Description:
Field test 2026-10-30 (R009): "校时" answered "收到" and did nothing; the
interim fix said "校时功能还没有接通执行端". User ruling (99 U102): no sudo;
say "时钟已通过 NTP 同步" or "未同步", that chronyd keeps the clock, and
"当前时间XX时XX分XX秒"; "强制校时" stays honestly not wired.
These cases run the answer function on the production query_templates.yaml
and the production orchestrator on the production configs:
  * synced / unsynced / stale / unknown-source / chronyd-down branches;
  * the time is the given wall clock, to the second, in the site timezone;
  * "校时" publishes nothing and opens no confirm; "强制校时" says not wired;
  * state/clock is in the subscribed set (G47 read an unsubscribed key).
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import os  # fake /proc

import pytest  # parametrize, marks
import yaml  # production configs

from xbrain.p4_agent.registry.intents import load_intent_registry  # 18 registry
from xbrain.p4_agent.runtime.orchestrator_turn import (  # turn path
    build_orchestrator, decision_to_publishes, make_time_sync_fn)
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # dialog state
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets
from xbrain.p4_agent.state.cache import STATE_TOPICS, StateCache  # live state
from xbrain.p4_agent.state.time_sync_answer import (  # under test
    chronyd_running, spoken_clock, time_sync_answer)

pytestmark = pytest.mark.no_device  # pure in-process

with open("/opt/xbrain_v6/configs/query_templates.yaml", encoding="utf-8") as _fh:  # production templates
    QT = yaml.safe_load(_fh)  # query templates
with open("/opt/xbrain_v6/configs/chitchat.yaml", encoding="utf-8") as _fh:  # production presets
    CC = yaml.safe_load(_fh)  # chitchat presets
TS = QT["time_sync"]  # the H03 block

NOW = 1793360326.0  # 2026-10-30 11:38:46 UTC = 19:38:46 Asia/Shanghai
TZ = "Asia/Shanghai"  # the site timezone


def _cache(data=None, at=1000):  # a cache holding one state/clock reading
    c = StateCache()  # empty
    if data is not None:  # put the reading in
        c.update("state/clock", {"v": 1, "src": "p1_motion", "data": data}, at)  # enveloped like p1
    return c  # the cache


def _ans(cache, text="校时", daemon=True, now_ms=1500):  # one answer
    return time_sync_answer(cache, QT, text, now_ms, max_age_ms=5000, tz_name=TZ,
                            now_fn=lambda: NOW, daemon_fn=lambda: daemon).text  # the reply


def _fill(branch, **values):  # the template filled by hand
    return TS[branch].format(**values)  # expected text


NOW_SPOKEN = "19时38分46秒"  # NOW in the site timezone
ON = TS["daemon_on"]  # chronyd running clause


def test_synced_ntp_names_the_source_and_the_time():
    """MUTATION: drop the source lookup (always synced_plain) -> no "NTP" -> fails."""
    got = _ans(_cache({"sync": True, "source": "ntp"}))  # the live case on .1.26
    assert got == _fill("synced", source="NTP", daemon=ON, now=NOW_SPOKEN)  # full sentence
    assert "时钟已通过NTP同步" in got and NOW_SPOKEN in got  # the ruling's two parts


@pytest.mark.parametrize("data", [{"sync": False, "source": "none"}, {"sync": "true", "source": "ntp"},
                                  {"source": "ntp"}, {"sync": 1, "source": "rtk"}])
def test_anything_but_an_explicit_true_is_unsynced(data):
    """MUTATION: `if not d.get("sync")` -> the string "true" / 1 pass as synced -> fails."""
    assert _ans(_cache(data)) == _fill("unsynced", daemon=ON, now=NOW_SPOKEN)  # CLK-A3 fail-safe


def test_stale_clock_is_no_data_not_a_guess():
    """MUTATION: read cache.latest instead of get_fresh -> a 10 s old reading says synced -> fails."""
    got = _ans(_cache({"sync": True, "source": "ntp"}, at=1000), now_ms=11000)  # 10 s old
    assert got == _fill("no_data", daemon=ON, now=NOW_SPOKEN)  # honest
    assert _ans(_cache(None)) == got  # never received: the same reply


def test_unknown_source_says_synced_without_naming_it():
    """A source outside 11 S3.11 is not read aloud."""
    got = _ans(_cache({"sync": True, "source": "gps_pps"}))  # unknown spelling
    assert got == _fill("synced_plain", daemon=ON, now=NOW_SPOKEN) and "gps" not in got


def test_chronyd_down_is_said():
    """MUTATION: hard-code daemon_on -> chronyd down still claims auto sync -> fails."""
    got = _ans(_cache({"sync": True, "source": "ntp"}), daemon=False)  # chronyd not running
    assert TS["daemon_off"] in got and ON not in got


@pytest.mark.parametrize("text", ["强制校时", "强制同步时间", "强行校时"])
def test_forced_sync_says_not_wired(text):
    """MUTATION: drop the force check -> "强制校时" reads the status as if done -> fails."""
    assert _ans(_cache({"sync": True, "source": "ntp"}), text=text) == _fill("force_unwired", now=NOW_SPOKEN)


def test_spoken_clock_is_to_the_second_in_the_site_zone():
    """MUTATION: drop .astimezone(tz) -> UTC 11时 -> fails."""
    assert spoken_clock(NOW, TZ) == NOW_SPOKEN  # 19:38:46 CST
    assert spoken_clock(NOW + 3, "UTC") == "11时38分49秒"  # seconds move


def test_chronyd_running_reads_proc(tmp_path):
    """MUTATION: match 'chrony' as a substring -> 'chronyc' counts -> fails."""
    for pid, comm in (("1", "systemd"), ("77", "chronyc")):  # no daemon yet
        os.makedirs(tmp_path / pid)  # /proc/<pid>
        (tmp_path / pid / "comm").write_text(comm + "\n", encoding="utf-8")  # its name
    os.makedirs(tmp_path / "self")  # non-numeric entry
    assert chronyd_running(str(tmp_path)) is False  # chronyc is not chronyd
    os.makedirs(tmp_path / "812")  # the daemon appears
    (tmp_path / "812" / "comm").write_text("chronyd\n", encoding="utf-8")  # its name
    assert chronyd_running(str(tmp_path)) is True
    assert chronyd_running(str(tmp_path / "missing")) is False  # no /proc: not running


def test_state_clock_is_subscribed():
    """G47 / H03 read state/clock; before 2026-10-30 nobody subscribed it."""
    assert "state/clock" in STATE_TOPICS


def _orch(cache):  # production orchestrator with the production H03 wiring
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:  # production registry
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    return build_orchestrator(reg, ChitchatResponder(CC), l2_timeout_ms=8000,  # real presets
                              system_fn=make_time_sync_fn(cache, QT, TZ, max_age_ms=5000))  # H03


def _turn(o, text):  # one cloud text turn -> (decision, spoken, other keys, session)
    s = OrchestratorSession()  # fresh dialog
    d = o.handle_turn(text, s, 1000, channel="cloud", modality="text")  # the turn
    pubs = decision_to_publishes(d)  # what would go out
    spoken = [p["text"] for k, p in pubs if k.endswith("audio/speak")]  # TTS
    return d, spoken, [k for k, _p in pubs if not k.endswith("audio/speak")], s


@pytest.mark.parametrize("text", ["校时", "同步时间"])
def test_time_sync_turn_answers_and_sends_nothing(text):
    """MUTATION: remove the H03 branch -> L1b dispatches cmd/system (or the
    unwired reply) -> this fails."""
    c = StateCache()  # live state/clock, as fresh as the turn
    import time as _t  # make_time_sync_fn reads the real monotonic clock
    c.update("state/clock", {"data": {"sync": True, "source": "ntp"}}, int(_t.monotonic() * 1000))  # fresh
    d, spoken, keys, s = _turn(_orch(c), text)  # the turn
    assert d.intent_id == "H03" and keys == [] and s.pending_confirm is None  # nothing sent, no confirm
    assert len(spoken) == 1 and spoken[0].startswith("时钟已通过NTP同步,")  # the status first
    assert "当前时间" in spoken[0] and spoken[0].endswith("秒")  # and the time to the second


def test_forced_time_sync_turn_is_not_wired():
    """"强制校时" reaches H03 and answers force_unwired; nothing is sent."""
    d, spoken, keys, _s = _turn(_orch(StateCache()), "强制校时")
    assert d.intent_id == "H03" and keys == []
    assert spoken[0].startswith(TS["force_unwired"].split("{now}")[0])  # the not-wired reply


def test_without_system_fn_h03_says_unwired():
    """A build with no system_fn never pretends: the unwired_system reply."""
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:  # production registry
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    o = build_orchestrator(reg, ChitchatResponder(CC), l2_timeout_ms=8000)  # no system_fn
    d, spoken, keys, _s = _turn(o, "校时")
    uw = CC["unwired_system"]  # template + names
    assert spoken == [uw["template"].replace("{what}", uw["names"]["H03"])] and keys == []
