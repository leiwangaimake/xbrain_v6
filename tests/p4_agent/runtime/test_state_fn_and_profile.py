"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_state_fn_and_profile.py
Brief: The orchestrator reads P3's live state through make_state_fn (task control / recording work while a task or session runs); A13 speed profile says it is not wired instead of "没听清"
Description:
2026-11-02 (99 U105), two defects found while adding A15:
  * production never passed a state_fn, so "暂缓任务 / 继续任务" answered "现在没
    有正在执行的任务" and "结束录制" "没有正在进行的录制" even while P3 was
    running one -- the snapshot was always empty;
  * A13 "慢一点 / 快一点 / 全速" answered "这个模式指令没听清" (the profile slot
    was never parsed), and even a parsed profile changes nothing: P2 only
    records it and P1 runs a fixed PROFILE_REQ. It now says it is not wired.
These cases run the production orchestrator on the production configs with
a StateCache holding the bodies P3 publishes (bare, not enveloped).
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import pytest  # parametrize, marks
import yaml  # production configs

from xbrain.p4_agent.registry.intents import load_intent_registry  # 18 registry
from xbrain.p4_agent.runtime.orchestrator_turn import (  # the turn path
    build_orchestrator, decision_to_publishes, make_state_fn)
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # dialog state
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets
from xbrain.p4_agent.state.cache import STATE_TOPICS, StateCache  # live cache

pytestmark = pytest.mark.no_device  # pure in-process

with open("/opt/xbrain_v6/configs/chitchat.yaml", encoding="utf-8") as _fh:  # production presets
    CC = yaml.safe_load(_fh)  # chitchat
UW = CC["unwired_system"]  # template + names

TASK = {"schema": "task_state_v1", "current": {"task_id": "t-1", "type": "patrol", "state": "running"},  # P3 body
        "queue": [], "suspended": []}  # nothing queued
TEACH = {"schema": "teach_state_v1", "session": {"session_id": "s-9", "state": "recording", "kind": "route"}}  # recording


def _orch(state_fn):  # production orchestrator with the given snapshot getter
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:  # production registry
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    return build_orchestrator(reg, ChitchatResponder(CC), state_fn=state_fn)  # wired


def _cache(**bodies):  # a StateCache holding P3's bare bodies
    c = StateCache()  # empty
    for key, body in bodies.items():  # e.g. task=TASK
        c.update("state/" + key, body, 1000)  # as the subscriber stores it
    return c  # filled


def _turn(o, text):  # one cloud text turn -> (decision, {short key: payload}, spoken)
    d = o.handle_turn(text, OrchestratorSession(), 1000, channel="cloud", modality="text")  # the turn
    pubs = decision_to_publishes(d)  # what would go out
    keys = {k.split("/", 2)[-1] if k.startswith("xbrain/") else k: p for k, p in pubs}  # short keys
    return d, keys, [p["text"] for k, p in pubs if k.endswith("audio/speak")]  # spoken last


def test_snapshot_covers_every_state_topic():
    """Every subscribed key is in the snapshot; unseen ones are None."""
    snap = make_state_fn(_cache(task=TASK))()  # one read
    assert set(snap) == set(STATE_TOPICS)  # the whole subscribed set
    assert snap["state/task"] == TASK and snap["state/teach"] is None  # value / never received


def test_snapshot_reads_the_latest_value_not_a_window():
    """MUTATION: get_fresh with any window -> an on-change topic quiet for a
    while reads as absent -> "没有正在执行的任务" again -> fails."""
    c = _cache(task=TASK)  # stored at t = 1000 ms
    assert make_state_fn(c)()["state/task"] == TASK  # whatever its age


#: The same task after the operator paused it (99 U111: resume aims here).
PAUSED = {**TASK, "current": None,
          "suspended": [{"task_id": "t-1", "suspend_kind": "passive",
                         "suspended_ts": 5.0}]}  # paused by the operator


@pytest.mark.parametrize("text, key, task", [("暂缓任务", "cmd/task", TASK),
                                             ("继续任务", "cmd/task", PAUSED)])
def test_task_control_reaches_the_running_task(text, key, task):
    """MUTATION: drop state_fn from build_orchestrator -> empty snapshot ->
    "现在没有正在执行的任务" -> fails. 99 U111: pause aims at the running task,
    continue at the paused one (it never was the running one)."""
    d, pubs, _spoken = _turn(_orch(make_state_fn(_cache(task=task))), text)  # the task
    assert d.kind == "dispatch" and pubs[key]["task_id"] == "t-1"  # aimed at it


def test_without_a_running_task_it_says_so():
    """The honest reply survives: no current task -> no command."""
    d, pubs, spoken = _turn(_orch(make_state_fn(_cache(task={**TASK, "current": None}))), "暂缓任务")
    assert "cmd/task" not in pubs and spoken == ["现在没有正在执行的任务"]  # nothing to control


def test_recording_stop_names_the_live_session():
    """F02 "结束录制" with a recording in progress names its session."""
    d, pubs, _spoken = _turn(_orch(make_state_fn(_cache(teach=TEACH))), "结束录制")  # recording
    assert d.kind == "dispatch" and pubs["cmd/teach"]["session_id"] == "s-9"  # the live one


@pytest.mark.parametrize("text", ["慢一点", "快一点", "全速", "转身快一点", "转身慢一点"])
def test_speed_profile_says_not_wired(text):
    """MUTATION: drop A13 from _UNWIRED_ACTIONS -> "这个模式指令没听清" again ->
    fails. "转身快一点" lands here too (longest keyword "快一点"), never on A11."""
    d, pubs, spoken = _turn(_orch(None), text)  # no state needed
    assert d.intent_id == "A13" and set(pubs) == {"cmd/audio/speak"}  # nothing sent, nothing moves
    assert spoken == [UW["template"].replace("{what}", UW["names"]["A13"])]  # the honest reason
