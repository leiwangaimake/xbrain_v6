"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_control_turn.py
Brief: pause / continue / cancel act on the right task; 原地待命 holds; 跳过 is answered honestly

Description:
99 U111, from the 2026-11-04 live test. "继续" resolved the RUNNING task (the
one the scheduler started after the pause) and P3 refused it; after an
estop it found nothing. A04 sent a frame P2 refused; B10 sent a frame nobody
read; both said they had done it. These turns run through the orchestrator
with this tree's registry and a scripted state snapshot. Each case names the
mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import json  # frames
import os  # this tree's configs

import pytest  # parametrize, marker
import yaml  # content files

from xbrain.p4_agent.registry.intents import load_intent_registry  # registry loader
from xbrain.p4_agent.runtime.orchestrator_turn import (  # p4 builders
    build_orchestrator,  # as p4's wiring builds it
    decision_to_publishes,  # the frames a turn sends
)
from xbrain.p4_agent.runtime.task_control_request import (  # resolution
    TaskControlError,  # nothing to act on
    to_task_control_command,  # target resolution
)
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # a fresh dialog
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets

pytestmark = pytest.mark.no_device  # no robot, no LLM

_CONFIGS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "configs")  # this tree
_SUSPENDED = [  # what state/task carried after the live test's pauses
    {"task_id": "t-old", "suspend_kind": "passive", "suspended_ts": 100.0},  # older passive
    {"task_id": "t-yield", "suspend_kind": "yielding", "suspended_ts": 300.0},  # newest, but yielding
    {"task_id": "t-paused", "suspend_kind": "passive", "suspended_ts": 200.0},  # newest passive
]  # newest passive = t-paused


def _task_state(current=None, suspended=()):  # snapshot builder
    """One 11 S4.4 TaskState snapshot."""
    return {"state/task": {"current": current, "queue": [],
                           "suspended": list(suspended)}}  # the key P4 reads


def test_resume_targets_the_latest_paused_task():  # resume target
    """MUTATION: resolve from current (the pre-U111 code) -> t-next, refused."""
    st = _task_state({"task_id": "t-next", "state": "running"}, _SUSPENDED)
    cmd = to_task_control_command("resume_task", state=st, cmd_id="c",
                                  source="voice")  # 继续
    assert cmd["task_id"] == "t-paused"  # newest passive, not yielding


def test_cancel_after_a_pause_targets_the_paused_task():  # cancel target
    """Nothing running: "取消任务" means the one just paused."""
    st = _task_state(None, _SUSPENDED)  # all paused / held
    cmd = to_task_control_command("cancel_task", state=st, cmd_id="c",
                                  source="voice")  # 取消任务
    assert cmd["task_id"] == "t-paused"  # newest passive
    running = _task_state({"task_id": "t-run"}, _SUSPENDED)  # one running
    assert to_task_control_command("cancel_task", state=running, cmd_id="c",
                                   source="voice")["task_id"] == "t-run"


def test_resume_with_nothing_paused_says_so():  # nothing paused
    with pytest.raises(TaskControlError, match="no_suspended_task"):
        to_task_control_command("resume_task", state=_task_state(), cmd_id="c",
                                source="voice")  # nothing to continue


def _turn(text, state, session=None):  # turn helper
    """One mic turn -> (decision, {key: frame})."""
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:
        cc = ChitchatResponder(yaml.safe_load(fh))  # presets
    orch = build_orchestrator(reg, cc,
                              state_fn=lambda: state)  # scripted snapshot
    d = orch.handle_turn(text, session or OrchestratorSession(), 1000,
                         channel="mic_local")  # one turn
    return d, {k: p for k, p in decision_to_publishes(d) if k != "cmd/audio/speak"}


@pytest.mark.parametrize("text", ["原地待命", "别动", "待命"])
def test_hold_is_a_motion_intent(text):  # case body
    """MUTATION: the legacy p4_intent_v1 frame P2 refused with E_SCHEMA."""
    d, frames = _turn(text, {})  # one turn
    data = frames["cmd/motion/intent"]["data"]  # the S3.0 envelope's body
    assert data["intent"] == "hold" and data["auth_level"] == "L0"  # A04


def test_continue_while_held_releases_the_hold():  # release
    """MUTATION: send a task resume -- P1 stays held and nothing moves."""
    st = dict(_task_state(None, _SUSPENDED),
              **{"state/mode": {"motion_behavior": "hold"}})  # P1 held
    d, frames = _turn("继续", st)  # 继续
    assert set(frames) == {"cmd/motion/intent"}  # a motion frame, no task frame
    assert frames["cmd/motion/intent"]["data"]["intent"] == "resume"


def test_continue_when_not_held_resumes_the_paused_task():  # task resume
    d, frames = _turn("继续", _task_state(None, _SUSPENDED))  # not held
    assert frames["cmd/task"]["action"] == "resume"  # S7.2
    assert frames["cmd/task"]["task_id"] == "t-paused"  # the right one


@pytest.mark.parametrize("text", ["跳过这个点", "下一个点"])
def test_skip_is_no_longer_an_intent(text):  # case body
    """99 U129: B10 skip_waypoint was deleted ("不支持, 不需要实现, 删除").
    The phrases classify as nothing and send nothing. MUTATION: restore the
    intents.yaml row -> the turn classifies as skip_waypoint -> red."""
    d, frames = _turn(text, {})  # one turn
    assert d.intent_name != "skip_waypoint" and d.intent_id != "B10"  # gone from the registry
    assert "cmd/task" not in (frames or {})  # nothing sent


def test_cancel_task_runs_on_its_own_turn():  # one question, one answer
    """99 U147: "取消任务" cancels the running task on THIS turn and says so;
    no "确认取消任务吗" round. MUTATION: keep the L2 confirm -> await_confirm
    -> red."""
    st = _task_state({"task_id": "t-run", "state": "running"})  # one running
    d = _turn("取消任务", st, OrchestratorSession())[0]  # one turn
    assert d.kind == "dispatch" and d.tts_text == "好的,取消当前任务"  # done, said (99 U198: which task)
    assert d.dispatch_result.payload["task_command"]["task_id"] == "t-run"  # by id
