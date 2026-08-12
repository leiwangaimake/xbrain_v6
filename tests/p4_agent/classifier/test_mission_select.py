"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_mission_select.py
Brief: tier-2 mission selection cue table (16 S5.2 layer 4)

Description:
Tests select_mission: each family cue picks its mission, the overlap order is
respected (turn before translate, mark before naming), and non-command text
returns None (so tier-2 declines without an LLM call). Mutation guards per
CLAUDE.md 3.3. The returned keys are cross-checked against the mission registry
so a typo cannot name a non-existent mission.
"""
from __future__ import annotations

import pytest

from xbrain.p4_agent.classifier.mission_select import select_mission
from xbrain.p4_agent.registry.missions import MISSIONS


pytestmark = pytest.mark.no_device


@pytest.mark.parametrize("text,mission", [
    ("给大家播报一下注意安全", "M5_speak"),
    ("把这里记为东门岗亭", "M6b_mark"),
    ("这条路线命名为东门线", "M6_naming"),
    ("删掉三号路径", "M7_objref"),
    ("最近都发生了什么事件", "M8_events"),
    ("盯住那个人别跟丢了", "M4_follow"),
    ("去南边操场巡逻一圈", "M3_nav"),
    ("向左转个身", "M2_turn"),
    ("往前平移两米", "M1_translate"),
])
def test_cue_picks_mission(text, mission):
    assert select_mission(text) == mission


def test_returned_missions_are_real():
    """Every mission the selector can return must exist in the registry.
    MUTATION: a typo'd key ('M6_namng') would fail here, not at grammar time."""
    for text in ("播报", "记为X", "命名为X", "删除X", "最近事件", "跟随",
                 "去巡逻", "掉头", "前进"):
        m = select_mission(text)
        assert m in MISSIONS


def test_turn_beats_translate():
    """'左转' is a turn (M2), not a straight move (M1) -- order matters.
    MUTATION: listing M1 before M2 would mis-file every turn as translate."""
    assert select_mission("原地左转") == "M2_turn"


def test_mark_beats_naming():
    """'记为' (mark a point, M6b) is more specific than '命名' (naming, M6).
    MUTATION: M6 before M6b would file a point-mark as a route naming."""
    assert select_mission("把当前位置记为集合点") == "M6b_mark"


@pytest.mark.parametrize("text", [
    "今天天气真不错", "我有点累了", "谢谢你的帮助", "", "嗯嗯好的",
])
def test_non_command_returns_none(text):
    """No family cue -> None -> the caller declines without a GPU call.
    MUTATION: a catch-all default mission would burn a GPU slot on chit-chat."""
    assert select_mission(text) is None
