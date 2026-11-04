"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_name_slots.py
Brief: the `name` slot -- one spelling for numbered names, grounding in what was said, 2-12 characters

Description:
99 U109. A name the operator makes up must be the name they said (16 EV-2:
never filled in by the model), stored in one spelling so "一号测试点" and
"1号测试点" are the same keypoint, and 2-12 characters (16 S8.0.4). Each
case names the mutant it kills.
"""
from __future__ import annotations

import pytest  # parametrize, marker

from xbrain.p4_agent.slots.name_slots import (
    canonical_name,  # the stored / looked-up spelling
    ground_name,  # said, or invented
    name_length_ok,  # 16 S8.0.4
)

pytestmark = pytest.mark.no_device  # pure functions


@pytest.mark.parametrize("raw,canon", [
    ("一号测试点", "1号测试点"),  # the ASR spelling
    ("1号测试点", "1号测试点"),  # the typed spelling, unchanged
    ("十二号哨位", "12号哨位"),  # positional numeral
    ("东门 岗亭\u3002", "东门岗亭"),  # spaces and an ideographic full stop dropped
    ("一楼大厅", "一楼大厅"),  # a word, not "N号": left alone
    ("第一哨位", "第一哨位"),  # same
])
def test_canonical_spelling(raw, canon):
    """MUTATION: rewrite every numeral and "一楼" becomes "1楼" -- a name
    the operator did not choose."""
    assert canonical_name(raw) == canon  # one spelling per name


def test_a_name_is_kept_only_when_it_was_said():
    """MUTATION: pass the model's name through and "标记一下这里" stores a
    keypoint called "这里" (measured on the 3B model)."""
    assert ground_name("1号测试点", "记录当前位置为一号测试点") == "1号测试点"  # either spelling
    assert ground_name("西门哨位", "把这个点保存为西门哨位") == "西门哨位"  # verbatim
    assert ground_name("东门岗亭", "记录当前位置") is None  # invented
    assert ground_name("", "记录当前位置为") is None  # nothing named
    assert ground_name(3, "记录当前位置为3") is None  # not a string


@pytest.mark.parametrize("answer", ["当前位置", "这里", "这个点"])
def test_the_place_itself_is_not_a_name(answer):
    """Measured on the 3B model: "记录当前位置" -> name "当前位置", which was
    said -- as the place. MUTATION: grounding alone and a keypoint called
    "当前位置" is stored."""
    assert ground_name(answer, "记录当前位置, 这里这个点") is None  # asked instead


def test_length_is_two_to_twelve_inclusive():
    """MUTATION: an off-by-one at either end."""
    assert not name_length_ok("东")  # 1
    assert name_length_ok("东门")  # 2
    assert name_length_ok("一" * 12)  # 12
    assert not name_length_ok("一" * 13)  # 13
