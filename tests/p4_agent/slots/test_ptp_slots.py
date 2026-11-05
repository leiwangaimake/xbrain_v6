"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ptp_slots.py
Brief: the rule parse of go to X / delete X / rename X to Y, with the prefixes operators use

Description:
99 U110. The three PTP sentences are parsed by rule because the LLM legs
invented catalogue ids. These cases pin what is taken (with courtesy,
urgency and time prefixes) and, as importantly, what is left to the other
intents. Each group names the mutant it kills.
"""
from __future__ import annotations

import pytest  # parametrize, marker

from xbrain.p4_agent.slots.ptp_slots import parse_ptp  # unit under test

pytestmark = pytest.mark.no_device  # pure


@pytest.mark.parametrize("text,name", [
    ("去1号测试点", "1号测试点"),  # the bare form
    ("前往1号测试点", "1号测试点"),  # 前往
    ("到1号测试点去", "1号测试点"),  # trailing 去
    ("导航到1号测试点", "1号测试点"),  # 导航到
    ("去航点1号测试点", "1号测试点"),  # type word dropped
    ("去1号测试点那边看看", "1号测试点"),  # trailing filler
    ("现在请立刻去1号测试点", "1号测试点"),  # stacked courtesy / urgency
    ("海卫, 马上前往东门岗亭!", "东门岗亭"),  # wake word + punctuation
    ("去快递站", "快递站"),  # a name starting with a lead word is kept
])
def test_goto_now(text, name):
    """MUTATION: no lead stripping and "现在请立刻去..." is not a goto."""
    got = parse_ptp(text)  # one parse
    assert got is not None and got.kind == "goto"  # a goto
    assert got.name == name and got.when is None  # the place, now


@pytest.mark.parametrize("text,delay,at,day", [
    ("5分钟后去1号测试点", 300, None, 0),  # relative minutes
    ("2小时后去1号测试点", 7200, None, 0),  # relative hours (was B03)
    ("半小时后去1号测试点", 1800, None, 0),  # 半小时
    ("下午3点去1号测试点", None, "15:00", 0),  # clock time
    ("明天14点去1号测试点", None, "14:00", 1),  # tomorrow
    ("明天下午3点半请去1号测试点", None, "15:30", 1),  # time then courtesy
    ("现在请在晚上8点前往1号测试点", None, None, None),  # 在 is not a prefix
])
def test_goto_with_a_time(text, delay, at, day):
    """MUTATION: drop split_time_prefix and every timed goto runs at once
    (or, for "2小时后", lands on B03)."""
    got = parse_ptp(text)  # one parse
    if delay is None and at is None:
        assert got is None or got.when is None  # unsupported shape: no guess
        return
    assert got is not None and got.kind == "goto" and got.name == "1号测试点"
    assert got.when.delay_seconds == delay and got.when.at_local == at  # the time
    assert got.when.day_offset == day  # the day


@pytest.mark.parametrize("text", [
    "去巡逻", "去东门巡逻", "去充电", "去掉围栏", "到底怎么回事", "去哪了",
    "回到起点", "灯开到最大", "前进三米", "到这里啊", "去那边",
])
def test_goto_leaves_other_intents_alone(text):
    """MUTATION: drop the exclusions and patrol / charge / delete-fence /
    questions are read as a goto to a place that does not exist."""
    got = parse_ptp(text)  # one parse
    assert got is None or got.kind != "goto"  # not a goto


@pytest.mark.parametrize("text,name", [
    ("删除1号测试点", "1号测试点"),  # verb first
    ("立刻删除1号测试点", "1号测试点"),  # urgency prefix
    ("删除航点1号测试点", "1号测试点"),  # type word dropped
    ("把1号测试点删掉", "1号测试点"),  # object first
    ("将1号测试点这个点删除吧", "1号测试点"),  # 这个点 + 吧
])
def test_delete(text, name):
    got = parse_ptp(text)  # one parse
    assert got is not None and got.kind == "delete" and got.name == name


@pytest.mark.parametrize("text", ["删除东门路线", "删除3号路径", "把2号围栏删掉"])
def test_delete_of_a_route_or_fence_is_not_ours(text):
    """F11 / F13 keep those. MUTATION: take them as a keypoint delete."""
    assert parse_ptp(text) is None  # left to the keyword layer


@pytest.mark.parametrize("text,old,new", [
    ("把1号测试点改名为2号测试点", "1号测试点", "2号测试点"),  # 把
    ("将1号测试点改名为2号测试点", "1号测试点", "2号测试点"),  # 将
    ("现在把东门岗亭重命名为西门岗亭", "东门岗亭", "西门岗亭"),  # prefix + 重命名为
    ("立刻将东门岗亭改名为西门岗亭吧", "东门岗亭", "西门岗亭"),  # prefix + 吧
    ("把1号测试点的名字改成3号点", "1号测试点", "3号点"),  # 名字改成
    ("东门路线更名为北门路线", "东门路线", "北门路线"),  # a route, no 把
])
def test_rename(text, old, new):
    """MUTATION: let the unstripped stage win and "现在把东门岗亭" becomes
    the old name."""
    got = parse_ptp(text)  # one parse
    assert got is not None and got.kind == "rename"  # a rename
    assert (got.name, got.new) == (old, new)  # both names


def test_a_change_without_name_is_not_a_rename():
    """"把照明灯改成最亮" is a lamp command. MUTATION: accept a bare 改成."""
    assert parse_ptp("把照明灯改成最亮") is None  # D class keeps it


def test_a_time_on_delete_is_reported_for_refusal():
    got = parse_ptp("下午3点删除1号测试点")  # timed delete
    assert got.kind == "delete" and got.when is not None  # the caller refuses


# -- 99 U121: a named patrol ------------------------------------------------
@pytest.mark.parametrize("text,name,loops", [
    ("沿1号路径巡逻", "1号路径", None),  # route first; the 路径 in the name is kept
    ("现在沿着东环线巡逻3圈", "东环线", 3),  # prefix + digit laps
    ("巡逻1号路径", "1号路径", None),  # verb first
    ("开始巡逻东门路线", "东门路线", None),  # 开始
    ("去1号路径巡逻两遍", "1号路径", 2),  # 去 X 巡逻 + CJK laps
    ("沿1号路径巡逻十二圈吧", "1号路径", 12),  # 十二 + particle
])
def test_named_patrol(text, name, loops):
    """MUTATION: a lazy name with an optional bare 路径 cuts "1号路径" to
    "1号"; no _patrol before _GOTO and "去X巡逻" is lost; a lap parser that
    ignores CJK numerals reads 两 / 十二 as None."""
    got = parse_ptp(text)  # one parse
    assert got is not None and got.kind == "patrol"  # a patrol
    assert got.name == name and got.loops == loops  # route as said, laps


def test_timed_named_patrol_keeps_its_time():
    """B03 shape through the same rule: the time prefix is split off and
    reported. MUTATION: no time stage for patrol and the route name holds
    "下午3点沿..."."""
    got = parse_ptp("下午3点沿1号路径巡逻")  # timed
    assert got is not None and got.kind == "patrol" and got.name == "1号路径"
    assert got.when is not None and got.when.at_local == "15:00"  # the time


@pytest.mark.parametrize("text", [
    "巡逻", "开始巡逻", "开始巡逻一圈", "按这条路径巡逻", "去那边巡逻", "巡逻什么",
])
def test_patrol_without_a_route_name_is_not_ours(text):
    """No route named -> None, so B02's own leg asks which route.
    MUTATION: take a lap count / 这条路径 / a question as the route name."""
    assert parse_ptp(text) is None  # left to the keyword layer
