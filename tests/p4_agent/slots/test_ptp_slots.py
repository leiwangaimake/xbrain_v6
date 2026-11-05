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


@pytest.mark.parametrize("text,kind,name", [  # 99 U133: route / fence deletes by name
    ("删除东门路线", "delete_route", "东门路线"), ("删除3号路径", "delete_route", "3号路径"),
    ("把2号围栏删掉", "delete_fence", "2号围栏"),
])
def test_delete_of_a_route_or_fence_is_parsed_by_name(text, kind, name):
    """F11 / F13 by name (still L2 at the orchestrator). MUTATION: take them
    as a keypoint delete -> kind delete -> red."""
    got = parse_ptp(text)  # one parse
    assert got is not None and got.kind == kind and got.name == name  # by name


@pytest.mark.parametrize("text", ["删除路径", "删除围栏"])
def test_a_bare_type_word_delete_is_not_ours(text):
    """No name -> F11 / F13's own keyword leg asks. MUTATION: drop the
    bare-word check -> a delete of "路径" -> red."""
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
    ("按东门路线巡逻", "东门路线", None),  # weak form with a route-shaped name
    ("沿着围墙巡逻", "围墙", None),  # strong form: any name, the catalogue decides
])
def test_named_patrol(text, name, loops):
    """MUTATION: a lazy name with an optional bare 路径 cuts "1号路径" to
    "1号"; no _patrol before _GOTO and "去X巡逻" is lost; a lap parser that
    ignores CJK numerals reads 两 / 十二 as None; requiring the route tail on
    the strong forms too drops 沿着围墙巡逻."""
    got = parse_ptp(text)  # one parse
    assert got is not None and got.kind == "patrol"  # a patrol
    assert got.name == name and got.loops == loops  # route as said, laps


@pytest.mark.parametrize("text,name,loops", [  # 99 U131 round trips
    ("沿1号路径来回巡逻三次", "1号路径", 3),  # strong form + 次
    ("沿着东环线来回巡两趟", "东环线", 2),  # 来回巡
    ("在1号路径上来回巡逻3次", "1号路径", 3),  # 在 X 上 (weak, route tail)
    ("1号路径来回巡逻一次", "1号路径", 1),  # no verb in front (weak, round trip only)
    ("沿1号路径来回走两次", "1号路径", 2),  # 来回走
    ("沿1号路径来回巡逻", "1号路径", None),  # count not said
])
def test_round_trip_patrol(text, name, loops):  # U131
    """MUTATION: drop the rt group -> round_trip False -> red; drop 次 from
    _LOOPS -> the count is not read -> red."""
    got = parse_ptp(text)  # one parse
    assert got is not None and got.kind == "patrol" and got.round_trip  # a round trip
    assert got.name == name and got.loops == loops  # route, count


def test_plain_laps_are_not_a_round_trip():  # U131
    """MUTATION: set round_trip on every patrol -> red."""
    got = parse_ptp("沿1号路径巡逻3圈")  # plain laps
    assert got.kind == "patrol" and got.loops == 3 and not got.round_trip  # not doubled


@pytest.mark.parametrize("text,loops,rt", [  # 99 U131 B04
    ("再巡一圈", 1, False), ("再巡逻两圈", 2, False), ("再巡", None, False),  # 再巡
    ("再来一圈", 1, False), ("再走两圈", 2, False),  # 来 / 走 with 圈
    ("再来回巡一次", 1, True), ("来回巡三次", 3, True), ("来回巡逻两次", 2, True),  # round trips
    ("又巡一圈吧", 1, False),  # 又 + particle
])
def test_repeat(text, loops, rt):  # B04
    """MUTATION: drop _repeat -> "再巡一圈" falls to the keyword layer with
    no count -> red; ignore rt -> red."""
    got = parse_ptp(text)  # one parse
    assert got is not None and got.kind == "repeat" and got.name == ""  # no route named
    assert got.loops == loops and got.round_trip is rt  # as said


@pytest.mark.parametrize("text", ["再来一次", "再走一趟", "来回巡逻", "来回走三次", "再巡一百圈"])
def test_not_a_repeat(text):  # U131 boundaries
    """再来一次 repeats anything; 再走一趟 goes somewhere; 来回巡逻 with no
    count asks which route; 来回走 alone is not patrol; 一百 is not a count.
    MUTATION: let 来 / 走 take any unit, or 来回走 stand alone -> red."""
    got = parse_ptp(text)  # one parse
    assert got is None or got.kind != "repeat"  # not B04 by rule


def test_timed_named_patrol_keeps_its_time():
    """B03 shape through the same rule: the time prefix is split off and
    reported. MUTATION: no time stage for patrol and the route name holds
    "下午3点沿..."."""
    got = parse_ptp("下午3点沿1号路径巡逻")  # timed
    assert got is not None and got.kind == "patrol" and got.name == "1号路径"
    assert got.when is not None and got.when.at_local == "15:00"  # the time


@pytest.mark.parametrize("text", [
    "巡逻", "开始巡逻", "开始巡逻一圈", "按这条路径巡逻", "去那边巡逻", "巡逻什么",
    # audit 1 (2026-11-05): weak forms whose object is not a route name
    "巡逻模式", "巡逻状态", "巡逻了多久", "巡逻一下", "巡逻一会儿", "巡逻吧",
    "巡逻任务", "巡逻速度快一点", "巡逻慢一点", "巡逻时注意安全", "按时巡逻",
    "按照计划巡逻", "走路巡逻", "巡检一下设备",
])
def test_patrol_without_a_route_name_is_not_ours(text):
    """No route named -> None, so B02's own leg (or the speed / status
    intent) handles it. MUTATION: take a lap count / 这条路径 / a question as
    the route name; drop the _ROUTE_TAIL check on the weak forms -> 巡逻模式
    and the rest of the audit-1 list are stolen."""
    assert parse_ptp(text) is None  # left to the keyword layer



# -- 99 U128: a named charging point (B09) ------------------------------------
@pytest.mark.parametrize("text,name", [
    ("去1号充电站", "1号充电站"),  # named station
    ("现在导航到东门充电桩", "东门充电桩"),  # prefix + named pile
])
def test_named_dock_goto(text, name):
    """MUTATION: no dock branch -> the goto rule rejects it (充电 is not a
    keypoint) and the name is lost -> red."""
    got = parse_ptp(text)  # one parse
    assert got is not None and got.kind == "dock" and got.name == name  # B09 with a name


@pytest.mark.parametrize("text", ["去充电站", "去充电桩", "回充电桩", "去充电"])
def test_bare_dock_is_left_to_the_keyword_layer(text):
    """No name -> None: B09's keyword leg sends a charge task with no dock and
    P3 picks the nearest. MUTATION: treat the bare word as a name -> red."""
    assert parse_ptp(text) is None  # not ours



# -- 99 U130: route membership questions (G48 / G49) ---------------------------
@pytest.mark.parametrize("text,kind,name", [
    ("1号路径上有哪些航点", "route_points", "1号路径"),  # the user's wording
    ("请问东门路线有哪些关键点", "route_points", "东门路线"),  # prefix, no 上
    ("1号测试点在哪条路径上", "point_routes", "1号测试点"),  # the user's wording
    ("返航终点站属于哪条路线", "point_routes", "返航终点站"),  # 属于
])
def test_membership_questions(text, kind, name):
    """MUTATION: no query branch -> None / a goto -> red; a lazy route name
    that stops before 路径 -> "1号" -> red."""
    got = parse_ptp(text)  # one parse
    assert got is not None and got.kind == kind and got.name == name  # the question and its object


@pytest.mark.parametrize("text", ["有哪些航点", "这里在哪条路径上", "一共有哪些路径"])
def test_membership_questions_need_a_name(text):
    """Unnamed forms stay with the keyword layer (G21 / G22) or are a different
    question. MUTATION: take 这里 or nothing as the object -> red."""
    assert parse_ptp(text) is None  # not ours
