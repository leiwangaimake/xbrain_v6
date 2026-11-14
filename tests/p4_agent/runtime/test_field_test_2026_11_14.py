"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_field_test_2026_11_14.py
Brief: 99 U197 -- the test engineer's 2026-11-14 voice / text report, pinned through a real orchestrator
Description:
The report listed 162 sentences that were unrecognised, routed to the wrong
intent, or answered "好的 / 收到" while nothing happened. Every case here is a
sentence from that report (or its direct sibling), run through the production
orchestrator with the production configs and no LLM (a call to it fails the
case), so a keyword table edit or a rule that is reverted turns a case red:
  * "什么" only means "repeat" as the whole sentence; inside a question it no
    longer replays the last reply;
  * a mode-switch QUESTION is answered from state and sends nothing, even when
    a C01 keyword ("报警模式") is inside it;
  * PTZ commands with no direction / step, an absolute zoom or a percentage
    speed are refused with the reason, and a degree'd scan is E10;
  * G17 names only the sensor that was asked about; "什么型号" names the model;
  * "你的位置在哪" is the robot's own position (G01), not a waypoint;
  * a sample of the synonyms the report found missing reach their intent.
"""
from __future__ import annotations  # postponed annotations

import os  # this tree's configs

import pytest  # parametrize, marker
import yaml  # content files

from xbrain.p4_agent.registry.intents import load_intent_registry  # the registry loader
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # as wired
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # a fresh dialog
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets

pytestmark = pytest.mark.no_device  # no robot, no LLM

_CONFIGS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "configs")  # this tree
#: G17's live answer shape (status_answers.sensor_answer): one item per sensor.
_SENSORS = "深度相机正常,激光雷达状态未知,RTK 定位降级,航向故障,时钟正常,云台正常,麦克风正常"
_ITEMS = [{"geo_id": "w-1", "type": "waypoint", "name": "1号测试点", "state": "active", "rev": 1}]  # catalogue


def _no_llm(*_a, **_k):  # tier-2 fake
    raise AssertionError("tier-2 LLM called")  # every case here is rule / keyword


def _orch(mode="idle", deter_ok=False, tier2=_no_llm):  # production orchestrator
    """An orchestrator over this tree's configs with a small live state."""
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:  # registry
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:  # presets
        cc = ChitchatResponder(yaml.safe_load(fh))  # presets
    item = {"state": "ok" if deter_ok else "fail"}  # the siren / strobe health
    state = {"state/geo/manifest": {"items": _ITEMS},  # the catalogue
             "state/teach": {"session": {"state": "idle"}},  # not recording
             "state/mode": {"voice_mode": mode},  # P2's mode
             "health/summary": {"items": {"payload_siren": item, "payload_strobe": item}}}  # deter health

    def _query(entry):  # G answers: only G17 has a canned value here
        return _SENSORS if entry.id == "G17" else "Q:" + entry.id  # marks which query ran
    return build_orchestrator(reg, cc, tier2_fn=tier2, state_fn=lambda: state,
                              query_fn=_query)  # as p4 wires it


def _turn(text, orch=None, session=None):  # one cloud-text turn
    """(decision, frames other than speech) for one sentence."""
    d = (orch or _orch()).handle_turn(text, session or OrchestratorSession(), 1000,
                                      channel="cloud", modality="text")  # a typed command
    return d, [k for k, _ in decision_to_publishes(d) if k != "cmd/audio/speak"]  # minus speech


# -- "什么" -------------------------------------------------------------------

@pytest.mark.parametrize("text, want", [("现在是什么模式", "G05"), ("前方有什么目标", "G19"),
                                        ("今天日期是什么", "G24"), ("开机自检结果是什么", "G18"),
                                        ("当前生效围栏是什么", "G39")])
def test_shi_me_inside_a_question_is_not_repeat(text, want):  # V0242 / V0278 / V0280 / V0303 / V0343
    """MUTATION: put "什么" back into I03's keywords -> these replay -> red."""
    d, _ = _turn(text)  # one turn
    assert d.intent_id == want  # the question's own intent


def test_shi_me_with_no_other_keyword_is_not_repeat():  # V0633 "海卫,你喜欢吃什么"
    """MUTATION: put "什么" back into I03's keywords -> this replays the last
    reply -> red. (The questions above now win on longer keywords of their
    own, so only a sentence with no other keyword shows the substring rule.)"""
    d, frames = _turn("你喜欢吃什么", _orch(tier2=lambda *_a, **_k: None))  # LLM finds nothing
    assert d.intent_id != "I03" and frames == []  # not a replay, nothing sent


def test_bare_shi_me_still_repeats():  # the one meaning left
    """MUTATION: drop the _BARE_REPEAT rule -> "什么" is unknown -> red."""
    orch, s = _orch(), OrchestratorSession()  # one dialog
    first, _ = _turn("你好", orch, s)  # something to repeat
    d, _ = _turn("什么", orch, s)  # the bare word
    assert d.intent_id == "I03" and d.tts_text == first.tts_text  # replayed


# -- mode-switch questions -----------------------------------------------------

def test_mode_question_never_switches():  # V0108 family and defect #7
    """MUTATION: drop the C01-C05 -> C08 reroute -> "能切到报警模式吗" sends cmd/mode -> red."""
    d, frames = _turn("能切到报警模式吗")  # a question with a C01 keyword inside
    assert d.intent_id == "C08" and frames == []  # answered, nothing sent
    assert "不能切到报警驱离模式" in d.tts_text  # deter items failing: the reason


@pytest.mark.parametrize("text, said", [("现在能切换到巡逻吗", "没有发现阻止切到巡逻对话模式"),
                                        ("能不能切到充电", "充电不是一种工作模式"),
                                        ("能切到待机吗", "已经是空闲待命模式"),
                                        ("当前允许切换模式吗", "没有说要切到哪个模式")])
def test_mode_question_answers(text, said):  # V0108 / V0110 / V0113 / V0114
    """Each branch of mode_switch_answer, and never a frame."""
    d, frames = _turn(text)  # one turn
    assert d.intent_id == "C08" and said in d.tts_text and frames == []  # answered only


def test_alarm_question_with_device_ok_says_how():  # the other alarm branch
    d, _ = _turn("可以切到驱离模式吗", _orch(deter_ok=True))  # deter items ok
    assert "要切换请说:报警模式" in d.tts_text  # the switching sentence


# -- PTZ refusals ---------------------------------------------------------------

@pytest.mark.parametrize("text, said", [("变焦", "没有说放大还是缩小"),
                                        ("放大到十倍", "不能调到指定倍数"),
                                        ("镜头缩小到两倍", "不能调到指定倍数"),
                                        ("云台转速", "没有说云台转速调快还是调慢"),
                                        ("云台转速设成百分之六十", "不能按百分比设置"),
                                        ("云台微调", "没有说云台往哪边转")])
def test_ptz_incomplete_or_impossible_is_refused(text, said):  # V0223 / V0944-V0948 / V0780
    """MUTATION: drop _ptz_limit -> "好的" and a cmd/ptz frame -> red."""
    d, frames = _turn(text)  # one turn
    assert d.kind == "denied" and said in d.tts_text and frames == []  # refused, nothing sent


def test_degree_scan_is_e10():  # V0236
    """MUTATION: drop the E07 degree branch -> a full orbit runs -> red."""
    d, frames = _turn("云台向左环视三十度")  # a degree'd scan
    assert d.intent_id == "E10" and frames == []  # refused


def test_zoom_query_is_not_a_zoom():  # V0331-V0333
    """MUTATION: let refine_ptz_intent reclaim G intents -> E06 zooms -> red."""
    d, frames = _turn("现在镜头放大多少倍")  # a question
    assert d.intent_id == "G33" and frames == []  # answered


# -- G17 / J02 / G01 ------------------------------------------------------------

def test_one_sensor_named_one_sensor_answered():  # V0498-V0500
    """MUTATION: drop _only_named_sensors -> every sensor is read -> red."""
    d, _ = _turn("麦克风正常吗")  # one sensor
    assert d.tts_text == "麦克风正常"  # only that one


def test_general_sensor_question_reads_all():  # V0502
    d, _ = _turn("传感器都正常吗")  # no sensor named
    assert d.tts_text == _SENSORS  # all of them


def test_model_question_names_the_model():  # V0907
    """MUTATION: drop the 型号 branch -> a line without the model may come back -> red."""
    for _ in range(3):  # the replies rotate: every one must name it
        d, _ = _turn("什么型号")  # one turn
        assert "SNARC-HW-001" in d.tts_text  # the model


@pytest.mark.parametrize("text", ["你的位置在哪", "海卫,你在哪儿", "当前位置", "报告当前位置", "现在什么位置"])
def test_own_position_is_g01(text):  # V0238-V0243
    """MUTATION: drop _asks_own_position -> G53 answers "没找到你的位置" -> red."""
    d, _ = _turn(text)  # one turn
    assert d.intent_id == "G01"  # the robot's position


# -- synonyms the report found missing ------------------------------------------

@pytest.mark.parametrize("text, want", [
    ("在原地等着", "A04"), ("往前挪半米", "A05"), ("向后半米", "A06"), ("向左横移半米", "A07"),
    ("往右平移一米", "A08"), ("向左旋转九十度", "A09"), ("向右旋转九十度", "A10"), ("转个身", "A11"),
    ("请把机器人转速调慢一点", "A15"), ("不要环视了", "E08"), ("平台别转了", "E08"),
    ("剩多少电", "G02"), ("还能跑多远", "G03"), ("报告一下当前速度", "G04"), ("当前有没有任务", "G06"),
    ("现在朝哪边", "G08"), ("你已经工作多久了", "G09"), ("有没有新的报警", "G10"),
    ("今天报警多少次", "G11"), ("上一次报警的时间", "G13"), ("现在有没有故障", "G15"),
    ("系统健康吗", "G16"), ("前面有多少人", "G20"), ("查看已有巡逻路线", "G21"),
    ("现在记录了哪些地点", "G22"), ("当前充电桩列表", "G23"), ("几点", "G24"),
    ("可以播放哪些警告", "G31"), ("镜头朝哪个方向", "G32"), ("云台在跟踪谁", "G34"),
    ("可以调用哪些预设位置", "G36"), ("配置了哪些电子围栏", "G38"), ("执行一次快速自检", "H01"),
    ("请再说一次", "I03"), ("不用继续了", "I04"), ("没事", "I04"), ("你可以做哪些事情", "I05"),
    ("hi", "J01"), ("介绍你自己", "J02")])
def test_reported_synonyms_reach_their_intent(text, want):  # the report's unrecognised rows
    """MUTATION: remove any of the keywords added for U197 -> its row goes red."""
    d, _ = _turn(text)  # one turn
    assert d.intent_id == want  # the intended family


@pytest.mark.parametrize("text", ["往前一点", "往后一点", "往左边移一点", "右边挪一点"])
def test_vague_distance_says_what_is_missing(text):  # V0021 / V0044 / V0052 / V0065
    """A vague amount is never guessed into metres: the missing-distance reply.
    The fast leg finds no number and asks the LLM once; an LLM that finds
    nothing either (here: None) ends in the reply, never in motion."""
    d, frames = _turn(text, _orch(tier2=lambda *_a, **_k: None))  # the LLM leg finds nothing
    assert "没有说走多远" in (d.tts_text or "") and frames == []  # explained, nothing sent


def test_bu_yong_ji_xu_does_not_resume():  # V0539: it used to answer "好的,继续任务"
    """MUTATION: drop I04 "不用继续" -> B06 resumes a task -> red."""
    d, frames = _turn("不用继续了")  # end the dialog
    assert d.intent_id == "I04" and frames == []  # nothing resumed
