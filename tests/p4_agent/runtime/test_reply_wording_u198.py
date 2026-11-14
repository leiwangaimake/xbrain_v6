"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_reply_wording_u198.py
Brief: 99 U198 -- every voice / text reply says what was understood, what was done, or why nothing was
Description:
The 2026-11-14 field run and the defect list built from it found replies that
were untrue or said nothing useful:
  * "收到 / 好的" for commands the executor drops (D03 / D04 / D05 / D11 / D12 /
    D13 / C06 / A14);
  * "喊话" spoke the word "喊话" instead of a preset;
  * "已上报等待批准" although nothing is reported;
  * "没太听清" for typed text, which was received exactly;
  * a mode refusal, a speed step's value and a save result never spoken;
  * "好的,前进" without the distance that will be driven.
Each case runs a real orchestrator (production configs, no LLM: a call fails
the case) or the real AckSpeaker, so a reverted line turns its case red.
"""
from __future__ import annotations  # postponed annotations

import json  # frames are JSON bytes
import os  # this tree's configs

import pytest  # parametrize, marker
import yaml  # content files

from xbrain.p4_agent.registry.intents import load_intent_registry  # the registry loader
from xbrain.p4_agent.runtime.ack_speech import ACK_KEYS, AckSpeaker  # the executor answers
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # as wired
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # a fresh dialog
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets
from xbrain.p4_agent.query.sources_g43_g47 import g46_render  # G46 wording

pytestmark = pytest.mark.no_device  # no robot, no LLM

_CONFIGS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "configs")  # this tree
_PRESETS = ("您已进入管制区域,请立即离开", "此处为管制区域,禁止拍摄", "请出示证件并原地等待")  # speech_presets.yaml
_ITEMS = [{"geo_id": "w-1", "type": "waypoint", "name": "1号测试点", "state": "active", "rev": 1}]  # catalogue
_SUSPENDED = {"current": None, "queue": [], "suspended": [  # one passive suspended task
    {"task_id": "t-old", "type": "goto", "state": "suspended", "suspend_kind": "passive",
     "suspended_ts": 1.0}]}


def _no_llm(*_a, **_k):  # tier-2 fake
    raise AssertionError("tier-2 LLM called")  # every case here is rule / keyword


def _orch(task=None, tier2=_no_llm):  # production orchestrator
    """An orchestrator over this tree's configs with a small live state."""
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:  # registry
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:  # presets
        cc = ChitchatResponder(yaml.safe_load(fh))  # presets
    state = {"state/geo/manifest": {"items": _ITEMS},  # the catalogue
             "state/teach": {"session": {"state": "idle"}},  # not recording
             "state/task": task or {"current": None, "queue": [], "suspended": []}}  # tasks
    return build_orchestrator(reg, cc, tier2_fn=tier2, state_fn=lambda: state,
                              speech_presets=_PRESETS)  # as p4 wires it


def _turn(text, orch=None, modality="text", channel="cloud"):  # one turn
    """(decision, frames other than speech) for one sentence."""
    d = (orch or _orch()).handle_turn(text, OrchestratorSession(), 1000, addressed=True,
                                      channel=channel, modality=modality)  # one turn
    return d, {k: p for k, p in decision_to_publishes(d) if k != "cmd/audio/speak"}  # minus speech


# -- honest "unwired" -------------------------------------------------------------

@pytest.mark.parametrize("text, iid", [("灯光自动", "D03"), ("开警笛", "D04"), ("关警笛", "D05"),
                                       ("打开底盘灯", "D11"), ("别喊了", "D12"), ("全部关掉", "D13"),
                                       ("休息", "C06"), ("切换楼梯步态", "A14"), ("走平地", "A14")])
def test_dropped_commands_say_they_are_not_wired(text, iid):  # defect list 1 / 2 / 4
    """MUTATION: drop the id from _UNWIRED_ACTIONS -> "收到" and a frame -> red."""
    d, frames = _turn(text)  # one turn
    assert d.intent_id == iid and "还没有接通执行端" in d.tts_text and frames == {}  # honest, nothing sent


# -- presets ----------------------------------------------------------------------

def test_speak_preset_plays_the_preset_not_the_words():  # defect list 6
    """MUTATION: dispatch D08 with the turn's text -> P2 speaks "喊第二条" -> red."""
    d, frames = _turn("喊第二条")  # the second preset
    assert d.tts_text == "好的,喊第2条预设语句"  # the restatement
    sent = [p for k, p in decision_to_publishes(d) if k == "cmd/audio/speak"]  # both speak frames
    assert any(p.get("text") == _PRESETS[1] for p in sent)  # the preset is what P2 speaks


def test_preset_out_of_range_is_refused():
    d, frames = _turn("喊第五条")  # only three
    assert d.tts_text == "预设语句只有第1到第3条,没有喊话" and frames == {}  # refused


def test_preset_list_is_answered():  # G31
    d, _ = _turn("能喊哪些")  # the library
    assert d.tts_text.startswith("可以喊3条预设语句:第1条,您已进入管制区域")  # read out


# -- approval / unknown ---------------------------------------------------------------

@pytest.mark.parametrize("text", ["跟着他", "喊一句请立即离开", "跟着目标走"])
def test_approval_does_not_claim_a_report(text):  # B11 / D09 / C07
    """MUTATION: restore "已上报等待批准" -> red (nothing is reported)."""
    d, frames = _turn(text, _orch(tier2=lambda *_a, **_k: None))  # LLM unused or empty
    assert d.tts_text == "这个操作需要云端审批,语音和文本不能直接执行,没有执行" and frames == {}


def test_typed_unknown_is_not_did_not_catch():  # defect list 24
    """MUTATION: drop the text-modality branch -> "没太听清" for typed text -> red."""
    d, _ = _turn("今天天气怎么样", _orch(tier2=lambda *_a, **_k: None))  # typed, unknown
    assert d.tts_text.startswith("这句话我没有理解,没有执行")  # U199: true for any typed miss


def test_spoken_unknown_keeps_did_not_catch():  # voice is not text
    d, _ = _turn("今天天气怎么样", _orch(tier2=lambda *_a, **_k: None), modality="voice")  # cloud voice
    assert d.kind == "denied" and not d.tts_text  # decision_to_publishes speaks "没太听清"


@pytest.mark.parametrize("text, iid", [("是", "I01"), ("好的", "I01"), ("不", "I02"), ("算了", "I02")])
def test_bare_confirm_words_answer(text, iid):  # defect list 20
    d, _ = _turn(text)  # one word
    assert d.intent_id == iid and d.tts_text == "现在没有需要确认的操作"  # said, nothing pending


# -- refusals that say what is missing -------------------------------------------------

@pytest.mark.parametrize("text, said", [
    ("重命名3号测试点", "没有说新名字,没有改名.请说:把3号测试点改名为新的名字"),
    ("给6号路径改个名", "没有说新名字,没有改名.请说:把6号路径改名为新的名字"),
    ("三天后去1号测试点", "定时只能在一天以内, 或一周内某天的某个时刻"),
    ("去那里有多远", "没有说去哪里, 算不出距离和时间. 可以这样问: 去1号测试点多远"),  # U199
    ("把喇叭拆了", "没有说音量调到多少,没有执行.可以说:音量调到最大,或者:大声点"),
    ("当前位置记为充电桩", "没有说充电站的名字,没有记录.请说:把这里记为1号充电站"),
    ("保存围栏", "现在没有在录制"),
    ("取消任务", "现在没有正在执行或挂起的任务, 没有可取消的"),
    ("暂缓任务", "现在没有正在执行的任务, 没有可暂停的")])
def test_refusal_says_what_is_missing(text, said):  # defect list 10 / 12 / 14 / 15 / 27 / 31
    d, frames = _turn(text)  # one turn
    assert d.tts_text == said and "cmd/geo" not in frames and "cmd/task" not in frames  # nothing sent


def test_light_level_is_not_a_place():  # defect list 27 "开到最亮"
    d, _ = _turn("开到最亮")  # the searchlight
    assert d.intent_id == "D17" and d.tts_text == "好的,照明调到最亮"  # not "没找到最亮"


def test_fence_boundary_is_not_a_distance():  # defect list 11
    d, _ = _turn("离边界还有多远")  # the fence question
    assert d.intent_id == "G41"  # G41, honest "not wired" until it is


def test_rename_with_gai_jiao():  # defect list 13
    d, frames = _turn("把1号测试点改叫一号点")  # 改叫 is a rename verb
    assert d.intent_id == "F14" and "cmd/geo" in frames  # renamed


# -- restatements with values --------------------------------------------------------------

@pytest.mark.parametrize("text, said", [
    ("往前挪半米", "好的,前进0.5米"), ("左转90度", "好的,左转90度"), ("朝北", "好的,转向正北"),
    ("往北走两米", "好的,向正北走2米"), ("音量调到50", "好的,音量调到50"),
    ("音量调到200", "音量最大是100,已调到100"), ("大声点", "好的,音量调大一些"), ("别出声", "好的,静音"),
    ("灯调亮一点", "好的,照明调亮一些"), ("用第三种", "好的,警示灯换成第3种闪法"),
    ("看左边一点", "好的,云台向左转一点"), ("拉远一点", "好的,云台缩小一点"),
    ("左右扫一遍", "好的,云台左右扫描一遍"), ("转速慢一点", "好的,云台转速调慢一档"),
    ("报警模式", "好的,进入报警模式"), ("退出喊话", "好的,退出喊话模式"), ("别跟了", "好的,不再跟随")])  # U199 wording
def test_restatement_says_what_was_understood(text, said):  # defect list 36 / 19
    """MUTATION: drop _restate -> "好的,前进" / "收到" -> red."""
    d, _ = _turn(text)  # one turn
    assert d.tts_text == said  # the understood value


def test_cancel_with_nothing_running_names_the_suspended_task():  # defect list 21
    d, frames = _turn("取消任务", _orch(task=_SUSPENDED))  # only a suspended task
    assert d.tts_text == "好的,取消最近挂起的任务"  # says which one
    assert frames["cmd/task"]["task_id"] == "t-old"  # and that is what is sent


def test_recording_start_says_preparing():  # defect list 17
    d, _ = _turn("现在开始录制7号测试路径")  # a valid start
    assert d.tts_text == "好的,准备开始录制7号测试路径"  # the arming result follows


def test_bare_morning_hour_past_hints_the_afternoon():  # defect list 32
    """The refusal stays (never guess the afternoon) but says how to say it."""
    orch = _orch()  # wall clock: now
    orch._wall_now_fn = lambda: 4102444800.0 + 13 * 3600  # 13:00 UTC on some day
    orch._site_timezone = "UTC"  # read in UTC
    d, _ = _turn("3点钟去1号测试点", orch)  # 03:00 has passed
    assert "如果是下午或晚上" in d.tts_text  # the hint


# -- executor answers ------------------------------------------------------------------------

def _speaker_with(key, cmd):  # an AckSpeaker that sent cmd on key
    sp = AckSpeaker()  # fresh
    sp.note_turn([(key, json.dumps(cmd).encode()),
                  ("cmd/audio/speak", json.dumps({"source": "tts_cloud"}).encode())])  # a spoken turn
    return sp


def _said(sp, ack):  # the spoken text of one ack, or None
    out = sp.on_ack(json.dumps(ack).encode())  # (key, frame) or None
    return json.loads(out[1])["text"] if out else None


def test_mode_refusal_is_spoken():  # defect list 16
    """MUTATION: leave cmd/mode out of ACK_KEYS / note_turn -> silent -> red."""
    assert "cmd/mode/ack" in ACK_KEYS  # subscribed
    sp = _speaker_with("cmd/mode", {"cmd_id": "m-1", "action": "set_voice_mode", "voice_mode": "alarm"})
    text = _said(sp, {"cmd_id": "m-1", "result": "rejected", "code": "E_UNHEALTHY",
                      "detail": {"reason": "payload_deter"}})  # deter failed
    assert text == "进入报警模式没有成功: 三合一设备的警笛或警示灯不可用"  # why


@pytest.mark.parametrize("applied, said", [
    ({"move_speed_mps": 0.6, "changed": True, "at_limit": None}, "移动速度现在是0.6米每秒"),
    ({"move_speed_mps": 1.0, "changed": False, "at_limit": "max"}, "移动速度已经是最快的1米每秒了")])
def test_speed_step_reads_back_the_applied_value(applied, said):
    sp = _speaker_with("cmd/mode", {"cmd_id": "m-2", "action": "set_move_speed", "move_speed": "faster"})
    assert _said(sp, {"cmd_id": "m-2", "result": "accepted", "detail": {"applied": applied}}) == said


def test_save_result_is_spoken():  # defect list 35
    sp = _speaker_with("cmd/teach", {"cmd_id": "s-1", "action": "save", "save": {"name": "9号路径"}})
    assert _said(sp, {"cmd_id": "s-1", "result": "accepted", "detail": {"applied": {"name": "9号路径"}}}) == "已保存9号路径"


def test_fence_gap_is_spoken():  # defect list 18
    sp = _speaker_with("cmd/teach", {"cmd_id": "f-1", "action": "finish"})
    text = _said(sp, {"cmd_id": "f-1", "result": "accepted", "detail": {
        "saved": True, "kind": "fence", "applied": {"name": "东门报警区域围栏", "point_count": 6},
        "validation": {"issues": [{"code": "close_gap_large", "gap_m": 7.4}]}}})  # a 7.4 m gap
    assert text.endswith("注意: 首尾没有回到一起, 相距7.4米, 已直接连起来")  # said


def test_heading_source_none_is_plain():  # defect list 26
    assert g46_render("none") == "当前没有可用航向"  # not "航向来自 无"
