"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: mode_switch_answer.py
Brief: C08 "能切到X模式吗" -- answer whether the voice mode can be switched, from live state, without switching
Description:
What problem this solves
------------------------
Field test 2026-11-14 (V0108-V0114): "现在能切换到巡逻吗", "可以切到驱离模式吗",
"能不能切到充电", "能切到待机吗" all answered "收到". C08 went to cmd/task, where
P3 logs "no contract action" and drops it. The operator asked a question and
got neither an answer nor an action. "当前允许切换模式吗" was not recognised.

What P4 can and cannot know
---------------------------
P2's mode machine (p2_core/mode/state_machine.py) refuses a switch for four
reasons: already in the target state, the minimum dwell not met, a domain held
by a non-preemptible source (preflight_blocked), and a failed entry side effect
-- for the alarm mode that is the payload device's deter step. P4 sees the
current mode (state/mode) and the payload device health (health/summary), so it
answers the first and the last for real. Dwell and domain holders live only in
P2; the answer says so instead of promising a switch it cannot vouch for.

Rules
-----
  * the target is read from the words; none -> say which question to ask;
  * "充电" is not a mode -> say how to go and charge;
  * target == current -> "已经是";
  * alarm with the siren or strobe item not ok -> "不能", with the reason;
  * otherwise -> no blocking condition known, here is the sentence that
    switches, and the final word is the switch itself.
Nothing is ever sent: a question must not change the mode (C08 is a query).
"""
from __future__ import annotations  # postponed annotations

from typing import Any, Mapping, Optional, Tuple  # type hints only

#: Spoken target -> (P2 voice_mode family, its spoken name, the sentence that
#: switches to it). Checked in order; the first word found decides. The
#: dialog family covers C05 "巡逻模式 / 开始工作", which P2 maps to dialog_a.
_TARGETS: Tuple[Tuple[Tuple[str, ...], str, str, str], ...] = (
    (("喊话", "广播"), "broadcast", "远程喊话", "开始喊话"),  # C03
    (("报警", "警戒", "驱离"), "alarm", "报警驱离", "报警模式"),  # C01
    (("巡逻", "工作", "对话"), "dialog", "巡逻对话", "进入巡逻模式"),  # C05
    (("待机", "待命", "空闲", "休息"), "idle", "空闲待命", ""),  # back to idle
)
#: Current voice_mode -> spoken name (the G05 wording, p4_agent/state/status_answers).
_MODE_WORDS = {"idle": "空闲待命", "dialog_a": "现场对话", "dialog_c": "云端对话",
               "dialog_e": "企业微信对话", "broadcast": "远程喊话", "alarm": "报警驱离"}  # id -> label
#: The alarm mode's entry runs the payload deter (siren + red/blue strobe);
#: these health items failing make that step fail (E_UNHEALTHY payload_deter).
_ALARM_ITEMS = ("payload_siren", "payload_strobe")  # health/summary item ids
_ITEM_WORDS = {"payload_siren": "警笛", "payload_strobe": "警示灯"}  # spoken item names

_NO_TARGET = "没有说要切到哪个模式,没法判断.可以这样问:能切到喊话模式吗"  # no target word
_CHARGE = "充电不是一种工作模式,要去充电请说:回充电桩"  # "能不能切到充电"
_UNREADABLE = "现在读不到当前模式,没法判断能不能切换"  # state/mode never received
_ALREADY = "现在已经是{name}模式了"  # target == current
_ALARM_BLOCKED = "现在不能切到报警驱离模式:{items}状态{state},报警需要警笛和警示灯"  # deter would fail
_OK = "当前是{cur}模式,没有发现阻止切到{name}模式的情况,要切换请说:{say}.能否切换以切换时的检查为准"  # P2 decides
_IDLE_OK = "当前是{cur}模式,要回到空闲待命请说:{say}"  # leaving broadcast / alarm
_IDLE_NO_WAY = "当前是{cur}模式,语音没有直接切到空闲待命的指令"  # dialog -> idle has no voice sentence


#: 99 U199: C01-C05 -> (family, spoken name) for mode_precheck.
_ENTER = {"enter_alarm": ("alarm", "报警"), "enter_broadcast": ("broadcast", "喊话"),
          "enter_patrol_mode": ("dialog", "巡逻")}  # set_voice_mode targets
_EXIT = {"exit_alarm": ("alarm", "报警"), "exit_broadcast": ("broadcast", "喊话")}  # exits
_PRE_ALREADY = "现在已经是{name}模式了"  # entering the current mode
_PRE_NOT_IN = "现在不在{name}模式, 不需要退出"  # leaving a mode not entered
_PRE_ALARM_BLOCKED = "现在不能进入报警模式: {items}状态{state}, 报警需要警笛和警示灯"  # deter


def _data(v: Any) -> Optional[Mapping[str, Any]]:
    """A state value's body, the S3.0 envelope unwrapped; None when absent."""
    if not isinstance(v, Mapping):  # never received
        return None  # absent
    inner = v.get("data")  # enveloped producers
    return inner if isinstance(inner, Mapping) else v  # bare producers


def _target(text: str) -> Optional[Tuple[str, str, str]]:
    """(family, spoken name, switching sentence) of the mode the words name."""
    for words, family, name, say in _TARGETS:  # first match decides
        if any(w in text for w in words):  # a target word
            return family, name, say  # found
    return None  # no target word


def _alarm_blockers(state: Mapping[str, Any]) -> Tuple[list, str]:
    """The deter items not ok, and their spoken state (C08 and the precheck)."""
    health = _data((state or {}).get("health/summary")) or {}  # P2 health
    items = health.get("items") if isinstance(health.get("items"), Mapping) else {}  # per item
    bad = [i for i in _ALARM_ITEMS
           if not isinstance(items.get(i), Mapping) or items[i].get("state") != "ok"]  # not ok
    states = {str((items.get(i) or {}).get("state") or "unknown") for i in bad}  # fail / unknown
    return bad, ("故障" if states == {"fail"} else "异常或未知")  # spoken state


def mode_precheck(intent_name: str, state: Mapping[str, Any]) -> Optional[str]:
    """99 U199: the answer to a C01-C05 command that live state already
    settles, or None to send it. Only the two refusals P4 can see for itself
    (same mode / not in the mode, and the alarm's deter items) are answered;
    an unreadable state/mode sends the command and lets P2 decide."""
    mode = _data((state or {}).get("state/mode"))  # P2's live mode
    cur = str(mode.get("voice_mode")) if mode is not None else ""  # e.g. dialog_a
    if not cur or cur == "None":  # nothing to compare against
        return None  # P2 decides
    if intent_name in _EXIT:  # leaving a mode
        family, name = _EXIT[intent_name]  # which one
        return None if cur == family else _PRE_NOT_IN.format(name=name)  # in it -> send
    if intent_name not in _ENTER:  # not a mode command
        return None  # nothing to say
    family, name = _ENTER[intent_name]  # the target
    if cur == family or (family == "dialog" and cur.startswith("dialog")):  # same mode
        return _PRE_ALREADY.format(name=name)  # nothing to switch
    if family == "alarm" and _data((state or {}).get("health/summary")) is not None:  # heard
        bad, word = _alarm_blockers(state)  # siren / strobe
        if bad:  # P2 would refuse it after "好的"
            return _PRE_ALARM_BLOCKED.format(
                items="和".join(_ITEM_WORDS[i] for i in bad), state=word)  # say why
    return None  # no known blocker: send


#: 99 U199: D-class id -> (health item, spoken device, what was not done).
_PAYLOAD_ITEM = {"D01": ("payload_light", "照明灯", "开灯"),
                 "D02": ("payload_light", "照明灯", "关灯"),
                 "D17": ("payload_light", "照明灯", "调亮度"),
                 "D06": ("payload_strobe", "警示灯", "开警示灯"),
                 "D07": ("payload_strobe", "警示灯", "关警示灯"),
                 "D18": ("payload_strobe", "警示灯", "换闪法"),
                 "D08": ("payload_speaker", "喇叭", "喊话"),
                 "D10": ("payload_speaker", "喇叭", "调音量")}  # the device each needs
_PAYLOAD_DOWN = "{device}现在故障, 这次没有{what}"  # the device reports fail


def payload_precheck(intent_id: str, state: Mapping[str, Any]) -> Optional[str]:
    """99 U199: the refusal of a D-class command whose device health/summary
    reports "fail", or None to send it. The live run answered "好的,音量调到50"
    with the three-in-one device unplugged, while "能切到报警模式吗" in the
    same minute said the siren and strobe were down. Only an explicit fail
    refuses: unknown or a missing item sends, and P2 decides."""
    if intent_id not in _PAYLOAD_ITEM:  # not a device command
        return None  # nothing to say
    item, device, what = _PAYLOAD_ITEM[intent_id]  # what it needs
    health = _data((state or {}).get("health/summary")) or {}  # P2 health
    items = health.get("items") if isinstance(health.get("items"), Mapping) else {}  # per item
    row = items.get(item)  # this device
    if isinstance(row, Mapping) and row.get("state") == "fail":  # reported down
        return _PAYLOAD_DOWN.format(device=device, what=what)  # say so
    return None  # ok / unknown: send


def mode_switch_answer(text: str, state: Mapping[str, Any]) -> str:
    """The C08 answer for this sentence and the live state snapshot."""
    if "充电" in text:  # not a mode (V0110)
        return _CHARGE  # say how to charge
    target = _target(text or "")  # what was asked about
    if target is None:  # "当前允许切换模式吗"
        return _NO_TARGET  # which question to ask
    family, name, say = target  # unpacked
    mode = _data((state or {}).get("state/mode"))  # P2's live mode
    cur = str(mode.get("voice_mode")) if mode is not None else ""  # e.g. dialog_a
    if not cur or cur == "None":  # nothing to compare against
        return _UNREADABLE  # honest
    cur_name = _MODE_WORDS.get(cur, cur)  # spoken current mode
    if cur == family or (family == "dialog" and cur.startswith("dialog")):  # same mode
        return _ALREADY.format(name=cur_name)  # nothing to switch
    if family == "idle":  # leaving a mode
        leave = {"broadcast": "退出喊话", "alarm": "退出报警"}.get(cur)  # the exit sentence
        return (_IDLE_OK.format(cur=cur_name, say=leave) if leave
                else _IDLE_NO_WAY.format(cur=cur_name))  # dialog has no voice exit
    if family == "alarm":  # the deter step needs the device
        bad, word = _alarm_blockers(state)  # 99 U199: shared with mode_precheck
        if bad:  # the switch would be refused
            return _ALARM_BLOCKED.format(items="和".join(_ITEM_WORDS[i] for i in bad), state=word)
    return _OK.format(cur=cur_name, name=name, say=say)  # no known blocker
