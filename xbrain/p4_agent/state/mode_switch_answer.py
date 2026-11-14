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
        health = _data((state or {}).get("health/summary")) or {}  # P2 health
        items = health.get("items") if isinstance(health.get("items"), Mapping) else {}  # per item
        bad = [i for i in _ALARM_ITEMS
               if not isinstance(items.get(i), Mapping) or items[i].get("state") != "ok"]  # not ok
        if bad:  # the switch would be refused
            states = {str((items.get(i) or {}).get("state") or "unknown") for i in bad}  # fail / unknown
            word = "故障" if states == {"fail"} else "异常或未知"  # spoken state
            return _ALARM_BLOCKED.format(items="和".join(_ITEM_WORDS[i] for i in bad), state=word)
    return _OK.format(cur=cur_name, name=name, say=say)  # no known blocker
