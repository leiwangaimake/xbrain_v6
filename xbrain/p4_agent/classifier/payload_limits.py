"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: payload_limits.py
Brief: Light requests the hardware cannot do -- red/blue dimming, strobe rate, searchlight strobe, pattern out of 1..16 -- caught before classification and answered with the reason

Description:
What problem this solves
------------------------
Field test 2026-10-29: "把警灯调亮" / "红蓝暗一点" (P239 / P240) turned the
SEARCHLIGHT on or down -- they classified as D17 lighting brightness, whose
target is the searchlight. "警示灯亮度调小", "让警灯闪快点", "闪慢点", "调一下
爆闪频率", "让探照灯闪", "探照灯爆闪", "用第十七种模式" (P236-P247) answered
"没太听清" or were overheard: no reason, no hint of what IS possible.

The hardware facts behind each refusal (00 PAY-02c / 14 GL-2..GL-4):
  * the red/blue warning lamp has NO dimming bit -- it is on (full) or off,
    and its flash pattern is chosen by number (GL-3);
  * the flash timing is animated by the lamp firmware; nothing in the command
    set changes its rate (GL-4);
  * the searchlight is steady illumination only; strobing it is banned
    (GL-2, PER-42: the camera images only during flashes);
  * the pattern number is 1..16 (0 = off is not a pattern).

The rule is conservative: a DEVICE word and an ACTION word must both be
present ("红蓝模式", "警灯亮着吗", "用第三种模式", "把灯调亮" do not match), so
a valid command is never swallowed. A hit returns a key; the orchestrator
speaks chitchat.yaml payload_limits[key] and dispatches nothing.

Boundary: text in, key out. No classification, no speaking, no publishing.
"""
from __future__ import annotations  # postponed annotations

import re  # pattern-number extraction
from typing import Optional

_REDBLUE = ("警灯", "红蓝", "警示灯", "警报灯")  # the red/blue warning lamp
_DIM = ("亮一点", "暗一点", "亮点", "暗点", "调亮", "调暗", "亮度", "亮些", "暗些",
        "调小", "调大", "更亮", "更暗")  # brightness actions
_RATE = ("闪快", "闪慢", "快点闪", "慢点闪", "频率", "闪得快", "闪得慢")  # flash-rate actions
_SEARCH = ("探照灯", "照明灯", "补光灯")  # the searchlight
_FLASH = ("闪",)  # any flashing of it ("闪", "爆闪", "闪起来")
_PATTERN = re.compile(r"第\s*([零〇一二两三四五六七八九十百\d]+)\s*种")  # "第十七种"
_CN = {"零": 0, "〇": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5,
       "六": 6, "七": 7, "八": 8, "九": 9}
PATTERN_MIN, PATTERN_MAX = 1, 16
_BRIGHT_NUM = re.compile(r"(?:调到|设为|设置为|调成)\s*[0-9零〇一二两三四五六七八九十百]+")  # "亮度调到23"  # 14 GL-3 / payload DeterCommand.mode range


def _has(text: str, words) -> bool:  # one query answer: has
    """Any of the words occurs in the text."""
    return any(w in text for w in words)  # result to the caller


def cn_number(s: str) -> Optional[int]:  # one query answer: cn number
    """'17' / '十七' / '零' / '二十' -> int (0..99); None when not a number."""
    if s.isdigit():  # branch on the live value
        return int(s)  # result to the caller
    if not s or any(c not in _CN and c != "十" for c in s):  # guard: nothing usable
        return None  # not ours: let another query group answer
    if "十" not in s:  # branch on the live value
        return _CN[s] if len(s) == 1 else None  # result to the caller
    tens, _, ones = s.partition("十")
    t = _CN.get(tens, None) if tens else 1  # local read of the state field
    o = _CN.get(ones, None) if ones else 0  # value used below
    return None if t is None or o is None else t * 10 + o  # not ours: let another query group answer


def limit_hit(text: str) -> Optional[str]:  # one query answer: limit hit
    """The payload_limits key this request runs into, or None."""
    if "云台" in text:  # PTZ phrases are never light requests
        return None  # not ours: let another query group answer
    if _has(text, _RATE) and (_has(text, _REDBLUE) or "爆闪" in text or len(text) <= 4):  # branch on the live value
        return "strobe_fixed_rate"  # rate is firmware-animated (GL-4)
    if _has(text, _REDBLUE) and _has(text, _DIM):  # branch on the live value
        return "redblue_no_dimming"  # no dimming bit (GL-3)
    if _has(text, _SEARCH) and _has(text, _FLASH):  # branch on the live value
        return "searchlight_no_strobe"  # steady only (GL-2)
    if "亮度" in text and "音量" not in text and _BRIGHT_NUM.search(text):
        return "brightness_numeric"  # D17 is an enum of steps (18-A S1.1)
    m = _PATTERN.search(text)  # local read of the state field
    if m:  # branch on the live value
        n = cn_number(m.group(1))  # value used below
        if n is not None and not PATTERN_MIN <= n <= PATTERN_MAX:  # branch on the live value
            return "pattern_out_of_range"  # 1..16 only
    return None  # not ours: let another query group answer


__all__ = ["limit_hit", "cn_number", "PATTERN_MIN", "PATTERN_MAX"]  # value used below
