"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: motion_slots.py
Brief: fast-leg numeric slots for A05-A10 (distance_m / angle_deg) and the grounding check on LLM-filled ones

Description:
A05-A10 (move_forward / backward / left / right, turn_left / right) are
`fastpath_then_llm` (16 S6.6.4): the fast leg dispatches on a keyword hit
when it can fill the slot, the LLM leg (missions M1 / M2) fills it otherwise,
and only when both fail does the turn end in the clarify question (MI-2).
Until 2026-10-26 the fast leg had no number reader at all and the LLM leg was
never entered for these intents, so "前进三米" and "左转九十度" always ended in
"要走多远?" / "要转多少度?" -- the robot could not be told a distance.

This module gives the fast leg its reader, and the LLM leg its guard:

  * motion_slots(intent, text) -- distance_m from "<number><unit>": 米 / 公尺
    / m, 厘米 / 公分 / cm divided by 100 (16 S8.0.4: "厘米/公分 ÷100"), "半米"
    = 0.5 and "一米半" = 1.5 (16 S6.7 M1 rule 2); angle_deg from "<number>度"
    or "<number>°". Arabic and Chinese numerals (零一二两三...十百千, 点 for the
    decimal point). Positive only (S8.0.4 "只填正数"): zero gives no slot.
  * ground_llm_slots(intent, slots, text) -- the LLM may fill distance_m /
    angle_deg only with a number that is actually in the utterance (as said,
    or divided by 100 for a centimetre reading). The 3B model was seen to
    INVENT a distance for a vague command ("往前走走" -> 1 m); with the LLM leg
    now open for these intents that invention would become real motion. An
    ungrounded number is dropped, so the turn falls to the MI-2 clarify --
    16 EV-2: a missing slot means "ask", never "pick a value".

A12 face_heading (2026-10-26, same day): heading, the 11 S9.3.2A.4 eight-way
enum, from "朝 / 面向 / 面朝 / 朝向 / 转向 [正] <方位>" -- 东 南 西 北 and the
four compounds (东北 ...), compounds first. The direction must END the phrase
or be followed by 方 / 边 / 面 / 侧: "朝东门" is a place (the east gate), not
a heading. An LLM heading is grounded when its Chinese word is in the text.

What this does NOT do: no limits or conversions beyond the unit rule above
(11 MI-1: P4 does not transform; 20 m caps etc. are P2's G-3 gate), no
"steps" or "a bit" -- those carry no distance and must be asked for.

Traps:
  * "米" is also the last character of "厘米": centimetres are matched first,
    and a metre match never starts inside a centimetre token.
  * "两" is 2 only as a numeral; it is in the numeral class so "两米" reads 2.
  * A bare number with no unit ("前进三") is NOT a fast-leg distance -- the
    unit is the evidence it is a distance -- but it does ground an LLM value
    of 3, which is how such a sentence still executes.
"""
from __future__ import annotations

import re  # the number / unit scanner
from typing import Any, Dict, List, Mapping, Optional  # plain values in and out

#: Intent name -> its numeric slot (11 S9.3.2A.4 / 16 S8.0.4). Only the two
#: numeric slots; heading (A12) is an enum and turn_around has no slot.
NUMERIC_MOTION_SLOT = {
    "move_forward": "distance_m",  # A05
    "move_backward": "distance_m",  # A06
    "move_left": "distance_m",  # A07
    "move_right": "distance_m",  # A08
    "turn_left": "angle_deg",  # A09
    "turn_right": "angle_deg",  # A10
}

#: Chinese digit values; 两 is 2 as a numeral ("两米").
_CN_DIGIT = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5,
             "六": 6, "七": 7, "八": 8, "九": 9}  # single digits only
#: Chinese unit multipliers inside one number (no 万: nothing here is that big).
_CN_UNIT = {"十": 10, "百": 100, "千": 1000}  # positional multipliers

#: One number as spoken: arabic with optional decimals, or a run of Chinese
#: numeral characters (点 = decimal point).
_NUM = r"(\d+(?:\.\d+)?|[零一二两三四五六七八九十百千点]+)"
#: Centimetres first (see Traps): number + 厘米 / 公分 / cm.
_CM = re.compile(_NUM + r"\s*(?:厘米|公分|cm)", re.IGNORECASE)  # cm before m (Traps)
#: Metres: number + 米 / 公尺 / m, optional trailing 半 (+0.5). The lookbehind
#: keeps "厘米" from being read as metres; the lookahead keeps "m" from
#: matching the start of a longer latin word.
_M = re.compile(_NUM + r"\s*(?<!厘)(?:米|公尺|m(?![a-z]))(半)?", re.IGNORECASE)
#: Degrees: number + 度 / °.
_DEG = re.compile(_NUM + r"\s*(?:度|°)")  # degrees
#: Any number at all, for the grounding check.
_ANY = re.compile(_NUM)  # grounding: any number


def parse_number(token: str) -> Optional[float]:
    """An arabic or Chinese numeral -> float; None when it is not a number.

    Chinese: digits combine with 十 / 百 / 千 positionally ("一百八十" = 180,
    "十二" = 12, "十" = 10), and 点 starts the fractional digits ("一点五" =
    1.5). Anything malformed ("点点", "百百") gives None rather than a guess.
    """
    try:  # arabic first: the common case
        return float(token)  # arabic, with or without decimals
    except ValueError:
        pass  # fall through to Chinese
    if not token or any(ch not in _CN_DIGIT and ch not in _CN_UNIT
                        and ch != "点" for ch in token):
        return None  # not a Chinese numeral either
    whole, _sep, frac = token.partition("点")  # split at the decimal point
    if "点" in frac or (_sep and not frac):
        return None  # two points, or nothing after the point
    value = _cn_integer(whole) if whole else 0.0  # "点五" alone = 0.5
    if value is None:  # malformed integer part
        return None  # refuse, never guess
    if frac:  # fractional digits present
        if any(ch not in _CN_DIGIT for ch in frac):
            return None  # only plain digits after the point
        value += float("0." + "".join(str(_CN_DIGIT[c]) for c in frac))
    return value  # a float, positive or 0


def _cn_integer(text: str) -> Optional[float]:
    """The integer part of a Chinese numeral ("一百八十" -> 180.0)."""
    total, digit, last_unit = 0, None, 10 ** 9  # last_unit: units must descend
    for ch in text:  # left to right
        if ch in _CN_DIGIT:  # a digit
            if digit is not None:
                return None  # two digits in a row: "一二" is not a number
            digit = _CN_DIGIT[ch]  # held until a unit or the end
        else:  # a unit character
            unit = _CN_UNIT[ch]  # 10 / 100 / 1000
            if unit >= last_unit:
                return None  # units must descend: "十百" is malformed
            total += (1 if digit is None else digit) * unit  # "十" = 1x10
            digit, last_unit = None, unit  # start the next group
    return float(total + (digit or 0))  # trailing digit: "十二" -> +2


def _first(pattern: "re.Pattern[str]", text: str) -> Optional["re.Match[str]"]:
    """First match whose number parses; skips "点点米"-style garbage."""
    for m in pattern.finditer(text):  # in text order
        if parse_number(m.group(1)) is not None:
            return m  # the first usable one
    return None  # none in the text


def parse_distance_m(text: str) -> Optional[float]:
    """distance_m from the text, metres, positive; None when not stated."""
    m = _first(_CM, text)  # centimetres first
    if m is not None:  # a cm reading
        value = parse_number(m.group(1)) / 100.0  # 16 S8.0.4: cm / 100
    else:  # no cm: try metres
        m = _first(_M, text)  # number + metre unit
        if m is not None:  # a metre reading
            value = parse_number(m.group(1)) + (0.5 if m.group(2) else 0.0)
        elif "半米" in text:
            value = 0.5  # 16 S6.7 M1 rule 2: "半米" = 0.5
        else:  # neither form
            return None  # no distance in the sentence: ask (MI-2)
    return value if value > 0 else None  # positive only (S8.0.4)


def parse_angle_deg(text: str) -> Optional[float]:
    """angle_deg from the text, degrees, positive; None when not stated."""
    m = _first(_DEG, text)  # number + degree unit
    if m is None:
        return None  # no angle stated
    value = parse_number(m.group(1))  # never None: _first checked it
    return value if value > 0 else None  # positive only (S8.0.4)


def motion_slots(intent_name: str, text: str) -> Dict[str, Any]:
    """The fast-leg numeric slot for an A05-A10 intent, or {} when absent.

    {} is the signal for the LLM leg (16 S6.6.4), not a value. A12
    face_heading returns its heading enum the same way.
    """
    if intent_name == "face_heading":
        heading = parse_heading(text)  # A12: the eight-way enum
        return {} if heading is None else {"heading": heading}
    slot = NUMERIC_MOTION_SLOT.get(intent_name)  # distance or angle
    if slot is None:  # not a motion intent
        return {}  # not a numeric motion intent
    value = (parse_distance_m(text) if slot == "distance_m"  # read the slot's own unit
             else parse_angle_deg(text))
    return {} if value is None else {slot: value}  # the slot dict, or {}


def missing_motion_slot(intent_name: str, slots: Optional[Mapping[str, Any]]
                        ) -> bool:
    """True iff an A05-A10 / A12 intent lacks its slot (number or heading)."""
    if intent_name == "face_heading":
        return (slots or {}).get("heading") is None  # A12's enum slot
    slot = NUMERIC_MOTION_SLOT.get(intent_name)  # the intent's slot
    return slot is not None and (slots or {}).get(slot) is None


#: Characters that start a unit of distance or angle. A bare "一" counts as a
#: number only when one of these follows it (see numbers_in).
_UNIT_START = set("米厘公度m°M")  # units that may follow a bare one
#: Measure words after which a number is a count, not a distance or angle:
#: "走两步" is two steps, "转一下" is "turn a bit", "看一会" is "a while".
_COUNT_AFTER = set("下步次遍圈会些点个声")  # counts, not quantities


def numbers_in(text: str) -> List[float]:
    """The numbers in the text that can ground a distance or an angle.

    *** Not every numeral is a quantity. Chinese uses 一 inside ordinary
    phrases -- "挪一挪", "看一下", "一会儿" -- and an LLM that invents 1 m for
    "往那边挪一挪" would be "grounded" by that 一 (a test caught exactly
    this). So: a token directly followed by a count measure word is skipped;
    a bare "一" counts only when a unit follows it ("一米", "一度"); and 0.5
    is added only for "半米" / "米半", not for any 半 ("走一半" is not 0.5 m).
    """
    out = []  # in text order
    for m in _ANY.finditer(text):  # every numeral run
        value = parse_number(m.group(1))  # None when malformed
        nxt = text[m.end():m.end() + 1]  # the character after the number
        if value is None or nxt in _COUNT_AFTER:
            continue  # not a number, or a count ("两步", "一下")
        if m.group(1) == "一" and nxt not in _UNIT_START:
            continue  # "挪一挪", "一会": a particle, not a quantity
        out.append(value)  # a quantity
    if "半米" in text or "米半" in text:
        out.append(0.5)  # 16 S6.7 M1: 半米 = 0.5, and the half in 一米半
    return out  # may be empty


def ground_llm_slots(intent_name: str, slots: Mapping[str, Any],
                     text: str) -> Dict[str, Any]:
    """The LLM's slots with an ungrounded numeric motion slot removed.

    Grounded = the value equals a number in the utterance, that number
    divided by 100 (a centimetre reading), or a number plus 0.5 ("一米半").
    Everything else the LLM returned is passed through unchanged.
    """
    out = dict(slots)  # a copy: the caller's dict is not touched
    if intent_name == "face_heading" and "heading" in out:
        # A12: the enum value's Chinese word must have been said.
        word = HEADING_CN.get(out["heading"]) if isinstance(
            out["heading"], str) else None
        if word is None or word not in text:
            del out["heading"]  # invented or off-set: ask (EV-2)
        return out
    slot = NUMERIC_MOTION_SLOT.get(intent_name)  # numeric slot, if any
    if slot is None or slot not in out:  # nothing to check
        return out  # nothing numeric to check
    value = out[slot]  # what the LLM said
    said = numbers_in(text)  # what the operator said
    candidates = said + [n / 100.0 for n in said] + [n + 0.5 for n in said]
    ok = (isinstance(value, (int, float)) and not isinstance(value, bool)
          and value > 0 and any(abs(value - c) < 1e-6 for c in candidates))
    if not ok:  # not grounded
        del out[slot]  # invented or malformed: ask instead (EV-2)
    return out  # everything else unchanged


#: 11 S9.3.2A.4 heading enum -> its Chinese word. Compounds are listed first
#: and parse_heading tries them first, so "东北" is never read as "东".
HEADING_CN = {"northeast": "东北", "northwest": "西北", "southeast": "东南",
              "southwest": "西南", "east": "东", "west": "西", "south": "南",
              "north": "北"}
#: Words that introduce a heading in A12 phrasing (18 A12 keywords 朝北 /
#: 面向 / 转向正). Two-character cues first so "面朝" is not read as "朝".
_HEADING_CUES = ("面朝", "面向", "朝向", "转向", "朝")
#: What may follow the direction word: end of text, or 方 / 边 / 面 / 侧
#: ("北方", "北边", "北面", "北侧"). Anything else ("东门") is a place name.
_HEADING_TAIL = ("", "方", "边", "面", "侧")


def parse_heading(text: str) -> Optional[str]:
    """A12 heading from "<cue>[正]<direction>", or None when not stated.

    "转向正北" -> north, "面向东北方向" -> northeast, "朝东门" -> None.
    """
    for cue in _HEADING_CUES:
        start = text.find(cue)
        if start < 0:
            continue  # this cue is not in the sentence
        rest = text[start + len(cue):]  # what follows the cue
        if rest.startswith("正"):
            rest = rest[1:]  # "正北" = due north
        for name, word in HEADING_CN.items():  # compounds first
            tail = rest[len(word):len(word) + 1]  # the char after the word
            if rest.startswith(word) and tail in _HEADING_TAIL:
                return name  # a heading, not a place name
        return None  # the cue is there but no valid direction follows
    return None  # no cue: not a heading phrase
