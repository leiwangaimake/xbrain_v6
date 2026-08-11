"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: payload_slots.py
Brief: fastpath slot parsers for D17 level / D18 mode / D10 volume

Description:
16 S8.0.4 / CMD-40: a fastpath payload intent fills its slot from the ASR
text by deterministic code, never the LLM. This module extracts:

  * D17 set_light_bright  -> level, a CLOSED SET (18-A S1.1):
      max / high / mid / low / min / up / down
    (deliberately NOT 0-100 -- 18 PL-3: brightness 0-30 and volume 0-100
    must not share a validator, and a closed set makes a mis-classified
    volume utterance fail at the SLOT layer instead of silently dimming a
    light). The level -> 0..30 mapping is p2's job (near the device).
  * D18 set_strobe_mode   -> mode 1..16 (18-A S1.2), or None meaning
    'cycle to the next pattern' ('换一种'/'换个样式'). mode 0 is invalid
    (0 = off = D07, 18-A S1.2).
  * D10 set_volume        -> volume, either an absolute 0..100 or a
    relative delta (18 S6.4: level is 0-100 int; '大声点'/'小声点' are
    relative). p2 resolves a relative delta against the tracked volume.

Boundary: these produce the SLOT value only. Mapping a level enum to a
device brightness (0-30) or resolving a relative volume against the
current value lives in p2 payload_wiring, next to the hardware.
"""
from __future__ import annotations

import re
from typing import Optional


# --- D17 brightness level (closed set, 18-A S1.1) -----------------------
# Keyword substrings -> level enum. Ordered LONGEST-FIRST at match time so
# '亮一点点' (high) beats '亮一点' (up), '调到最暗' (min) beats '暗' (down).
_LIGHT_LEVEL: dict = {
    "开到最大": "max", "全功率": "max", "最亮": "max",
    "调到最暗": "min", "最暗": "min", "最低": "min",
    "亮度一半": "mid", "亮一半": "mid", "一半": "mid", "中等": "mid", "一般亮": "mid",
    "亮一点点": "high", "调亮些": "high", "亮些": "high",
    "别那么亮": "low", "太刺眼": "low", "暗一些": "low", "暗些": "low",
    "亮一点": "up", "再亮": "up", "调亮": "up", "亮点": "up",
    "暗一点": "down", "再暗": "down", "调暗": "down", "暗点": "down",
}


def parse_light_level(text: str) -> Optional[str]:
    """Return the D17 level enum (max/high/mid/low/min/up/down) or None.

    Longest-first so a more specific phrase wins over a shorter substring
    (18-A S1.1: '亮一点点' is high, '亮一点' is up)."""
    text = text or ""
    for kw in sorted(_LIGHT_LEVEL, key=len, reverse=True):
        if kw in text:
            return _LIGHT_LEVEL[kw]
    return None


# --- D18 strobe mode 1..16 (18-A S1.2) ----------------------------------
# Chinese numerals 1..16 for the 'used the Nth pattern' phrasing.
_CN_NUM = {
    "一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5, "六": 6, "七": 7,
    "八": 8, "九": 9, "十": 10, "十一": 11, "十二": 12, "十三": 13,
    "十四": 14, "十五": 15, "十六": 16,
}
_STROBE_MODE_RE = re.compile(r"(?:第|用|换成|模式|图案)\s*([0-9]{1,2})")

# The standard red-blue alternating pattern (mode 1). '切换到红蓝爆闪模式' etc.
# ask for this specific pattern (2026-08-11 ORIN: mode 3 turned out to be
# pure red, so the operator needs a way back to red-blue). D06 also defaults
# to pattern 1.
_REDBLUE_PATTERN = 1


def parse_strobe_mode(text: str) -> Optional[int]:
    """Return the D18 mode 1..16, or None meaning 'cycle to next'.

    18-A S1.2: an explicit ordinal ('用第三种' / '换成5') sets the mode;
    '换一种'/'换个样式' leaves it None (p2 cycles current+1). A '红蓝' phrase
    ('切换到红蓝爆闪模式') selects the red-blue pattern (mode 1). mode 0 is
    NEVER returned (0 = off = D07); an out-of-range number returns None so
    the caller cycles rather than dispatching an invalid pattern."""
    text = text or ""
    if "红蓝" in text:
        return _REDBLUE_PATTERN
    m = _STROBE_MODE_RE.search(text)
    if m:
        n = int(m.group(1))
        return n if 1 <= n <= 16 else None
    # Chinese numeral after 第/用/换成/模式/图案.
    for marker in ("第", "用第", "换成第", "模式", "图案"):
        idx = text.find(marker)
        if idx >= 0:
            tail = text[idx + len(marker):]
            for cn in sorted(_CN_NUM, key=len, reverse=True):
                if tail.startswith(cn):
                    return _CN_NUM[cn]
    return None


# --- D10 volume 0..100 (18 S6.4) ----------------------------------------
# Absolute anchors and relative deltas. The relative step (+/-20) is an
# initial value (18 gives no delta), tuned in the field like 18-A S1.1's
# brightness +/-7. Not a safety value (audio level).
# '音量调到最小' etc. -> a low NONZERO value, not 0. 2026-08-11 ORIN: the
# device accepts [14]00 with ok but does NOT actually change volume (0 is
# ignored/no-op); any nonzero value works (25/50/75/100 all effective). So
# '最小音量' = quietest audible (5), distinct from '静音' (explicit mute = 0,
# which may itself be device-limited -- see 18-A Q-18A-4).
_VOL_MIN_AUDIBLE = 5
_VOL_STEP = 25
_VOL_ABS = {
    "音量调到最大": 100, "音量最大": 100, "开到最大": 100, "最大声": 100,
    "声音最大": 100, "音量调到最高": 100, "声音开到最大": 100,
    "音量调到最小": _VOL_MIN_AUDIBLE, "音量最小": _VOL_MIN_AUDIBLE,
    "最小声": _VOL_MIN_AUDIBLE, "声音最小": _VOL_MIN_AUDIBLE,
    "音量调到最低": _VOL_MIN_AUDIBLE,
    "静音": 0, "关掉声音": 0,
    "音量一半": 50, "中等音量": 50, "音量中等": 50,
}
# Relative up/down. Word-order variants matter: ASR gives both '大声点' and
# '大点声' (2026-08-11 ORIN: '大点声'/'音量大一点' missed and went overheard).
_VOL_UP = ("大声", "大点声", "大一点", "大点", "响一点", "响点", "调大",
           "声音大", "音量大")
_VOL_DOWN = ("小声", "小点声", "小一点", "小点", "轻一点", "轻点", "调小",
             "声音小", "音量小")


def parse_volume(text: str) -> Optional[dict]:
    """Return {'abs': 0..100} or {'rel': +/-step} for D10, or None.

    18 S6.4: the volume slot is a 0..100 int. Absolute phrases map to a
    value; '大声点'/'小声点' are relative (p2 applies the delta to the
    tracked volume, since this unit has no [99] volume readback)."""
    text = text or ""
    for kw in sorted(_VOL_ABS, key=len, reverse=True):
        if kw in text:
            return {"abs": _VOL_ABS[kw]}
    for kw in _VOL_UP:
        if kw in text:
            return {"rel": _VOL_STEP}
    for kw in _VOL_DOWN:
        if kw in text:
            return {"rel": -_VOL_STEP}
    return None
