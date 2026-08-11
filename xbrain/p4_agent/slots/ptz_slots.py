"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ptz_slots.py
Brief: fastpath slot parsers for E01 move / E06 zoom / E09 speed

Description:
16 S8.0.4: fastpath PTZ intents fill their CLOSED-SET slots from the ASR
text by deterministic code (the LLM produces no data, CMD-40). This module
extracts:

  * E01 ptz_move  -> direction (left/right/up/down) + amount (small/normal/
    large). 18-B E01: a move is a TIMED PULSE, the amount picking the pulse
    length (three archives 350/1000/2800 ms); the archive->ms mapping is
    p2's job (near the device).
  * E06 ptz_zoom  -> zoom_dir (in/out) + amount. '拉近/放大'=in, '拉远/缩小'
    =out (18-B E06).
  * E09 set_ptz_speed -> level (slow/normal/fast/up/down), 18-B S2.

Direction/amount are closed enums, distinct from any numeric slot, so a
mis-classified command fails at the slot layer rather than moving the head
the wrong way. p2 maps a direction enum to the ONVIF pan/tilt velocity sign
and the amount/level to the pulse length / velocity magnitude.
"""
from __future__ import annotations

from typing import Optional


# --- E01 pan/tilt direction (18-B E01) ----------------------------------
# Keyword substrings -> direction enum. Longest-first at match time.
_MOVE_DIR = {
    "向左": "left", "往左": "left", "左边": "left", "左转": "left", "左": "left",
    "向右": "right", "往右": "right", "右边": "right", "右转": "right", "右": "right",
    "往上": "up", "向上": "up", "上面": "up", "上仰": "up", "抬高": "up", "上": "up",
    "往下": "down", "向下": "down", "下面": "down", "下俯": "down", "低头": "down",
    "下": "down",
}


def parse_ptz_direction(text: str) -> Optional[str]:
    """Return the E01 direction (left/right/up/down) or None. Longest-first
    so '向左' wins over the bare '左' and '左转' is not mistaken."""
    text = text or ""
    for kw in sorted(_MOVE_DIR, key=len, reverse=True):
        if kw in text:
            return _MOVE_DIR[kw]
    return None


# --- E06 zoom direction (18-B E06) --------------------------------------
_ZOOM_IN = ("拉近", "放大", "近一点", "近点", "推近", "变大", "大一点", "大点")
_ZOOM_OUT = ("拉远", "推远", "缩小", "远一点", "远点", "变小", "小一点", "小点")


def parse_zoom_direction(text: str) -> Optional[str]:
    """Return 'in' (拉近/放大) or 'out' (拉远/缩小) for E06, or None."""
    text = text or ""
    for kw in _ZOOM_IN:
        if kw in text:
            return "in"
    for kw in _ZOOM_OUT:
        if kw in text:
            return "out"
    return None


# --- amount (shared E01/E06): small/normal/large ------------------------
# Small if a 'a little' marker is present; large if a 'a lot' marker is;
# else normal. Drives the pulse archive in p2 (18-B E01 350/1000/2800 ms).
_AMOUNT_SMALL = ("一点点", "稍微", "一丢丢", "一点", "一些", "一下", "些")
_AMOUNT_LARGE = ("大幅", "使劲", "很多", "好多", "多转", "多一点", "转很多")


def parse_ptz_amount(text: str) -> str:
    """Return 'small' / 'normal' / 'large'. Default 'normal' when no
    amount marker is present."""
    text = text or ""
    for kw in _AMOUNT_LARGE:
        if kw in text:
            return "large"
    for kw in _AMOUNT_SMALL:
        if kw in text:
            return "small"
    return "normal"


# --- E09 ptz speed level (18-B S2) --------------------------------------
_SPEED_LEVEL = {
    "转速最慢": "slow", "最慢": "slow",
    "转速最快": "fast", "最快": "fast",
    "恢复正常转速": "normal", "正常转速": "normal", "转速正常": "normal",
    "转速加快": "up", "转速快一点": "up", "快一点": "up", "加快": "up",
    "转速减慢": "down", "转速慢一点": "down", "慢一点": "down", "减慢": "down",
}


def parse_ptz_speed_level(text: str) -> Optional[str]:
    """Return the E09 speed level (slow/normal/fast/up/down) or None.
    Longest-first so '转速最快' wins over '快一点'."""
    text = text or ""
    for kw in sorted(_SPEED_LEVEL, key=len, reverse=True):
        if kw in text:
            return _SPEED_LEVEL[kw]
    return None
