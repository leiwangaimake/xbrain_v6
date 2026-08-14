"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: sources_g43_g47.py
Brief: 18-C G43-G47 RTK/heading query bindings + deterministic renders (VD-3)

Description:
18-C adds five G-class queries for RTK fix + heading status; each maps a
structured state/pose or state/clock field into a canned reply. VD-3: the reply
is CODE, never LLM -- these render functions are the template fill. Load-bearing
rules:
  * G43/G46 read CLOSED SETS (fix_type / heading_source). An out-of-set value
    RAISES (11 S13.6), it is never "explained as unknown" -- a 6th fix_type is an
    upstream bug, not a phrase to invent.
  * G45 obeys H-1: heading_valid alone decides valid/invalid; level only adds the
    source wording. A valid=False with level=1 still answers "no heading".
  * G44 fix_type/sat data comes from rt/gnss/fix, which rtk_driver does not
    publish yet: num_satellites=None answers the no_data template, never a made-up
    count.
  * G47 fail-safe: sync False -> "not synced" wording; never report "synced" on a
    missing/false ClockStatus (CLK-A3).

Punctuation in the reply strings is ASCII (CLAUDE.md 2.2, 2026-08-11): the Chinese
text is spoken by TTS, the commas are ASCII.
"""

from __future__ import annotations

from typing import Dict, Optional

from xbrain.p4_agent.query.sources_g01_g24 import QueryBinding, QueryBindingError


C_QUERY_IDS = ("G43", "G44", "G45", "G46", "G47")


# fix_type (closed set, 11 S4.5) -> (wording, accuracy). no_fix has no accuracy.
_FIX_DESC: Dict[str, str] = {
    "rtk_fixed": "RTK 固定解",
    "rtk_float": "RTK 浮动解",
    "dgps": "差分定位",
    "single": "单点定位",
    "no_fix": "无定位",
}
_FIX_ACC: Dict[str, str] = {
    "rtk_fixed": "厘米级",
    "rtk_float": "分米级",
    "dgps": "米级",
    "single": "较低",
}
# heading_source (closed set, 11 S3.3) -> wording.
_HEADING_SRC_DESC: Dict[str, str] = {
    "dual_antenna": "双天线 RTK",
    "cog": "行进航迹 COG",
    "none": "无",
}
# heading_level -> source wording (only used when heading_valid is True).
_HEADING_LEVEL_DESC: Dict[int, str] = {
    1: "双天线固定航向",
    2: "航迹推算航向",
}


BINDINGS_C: Dict[str, QueryBinding] = {
    "G43": QueryBinding(
        intent_id="G43", source="state/pose",
        templates={"ok": "当前 {fix_desc}, 定位精度 {acc_desc}",
                    "no_fix": "当前没有定位",
                    "no_data": "定位状态暂不可用"}),
    "G44": QueryBinding(
        intent_id="G44", source="state/pose",
        templates={"ok": "当前锁定 {n} 颗卫星",
                    "no_data": "暂不支持查询卫星数量"}),
    "G45": QueryBinding(
        intent_id="G45", source="state/pose",
        templates={"ok": "航向有效, {level_desc}",
                    "invalid": "当前没有可用航向",
                    "no_data": "航向状态暂不可用"}),
    "G46": QueryBinding(
        intent_id="G46", source="state/pose",
        templates={"ok": "航向来自 {src_desc}",
                    "no_data": "航向状态暂不可用"}),
    "G47": QueryBinding(
        intent_id="G47", source="state/clock",
        templates={"ok": "授时已同步, 来源 {source}",
                    "unsynced": "授时未同步, 时间可能不准",
                    "no_data": "授时状态暂不可用"}),
}


def assert_bindings_cover_c():
    """Meta-test: all five 18-C IDs have a binding with ok + no_data."""
    missing = [q for q in C_QUERY_IDS if q not in BINDINGS_C]
    if missing:
        raise QueryBindingError(f"missing 18-C query bindings: {missing}")
    for q, b in BINDINGS_C.items():
        for k in ("ok", "no_data"):
            if k not in b.templates:
                raise QueryBindingError(f"binding {q!r} missing template {k!r}")


# --- deterministic renders (VD-3 hard branches) ---------------------

def g43_render(fix_type: Optional[str]) -> str:
    """RTK fix status. None (rt/gnss/fix not published yet) -> no_data; no_fix ->
    its own line; a value outside the closed set RAISES (11 S13.6)."""
    if fix_type is None:
        return BINDINGS_C["G43"].templates["no_data"]
    if fix_type not in _FIX_DESC:
        raise QueryBindingError(f"G43 fix_type out of closed set: {fix_type!r}")
    if fix_type == "no_fix":
        return BINDINGS_C["G43"].templates["no_fix"]
    return BINDINGS_C["G43"].templates["ok"].format(
        fix_desc=_FIX_DESC[fix_type], acc_desc=_FIX_ACC[fix_type])


def g44_render(num_satellites: Optional[int]) -> str:
    """Satellite count. Field not on the general plane yet -> no_data, never a
    fabricated number (VD-3)."""
    if num_satellites is None:
        return BINDINGS_C["G44"].templates["no_data"]
    return BINDINGS_C["G44"].templates["ok"].format(n=int(num_satellites))


def g45_render(heading_valid: Optional[bool], heading_level: Optional[int]) -> str:
    """Heading status. H-1: heading_valid alone decides valid/invalid; level only
    picks the source wording. valid=False (even with level=1) -> no heading."""
    if heading_valid is None:
        return BINDINGS_C["G45"].templates["no_data"]
    if not heading_valid:
        return BINDINGS_C["G45"].templates["invalid"]
    level_desc = _HEADING_LEVEL_DESC.get(heading_level, "航向有效")
    return BINDINGS_C["G45"].templates["ok"].format(level_desc=level_desc)


def g46_render(heading_source: Optional[str]) -> str:
    """Heading source. Closed set; out-of-set RAISES (11 S13.6)."""
    if heading_source is None:
        return BINDINGS_C["G46"].templates["no_data"]
    if heading_source not in _HEADING_SRC_DESC:
        raise QueryBindingError(
            f"G46 heading_source out of closed set: {heading_source!r}")
    return BINDINGS_C["G46"].templates["ok"].format(
        src_desc=_HEADING_SRC_DESC[heading_source])


def g47_render(sync: Optional[bool], source: Optional[str]) -> str:
    """Clock sync. Fail-safe: anything but an explicit True answers 'not synced'
    (CLK-A3); never report 'synced' on a missing/false ClockStatus."""
    if sync is None:
        return BINDINGS_C["G47"].templates["no_data"]
    if not sync:
        return BINDINGS_C["G47"].templates["unsynced"]
    return BINDINGS_C["G47"].templates["ok"].format(source=source or "none")
