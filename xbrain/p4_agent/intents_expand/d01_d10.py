"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: d01_d10.py
Brief: CHK-2-35 18-A section-2 few-shot expansion for D01/D02/D06/D07/D10 (with D10 mute/restore)

Description:
18-A §2 table lists five intents whose keyword-and-regex coverage
was expanded in the 2026-08 few-shot polish pass:

  D01  广播开始
  D02  广播结束
  D06  音量调整 (relative or absolute)
  D07  设置广播段落
  D10  静音 / 音量恢复

D10 hard branches:
  * '静音' / '别出声'       -> level == 0   (mute, explicit zero)
  * '音量恢复正常'           -> level == U76-4 一般音量固定档
                              (NO default-value key; the fixed
                               tier is a single constant from
                               resolved products, not a code default)

The expansion table below is a PROJECTION of 18-A §2 -- production
code MUST NOT keep a second hand-maintained copy; instead the
intents.yaml file is projected and diffed against this list at
startup (any drift reddens the projection meta-test).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List


# 18-A §2 expansion pass. Values are AT-LEAST -- production
# intents.yaml may add more synonyms, but every keyword here must
# appear in intents.yaml's keywords[] for that intent (bidirectional
# diff empty).
D_EXPANSION_TABLE: Dict[str, List[str]] = {
    "D01": ["开始广播", "广播开始", "开播", "打开广播", "开始喊话"],
    "D02": ["结束广播", "广播结束", "停播", "关闭广播", "停止喊话"],
    "D06": ["音量加大", "音量减小", "调大声", "调小声", "把声音调到"],
    "D07": ["切换广播段", "换一段广播", "下一段广播", "上一段广播"],
    "D10": ["静音", "别出声", "不要说话", "音量恢复正常"],
}


D10_MUTE_UTTERANCES = frozenset({
    "静音", "别出声", "不要说话",
})


D10_RESTORE_UTTERANCES = frozenset({
    "音量恢复正常",
})


D10_MUTE_LEVEL = 0        # explicit zero for 'mute'
# NB: no default_level constant here. 'restore' uses a runtime
# lookup into resolved products (fixed tier per U76-4). This
# module intentionally does NOT define D10_RESTORE_LEVEL as a
# module constant -- doing so would be a code-side default and
# fail CFG-CM-13 no-safety-default lint.


class D10ClassificationError(Exception):
    pass


def classify_d10(utterance: str) -> str:
    """Return 'mute' / 'restore' / 'unknown' based on the CJK
    utterance. 'unknown' means the caller falls through to the
    generic D06 path (relative volume adjust)."""
    if utterance in D10_MUTE_UTTERANCES:
        return "mute"
    if utterance in D10_RESTORE_UTTERANCES:
        return "restore"
    return "unknown"


def resolve_d10_level(kind: str, resolved_restore_level: int) -> int:
    """Given the classification + the resolved-product default
    tier value, return the target volume level.

    'kind' is one of {'mute', 'restore'}. Passing 'unknown' is a
    programming error at this layer -- caller must have chosen a
    different intent before reaching here."""
    if kind == "mute":
        return D10_MUTE_LEVEL
    if kind == "restore":
        if not isinstance(resolved_restore_level, int):
            raise D10ClassificationError(
                "restore level must come from resolved product "
                "as integer; got %r" % (resolved_restore_level,))
        return resolved_restore_level
    raise D10ClassificationError(
        f"resolve_d10_level: unexpected kind {kind!r}")


def bidirectional_diff_vs_yaml(intents_yaml_keywords: dict) -> dict:
    """Meta-check: intents.yaml keywords[] must cover the entire
    D_EXPANSION_TABLE. Returns {intent_id: {expansion_only,
    yaml_only}} for every intent that diverges. Empty dict = ok."""
    out: dict = {}
    for intent_id, expansion in D_EXPANSION_TABLE.items():
        yaml_kws = set(intents_yaml_keywords.get(intent_id, []))
        exp_set = set(expansion)
        expansion_only = exp_set - yaml_kws
        yaml_only = yaml_kws - exp_set
        if expansion_only or yaml_only:
            out[intent_id] = {
                "expansion_only": sorted(expansion_only),
                "yaml_only": sorted(yaml_only),
            }
    return out
